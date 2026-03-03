"""
WholeBodyQP - Whole-body Quadratic Program for high-rate tracking.

Wraps HierarchicalQP with the whole-body dynamics of the crawling
space robot. Tracks centroidal references from Stage 1 (CentroidalNMPC)
while enforcing full multibody dynamics, actuator limits, contact
constraints, and momentum safety bounds.

Architecture:
    Stage 2 of the two-stage controller (see Chelikh et al., IEEE Access 2024).
    Runs at 125 Hz with instantaneous (single time-step) optimization.

Decision variables:
    z = [q̈_t (6), q̈ (nq), λ (6·nc_max), τ_q (nq)]
    - q̈_t:  Torso (floating-base) acceleration
    - q̈:    Joint accelerations
    - λ:     Contact wrenches [f1, τ1, f2, τ2]
    - τ_q:   Joint torques (direct output to actuators)

Equality constraints:
    1. Full robot dynamics (Eq. VI-F.7):
       H_robot q̈_robot + C_robot = B_u τ_q + J_robot^T λ
    2. Contact acceleration constraint (bilateral, q̈_s ≈ 0):
       J_contact q̈_robot = -J̇_contact q̇_robot

Inequality constraints:
    1. Momentum safety (the "box"):
       h_min ≤ hw - dt·M_λ·λ ≤ h_max
    2. Joint torque limits:    τ_min ≤ τ_q ≤ τ_max
    3. Joint acceleration limits (from barrier functions)

Tasks (weighted hierarchy, Eq. VI-F.6):
    Priority 1 (α=10³): CoM tracking
    Priority 2 (α=10²): Posture regulation
    Priority 3 (α=10¹): Contact wrench tracking (from NMPC)
    Priority 4 (α=10⁰): Joint torque minimization
    Priority 5 (α=10⁻²): Acceleration regularization

Reference:
    Eq. (VI-F.1)-(VI-F.11) of the paper.
"""

import numpy as np
import pinocchio as pin
from dataclasses import dataclass, field
from typing import Optional, Tuple, Dict, Any

from .hierarchical_qp import HierarchicalQP, QPSolveInfo
from .contact_phase import ContactConfig, ContactPhase, skew, compute_momentum_map


@dataclass
class WholeBodyQPConfig:
    """Configuration for WholeBodyQP.

    Parameters
    ----------
    nq : int
        Number of active robot joints (excluding 6-DoF torso).
    nc_max : int
        Maximum number of contacts (typically 2 for dual-arm).
    """
    nq: int = 14                  # Joint DoFs (2 × 7-DoF arms)
    nc_max: int = 2               # Max contacts

    # QP method
    method: str = 'weighted'      # 'weighted' or 'strict'
    solver: str = 'qpoases'

    # Task weights (Eq. VI-F.6)
    alpha_com: float = 1e3        # CoM tracking (highest priority)
    alpha_torso: float = 0.0      # Torso 6D tracking (replaces CoM when > 0)
    alpha_ee: float = 5e2         # End-effector tracking (swing arm)
    alpha_posture: float = 1e2    # Posture regulation
    alpha_wrench: float = 1e1     # Wrench tracking (from NMPC)
    alpha_torque: float = 1e0     # Joint torque minimization
    alpha_reg: float = 1e-2       # Acceleration regularization (lowest)

    # PD gains for CoM tracking (Eq. VI-F.4)
    Kp_com: np.ndarray = field(default_factory=lambda: 100.0 * np.ones(3))
    Kd_com: np.ndarray = field(default_factory=lambda: 20.0 * np.ones(3))

    # PD gains for Torso 6D tracking [linear(3), angular(3)]
    Kp_torso: np.ndarray = field(default_factory=lambda: np.array([8., 8., 8., 5., 5., 5.]))
    Kd_torso: np.ndarray = field(default_factory=lambda: np.array([6., 6., 6., 4., 4., 4.]))

    # PD gains for end-effector tracking (swing arm, position only)
    Kp_ee: np.ndarray = field(default_factory=lambda: 80.0 * np.ones(3))
    Kd_ee: np.ndarray = field(default_factory=lambda: 15.0 * np.ones(3))

    # PD gains for posture regulation
    Kp_posture: float = 25.0
    Kd_posture: float = 10.0

    # Actuator limits
    tau_max: np.ndarray = field(default_factory=lambda: 50.0 * np.ones(14))  # [Nm]

    # Joint acceleration limits
    qdd_max: float = 50.0         # [rad/s²]

    # Momentum safety
    dt_qp: float = 0.008          # QP time step [s] (125 Hz)

    # Contact wrench limits (HOTDOCK)
    f_max: float = 3000.0         # [N]
    tau_contact_max: float = 300.0  # [Nm]

    # Robot angular momentum constraints
    L_max: float = np.inf          # |L_robot| ≤ L_max [Nms]
    tau_w_max: float = np.inf      # |L̇_robot| ≤ τ_w_max [Nm]


class WholeBodyQP:
    """Whole-body QP for high-rate tracking of centroidal references.

    Parameters
    ----------
    config : WholeBodyQPConfig
        Problem configuration.
    """

    def __init__(self, config: Optional[WholeBodyQPConfig] = None):
        if config is None:
            config = WholeBodyQPConfig()
        self.config = config

        nq = config.nq
        nc_max = config.nc_max

        # Decision variable dimensions
        self._dim_qdd_t = 6                   # Torso acceleration
        self._dim_qdd = nq                    # Joint accelerations
        self._dim_lambda = 6 * nc_max         # Contact wrenches
        self._dim_tau = nq                     # Joint torques

        self._n_vars = (self._dim_qdd_t + self._dim_qdd +
                        self._dim_lambda + self._dim_tau)

        # Variable index ranges in z
        self._idx = self._compute_indices()

        # Nominal posture (set by user, default: zero)
        self._q_nominal = np.zeros(nq)

    def set_nominal_posture(self, q_nom: np.ndarray) -> None:
        """Set the nominal joint posture for regularization.

        Parameters
        ----------
        q_nom : ndarray (nq,)
            Preferred joint configuration (e.g., arms mid-range).
        """
        self._q_nominal = np.asarray(q_nom).ravel()

    def solve(
        self,
        # Robot state
        q_t: np.ndarray,
        dq_t: np.ndarray,
        q: np.ndarray,
        dq: np.ndarray,
        # References from Stage 1 (CentroidalNMPC)
        r_com_ref: np.ndarray,
        v_com_ref: np.ndarray,
        lambda_ref: np.ndarray,
        a_com_ff: np.ndarray,
        # Dynamics data (from Pinocchio)
        H_robot: np.ndarray,
        C_robot: np.ndarray,
        J_com: np.ndarray,
        Jdot_dq_com: np.ndarray,
        # Contact data
        contact_config: ContactConfig,
        J_contacts: Optional[np.ndarray] = None,
        Jdot_dq_contacts: Optional[np.ndarray] = None,
        # Momentum data
        hw_current: Optional[np.ndarray] = None,
        hw_min: Optional[np.ndarray] = None,
        hw_max: Optional[np.ndarray] = None,
        # Current CoM (for momentum map computation)
        r_com: Optional[np.ndarray] = None,
        # Current robot angular momentum
        L_com_current: Optional[np.ndarray] = None,
        # End-effector tracking (swing arm, optional)
        J_ee: Optional[np.ndarray] = None,         # (3, 6+nq) or (6, 6+nq)
        Jdot_dq_ee: Optional[np.ndarray] = None,   # (3,) or (6,)
        p_ee_ref: Optional[np.ndarray] = None,      # (3,) desired EE position
        v_ee_ref: Optional[np.ndarray] = None,      # (3,) desired EE velocity
        a_ee_ff: Optional[np.ndarray] = None,       # (3,) feedforward EE acceleration
        p_ee: Optional[np.ndarray] = None,           # (3,) current EE position
        # Torso 6D tracking (optional, replaces CoM task when active)
        J_torso: Optional[np.ndarray] = None,       # (6, 6+nq) torso Jacobian
        Jdot_dq_torso: Optional[np.ndarray] = None, # (6,) J̇_torso · q̇
        p_torso: Optional[np.ndarray] = None,        # (3,) current torso position
        R_torso: Optional[np.ndarray] = None,        # (3,3) current torso rotation
        p_torso_ref: Optional[np.ndarray] = None,    # (3,) desired torso position
        R_torso_ref: Optional[np.ndarray] = None,    # (3,3) desired torso rotation
        v_torso_ref: Optional[np.ndarray] = None,    # (6,) desired torso twist [lin(3), ang(3)]
        a_torso_ff: Optional[np.ndarray] = None,     # (6,) feedforward torso accel [lin(3), ang(3)]
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, QPSolveInfo]:
        """Solve the whole-body QP.

        Parameters
        ----------
        q_t : ndarray (7,)
            Torso pose [quaternion(4), position(3)]. Used for state only.
        dq_t : ndarray (6,)
            Torso twist [angular(3), linear(3)].
        q : ndarray (nq,)
            Joint positions.
        dq : ndarray (nq,)
            Joint velocities.
        r_com_ref : ndarray (3,)
            Reference CoM position from Stage 1.
        v_com_ref : ndarray (3,)
            Reference CoM velocity from Stage 1.
        lambda_ref : ndarray (12,)
            Reference contact wrenches from Stage 1.
        a_com_ff : ndarray (3,)
            Feedforward CoM acceleration = (1/m)Σf_j_ref.
        H_robot : ndarray (6+nq, 6+nq)
            Robot mass/inertia matrix (from Pinocchio CRBA).
        C_robot : ndarray (6+nq,)
            Coriolis/centrifugal + gravity terms (from Pinocchio RNEA).
        J_com : ndarray (3, 6+nq)
            Robot CoM Jacobian (from Pinocchio).
        Jdot_dq_com : ndarray (3,)
            J̇_com · q̇_robot (Pinocchio: computeAllTerms provides this).
        contact_config : ContactConfig
            Contact phase and positions.
        J_contacts : ndarray (6·nc, 6+nq), optional
            Stacked contact Jacobians (6 rows per active contact).
            Required if nc > 0.
        Jdot_dq_contacts : ndarray (6·nc,), optional
            J̇_contact · q̇_robot for active contacts.
            Required if nc > 0.
        hw_current : ndarray (3,), optional
            Current wheel momentum. Required for momentum constraint.
        hw_min, hw_max : ndarray (3,), optional
            Wheel momentum bounds. Required for momentum constraint.
        r_com : ndarray (3,), optional
            Current CoM position. Required for momentum constraint.

        Returns
        -------
        qdd_t : ndarray (6,)
            Optimal torso acceleration.
        qdd : ndarray (nq,)
            Optimal joint accelerations.
        lambda_opt : ndarray (12,)
            Optimal contact wrenches.
        tau_q : ndarray (nq,)
            Optimal joint torques (direct actuator commands).
        info : QPSolveInfo
            Solver information.
        """
        cfg = self.config
        idx = self._idx
        n = self._n_vars
        nq = cfg.nq
        n_robot = 6 + nq  # dimension of q̈_robot = [q̈_t; q̈]

        # --- Build QP ---
        qp = HierarchicalQP(
            n_vars=n, method=cfg.method, solver=cfg.solver,
        )

        # ============================================================ #
        #  EQUALITY CONSTRAINTS                                         #
        # ============================================================ #

        # 1. Full robot dynamics: H q̈_robot + C = B_u τ_q + J_robot^T λ
        #    → [H, -J_robot^T, -B_u] [q̈_robot; λ; τ_q] = -C
        #
        # Decision vector z = [q̈_t, q̈, λ, τ_q]
        # q̈_robot = [q̈_t; q̈] are the first (6+nq) components

        A_dyn = np.zeros((n_robot, n))

        # H_robot @ q̈_robot
        A_dyn[:, idx['qdd_t'][0]: idx['qdd_t'][1]] = H_robot[:, :6]
        A_dyn[:, idx['qdd'][0]: idx['qdd'][1]] = H_robot[:, 6:]

        # -J_robot^T @ λ  (only active contacts contribute)
        n_lambda = self._dim_lambda
        J_robot_T = np.zeros((n_robot, n_lambda))
        if J_contacts is not None and J_contacts.size > 0:
            # J_contacts is (6·nc_active, n_robot) for active contacts
            # We need to place it in the right columns of the full λ vector
            nc_active = J_contacts.shape[0] // 6
            col_offset = 0
            contact_idx = 0
            for j in range(cfg.nc_max):
                if contact_config.active_contacts[j]:
                    rows = slice(contact_idx * 6, (contact_idx + 1) * 6)
                    J_robot_T[:, j * 6: (j + 1) * 6] = J_contacts[rows, :].T
                    contact_idx += 1

        A_dyn[:, idx['lambda'][0]: idx['lambda'][1]] = -J_robot_T

        # -B_u @ τ_q  (B_u = [0_{6×nq}; I_nq])
        A_dyn[6:, idx['tau'][0]: idx['tau'][1]] = -np.eye(nq)

        b_dyn = -C_robot

        qp.add_equality_constraint(A_dyn, b_dyn)

        # 2. Contact acceleration constraint: J_contact q̈_robot = -J̇_contact q̇_robot
        if J_contacts is not None and J_contacts.size > 0:
            nc_active_rows = J_contacts.shape[0]
            A_contact = np.zeros((nc_active_rows, n))
            A_contact[:, idx['qdd_t'][0]: idx['qdd_t'][1]] = J_contacts[:, :6]
            A_contact[:, idx['qdd'][0]: idx['qdd'][1]] = J_contacts[:, 6:]

            b_contact = np.zeros(nc_active_rows)
            if Jdot_dq_contacts is not None:
                b_contact = -Jdot_dq_contacts

            qp.add_equality_constraint(A_contact, b_contact)

        # ============================================================ #
        #  INEQUALITY CONSTRAINTS                                       #
        # ============================================================ #

        # 1. Momentum safety: h_min ≤ hw - dt·M_λ·λ ≤ h_max
        #    Upper: -dt·M_λ·λ ≤ h_max - hw  →  [0, 0, -dt·M_λ, 0] z ≤ h_max - hw
        #    Lower:  dt·M_λ·λ ≤ hw - h_min   →  [0, 0,  dt·M_λ, 0] z ≤ hw - h_min
        if hw_current is not None and hw_min is not None and r_com is not None:
            M_lambda = compute_momentum_map(r_com, contact_config)

            A_mom_upper = np.zeros((3, n))
            A_mom_upper[:, idx['lambda'][0]: idx['lambda'][1]] = -cfg.dt_qp * M_lambda
            b_mom_upper = hw_max - hw_current

            A_mom_lower = np.zeros((3, n))
            A_mom_lower[:, idx['lambda'][0]: idx['lambda'][1]] = cfg.dt_qp * M_lambda
            b_mom_lower = hw_current - hw_min

            qp.add_inequality_constraint(
                np.vstack([A_mom_upper, A_mom_lower]),
                np.concatenate([b_mom_upper, b_mom_lower])
            )

            # 2. Robot angular momentum box: |L_com + dt·M_λ·λ| ≤ L_max
            if np.isfinite(cfg.L_max) and L_com_current is not None:
                A_L_upper = np.zeros((3, n))
                A_L_upper[:, idx['lambda'][0]: idx['lambda'][1]] = cfg.dt_qp * M_lambda
                b_L_upper = cfg.L_max * np.ones(3) - L_com_current

                A_L_lower = np.zeros((3, n))
                A_L_lower[:, idx['lambda'][0]: idx['lambda'][1]] = -cfg.dt_qp * M_lambda
                b_L_lower = cfg.L_max * np.ones(3) + L_com_current

                qp.add_inequality_constraint(
                    np.vstack([A_L_upper, A_L_lower]),
                    np.concatenate([b_L_upper, b_L_lower])
                )

            # 3. Momentum rate box: |L̇_robot| = |M_λ·λ| ≤ τ_w_max
            if np.isfinite(cfg.tau_w_max):
                A_Ld_upper = np.zeros((3, n))
                A_Ld_upper[:, idx['lambda'][0]: idx['lambda'][1]] = M_lambda
                b_Ld_upper = cfg.tau_w_max * np.ones(3)

                A_Ld_lower = np.zeros((3, n))
                A_Ld_lower[:, idx['lambda'][0]: idx['lambda'][1]] = -M_lambda
                b_Ld_lower = cfg.tau_w_max * np.ones(3)

                qp.add_inequality_constraint(
                    np.vstack([A_Ld_upper, A_Ld_lower]),
                    np.concatenate([b_Ld_upper, b_Ld_lower])
                )

        # ============================================================ #
        #  BOUNDS                                                       #
        # ============================================================ #

        lb = np.full(n, -np.inf)
        ub = np.full(n, np.inf)

        # Joint acceleration bounds
        lb[idx['qdd'][0]: idx['qdd'][1]] = -cfg.qdd_max
        ub[idx['qdd'][0]: idx['qdd'][1]] = cfg.qdd_max

        # Joint torque bounds
        lb[idx['tau'][0]: idx['tau'][1]] = -cfg.tau_max
        ub[idx['tau'][0]: idx['tau'][1]] = cfg.tau_max

        # Contact wrench bounds (zero for inactive contacts)
        for j in range(cfg.nc_max):
            s = idx['lambda'][0] + j * 6
            if contact_config.active_contacts[j]:
                lb[s: s + 3] = -cfg.f_max
                ub[s: s + 3] = cfg.f_max
                lb[s + 3: s + 6] = -cfg.tau_contact_max
                ub[s + 3: s + 6] = cfg.tau_contact_max
            else:
                lb[s: s + 6] = 0.0
                ub[s: s + 6] = 0.0

        qp.set_bounds(lb, ub)

        # ============================================================ #
        #  TASKS                                                        #
        # ============================================================ #

        dq_robot = np.concatenate([dq_t, dq])

        # --- Task 1: CoM tracking (highest priority) ---
        # r̈_com_des = a_ff + Kp(r_ref - r_com_actual) + Kd(v_ref - v_com_actual)
        # We need current CoM pos/vel from J_com and state
        r_com_actual = r_com if r_com is not None else np.zeros(3)
        v_com_actual = J_com @ dq_robot

        Kp = np.diag(cfg.Kp_com)
        Kd = np.diag(cfg.Kd_com)
        a_com_des = (a_com_ff +
                     Kp @ (r_com_ref - r_com_actual) +
                     Kd @ (v_com_ref - v_com_actual))

        # Task: J_com @ q̈_robot = a_com_des - J̇_com q̇_robot
        A_com = np.zeros((3, n))
        A_com[:, idx['qdd_t'][0]: idx['qdd_t'][1]] = J_com[:, :6]
        A_com[:, idx['qdd'][0]: idx['qdd'][1]] = J_com[:, 6:]
        b_com = a_com_des - Jdot_dq_com

        if cfg.alpha_com > 0:
            qp.add_task(A_com, b_com, cfg.alpha_com, priority=1)

        # --- Task 1b: Torso 6D tracking (replaces CoM when active) ---
        # Controls both position and orientation of the torso body frame.
        # a_torso_des = [a_lin_ff + Kp_lin(p_ref - p) + Kd_lin(v_ref - v);
        #                a_ang_ff + Kp_ang(log3(R^T R_ref)) + Kd_ang(ω_ref - ω)]
        if cfg.alpha_torso > 0 and J_torso is not None and p_torso_ref is not None:
            Kp_t = np.diag(cfg.Kp_torso)   # (6,6)
            Kd_t = np.diag(cfg.Kd_torso)   # (6,6)

            # Current torso twist from Jacobian
            v_torso_actual = J_torso @ dq_robot  # (6,): [lin(3), ang(3)]

            # Position error (3,)
            p_t = p_torso if p_torso is not None else np.zeros(3)
            e_pos = p_torso_ref - p_t

            # Orientation error via log map (3,)
            R_t = R_torso if R_torso is not None else np.eye(3)
            R_ref_t = R_torso_ref if R_torso_ref is not None else np.eye(3)
            e_ori = pin.log3(R_t.T @ R_ref_t)

            # 6D error
            e_6d = np.concatenate([e_pos, e_ori])

            # Reference twist and feedforward
            v_ref_t = v_torso_ref if v_torso_ref is not None else np.zeros(6)
            a_ff_t = a_torso_ff if a_torso_ff is not None else np.zeros(6)

            # Desired 6D acceleration
            a_torso_des = a_ff_t + Kp_t @ e_6d + Kd_t @ (v_ref_t - v_torso_actual)

            # Task: J_torso @ q̈_robot = a_torso_des - J̇_torso q̇_robot
            jdq = Jdot_dq_torso if Jdot_dq_torso is not None else np.zeros(6)
            A_torso = np.zeros((6, n))
            A_torso[:, idx['qdd_t'][0]: idx['qdd_t'][1]] = J_torso[:, :6]
            A_torso[:, idx['qdd'][0]: idx['qdd'][1]] = J_torso[:, 6:]
            b_torso = a_torso_des - jdq

            qp.add_task(A_torso, b_torso, cfg.alpha_torso, priority=1)

        # --- Task 2: End-effector tracking (swing arm, optional) ---
        if J_ee is not None and p_ee_ref is not None:
            # Position-only tracking (use top 3 rows if J_ee is 6×nv)
            J_ee_pos = J_ee[:3, :] if J_ee.shape[0] >= 6 else J_ee
            Jdot_dq = Jdot_dq_ee[:3] if (Jdot_dq_ee is not None and
                                           Jdot_dq_ee.shape[0] >= 6) else (
                Jdot_dq_ee if Jdot_dq_ee is not None else np.zeros(3))

            # Current EE velocity from Jacobian
            v_ee_actual = J_ee_pos @ dq_robot
            p_ee_actual = p_ee if p_ee is not None else np.zeros(3)
            v_ref = v_ee_ref if v_ee_ref is not None else np.zeros(3)

            Kp_ee = np.diag(cfg.Kp_ee)
            Kd_ee = np.diag(cfg.Kd_ee)
            a_ff_ee = a_ee_ff if a_ee_ff is not None else np.zeros(3)
            a_ee_des = (a_ff_ee +
                        Kp_ee @ (p_ee_ref - p_ee_actual) +
                        Kd_ee @ (v_ref - v_ee_actual))

            A_ee = np.zeros((3, n))
            A_ee[:, idx['qdd_t'][0]: idx['qdd_t'][1]] = J_ee_pos[:, :6]
            A_ee[:, idx['qdd'][0]: idx['qdd'][1]] = J_ee_pos[:, 6:]
            b_ee = a_ee_des - Jdot_dq

            qp.add_task(A_ee, b_ee, cfg.alpha_ee, priority=2)

        # --- Task 3: Posture regulation ---
        # q̈_posture = Kp_post (q_nom - q) + Kd_post (0 - dq)
        qdd_posture = (cfg.Kp_posture * (self._q_nominal - q) -
                       cfg.Kd_posture * dq)

        A_posture = np.zeros((nq, n))
        A_posture[:, idx['qdd'][0]: idx['qdd'][1]] = np.eye(nq)
        b_posture = qdd_posture

        qp.add_task(A_posture, b_posture, cfg.alpha_posture, priority=3)

        # --- Task 3: Contact wrench tracking ---
        A_wrench = np.zeros((self._dim_lambda, n))
        A_wrench[:, idx['lambda'][0]: idx['lambda'][1]] = np.eye(self._dim_lambda)
        b_wrench = lambda_ref

        qp.add_task(A_wrench, b_wrench, cfg.alpha_wrench, priority=4)

        # --- Task 4: Joint torque minimization ---
        A_torque = np.zeros((nq, n))
        A_torque[:, idx['tau'][0]: idx['tau'][1]] = np.eye(nq)
        b_torque = np.zeros(nq)

        qp.add_task(A_torque, b_torque, cfg.alpha_torque, priority=5)

        # --- Task 5: Acceleration regularization ---
        A_reg = np.zeros((6 + nq, n))
        A_reg[:6, idx['qdd_t'][0]: idx['qdd_t'][1]] = np.eye(6)
        A_reg[6:, idx['qdd'][0]: idx['qdd'][1]] = np.eye(nq)
        b_reg = np.zeros(6 + nq)

        qp.add_task(A_reg, b_reg, cfg.alpha_reg, priority=6)

        # ============================================================ #
        #  SOLVE                                                        #
        # ============================================================ #

        z_opt, info = qp.solve()

        # --- Extract solution ---
        qdd_t_opt = z_opt[idx['qdd_t'][0]: idx['qdd_t'][1]]
        qdd_opt = z_opt[idx['qdd'][0]: idx['qdd'][1]]
        lambda_opt = z_opt[idx['lambda'][0]: idx['lambda'][1]]
        tau_q_opt = z_opt[idx['tau'][0]: idx['tau'][1]]

        return qdd_t_opt, qdd_opt, lambda_opt, tau_q_opt, info

    # ------------------------------------------------------------------ #
    #  Private helpers                                                     #
    # ------------------------------------------------------------------ #

    def _compute_indices(self) -> Dict[str, Tuple[int, int]]:
        """Compute start/end indices for each variable block in z."""
        nq = self.config.nq
        n_lambda = 6 * self.config.nc_max

        s = 0
        idx = {}

        idx['qdd_t'] = (s, s + 6);           s += 6
        idx['qdd'] = (s, s + nq);            s += nq
        idx['lambda'] = (s, s + n_lambda);    s += n_lambda
        idx['tau'] = (s, s + nq);             s += nq

        assert s == self._n_vars
        return idx

    @property
    def n_vars(self) -> int:
        return self._n_vars

    @property
    def variable_indices(self) -> Dict[str, Tuple[int, int]]:
        """Index ranges for each variable block."""
        return self._idx

    def __repr__(self) -> str:
        return (
            f"WholeBodyQP(nq={self.config.nq}, nc_max={self.config.nc_max}, "
            f"n_vars={self._n_vars}, method='{self.config.method}')"
        )
