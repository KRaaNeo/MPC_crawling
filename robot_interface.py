"""
RobotInterface — Pinocchio wrapper for the VISPA crawling controller.

Computes at each timestep all quantities needed by the two-stage controller:
    Stage 1 (CentroidalNMPC): r_com, v_com, L_com
    Stage 2 (WholeBodyQP):    H_robot, C_robot, J_com, J̇_com·q̇,
                               J_contacts, J̇_contacts·q̇, joint limits

Conventions (Pinocchio free-flyer):
    q  = [pos(3), quat_xyzw(4), joints(12)]   → nq = 19
    v  = [twist(6), qdot(12)]                  → nv = 18
    q̈_robot = [q̈_t(6), q̈(12)]               → 18-dim
    τ  = [base_wrench(6)=0, torques(12)]       → 18-dim

Frame IDs (from URDF):
    FRAME_TOOL_A = 18    (end-effector arm A)
    FRAME_TOOL_B = 32    (end-effector arm B)
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional, Dict, Any

import pinocchio as pin


# ── Frame / joint indices (must match dynamics.py) ────────────────────────────
FRAME_TORSO = 4    # Link_0 (torso body frame)
FRAME_TOOL_A = 18
FRAME_TOOL_B = 32
JOINT_6A_ID = 7
JOINT_6B_ID = 13

# Number of actuated joints
N_JOINTS = 12
NQ = 19
NV = 18


@dataclass
class RobotState:
    """All Pinocchio-computed quantities at one timestep.

    Populated by RobotInterface.update(q, v).
    """
    # ── State ────────────────────────────────────────────────
    q: np.ndarray           # (19,) Pinocchio generalized positions
    v: np.ndarray           # (18,) Pinocchio generalized velocities
    q_joints: np.ndarray    # (12,) joint positions only
    dq_joints: np.ndarray   # (12,) joint velocities only
    q_torso: np.ndarray     # (7,)  torso pose [pos(3), quat_xyzw(4)]
    dq_torso: np.ndarray    # (6,)  torso twist

    # ── Dynamics ─────────────────────────────────────────────
    H: np.ndarray           # (18,18) Mass matrix (CRBA)
    C: np.ndarray           # (18,) Bias term (Coriolis + gravity via RNEA)

    # ── Center of Mass ───────────────────────────────────────
    r_com: np.ndarray       # (3,) CoM position
    v_com: np.ndarray       # (3,) CoM velocity
    J_com: np.ndarray       # (3,18) CoM Jacobian
    Jdot_dq_com: np.ndarray # (3,) J̇_com · q̇

    # ── Centroidal momentum ──────────────────────────────────
    h_centroidal: np.ndarray  # (6,) [linear(3); angular(3)]
    L_com: np.ndarray         # (3,) angular part only

    # ── Contact frames ───────────────────────────────────────
    oMf_tool_a: Any         # SE3 placement of tool_a in world
    oMf_tool_b: Any         # SE3 placement of tool_b in world
    J_tool_a: np.ndarray    # (6,18) Jacobian of tool_a (LOCAL_WORLD_ALIGNED)
    J_tool_b: np.ndarray    # (6,18) Jacobian of tool_b
    Jdot_dq_tool_a: np.ndarray  # (6,) J̇_a · q̇
    Jdot_dq_tool_b: np.ndarray  # (6,) J̇_b · q̇

    # ── Torso frame ──────────────────────────────────────────
    oMf_torso: Any          # SE3 placement of torso in world
    J_torso: np.ndarray     # (6,18) Jacobian of torso frame (LOCAL_WORLD_ALIGNED)
    Jdot_dq_torso: np.ndarray  # (6,) J̇_torso · q̇

    # ── Joint limits ─────────────────────────────────────────
    q_min: np.ndarray       # (12,) lower joint limits
    q_max: np.ndarray       # (12,) upper joint limits
    tau_max: np.ndarray     # (12,) torque limits (absolute)

    # ── Total mass ───────────────────────────────────────────
    total_mass: float


class RobotInterface:
    """Pinocchio computation wrapper for the VISPA controller.

    Call update(q, v) at each timestep to compute all required quantities.
    Then access them via the `state` property.

    Parameters
    ----------
    urdf_path : str
        Path to the VISPA URDF (free-flyer base).
    tau_max : float or ndarray (12,), optional
        Joint torque limits. Default: 10 Nm per joint.
    gravity : str, optional
        'zero'/'micro' (default) for orbital operations, 'earth' for ground tests.
    """

    def __init__(self, urdf_path: str, tau_max=10.0, gravity: str = 'zero',
                 torso_mass: float = None):
        # Build model with free-flyer
        self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()

        # Override torso mass if specified (URDF value may be incorrect)
        if torso_mass is not None:
            old_I = self.model.inertias[1]  # idx 1 = root_joint body
            ratio = torso_mass / max(old_I.mass, 1e-6)
            new_inertia = old_I.inertia * ratio  # scale tensor
            self.model.inertias[1] = pin.Inertia(
                torso_mass, old_I.lever, new_inertia)
            self.data = self.model.createData()

        # Gravity setting
        if gravity == 'zero' or gravity == 'micro':
            self.model.gravity = pin.Motion.Zero()
        elif gravity == 'earth':
            pass  # keep default 9.81
        # else: keep default

        assert self.model.nq == NQ, f"Expected nq={NQ}, got {self.model.nq}"
        assert self.model.nv == NV, f"Expected nv={NV}, got {self.model.nv}"

        # Joint limits (from URDF)
        # Free-flyer has no limits in Pinocchio → joints start at index 1
        # model.lowerPositionLimit / upperPositionLimit are (nq,)
        # Joint DOFs in velocity space: indices 6..17
        self._q_min = self.model.lowerPositionLimit[7:19].copy()
        self._q_max = self.model.upperPositionLimit[7:19].copy()

        # Torque limits
        if np.isscalar(tau_max):
            self._tau_max = np.full(N_JOINTS, tau_max)
        else:
            self._tau_max = np.array(tau_max).ravel()

        # Total mass (computed once)
        data_tmp = self.model.createData()
        pin.computeTotalMass(self.model, data_tmp)
        self._total_mass = data_tmp.mass[0]

        # State placeholder
        self._state: Optional[RobotState] = None

    def update(self, q: np.ndarray, v: np.ndarray) -> RobotState:
        """Compute all controller-required quantities from (q, v).

        Parameters
        ----------
        q : ndarray (19,)
            Pinocchio generalized positions.
        v : ndarray (18,)
            Pinocchio generalized velocities.

        Returns
        -------
        state : RobotState
            All computed quantities.
        """
        model = self.model
        data = self.data

        # ── Kinematics + dynamics in one pass ────────────────────
        # computeAllTerms computes: CRBA, RNEA(bias), CoM, Jacobians, etc.
        pin.computeAllTerms(model, data, q, v)
        # Frame placements need explicit update
        pin.updateFramePlacements(model, data)

        # ── Mass matrix H = M(q) ────────────────────────────────
        # CRBA is already called by computeAllTerms
        H = data.M.copy()
        # Make symmetric (Pinocchio fills upper triangle)
        H = (H + H.T) / 2.0

        # ── Bias vector C(q, v) = h(q, v) ───────────────────────
        # RNEA bias = h(q, v, 0) = C(q,v)q̇ + g(q)
        # computeAllTerms already computed nle (non-linear effects)
        C = data.nle.copy()

        # ── Center of Mass ───────────────────────────────────────
        r_com = data.com[0].copy()

        # CoM velocity: v_com = J_com @ v
        J_com = data.Jcom.copy()  # (3, nv)
        v_com = J_com @ v

        # J̇_com · q̇: computeAllTerms does NOT populate data.acom.
        # Must call centerOfMass(q, v, a=0) explicitly:
        #   acom = J_com @ q̈ + J̇_com @ q̇  with q̈=0 → acom = J̇_com @ q̇
        a_zero = np.zeros(model.nv)
        pin.centerOfMass(model, data, q, v, a_zero)
        Jdot_dq_com = data.acom[0].copy()

        # ── Centroidal momentum ──────────────────────────────────
        # Already computed by computeAllTerms (via computeCentroidalMomentum)
        h_centroidal = data.hg.vector.copy()  # (6,)
        L_com = h_centroidal[3:6]  # angular part

        # ── Contact frame Jacobians ──────────────────────────────
        # Full frame Jacobians in LOCAL_WORLD_ALIGNED
        J_tool_a = pin.getFrameJacobian(
            model, data, FRAME_TOOL_A, pin.LOCAL_WORLD_ALIGNED).copy()
        J_tool_b = pin.getFrameJacobian(
            model, data, FRAME_TOOL_B, pin.LOCAL_WORLD_ALIGNED).copy()

        # J̇·q̇ for contact frames
        # We need computeJointJacobiansTimeVariation which is NOT part
        # of computeAllTerms → call it explicitly
        pin.computeJointJacobiansTimeVariation(model, data, q, v)
        dJ_a = pin.getFrameJacobianTimeVariation(
            model, data, FRAME_TOOL_A, pin.LOCAL_WORLD_ALIGNED)
        dJ_b = pin.getFrameJacobianTimeVariation(
            model, data, FRAME_TOOL_B, pin.LOCAL_WORLD_ALIGNED)
        Jdot_dq_tool_a = (dJ_a @ v).copy()
        Jdot_dq_tool_b = (dJ_b @ v).copy()

        # ── Frame placements ─────────────────────────────────────
        oMf_tool_a = data.oMf[FRAME_TOOL_A].copy()
        oMf_tool_b = data.oMf[FRAME_TOOL_B].copy()
        oMf_torso = data.oMf[FRAME_TORSO].copy()

        # ── Torso frame Jacobian ─────────────────────────────────
        J_torso = pin.getFrameJacobian(
            model, data, FRAME_TORSO, pin.LOCAL_WORLD_ALIGNED).copy()
        dJ_torso = pin.getFrameJacobianTimeVariation(
            model, data, FRAME_TORSO, pin.LOCAL_WORLD_ALIGNED)
        Jdot_dq_torso = (dJ_torso @ v).copy()

        # ── Assemble state ───────────────────────────────────────
        self._state = RobotState(
            q=q.copy(),
            v=v.copy(),
            q_joints=q[7:19].copy(),
            dq_joints=v[6:18].copy(),
            q_torso=q[0:7].copy(),
            dq_torso=v[0:6].copy(),
            H=H,
            C=C,
            r_com=r_com,
            v_com=v_com,
            J_com=J_com,
            Jdot_dq_com=Jdot_dq_com,
            h_centroidal=h_centroidal,
            L_com=L_com,
            oMf_tool_a=oMf_tool_a,
            oMf_tool_b=oMf_tool_b,
            J_tool_a=J_tool_a,
            J_tool_b=J_tool_b,
            Jdot_dq_tool_a=Jdot_dq_tool_a,
            Jdot_dq_tool_b=Jdot_dq_tool_b,
            oMf_torso=oMf_torso,
            J_torso=J_torso,
            Jdot_dq_torso=Jdot_dq_torso,
            q_min=self._q_min,
            q_max=self._q_max,
            tau_max=self._tau_max,
            total_mass=self._total_mass,
        )
        return self._state

    @property
    def state(self) -> RobotState:
        """Most recently computed state."""
        if self._state is None:
            raise RuntimeError("Call update(q, v) first.")
        return self._state

    # ── Convenience methods ──────────────────────────────────────

    def get_contact_jacobians(
        self, active_A: bool, active_B: bool
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Return stacked contact Jacobian and J̇·q̇ for active contacts.

        Returns
        -------
        J_contacts : ndarray (6*nc, 18) or None
            Stacked Jacobians, A then B.
        Jdot_dq_contacts : ndarray (6*nc,) or None
            Stacked J̇·q̇ vectors.
        """
        s = self.state
        Js, Jds = [], []
        if active_A:
            Js.append(s.J_tool_a)
            Jds.append(s.Jdot_dq_tool_a)
        if active_B:
            Js.append(s.J_tool_b)
            Jds.append(s.Jdot_dq_tool_b)
        if not Js:
            return None, None
        return np.vstack(Js), np.concatenate(Jds)

    def neutral_configuration(self) -> np.ndarray:
        """Pinocchio neutral configuration (identity base, zero joints)."""
        return pin.neutral(self.model)
