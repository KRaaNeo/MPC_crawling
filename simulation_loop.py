"""
SimulationLoop — Closed-loop MuJoCo simulation with two-stage controller.

Orchestrates full locomotion pipeline for the VISPA crawling robot:

    TorsoPlanner → CoM ref → CentroidalNMPC → WholeBodyQP → MuJoCo

Architecture per NMPC step (10 Hz):
    1. Read MuJoCo state → Pinocchio
    2. TorsoPlanner.reference_at(t) → 6D torso ref
    3. TorsoPlanner.com_reference_at(t) → CoM ref (derived from torso)
    4. CentroidalNMPC.solve(r_com_ref, ...) → λ_ref, a_ff
    5. Inner loop (100 Hz QP):
        a. WholeBodyQP.solve(torso, EE, CoM, wrenches, momentum) → τ
        b. τ → MuJoCo actuators → mj_step
        c. hw update (simplified AOCS)

Phase machine per step:
    DS (double support, 0.5s) → SS (single support, T_swing) → EXT (extension)
    EXT ends when d_ee < WELD_R (real dock) or timeout → DS of next step

Constraints:
    |L_com| ≤ L_max           (robot angular momentum box)
    |L̇_com| ≤ τ_w_max         (reaction wheel torque box)
    h_min ≤ h_w ≤ h_max       (wheel storage box)
    |τ_joint| ≤ τ_max          (joint actuator limits)

Usage:
    sim = SimulationLoop(
        mjcf_path='VISPA_crawling.xml',
        urdf_path='VISPA_crawling_fixed.urdf',
    )
    sim.setup(n_steps=3, start_a=2, start_b=2)
    log = sim.run()
    sim.plot(log)
"""

import numpy as np
import time
import json
from dataclasses import dataclass, field
from typing import Optional, Tuple, Dict, Any, List
from pathlib import Path

try:
    import mujoco
except ImportError:
    mujoco = None

try:
    import pinocchio as pin
except ImportError:
    pin = None

from robot_interface import RobotInterface
from contact_scheduler import ContactScheduler, read_anchors_from_mujoco
from locomotion_planner import LocomotionPlanner
from swing_planner import SwingPlanner
from torso_planner import TorsoPlanner
from ik import dock_configuration
from solvers.centroidal_nmpc import CentroidalNMPC, CentroidalNMPCConfig
from solvers.wholebody_qp import WholeBodyQP, WholeBodyQPConfig
from solvers.contact_phase import ContactConfig


# ── State conversions ────────────────────────────────────────────────────────

def mujoco_to_pinocchio(mj_qpos, mj_qvel):
    """Convert MuJoCo state to Pinocchio convention.

    MuJoCo: qpos = [struct_pos(3), struct_quat_wxyz(4),
                     torso_pos(3), torso_quat_wxyz(4), joints(12)]
    Pinocchio: q = [torso_pos(3), torso_quat_xyzw(4), joints(12)]
    """
    pin_q = np.zeros(19)
    pin_v = np.zeros(18)
    pin_q[0:3] = mj_qpos[7:10]
    w, x, y, z = mj_qpos[10:14]
    pin_q[3:7] = [x, y, z, w]
    pin_q[7:19] = mj_qpos[14:26]
    pin_v[0:3] = mj_qvel[6:9]
    pin_v[3:6] = mj_qvel[9:12]
    pin_v[6:18] = mj_qvel[12:24]
    return pin_q, pin_v


def pinocchio_to_mujoco(pin_q, pin_v, struct_pos=None, struct_quat=None):
    """Convert Pinocchio state to MuJoCo convention."""
    mj_qpos = np.zeros(26)
    mj_qvel = np.zeros(24)
    mj_qpos[0:3] = struct_pos if struct_pos is not None else np.zeros(3)
    mj_qpos[3:7] = struct_quat if struct_quat is not None else [1, 0, 0, 0]
    mj_qpos[7:10] = pin_q[0:3]
    x, y, z, w_ = pin_q[3:7]
    mj_qpos[10:14] = [w_, x, y, z]
    mj_qpos[14:26] = pin_q[7:19]
    mj_qvel[6:9] = pin_v[0:3]
    mj_qvel[9:12] = pin_v[3:6]
    mj_qvel[12:24] = pin_v[6:18]
    return mj_qpos, mj_qvel


# ── Configuration ────────────────────────────────────────────────────────────

@dataclass
class SimConfig:
    """Full simulation configuration."""

    # Timing
    dt_nmpc: float = 0.1          # NMPC period [s] (10 Hz)
    dt_qp: float = 0.01           # QP/MuJoCo period [s] (100 Hz)
    t_ds: float = 0.5             # Double-support duration [s]
    t_swing: float = 6.0          # Single-support (swing) duration [s]
    t_ext_max: float = 5.0        # Max extension phase before timeout [s]

    # Torso trajectory
    torso_frac: float = 0.70      # Fraction of full IK displacement
    torso_delay: float = 0.20     # Delay before torso starts (fraction of t_swing)

    # Joint limits
    tau_max: float = 10.0         # Joint torque limit [Nm]
    torso_mass: float = 40.0      # Corrected torso mass [kg]

    # Docking
    weld_radius: float = 0.005    # Real dock threshold [m]

    # Momentum constraints
    hw_init: np.ndarray = field(default_factory=lambda: np.array([2., -1., 0.5]))
    hw_min: np.ndarray = field(default_factory=lambda: np.full(3, -50.))
    hw_max: np.ndarray = field(default_factory=lambda: np.full(3, 50.))
    L_max: float = 5.0            # Robot angular momentum limit [Nms]
    tau_w_max: float = 2.0        # Reaction wheel torque limit [Nm]

    # NMPC parameters
    nmpc_N: int = 8
    nmpc_dt: float = 0.1
    nmpc_f_max: float = 25.0
    nmpc_tau_max: float = 8.0

    # QP weights — Single-support phase
    ss_alpha_com: float = 2e2
    ss_alpha_torso: float = 5e2
    ss_alpha_ee: float = 3e3
    ss_alpha_posture: float = 2e1
    ss_alpha_wrench: float = 1e2

    # QP weights — Extension phase (freeze torso, max EE)
    ext_alpha_com: float = 1e2
    ext_alpha_torso: float = 5e1
    ext_alpha_ee: float = 1e4
    ext_alpha_posture: float = 5e0
    ext_alpha_wrench: float = 1e2

    # QP gains — Single-support
    ss_Kp_com: float = 3.0
    ss_Kd_com: float = 3.0
    ss_Kp_torso: float = 6.0
    ss_Kd_torso: float = 5.0
    ss_Kp_ee: float = 10.0
    ss_Kd_ee: float = 7.0

    # QP gains — Extension
    ext_Kp_com: float = 2.0
    ext_Kd_com: float = 2.0
    ext_Kp_torso: float = 3.0
    ext_Kd_torso: float = 3.0
    ext_Kp_ee: float = 25.0
    ext_Kd_ee: float = 12.0

    # Swing planner
    swing_clearance: float = 0.03  # [m]

    # MuJoCo settling
    n_settle_steps: int = 200


# ── Simulation log ───────────────────────────────────────────────────────────

@dataclass
class SimLog:
    """Comprehensive logged data from a simulation run."""

    t: list = field(default_factory=list)
    phase: list = field(default_factory=list)
    step_idx: list = field(default_factory=list)

    # Torso
    p_torso: list = field(default_factory=list)
    p_torso_ref: list = field(default_factory=list)
    e_torso_pos: list = field(default_factory=list)

    # End-effector
    d_grip_swing: list = field(default_factory=list)
    d_grip_stance: list = field(default_factory=list)
    swing_arm: list = field(default_factory=list)

    # CoM
    r_com: list = field(default_factory=list)
    r_com_ref: list = field(default_factory=list)
    e_com: list = field(default_factory=list)

    # Momentum
    L_com: list = field(default_factory=list)
    L_com_norm: list = field(default_factory=list)
    L_dot: list = field(default_factory=list)
    L_dot_norm: list = field(default_factory=list)
    hw: list = field(default_factory=list)

    # Torques
    tau: list = field(default_factory=list)
    tau_max_joint: list = field(default_factory=list)

    # Structure (free-floating body)
    struct_pos: list = field(default_factory=list)
    struct_quat: list = field(default_factory=list)
    struct_euler_deg: list = field(default_factory=list)

    # Solver diagnostics
    nmpc_ok: list = field(default_factory=list)
    qp_ok: list = field(default_factory=list)
    lambda_ref_norm: list = field(default_factory=list)

    # Dock events
    dock_events: list = field(default_factory=list)

    def to_dict(self) -> dict:
        d = {}
        for k, v in self.__dict__.items():
            d[k] = [x.tolist() if hasattr(x, 'tolist') else x for x in v]
        return d

    def save(self, path: str):
        with open(path, 'w') as f:
            json.dump(self.to_dict(), f)

    @staticmethod
    def load(path: str) -> 'SimLog':
        with open(path) as f:
            d = json.load(f)
        log = SimLog()
        for k, v in d.items():
            if hasattr(log, k):
                setattr(log, k, v)
        return log


# ── Helper ───────────────────────────────────────────────────────────────────

def quat_wxyz_to_euler_deg(qw, qx, qy, qz):
    """Quaternion (w,x,y,z) → Euler (roll,pitch,yaw) in degrees."""
    sinr = 2 * (qw * qx + qy * qz)
    cosr = 1 - 2 * (qx**2 + qy**2)
    roll = np.arctan2(sinr, cosr)
    sinp = np.clip(2 * (qw * qy - qz * qx), -1, 1)
    pitch = np.arcsin(sinp)
    siny = 2 * (qw * qz + qx * qy)
    cosy = 1 - 2 * (qy**2 + qz**2)
    yaw = np.arctan2(siny, cosy)
    return np.degrees(np.array([roll, pitch, yaw]))


# ── Simulation loop ──────────────────────────────────────────────────────────

class SimulationLoop:
    """Closed-loop MuJoCo simulation with hierarchical NMPC+QP controller."""

    def __init__(self, mjcf_path: str, urdf_path: str,
                 config: Optional[SimConfig] = None):
        assert mujoco is not None, "mujoco package required"
        assert pin is not None, "pinocchio package required"
        self.mjcf_path = mjcf_path
        self.urdf_path = urdf_path
        self.cfg = config or SimConfig()
        self.n_qp_per_nmpc = int(round(self.cfg.dt_nmpc / self.cfg.dt_qp))

        self.mj_model = None
        self.mj_data = None
        self.robot = None
        self.sched = None
        self.swing_planner = None
        self.torso_planner = None
        self.nmpc = None
        self.qp_ss = None
        self.qp_ext = None
        self._weld_map = {}
        self._site_ids = {}
        self.plan = None

    # ── Setup ────────────────────────────────────────────────────────────

    def setup(self, n_steps: int = 3, start_a: int = 2, start_b: int = 2):
        """Initialize all components."""
        cfg = self.cfg

        # MuJoCo
        self.mj_model = mujoco.MjModel.from_xml_path(self.mjcf_path)
        self.mj_data = mujoco.MjData(self.mj_model)
        self.mj_model.opt.timestep = cfg.dt_qp

        # Correct torso mass
        tid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_BODY, 'torso')
        ratio = cfg.torso_mass / self.mj_model.body_mass[tid]
        self.mj_model.body_mass[tid] = cfg.torso_mass
        self.mj_model.body_inertia[tid] *= ratio
        mujoco.mj_forward(self.mj_model, self.mj_data)

        mj_a, mj_b = read_anchors_from_mujoco(self.mj_model, self.mj_data)

        # Pinocchio
        self.robot = RobotInterface(
            self.urdf_path, gravity='zero', torso_mass=cfg.torso_mass)

        # Scheduler
        self.sched = ContactScheduler(
            anchors_a=mj_a, anchors_b=mj_b,
            dt_ds=cfg.t_ds, dt_ss=cfg.t_swing)
        self.plan = self.sched.plan_traversal(
            start_a=start_a, start_b=start_b, n_steps=n_steps)

        # Swing planner
        self.swing_planner = SwingPlanner(self.sched, clearance=cfg.swing_clearance)

        # Torso planner (reconfigured per step)
        self.torso_planner = TorsoPlanner()

        # Initial IK
        self.q_dock_init = dock_configuration(
            self.robot.model,
            self.sched.anchor_se3('a', start_a),
            self.sched.anchor_se3('b', start_b))

        sp = self.mj_data.qpos[0:3].copy()
        sq = self.mj_data.qpos[3:7].copy()
        mj_qpos, _ = pinocchio_to_mujoco(
            self.q_dock_init, np.zeros(18), struct_pos=sp, struct_quat=sq)
        self.mj_data.qpos[:] = mj_qpos
        self.mj_data.qvel[:] = 0.0

        # Welds
        self._build_weld_map()
        self._deactivate_all_welds()
        self._activate_weld('a', start_a)
        self._activate_weld('b', start_b)
        mujoco.mj_forward(self.mj_model, self.mj_data)

        for _ in range(cfg.n_settle_steps):
            mujoco.mj_step(self.mj_model, self.mj_data)

        # CoM calibration
        rs0 = self.robot.update(
            *mujoco_to_pinocchio(self.mj_data.qpos, self.mj_data.qvel))
        am = sum(self.robot.model.inertias[i].mass for i in range(2, 8))
        self.loco_planner = LocomotionPlanner(
            self.sched, arm_mass=am, total_mass=rs0.total_mass)
        self.loco_planner.calibrate_from_config(rs0.r_com)

        # Site IDs
        self._cache_site_ids()

        # NMPC
        self.nmpc = CentroidalNMPC(CentroidalNMPCConfig(
            robot_mass=rs0.total_mass,
            N=cfg.nmpc_N, dt=cfg.nmpc_dt,
            f_max=cfg.nmpc_f_max, tau_max=cfg.nmpc_tau_max,
            hw_min=cfg.hw_min, hw_max=cfg.hw_max,
            L_max=cfg.L_max, tau_w_max=cfg.tau_w_max))
        self.nmpc.build()

        # QP variants
        self.qp_ss = self._build_qp(
            cfg.ss_alpha_com, cfg.ss_alpha_torso, cfg.ss_alpha_ee,
            cfg.ss_alpha_posture, cfg.ss_alpha_wrench,
            cfg.ss_Kp_com, cfg.ss_Kd_com,
            cfg.ss_Kp_torso, cfg.ss_Kd_torso,
            cfg.ss_Kp_ee, cfg.ss_Kd_ee)
        self.qp_ext = self._build_qp(
            cfg.ext_alpha_com, cfg.ext_alpha_torso, cfg.ext_alpha_ee,
            cfg.ext_alpha_posture, cfg.ext_alpha_wrench,
            cfg.ext_Kp_com, cfg.ext_Kd_com,
            cfg.ext_Kp_torso, cfg.ext_Kd_torso,
            cfg.ext_Kp_ee, cfg.ext_Kd_ee)

        print(f"[SimulationLoop] Initialized:")
        print(f"  Robot mass:     {rs0.total_mass:.1f} kg (torso {cfg.torso_mass} kg)")
        print(f"  NMPC:           {1/cfg.dt_nmpc:.0f} Hz, N={cfg.nmpc_N}")
        print(f"  QP:             {1/cfg.dt_qp:.0f} Hz, {self.n_qp_per_nmpc} per NMPC")
        print(f"  Gait:           {n_steps} step(s), T_swing={cfg.t_swing}s")
        print(f"  Constraints:    L_max={cfg.L_max} Nms, tau_w={cfg.tau_w_max} Nm, "
              f"tau_joint={cfg.tau_max} Nm")
        print(f"  Dock threshold: {cfg.weld_radius*1000:.1f} mm")

    def _build_qp(self, ac, at, ae, ap, aw, kpc, kdc, kpt, kdt, kpe, kde):
        cfg = self.cfg
        c = WholeBodyQPConfig(
            nq=12, nc_max=2, dt_qp=cfg.dt_qp,
            tau_max=cfg.tau_max * np.ones(12),
            alpha_com=ac, alpha_torso=at, alpha_ee=ae,
            alpha_posture=ap, alpha_wrench=aw,
            alpha_torque=1e0, alpha_reg=1e-2,
            Kp_com=np.diag([kpc]*3), Kd_com=np.diag([kdc]*3),
            Kp_torso=np.array([kpt]*3 + [kpt*0.6]*3),
            Kd_torso=np.array([kdt]*3 + [kdt*0.6]*3),
            Kp_ee=kpe * np.ones(3), Kd_ee=kde * np.ones(3),
            Kp_posture=1.0, Kd_posture=1.5,
            L_max=cfg.L_max, tau_w_max=cfg.tau_w_max)
        qp = WholeBodyQP(c)
        qp.set_nominal_posture(self.q_dock_init[7:19])
        return qp

    # ── Weld management ──────────────────────────────────────────────────

    def _build_weld_map(self):
        self._weld_map = {}
        for i in range(self.mj_model.neq):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
            if name and name.startswith('grip_'):
                parts = name.split('_to_')
                arm = parts[0].split('_')[1]
                anchor_idx = int(parts[1][0]) - 1
                self._weld_map[(arm, anchor_idx)] = i

    def _deactivate_all_welds(self):
        for eq_id in range(self.mj_model.neq):
            self.mj_data.eq_active[eq_id] = 0

    def _activate_weld(self, arm, anchor_idx):
        key = (arm, anchor_idx)
        if key in self._weld_map:
            self.mj_data.eq_active[self._weld_map[key]] = 1

    def _deactivate_weld(self, arm, anchor_idx):
        key = (arm, anchor_idx)
        if key in self._weld_map:
            self.mj_data.eq_active[self._weld_map[key]] = 0

    def _cache_site_ids(self):
        for name in ['gripper_a', 'gripper_b']:
            sid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SITE, name)
            self._site_ids[name] = sid
        for arm in ['a', 'b']:
            for idx in range(5):
                name = f'anchor_{idx+1}{arm}'
                sid = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SITE, name)
                if sid >= 0:
                    self._site_ids[name] = sid

    def _gripper_distance(self, arm, anchor_idx):
        grip_sid = self._site_ids.get(f'gripper_{arm}', -1)
        anch_sid = self._site_ids.get(f'anchor_{anchor_idx+1}{arm}', -1)
        if grip_sid < 0 or anch_sid < 0:
            return np.inf
        return float(np.linalg.norm(
            self.mj_data.site_xpos[grip_sid] - self.mj_data.site_xpos[anch_sid]))

    def _anchor_site_pos(self, arm, anchor_idx):
        anch_sid = self._site_ids.get(f'anchor_{anchor_idx+1}{arm}', -1)
        if anch_sid < 0:
            return np.zeros(3)
        return self.mj_data.site_xpos[anch_sid].copy()

    # ── Torso planner setup per step ─────────────────────────────────────

    def _setup_torso_for_step(self, t_ss_start, t_ss_end, swing_arm,
                              stance_a, stance_b, target_arm, target_idx):
        cfg = self.cfg
        model = self.robot.model

        se3_a = self.sched.anchor_se3('a', stance_a)
        se3_b = self.sched.anchor_se3('b', stance_b)

        # IK at start
        q_start = dock_configuration(model, se3_a, se3_b)
        rs_s = self.robot.update(q_start, np.zeros(18))
        p_t0 = rs_s.oMf_torso.translation.copy()
        R_t0 = rs_s.oMf_torso.rotation.copy()
        r_com0 = rs_s.r_com.copy()
        delta0 = R_t0.T @ (r_com0 - p_t0)

        # IK at end (swing arm on new anchor)
        mj_a, mj_b = read_anchors_from_mujoco(self.mj_model, self.mj_data)
        if target_arm == 'b':
            p_tgt = mj_b[target_idx][:3]
            se3_b_end = pin.SE3(se3_b.rotation, p_tgt)
            q_end = dock_configuration(model, se3_a, se3_b_end)
        else:
            p_tgt = mj_a[target_idx][:3]
            se3_a_end = pin.SE3(se3_a.rotation, p_tgt)
            q_end = dock_configuration(model, se3_a_end, se3_b)

        rs_e = self.robot.update(q_end, np.zeros(18))
        p_t1_full = rs_e.oMf_torso.translation.copy()
        R_t1_full = rs_e.oMf_torso.rotation.copy()
        r_com1_full = rs_e.r_com.copy()
        delta1_full = R_t1_full.T @ (r_com1_full - p_t1_full)

        frac = cfg.torso_frac
        dp = p_t1_full - p_t0
        dR = R_t0.T @ R_t1_full
        omega = pin.log3(dR)
        p_t1 = p_t0 + frac * dp
        R_t1 = R_t0 @ pin.exp3(frac * omega)
        delta1 = (1 - frac) * delta0 + frac * delta1_full

        t_torso_start = t_ss_start + cfg.torso_delay * cfg.t_swing
        self.torso_planner.clear_phases()
        self.torso_planner.set_hold(p_t0, R_t0, r_com=r_com0)
        self.torso_planner.add_phase(
            t_torso_start, t_ss_end,
            p_t0, R_t0, p_t1, R_t1,
            delta_com_start=delta0, delta_com_end=delta1)

        return q_start

    # ── Run ──────────────────────────────────────────────────────────────

    def run(self, verbose=True):
        """Run full multi-step locomotion simulation."""
        cfg = self.cfg
        log = SimLog()
        plan = self.plan

        hw = cfg.hw_init.copy()
        t = 0.0
        L_com_prev = None

        # Parse phases: DS-SS pairs
        phases = plan.phases
        step_idx = 0
        i = 0
        while i < len(phases):
            gp = phases[i]
            if gp.phase.value == 'double':
                # DS phase
                t_ds_start = plan.t_start[i]
                t_ds_end = plan.t_end[i]

                # Look ahead for SS phase
                if i + 1 < len(phases) and phases[i+1].phase.value != 'double':
                    ss_gp = phases[i+1]
                    t_ss_start = plan.t_start[i+1]
                    t_ss_end = plan.t_end[i+1]

                    swing_arm = ss_gp.swing_arm
                    stance_arm = 'a' if swing_arm == 'b' else 'b'
                    stance_a = ss_gp.anchor_a_idx
                    stance_b = ss_gp.anchor_b_idx
                    target_idx = ss_gp.swing_to_idx

                    if verbose:
                        print(f"\n[Step {step_idx}] swing={swing_arm}, "
                              f"stance=({stance_a}a,{stance_b}b), "
                              f"target={target_idx}{swing_arm}")

                    # Torso planner
                    q_dock = self._setup_torso_for_step(
                        t_ss_start, t_ss_end, swing_arm,
                        stance_a, stance_b, swing_arm, target_idx)
                    self.qp_ss.set_nominal_posture(q_dock[7:19])
                    self.qp_ext.set_nominal_posture(q_dock[7:19])
                    cc_ss = self.sched.contact_config_at(t_ss_start + 0.1)

                    # DS
                    cc_ds = self.sched.contact_config_at(t_ds_start + 0.1)
                    if verbose:
                        print(f"  DS: [{t_ds_start:.2f}, {t_ds_end:.2f}]")
                    while t < t_ds_end:
                        hw, L_com_prev = self._step(
                            t, 'DS', step_idx, swing_arm, stance_arm,
                            cc_ds, target_idx, stance_a, stance_b,
                            hw, L_com_prev, log, ss_end=t_ss_end)
                        t += cfg.dt_nmpc

                    # SS: release swing arm
                    old_anchor = ss_gp.swing_from_idx
                    self._deactivate_weld(swing_arm, old_anchor)
                    if verbose:
                        print(f"  SS: [{t_ss_start:.2f}, {t_ss_end:.2f}] "
                              f"released {swing_arm}@{old_anchor}")
                    while t < t_ss_end:
                        hw, L_com_prev = self._step(
                            t, 'SS', step_idx, swing_arm, stance_arm,
                            cc_ss, target_idx, stance_a, stance_b,
                            hw, L_com_prev, log, ss_end=t_ss_end)
                        t += cfg.dt_nmpc

                    # EXT: capture torso hold
                    pq, pv = mujoco_to_pinocchio(
                        self.mj_data.qpos, self.mj_data.qvel)
                    rs_snap = self.robot.update(pq, pv)
                    self.torso_planner.set_hold(
                        rs_snap.oMf_torso.translation.copy(),
                        rs_snap.oMf_torso.rotation.copy(),
                        r_com=rs_snap.r_com.copy())

                    if verbose:
                        print(f"  EXT: {t:.2f} → dock or +{cfg.t_ext_max}s")

                    t_ext_start = t
                    docked = False
                    while t < t_ext_start + cfg.t_ext_max and not docked:
                        hw, L_com_prev = self._step(
                            t, 'EXT', step_idx, swing_arm, stance_arm,
                            cc_ss, target_idx, stance_a, stance_b,
                            hw, L_com_prev, log, ss_end=t_ss_end)
                        t += cfg.dt_nmpc

                        mujoco.mj_forward(self.mj_model, self.mj_data)
                        d = self._gripper_distance(swing_arm, target_idx)
                        if d < cfg.weld_radius:
                            docked = True
                            log.dock_events.append({
                                't': round(t, 3), 'step': step_idx,
                                'd_mm': round(d*1000, 2),
                                'arm': swing_arm, 'anchor': target_idx})
                            if verbose:
                                print(f"  *** DOCK step {step_idx}: t={t:.2f}s "
                                      f"d={d*1000:.1f}mm ***")

                    if not docked and verbose:
                        recent = log.d_grip_swing[-20:] if len(log.d_grip_swing) >= 20 else log.d_grip_swing
                        print(f"  TIMEOUT step {step_idx}: "
                              f"min d={min(recent)*1000:.1f}mm")

                    # Post-dock: activate weld
                    if docked:
                        self._activate_weld(swing_arm, target_idx)
                        mujoco.mj_forward(self.mj_model, self.mj_data)

                    step_idx += 1
                    i += 2  # skip SS phase (already processed)
                else:
                    # Trailing DS (end of gait), skip
                    i += 1
            else:
                # Standalone SS phase (shouldn't happen in normal plan)
                i += 1

        if verbose:
            self._print_summary(log)
        return log

    # ── Single NMPC+QP step ──────────────────────────────────────────────

    def _step(self, t, phase, step_idx, swing_arm, stance_arm,
              cc_ss, target_anchor, stance_a, stance_b,
              hw, L_com_prev, log, ss_end=None):
        cfg = self.cfg
        tref = self.torso_planner.reference_at(t)
        cref = self.torso_planner.com_reference_at(t)

        pq, pv = mujoco_to_pinocchio(self.mj_data.qpos, self.mj_data.qvel)
        rs = self.robot.update(pq, pv)
        if L_com_prev is None:
            L_com_prev = rs.L_com.copy()

        # NMPC
        nmpc_ok = True
        try:
            rp, vp, _, lr, info_n = self.nmpc.solve(
                r_com=rs.r_com, v_com=rs.v_com, L_com=rs.L_com,
                hw_current=hw, r_com_ref=cref.r_com, v_com_ref=cref.v_com,
                contact_config=cc_ss, warm_start=True)
            af = self.nmpc.compute_feedforward_acceleration(lr)
            nmpc_ok = info_n.success
        except Exception:
            rp, vp, lr, af = cref.r_com, cref.v_com, np.zeros(12), np.zeros(3)
            nmpc_ok = False

        # QP inner loop
        qp = self.qp_ext if phase == 'EXT' else self.qp_ss
        tau_last = np.zeros(12)
        qp_ok = True

        if ss_end is None:
            ss_end = t + cfg.dt_nmpc  # fallback

        for qs in range(self.n_qp_per_nmpc):
            tq = t + qs * cfg.dt_qp
            pq, pv = mujoco_to_pinocchio(self.mj_data.qpos, self.mj_data.qvel)
            rs = self.robot.update(pq, pv)
            Jc, Jdc = self.robot.get_contact_jacobians(
                cc_ss.active_contacts[0], cc_ss.active_contacts[1])

            tr = self.torso_planner.reference_at(tq)
            tkw = dict(
                J_torso=rs.J_torso, Jdot_dq_torso=rs.Jdot_dq_torso,
                p_torso=rs.oMf_torso.translation,
                R_torso=rs.oMf_torso.rotation,
                p_torso_ref=tr.p, R_torso_ref=tr.R,
                v_torso_ref=tr.v, a_torso_ff=tr.a)

            ek = {}
            if phase == 'SS':
                sr = self.swing_planner.reference_at(min(tq, ss_end - 0.01))
                if sr.is_swinging and sr.swing_arm == swing_arm:
                    J_ee, Jdq_ee, p_ee = self._get_ee_data(rs, swing_arm)
                    ek = dict(J_ee=J_ee, Jdot_dq_ee=Jdq_ee,
                              p_ee=p_ee, p_ee_ref=sr.p_ee,
                              v_ee_ref=sr.v_ee, a_ee_ff=sr.a_ee)
            elif phase == 'EXT':
                mujoco.mj_forward(self.mj_model, self.mj_data)
                p_tgt = self._anchor_site_pos(swing_arm, target_anchor)
                J_ee, Jdq_ee, p_ee = self._get_ee_data(rs, swing_arm)
                ek = dict(J_ee=J_ee, Jdot_dq_ee=Jdq_ee,
                          p_ee=p_ee, p_ee_ref=p_tgt,
                          v_ee_ref=np.zeros(3), a_ee_ff=np.zeros(3))

            try:
                _, _, _, tau, _ = qp.solve(
                    q_t=rs.q_torso, dq_t=rs.dq_torso,
                    q=rs.q_joints, dq=rs.dq_joints,
                    r_com_ref=rp, v_com_ref=vp,
                    lambda_ref=lr, a_com_ff=af,
                    H_robot=rs.H, C_robot=rs.C,
                    J_com=rs.J_com, Jdot_dq_com=rs.Jdot_dq_com,
                    contact_config=cc_ss, J_contacts=Jc, Jdot_dq_contacts=Jdc,
                    hw_current=hw, hw_min=cfg.hw_min, hw_max=cfg.hw_max,
                    r_com=rs.r_com, L_com_current=rs.L_com,
                    **tkw, **ek)
            except Exception:
                tau = np.zeros(12)
                qp_ok = False

            tau = np.clip(tau, -cfg.tau_max, cfg.tau_max)
            tau_last = tau.copy()
            self.mj_data.ctrl[:12] = tau
            mujoco.mj_step(self.mj_model, self.mj_data)

            rs2 = self.robot.update(
                *mujoco_to_pinocchio(self.mj_data.qpos, self.mj_data.qvel))
            hw -= (rs2.L_com - rs.L_com) / cfg.dt_qp * cfg.dt_qp
            hw = np.clip(hw, cfg.hw_min, cfg.hw_max)

        # Logging
        mujoco.mj_forward(self.mj_model, self.mj_data)
        rs_f = self.robot.update(
            *mujoco_to_pinocchio(self.mj_data.qpos, self.mj_data.qvel))
        d_swing = self._gripper_distance(swing_arm, target_anchor)
        d_stance = self._gripper_distance(
            stance_arm, stance_a if stance_arm == 'a' else stance_b)
        L_dot_est = (rs_f.L_com - L_com_prev) / cfg.dt_nmpc
        sq = self.mj_data.qpos[3:7].copy()
        euler = quat_wxyz_to_euler_deg(sq[0], sq[1], sq[2], sq[3])

        log.t.append(t)
        log.phase.append(phase)
        log.step_idx.append(step_idx)
        log.p_torso.append(rs_f.oMf_torso.translation.copy())
        log.p_torso_ref.append(tref.p.copy())
        log.e_torso_pos.append(float(np.linalg.norm(
            rs_f.oMf_torso.translation - tref.p)))
        log.d_grip_swing.append(d_swing)
        log.d_grip_stance.append(d_stance)
        log.swing_arm.append(swing_arm)
        log.r_com.append(rs_f.r_com.copy())
        log.r_com_ref.append(cref.r_com.copy())
        log.e_com.append(float(np.linalg.norm(rs_f.r_com - cref.r_com)))
        log.L_com.append(rs_f.L_com.copy())
        log.L_com_norm.append(float(np.linalg.norm(rs_f.L_com)))
        log.L_dot.append(L_dot_est.copy())
        log.L_dot_norm.append(float(np.linalg.norm(L_dot_est)))
        log.hw.append(hw.copy())
        log.tau.append(tau_last.copy())
        log.tau_max_joint.append(float(np.max(np.abs(tau_last))))
        log.struct_pos.append(self.mj_data.qpos[0:3].copy())
        log.struct_quat.append(sq)
        log.struct_euler_deg.append(euler)
        log.nmpc_ok.append(nmpc_ok)
        log.qp_ok.append(qp_ok)
        log.lambda_ref_norm.append(float(np.linalg.norm(lr)))

        return hw, rs_f.L_com.copy()

    def _get_ee_data(self, rs, arm):
        if arm == 'b':
            return rs.J_tool_b, rs.Jdot_dq_tool_b, rs.oMf_tool_b.translation
        else:
            return rs.J_tool_a, rs.Jdot_dq_tool_a, rs.oMf_tool_a.translation

    # ── Summary ──────────────────────────────────────────────────────────

    def _print_summary(self, log):
        t = np.array(log.t)
        Ln = np.array(log.L_com_norm)
        Ldn = np.array(log.L_dot_norm)
        euler = np.array(log.struct_euler_deg)
        sp = np.array(log.struct_pos)

        print(f"\n{'='*60}")
        print(f"SIMULATION SUMMARY")
        print(f"{'='*60}")
        print(f"Duration:        {t[-1]:.1f}s")
        print(f"Dock events:     {len(log.dock_events)}")
        for ev in log.dock_events:
            print(f"  Step {ev['step']}: t={ev['t']}s d={ev['d_mm']}mm arm={ev['arm']}")
        print(f"max |tau_joint|:  {max(log.tau_max_joint):.2f} Nm")
        print(f"max ||L_com||:    {Ln.max():.2f} Nms (lim {self.cfg.L_max})")
        print(f"max ||L̇_com||:    {Ldn.max():.2f} Nm (lim {self.cfg.tau_w_max})")
        print(f"Struct drift:     {np.linalg.norm(sp[-1]-sp[0])*100:.1f} cm")
        print(f"Struct rotation:  roll={euler[-1,0]:.2f}° "
              f"pitch={euler[-1,1]:.2f}° yaw={euler[-1,2]:.2f}°")
        print(f"Max |angle|:      {np.max(np.abs(euler)):.2f}°")
        nf_nmpc = sum(1 for x in log.nmpc_ok if not x)
        nf_qp = sum(1 for x in log.qp_ok if not x)
        print(f"NMPC fails:       {nf_nmpc}/{len(log.nmpc_ok)}")
        print(f"QP fails:         {nf_qp}/{len(log.qp_ok)}")

    # ── Plotting ─────────────────────────────────────────────────────────

    @staticmethod
    def plot(log, save_path=None, cfg=None):
        """Generate 7-panel diagnostic plot."""
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        t = np.array(log.t)
        pt = np.array(log.p_torso)
        pt_ref = np.array(log.p_torso_ref)
        d = np.array(log.d_grip_swing)
        tau = np.array(log.tau)
        ecom = np.array(log.e_com)
        rcom = np.array(log.r_com)
        rcom_ref = np.array(log.r_com_ref)
        Lcom = np.array(log.L_com)
        Lnorm = np.array(log.L_com_norm)
        sp = np.array(log.struct_pos)
        euler = np.array(log.struct_euler_deg)
        ph = log.phase

        L_max = cfg.L_max if cfg else 5.0
        tw = cfg.tau_w_max if cfg else 2.0
        wr = cfg.weld_radius if cfg else 0.005
        tm = cfg.tau_max if cfg else 10.0

        def shade(ax):
            for i in range(len(t)):
                if ph[i] == 'DS':
                    ax.axvspan(t[i]-.04, t[i]+.04, alpha=.08, color='blue')
                elif ph[i] == 'EXT':
                    ax.axvspan(t[i]-.04, t[i]+.04, alpha=.08, color='red')
            for i in range(1, len(ph)):
                if ph[i] != ph[i-1]:
                    ax.axvline(t[i], color='gray', ls=':', alpha=.5)

        fig, axes = plt.subplots(7, 1, figsize=(14, 28), sharex=True)
        nd = len(log.dock_events)
        fig.suptitle(
            f'VISPA — $L_{{max}}$={L_max}, $\\tau_w$={tw} Nm, '
            f'$\\tau_j$={tm} Nm | {nd} dock(s)',
            fontsize=14, fontweight='bold')

        ax = axes[0]; shade(ax)
        ax.semilogy(t, d*100, 'r-', lw=2.5, label='||grip−anchor||')
        ax.axhline(wr*100, color='g', ls='--', lw=2, label=f'seuil {wr*1000:.0f}mm')
        for ev in log.dock_events:
            ax.axvline(ev['t'], color='green', ls='-', lw=2, alpha=.4)
        ax.set_ylabel('Distance [cm] (log)'); ax.set_title('① Distance EE → ancre')
        ax.legend(fontsize=9); ax.grid(True, alpha=.3, which='both'); ax.set_ylim([0.1, 200])

        ax = axes[1]; shade(ax)
        ax.plot(t, pt[:,0]*100, 'r-', lw=2.5, label='torse x')
        ax.plot(t, pt_ref[:,0]*100, 'r--', lw=1.5, alpha=.5, label='ref')
        ax.set_ylabel('[cm]'); ax.set_title('② Avancement torse')
        ax.legend(fontsize=9); ax.grid(True, alpha=.3)

        ax = axes[2]; shade(ax)
        ax.plot(t, rcom[:,0]*100, 'r-', lw=2, label='CoM x')
        ax.plot(t, rcom_ref[:,0]*100, 'r--', lw=1.5, alpha=.6, label='ref')
        ax.plot(t, ecom*100, 'k-', lw=2, label='||e_com||')
        ax.set_ylabel('[cm]'); ax.set_title('③ Suivi CoM')
        ax.legend(fontsize=9); ax.grid(True, alpha=.3)

        ax = axes[3]; shade(ax)
        ax.plot(t, Lcom[:,0], 'r-', lw=1.5, alpha=.7, label='$L_x$')
        ax.plot(t, Lcom[:,1], 'g-', lw=1.5, alpha=.7, label='$L_y$')
        ax.plot(t, Lcom[:,2], 'b-', lw=1.5, alpha=.7, label='$L_z$')
        ax.plot(t, Lnorm, 'k-', lw=2.5, label='$||L||$')
        ax.axhline(L_max, color='r', ls='--', lw=2); ax.axhline(-L_max, color='r', ls='--', lw=2)
        ax.fill_between(t, -L_max, L_max, alpha=.05, color='green')
        ax.set_ylabel('[Nms]'); ax.set_title('④ Moment cinétique robot')
        ax.legend(fontsize=9, ncol=3); ax.grid(True, alpha=.3)

        ax = axes[4]; shade(ax)
        for j in range(6): ax.plot(t, tau[:,j], '-', color='C0', alpha=.3, lw=1)
        for j in range(6,12): ax.plot(t, tau[:,j], '-', color='C1', alpha=.3, lw=1)
        ax.plot(t, np.max(np.abs(tau),axis=1), 'k-', lw=2, label='max |τ|')
        ax.axhline(tm, color='r', ls='--', lw=1.5); ax.axhline(-tm, color='r', ls='--', lw=1.5)
        ax.set_ylabel('[Nm]'); ax.set_title('⑤ Couples articulaires')
        ax.legend(fontsize=9); ax.grid(True, alpha=.3)

        ax = axes[5]; shade(ax)
        sd = np.linalg.norm(sp - sp[0], axis=1) * 100
        ax.plot(t, sd, 'k-', lw=2)
        ax.set_ylabel('[cm]'); ax.set_title('⑥ Dérive structure (translation)')
        ax.grid(True, alpha=.3)

        ax = axes[6]; shade(ax)
        ax.plot(t, euler[:,0], 'r-', lw=1.5, label='roll')
        ax.plot(t, euler[:,1], 'g-', lw=1.5, label='pitch')
        ax.plot(t, euler[:,2], 'b-', lw=1.5, label='yaw')
        ax.plot(t, np.max(np.abs(euler), axis=1), 'k-', lw=2, label='max |angle|')
        ax.set_ylabel('[deg]'); ax.set_xlabel('Temps [s]')
        ax.set_title('⑦ Orientation structure (Euler)')
        ax.legend(fontsize=9); ax.grid(True, alpha=.3)

        plt.tight_layout()
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
        return fig
