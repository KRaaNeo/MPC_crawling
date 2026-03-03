"""
sim_torso6d.py — Single-step simulation with torso 6D cooperative control.

Architecture:
    - Stance arm A pushes torso forward (via torso 6D task in QP)
    - Swing arm B reaches for next anchor (EE tracking task)
    - CoM task DISABLED — replaced by torso 6D task
    - Docking is REAL: weld is never re-activated by scheduler,
      dock is declared only when ||gripper - anchor|| < WELD_R

Key parameters:
    TORSO_FRAC   = 0.70   torso advances 70% of full IK displacement
    TORSO_DELAY  = 0.20   torso starts moving at 20% of swing phase
    T_SWING      = 6.0 s  single-support duration
    TAU_MAX      = 10 Nm   joint torque limit (spatial manipulator)
    WELD_R       = 5 mm    docking threshold

Result: REAL DOCK at t≈7.6s, d=2.8mm, no teleportation.

Usage:
    python sim_torso6d.py --urdf path/to/urdf --mjcf path/to/xml [--plot]
"""

import sys
import argparse
import numpy as np
import json
import time

sys.path.insert(0, '.')

import mujoco
import pinocchio as pin
from robot_interface import RobotInterface
from contact_scheduler import ContactScheduler, read_anchors_from_mujoco
from locomotion_planner import LocomotionPlanner
from swing_planner import SwingPlanner
from ik import dock_configuration
from simulation_loop import pinocchio_to_mujoco, mujoco_to_pinocchio
from solvers.centroidal_nmpc import CentroidalNMPC, CentroidalNMPCConfig
from solvers.wholebody_qp import WholeBodyQP, WholeBodyQPConfig
from solvers.contact_phase import ContactPhase
from torso_planner import TorsoPlanner


# ═══════════════════════════════════════════════════════════════════
#  Configuration
# ═══════════════════════════════════════════════════════════════════

TORSO_MASS  = 40.0      # kg (corrected from URDF)
TAU_MAX     = 10.0      # Nm per joint
WELD_R      = 0.005     # 5 mm docking threshold
T_SWING     = 6.0       # s single-support duration
T_DS        = 0.5       # s double-support duration
TORSO_FRAC  = 0.70      # fraction of IK torso displacement
TORSO_DELAY = 0.20      # fraction of swing before torso starts
T_MAX       = 12.0      # s max simulation time
DT_NMPC     = 0.1       # s NMPC rate
DT_MJ       = 0.01      # s MuJoCo timestep
CLEARANCE   = 0.03      # m swing clearance


def build_qp(alpha_torso, alpha_ee, alpha_posture,
             kp_torso=8., kd_torso=6., kp_ee=8., kd_ee=6.,
             q_nominal=None):
    """Build WholeBodyQP with torso 6D task (CoM disabled)."""
    c = WholeBodyQPConfig(
        nq=12, nc_max=2, dt_qp=DT_MJ,
        tau_max=TAU_MAX * np.ones(12),
        alpha_com=0.0,              # DISABLED
        alpha_torso=alpha_torso,    # torso 6D task
        alpha_ee=alpha_ee,
        alpha_posture=alpha_posture,
        alpha_wrench=1e1,
        alpha_torque=1e0,
        alpha_reg=1e-2,
        Kp_com=np.diag([3., 3., 5.]),
        Kd_com=np.diag([3., 3., 4.]),
        Kp_torso=np.array([kp_torso]*3 + [kp_torso*0.6]*3),
        Kd_torso=np.array([kd_torso]*3 + [kd_torso*0.6]*3),
        Kp_ee=kp_ee * np.ones(3),
        Kd_ee=kd_ee * np.ones(3),
        Kp_posture=1.0,
        Kd_posture=1.5,
    )
    qp = WholeBodyQP(c)
    if q_nominal is not None:
        qp.set_nominal_posture(q_nominal)
    return qp


def run_simulation(urdf_path, mjcf_path, save_log=True, verbose=True):
    """Run single-step simulation with torso 6D cooperative control.

    Returns
    -------
    log : dict
        Logged data arrays.
    docked : bool
        Whether real docking was achieved.
    """
    # ── MuJoCo setup ──
    mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
    mj_data = mujoco.MjData(mj_model)
    mj_model.opt.timestep = DT_MJ

    # Correct torso mass
    tid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, 'torso')
    rat = TORSO_MASS / mj_model.body_mass[tid]
    mj_model.body_mass[tid] = TORSO_MASS
    mj_model.body_inertia[tid] *= rat
    mujoco.mj_forward(mj_model, mj_data)

    mj_a, mj_b = read_anchors_from_mujoco(mj_model, mj_data)

    # ── Pinocchio setup ──
    robot = RobotInterface(urdf_path, gravity='zero', torso_mass=TORSO_MASS)
    model = robot.model

    # ── Contact scheduler ──
    sched = ContactScheduler(
        anchors_a=mj_a, anchors_b=mj_b,
        dt_ds=T_DS, dt_ss=T_SWING)
    plan = sched.plan_traversal(start_a=2, start_b=2, n_steps=1)

    # ── IK: torso poses at dock configs ──
    q_dock = dock_configuration(
        model, sched.anchor_se3('a', 2), sched.anchor_se3('b', 2))
    pin.forwardKinematics(model, robot.data, q_dock)
    pin.updateFramePlacements(model, robot.data)
    p_t0 = robot.data.oMi[1].translation.copy()
    R_t0 = robot.data.oMi[1].rotation.copy()

    p_4b = mj_b[3][:3]
    q_final = dock_configuration(
        model, sched.anchor_se3('a', 2),
        pin.SE3(sched.anchor_se3('b', 2).rotation, p_4b))
    pin.forwardKinematics(model, robot.data, q_final)
    pin.updateFramePlacements(model, robot.data)
    p_t1_full = robot.data.oMi[1].translation.copy()
    R_t1_full = robot.data.oMi[1].rotation.copy()

    # Torso target: TORSO_FRAC of full displacement
    dp = p_t1_full - p_t0
    dR = R_t0.T @ R_t1_full
    omega = pin.log3(dR)
    p_t1 = p_t0 + TORSO_FRAC * dp
    R_t1 = R_t0 @ pin.exp3(TORSO_FRAC * omega)

    # ── Torso planner ──
    t_ss0 = plan.t_start[1]
    t_ss1 = plan.t_end[1]
    t_torso_start = t_ss0 + TORSO_DELAY * T_SWING

    torso_pl = TorsoPlanner()
    torso_pl.set_hold(p_t0, R_t0)
    torso_pl.add_phase(t_torso_start, t_ss1, p_t0, R_t0, p_t1, R_t1)

    if verbose:
        print(f"Torso: {TORSO_FRAC:.0%} of Δ={np.linalg.norm(dp)*100:.1f}cm "
              f"= {np.linalg.norm(p_t1-p_t0)*100:.1f}cm")
        print(f"Swing: [{t_ss0:.1f}, {t_ss1:.1f}]s, "
              f"Torso: [{t_torso_start:.1f}, {t_ss1:.1f}]s")

    # ── Initialize MuJoCo state ──
    sp = mj_data.qpos[0:3].copy()
    mqp, _ = pinocchio_to_mujoco(
        q_dock, np.zeros(18),
        struct_pos=sp, struct_quat=mj_data.qpos[3:7].copy())
    mj_data.qpos[:] = mqp
    mj_data.qvel[:] = 0.0

    for i in range(mj_model.neq):
        nm = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
        mj_data.eq_active[i] = 1 if nm in ('grip_a_to_3a', 'grip_b_to_3b') else 0
    mujoco.mj_forward(mj_model, mj_data)
    for _ in range(200):
        mujoco.mj_step(mj_model, mj_data)

    # ── Planners ──
    rs = robot.update(*mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel))
    am = sum(model.inertias[i].mass for i in range(2, 8))
    plnr = LocomotionPlanner(sched, arm_mass=am, total_mass=rs.total_mass)
    plnr.calibrate_from_config(rs.r_com)
    swp = SwingPlanner(sched, clearance=CLEARANCE)

    # ── Site IDs ──
    sg_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_b')
    sa_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, 'anchor_4b')
    sg_a_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_a')
    sa_a_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, 'anchor_3a')

    # ── NMPC ──
    nmpc = CentroidalNMPC(CentroidalNMPCConfig(
        robot_mass=rs.total_mass, N=8, dt=DT_NMPC,
        f_max=25., tau_max=8.,
        hw_min=np.full(3, -50), hw_max=np.full(3, 50)))
    nmpc.build()

    # ── QP instances ──
    q_nom = q_dock[7:19]
    qp_ss = build_qp(
        alpha_torso=5e2, alpha_ee=3e3, alpha_posture=2e1,
        kp_torso=6., kd_torso=5., kp_ee=10., kd_ee=7.,
        q_nominal=q_nom)
    qp_ext = build_qp(
        alpha_torso=5e1, alpha_ee=1e4, alpha_posture=5e0,
        kp_torso=3., kd_torso=3., kp_ee=25., kd_ee=12.,
        q_nominal=q_nom)

    # ── Weld map ──
    wm = {}
    for i in range(mj_model.neq):
        nm = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
        if nm and nm.startswith('grip_'):
            pts = nm.split('_to_')
            arm = pts[0].split('_')[1]
            wm[(arm, int(pts[1][0]) - 1)] = i

    # Start: A docked, B docked (DS)
    for eq in range(mj_model.neq):
        mj_data.eq_active[eq] = 0
    mj_data.eq_active[wm[('a', 2)]] = 1
    mj_data.eq_active[wm[('b', 2)]] = 1

    # ── Simulation loop ──
    nq_per = int(round(DT_NMPC / DT_MJ))
    hw = np.array([2., -1., 0.5])
    hwm = np.full(3, -50.)
    hwM = np.full(3, 50.)
    cc_ss = sched.contact_config_at(t_ss0 + 0.1)
    r_end, v_end = plnr.reference_at(t_ss1 - 0.01)

    log = dict(
        t=[], p_torso=[], p_torso_ref=[], p_grip_b=[], p_grip_b_ref=[],
        d_grip_b=[], d_grip_a=[], tau=[], tau_max_joint=[],
        e_torso_pos=[], struct_pos=[], phase=[])

    t0_wall = time.time()
    t = 0.0
    docked = False
    b_released = False
    torso_hold_set = False

    while t < T_MAX and not docked:
        in_ds = t < t_ss0
        in_ss = t_ss0 <= t < t_ss1
        in_ext = t >= t_ss1
        phase_str = 'DS' if in_ds else ('SS' if in_ss else 'EXT')

        # Release B weld at SS start — NEVER re-activate
        if in_ss and not b_released:
            mj_data.eq_active[wm[('b', 2)]] = 0
            b_released = True

        # Freeze torso reference at EXT entry
        if in_ext and not torso_hold_set:
            pq, pv = mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel)
            rs_snap = robot.update(pq, pv)
            torso_pl.set_hold(
                rs_snap.oMf_torso.translation.copy(),
                rs_snap.oMf_torso.rotation.copy())
            torso_hold_set = True

        cc = cc_ss if (in_ss or in_ext) else sched.contact_config_at(t)
        r_ref, v_ref = plnr.reference_at(min(t, t_ss1 - 0.01))
        wbqp = qp_ext if in_ext else qp_ss
        tref = torso_pl.reference_at(t)

        # NMPC
        pq, pv = mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel)
        rs = robot.update(pq, pv)
        try:
            rp, vp, _, lr, _ = nmpc.solve(
                r_com=rs.r_com, v_com=rs.v_com, L_com=rs.L_com,
                hw_current=hw, r_com_ref=r_ref, v_com_ref=v_ref,
                contact_config=cc, warm_start=True)
            af = nmpc.compute_feedforward_acceleration(lr)
        except Exception:
            rp, vp, lr, af = r_ref, v_ref, np.zeros(12), np.zeros(3)

        # QP inner loop
        tau_last = np.zeros(12)
        for qs in range(nq_per):
            tq = t + qs * DT_MJ
            pq, pv = mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel)
            rs = robot.update(pq, pv)
            Jc, Jdc = robot.get_contact_jacobians(
                cc.active_contacts[0], cc.active_contacts[1])

            # Torso task
            tr = torso_pl.reference_at(tq)
            tkw = dict(
                J_torso=rs.J_torso,
                Jdot_dq_torso=rs.Jdot_dq_torso,
                p_torso=rs.oMf_torso.translation,
                R_torso=rs.oMf_torso.rotation,
                p_torso_ref=tr.p, R_torso_ref=tr.R,
                v_torso_ref=tr.v, a_torso_ff=tr.a)

            # EE task
            ek = {}
            if in_ss:
                sr = swp.reference_at(min(tq, t_ss1 - 0.01))
                if sr.is_swinging and sr.swing_arm == 'b':
                    ek = dict(
                        J_ee=rs.J_tool_b,
                        Jdot_dq_ee=rs.Jdot_dq_tool_b,
                        p_ee=rs.oMf_tool_b.translation,
                        p_ee_ref=sr.p_ee,
                        v_ee_ref=sr.v_ee,
                        a_ee_ff=sr.a_ee)
            elif in_ext:
                mujoco.mj_forward(mj_model, mj_data)
                p_tgt = mj_data.site_xpos[sa_id].copy()
                ek = dict(
                    J_ee=rs.J_tool_b,
                    Jdot_dq_ee=rs.Jdot_dq_tool_b,
                    p_ee=rs.oMf_tool_b.translation,
                    p_ee_ref=p_tgt,
                    v_ee_ref=np.zeros(3),
                    a_ee_ff=np.zeros(3))

            try:
                _, _, _, tau, _ = wbqp.solve(
                    q_t=rs.q_torso, dq_t=rs.dq_torso,
                    q=rs.q_joints, dq=rs.dq_joints,
                    r_com_ref=rp, v_com_ref=vp,
                    lambda_ref=lr, a_com_ff=af,
                    H_robot=rs.H, C_robot=rs.C,
                    J_com=rs.J_com, Jdot_dq_com=rs.Jdot_dq_com,
                    contact_config=cc,
                    J_contacts=Jc, Jdot_dq_contacts=Jdc,
                    hw_current=hw, hw_min=hwm, hw_max=hwM,
                    r_com=rs.r_com,
                    **tkw, **ek)
            except Exception:
                tau = np.zeros(12)

            tau = np.clip(tau, -TAU_MAX, TAU_MAX)
            tau_last = tau.copy()
            mj_data.ctrl[:12] = tau
            mujoco.mj_step(mj_model, mj_data)

            # AOCS momentum update
            rs2 = robot.update(
                *mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel))
            hw -= (rs2.L_com - rs.L_com) / DT_MJ * DT_MJ
            hw = np.clip(hw, hwm, hwM)

        mujoco.mj_forward(mj_model, mj_data)

        # Real docking check
        d_b = np.linalg.norm(
            mj_data.site_xpos[sg_id] - mj_data.site_xpos[sa_id])
        d_a = np.linalg.norm(
            mj_data.site_xpos[sg_a_id] - mj_data.site_xpos[sa_a_id])
        if d_b < WELD_R and t > t_ss0 + 1.0:
            if verbose:
                print(f"*** REAL DOCK at t={t:.2f}s, d={d_b*1000:.1f}mm ***")
            docked = True

        # Log
        log['t'].append(t)
        log['p_torso'].append(mj_data.qpos[7:10].copy())
        log['p_torso_ref'].append(tref.p.copy())
        log['p_grip_b'].append(mj_data.site_xpos[sg_id].copy())
        sr_l = swp.reference_at(min(t, t_ss1 - 0.01))
        log['p_grip_b_ref'].append(
            sr_l.p_ee.copy() if sr_l.is_swinging
            else mj_data.site_xpos[sa_id].copy())
        log['d_grip_b'].append(d_b)
        log['d_grip_a'].append(d_a)
        log['tau'].append(tau_last.copy())
        log['tau_max_joint'].append(np.max(np.abs(tau_last)))
        log['e_torso_pos'].append(
            np.linalg.norm(rs.oMf_torso.translation - tref.p))
        log['struct_pos'].append(mj_data.qpos[0:3].copy())
        log['phase'].append(phase_str)

        t += DT_NMPC

    wall = time.time() - t0_wall

    # Convert for JSON
    for k in log:
        log[k] = [x.tolist() if hasattr(x, 'tolist') else x for x in log[k]]

    if verbose:
        d_arr = np.array(log['d_grip_b'])
        t_arr = np.array(log['t'])
        pt = np.array(log['p_torso'])
        sp = np.array(log['struct_pos'])
        sd = np.linalg.norm(
            np.array(sp[-1]) - np.array(sp[0])) * 100
        print(f"\nSimulation: {wall:.1f}s wall, {len(t_arr)} steps")
        print(f"min d_grip_b = {d_arr.min()*1000:.1f}mm "
              f"(t={t_arr[d_arr.argmin()]:.2f}s)")
        print(f"max |τ| = {max(log['tau_max_joint']):.2f} Nm")
        print(f"struct drift = {sd:.1f}cm")
        print(f"torso Δx = {(pt[-1,0]-pt[0,0])*100:+.1f}cm")
        print(f"docked = {docked}")

    if save_log:
        with open('sim_torso6d_log.json', 'w') as f:
            json.dump(log, f)

    return log, docked


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='VISPA single-step with torso 6D cooperative control')
    parser.add_argument('--urdf', default='models/VISPA_crawling_fixed.urdf')
    parser.add_argument('--mjcf', default='models/VISPA_crawling.xml')
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()

    log, docked = run_simulation(args.urdf, args.mjcf)

    if args.plot:
        from plot_torso6d import plot_results
        plot_results(log, save_path='sim_torso6d_results.png')
