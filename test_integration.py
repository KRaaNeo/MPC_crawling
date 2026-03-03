#!/usr/bin/env python3
"""
VISPA Crawling Controller — Integration Test Suite
====================================================

Validates every module in isolation (unit-level) then runs a short
closed-loop MuJoCo simulation (integration-level).

Usage:
    python test_integration.py VISPA_crawling_fixed.urdf VISPA_crawling.xml

Exit code 0 = all tests pass.
"""
import sys, os, time, argparse, traceback
import numpy as np

np.set_printoptions(precision=4, suppress=True, linewidth=120)

# ── Helpers ──────────────────────────────────────────────────────────
PASS = '\033[92m PASS \033[0m'
FAIL = '\033[91m FAIL \033[0m'
n_pass = n_fail = 0


def check(name, condition, detail=''):
    global n_pass, n_fail
    if condition:
        n_pass += 1
        print(f'  [{PASS}] {name}')
    else:
        n_fail += 1
        print(f'  [{FAIL}] {name}  {detail}')


# ── Parse args ───────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('urdf', help='Path to VISPA URDF')
parser.add_argument('mjcf', help='Path to VISPA MJCF (.xml)')
args = parser.parse_args()

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# ═══════════════════════════════════════════════════════════════════
#  TEST 1: RobotInterface
# ═══════════════════════════════════════════════════════════════════
print('\n' + '='*60)
print('  TEST 1: RobotInterface (Pinocchio wrapper)')
print('='*60)

from robot_interface import RobotInterface

robot = RobotInterface(args.urdf, gravity='zero')
check('model loads', robot.model.nq == 19 and robot.model.nv == 18,
      f'nq={robot.model.nq}, nv={robot.model.nv}')

q0 = np.zeros(19); q0[6] = 1.0  # identity quaternion
v0 = np.zeros(18)
rs = robot.update(q0, v0)

check('mass > 0', rs.total_mass > 30,
      f'mass={rs.total_mass:.2f}')
check('H shape', rs.H.shape == (18, 18))
check('H symmetric', np.allclose(rs.H, rs.H.T, atol=1e-10))
check('H positive definite', np.all(np.linalg.eigvalsh(rs.H) > 0))
check('C shape', rs.C.shape == (18,))
check('C=0 at v=0 (microgravity)', np.linalg.norm(rs.C) < 1e-10,
      f'||C||={np.linalg.norm(rs.C):.2e}')
check('J_com shape', rs.J_com.shape == (3, 18))
check('J_com rank 3', np.linalg.matrix_rank(rs.J_com) == 3)

J_c, Jdot_c = robot.get_contact_jacobians(True, True)
check('J_contacts shape', J_c.shape == (12, 18))
# Note: rank test at q=0 may be singular; tested at docked config below

# ═══════════════════════════════════════════════════════════════════
#  TEST 2: IK + Docking
# ═══════════════════════════════════════════════════════════════════
print('\n' + '='*60)
print('  TEST 2: Inverse Kinematics')
print('='*60)

import mujoco
from contact_scheduler import ContactScheduler, read_anchors_from_mujoco
from ik import dock_configuration

mj_model = mujoco.MjModel.from_xml_path(args.mjcf)
mj_data = mujoco.MjData(mj_model)
mujoco.mj_forward(mj_model, mj_data)
mj_a, mj_b = read_anchors_from_mujoco(mj_model, mj_data)

check('6 anchors per arm', len(mj_a) == 6 and len(mj_b) == 6)

sched = ContactScheduler(anchors_a=mj_a, anchors_b=mj_b)
anchor_a = sched.anchor_se3('a', 2)
anchor_b = sched.anchor_se3('b', 2)
q_dock = dock_configuration(robot.model, anchor_a, anchor_b)

check('q_dock shape', q_dock.shape == (19,))
check('joints finite', np.all(np.isfinite(q_dock[7:])),
      f'max |q|={np.max(np.abs(q_dock[7:])):.2f}')

rs_dock = robot.update(q_dock, np.zeros(18))
err_a = np.linalg.norm(rs_dock.oMf_tool_a.translation - mj_a[2])
err_b = np.linalg.norm(rs_dock.oMf_tool_b.translation - mj_b[2])
check(f'tool_a error < 1e-6', err_a < 1e-6, f'{err_a:.2e}')
check(f'tool_b error < 1e-6', err_b < 1e-6, f'{err_b:.2e}')
check(f'CoM below structure', rs_dock.r_com[2] < mj_a[2][2],
      f'CoM_z={rs_dock.r_com[2]:.3f} vs anchor_z={mj_a[2][2]:.3f}')

J_c_dock, _ = robot.get_contact_jacobians(True, True)
check('J_contacts rank 12 (docked)',
      np.linalg.matrix_rank(J_c_dock, tol=1e-6) == 12,
      f'rank={np.linalg.matrix_rank(J_c_dock, tol=1e-6)}')

# ═══════════════════════════════════════════════════════════════════
#  TEST 3: Contact Scheduler + Gait Plan
# ═══════════════════════════════════════════════════════════════════
print('\n' + '='*60)
print('  TEST 3: Contact Scheduler & Gait Plan')
print('='*60)

plan = sched.plan_traversal(start_a=2, start_b=2, n_steps=4)
check('9 phases', len(plan.phases) == 9)
check('starts with DS', plan.phases[0].phase.value == 'double')
check('ends with DS', plan.phases[-1].phase.value == 'double')

# Check swing info
ss_phases = [p for p in plan.phases if p.swing_arm]
check('4 swing phases', len(ss_phases) == 4)
for i, sp in enumerate(ss_phases):
    dist = np.linalg.norm(
        (mj_a if sp.swing_arm == 'a' else mj_b)[sp.swing_to_idx] -
        (mj_a if sp.swing_arm == 'a' else mj_b)[sp.swing_from_idx])
    check(f'swing {i}: d={dist:.2f}m > 0', dist > 0.5)

# Contact config queries
from solvers.contact_phase import ContactPhase
cc_ds = sched.contact_config_at(0.1)
cc_ss = sched.contact_config_at(plan.t_start[1] + 0.1)
check('DS: nc=2', cc_ds.nc == 2)
check('SS: nc=1', cc_ss.nc == 1)

# ═══════════════════════════════════════════════════════════════════
#  TEST 4: Swing Planner
# ═══════════════════════════════════════════════════════════════════
print('\n' + '='*60)
print('  TEST 4: Swing Planner')
print('='*60)

from swing_planner import SwingPlanner

swing = SwingPlanner(sched, clearance=0.08)

# Phase 1 is first SS (arm B swings)
_, p_traj, v_traj = swing.swing_trajectory(1, dt=0.01)
check('v(0) ≈ 0', np.linalg.norm(v_traj[0]) < 1e-6,
      f'||v(0)||={np.linalg.norm(v_traj[0]):.2e}')
check('v(T) ≈ 0', np.linalg.norm(v_traj[-1]) < 1e-6,
      f'||v(T)||={np.linalg.norm(v_traj[-1]):.2e}')
check('p(0) = anchor_from', np.linalg.norm(p_traj[0] - mj_b[2]) < 1e-6)
check('p(T) = anchor_to', np.linalg.norm(p_traj[-1] - mj_b[3]) < 1e-6)

clearance = (p_traj[:, 2] - mj_b[2][2]).min()
check(f'clearance ≈ -0.08m', abs(clearance + 0.08) < 0.01,
      f'clearance={clearance:.4f}')

# DS query
ref_ds = swing.reference_at(0.1)
check('DS: not swinging', not ref_ds.is_swinging)

# ═══════════════════════════════════════════════════════════════════
#  TEST 5: Locomotion Planner
# ═══════════════════════════════════════════════════════════════════
print('\n' + '='*60)
print('  TEST 5: Locomotion Planner (CoM trajectory)')
print('='*60)

from locomotion_planner import LocomotionPlanner

lp = LocomotionPlanner(sched)
lp.calibrate_from_config(rs_dock.r_com)

r0, v0 = lp.reference_at(0.0)
rT, vT = lp.reference_at(plan.total_duration - 0.01)
check('r_ref shape', r0.shape == (3,))
check('v_ref shape', v0.shape == (3,))
check('CoM moves forward', rT[0] > r0[0],
      f'x: {r0[0]:.3f} → {rT[0]:.3f}')

# ═══════════════════════════════════════════════════════════════════
#  TEST 6: State Conversion
# ═══════════════════════════════════════════════════════════════════
print('\n' + '='*60)
print('  TEST 6: Pinocchio ↔ MuJoCo State Conversion')
print('='*60)

from simulation_loop import pinocchio_to_mujoco, mujoco_to_pinocchio

struct_pos = mj_data.qpos[0:3].copy()
struct_quat = mj_data.qpos[3:7].copy()
mj_qpos, mj_qvel = pinocchio_to_mujoco(
    q_dock, np.zeros(18), struct_pos=struct_pos, struct_quat=struct_quat)
q_back, v_back = mujoco_to_pinocchio(mj_qpos, mj_qvel)

check('roundtrip q', np.allclose(q_back, q_dock, atol=1e-12),
      f'||Δq||={np.linalg.norm(q_back - q_dock):.2e}')
check('roundtrip v', np.allclose(v_back, np.zeros(18), atol=1e-12))

# ═══════════════════════════════════════════════════════════════════
#  TEST 7: Centroidal NMPC
# ═══════════════════════════════════════════════════════════════════
print('\n' + '='*60)
print('  TEST 7: Centroidal NMPC (CasADi/IPOPT)')
print('='*60)

from solvers.centroidal_nmpc import CentroidalNMPC, CentroidalNMPCConfig

nmpc_cfg = CentroidalNMPCConfig(
    robot_mass=rs_dock.total_mass, N=20, dt=0.05,
    f_max=30.0, tau_max=5.0,
    hw_min=np.full(3, -22), hw_max=np.full(3, 18))
nmpc = CentroidalNMPC(nmpc_cfg)
nmpc.build()

cc = sched.contact_config_at(0.0)
r_ref, v_ref = lp.reference_at(0.0)
hw = np.zeros(3)

t0 = time.perf_counter()
r_plan, v_plan, L_plan, lam_ref, info = nmpc.solve(
    r_com=rs_dock.r_com, v_com=np.zeros(3), L_com=np.zeros(3),
    hw_current=hw, r_com_ref=r_ref, v_com_ref=v_ref,
    contact_config=cc, warm_start=False)
cold_ms = (time.perf_counter() - t0) * 1000

check('NMPC success', info.success)
check(f'cold start < 500ms', cold_ms < 500, f'{cold_ms:.0f}ms')
check('r_plan shape', r_plan.shape == (3,))
check('lam_ref shape', lam_ref.shape == (12,))

# Warm start
t0 = time.perf_counter()
_, _, _, _, info2 = nmpc.solve(
    r_com=rs_dock.r_com, v_com=np.zeros(3), L_com=np.zeros(3),
    hw_current=hw, r_com_ref=r_ref, v_com_ref=v_ref,
    contact_config=cc, warm_start=True)
warm_ms = (time.perf_counter() - t0) * 1000

check(f'warm start < 50ms', warm_ms < 50, f'{warm_ms:.0f}ms')
check('warm start success', info2.success)

# ═══════════════════════════════════════════════════════════════════
#  TEST 8: WholeBody QP
# ═══════════════════════════════════════════════════════════════════
print('\n' + '='*60)
print('  TEST 8: WholeBody QP (qpOASES)')
print('='*60)

from solvers.wholebody_qp import WholeBodyQP, WholeBodyQPConfig

qp_cfg = WholeBodyQPConfig(
    nq=12, nc_max=2, dt_qp=0.008,
    tau_max=10.0 * np.ones(12),
    alpha_com=1e3, alpha_ee=5e2, alpha_posture=1e2,
    alpha_wrench=1e1, alpha_torque=1e0, alpha_reg=1e-2,
    Kp_com=np.diag([50., 50., 100.]),
    Kd_com=np.diag([10., 10., 20.]),
    Kp_ee=80. * np.ones(3), Kd_ee=15. * np.ones(3),
    Kp_posture=20., Kd_posture=5.)
wbqp = WholeBodyQP(qp_cfg)
wbqp.set_nominal_posture(q_dock[7:19])

a_ff = nmpc.compute_feedforward_acceleration(lam_ref)
J_c, Jdot_c = robot.get_contact_jacobians(True, True)

t0 = time.perf_counter()
qdd_t, qdd, lam_opt, tau_q, qp_info = wbqp.solve(
    q_t=rs_dock.q_torso, dq_t=rs_dock.dq_torso,
    q=rs_dock.q_joints, dq=rs_dock.dq_joints,
    r_com_ref=r_plan, v_com_ref=v_plan,
    lambda_ref=lam_ref, a_com_ff=a_ff,
    H_robot=rs_dock.H, C_robot=rs_dock.C,
    J_com=rs_dock.J_com, Jdot_dq_com=rs_dock.Jdot_dq_com,
    contact_config=cc, J_contacts=J_c, Jdot_dq_contacts=Jdot_c,
    hw_current=hw, hw_min=np.full(3, -22), hw_max=np.full(3, 18),
    r_com=rs_dock.r_com)
qp_ms = (time.perf_counter() - t0) * 1000

check('QP success', qp_info.success)
check(f'QP time < 50ms', qp_ms < 50, f'{qp_ms:.0f}ms')

# Dynamics residual
qdd_robot = np.concatenate([qdd_t, qdd])
B_u = np.zeros((18, 12)); B_u[6:, :] = np.eye(12)
lhs = rs_dock.H @ qdd_robot + rs_dock.C
rhs = B_u @ tau_q + J_c.T @ lam_opt
dyn_res = np.linalg.norm(lhs - rhs)
check(f'dynamics residual < 1e-8', dyn_res < 1e-8, f'{dyn_res:.2e}')

# Contact acceleration
a_contact = J_c @ qdd_robot + Jdot_c
contact_res = np.linalg.norm(a_contact)
check(f'contact accel ≈ 0', contact_res < 1e-8, f'{contact_res:.2e}')

# QP with EE task (single support)
cc_ss = sched.contact_config_at(plan.t_start[1] + 0.3)
s_ref = swing.reference_at(plan.t_start[1] + 0.3)
J_c_ss, Jdot_c_ss = robot.get_contact_jacobians(
    cc_ss.active_contacts[0], cc_ss.active_contacts[1])

qdd_t2, qdd2, lam2, tau2, qi2 = wbqp.solve(
    q_t=rs_dock.q_torso, dq_t=rs_dock.dq_torso,
    q=rs_dock.q_joints, dq=rs_dock.dq_joints,
    r_com_ref=r_plan, v_com_ref=v_plan,
    lambda_ref=np.zeros(12), a_com_ff=np.zeros(3),
    H_robot=rs_dock.H, C_robot=rs_dock.C,
    J_com=rs_dock.J_com, Jdot_dq_com=rs_dock.Jdot_dq_com,
    contact_config=cc_ss, J_contacts=J_c_ss, Jdot_dq_contacts=Jdot_c_ss,
    hw_current=hw, hw_min=np.full(3, -22), hw_max=np.full(3, 18),
    r_com=rs_dock.r_com,
    J_ee=rs_dock.J_tool_b, Jdot_dq_ee=rs_dock.Jdot_dq_tool_b,
    p_ee=rs_dock.oMf_tool_b.translation,
    p_ee_ref=s_ref.p_ee, v_ee_ref=s_ref.v_ee)
check('QP + EE task success', qi2.success)

# ═══════════════════════════════════════════════════════════════════
#  TEST 9: MuJoCo Weld Management
# ═══════════════════════════════════════════════════════════════════
print('\n' + '='*60)
print('  TEST 9: MuJoCo Weld Management')
print('='*60)

# Set MuJoCo to IK config
mj_model.opt.timestep = 0.008
mj_qpos, _ = pinocchio_to_mujoco(
    q_dock, np.zeros(18), struct_pos=struct_pos, struct_quat=struct_quat)
mj_data.qpos[:] = mj_qpos
mj_data.qvel[:] = 0

# Build weld map
weld_map = {}
for i in range(mj_model.neq):
    name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
    if name and 'grip_a_to_' in name:
        weld_map[('a', int(name.split('_to_')[1].replace('a', '')) - 1)] = i
    elif name and 'grip_b_to_' in name:
        weld_map[('b', int(name.split('_to_')[1].replace('b', '')) - 1)] = i

check('12 welds found', len(weld_map) == 12, f'found {len(weld_map)}')

# Activate initial welds
for eq_id in weld_map.values():
    mj_data.eq_active[eq_id] = 0
mj_data.eq_active[weld_map[('a', 2)]] = 1
mj_data.eq_active[weld_map[('b', 2)]] = 1
mujoco.mj_forward(mj_model, mj_data)

grip_a = mj_data.site_xpos[
    mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_a')]
anch_3a = mj_data.site_xpos[
    mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, 'anchor_3a')]
check('gripper_a ≈ anchor_3a',
      np.linalg.norm(grip_a - anch_3a) < 0.001,
      f'dist={np.linalg.norm(grip_a - anch_3a):.6f}')

# Step and check weld holds
for _ in range(100):
    mujoco.mj_step(mj_model, mj_data)
mujoco.mj_forward(mj_model, mj_data)
grip_a_2 = mj_data.site_xpos[
    mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_a')]
check('weld holds after 100 steps',
      np.linalg.norm(grip_a_2 - anch_3a) < 0.005,
      f'dist={np.linalg.norm(grip_a_2 - anch_3a):.6f}')

# ═══════════════════════════════════════════════════════════════════
#  TEST 10: Short Closed-Loop (1s, double-support regulation)
# ═══════════════════════════════════════════════════════════════════
print('\n' + '='*60)
print('  TEST 10: Closed-Loop Regulation (1s, DS)')
print('='*60)

# Reset MuJoCo
mj_data.qpos[:] = mj_qpos
mj_data.qvel[:] = 0
mj_data.eq_active[weld_map[('a', 2)]] = 1
mj_data.eq_active[weld_map[('b', 2)]] = 1
mujoco.mj_forward(mj_model, mj_data)
for _ in range(50):
    mujoco.mj_step(mj_model, mj_data)

pin_q, pin_v = mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel)
rs = robot.update(pin_q, pin_v)
lp.calibrate_from_config(rs.r_com)
wbqp.set_nominal_posture(rs.q_joints)

dt_nmpc, dt_qp = 0.05, 0.008
n_qp = int(round(dt_nmpc / dt_qp))
hw_cl = np.zeros(3)
hw_min_cl, hw_max_cl = np.full(3, -22), np.full(3, 18)
n_steps = 20  # 1 second
nmpc_ok_cl = qp_ok_cl = 0
t_cl = 0.0

for step in range(n_steps):
    pin_q, pin_v = mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel)
    rs = robot.update(pin_q, pin_v)
    cc = sched.contact_config_at(0.0)  # stay in DS
    r_ref, v_ref = lp.reference_at(0.0)

    try:
        r_plan, v_plan, _, lam_ref, info_n = nmpc.solve(
            r_com=rs.r_com, v_com=rs.v_com, L_com=rs.L_com,
            hw_current=hw_cl, r_com_ref=r_ref, v_com_ref=v_ref,
            contact_config=cc, warm_start=(step > 0))
        a_ff = nmpc.compute_feedforward_acceleration(lam_ref)
        if info_n.success:
            nmpc_ok_cl += 1
    except:
        r_plan, v_plan, lam_ref, a_ff = r_ref, v_ref, np.zeros(12), np.zeros(3)

    for qs in range(n_qp):
        pin_q, pin_v = mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel)
        rs = robot.update(pin_q, pin_v)
        J_c, Jdot_c = robot.get_contact_jacobians(True, True)

        try:
            _, _, _, tau_q, qi = wbqp.solve(
                q_t=rs.q_torso, dq_t=rs.dq_torso,
                q=rs.q_joints, dq=rs.dq_joints,
                r_com_ref=r_plan, v_com_ref=v_plan,
                lambda_ref=lam_ref, a_com_ff=a_ff,
                H_robot=rs.H, C_robot=rs.C,
                J_com=rs.J_com, Jdot_dq_com=rs.Jdot_dq_com,
                contact_config=cc, J_contacts=J_c, Jdot_dq_contacts=Jdot_c,
                hw_current=hw_cl.copy(), hw_min=hw_min_cl, hw_max=hw_max_cl,
                r_com=rs.r_com)
            if qi.success:
                qp_ok_cl += 1
        except:
            tau_q = np.zeros(12)

        tau_q = np.clip(tau_q, -10, 10)
        mj_data.ctrl[:12] = tau_q
        mujoco.mj_step(mj_model, mj_data)

        rs_new = robot.update(*mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel))
        hw_cl -= (rs_new.L_com - rs.L_com)
        hw_cl = np.clip(hw_cl, hw_min_cl, hw_max_cl)

    t_cl += dt_nmpc

total_qp_cl = n_steps * n_qp
check(f'NMPC: {nmpc_ok_cl}/{n_steps} success',
      nmpc_ok_cl >= n_steps * 0.8)
check(f'QP: {qp_ok_cl}/{total_qp_cl} success',
      qp_ok_cl >= total_qp_cl * 0.9)

pin_qf, pin_vf = mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel)
rs_f = robot.update(pin_qf, pin_vf)
r_err_final = np.linalg.norm(rs_f.r_com - r_ref)
check(f'final CoM error < 0.3m', r_err_final < 0.3,
      f'{r_err_final:.4f}m')

struct_drift = np.linalg.norm(mj_data.qpos[0:3] - struct_pos)
check(f'structure drift < 50mm', struct_drift < 0.05,
      f'{struct_drift*1000:.1f}mm')

# ═══════════════════════════════════════════════════════════════════
#  SUMMARY
# ═══════════════════════════════════════════════════════════════════
print('\n' + '='*60)
total = n_pass + n_fail
print(f'  SUMMARY: {n_pass}/{total} passed, {n_fail} failed')
print('='*60)

sys.exit(0 if n_fail == 0 else 1)
