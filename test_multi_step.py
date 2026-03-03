"""
Multi-step locomotion test with cooperative torso task.
Tests 2 full steps: swing B (3b→4b), then swing A (3a→4a).
"""
import sys, time, numpy as np
sys.path.insert(0, '.')

URDF = '/mnt/user-data/uploads/VISPA_crawling_fixed.urdf'
MJCF = '/mnt/user-data/uploads/VISPA_crawling.xml'
TORSO_MASS = 40.0; TAU_MAX = 10.0; WELD_R = 0.005

import mujoco
from robot_interface import RobotInterface
from contact_scheduler import ContactScheduler, read_anchors_from_mujoco
from swing_planner import SwingPlanner
from ik import dock_configuration
from simulation_loop import pinocchio_to_mujoco, mujoco_to_pinocchio
from torso_planner import TorsePlanner
from solvers.centroidal_nmpc import CentroidalNMPC, CentroidalNMPCConfig
from solvers.wholebody_qp import WholeBodyQP, WholeBodyQPConfig
from solvers.contact_phase import ContactPhase

# === Setup ===
mj_model = mujoco.MjModel.from_xml_path(MJCF)
mj_data = mujoco.MjData(mj_model)
mj_model.opt.timestep = 0.01
tid = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_BODY, 'torso')
rat = TORSO_MASS / mj_model.body_mass[tid]
mj_model.body_mass[tid] = TORSO_MASS
mj_model.body_inertia[tid] *= rat
mujoco.mj_forward(mj_model, mj_data)

mj_a, mj_b = read_anchors_from_mujoco(mj_model, mj_data)
robot = RobotInterface(URDF, gravity='zero', torso_mass=TORSO_MASS)
sched = ContactScheduler(anchors_a=mj_a, anchors_b=mj_b, dt_ds=1.0, dt_ss=4.0)
plan = sched.plan_traversal(start_a=2, start_b=2, n_steps=2)

print("=== PLAN ===")
for i, p in enumerate(plan.phases):
    print(f"  Phase {i}: {p.phase.value:10s} [{plan.t_start[i]:.1f}, {plan.t_end[i]:.1f}]s "
          f"A@{p.anchor_a_idx} B@{p.anchor_b_idx} swing={p.swing_arm} "
          f"from={p.swing_from_idx}→{p.swing_to_idx}")

# Dock config
q_dock = dock_configuration(robot.model, sched.anchor_se3('a', 2), sched.anchor_se3('b', 2))
sp = mj_data.qpos[0:3].copy()
mqp, _ = pinocchio_to_mujoco(q_dock, np.zeros(18),
    struct_pos=sp, struct_quat=mj_data.qpos[3:7].copy())
mj_data.qpos[:] = mqp; mj_data.qvel[:] = 0.0

weld_map = {}
for i in range(mj_model.neq):
    nm = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
    if nm and nm.startswith('grip_'):
        pts = nm.split('_to_'); arm = pts[0].split('_')[1]
        weld_map[(arm, int(pts[1][0]) - 1)] = i

for eq in range(mj_model.neq):
    mj_data.eq_active[eq] = 0
mj_data.eq_active[weld_map[('a', 2)]] = 1
mj_data.eq_active[weld_map[('b', 2)]] = 1
mujoco.mj_forward(mj_model, mj_data)
for _ in range(200):
    mujoco.mj_step(mj_model, mj_data)

rs = robot.update(*mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel))

# Planners
torso_pl = TorsePlanner(sched, advance_ratio=0.5)
torso_pl.build_for_plan(plan, rs.oMf_torso)
swing_pl = SwingPlanner(sched, clearance=0.03)

print("\n=== TORSO TRAJECTORY ===")
for s in torso_pl._segments:
    dp = s['p_end'] - s['p_start']
    print(f"  {s['phase'].value:10s} [{s['t_start']:.1f},{s['t_end']:.1f}] dp={dp.round(3)} |dp|={np.linalg.norm(dp):.3f}m")

nmpc = CentroidalNMPC(CentroidalNMPCConfig(
    robot_mass=rs.total_mass, N=8, dt=0.1,
    f_max=20., tau_max=5., hw_min=np.full(3, -50), hw_max=np.full(3, 50)))
nmpc.build()

def make_qp(at, ae, ap, kpe=10., kde=7.):
    cfg = WholeBodyQPConfig(nq=12, nc_max=2, dt_qp=0.01,
        tau_max=TAU_MAX * np.ones(12), alpha_com=0., alpha_torso=at,
        alpha_ee=ae, alpha_posture=ap,
        alpha_wrench=1e1, alpha_torque=1e0, alpha_reg=1e-2,
        Kp_torso=np.array([8., 8., 8., 5., 5., 5.]),
        Kd_torso=np.array([6., 6., 6., 4., 4., 4.]),
        Kp_com=np.diag([3., 3., 5.]), Kd_com=np.diag([3., 3., 4.]),
        Kp_ee=kpe * np.ones(3), Kd_ee=kde * np.ones(3),
        Kp_posture=1., Kd_posture=1.5)
    qp = WholeBodyQP(cfg)
    qp.set_nominal_posture(q_dock[7:19])
    return qp

qp_ds = make_qp(1e3, 0, 1e2)
qp_ss = make_qp(5e2, 2e3, 2e1)
qp_fin = make_qp(1e2, 5e3, 5e0, kpe=18., kde=10.)

# === Main loop ===
dt_n, dt_q = 0.1, 0.01; nq = int(round(dt_n / dt_q))
hw = np.array([2., -1., .5]); hwm = np.full(3, -50.); hwM = np.full(3, 50.)

T_nom = plan.t_end[-1]
T_max = T_nom + 10.0  # max extension time
last_pidx = -1

# Track which swing target we're going for
current_swing_phase_idx = None  # index of the SS phase being extended

print(f"\n=== SIMULATION: T_nom={T_nom:.1f}s, T_max={T_max:.1f}s ===")
print(f"    tau_max={TAU_MAX}Nm, weld_r={WELD_R*1000:.0f}mm\n")

step_results = []  # (step_idx, t_dock, d_min)
t = 0.0

while t < T_max:
    extending = (t >= T_nom)

    if not extending:
        gp, pidx = plan.phase_at(t)
        cc = sched.contact_config_at(t)
    else:
        # Stay in last swing phase
        if current_swing_phase_idx is not None:
            gp = plan.phases[current_swing_phase_idx]
            pidx = current_swing_phase_idx
        else:
            gp = plan.phases[-1]
            pidx = len(plan.phases) - 1
        cc = sched.contact_config_at(plan.t_start[pidx] + 0.1)

    is_ss = gp.phase in (ContactPhase.SINGLE_A, ContactPhase.SINGLE_B)

    # Weld management on phase change
    if pidx != last_pidx and not extending:
        for eq_id in weld_map.values():
            mj_data.eq_active[eq_id] = 0
        gpc = plan.phases[pidx]
        if gpc.phase in (ContactPhase.SINGLE_A, ContactPhase.DOUBLE):
            k = ('a', gpc.anchor_a_idx)
            if k in weld_map:
                mj_data.eq_active[weld_map[k]] = 1
        if gpc.phase in (ContactPhase.SINGLE_B, ContactPhase.DOUBLE):
            k = ('b', gpc.anchor_b_idx)
            if k in weld_map:
                mj_data.eq_active[weld_map[k]] = 1

        if is_ss:
            current_swing_phase_idx = pidx

        aw = sum(1 for eq in weld_map.values() if mj_data.eq_active[eq])
        print(f">> Phase {pidx}: {gpc.phase.value} t={t:.2f}s welds={aw}")
        last_pidx = pidx

    wbqp = qp_fin if extending else (qp_ss if is_ss else qp_ds)

    # NMPC
    pq, pv = mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel)
    rs = robot.update(pq, pv)
    tr = torso_pl.reference_at(min(t, T_nom - 0.01)) if not extending else torso_pl.final_reference()
    com_off = rs.r_com - rs.oMf_torso.translation
    r_ref = tr.p + com_off; v_ref = tr.v_lin * 0.6
    try:
        rp, vp, _, lr, _ = nmpc.solve(r_com=rs.r_com, v_com=rs.v_com, L_com=rs.L_com,
            hw_current=hw, r_com_ref=r_ref, v_com_ref=v_ref, contact_config=cc, warm_start=True)
        af = nmpc.compute_feedforward_acceleration(lr)
    except:
        rp, vp, lr, af = r_ref, v_ref, np.zeros(12), np.zeros(3)

    # Determine swing target
    swing_arm = gp.swing_arm if is_ss else ''
    if swing_arm == 'b':
        sa_name = f'anchor_{gp.swing_to_idx + 1}b'
        sg_name = 'gripper_b'
    elif swing_arm == 'a':
        sa_name = f'anchor_{gp.swing_to_idx + 1}a'
        sg_name = 'gripper_a'
    else:
        sa_name, sg_name = None, None

    if sa_name:
        sa_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, sa_name)
        sg_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, sg_name)
        p_target = mj_data.site_xpos[sa_id].copy()

    docked_this_step = False
    for qs in range(nq):
        pq, pv = mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel)
        rs = robot.update(pq, pv)
        Jc, Jdc = robot.get_contact_jacobians(cc.active_contacts[0], cc.active_contacts[1])

        tq = t + qs * dt_q
        tr = torso_pl.reference_at(min(tq, T_nom - 0.01)) if not extending else torso_pl.final_reference()
        torso_kw = dict(J_torso=rs.J_torso, Jdot_dq_torso=rs.Jdot_dq_torso,
            p_torso=rs.oMf_torso.translation, R_torso=rs.oMf_torso.rotation,
            p_torso_ref=tr.p, R_torso_ref=tr.R,
            v_torso_ref=np.concatenate([tr.v_lin, tr.v_ang]),
            a_torso_ff=np.concatenate([tr.a_lin, tr.a_ang]))

        ee_kw = {}
        if swing_arm:
            J_ee = rs.J_tool_b if swing_arm == 'b' else rs.J_tool_a
            Jdq = rs.Jdot_dq_tool_b if swing_arm == 'b' else rs.Jdot_dq_tool_a
            oMf = rs.oMf_tool_b if swing_arm == 'b' else rs.oMf_tool_a
            if not extending:
                sr = swing_pl.reference_at(min(tq, T_nom - 0.01))
                if sr.is_swinging:
                    ee_kw = dict(J_ee=J_ee, Jdot_dq_ee=Jdq,
                        p_ee=oMf.translation, p_ee_ref=sr.p_ee,
                        v_ee_ref=sr.v_ee, a_ee_ff=sr.a_ee)
            else:
                ee_kw = dict(J_ee=J_ee, Jdot_dq_ee=Jdq,
                    p_ee=oMf.translation, p_ee_ref=p_target,
                    v_ee_ref=np.zeros(3), a_ee_ff=np.zeros(3))

        try:
            _, _, _, tau, _ = wbqp.solve(
                q_t=rs.q_torso, dq_t=rs.dq_torso, q=rs.q_joints, dq=rs.dq_joints,
                r_com_ref=rp, v_com_ref=vp, lambda_ref=lr, a_com_ff=af,
                H_robot=rs.H, C_robot=rs.C, J_com=rs.J_com, Jdot_dq_com=rs.Jdot_dq_com,
                contact_config=cc, J_contacts=Jc, Jdot_dq_contacts=Jdc,
                hw_current=hw, hw_min=hwm, hw_max=hwM, r_com=rs.r_com,
                **torso_kw, **ee_kw)
        except:
            tau = np.zeros(12)
        tau = np.clip(tau, -TAU_MAX, TAU_MAX)
        mj_data.ctrl[:12] = tau
        mujoco.mj_step(mj_model, mj_data)
        rs2 = robot.update(*mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel))
        hw -= (rs2.L_com - rs.L_com) / dt_q * dt_q
        hw = np.clip(hw, hwm, hwM)

        # Docking check
        if sg_name and sa_name:
            mujoco.mj_forward(mj_model, mj_data)
            d = np.linalg.norm(mj_data.site_xpos[sg_id] - mj_data.site_xpos[sa_id])
            if d < WELD_R and t > 0.5:
                weld_key = (swing_arm, gp.swing_to_idx)
                if weld_key in weld_map:
                    mj_data.eq_active[weld_map[weld_key]] = 1
                print(f"   DOCK! {swing_arm.upper()} arm at t={t+qs*dt_q:.2f}s d={d*1000:.1f}mm")
                docked_this_step = True
                break

    # Log every 0.5s
    step_i = int(round(t / dt_n))
    if sg_name and sa_name:
        mujoco.mj_forward(mj_model, mj_data)
        d = np.linalg.norm(mj_data.site_xpos[sg_id] - mj_data.site_xpos[sa_id])
        if step_i % 5 == 0:
            ext = " [EXT]" if extending else ""
            print(f"   t={t:5.2f}  d_{swing_arm}={d*100:6.2f}cm  ||τ||={np.linalg.norm(tau):5.1f}Nm{ext}")

    if docked_this_step:
        step_results.append(('dock', t, swing_arm))
        # If the plan has more phases, continue
        if extending:
            break  # extension completed

    t += dt_n

# === Summary ===
print("\n" + "=" * 50)
print("MULTI-STEP RESULTS")
print("=" * 50)
for ev, te, arm in step_results:
    print(f"  {ev}: arm {arm.upper()} at t={te:.2f}s")
print(f"\nTotal steps completed: {len(step_results)}")
