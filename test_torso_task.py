"""
Test: Cooperative dual-arm locomotion with torso task.

Key change: the QP tracks a 6D torso reference instead of a static CoM.
The stance arm pushes the torso forward (inverted manipulator) while
the swing arm reaches for the next anchor. Both tasks cooperate.
"""
import sys, time, numpy as np
sys.path.insert(0, '.')

URDF = '/mnt/user-data/uploads/VISPA_crawling_fixed.urdf'
MJCF = '/mnt/user-data/uploads/VISPA_crawling.xml'
TORSO_MASS = 40.0
TAU_MAX = 10.0       # realistic space actuator
WELD_R = 0.005       # 5 mm docking tolerance

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
sched = ContactScheduler(anchors_a=mj_a, anchors_b=mj_b, dt_ds=0.5, dt_ss=4.0)
plan = sched.plan_traversal(start_a=2, start_b=2, n_steps=1)

# Dock configuration
q_dock = dock_configuration(robot.model, sched.anchor_se3('a', 2), sched.anchor_se3('b', 2))
sp = mj_data.qpos[0:3].copy()
mqp, _ = pinocchio_to_mujoco(q_dock, np.zeros(18),
    struct_pos=sp, struct_quat=mj_data.qpos[3:7].copy())
mj_data.qpos[:] = mqp
mj_data.qvel[:] = 0.0

# Activate initial welds (both grippers docked at anchors 3a, 3b)
weld_map = {}
for i in range(mj_model.neq):
    nm = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
    if nm and nm.startswith('grip_'):
        pts = nm.split('_to_')
        arm = pts[0].split('_')[1]
        weld_map[(arm, int(pts[1][0]) - 1)] = i

for eq_id in range(mj_model.neq):
    mj_data.eq_active[eq_id] = 0
mj_data.eq_active[weld_map[('a', 2)]] = 1
mj_data.eq_active[weld_map[('b', 2)]] = 1

mujoco.mj_forward(mj_model, mj_data)
for _ in range(200):
    mujoco.mj_step(mj_model, mj_data)

# Initial state
rs = robot.update(*mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel))
print(f"Initial torso pos: {rs.oMf_torso.translation.round(3)}")
print(f"Initial CoM:       {rs.r_com.round(3)}")
print(f"Total mass:        {rs.total_mass:.1f} kg")

# === Planners ===
# Torso planner (replaces CoM planner)
torso_pl = TorsePlanner(sched, advance_ratio=0.5)
torso_pl.build_for_plan(plan, rs.oMf_torso)

# Swing planner
swing_pl = SwingPlanner(sched, clearance=0.03)

# NMPC (still needed for force distribution + momentum safety)
nmpc = CentroidalNMPC(CentroidalNMPCConfig(
    robot_mass=rs.total_mass, N=8, dt=0.1,
    f_max=20.0, tau_max=5.0,
    hw_min=np.full(3, -50), hw_max=np.full(3, 50)))
nmpc.build()

# === QP with torso task ===
def make_qp(alpha_torso, alpha_ee, alpha_post, Kp_t_lin=8., Kp_t_ang=5., Kp_ee=10., Kd_ee=7.):
    """Create QP with torso task (CoM disabled)."""
    cfg = WholeBodyQPConfig(
        nq=12, nc_max=2, dt_qp=0.01,
        tau_max=TAU_MAX * np.ones(12),
        # Disable CoM, enable Torso
        alpha_com=0.0,
        alpha_torso=alpha_torso,
        alpha_ee=alpha_ee,
        alpha_posture=alpha_post,
        alpha_wrench=1e1,
        alpha_torque=1e0,
        alpha_reg=1e-2,
        # Torso 6D gains [lin, ang]
        Kp_torso=np.array([Kp_t_lin]*3 + [Kp_t_ang]*3),
        Kd_torso=np.array([6., 6., 6., 4., 4., 4.]),
        # Unused CoM gains
        Kp_com=np.diag([3., 3., 5.]),
        Kd_com=np.diag([3., 3., 4.]),
        # EE gains
        Kp_ee=Kp_ee * np.ones(3),
        Kd_ee=Kd_ee * np.ones(3),
        Kp_posture=1.0,
        Kd_posture=1.5)
    qp = WholeBodyQP(cfg)
    qp.set_nominal_posture(q_dock[7:19])
    return qp

# Phase-dependent QPs
#   DS: torso dominant (both arms available for torso control)
#   SS: torso + EE balanced (cooperative motion)
#   Extension: EE dominant (final approach)
qp_ds  = make_qp(alpha_torso=1e3, alpha_ee=0,    alpha_post=1e2)
qp_ss  = make_qp(alpha_torso=5e2, alpha_ee=2e3,  alpha_post=2e1)
qp_fin = make_qp(alpha_torso=1e2, alpha_ee=5e3,  alpha_post=5e0, Kp_ee=18., Kd_ee=10.)

# === Simulation loop ===
dt_nmpc, dt_qp = 0.1, 0.01
nq_per = int(round(dt_nmpc / dt_qp))
hw = np.array([2., -1., 0.5])
hwm, hwM = np.full(3, -50.), np.full(3, 50.)

sg_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, 'gripper_b')
sa_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, 'anchor_4b')
p_target = mj_data.site_xpos[sa_id].copy()

T_nom = plan.t_end[-1]
last_pidx = -1
docked = False
min_d = 999.0
results = []

t0_wall = time.perf_counter()
t = 0.0

while t < 12.0:
    extending = (t >= T_nom)

    # --- Phase determination ---
    if not extending:
        gp, pidx = plan.phase_at(t)
        cc = sched.contact_config_at(t)
    else:
        gp = plan.phases[1]  # stay in single_a
        pidx = 1
        cc = sched.contact_config_at(plan.t_start[1] + 0.1)

    is_ss = gp.phase in (ContactPhase.SINGLE_A, ContactPhase.SINGLE_B)

    # Weld management on phase change (only during nominal plan)
    if pidx != last_pidx and not extending:
        for eq_id in weld_map.values():
            mj_data.eq_active[eq_id] = 0
        gpc = plan.phases[pidx]
        # Activate welds for docked arms
        if gpc.phase in (ContactPhase.SINGLE_A, ContactPhase.DOUBLE):
            k = ('a', gpc.anchor_a_idx)
            if k in weld_map:
                mj_data.eq_active[weld_map[k]] = 1
        if gpc.phase in (ContactPhase.SINGLE_B, ContactPhase.DOUBLE):
            k = ('b', gpc.anchor_b_idx)
            if k in weld_map:
                mj_data.eq_active[weld_map[k]] = 1
        last_pidx = pidx

    # Select QP
    if extending:
        wbqp = qp_fin
    elif is_ss:
        wbqp = qp_ss
    else:
        wbqp = qp_ds

    # --- NMPC solve (coarse rate) ---
    pq, pv = mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel)
    rs = robot.update(pq, pv)

    # CoM reference derived from torso plan (approximate)
    if not extending:
        t_ref = torso_pl.reference_at(t)
    else:
        t_ref = torso_pl.final_reference()

    # Use torso position as CoM reference (torso is ~56% of mass)
    # This is approximate but keeps NMPC stable
    com_offset = rs.r_com - rs.oMf_torso.translation
    r_com_ref = t_ref.p + com_offset
    v_com_ref = t_ref.v_lin * 0.6  # approximate scaling

    try:
        rp, vp, _, lr, _ = nmpc.solve(
            r_com=rs.r_com, v_com=rs.v_com, L_com=rs.L_com,
            hw_current=hw, r_com_ref=r_com_ref, v_com_ref=v_com_ref,
            contact_config=cc, warm_start=True)
        af = nmpc.compute_feedforward_acceleration(lr)
    except:
        rp, vp, lr, af = r_com_ref, v_com_ref, np.zeros(12), np.zeros(3)

    # --- QP loop (fine rate) ---
    for qs in range(nq_per):
        pq, pv = mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel)
        rs = robot.update(pq, pv)
        Jc, Jdc = robot.get_contact_jacobians(
            cc.active_contacts[0], cc.active_contacts[1])

        # Torso reference
        if not extending:
            tq = t + qs * dt_qp
            tr = torso_pl.reference_at(tq)
        else:
            tr = torso_pl.final_reference()

        # Torso task kwargs
        torso_kw = dict(
            J_torso=rs.J_torso,
            Jdot_dq_torso=rs.Jdot_dq_torso,
            p_torso=rs.oMf_torso.translation,
            R_torso=rs.oMf_torso.rotation,
            p_torso_ref=tr.p,
            R_torso_ref=tr.R,
            v_torso_ref=np.concatenate([tr.v_lin, tr.v_ang]),
            a_torso_ff=np.concatenate([tr.a_lin, tr.a_ang]),
        )

        # EE task (only during swing or extension)
        if not extending:
            sr = swing_pl.reference_at(min(t + qs * dt_qp, T_nom - 0.01))
            if sr.is_swinging and sr.swing_arm == 'b':
                ee_kw = dict(
                    J_ee=rs.J_tool_b,
                    Jdot_dq_ee=rs.Jdot_dq_tool_b,
                    p_ee=rs.oMf_tool_b.translation,
                    p_ee_ref=sr.p_ee,
                    v_ee_ref=sr.v_ee,
                    a_ee_ff=sr.a_ee)
            else:
                ee_kw = {}
        else:
            # Extension: PD to target anchor, no feedforward
            ee_kw = dict(
                J_ee=rs.J_tool_b,
                Jdot_dq_ee=rs.Jdot_dq_tool_b,
                p_ee=rs.oMf_tool_b.translation,
                p_ee_ref=p_target,
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
                contact_config=cc, J_contacts=Jc, Jdot_dq_contacts=Jdc,
                hw_current=hw, hw_min=hwm, hw_max=hwM,
                r_com=rs.r_com,
                **torso_kw, **ee_kw)
        except Exception as ex:
            tau = np.zeros(12)

        tau = np.clip(tau, -TAU_MAX, TAU_MAX)
        mj_data.ctrl[:12] = tau
        mujoco.mj_step(mj_model, mj_data)

        # Momentum bookkeeping
        rs2 = robot.update(*mujoco_to_pinocchio(mj_data.qpos, mj_data.qvel))
        hw -= (rs2.L_com - rs.L_com) / dt_qp * dt_qp
        hw = np.clip(hw, hwm, hwM)

        # Check docking at QP rate (critical: don't miss the approach)
        mujoco.mj_forward(mj_model, mj_data)
        d_grip_qp = np.linalg.norm(
            mj_data.site_xpos[sg_id] - mj_data.site_xpos[sa_id])
        if d_grip_qp < min_d:
            min_d = d_grip_qp
        if d_grip_qp < WELD_R and t > 0.5:
            # Activate weld for arm B at target anchor
            weld_key = ('b', plan.phases[1].swing_to_idx)
            if weld_key in weld_map:
                mj_data.eq_active[weld_map[weld_key]] = 1
            docked = True
            d_grip = d_grip_qp
            break

    if docked:
        break

    # --- Logging ---
    mujoco.mj_forward(mj_model, mj_data)
    d_grip = np.linalg.norm(mj_data.site_xpos[sg_id] - mj_data.site_xpos[sa_id])
    e_torso = np.linalg.norm(rs.oMf_torso.translation - tr.p)
    results.append((t, d_grip, e_torso, np.linalg.norm(tau)))

    t += dt_nmpc

wall_time = time.perf_counter() - t0_wall

# === Results ===
print("\n=== TORSO TASK LOCOMOTION TEST ===")
print(f"tau_max = {TAU_MAX} Nm, dock threshold = {WELD_R*1000:.0f} mm")
print(f"Wall time: {wall_time:.1f}s for {t:.1f}s sim ({t/max(wall_time,1e-6):.1f}× realtime)")
print()
print("t(s)     d_grip      e_torso  ||τ||    phase")
print("-" * 60)
for tt, dg, et, tn in results[::4]:
    ext = " [EXT]" if tt >= T_nom else ""
    flag = " <<<" if dg < 0.05 else ""
    print(f"{tt:5.2f}    {dg*100:6.2f}cm   {et:.4f}m  {tn:5.1f}Nm{ext}{flag}")

print()
print(f"Min grip distance: {min_d*100:.2f} cm = {min_d*1000:.1f} mm")
print(f"Threshold:         {WELD_R*100:.2f} cm = {WELD_R*1000:.1f} mm")
if docked:
    print(f"*** DOCK SUCCESS at t={t:.2f}s ***")
else:
    print(f"DOCK FAIL — need to close {(min_d-WELD_R)*100:.1f} cm more")
    print(f"Final torso pos: {rs.oMf_torso.translation.round(3)}")
    print(f"Final grip B pos: {mj_data.site_xpos[sg_id].round(3)}")
    print(f"Target anchor 4b: {p_target.round(3)}")
