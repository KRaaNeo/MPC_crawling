# VISPA Whole-Body Controller for Space Crawling Locomotion

## 1. Overview

This package implements a **hierarchical whole-body controller** for the VISPA dual-arm crawling robot operating in microgravity. The robot locomotes by grasping discrete anchor points on a free-floating structure (e.g. an orbital module truss), alternating stance/swing phases in a quasi-static gait.

The controller follows a two-stage architecture:

- **Stage 1 — Centroidal NMPC**: plans momentum-feasible contact wrenches over a receding horizon, respecting reaction wheel capacity constraints.
- **Stage 2 — Whole-Body QP**: tracks the centroidal references at the joint level while simultaneously controlling the torso pose, swing end-effector trajectory, and posture regulation.

A key design feature is the **torso 6D task** that replaces the traditional CoM-only tracking. In microgravity with tight torque limits (10 Nm), a pure CoM task creates an irreconcilable conflict between advancing the CoM and reaching the next anchor — the stance arm cannot simultaneously push the torso forward and position the CoM. The torso 6D task resolves this by directly controlling the torso body as an inverted manipulator, with the CoM reference derived from the torso trajectory to maintain NMPC coherence.

### Validated performance (single step, MuJoCo simulation)

| Metric | Value |
|--------|-------|
| Real docking distance | 4.1 mm (no teleportation) |
| Torso advancement | +31 cm (vs −54 cm with CoM-only) |
| Joint torque limit | 10 Nm (space-grade actuators) |
| max ‖L_robot‖ | 3.25 Nms (limit: 5 Nms) |
| Structure drift | 5.6 cm (500 kg free-floating) |
| NMPC solve rate | 10 Hz (IPOPT, N=8, dt=0.1s) |
| QP solve rate | 100 Hz (qpOASES) |


## 2. Theoretical Background

### 2.1 System Description

VISPA is a 71 kg dual-arm robot with a 40 kg central torso and two 6-DOF arms (≈15.5 kg each). The robot is modelled as a free-floating multibody system with generalized coordinates:

```
q = [q_torso(7), q_joints(12)]  ∈ SE(3) × ℝ¹²
v = [v_torso(6), q̇_joints(12)]  ∈ ℝ¹⁸
```

where `q_torso = [quaternion(4), position(3)]` parameterises the torso pose in the structure frame R_s. The robot crawls along a free-floating structure (500 kg) equipped with discrete anchor points. In microgravity, there is no gravity term in the equations of motion.

### 2.2 Equations of Motion

The unconstrained dynamics in the structure frame are:

```
H(q) v̇ + C(q, v) = S^T τ + J_c^T λ
```

where:
- `H ∈ ℝ^{18×18}` is the joint-space inertia matrix (CRBA)
- `C ∈ ℝ^{18}` is the Coriolis/centrifugal vector (RNEA, no gravity)
- `S = [0_{12×6}, I_{12}]` selects actuated joints
- `J_c` stacks the contact Jacobians of active grippers
- `λ` is the vector of contact wrenches
- `τ ∈ ℝ^{12}` is the joint torque vector

The holonomic contact constraint is `J_c v̇ + J̇_c v = 0`, enforced as an equality in the QP. In MuJoCo, contacts are modelled via penalty-based weld constraints (`solref="0.003 1"`), creating a deliberate model mismatch with the exact-constraint formulation used in the controller — this tests robustness.

### 2.3 Centroidal Dynamics

The centroidal momentum dynamics describe the aggregate effect of contact wrenches on the robot's centre of mass and angular momentum:

```
ṙ_com = v_com
m v̇_com = Σ_j f_j                                          (no gravity)
L̇_robot = Σ_j [(r_Cj − r_com) × f_j + τ_j]
```

where `f_j, τ_j` are the force and torque at contact j, and `r_Cj` is the contact position. The total angular momentum of the system (robot + structure) is conserved:

```
L_robot + h_w = const
```

where `h_w` is the reaction wheel momentum vector. The robot's angular momentum must remain within the capacity of the wheels:

```
‖L_robot‖ ≤ L_max        (wheel storage capacity)
‖L̇_robot‖ ≤ τ_w,max      (wheel torque capacity)
```

These constraints are enforced both in the NMPC (proactively, over the horizon) and in the QP (reactively, at each time step).


### 2.4 Stage 1 — Centroidal NMPC

The NMPC plans momentum-feasible trajectories by solving a nonlinear program at each control cycle (10 Hz):

**State**: `x = [r_com(3), v_com(3), L_com(3), h_w(3)]  ∈ ℝ^{12}`

**Control**: `u = [f_1(3), τ_1(3), f_2(3), τ_2(3)]  ∈ ℝ^{12}` (contact wrenches)

**Cost function**:
```
J = Σ_{k=0}^{N-1} [ ‖r_com,k − r_ref‖²_Wr + ‖v_com,k − v_ref‖²_Wv + ‖u_k‖²_Wu ]
    + ‖r_com,N − r_ref‖²_Qf_r + ‖v_com,N − v_ref‖²_Qf_v
```

**Constraints**:
- Dynamics (RK4 integration of centroidal ODE)
- State bounds: `|L_com| ≤ L_max` (wheel capacity, per axis)
- State bounds: `|h_w| ≤ h_w,max` (wheel saturation envelope)
- Path constraints: `‖f_j‖ ≤ f_max`, `‖τ_j‖ ≤ τ_max` (SOC on wrenches)
- Path constraints: `|L̇_robot| ≤ τ_w,max` (wheel torque rate, per axis)
- Inactive contacts: `u_j = 0` for released grippers

The solver is IPOPT via CasADi, with warm-starting from the shifted previous solution.

**Outputs to Stage 2**: `r_com_ref, v_com_ref, λ_ref, a_com_ff` at the first horizon step.


### 2.5 CoM Reference from Torso Trajectory

A pure CoM tracking task conflicts with end-effector reaching under tight torque limits. The torso 6D task resolves this but creates a coupling problem: the NMPC needs a CoM reference consistent with the actual torso motion.

The solution derives the CoM reference from the planned torso trajectory:

```
r_com(t) = p_torso(t) + R_torso(t) · δ_com(s(t))
```

where `δ_com` is the CoM offset in the torso body frame, interpolated between start and end dock configurations:

```
δ_com(s) = (1 − s) · δ_start + s · δ_end
```

This interpolation is necessary because the offset varies by ≈6 cm as the arms reconfigure during a step (the CoM shifts relative to the torso as limb mass redistributes). The quintic time scaling `s(t)` ensures C² continuity.

The velocity is obtained by chain rule:

```
v_com = v_torso_lin + ω_torso × (R · δ) + R · δ̇
```

This derived CoM reference feeds the NMPC, which then plans contact wrenches consistent with the actual torso motion. The coupling loop is:

```
TorsoPlanner → CoM ref → NMPC → λ_ref → QP → τ → MuJoCo
     ↑                                              |
     └──────────── feedback (measured state) ───────┘
```


### 2.6 Stage 2 — Whole-Body QP

The QP solves at each control step (100 Hz) for decision variables `z = [v̇_torso(6), q̈(12), λ(12), τ(12)]`:

**Tasks** (weighted sum, not strict priority):

| Task | Weight α | Formulation |
|------|----------|-------------|
| Torso 6D | 500 | `‖J_torso v̇ + J̇_torso v − ä_torso_des‖²` |
| End-effector | 3000 | `‖J_ee v̇ + J̇_ee v − a_ee_des‖²` |
| CoM tracking | 200 | `‖J_com v̇ + J̇_com v − a_com_des‖²` |
| Wrench tracking | 100 | `‖λ − λ_ref‖²` |
| Posture | 20 | `‖q̈ − q̈_posture‖²` |
| Torque min. | 1 | `‖τ‖²` |
| Regularisation | 0.01 | `‖z‖²` |

Each Cartesian task uses a PD law for the desired acceleration. For example, the torso task:

```
a_torso_des = a_ff + Kp · e_pos + Kd · e_vel
```

where `e_pos` combines position error and orientation error (via `log(R_ref^T R)` for the angular part), and `e_vel = v_ref − v`.

**Equality constraints**:
- Dynamics: `H v̇ + C = S^T τ + J_c^T λ`
- Contact kinematics: `J_c v̇ + J̇_c v = 0` (for active contacts)

**Inequality constraints**:

| Constraint | Rows | Formulation |
|------------|------|-------------|
| Wheel momentum box | 6 | `h_w,min ≤ h_w − dt·M_λ·λ ≤ h_w,max` |
| Robot L box | 6 | `−L_max ≤ L_com + dt·M_λ·λ ≤ L_max` |
| Robot L̇ box | 6 | `−τ_w,max ≤ M_λ·λ ≤ τ_w,max` |

where `M_λ` is the momentum map matrix (3×12) that maps contact wrenches to angular momentum rate:

```
M_λ = [skew(r_C1 − r_com)  I₃  skew(r_C2 − r_com)  I₃]
```

**Bounds**: `|τ_i| ≤ τ_max`, `|q̈_i| ≤ q̈_max`, wrench bounds per contact (zero for inactive).

The solver is qpOASES (warm-started, interior-point).


### 2.7 Locomotion Phases

Each crawling step follows the sequence:

```
DS (0.5s) → SS (6.0s) → EXT (until dock or timeout)
```

- **DS (Double Support)**: both grippers attached, torso holds position, NMPC regulates momentum.
- **SS (Single Support)**: swing arm released, torso advances via quintic trajectory (delayed start at 20% of phase), swing arm follows quintic Cartesian path with 3 cm clearance bump. A fraction `TORSO_FRAC=0.70` of the full IK displacement is used to avoid workspace limits.
- **EXT (Extension)**: swing phase nominally ended, torso reference frozen at captured pose, end-effector tracking weight boosted to maximum (α=10000) for final approach. Dock declared when `‖gripper − anchor‖ < 5 mm`.

The torso trajectory is parameterised by a quintic polynomial (C² continuity) with SLERP orientation interpolation via the exponential map. Velocity and acceleration profiles are analytically derived for feedforward control.

### 2.8 Model Mismatch and Robustness

The controller uses Pinocchio for exact constrained dynamics (Lagrange multipliers), while MuJoCo simulates with penalty-based contacts (`solref`). This creates intentional model mismatch:

- The QP assumes perfect holonomic constraints; MuJoCo enforces them with stiff springs.
- Contact wrenches in the QP are decision variables; in MuJoCo they emerge from the penalty solver.
- The L̇ constraint in the QP is satisfied exactly in the model, but the measured L̇ in MuJoCo shows transient violations (up to 11 Nm vs 2 Nm limit) due to penalty dynamics.

Despite this, the L_com state bound is respected (3.25 Nms vs 5 Nms limit), demonstrating that the proactive NMPC planning absorbs model mismatch through integrated momentum management.

The free-floating structure (500 kg) recoils under Newton's third law: for every 1 cm of torso advancement, the structure retreats ≈0.14 cm. The controller tracks anchor positions in real time via MuJoCo site queries, compensating for this drift. With a docking threshold of 5 mm, the last centimetre of approach creates an equilibrium between EE convergence and structure recoil — a physical limit that would require a capture mechanism (guide cone, magnetic latch) in practice, consistent with standard space docking design.


## 3. Architecture Diagram

```
                              ┌──────────────────────┐
                              │   ContactScheduler    │
                              │  (gait timing, phase  │
                              │   transitions, welds) │
                              └──────┬───────────────┘
                                     │ ContactConfig
                    ┌────────────────┼────────────────┐
                    │                │                 │
                    ▼                ▼                 ▼
 ┌─────────────────────┐  ┌──────────────────┐  ┌──────────────────┐
 │    TorsoPlanner      │  │   SwingPlanner   │  │ LocomotionPlanner│
 │  quintic + SLERP     │  │  quintic + bump  │  │ (CoM quintic,    │
 │  → p,R,v,a torso     │  │  → p,v,a EE      │  │  calibration)    │
 │  → r_com,v_com (CoM  │  │                  │  │                  │
 │    derived from δ)    │  │                  │  │                  │
 └───────┬──────────────┘  └──────┬───────────┘  └──────────────────┘
         │ r_com_ref, v_com_ref   │ p_ee_ref
         ▼                        │
 ┌───────────────────────────┐    │
 │  CentroidalNMPC (Stage 1) │    │
 │  CasADi / IPOPT — 10 Hz   │    │
 │                            │    │
 │  x = [r, v, L, hw]        │    │
 │  u = [f₁,τ₁, f₂,τ₂]      │    │
 │                            │    │
 │  Constraints:              │    │
 │   |L_com| ≤ L_max         │    │
 │   |L̇_com| ≤ τ_w_max      │    │
 │   |hw| ≤ hw_max           │    │
 │   ‖fⱼ‖ ≤ f_max (SOC)     │    │
 └───────┬───────────────────┘    │
         │ r_plan, v_plan,        │
         │ λ_ref, a_com_ff        │
         ▼                        ▼
 ┌────────────────────────────────────────┐
 │       WholeBodyQP (Stage 2)            │
 │       qpOASES — 100 Hz                 │
 │                                        │
 │  z = [v̇_t, q̈, λ, τ]                 │
 │                                        │
 │  Tasks (weighted):                     │
 │   Torso 6D    (α=500)                 │
 │   EE swing    (α=3000)                │
 │   CoM NMPC    (α=200)                 │
 │   Wrench NMPC (α=100)                 │
 │   Posture     (α=20)                  │
 │   Torque min  (α=1)                   │
 │                                        │
 │  Eq:  H v̇ + C = Sᵀτ + Jcᵀλ          │
 │       Jc v̇ + J̇c v = 0               │
 │  Ineq: L box, L̇ box, hw box          │
 └──────────────┬─────────────────────────┘
                │ τ (12 joint torques)
                ▼
 ┌────────────────────────────────────────┐
 │            MuJoCo Simulation           │
 │  penalty contacts, free-floating       │
 │  structure (500 kg), dt=0.01s          │
 └──────────────┬─────────────────────────┘
                │ qpos, qvel
                ▼
 ┌────────────────────────────────────────┐
 │  RobotInterface (Pinocchio wrapper)    │
 │  CRBA, RNEA, Jacobians, kinematics    │
 │  mujoco_to_pinocchio() conversion     │
 └────────────────────────────────────────┘
```


## 4. File Layout

```
vispa_controller/
├── README.md                  # This file
│
├── sim_torso6d.py             # Main simulation script (run this)
├── plot_torso6d.py            # 6-panel visualisation
│
├── robot_interface.py         # Pinocchio wrapper: CRBA, RNEA, Jacobians,
│                              #   CoM, centroidal momentum, torso/EE frames
├── contact_scheduler.py       # Gait plan, phase timing, weld activation,
│                              #   anchor SE3 poses from MuJoCo sites
├── locomotion_planner.py      # CoM quintic reference (legacy, used for
│                              #   calibration; superseded by TorsoPlanner)
├── swing_planner.py           # End-effector quintic trajectory with
│                              #   clearance bump for obstacle avoidance
├── torso_planner.py           # Torso 6D planner: quintic position + SLERP
│                              #   orientation, CoM reference derivation
│                              #   from interpolated body-frame offset
│
├── simulation_loop.py         # Generic simulation orchestrator
├── dynamics.py                # VISPA constrained dynamics (Pinocchio)
├── convert.py                 # MuJoCo ↔ Pinocchio state conversion
│                              #   (quaternion ordering, coordinate frames)
├── ik.py                      # Inverse kinematics for dock configurations
│
├── solvers/
│   ├── centroidal_nmpc.py     # Stage 1: CasADi/IPOPT NMPC with L and L̇
│   │                          #   constraints, SOC on wrenches, RK4 dynamics
│   ├── wholebody_qp.py        # Stage 2: qpOASES QP with torso 6D, CoM,
│   │                          #   EE, wrench tracking, L/L̇/hw inequalities
│   ├── nmpc_solver.py         # Generic NMPC backend (CasADi NLP builder)
│   ├── hierarchical_qp.py     # Generic HQP backend (strict/weighted modes)
│   └── contact_phase.py       # Contact phase definitions, momentum map
│                              #   M_λ computation, skew-symmetric utilities
│
├── test_integration.py        # Integration tests: Pinocchio/MuJoCo/NMPC/QP
├── test_torso_task.py         # Unit tests for torso 6D task
└── test_multi_step.py         # Multi-step locomotion test harness
```


## 5. Dependencies

```bash
pip install numpy pinocchio casadi mujoco matplotlib
# qpOASES: pip install qpoases  (or conda install -c conda-forge qpoases)
```

Tested with Python 3.10+, Pinocchio 2.7+, CasADi 3.6+, MuJoCo 3.1+.


## 6. Quick Start

### Single-step simulation with real docking

```bash
python sim_torso6d.py
```

This runs one crawling step (anchor 3b → 4b) with the full NMPC+QP pipeline
and produces a 6-panel plot showing torso advancement, EE convergence,
CoM tracking, momentum, joint torques, and structure drift.

### Key parameters (in sim_torso6d.py)

```python
TORSO_MASS  = 40.0    # kg — corrected torso mass
TAU_MAX     = 10.0    # Nm — space-grade actuator limit
T_SWING     = 6.0     # s  — single-support phase duration
TORSO_FRAC  = 0.70    # fraction of full IK displacement
TORSO_DELAY = 0.20    # torso starts at 20% of swing phase
L_MAX       = 5.0     # Nms — reaction wheel momentum capacity
TAU_W_MAX   = 2.0     # Nm  — reaction wheel torque capacity
WELD_R      = 0.005   # m   — docking success threshold (5 mm)
```

### Programmatic usage

```python
from robot_interface import RobotInterface
from solvers.centroidal_nmpc import CentroidalNMPC, CentroidalNMPCConfig
from solvers.wholebody_qp import WholeBodyQP, WholeBodyQPConfig
from torso_planner import TorsoPlanner

# Build NMPC with momentum constraints
nmpc = CentroidalNMPC(CentroidalNMPCConfig(
    robot_mass=71.0, N=8, dt=0.1,
    L_max=5.0, tau_w_max=2.0,
    hw_min=-50*np.ones(3), hw_max=50*np.ones(3),
))
nmpc.build()

# Build QP with torso 6D task and momentum constraints
qp = WholeBodyQP(WholeBodyQPConfig(
    nq=12, nc_max=2, dt_qp=0.01,
    tau_max=10.0*np.ones(12),
    alpha_torso=500, alpha_ee=3000, alpha_com=200,
    alpha_wrench=100, alpha_posture=20,
    L_max=5.0, tau_w_max=2.0,
))

# Setup torso planner with CoM offset derivation
torso_pl = TorsoPlanner()
torso_pl.add_phase(t0, tf, p_start, R_start, p_end, R_end,
                   delta_com_start=delta0, delta_com_end=delta1)

# Control loop
cref = torso_pl.com_reference_at(t)          # CoM ref from torso
r_plan, v_plan, _, lam, _ = nmpc.solve(      # NMPC with coherent ref
    r_com=rs.r_com, ..., r_com_ref=cref.r_com, v_com_ref=cref.v_com)
_, _, _, tau, _ = qp.solve(                   # QP tracks everything
    r_com_ref=r_plan, lambda_ref=lam,
    L_com_current=rs.L_com, ...)
```


## 7. Tuning Guide

### Momentum constraints

| Parameter | Typical range | Effect |
|-----------|--------------|--------|
| `L_max` | 2–50 Nms | Tighter = smoother motion, may slow convergence |
| `tau_w_max` | 1–10 Nm | Tighter = limits peak angular acceleration |
| `hw_min/max` | ±20–100 Nms | Wheel saturation envelope |

With very tight constraints (L_max=5, τ_w=2), docking at 5 mm is achievable but the controller cannot converge below ≈10 mm due to the equilibrium between EE approach forces and structure recoil. A capture mechanism would bridge this gap in practice.

### QP task weights

The weights define a soft priority hierarchy. The key trade-offs:

- Increase `α_torso`: torso tracks better, but EE may deviate.
- Increase `α_ee`: EE converges faster, but may violate momentum limits.
- Increase `α_com`: better NMPC coherence, but torso may lag.
- Increase `α_wrench`: wrenches closer to NMPC plan, less QP flexibility.

Phase-dependent tuning: during EXT (final approach), `α_ee` is boosted to 10000 and `α_torso` reduced to 50, freezing the torso and maximising EE authority.

### PD gains (microgravity-specific)

Gains must be much lower than terrestrial values due to the absence of gravity and tight torque limits:

| Gain | Terrestrial | Microgravity |
|------|------------|--------------|
| Kp_torso | 50–200 | 6–8 |
| Kd_torso | 20–50 | 5–6 |
| Kp_ee | 50–100 | 10–25 |
| Kd_ee | 20–40 | 7–12 |
| Kp_com | 20–50 | 2–3 |

Using terrestrial-scale gains in microgravity causes immediate torque saturation and instability.


## 8. Common Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| NMPC infeasible | L_max or τ_w_max too tight | Increase limits or slow down (longer T_SWING) |
| QP infeasible | Conflicting constraints | Check M_λ conditioning; reduce dt_qp |
| EE oscillates near target | Structure recoil equilibrium | Expected below ≈10 mm; add capture mechanism |
| Torque saturation | Gains too high or TORSO_FRAC too large | Reduce PD gains or TORSO_FRAC |
| L̇ measured exceeds limit | MuJoCo penalty mismatch | Expected; L_com in box confirms proactive control |
| Torso retreats in swing | CoM task dominates | Ensure α_torso > α_com |
| Drift > 10 cm/step | Mass ratio issue | Check TORSO_MASS = 40 kg |


## 9. References

- VISPA robot design and locomotion concept (Idriss, in preparation)
- Hierarchical MPC + QP architecture for legged locomotion: Bellicoso et al., IEEE-RA Letters, 2018
- Centroidal dynamics for momentum control: Orin et al., Autonomous Robots, 2013
- Whole-body QP formulation: Herzog et al., Autonomous Robots, 2016
- CasADi NLP framework: Andersson et al., Mathematical Programming Computation, 2019
- Pinocchio rigid-body dynamics: Carpentier et al., SoftwareX, 2019
