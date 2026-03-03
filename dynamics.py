"""
Pinocchio constrained forward dynamics for VISPA dual-arm crawler.

Three locomotion modes with exact holonomic constraints (Lagrange multipliers):
    ARM_A_DOCKED  :  tool_a == anchor_a  (6 DOF)
    ARM_B_DOCKED  :  tool_b == anchor_b  (6 DOF)
    BOTH_DOCKED   :  both tools fixed    (12 DOF)

Integration: semi-implicit Euler + SHAKE (position) + RATTLE (velocity)
    -> constraint violation ~1e-11
"""

import numpy as np
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict
from pathlib import Path

import pinocchio as pin

# ─── Model discovery ─────────────────────────────────────────────────────────
_PKG_DIR = Path(__file__).resolve().parent
_MODELS_DIR = _PKG_DIR.parent / "models"
URDF_PATH = str(_MODELS_DIR / "VISPA_crawling_fixed.urdf")
MJCF_PATH = str(_MODELS_DIR / "VISPA_crawling.xml")

# ─── Frame / joint indices (from URDF inspection) ────────────────────────────
FRAME_TOOL_A = 18   # "tool_a"  parent_joint = 7 (Joint_6_a)
FRAME_TOOL_B = 32   # "tool_b"  parent_joint = 13 (Joint_6_b)
JOINT_6A_ID  = 7
JOINT_6B_ID  = 13

# ─── Anchor grid (world frame, identity orientation) ─────────────────────────
DEFAULT_ANCHORS: Dict[str, pin.SE3] = {}
for _i in range(6):
    _x = _i * 0.8
    DEFAULT_ANCHORS[f"anchor_{_i+1}a"] = pin.SE3(np.eye(3), np.array([_x,  0.3, 0.0]))
    DEFAULT_ANCHORS[f"anchor_{_i+1}b"] = pin.SE3(np.eye(3), np.array([_x, -0.3, 0.0]))


class LocomotionMode(Enum):
    """Locomotion phases for dual-arm crawling."""
    ARM_A_DOCKED = auto()   # Arm A grasps structure, arm B free
    ARM_B_DOCKED = auto()   # Arm B grasps structure, arm A free
    BOTH_DOCKED  = auto()   # Both arms grasp (transition phase)


@dataclass
class AnchorConfig:
    """SE3 poses of contact anchors on the structure (world frame)."""
    anchor_a: pin.SE3 = field(default_factory=lambda: DEFAULT_ANCHORS["anchor_3a"])
    anchor_b: pin.SE3 = field(default_factory=lambda: DEFAULT_ANCHORS["anchor_3b"])


@dataclass
class DynamicsResult:
    """Output of constrained forward dynamics."""
    ddq: np.ndarray            # (nv,) joint accelerations
    lambda_c: np.ndarray       # (n_c,) Lagrange multipliers
    constraint_violation: float # ||log6(oMf_tool^{-1} * anchor)||
    prox_iters: int            # proximal solver iterations


class ConstraintProjector:
    """
    SHAKE (position) + RATTLE (velocity) projection onto the
    constraint manifold  c(q) = 0,  J_c(q) v = 0.
    """

    def __init__(self, model: pin.Model, frame_ids: List[int],
                 targets: List[pin.SE3], max_iter: int = 5, tol: float = 1e-10):
        self.model = model
        self.frame_ids = frame_ids
        self.targets = targets
        self.max_iter = max_iter
        self.tol = tol
        self._data = model.createData()

    def project_position(self, q: np.ndarray) -> np.ndarray:
        """SHAKE: Newton projection q -> c(q) = 0."""
        q = q.copy()
        for _ in range(self.max_iter):
            pin.forwardKinematics(self.model, self._data, q)
            pin.updateFramePlacements(self.model, self._data)
            pin.computeJointJacobians(self.model, self._data, q)
            errs, Js = [], []
            for fid, tgt in zip(self.frame_ids, self.targets):
                errs.append(pin.log6(self._data.oMf[fid].actInv(tgt)).vector)
                Js.append(pin.getFrameJacobian(self.model, self._data, fid, pin.LOCAL))
            err_stack = np.concatenate(errs)
            if np.linalg.norm(err_stack) < self.tol:
                break
            J = np.vstack(Js)
            nc = J.shape[0]
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-12 * np.eye(nc), err_stack)
            q = pin.integrate(self.model, q, dq)
        return q

    def project_velocity(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """RATTLE: project v onto ker(J_c)."""
        pin.forwardKinematics(self.model, self._data, q)
        pin.updateFramePlacements(self.model, self._data)
        pin.computeJointJacobians(self.model, self._data, q)
        Js = [pin.getFrameJacobian(self.model, self._data, fid, pin.LOCAL)
              for fid in self.frame_ids]
        J = np.vstack(Js)
        nc = J.shape[0]
        dv = J.T @ np.linalg.solve(J @ J.T + 1e-12 * np.eye(nc), J @ v)
        return v - dv


class VISPAConstrainedDynamics:
    """
    Constrained forward dynamics for VISPA dual-arm crawler.

    Uses Pinocchio's constraintDynamics (proximal Lagrangian) + SHAKE/RATTLE.

    Parameters
    ----------
    urdf_path : str, optional
        Path to corrected URDF. Defaults to bundled model.

    Examples
    --------
    >>> vcd = VISPAConstrainedDynamics()
    >>> anchors = AnchorConfig(
    ...     anchor_a=DEFAULT_ANCHORS["anchor_3a"],
    ...     anchor_b=DEFAULT_ANCHORS["anchor_3b"],
    ... )
    >>> vcd.set_mode(LocomotionMode.ARM_A_DOCKED, anchors)
    >>> res = vcd.forward_dynamics(q, v, tau)
    >>> q_next, v_next = vcd.integrate(q, v, res.ddq, dt=1e-3)
    """

    def __init__(self, urdf_path: str = URDF_PATH):
        self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.data  = self.model.createData()
        assert self.model.nq == 19 and self.model.nv == 18, \
            f"Unexpected model dimensions: nq={self.model.nq}, nv={self.model.nv}"
        self.tool_a_placement = self.model.frames[FRAME_TOOL_A].placement
        self.tool_b_placement = self.model.frames[FRAME_TOOL_B].placement
        self.prox_settings = pin.ProximalSettings(1e-12, 1e-12, 1e-8, 50)
        self._mode: Optional[LocomotionMode] = None
        self._contact_models: List[pin.RigidConstraintModel] = []
        self._contact_datas:  List[pin.RigidConstraintData]  = []
        self._projector: Optional[ConstraintProjector] = None
        self._anchors: Optional[AnchorConfig] = None
        self._initialized = False

    # ── Mode configuration ───────────────────────────────────────
    def set_mode(self, mode: LocomotionMode,
                 anchors: Optional[AnchorConfig] = None) -> None:
        """
        Configure holonomic constraints for a locomotion phase.

        Parameters
        ----------
        mode : LocomotionMode
        anchors : AnchorConfig, optional
            World-frame SE3 of dock points. Defaults to anchor_3a / anchor_3b.
        """
        if anchors is None:
            anchors = AnchorConfig()
        self._anchors = anchors
        self._mode = mode
        self._contact_models = []
        frame_ids, targets = [], []

        if mode in (LocomotionMode.ARM_A_DOCKED, LocomotionMode.BOTH_DOCKED):
            cm = pin.RigidConstraintModel(
                pin.ContactType.CONTACT_6D, self.model,
                JOINT_6A_ID, self.tool_a_placement,
                0, anchors.anchor_a, pin.LOCAL)
            cm.name = "dock_arm_a"
            self._contact_models.append(cm)
            frame_ids.append(FRAME_TOOL_A)
            targets.append(anchors.anchor_a)

        if mode in (LocomotionMode.ARM_B_DOCKED, LocomotionMode.BOTH_DOCKED):
            cm = pin.RigidConstraintModel(
                pin.ContactType.CONTACT_6D, self.model,
                JOINT_6B_ID, self.tool_b_placement,
                0, anchors.anchor_b, pin.LOCAL)
            cm.name = "dock_arm_b"
            self._contact_models.append(cm)
            frame_ids.append(FRAME_TOOL_B)
            targets.append(anchors.anchor_b)

        self._contact_datas = [cm.createData() for cm in self._contact_models]
        self.data = self.model.createData()
        pin.initConstraintDynamics(self.model, self.data, self._contact_models)
        self._projector = ConstraintProjector(self.model, frame_ids, targets)
        self._initialized = True

    # ── Forward dynamics ─────────────────────────────────────────
    def forward_dynamics(self, q: np.ndarray, v: np.ndarray,
                         tau: np.ndarray) -> DynamicsResult:
        """
        Constrained forward dynamics.

        Solves M q'' + h = S^T tau + J_c^T lambda,  J_c q'' + dJ_c q' = 0

        Parameters
        ----------
        q   : (19,) positions  [pos(3), quat_xyzw(4), joints(12)]
        v   : (18,) velocities [twist(6), qdot(12)]
        tau : (18,) forces     [base_wrench(6)=0, torques(12)]
        """
        assert self._initialized, "Call set_mode() first"
        ddq = pin.constraintDynamics(
            self.model, self.data, q, v, tau,
            self._contact_models, self._contact_datas, self.prox_settings)
        return DynamicsResult(
            ddq=ddq.copy(),
            lambda_c=self.data.lambda_c.copy(),
            constraint_violation=self._violation(q),
            prox_iters=self.prox_settings.iter)

    # ── Integration (with projection) ────────────────────────────
    def integrate(self, q: np.ndarray, v: np.ndarray,
                  ddq: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """Semi-implicit Euler + SHAKE + RATTLE. Returns (q_next, v_next)."""
        v_next = v + ddq * dt
        q_next = pin.integrate(self.model, q, v_next * dt)
        q_next = self._projector.project_position(q_next)
        v_next = self._projector.project_velocity(q_next, v_next)
        return q_next, v_next

    # ── Analytical derivatives (for MPC) ─────────────────────────
    def compute_derivatives(self, q: np.ndarray, v: np.ndarray,
                            tau: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Analytical derivatives of constrained dynamics.
        MUST be called after forward_dynamics() with same (q, v, tau).

        Returns
        -------
        dict with keys:
            dddq_dq, dddq_dv, dddq_dtau    : (nv, nv)
            dlambda_dq, dlambda_dv, dlambda_dtau : (nc, nv)
        """
        pin.computeConstraintDynamicsDerivatives(
            self.model, self.data,
            self._contact_models, self._contact_datas, self.prox_settings)
        return {
            "dddq_dq":     self.data.ddq_dq.copy(),
            "dddq_dv":     self.data.ddq_dv.copy(),
            "dddq_dtau":   self.data.ddq_dtau.copy(),
            "dlambda_dq":  self.data.dlambda_dq.copy(),
            "dlambda_dv":  self.data.dlambda_dv.copy(),
            "dlambda_dtau": self.data.dlambda_dtau.copy(),
        }

    # ── Kinematic / dynamic queries ──────────────────────────────
    def _violation(self, q: np.ndarray) -> float:
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        errs = []
        if self._mode in (LocomotionMode.ARM_A_DOCKED, LocomotionMode.BOTH_DOCKED):
            errs.append(np.linalg.norm(
                pin.log6(self.data.oMf[FRAME_TOOL_A].actInv(self._anchors.anchor_a)).vector))
        if self._mode in (LocomotionMode.ARM_B_DOCKED, LocomotionMode.BOTH_DOCKED):
            errs.append(np.linalg.norm(
                pin.log6(self.data.oMf[FRAME_TOOL_B].actInv(self._anchors.anchor_b)).vector))
        return max(errs) if errs else 0.0

    def tool_poses(self, q: np.ndarray) -> Tuple[pin.SE3, pin.SE3]:
        """World-frame SE3 of tool_a and tool_b."""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[FRAME_TOOL_A].copy(), self.data.oMf[FRAME_TOOL_B].copy()

    def tool_jacobians(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """6 x nv Jacobians (LOCAL_WORLD_ALIGNED) of tool_a, tool_b."""
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        Ja = pin.getFrameJacobian(self.model, self.data, FRAME_TOOL_A, pin.LOCAL_WORLD_ALIGNED)
        Jb = pin.getFrameJacobian(self.model, self.data, FRAME_TOOL_B, pin.LOCAL_WORLD_ALIGNED)
        return Ja.copy(), Jb.copy()

    def centroidal_momentum(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """6D centroidal momentum [linear(3); angular(3)]."""
        pin.computeCentroidalMomentum(self.model, self.data, q, v)
        return self.data.hg.vector.copy()

    def com(self, q: np.ndarray) -> np.ndarray:
        return pin.centerOfMass(self.model, self.data, q).copy()

    def kinetic_energy(self, q: np.ndarray, v: np.ndarray) -> float:
        pin.computeKineticEnergy(self.model, self.data, q, v)
        return self.data.kinetic_energy

    def mass_matrix(self, q: np.ndarray) -> np.ndarray:
        """(nv, nv) joint-space mass matrix."""
        return pin.crba(self.model, self.data, q).copy()

    def gravity_torques(self, q: np.ndarray) -> np.ndarray:
        """(nv,) gravity torques (zero in microgravity, but available for generality)."""
        return pin.computeGeneralizedGravity(self.model, self.data, q).copy()

    @property
    def mode(self): return self._mode
    @property
    def anchors(self): return self._anchors
    @property
    def n_constraints(self): return 6 * len(self._contact_models)
    @property
    def nq(self): return self.model.nq
    @property
    def nv(self): return self.model.nv
    @property
    def total_mass(self):
        d = self.model.createData()
        pin.computeTotalMass(self.model, d)
        return d.mass[0]
