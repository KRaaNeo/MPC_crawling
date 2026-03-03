"""Inverse kinematics for VISPA docking configurations."""

import numpy as np
from typing import Dict, Tuple
import pinocchio as pin

from dynamics import FRAME_TOOL_A, FRAME_TOOL_B


def solve_ik(
    model: pin.Model,
    q0: np.ndarray,
    targets: Dict[int, pin.SE3],
    max_iter: int = 500,
    tol: float = 1e-8,
) -> Tuple[np.ndarray, float]:
    """
    Iterative IK placing tool frames at target SE3 poses.

    Parameters
    ----------
    model : pin.Model
        Pinocchio model (free-flyer).
    q0 : (nq,) initial configuration guess.
    targets : {frame_id: SE3}
        e.g. {FRAME_TOOL_A: anchor_3a, FRAME_TOOL_B: anchor_3b}
    max_iter : int
    tol : float
        Convergence on sum of ||log6(err)||.

    Returns
    -------
    q : (nq,) converged configuration
    err : float, final error norm
    """
    q = q0.copy()
    data = model.createData()

    for it in range(max_iter):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        pin.computeJointJacobians(model, data, q)
        dq = np.zeros(model.nv)
        err_tot = 0.0

        for fid, tgt in targets.items():
            err = pin.log6(data.oMf[fid].actInv(tgt)).vector
            J = pin.getFrameJacobian(model, data, fid, pin.LOCAL)

            # Arm-specific joint slice
            if fid == FRAME_TOOL_A:
                idx = slice(6, 12)
            else:
                idx = slice(12, 18)
            Ja = J[:, idx]
            dq[idx] = np.linalg.solve(Ja.T @ Ja + 1e-4 * np.eye(6), Ja.T @ err)

            # Base contribution (conservative)
            Jb = J[:, :6]
            dq[:6] += np.linalg.solve(Jb.T @ Jb + 1e-3 * np.eye(6), Jb.T @ err) * 0.3

            err_tot += np.linalg.norm(err)

        alpha = min(1.0, 0.5 / max(np.max(np.abs(dq)), 1e-10))
        q = pin.integrate(model, q, alpha * dq)

        if err_tot < tol:
            break

    return q, err_tot


def dock_configuration(
    model: pin.Model,
    anchor_a: pin.SE3,
    anchor_b: pin.SE3,
    torso_pos: np.ndarray = None,
) -> np.ndarray:
    """
    Convenience: compute a valid configuration with both tools at anchors.

    Parameters
    ----------
    model : pin.Model
    anchor_a, anchor_b : SE3 target poses for tool_a, tool_b
    torso_pos : (3,) initial torso position guess (default: midpoint of anchors)

    Returns
    -------
    q : (nq,) valid docking configuration
    """
    q0 = pin.neutral(model)
    if torso_pos is None:
        torso_pos = 0.5 * (anchor_a.translation + anchor_b.translation)
    q0[:3] = torso_pos

    targets = {FRAME_TOOL_A: anchor_a, FRAME_TOOL_B: anchor_b}
    q, err = solve_ik(model, q0, targets)
    if err > 1e-4:
        raise RuntimeError(f"IK failed to converge: err={err:.2e}")
    return q
