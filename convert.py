"""
State conversion between MuJoCo and Pinocchio conventions.

Key differences
---------------
Quaternion ordering:
    MuJoCo    : (w, x, y, z)
    Pinocchio : (x, y, z, w)

State layout:
    MuJoCo qpos (26) : struct_pos(3) struct_quat(4) torso_pos(3) torso_quat(4) joints(12)
    MuJoCo qvel (24) : struct_v(3) struct_w(3) torso_v(3) torso_w(3) qdot(12)

    Pinocchio qpos (19) : torso_pos(3) torso_quat_xyzw(4) joints(12)
    Pinocchio qvel (18) : torso_v(3) torso_w(3) qdot(12)

Note: Pinocchio model is robot-only (no structure body).
      Structure state must be tracked separately.
"""

import numpy as np
from typing import Tuple


def mujoco_to_pinocchio(
    mj_qpos: np.ndarray, mj_qvel: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Convert MuJoCo full state to Pinocchio robot-only state.

    Parameters
    ----------
    mj_qpos : (26,) MuJoCo generalized positions
    mj_qvel : (24,) MuJoCo generalized velocities

    Returns
    -------
    pin_q : (19,) Pinocchio positions
    pin_v : (18,) Pinocchio velocities
    """
    pin_q = np.zeros(19)
    pin_v = np.zeros(18)

    # Position
    pin_q[0:3] = mj_qpos[7:10]       # torso position

    # Quaternion: MuJoCo (w,x,y,z) -> Pinocchio (x,y,z,w)
    w, x, y, z = mj_qpos[10:14]
    pin_q[3] = x
    pin_q[4] = y
    pin_q[5] = z
    pin_q[6] = w

    # Joint angles (same order)
    pin_q[7:19] = mj_qpos[14:26]

    # Velocities
    pin_v[0:3] = mj_qvel[6:9]        # torso linear velocity
    pin_v[3:6] = mj_qvel[9:12]       # torso angular velocity
    pin_v[6:18] = mj_qvel[12:24]     # joint velocities

    return pin_q, pin_v


def pinocchio_to_mujoco(
    pin_q: np.ndarray, pin_v: np.ndarray,
    struct_pos: np.ndarray = None,
    struct_quat: np.ndarray = None,
    struct_vel: np.ndarray = None,
    struct_angvel: np.ndarray = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Convert Pinocchio robot state to MuJoCo full state.

    Structure state must be provided separately (default: at origin, stationary).

    Parameters
    ----------
    pin_q : (19,) Pinocchio positions
    pin_v : (18,) Pinocchio velocities
    struct_pos : (3,) structure position (default: origin)
    struct_quat : (4,) structure quaternion MuJoCo convention w,x,y,z (default: identity)
    struct_vel : (3,) structure linear velocity (default: zeros)
    struct_angvel : (3,) structure angular velocity (default: zeros)

    Returns
    -------
    mj_qpos : (26,)
    mj_qvel : (24,)
    """
    mj_qpos = np.zeros(26)
    mj_qvel = np.zeros(24)

    # Structure (defaults)
    mj_qpos[0:3] = struct_pos if struct_pos is not None else np.zeros(3)
    mj_qpos[3:7] = struct_quat if struct_quat is not None else [1, 0, 0, 0]

    # Torso position
    mj_qpos[7:10] = pin_q[0:3]

    # Quaternion: Pinocchio (x,y,z,w) -> MuJoCo (w,x,y,z)
    x, y, z, w = pin_q[3:7]
    mj_qpos[10] = w
    mj_qpos[11] = x
    mj_qpos[12] = y
    mj_qpos[13] = z

    # Joints
    mj_qpos[14:26] = pin_q[7:19]

    # Structure velocity
    mj_qvel[0:3] = struct_vel if struct_vel is not None else np.zeros(3)
    mj_qvel[3:6] = struct_angvel if struct_angvel is not None else np.zeros(3)

    # Torso velocity
    mj_qvel[6:9]  = pin_v[0:3]
    mj_qvel[9:12] = pin_v[3:6]

    # Joint velocities
    mj_qvel[12:24] = pin_v[6:18]

    return mj_qpos, mj_qvel


def extract_structure_state(
    mj_qpos: np.ndarray, mj_qvel: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Extract structure state from MuJoCo full state.

    Returns (pos, quat_wxyz, vel, angvel)
    """
    return (
        mj_qpos[0:3].copy(),
        mj_qpos[3:7].copy(),
        mj_qvel[0:3].copy(),
        mj_qvel[3:6].copy(),
    )
