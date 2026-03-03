"""
ContactScheduler — Gait timing and contact management for VISPA crawling.

Manages the temporal sequence of locomotion phases:
    SS_A → DS → SS_B → DS → SS_A → ...

Each phase has a fixed duration and specific contact configuration.
The scheduler provides:
    - Current ContactConfig at any time t
    - Future contact sequence over NMPC horizon
    - Anchor SE3 poses from a predefined grasp plan

Phase timing (configurable):
    SS_A : 1.0 s  (single-support A, arm B swings)
    DS   : 0.5 s  (double-support transition)
    SS_B : 1.0 s  (single-support B, arm A swings)
    DS   : 0.5 s  (double-support transition)
    Total cycle: 3.0 s

Anchor grid convention:
    Anchors are numbered along the structure x-axis at spacing d_x,
    with ±d_y offset for arms A (+y) and B (-y):
        anchor_ia = (i * d_x,  d_y, 0)
        anchor_ib = (i * d_x, -d_y, 0)
"""

import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

import pinocchio as pin

from solvers.contact_phase import ContactPhase, ContactConfig


# ── Default anchor grid (matches dynamics.py DEFAULT_ANCHORS) ─────────────
DEFAULT_DX = 0.8   # spacing along x
DEFAULT_DY = 0.3   # ±y offset
DEFAULT_N_ANCHORS = 6


def make_anchor_grid(
    n: int = DEFAULT_N_ANCHORS,
    dx: float = DEFAULT_DX,
    dy: float = DEFAULT_DY,
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """Generate anchor position lists for arms A (+y) and B (-y).

    Returns
    -------
    anchors_a : list of ndarray (3,)
        Positions of arm-A anchors (ascending x).
    anchors_b : list of ndarray (3,)
        Positions of arm-B anchors (ascending x).
    """
    anchors_a = [np.array([i * dx,  dy, 0.0]) for i in range(n)]
    anchors_b = [np.array([i * dx, -dy, 0.0]) for i in range(n)]
    return anchors_a, anchors_b


def read_anchors_from_mujoco(mj_model, mj_data) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """Read anchor site positions from a MuJoCo model (world frame).

    Expects sites named 'anchor_1a'..'anchor_Na' and 'anchor_1b'..'anchor_Nb'.

    Parameters
    ----------
    mj_model : mujoco.MjModel
    mj_data : mujoco.MjData
        Must have been forwarded (mj_forward) at least once.

    Returns
    -------
    anchors_a, anchors_b : list of ndarray (3,)
        Anchor positions in MuJoCo world frame.
    """
    try:
        import mujoco
    except ImportError:
        raise ImportError("mujoco package required for read_anchors_from_mujoco")

    anchors_a, anchors_b = [], []
    for i in range(1, 20):  # generous upper bound
        name_a = f"anchor_{i}a"
        name_b = f"anchor_{i}b"
        try:
            sid_a = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, name_a)
            sid_b = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_SITE, name_b)
            if sid_a < 0 or sid_b < 0:
                break
            anchors_a.append(mj_data.site_xpos[sid_a].copy())
            anchors_b.append(mj_data.site_xpos[sid_b].copy())
        except Exception:
            break

    if not anchors_a:
        raise RuntimeError("No anchor sites found in MuJoCo model")

    return anchors_a, anchors_b


@dataclass
class GaitPhase:
    """One phase in the gait cycle."""
    phase: ContactPhase
    duration: float          # seconds
    anchor_a_idx: int        # index into anchor grid for arm A (docked anchor)
    anchor_b_idx: int        # index into anchor grid for arm B (docked anchor)
    # Swing info (only valid during single-support)
    swing_arm: str = ''      # 'a' or 'b' — which arm is swinging; '' during DS
    swing_from_idx: int = -1 # anchor index the free arm departs from
    swing_to_idx: int = -1   # anchor index the free arm travels to


@dataclass
class GaitPlan:
    """Ordered sequence of gait phases for a locomotion traversal.

    Built by ContactScheduler.plan_traversal().
    """
    phases: List[GaitPhase]
    t_start: List[float]     # cumulative start times
    t_end: List[float]       # cumulative end times
    total_duration: float

    def phase_at(self, t: float) -> Tuple[GaitPhase, int]:
        """Return (phase, index) active at time t."""
        t = np.clip(t, 0.0, self.total_duration - 1e-10)
        for i, (ts, te) in enumerate(zip(self.t_start, self.t_end)):
            if ts <= t < te:
                return self.phases[i], i
        # Edge case: exactly at end
        return self.phases[-1], len(self.phases) - 1


class ContactScheduler:
    """Generates and queries locomotion gait plans.

    Parameters
    ----------
    dt_ss : float
        Duration of single-support phases [s]. Default: 1.0.
    dt_ds : float
        Duration of double-support transitions [s]. Default: 0.5.
    anchors_a : list of ndarray (3,)
        Arm-A anchor positions (ascending along structure).
    anchors_b : list of ndarray (3,)
        Arm-B anchor positions.
    """

    def __init__(
        self,
        dt_ss: float = 1.0,
        dt_ds: float = 0.5,
        anchors_a: Optional[List[np.ndarray]] = None,
        anchors_b: Optional[List[np.ndarray]] = None,
    ):
        self.dt_ss = dt_ss
        self.dt_ds = dt_ds

        if anchors_a is None or anchors_b is None:
            anchors_a, anchors_b = make_anchor_grid()
        self.anchors_a = anchors_a
        self.anchors_b = anchors_b

        self._plan: Optional[GaitPlan] = None

    def plan_traversal(
        self,
        start_a: int = 0,
        start_b: int = 0,
        n_steps: int = 4,
    ) -> GaitPlan:
        """Plan a forward crawling traversal.

        The robot starts with both arms docked at (start_a, start_b).
        Each "step" moves one arm forward by one anchor:

            Step 1: DS → SS_A (B swings to start_b+1) → DS
            Step 2: DS → SS_B (A swings to start_a+1) → DS
            Step 3: DS → SS_A (B swings to start_b+2) → DS
            ...

        Parameters
        ----------
        start_a : int
            Starting anchor index for arm A.
        start_b : int
            Starting anchor index for arm B.
        n_steps : int
            Number of arm swing steps.

        Returns
        -------
        plan : GaitPlan
        """
        phases = []
        ia, ib = start_a, start_b  # current anchor indices

        # Initial double-support
        phases.append(GaitPhase(
            phase=ContactPhase.DOUBLE, duration=self.dt_ds,
            anchor_a_idx=ia, anchor_b_idx=ib))

        swing_b_next = True  # alternate: first swing B, then A

        for step in range(n_steps):
            if swing_b_next:
                # Single-support A (B swings forward)
                ib_next = ib + 1
                if ib_next >= len(self.anchors_b):
                    break  # no more anchors
                phases.append(GaitPhase(
                    phase=ContactPhase.SINGLE_A, duration=self.dt_ss,
                    anchor_a_idx=ia, anchor_b_idx=ib,
                    swing_arm='b', swing_from_idx=ib, swing_to_idx=ib_next))
                ib = ib_next
                # Transition DS (B now docked at new anchor)
                phases.append(GaitPhase(
                    phase=ContactPhase.DOUBLE, duration=self.dt_ds,
                    anchor_a_idx=ia, anchor_b_idx=ib))
            else:
                # Single-support B (A swings forward)
                ia_next = ia + 1
                if ia_next >= len(self.anchors_a):
                    break
                phases.append(GaitPhase(
                    phase=ContactPhase.SINGLE_B, duration=self.dt_ss,
                    anchor_a_idx=ia, anchor_b_idx=ib,
                    swing_arm='a', swing_from_idx=ia, swing_to_idx=ia_next))
                ia = ia_next
                # Transition DS (A now docked at new anchor)
                phases.append(GaitPhase(
                    phase=ContactPhase.DOUBLE, duration=self.dt_ds,
                    anchor_a_idx=ia, anchor_b_idx=ib))

            swing_b_next = not swing_b_next

        # Build timing
        t_start, t_end = [], []
        t = 0.0
        for gp in phases:
            t_start.append(t)
            t += gp.duration
            t_end.append(t)

        self._plan = GaitPlan(
            phases=phases,
            t_start=t_start,
            t_end=t_end,
            total_duration=t,
        )
        return self._plan

    @property
    def plan(self) -> GaitPlan:
        if self._plan is None:
            raise RuntimeError("Call plan_traversal() first.")
        return self._plan

    def contact_config_at(self, t: float) -> ContactConfig:
        """Return ContactConfig for simulation time t.

        Parameters
        ----------
        t : float
            Simulation time [s].

        Returns
        -------
        config : ContactConfig
        """
        gp, _ = self.plan.phase_at(t)
        r_a = self.anchors_a[gp.anchor_a_idx]
        r_b = self.anchors_b[gp.anchor_b_idx]
        return ContactConfig.from_phase(gp.phase, r_a, r_b)

    def contact_sequence_over_horizon(
        self, t: float, dt: float, N: int
    ) -> List[ContactConfig]:
        """Return the contact config at each NMPC prediction step.

        Parameters
        ----------
        t : float
            Current time.
        dt : float
            NMPC timestep.
        N : int
            Horizon length.

        Returns
        -------
        configs : list of ContactConfig, length N
        """
        return [self.contact_config_at(t + k * dt) for k in range(N)]

    def anchor_se3(self, arm: str, idx: int) -> pin.SE3:
        """Get SE3 anchor pose (identity orientation).

        Parameters
        ----------
        arm : 'a' or 'b'
        idx : int, anchor index

        Returns
        -------
        pose : pin.SE3
        """
        if arm.lower() == 'a':
            pos = self.anchors_a[idx]
        else:
            pos = self.anchors_b[idx]
        return pin.SE3(np.eye(3), pos)
