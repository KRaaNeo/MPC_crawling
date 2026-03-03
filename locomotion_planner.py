"""
LocomotionPlanner — CoM reference trajectory generation for VISPA.

Given a GaitPlan (from ContactScheduler), generates:
    - r_com_ref(t): CoM position reference (smooth interpolation)
    - v_com_ref(t): CoM velocity reference (derivative of r_com_ref)

Strategy:
    During each phase, the CoM reference moves from the current static
    equilibrium position to the next one. Static equilibrium for the CoM
    is above the support polygon centroid:
        - DS:   midpoint of active anchors
        - SS_A: above anchor_a (shifted toward structure center)
        - SS_B: above anchor_b (shifted toward structure center)

    Quintic (5th-order) polynomial interpolation ensures smooth
    acceleration profiles at phase transitions (zero velocity and
    acceleration at boundaries).
"""

import numpy as np
from typing import Tuple, List, Optional

from contact_scheduler import ContactScheduler, GaitPlan, GaitPhase
from solvers.contact_phase import ContactPhase


# Height of CoM below the structure plane (robot hangs underneath)
DEFAULT_COM_HEIGHT = -0.47


class LocomotionPlanner:
    """Generates smooth CoM reference trajectories.

    Parameters
    ----------
    scheduler : ContactScheduler
        Provides the gait plan and anchor positions.
    com_height : float
        Desired CoM height above structure plane [m]. Default: 0.25.
    y_offset_factor : float
        During SS, how much to shift CoM toward structure centerline.
        0 = stay above anchor, 1 = move to centerline. Default: 0.3.
    """

    def __init__(
        self,
        scheduler: ContactScheduler,
        com_height: float = DEFAULT_COM_HEIGHT,
        y_offset_factor: float = 0.3,
        arm_mass: float = None,
        total_mass: float = None,
    ):
        self.scheduler = scheduler
        self.com_height = com_height
        self.y_offset = y_offset_factor
        self.arm_mass = arm_mass
        self.total_mass = total_mass

        # Precompute waypoints from the gait plan
        self._waypoints: List[np.ndarray] = []
        self._build_waypoints()

    def calibrate_from_config(self, r_com_init: np.ndarray):
        """Auto-calibrate CoM height from an actual docked configuration.

        Parameters
        ----------
        r_com_init : ndarray (3,)
            CoM position at the initial docked pose.
        """
        self.com_height = r_com_init[2]
        self._build_waypoints()

    def _build_waypoints(self):
        """Compute one CoM waypoint per phase from the gait plan."""
        plan = self.scheduler.plan
        self._waypoints = []

        for gp in plan.phases:
            r_a = self.scheduler.anchors_a[gp.anchor_a_idx]
            r_b = self.scheduler.anchors_b[gp.anchor_b_idx]
            self._waypoints.append(self._equilibrium_com(gp.phase, r_a, r_b))

    def _equilibrium_com(
        self, phase: ContactPhase,
        r_a: np.ndarray, r_b: np.ndarray,
    ) -> np.ndarray:
        """Static equilibrium CoM position for a phase.

        Parameters
        ----------
        phase : ContactPhase
        r_a, r_b : ndarray (3,)
            Anchor positions.

        Returns
        -------
        r_com : ndarray (3,)
        """
        if phase == ContactPhase.DOUBLE:
            # Midpoint of both anchors
            xy = 0.5 * (r_a[:2] + r_b[:2])
        elif phase == ContactPhase.SINGLE_A:
            # Above anchor A, shifted toward centerline
            xy = r_a[:2].copy()
            xy[1] = r_a[1] * (1.0 - self.y_offset)
        elif phase == ContactPhase.SINGLE_B:
            xy = r_b[:2].copy()
            xy[1] = r_b[1] * (1.0 - self.y_offset)
        else:
            xy = 0.5 * (r_a[:2] + r_b[:2])

        return np.array([xy[0], xy[1], self.com_height])

    def reference_at(self, t: float) -> Tuple[np.ndarray, np.ndarray]:
        """Compute (r_com_ref, v_com_ref) at time t.

        Uses quintic interpolation between phase waypoints for smooth
        position and velocity references.

        Parameters
        ----------
        t : float
            Current simulation time [s].

        Returns
        -------
        r_com_ref : ndarray (3,)
        v_com_ref : ndarray (3,)
        """
        plan = self.scheduler.plan
        _, phase_idx = plan.phase_at(t)

        t_start = plan.t_start[phase_idx]
        t_end = plan.t_end[phase_idx]
        duration = t_end - t_start

        # Waypoints: current phase target and previous phase target
        r_end = self._waypoints[phase_idx]
        if phase_idx > 0:
            r_start = self._waypoints[phase_idx - 1]
        else:
            r_start = r_end  # first phase: already at target

        # Normalized phase time s ∈ [0, 1]
        s = np.clip((t - t_start) / max(duration, 1e-6), 0.0, 1.0)

        # Quintic polynomial: s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
        # Ensures: s(0)=0, s(1)=1, ṡ(0)=ṡ(1)=0, s̈(0)=s̈(1)=0
        sigma = 10 * s**3 - 15 * s**4 + 6 * s**5
        dsigma_ds = 30 * s**2 - 60 * s**3 + 30 * s**4
        dsigma_dt = dsigma_ds / max(duration, 1e-6)

        # Interpolate
        r_com_ref = r_start + sigma * (r_end - r_start)
        v_com_ref = dsigma_dt * (r_end - r_start)

        # ── Swing arm CoM pre-compensation ───────────────────────
        # During SS, the swing arm shifts the CoM by (m_arm/m_total)·Δp_arm.
        # Instead of fighting this, we include it in the reference.
        gp = plan.phases[phase_idx]
        if (self.arm_mass is not None and self.total_mass is not None
                and gp.swing_arm):
            ratio = self.arm_mass / self.total_mass
            if gp.swing_arm == 'b':
                p_from = self.scheduler.anchors_b[gp.swing_from_idx]
                p_to = self.scheduler.anchors_b[gp.swing_to_idx]
            else:
                p_from = self.scheduler.anchors_a[gp.swing_from_idx]
                p_to = self.scheduler.anchors_a[gp.swing_to_idx]
            dp_arm = p_to - p_from
            # Same quintic profile as the swing itself
            r_com_ref += ratio * sigma * dp_arm
            v_com_ref += ratio * dsigma_dt * dp_arm

        return r_com_ref, v_com_ref

    def full_trajectory(self, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Generate the full reference trajectory at uniform timestep.

        Parameters
        ----------
        dt : float
            Sampling period [s].

        Returns
        -------
        t_vec : ndarray (M,)
            Time vector.
        r_com_ref : ndarray (M, 3)
            CoM position references.
        v_com_ref : ndarray (M, 3)
            CoM velocity references.
        """
        T = self.scheduler.plan.total_duration
        N = int(T / dt)
        t_vec = np.arange(N) * dt
        r_refs = np.zeros((N, 3))
        v_refs = np.zeros((N, 3))

        for i, t in enumerate(t_vec):
            r_refs[i], v_refs[i] = self.reference_at(t)

        return t_vec, r_refs, v_refs
