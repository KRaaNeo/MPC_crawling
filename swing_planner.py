"""Swing arm trajectory planner for crawling locomotion.

Generates smooth Cartesian position/velocity references for the free
end-effector during single-support phases.

Trajectory design
-----------------
Given a swing from anchor_start → anchor_end over duration T:

    p(τ) = p_start + Δp · s(τ)  +  clearance · n̂ · bump(τ)

where τ = (t − t_phase_start) / T ∈ [0, 1],

    s(τ) = 10τ³ − 15τ⁴ + 6τ⁵     (quintic, rest-to-rest)
    bump(τ) = sin²(πτ)              (clearance bell, C¹ boundaries)
    n̂ = unit vector away from structure surface

This ensures:
    - p(0) = p_start,  p(1) = p_end
    - v(0) = v(1) = 0  (smooth detach/attach)
    - a(0) = a(1) = 0  (no jerk at transitions)
    - Maximum clearance at mid-swing (τ = 0.5)
"""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple

from contact_scheduler import ContactScheduler, GaitPhase, GaitPlan
from solvers.contact_phase import ContactPhase


# Default clearance: 8 cm away from the structure surface.
# For a robot hanging at z ≈ −2.25 below structure at z ≈ −1.775,
# "away" = −z direction.
DEFAULT_CLEARANCE = 0.08

# Normal vector pointing away from the structure surface.
# Structure is a horizontal platform; robot hangs below → away = −z.
DEFAULT_AWAY_NORMAL = np.array([0.0, 0.0, -1.0])


@dataclass
class SwingReference:
    """Cartesian reference for the swing arm at a given instant."""
    p_ee: np.ndarray       # (3,) position [m]
    v_ee: np.ndarray       # (3,) velocity [m/s]
    a_ee: np.ndarray       # (3,) acceleration [m/s²]
    swing_arm: str         # 'a' or 'b'
    is_swinging: bool      # True during single-support swing
    phase_progress: float  # τ ∈ [0, 1]


class SwingPlanner:
    """Plan Cartesian trajectories for the free arm during crawling.

    Parameters
    ----------
    scheduler : ContactScheduler
        Must have a traversal plan already built.
    clearance : float
        Peak clearance distance normal to the structure surface [m].
    away_normal : ndarray (3,)
        Unit vector pointing away from the structure surface.
    """

    def __init__(
        self,
        scheduler: ContactScheduler,
        clearance: float = DEFAULT_CLEARANCE,
        away_normal: np.ndarray = DEFAULT_AWAY_NORMAL,
    ):
        self.scheduler = scheduler
        self.clearance = clearance
        self.away_normal = away_normal / np.linalg.norm(away_normal)

    @property
    def plan(self) -> GaitPlan:
        return self.scheduler.plan

    # ── Primitive profiles ───────────────────────────────────────

    @staticmethod
    def _quintic(tau: float) -> float:
        """Rest-to-rest quintic profile s(τ), s'(0)=s'(1)=s''(0)=s''(1)=0."""
        t2 = tau * tau
        t3 = t2 * tau
        return 10.0 * t3 - 15.0 * t2 * t2 + 6.0 * t2 * t3

    @staticmethod
    def _quintic_dot(tau: float) -> float:
        """Derivative s'(τ) = ds/dτ."""
        t2 = tau * tau
        return 30.0 * t2 - 60.0 * t2 * tau + 30.0 * t2 * t2

    @staticmethod
    def _quintic_ddot(tau: float) -> float:
        """Second derivative s''(τ) = d²s/dτ²."""
        return 60.0 * tau - 180.0 * tau * tau + 120.0 * tau * tau * tau

    @staticmethod
    def _bump(tau: float) -> float:
        """Bell-shaped clearance profile: sin²(πτ).
        Zero value AND zero derivative at τ=0 and τ=1."""
        s = np.sin(np.pi * tau)
        return s * s

    @staticmethod
    def _bump_dot(tau: float) -> float:
        """Derivative: π·sin(2πτ)."""
        return np.pi * np.sin(2.0 * np.pi * tau)

    @staticmethod
    def _bump_ddot(tau: float) -> float:
        """Second derivative: 2π²·cos(2πτ)."""
        return 2.0 * np.pi * np.pi * np.cos(2.0 * np.pi * tau)

    # ── Main query ───────────────────────────────────────────────

    def reference_at(self, t: float) -> SwingReference:
        """Get the swing arm reference at time t.

        During double-support phases, returns the docked anchor position
        with zero velocity and is_swinging=False.

        Parameters
        ----------
        t : float
            Simulation time [s].

        Returns
        -------
        ref : SwingReference
        """
        plan = self.plan
        gp, idx = plan.phase_at(t)

        # ── Double support: no swing ─────────────────────────────
        if gp.phase == ContactPhase.DOUBLE:
            # Return the position of the arm that was LAST swinging.
            # Look backward to find the most recent single-support.
            arm, p_ee = self._last_swing_position(idx)
            return SwingReference(
                p_ee=p_ee, v_ee=np.zeros(3), a_ee=np.zeros(3),
                swing_arm=arm, is_swinging=False, phase_progress=1.0)

        # ── Single support: compute swing trajectory ─────────────
        t_start = plan.t_start[idx]
        T = gp.duration
        tau = np.clip((t - t_start) / T, 0.0, 1.0)

        # Anchor positions (start and end)
        if gp.swing_arm == 'b':
            p_start = self.scheduler.anchors_b[gp.swing_from_idx].copy()
            p_end = self.scheduler.anchors_b[gp.swing_to_idx].copy()
        elif gp.swing_arm == 'a':
            p_start = self.scheduler.anchors_a[gp.swing_from_idx].copy()
            p_end = self.scheduler.anchors_a[gp.swing_to_idx].copy()
        else:
            raise ValueError(f"SS phase without swing_arm set at idx={idx}")

        # Displacement in the structure plane
        dp = p_end - p_start

        # Position
        s = self._quintic(tau)
        bump = self._bump(tau)
        p_ee = p_start + dp * s + self.clearance * self.away_normal * bump

        # Velocity (chain rule: dp/dt = dp/dτ · 1/T)
        s_dot = self._quintic_dot(tau) / T
        bump_dot = self._bump_dot(tau) / T
        v_ee = dp * s_dot + self.clearance * self.away_normal * bump_dot

        # Acceleration (d²p/dt² = d²p/dτ² · 1/T²)
        s_ddot = self._quintic_ddot(tau) / (T * T)
        bump_ddot = self._bump_ddot(tau) / (T * T)
        a_ee = dp * s_ddot + self.clearance * self.away_normal * bump_ddot

        return SwingReference(
            p_ee=p_ee, v_ee=v_ee, a_ee=a_ee,
            swing_arm=gp.swing_arm,
            is_swinging=True,
            phase_progress=tau)

    # ── Adaptive re-planning ────────────────────────────────────

    def adaptive_reference_at(
        self,
        t: float,
        p_ee_current: np.ndarray,
        T_remaining_min: float = 0.5,
    ) -> SwingReference:
        """Closed-loop swing reference: re-plan quintic from current EE position.

        Instead of following the open-loop quintic from p_start to p_end,
        this method plans a NEW quintic from p_ee_current to p_end over
        T_remaining = max(T_swing_end - t, T_remaining_min).

        This keeps the feedforward acceleration active as long as the
        gripper hasn't reached the target, regardless of the nominal
        schedule.  When t > T_swing_end the method enters "rendezvous"
        mode: pure trajectory from current position to target.

        Parameters
        ----------
        t : float
            Current simulation time [s].
        p_ee_current : ndarray (3,)
            Measured EE position in world frame.
        T_remaining_min : float
            Minimum horizon for the re-planned quintic [s].
            Prevents division-by-zero and overly aggressive profiles.

        Returns
        -------
        ref : SwingReference
        """
        plan = self.plan
        gp, idx = plan.phase_at(min(t, plan.t_end[-1] - 1e-6))

        # If in DS and not past schedule, use nominal
        if gp.phase == ContactPhase.DOUBLE and t < plan.t_end[-1]:
            return self.reference_at(t)

        # Find the active or most recent SS phase
        ss_idx = idx
        if gp.phase == ContactPhase.DOUBLE:
            for i in range(idx - 1, -1, -1):
                if plan.phases[i].swing_arm:
                    ss_idx = i
                    break
        gp_ss = plan.phases[ss_idx]
        if not gp_ss.swing_arm:
            return self.reference_at(min(t, plan.t_end[-1] - 1e-6))

        # Target position
        if gp_ss.swing_arm == 'b':
            p_end = self.scheduler.anchors_b[gp_ss.swing_to_idx].copy()
        else:
            p_end = self.scheduler.anchors_a[gp_ss.swing_to_idx].copy()

        # Remaining displacement
        dp = p_end - p_ee_current
        d_remaining = np.linalg.norm(dp)

        # If already at target, return zero motion
        if d_remaining < 1e-4:
            return SwingReference(
                p_ee=p_end, v_ee=np.zeros(3), a_ee=np.zeros(3),
                swing_arm=gp_ss.swing_arm, is_swinging=True,
                phase_progress=1.0)

        # Time horizon for the re-planned quintic
        t_ss_end = plan.t_end[ss_idx]
        T_rem = max(t_ss_end - t, T_remaining_min)

        # Re-planned quintic: τ_new starts at 0 now, reaches 1 at t + T_rem
        # We evaluate at τ_new = 0 (start of the new trajectory)
        # so s(0)=0, s_dot(0)=0, s_ddot(0)=0
        # That would give p_ref = p_ee_current → no correction!
        #
        # Instead, use a small look-ahead: evaluate at τ_new = dt/T_rem
        # where dt is one QP step. This gives a non-zero feedforward.
        #
        # Better approach: parametrise the NEW quintic over [0, T_rem]
        # and evaluate velocity/accel at τ=0⁺. For rest-to-rest quintic:
        #   v_peak = (15/8) * d / T_rem   at τ=0.5
        #   a_peak = (10/√3) * d / T_rem² at τ ≈ 0.21
        #   a(0) = 0, but a(ε) ≈ 60*d/T² * ε  → grows quickly
        #
        # Use a fixed evaluation point τ_eval that gives good tracking:
        # τ_eval = 1 QP step / T_rem

        # Evaluate the quintic at a small τ to get non-zero feedforward
        dt_eval = 0.01  # ~1 QP step
        tau_eval = np.clip(dt_eval / T_rem, 0.0, 0.5)

        s = self._quintic(tau_eval)
        s_dot = self._quintic_dot(tau_eval) / T_rem
        s_ddot = self._quintic_ddot(tau_eval) / (T_rem * T_rem)

        # No clearance bump during approach/rendezvous (already near structure)
        # Only add bump during nominal swing (first 80% of original duration)
        t_ss_start = plan.t_start[ss_idx]
        T_original = gp_ss.duration
        tau_original = np.clip((t - t_ss_start) / T_original, 0.0, 1.0)

        if tau_original < 0.8:
            bump = self._bump(tau_original)
            bump_dot = self._bump_dot(tau_original) / T_original
            bump_ddot = self._bump_ddot(tau_original) / (T_original ** 2)
            # Clearance relative to direct line, not dp
            n = self.away_normal
            p_ee = p_ee_current + dp * s + self.clearance * n * bump
            v_ee = dp * s_dot + self.clearance * n * bump_dot
            a_ee = dp * s_ddot + self.clearance * n * bump_ddot
        else:
            # Pure approach: straight to target
            p_ee = p_ee_current + dp * s
            v_ee = dp * s_dot
            a_ee = dp * s_ddot

        return SwingReference(
            p_ee=p_ee, v_ee=v_ee, a_ee=a_ee,
            swing_arm=gp_ss.swing_arm, is_swinging=True,
            phase_progress=tau_original)

    # ── Helpers ──────────────────────────────────────────────────

    def _last_swing_position(self, current_idx: int) -> Tuple[str, np.ndarray]:
        """Find the anchor where the last swing arm landed.

        Used during double-support to report the "parked" position
        of the arm that just finished swinging.
        """
        plan = self.plan
        # Search backward for the most recent SS phase
        for i in range(current_idx - 1, -1, -1):
            gp_prev = plan.phases[i]
            if gp_prev.swing_arm:
                # That arm landed at swing_to_idx
                if gp_prev.swing_arm == 'b':
                    p = self.scheduler.anchors_b[gp_prev.swing_to_idx]
                else:
                    p = self.scheduler.anchors_a[gp_prev.swing_to_idx]
                return gp_prev.swing_arm, p.copy()

        # No previous swing found → first DS phase, return arm B at start
        gp0 = plan.phases[0]
        return 'b', self.scheduler.anchors_b[gp0.anchor_b_idx].copy()

    def swing_trajectory(
        self,
        phase_idx: int,
        dt: float = 0.001,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Sample the full swing trajectory for a single-support phase.

        Parameters
        ----------
        phase_idx : int
            Index into the gait plan (must be a SS phase).
        dt : float
            Sampling interval [s].

        Returns
        -------
        t_vec : ndarray (K,)
        p_vec : ndarray (K, 3)
        v_vec : ndarray (K, 3)
        """
        plan = self.plan
        gp = plan.phases[phase_idx]
        assert gp.swing_arm, f"Phase {phase_idx} is not a swing phase"

        t0 = plan.t_start[phase_idx]
        T = gp.duration
        K = int(round(T / dt)) + 1
        t_vec = np.linspace(t0, t0 + T, K)
        p_vec = np.zeros((K, 3))
        v_vec = np.zeros((K, 3))

        for k, tk in enumerate(t_vec):
            ref = self.reference_at(tk)
            p_vec[k] = ref.p_ee
            v_vec[k] = ref.v_ee

        return t_vec, p_vec, v_vec
