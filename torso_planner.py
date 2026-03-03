"""
TorsoPlanner — Generates 6D torso + CoM reference trajectories.

Instead of tracking a static CoM, the torso advances during swing using
the stance arm as an inverted manipulator. This eliminates the CoM vs EE
conflict that prevents docking under tight torque limits.

The planner also derives a CoM reference from the torso trajectory:

    r_com(t) = p_torso(t) + R_torso(t) · δ_com(s(t))

where δ_com is the CoM offset in the torso body frame, interpolated
between start and end configurations to account for arm reconfiguration.
This CoM reference feeds the NMPC centroidal planner so it generates
momentum-feasible contact wrenches consistent with the actual motion.

Usage:
    planner = TorsoPlanner()
    planner.add_phase(t0, tf, p0, R0, pf, Rf,
                      delta_com_start, delta_com_end)
    ref = planner.reference_at(t)
    r_com, v_com = planner.com_reference_at(t)
"""

import numpy as np
import pinocchio as pin
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class TorsoReference:
    """Torso 6D reference at a given time instant."""
    p: np.ndarray          # (3,) position
    R: np.ndarray          # (3,3) rotation matrix
    v: np.ndarray          # (6,) twist [linear(3), angular(3)]
    a: np.ndarray          # (6,) acceleration [linear(3), angular(3)]


@dataclass
class ComReference:
    """CoM reference derived from torso trajectory."""
    r_com: np.ndarray      # (3,) position
    v_com: np.ndarray      # (3,) velocity


class TorsoPlanner:
    """Plan 6D torso + CoM trajectories synchronized with locomotion.

    The CoM reference is derived from the torso pose plus an offset
    δ_com(s) that is interpolated in the torso body frame between
    the start and end dock configurations.
    """

    def __init__(self):
        self._phases = []
        self._hold_p = None
        self._hold_R = None
        self._hold_com = None

    def set_hold(self, p: np.ndarray, R: np.ndarray,
                 r_com: Optional[np.ndarray] = None):
        """Set a static hold reference (DS or before swing starts)."""
        self._hold_p = p.copy()
        self._hold_R = R.copy()
        if r_com is not None:
            self._hold_com = r_com.copy()

    def add_phase(self, t_start: float, t_end: float,
                  p_start: np.ndarray, R_start: np.ndarray,
                  p_end: np.ndarray, R_end: np.ndarray,
                  delta_com_start: Optional[np.ndarray] = None,
                  delta_com_end: Optional[np.ndarray] = None):
        """Add a trajectory phase.

        Parameters
        ----------
        t_start, t_end : float
            Phase timing.
        p_start, p_end : ndarray (3,)
            Torso positions.
        R_start, R_end : ndarray (3,3)
            Torso orientations.
        delta_com_start : ndarray (3,), optional
            CoM offset in torso body frame at start config.
            Computed as R_start.T @ (r_com_start - p_start).
        delta_com_end : ndarray (3,), optional
            CoM offset in torso body frame at end config.
            Computed as R_end.T @ (r_com_end - p_end).
        """
        self._phases.append({
            't_start': t_start, 't_end': t_end,
            'p_start': p_start.copy(), 'R_start': R_start.copy(),
            'p_end': p_end.copy(), 'R_end': R_end.copy(),
            'duration': t_end - t_start,
            'delta_com_start': delta_com_start.copy() if delta_com_start is not None else None,
            'delta_com_end': delta_com_end.copy() if delta_com_end is not None else None,
        })

    def clear_phases(self):
        self._phases = []

    def reference_at(self, t: float) -> TorsoReference:
        """Compute 6D torso reference at time t."""
        for phase in self._phases:
            if phase['t_start'] - 1e-6 <= t <= phase['t_end'] + 1e-6:
                return self._interpolate_phase(t, phase)

        # Outside all phases: hold
        if self._hold_p is not None:
            return TorsoReference(
                p=self._hold_p.copy(), R=self._hold_R.copy(),
                v=np.zeros(6), a=np.zeros(6))

        if self._phases:
            last = self._phases[-1]
            if t > last['t_end']:
                return TorsoReference(
                    p=last['p_end'].copy(), R=last['R_end'].copy(),
                    v=np.zeros(6), a=np.zeros(6))
            else:
                first = self._phases[0]
                return TorsoReference(
                    p=first['p_start'].copy(), R=first['R_start'].copy(),
                    v=np.zeros(6), a=np.zeros(6))

        return TorsoReference(
            p=np.zeros(3), R=np.eye(3), v=np.zeros(6), a=np.zeros(6))

    def com_reference_at(self, t: float) -> ComReference:
        """Compute CoM reference derived from torso trajectory.

        r_com(t) = p_torso(t) + R_torso(t) · δ_com(s(t))

        where δ_com(s) = (1-s)·δ_start + s·δ_end is the interpolated
        CoM offset in torso body frame.

        The velocity is obtained by chain rule:
        v_com = v_torso_lin + ω_torso × (R·δ) + R·δ̇
        """
        for phase in self._phases:
            if phase['t_start'] - 1e-6 <= t <= phase['t_end'] + 1e-6:
                return self._interpolate_com(t, phase)

        # Outside phases: hold
        if self._hold_com is not None:
            return ComReference(r_com=self._hold_com.copy(), v_com=np.zeros(3))

        # Fallback: use torso position (no offset)
        tref = self.reference_at(t)
        return ComReference(r_com=tref.p.copy(), v_com=tref.v[:3].copy())

    def _quintic_params(self, t: float, phase: dict):
        """Compute quintic time scaling parameters."""
        T = phase['duration']
        tau = np.clip((t - phase['t_start']) / T, 0.0, 1.0)
        s = 10*tau**3 - 15*tau**4 + 6*tau**5
        ds = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
        dds = (60*tau - 180*tau**2 + 120*tau**3) / (T**2)
        return tau, s, ds, dds

    def _interpolate_phase(self, t: float, phase: dict) -> TorsoReference:
        """Interpolate with quintic time scaling."""
        _, s, ds, dds = self._quintic_params(t, phase)

        # Position
        dp = phase['p_end'] - phase['p_start']
        p = phase['p_start'] + s * dp
        v_lin = ds * dp
        a_lin = dds * dp

        # Orientation via log map
        R0 = phase['R_start']
        dR = R0.T @ phase['R_end']
        omega_total = pin.log3(dR)

        R = R0 @ pin.exp3(s * omega_total)
        omega_world = R @ (ds * omega_total)
        alpha_world = R @ (dds * omega_total)

        v = np.concatenate([v_lin, omega_world])
        a = np.concatenate([a_lin, alpha_world])

        return TorsoReference(p=p, R=R, v=v, a=a)

    def _interpolate_com(self, t: float, phase: dict) -> ComReference:
        """Derive CoM reference from torso trajectory + interpolated offset."""
        _, s, ds, _ = self._quintic_params(t, phase)

        d0 = phase['delta_com_start']
        d1 = phase['delta_com_end']
        if d0 is None or d1 is None:
            # Fallback: no offset data, return torso position
            tref = self._interpolate_phase(t, phase)
            return ComReference(r_com=tref.p.copy(), v_com=tref.v[:3].copy())

        # Interpolated offset in torso body frame
        delta = (1 - s) * d0 + s * d1       # δ(s)
        delta_dot = ds * (d1 - d0)           # dδ/dt = ds/dt · (d1-d0)

        # Torso pose at this instant
        tref = self._interpolate_phase(t, phase)
        R = tref.R
        omega = tref.v[3:6]  # angular velocity in world frame

        # r_com = p_torso + R · δ
        r_com = tref.p + R @ delta

        # v_com = v_torso_lin + ω × (R·δ) + R · δ̇
        Rd = R @ delta
        v_com = tref.v[:3] + np.cross(omega, Rd) + R @ delta_dot

        return ComReference(r_com=r_com, v_com=v_com)
