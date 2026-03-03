"""
Contact phase definitions for crawling multi-arm robot locomotion.

Defines the locomotion phases and provides utilities for managing
contact schedules during assembly operations.

Locomotion cycle: SS_A → DS → SS_B → DS → SS_A → ...
  SS_A: Single-support, arm A docked (arm B is swing leg)
  DS:   Double-support, both arms docked (transition phase)
  SS_B: Single-support, arm B docked (arm A is swing leg)
"""

from enum import Enum
from dataclasses import dataclass
from typing import Tuple, List
import numpy as np


class ContactPhase(Enum):
    """Locomotion contact phase."""
    SINGLE_A = 'single_a'   # Arm A docked only
    SINGLE_B = 'single_b'   # Arm B docked only
    DOUBLE = 'double'        # Both arms docked


@dataclass
class ContactConfig:
    """Contact configuration for a given phase.

    Attributes
    ----------
    phase : ContactPhase
        Current locomotion phase.
    nc : int
        Number of active contacts (1 or 2).
    active_contacts : Tuple[bool, bool]
        (arm_A_active, arm_B_active).
    r_contact_A : np.ndarray, shape (3,)
        Position of contact point A in spacecraft frame R_s.
    r_contact_B : np.ndarray, shape (3,)
        Position of contact point B in spacecraft frame R_s.
    """
    phase: ContactPhase
    nc: int
    active_contacts: Tuple[bool, bool]
    r_contact_A: np.ndarray
    r_contact_B: np.ndarray

    @classmethod
    def from_phase(
        cls,
        phase: ContactPhase,
        r_contact_A: np.ndarray,
        r_contact_B: np.ndarray,
    ) -> 'ContactConfig':
        """Create ContactConfig from phase enum."""
        if phase == ContactPhase.SINGLE_A:
            return cls(phase=phase, nc=1, active_contacts=(True, False),
                       r_contact_A=r_contact_A, r_contact_B=r_contact_B)
        elif phase == ContactPhase.SINGLE_B:
            return cls(phase=phase, nc=1, active_contacts=(False, True),
                       r_contact_A=r_contact_A, r_contact_B=r_contact_B)
        elif phase == ContactPhase.DOUBLE:
            return cls(phase=phase, nc=2, active_contacts=(True, True),
                       r_contact_A=r_contact_A, r_contact_B=r_contact_B)
        else:
            raise ValueError(f"Unknown phase: {phase}")

    @property
    def active_contact_positions(self) -> List[np.ndarray]:
        """Return list of active contact positions."""
        positions = []
        if self.active_contacts[0]:
            positions.append(self.r_contact_A)
        if self.active_contacts[1]:
            positions.append(self.r_contact_B)
        return positions


def skew(v: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix [v]_x such that [v]_x @ w = v × w.

    Parameters
    ----------
    v : ndarray, shape (3,)
        3D vector.

    Returns
    -------
    S : ndarray, shape (3, 3)
        Skew-symmetric matrix.
    """
    v = np.asarray(v).ravel()
    return np.array([
        [0,    -v[2],  v[1]],
        [v[2],  0,    -v[0]],
        [-v[1], v[0],  0   ],
    ])


def compute_momentum_map(
    r_com: np.ndarray,
    contact_config: ContactConfig,
) -> np.ndarray:
    """Compute the momentum map matrix M_λ.

    Maps contact wrenches λ = [f1, τ1, f2, τ2] to angular momentum rate:
        L̇_com = M_λ @ λ

    For each active contact j:
        L̇_j = (r_Cj - r_com) × f_j + τ_j = [S(r_Cj - r_com), I_3] @ [f_j; τ_j]

    Parameters
    ----------
    r_com : ndarray, shape (3,)
        Robot center of mass position in R_s.
    contact_config : ContactConfig
        Current contact configuration.

    Returns
    -------
    M_lambda : ndarray, shape (3, 12)
        Momentum map matrix. Columns for inactive contacts are zero.
    """
    M = np.zeros((3, 12))

    if contact_config.active_contacts[0]:
        lever_A = contact_config.r_contact_A - r_com
        M[:, 0:3] = skew(lever_A)     # f1 contribution
        M[:, 3:6] = np.eye(3)         # τ1 contribution

    if contact_config.active_contacts[1]:
        lever_B = contact_config.r_contact_B - r_com
        M[:, 6:9] = skew(lever_B)     # f2 contribution
        M[:, 9:12] = np.eye(3)        # τ2 contribution

    return M
