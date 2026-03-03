"""
Optimization solvers for hierarchical whole-body control of crawling space robots.

Generic solvers:
    HierarchicalQP  : Weighted and strict-priority QP (qpOASES backend)
    NMPCSolver       : Nonlinear MPC (CasADi/IPOPT backend)

Problem-specific wrappers:
    CentroidalNMPC   : Stage 1 — Momentum-feasible centroidal trajectory planning
    WholeBodyQP      : Stage 2 — High-rate whole-body tracking with dynamics constraints
    ContactConfig    : Contact phase management for locomotion
"""

from .hierarchical_qp import HierarchicalQP, Task, QPSolveInfo
from .nmpc_solver import NMPCSolver, NMPCSolveInfo
from .contact_phase import ContactPhase, ContactConfig, skew, compute_momentum_map
from .centroidal_nmpc import CentroidalNMPC, CentroidalNMPCConfig
from .wholebody_qp import WholeBodyQP, WholeBodyQPConfig

__all__ = [
    'HierarchicalQP', 'Task', 'QPSolveInfo',
    'NMPCSolver', 'NMPCSolveInfo',
    'CentroidalNMPC', 'CentroidalNMPCConfig',
    'WholeBodyQP', 'WholeBodyQPConfig',
    'ContactPhase', 'ContactConfig',
    'skew', 'compute_momentum_map',
]
