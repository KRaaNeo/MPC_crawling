"""
HierarchicalQP - Generic hierarchical quadratic program solver.

Supports two formulation modes:
  - 'strict':   Cascade approach with null-space projection (exact priorities)
  - 'weighted': Single QP with large weight ratios (approximate priorities)

Standard QP form per task:
    minimize    (1/2) ||A_i @ x - b_i||^2_{W_i}
    subject to  C_eq @ x  = d_eq    (equality constraints)
                C_ineq @ x <= d_ineq (inequality constraints)
                lb <= x <= ub

Backend solvers (via CasADi conic interface):
    - 'qpoases' (default, active-set, warm-startable)
    - 'osqp'    (ADMM-based, good for sparse large-scale)

Usage:
    qp = HierarchicalQP(n_vars=36, method='weighted')
    qp.add_task(A1, b1, W1, priority=1)
    qp.add_task(A2, b2, W2, priority=2)
    qp.add_equality_constraint(C_eq, d_eq)
    qp.add_inequality_constraint(C_ineq, d_ineq)
    qp.set_bounds(lb, ub)
    x_opt, info = qp.solve()

References:
    - Escande et al., "Hierarchical quadratic programming: Fast online
      humanoid-robot motion generation", IJRR 2014.
    - Wensing et al., "Optimization-Based Control for Dynamic Legged
      Robots", IEEE T-RO 2024.

Author: Translated from MATLAB (I. Chelikh) to Python.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Tuple
import warnings
import logging

logger = logging.getLogger(__name__)


@dataclass
class Task:
    """Single task in the hierarchy: minimize (1/2)||A @ x - b||^2_W."""
    A: np.ndarray          # Task Jacobian (m x n_vars)
    b: np.ndarray          # Desired value (m,)
    W: np.ndarray          # Weight matrix (m x m) — symmetric positive semi-definite
    priority: int          # Priority level (1 = highest)


@dataclass
class QPSolveInfo:
    """Information returned by the QP solver."""
    method: str = ''
    success: bool = False
    exitflag: int = -1
    cost: float = np.inf
    lambda_eq: Optional[np.ndarray] = None
    lambda_ineq: Optional[np.ndarray] = None
    lambda_lb: Optional[np.ndarray] = None
    lambda_ub: Optional[np.ndarray] = None
    failed_priority: Optional[int] = None
    solve_time_ms: float = 0.0
    n_iter: int = 0


class HierarchicalQP:
    """Generic hierarchical quadratic program solver.

    Parameters
    ----------
    n_vars : int
        Number of decision variables.
    method : str
        'strict' (cascade/null-space) or 'weighted' (single QP with
        weight ratios). Default: 'strict'.
    weight_ratio : float
        Ratio between consecutive priority levels in weighted mode.
        Priority p gets effective weight W_task / weight_ratio^(p-1).
        Default: 1000.
    regularization : float
        Small Tikhonov regularization added to the Hessian for numerical
        conditioning. Default: 1e-6.
    solver : str
        QP backend solver. 'qpoases' (default) or 'osqp'.
    solver_opts : dict
        Additional solver-specific options passed to CasADi conic.
    """

    def __init__(
        self,
        n_vars: int,
        method: str = 'strict',
        weight_ratio: float = 1000.0,
        regularization: float = 1e-6,
        solver: str = 'qpoases',
        solver_opts: Optional[Dict[str, Any]] = None,
    ):
        self.n_vars = n_vars
        self.method = method
        self.weight_ratio = weight_ratio
        self.regularization = regularization
        self.solver_name = solver
        self.solver_opts = solver_opts or {}

        # Task list
        self._tasks: List[Task] = []

        # Constraints (accumulated)
        self._C_eq: Optional[np.ndarray] = None
        self._d_eq: Optional[np.ndarray] = None
        self._C_ineq: Optional[np.ndarray] = None
        self._d_ineq: Optional[np.ndarray] = None

        # Bounds
        self._lb = np.full(n_vars, -np.inf)
        self._ub = np.full(n_vars, np.inf)

        # Solver cache (for repeated calls with same dimensions)
        self._solver_cache: Dict[Tuple[int, int], Any] = {}

    # ------------------------------------------------------------------ #
    #  Public interface — Problem setup                                    #
    # ------------------------------------------------------------------ #

    def add_task(
        self,
        A: np.ndarray,
        b: np.ndarray,
        W,  # float, 1-D array, or 2-D array
        priority: int,
    ) -> None:
        """Add a task to the hierarchy.

        Parameters
        ----------
        A : ndarray, shape (m, n_vars)
            Task Jacobian.
        b : ndarray, shape (m,)
            Desired task value.
        W : float, ndarray (m,), or ndarray (m, m)
            Weight. Scalar → W * I_m.  1-D → diag(W).  2-D → used directly.
        priority : int
            Priority level (1 = highest).
        """
        A = np.atleast_2d(A)
        b = np.asarray(b).ravel()
        m = A.shape[0]

        assert A.shape[1] == self.n_vars, (
            f"Task A has {A.shape[1]} columns, expected {self.n_vars}"
        )
        assert b.shape[0] == m, (
            f"Task b has length {b.shape[0]}, expected {m}"
        )

        # Build weight matrix
        if np.isscalar(W):
            W_mat = float(W) * np.eye(m)
        elif np.ndim(W) == 1:
            W_mat = np.diag(np.asarray(W).ravel())
        else:
            W_mat = np.asarray(W)
            assert W_mat.shape == (m, m), f"W shape {W_mat.shape} != ({m},{m})"

        self._tasks.append(Task(A=A, b=b, W=W_mat, priority=priority))

    def add_equality_constraint(self, C: np.ndarray, d: np.ndarray) -> None:
        """Add equality constraint C @ x = d."""
        C = np.atleast_2d(C)
        d = np.asarray(d).ravel()
        if self._C_eq is None:
            self._C_eq = C
            self._d_eq = d
        else:
            self._C_eq = np.vstack([self._C_eq, C])
            self._d_eq = np.concatenate([self._d_eq, d])

    def add_inequality_constraint(self, C: np.ndarray, d: np.ndarray) -> None:
        """Add inequality constraint C @ x <= d."""
        C = np.atleast_2d(C)
        d = np.asarray(d).ravel()
        if self._C_ineq is None:
            self._C_ineq = C
            self._d_ineq = d
        else:
            self._C_ineq = np.vstack([self._C_ineq, C])
            self._d_ineq = np.concatenate([self._d_ineq, d])

    def set_bounds(self, lb: np.ndarray, ub: np.ndarray) -> None:
        """Set variable bounds lb <= x <= ub."""
        self._lb = np.asarray(lb).ravel()
        self._ub = np.asarray(ub).ravel()

    def clear_tasks(self) -> None:
        """Remove all tasks (keep constraints and bounds)."""
        self._tasks.clear()

    def clear_constraints(self) -> None:
        """Remove all equality and inequality constraints (keep tasks and bounds)."""
        self._C_eq = None
        self._d_eq = None
        self._C_ineq = None
        self._d_ineq = None

    def clear_all(self) -> None:
        """Remove all tasks, constraints, and reset bounds."""
        self.clear_tasks()
        self.clear_constraints()
        self._lb = np.full(self.n_vars, -np.inf)
        self._ub = np.full(self.n_vars, np.inf)

    # ------------------------------------------------------------------ #
    #  Public interface — Solve                                            #
    # ------------------------------------------------------------------ #

    def solve(self, x0: Optional[np.ndarray] = None) -> Tuple[np.ndarray, QPSolveInfo]:
        """Solve the hierarchical QP.

        Parameters
        ----------
        x0 : ndarray, shape (n_vars,), optional
            Warm-start initial guess.

        Returns
        -------
        x_opt : ndarray, shape (n_vars,)
            Optimal solution.
        info : QPSolveInfo
            Solver information.
        """
        import time
        t_start = time.perf_counter()

        if not self._tasks:
            raise ValueError("No tasks defined. Call add_task() first.")

        # Sort tasks by priority
        sorted_tasks = sorted(self._tasks, key=lambda t: t.priority)

        if self.method == 'strict':
            x_opt, info = self._solve_strict(sorted_tasks, x0)
        elif self.method == 'weighted':
            x_opt, info = self._solve_weighted(sorted_tasks, x0)
        else:
            raise ValueError(f"Unknown method: '{self.method}'")

        info.solve_time_ms = (time.perf_counter() - t_start) * 1000.0
        return x_opt, info

    # ------------------------------------------------------------------ #
    #  Private — Weighted mode                                             #
    # ------------------------------------------------------------------ #

    def _solve_weighted(
        self, sorted_tasks: List[Task], x0: Optional[np.ndarray]
    ) -> Tuple[np.ndarray, QPSolveInfo]:
        """Solve using weighted approach (single QP).

        Each priority level p receives effective weight 1/weight_ratio^(p-1).
        Priority 1 (highest) → full weight; priority p → weight / wr^(p-1).
        """
        n = self.n_vars
        H = np.zeros((n, n))
        g = np.zeros(n)

        for task in sorted_tasks:
            # Scale weight by priority: higher priority → less division
            w_p = self.weight_ratio ** (task.priority - 1)
            W_scaled = task.W / w_p

            # Accumulate: (1/2) x^T (A^T W A) x - (A^T W b)^T x
            AtW = task.A.T @ W_scaled
            H += AtW @ task.A
            g -= AtW @ task.b

        # Regularization + symmetry
        H += self.regularization * np.eye(n)
        H = 0.5 * (H + H.T)

        # Solve
        x_opt, qp_info = self._solve_qp_raw(
            H, g, self._C_eq, self._d_eq,
            self._C_ineq, self._d_ineq,
            self._lb, self._ub, x0
        )

        info = QPSolveInfo(
            method='weighted',
            success=qp_info['success'],
            exitflag=qp_info['exitflag'],
            cost=qp_info.get('cost', np.inf),
        )
        return x_opt, info

    # ------------------------------------------------------------------ #
    #  Private — Strict (cascade) mode                                     #
    # ------------------------------------------------------------------ #

    def _solve_strict(
        self, sorted_tasks: List[Task], x0: Optional[np.ndarray]
    ) -> Tuple[np.ndarray, QPSolveInfo]:
        """Solve using cascade approach with null-space projection.

        At each priority level p:
          1. Solve QP for tasks at level p, subject to constraints from
             levels 1..p-1 (enforced as equality constraints: A_i x = A_i x*).
          2. The optimal residual of level p tasks becomes a new equality
             constraint for level p+1.
        """
        n = self.n_vars
        x_opt = np.zeros(n)

        # Accumulated equality constraints (starts with user-provided)
        if self._C_eq is not None:
            C_eq_accum = self._C_eq.copy()
            d_eq_accum = self._d_eq.copy()
        else:
            C_eq_accum = None
            d_eq_accum = None

        # Unique sorted priority levels
        unique_priorities = sorted(set(t.priority for t in sorted_tasks))

        for p in unique_priorities:
            # Collect tasks at this priority
            H_p = np.zeros((n, n))
            g_p = np.zeros(n)

            tasks_at_p = [t for t in sorted_tasks if t.priority == p]
            for task in tasks_at_p:
                AtW = task.A.T @ task.W
                H_p += AtW @ task.A
                g_p -= AtW @ task.b

            # Regularization + symmetry
            H_p += self.regularization * np.eye(n)
            H_p = 0.5 * (H_p + H_p.T)

            # Solve QP at this priority level
            x_p, qp_info = self._solve_qp_raw(
                H_p, g_p, C_eq_accum, d_eq_accum,
                self._C_ineq, self._d_ineq,
                self._lb, self._ub, x0
            )

            if not qp_info['success']:
                warnings.warn(
                    f"Priority {p} failed (exitflag={qp_info['exitflag']})"
                )
                info = QPSolveInfo(
                    method='strict',
                    success=False,
                    exitflag=qp_info['exitflag'],
                    failed_priority=p,
                )
                return x_opt, info

            x_opt = x_p
            x0 = x_p  # Warm-start next level

            # Constrain this level's residual for next levels
            # A_i @ x = A_i @ x_opt  (freeze higher-priority task residuals)
            if p < unique_priorities[-1]:
                for task in tasks_at_p:
                    new_C = task.A
                    new_d = task.A @ x_opt
                    if C_eq_accum is None:
                        C_eq_accum = new_C
                        d_eq_accum = new_d
                    else:
                        C_eq_accum = np.vstack([C_eq_accum, new_C])
                        d_eq_accum = np.concatenate([d_eq_accum, new_d])

        info = QPSolveInfo(method='strict', success=True, exitflag=0)
        return x_opt, info

    # ------------------------------------------------------------------ #
    #  Private — Raw QP solve (backend dispatch)                           #
    # ------------------------------------------------------------------ #

    def _solve_qp_raw(
        self,
        H: np.ndarray,
        g: np.ndarray,
        C_eq: Optional[np.ndarray],
        d_eq: Optional[np.ndarray],
        C_ineq: Optional[np.ndarray],
        d_ineq: Optional[np.ndarray],
        lb: np.ndarray,
        ub: np.ndarray,
        x0: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Solve a single QP via CasADi conic interface.

        Standard form for CasADi conic:
            min  (1/2) x^T H x + g^T x
            s.t. lba <= A x <= uba
                 lb  <= x   <= ub

        We stack equality and inequality constraints into a single A matrix:
            A     = [C_eq   ]     lba = [d_eq  ]     uba = [d_eq    ]
                    [C_ineq ]           [-inf   ]           [d_ineq  ]
        """
        import casadi as ca

        n = H.shape[0]

        # --- Assemble constraint matrix ---
        A_rows = []
        lba_parts = []
        uba_parts = []

        if C_eq is not None and C_eq.size > 0:
            n_eq = C_eq.shape[0]
            A_rows.append(C_eq)
            lba_parts.append(d_eq)
            uba_parts.append(d_eq)

        if C_ineq is not None and C_ineq.size > 0:
            n_ineq = C_ineq.shape[0]
            A_rows.append(C_ineq)
            lba_parts.append(np.full(n_ineq, -1e16))  # CasADi doesn't like -inf
            uba_parts.append(d_ineq)

        if A_rows:
            A = np.vstack(A_rows)
            lba = np.concatenate(lba_parts)
            uba = np.concatenate(uba_parts)
        else:
            A = np.zeros((0, n))
            lba = np.zeros(0)
            uba = np.zeros(0)

        m = A.shape[0]

        # --- Get or create cached solver ---
        cache_key = (n, m)
        if cache_key not in self._solver_cache:
            qp_struct = {
                'h': ca.Sparsity.dense(n, n),
                'a': ca.Sparsity.dense(m, n) if m > 0 else ca.Sparsity(0, n),
            }
            opts = self._get_solver_options()
            try:
                solver = ca.conic('qp', self.solver_name, qp_struct, opts)
            except Exception:
                # Fallback to osqp if qpoases unavailable
                warnings.warn(
                    f"'{self.solver_name}' unavailable, falling back to 'osqp'"
                )
                self.solver_name = 'osqp'
                opts = self._get_solver_options()
                solver = ca.conic('qp', 'osqp', qp_struct, opts)
            self._solver_cache[cache_key] = solver

        solver = self._solver_cache[cache_key]

        # --- Build CasADi DM matrices ---
        H_dm = ca.DM(H)
        g_dm = ca.DM(g)
        A_dm = ca.DM(A) if m > 0 else ca.DM(0, n)
        lba_dm = ca.DM(lba) if m > 0 else ca.DM(0, 1)
        uba_dm = ca.DM(uba) if m > 0 else ca.DM(0, 1)
        lb_dm = ca.DM(lb)
        ub_dm = ca.DM(ub)

        # --- Solve ---
        solve_args = {
            'h': H_dm, 'g': g_dm,
            'a': A_dm, 'lba': lba_dm, 'uba': uba_dm,
            'lbx': lb_dm, 'ubx': ub_dm,
        }
        if x0 is not None:
            solve_args['x0'] = ca.DM(x0)

        try:
            sol = solver(**solve_args)
            x_opt = np.array(sol['x']).ravel()
            cost = float(sol['cost'])
            success = True
            exitflag = 0
        except RuntimeError as e:
            logger.warning(f"QP solver failed: {e}")
            x_opt = np.zeros(n)
            cost = np.inf
            success = False
            exitflag = -1

        return x_opt, {
            'success': success,
            'exitflag': exitflag,
            'cost': cost,
        }

    def _get_solver_options(self) -> Dict[str, Any]:
        """Default options for the QP backend."""
        opts: Dict[str, Any] = {'error_on_fail': False}

        if self.solver_name == 'qpoases':
            opts['printLevel'] = 'none'
            # qpOASES-specific options
            opts['nWSR'] = 500           # max working set recalculations
            opts['CPUtime'] = 0.005      # 5 ms budget

        elif self.solver_name == 'osqp':
            opts['osqp'] = {
                'verbose': False,
                'max_iter': 1000,
                'eps_abs': 1e-5,
                'eps_rel': 1e-5,
                'polish': True,
            }

        # Merge user options
        opts.update(self.solver_opts)
        return opts

    # ------------------------------------------------------------------ #
    #  Convenience: update tasks without rebuilding                        #
    # ------------------------------------------------------------------ #

    def update_task(self, index: int, b: Optional[np.ndarray] = None,
                    A: Optional[np.ndarray] = None,
                    W=None) -> None:
        """Update an existing task's data in-place (avoids list rebuild).

        Parameters
        ----------
        index : int
            Index into the task list (insertion order).
        b : ndarray, optional
            New desired value.
        A : ndarray, optional
            New Jacobian.
        W : optional
            New weight.
        """
        task = self._tasks[index]
        if b is not None:
            task.b = np.asarray(b).ravel()
        if A is not None:
            task.A = np.atleast_2d(A)
        if W is not None:
            m = task.A.shape[0]
            if np.isscalar(W):
                task.W = float(W) * np.eye(m)
            elif np.ndim(W) == 1:
                task.W = np.diag(np.asarray(W).ravel())
            else:
                task.W = np.asarray(W)

    def update_equality_constraint(self, C: np.ndarray, d: np.ndarray) -> None:
        """Replace all equality constraints."""
        self._C_eq = np.atleast_2d(C)
        self._d_eq = np.asarray(d).ravel()

    def update_inequality_constraint(self, C: np.ndarray, d: np.ndarray) -> None:
        """Replace all inequality constraints."""
        self._C_ineq = np.atleast_2d(C)
        self._d_ineq = np.asarray(d).ravel()

    # ------------------------------------------------------------------ #
    #  Properties for inspection                                           #
    # ------------------------------------------------------------------ #

    @property
    def n_tasks(self) -> int:
        return len(self._tasks)

    @property
    def tasks(self) -> List[Task]:
        return self._tasks

    def __repr__(self) -> str:
        return (
            f"HierarchicalQP(n_vars={self.n_vars}, method='{self.method}', "
            f"n_tasks={self.n_tasks}, solver='{self.solver_name}')"
        )
