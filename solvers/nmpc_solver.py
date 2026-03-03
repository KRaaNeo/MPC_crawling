"""
NMPCSolver - Generic Nonlinear Model Predictive Control solver with CasADi.

Solves the discrete-time NMPC problem:

    min   sum_{i=0}^{N-1} L(x_i, u_i, p) + Lf(x_N, p)
    s.t.  x_{i+1} = f(x_i, u_i, p)        (dynamics)
          x_0 = x_init                      (initial condition)
          g(x_i, u_i, p) <= 0               (path constraints)
          x_min <= x_i <= x_max             (state bounds)
          u_min <= u_i <= u_max             (control bounds)

Supports:
    - Discrete or continuous dynamics (with RK4 integration)
    - General nonlinear path constraints (including SOC)
    - Terminal cost and terminal constraints
    - Warm-starting via previous solution shift
    - IPOPT (default, nonlinear) and qpOASES (for QP subproblems)

Usage:
    nmpc = NMPCSolver(nx=9, nu=12, N=20, dt=0.05)
    nmpc.set_continuous_dynamics(lambda x, u, p: f_ode(x, u, p))
    nmpc.set_stage_cost(lambda x, u, p: (x - x_ref).T @ Q @ (x - x_ref) + u.T @ R @ u)
    nmpc.set_terminal_cost(lambda x, p: (x - x_ref).T @ Qf @ (x - x_ref))
    nmpc.set_path_constraints(lambda x, u, p: ca.vertcat(u[:3].T @ u[:3] - fmax**2), ng=1)
    nmpc.set_state_bounds(x_min, x_max)
    nmpc.set_control_bounds(u_min, u_max)
    nmpc.build()
    x_opt, u_opt, info = nmpc.solve(x0)

References:
    - Chelikh et al., "Modeling and Whole-Body motion generation of a
      crawling Multi-arm Orbital Robot servicer under momentum saturation
      constraints", IEEE Access 2024.
    - Rawlings, Mayne & Diehl, "Model Predictive Control: Theory,
      Computation, and Design", 2nd ed., 2017.

Author: Translated from MATLAB (I. Chelikh) to Python.
"""

import numpy as np
import casadi as ca
from dataclasses import dataclass
from typing import Optional, Callable, Dict, Any, Tuple, List
import warnings
import logging
import time

logger = logging.getLogger(__name__)


@dataclass
class NMPCSolveInfo:
    """Information returned by the NMPC solver."""
    cost: float = np.inf
    success: bool = False
    status: str = ''
    iterations: int = 0
    solve_time_ms: float = 0.0
    solver_stats: Optional[Dict] = None


class NMPCSolver:
    """Generic Nonlinear MPC solver using CasADi.

    Parameters
    ----------
    nx : int
        Number of states.
    nu : int
        Number of controls.
    N : int
        Prediction horizon length.
    dt : float
        Discretization time step [s].
    solver_name : str
        NLP solver: 'ipopt' (default) or 'sqpmethod'.
    """

    def __init__(
        self,
        nx: int,
        nu: int,
        N: int,
        dt: float,
        solver_name: str = 'ipopt',
    ):
        self.nx = nx
        self.nu = nu
        self.N = N
        self.dt = dt
        self.solver_name = solver_name

        # CasADi symbolic variables
        self.x_sym = ca.SX.sym('x', nx)
        self.u_sym = ca.SX.sym('u', nu)
        self.p_sym = ca.SX.sym('p', 0)  # empty by default

        # System functions (set by user)
        self._f_dynamics: Optional[ca.SX] = None   # x_next = f(x, u, p)
        self._stage_cost: Optional[ca.SX] = None    # L(x, u, p)
        self._terminal_cost: Optional[ca.SX] = None # Lf(x, p)
        self._g_path: Optional[ca.SX] = None         # g(x, u, p) <= 0
        self._ng_path: int = 0
        self._g_terminal: Optional[ca.SX] = None     # g_f(x, p) <= 0
        self._ng_terminal: int = 0

        # Bounds (default: unbounded)
        self.x_min = np.full(nx, -np.inf)
        self.x_max = np.full(nx, np.inf)
        self.u_min = np.full(nu, -np.inf)
        self.u_max = np.full(nu, np.inf)

        # Solver instance (built by build())
        self._solver = None
        self._lbw: Optional[np.ndarray] = None
        self._ubw: Optional[np.ndarray] = None
        self._lbg: Optional[np.ndarray] = None
        self._ubg: Optional[np.ndarray] = None
        self._np_total: int = 0  # nx + np (parameter vector length)

        # Warm-start storage
        self._w0_prev: Optional[np.ndarray] = None
        self._lam_g0_prev: Optional[np.ndarray] = None
        self._lam_x0_prev: Optional[np.ndarray] = None

    # ------------------------------------------------------------------ #
    #  System definition                                                   #
    # ------------------------------------------------------------------ #

    def set_dynamics(self, dynamics_func: Callable) -> None:
        """Set discrete dynamics x_{k+1} = f(x_k, u_k, p).

        Parameters
        ----------
        dynamics_func : callable
            (x, u, p) -> x_next, where x, u, p are CasADi SX symbols.
        """
        self._f_dynamics = dynamics_func(self.x_sym, self.u_sym, self.p_sym)

    def set_continuous_dynamics(self, ode_func: Callable) -> None:
        """Set continuous dynamics dx/dt = f(x, u, p), integrated with RK4.

        Parameters
        ----------
        ode_func : callable
            (x, u, p) -> dx/dt, where x, u, p are CasADi SX symbols.
        """
        x = self.x_sym
        u = self.u_sym
        p = self.p_sym
        dt = self.dt

        k1 = ode_func(x, u, p)
        k2 = ode_func(x + dt / 2 * k1, u, p)
        k3 = ode_func(x + dt / 2 * k2, u, p)
        k4 = ode_func(x + dt * k3, u, p)

        self._f_dynamics = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    def set_stage_cost(self, cost_func: Callable) -> None:
        """Set stage cost L(x, u, p).

        Parameters
        ----------
        cost_func : callable
            (x, u, p) -> scalar CasADi expression.
        """
        self._stage_cost = cost_func(self.x_sym, self.u_sym, self.p_sym)

    def set_terminal_cost(self, cost_func: Callable) -> None:
        """Set terminal cost Lf(x_N, p).

        Parameters
        ----------
        cost_func : callable
            (x, p) -> scalar CasADi expression.
        """
        self._terminal_cost = cost_func(self.x_sym, self.p_sym)

    def set_path_constraints(self, constraint_func: Callable, ng: int) -> None:
        """Set path constraints g(x, u, p) <= 0, applied at each horizon step.

        Parameters
        ----------
        constraint_func : callable
            (x, u, p) -> vector of ng CasADi expressions.
        ng : int
            Number of path constraints.

        Notes
        -----
        For SOC constraints like ||f||_2 <= f_max, formulate as:
            g = f[0]**2 + f[1]**2 + f[2]**2 - f_max**2  <= 0
        IPOPT handles these nonlinear constraints natively.
        """
        self._g_path = constraint_func(self.x_sym, self.u_sym, self.p_sym)
        self._ng_path = ng

    def set_terminal_constraints(self, constraint_func: Callable, ng: int) -> None:
        """Set terminal constraints g_f(x_N, p) <= 0.

        Parameters
        ----------
        constraint_func : callable
            (x, p) -> vector of ng CasADi expressions.
        ng : int
            Number of terminal constraints.
        """
        self._g_terminal = constraint_func(self.x_sym, self.p_sym)
        self._ng_terminal = ng

    def set_state_bounds(self, x_min: np.ndarray, x_max: np.ndarray) -> None:
        """Set state bounds x_min <= x <= x_max."""
        self.x_min = np.asarray(x_min).ravel()
        self.x_max = np.asarray(x_max).ravel()

    def set_control_bounds(self, u_min: np.ndarray, u_max: np.ndarray) -> None:
        """Set control bounds u_min <= u <= u_max."""
        self.u_min = np.asarray(u_min).ravel()
        self.u_max = np.asarray(u_max).ravel()

    def set_parameters(self, np_: int) -> None:
        """Redefine the parameter vector dimension.

        Must be called BEFORE build() and BEFORE set_*_cost/dynamics
        if parameters are used.

        Parameters
        ----------
        np_ : int
            Number of additional parameters (beyond initial state x0).
        """
        self.p_sym = ca.SX.sym('p', np_)

    # ------------------------------------------------------------------ #
    #  Build the NLP                                                       #
    # ------------------------------------------------------------------ #

    def build(self, solver_opts: Optional[Dict[str, Any]] = None) -> None:
        """Construct the optimization problem and instantiate the solver.

        Parameters
        ----------
        solver_opts : dict, optional
            Solver-specific options (merged with defaults).
        """
        if self._f_dynamics is None:
            raise ValueError("Dynamics not defined. Call set_dynamics() or "
                             "set_continuous_dynamics() first.")
        if self._stage_cost is None:
            raise ValueError("Stage cost not defined. Call set_stage_cost().")

        if solver_opts is None:
            solver_opts = {}

        # --- CasADi functions ---
        f_eval = ca.Function('f', [self.x_sym, self.u_sym, self.p_sym],
                             [self._f_dynamics], ['x', 'u', 'p'], ['x_next'])
        L_eval = ca.Function('L', [self.x_sym, self.u_sym, self.p_sym],
                             [self._stage_cost], ['x', 'u', 'p'], ['cost'])

        Lf_eval = None
        if self._terminal_cost is not None:
            Lf_eval = ca.Function('Lf', [self.x_sym, self.p_sym],
                                  [self._terminal_cost], ['x', 'p'], ['cost'])

        g_eval = None
        if self._ng_path > 0:
            g_eval = ca.Function('g', [self.x_sym, self.u_sym, self.p_sym],
                                 [self._g_path], ['x', 'u', 'p'], ['g'])

        gf_eval = None
        if self._ng_terminal > 0:
            gf_eval = ca.Function('gf', [self.x_sym, self.p_sym],
                                  [self._g_terminal], ['x', 'p'], ['g'])

        # --- NLP construction (multiple-shooting) ---
        w = []        # decision variables
        lbw = []      # lower bounds on w
        ubw = []      # upper bounds on w
        w0 = []       # initial guess

        g_list = []   # constraints
        lbg = []      # lower bounds on g
        ubg = []      # upper bounds on g

        J = 0         # cost

        # Parameter vector: [x0; p_user]
        np_user = self.p_sym.shape[0]
        self._np_total = self.nx + np_user
        P = ca.SX.sym('P', self._np_total)
        x0_param = P[:self.nx]
        p_param = P[self.nx:] if np_user > 0 else self.p_sym  # keep empty sym if no params

        # Storage for symbolic variables (for structure inspection)
        X_sym = []
        U_sym = []

        # --- Initial state (k=0) ---
        Xk = ca.SX.sym('X0', self.nx)
        X_sym.append(Xk)
        w.append(Xk)
        lbw.append(self.x_min)
        ubw.append(self.x_max)
        w0.append(np.zeros(self.nx))

        # Pin initial state
        g_list.append(Xk - x0_param)
        lbg.append(np.zeros(self.nx))
        ubg.append(np.zeros(self.nx))

        # --- Horizon loop ---
        for k in range(self.N):
            # Control at step k
            Uk = ca.SX.sym(f'U{k}', self.nu)
            U_sym.append(Uk)
            w.append(Uk)
            lbw.append(self.u_min)
            ubw.append(self.u_max)
            w0.append(np.zeros(self.nu))

            # Stage cost
            J += L_eval(x=Xk, u=Uk, p=p_param)['cost']

            # Path constraints at step k
            if self._ng_path > 0:
                g_val = g_eval(x=Xk, u=Uk, p=p_param)['g']
                g_list.append(g_val)
                lbg.append(np.full(self._ng_path, -np.inf))
                ubg.append(np.zeros(self._ng_path))

            # Next state
            Xk_next = ca.SX.sym(f'X{k + 1}', self.nx)
            X_sym.append(Xk_next)
            w.append(Xk_next)
            lbw.append(self.x_min)
            ubw.append(self.x_max)
            w0.append(np.zeros(self.nx))

            # Dynamics constraint: x_{k+1} = f(x_k, u_k, p)
            x_pred = f_eval(x=Xk, u=Uk, p=p_param)['x_next']
            g_list.append(Xk_next - x_pred)
            lbg.append(np.zeros(self.nx))
            ubg.append(np.zeros(self.nx))

            Xk = Xk_next

        # --- Terminal cost ---
        if Lf_eval is not None:
            J += Lf_eval(x=Xk, p=p_param)['cost']

        # --- Terminal constraints ---
        if self._ng_terminal > 0:
            gf_val = gf_eval(x=Xk, p=p_param)['g']
            g_list.append(gf_val)
            lbg.append(np.full(self._ng_terminal, -np.inf))
            ubg.append(np.zeros(self._ng_terminal))

        # --- Concatenate ---
        w_vec = ca.vertcat(*w)
        g_vec = ca.vertcat(*g_list)

        self._lbw = np.concatenate(lbw)
        self._ubw = np.concatenate(ubw)
        self._lbg = np.concatenate(lbg)
        self._ubg = np.concatenate(ubg)
        self._w0_default = np.concatenate(w0)

        # --- Create NLP ---
        nlp = {'f': J, 'x': w_vec, 'g': g_vec, 'p': P}

        opts = self._get_default_solver_options()
        opts = _merge_dicts(opts, solver_opts)

        self._solver = ca.nlpsol('nmpc', self.solver_name, nlp, opts)

        logger.info(
            f"NMPC built: {w_vec.shape[0]} variables, "
            f"{g_vec.shape[0]} constraints, solver={self.solver_name}"
        )

    # ------------------------------------------------------------------ #
    #  Solve                                                               #
    # ------------------------------------------------------------------ #

    def solve(
        self,
        x0: np.ndarray,
        u_guess: Optional[np.ndarray] = None,
        x_guess: Optional[np.ndarray] = None,
        params: Optional[np.ndarray] = None,
        warm_start: bool = True,
    ) -> Tuple[np.ndarray, np.ndarray, NMPCSolveInfo]:
        """Solve the NMPC problem.

        Parameters
        ----------
        x0 : ndarray, shape (nx,)
            Current measured/estimated state.
        u_guess : ndarray, shape (nu, N), optional
            Initial guess for controls. If None, uses zero or warm-start.
        x_guess : ndarray, shape (nx, N+1), optional
            Initial guess for states. If None, uses x0 repeated or warm-start.
        params : ndarray, shape (np,), optional
            Additional parameters.
        warm_start : bool
            If True, use the shifted previous solution as initial guess.

        Returns
        -------
        x_opt : ndarray, shape (nx, N+1)
            Optimal state trajectory.
        u_opt : ndarray, shape (nu, N)
            Optimal control trajectory.
        info : NMPCSolveInfo
            Solver information.
        """
        if self._solver is None:
            raise RuntimeError("Solver not built. Call build() first.")

        t_start = time.perf_counter()

        x0 = np.asarray(x0).ravel()
        if params is None:
            params = np.array([])
        else:
            params = np.asarray(params).ravel()

        P = np.concatenate([x0, params])

        # --- Initial guess ---
        w0 = self._build_initial_guess(x0, u_guess, x_guess, warm_start)

        # --- Solve NLP ---
        solve_args = {
            'x0': ca.DM(w0),
            'lbx': ca.DM(self._lbw),
            'ubx': ca.DM(self._ubw),
            'lbg': ca.DM(self._lbg),
            'ubg': ca.DM(self._ubg),
            'p': ca.DM(P),
        }

        # Warm-start dual variables
        if warm_start and self._lam_g0_prev is not None:
            solve_args['lam_g0'] = ca.DM(self._lam_g0_prev)
            solve_args['lam_x0'] = ca.DM(self._lam_x0_prev)

        sol = self._solver(**solve_args)

        # --- Extract solution ---
        w_opt = np.array(sol['x']).ravel()
        x_opt, u_opt = self._parse_solution(w_opt)

        # --- Store for warm-start ---
        self._w0_prev = w_opt
        self._lam_g0_prev = np.array(sol['lam_g']).ravel()
        self._lam_x0_prev = np.array(sol['lam_x']).ravel()

        # --- Info ---
        stats = self._solver.stats()
        info = NMPCSolveInfo(
            cost=float(sol['f']),
            success=stats.get('success', False),
            status=stats.get('return_status', 'unknown'),
            iterations=stats.get('iter_count', 0),
            solve_time_ms=(time.perf_counter() - t_start) * 1000.0,
            solver_stats=stats,
        )

        return x_opt, u_opt, info

    def get_first_control(
        self,
        x0: np.ndarray,
        u_guess: Optional[np.ndarray] = None,
        x_guess: Optional[np.ndarray] = None,
        params: Optional[np.ndarray] = None,
        warm_start: bool = True,
    ) -> Tuple[np.ndarray, NMPCSolveInfo]:
        """Convenience: solve and return only the first control action u_0*.

        This is the standard receding-horizon call.
        """
        x_opt, u_opt, info = self.solve(x0, u_guess, x_guess, params, warm_start)
        return u_opt[:, 0], info

    def shift_warm_start(self) -> None:
        """Shift the stored warm-start by one time step.

        Call this after applying u_0* and before the next solve() to
        provide a good initial guess via temporal shifting:
            x_guess[k] <- x_prev[k+1],  u_guess[k] <- u_prev[k+1]
        with the last element repeated.
        """
        if self._w0_prev is None:
            return

        x_prev, u_prev = self._parse_solution(self._w0_prev)

        # Shift: drop first, repeat last
        x_shifted = np.hstack([x_prev[:, 1:], x_prev[:, -1:]])
        u_shifted = np.hstack([u_prev[:, 1:], u_prev[:, -1:]])

        self._w0_prev = self._build_w0_from_trajectories(x_shifted, u_shifted)

    # ------------------------------------------------------------------ #
    #  Private helpers                                                     #
    # ------------------------------------------------------------------ #

    def _build_initial_guess(
        self,
        x0: np.ndarray,
        u_guess: Optional[np.ndarray],
        x_guess: Optional[np.ndarray],
        warm_start: bool,
    ) -> np.ndarray:
        """Build the decision variable initial guess vector."""
        # Use warm-start from previous solution if available
        if warm_start and self._w0_prev is not None:
            return self._w0_prev

        # Otherwise build from provided guesses
        if x_guess is None:
            x_guess = np.tile(x0.reshape(-1, 1), (1, self.N + 1))
        if u_guess is None:
            u_guess = np.zeros((self.nu, self.N))

        return self._build_w0_from_trajectories(x_guess, u_guess)

    def _build_w0_from_trajectories(
        self, x_traj: np.ndarray, u_traj: np.ndarray
    ) -> np.ndarray:
        """Interleave state and control trajectories into a flat vector.

        Layout: [X0, U0, X1, U1, ..., X_{N-1}, U_{N-1}, X_N]
        """
        parts = []
        for k in range(self.N + 1):
            parts.append(x_traj[:, k])
            if k < self.N:
                parts.append(u_traj[:, k])
        return np.concatenate(parts)

    def _parse_solution(self, w: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Parse flat decision vector into state and control trajectories."""
        x_opt = np.zeros((self.nx, self.N + 1))
        u_opt = np.zeros((self.nu, self.N))

        idx = 0
        for k in range(self.N + 1):
            x_opt[:, k] = w[idx: idx + self.nx]
            idx += self.nx
            if k < self.N:
                u_opt[:, k] = w[idx: idx + self.nu]
                idx += self.nu

        return x_opt, u_opt

    def _get_default_solver_options(self) -> Dict[str, Any]:
        """Default IPOPT/qpOASES options."""
        opts: Dict[str, Any] = {}

        if self.solver_name == 'ipopt':
            opts['ipopt.print_level'] = 0
            opts['print_time'] = 0
            opts['ipopt.max_iter'] = 100
            opts['ipopt.tol'] = 1e-6
            opts['ipopt.acceptable_tol'] = 1e-4
            opts['ipopt.acceptable_iter'] = 5
            opts['ipopt.warm_start_init_point'] = 'yes'
            # Mumps linear solver (default, widely available)
            opts['ipopt.linear_solver'] = 'mumps'

        elif self.solver_name == 'sqpmethod':
            opts['print_time'] = 0
            opts['qpsol'] = 'qpoases'
            opts['qpsol_options'] = {'printLevel': 'none'}
            opts['max_iter'] = 50

        return opts

    # ------------------------------------------------------------------ #
    #  Introspection                                                       #
    # ------------------------------------------------------------------ #

    @property
    def n_decision_vars(self) -> int:
        """Total number of NLP decision variables."""
        return self.nx * (self.N + 1) + self.nu * self.N

    @property
    def n_constraints(self) -> int:
        """Total number of NLP constraints."""
        n_dyn = self.nx * self.N          # dynamics
        n_init = self.nx                   # initial condition
        n_path = self._ng_path * self.N    # path constraints
        n_term = self._ng_terminal         # terminal constraints
        return n_dyn + n_init + n_path + n_term

    def __repr__(self) -> str:
        built = "built" if self._solver is not None else "not built"
        return (
            f"NMPCSolver(nx={self.nx}, nu={self.nu}, N={self.N}, "
            f"dt={self.dt}, solver='{self.solver_name}', {built})"
        )


# ====================================================================== #
#  Utility                                                                 #
# ====================================================================== #

def _merge_dicts(base: Dict, override: Dict) -> Dict:
    """Recursively merge override into base."""
    merged = base.copy()
    for key, value in override.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = _merge_dicts(merged[key], value)
        else:
            merged[key] = value
    return merged
