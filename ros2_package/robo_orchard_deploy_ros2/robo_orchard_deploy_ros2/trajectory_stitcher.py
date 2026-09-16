# Project RoboOrchard
#
# Copyright (c) 2024-2026 Horizon Robotics. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.

"""Re-solve each action chunk so it joins the one being executed smoothly.

Each inference returns a new chunk while the robot is part way through the
previous one. Installing it makes the command continuous in position but
says nothing about velocity or acceleration, so the robot reaches the right
place moving at the wrong speed.

This solves a piecewise-jerk quadratic program over the new chunk whose
initial position, velocity and acceleration are those of the trajectory the
robot is currently following, imposed as a hard equality. The result tracks
the requested chunk under bounds on velocity, acceleration and jerk.
"""

import threading

import numpy as np

try:
    import osqp
    from scipy import sparse

    _HAVE_OSQP = True
except ImportError:  # pragma: no cover - reported once at construction
    _HAVE_OSQP = False


class _PiecewiseJerkQP:
    """One horizon length. P and A are built once; only q/l/u change.

    Rebuilding the solver costs more than a solve, and the horizon only
    changes when the chunk length shifts by a whole grid step.
    """

    def __init__(self, n, config):
        self._n = n
        nx = 3 * (n + 1)
        nv = nx + n
        self._nx = nx
        dt = config.solver_dt
        dt2, dt3 = dt * dt, dt**3

        w = np.full(n + 1, config.track_weight)
        w[-1] += config.terminal_track_weight
        w[0] = 0.0  # fixed by the initial equality
        self._w = w

        p = np.zeros(nv)
        p[0:nx:3] = w
        p[2:nx:3] += config.acceleration_weight
        p[nx - 2] += config.terminal_velocity_weight
        p[nx - 1] += config.terminal_acceleration_weight
        p[nx:] = config.jerk_weight

        rows, cols, vals = [], [], []

        def put(r, c, v):
            rows.append(r)
            cols.append(c)
            vals.append(v)

        for r in range(3):  # x_0 = x_init
            put(r, r, 1.0)
        for k in range(n):  # exact discretisation, not forward Euler
            r0, xk, xk1, uk = 3 + 3 * k, 3 * k, 3 * (k + 1), nx + k
            put(r0, xk1, 1.0)
            put(r0, xk, -1.0)
            put(r0, xk + 1, -dt)
            put(r0, xk + 2, -dt2 / 2.0)
            put(r0, uk, -dt3 / 6.0)
            put(r0 + 1, xk1 + 1, 1.0)
            put(r0 + 1, xk + 1, -1.0)
            put(r0 + 1, xk + 2, -dt)
            put(r0 + 1, uk, -dt2 / 2.0)
            put(r0 + 2, xk1 + 2, 1.0)
            put(r0 + 2, xk + 2, -1.0)
            put(r0 + 2, uk, -dt)
        row = 3 + 3 * n
        # Boxes skip knot 0: it is fixed by the equality above, so bounding
        # it makes the problem infeasible whenever the inherited state is
        # already marginally over the limit.
        for k in range(1, n + 1):
            put(row, 3 * k + 1, 1.0)
            row += 1
        for k in range(1, n + 1):
            put(row, 3 * k + 2, 1.0)
            row += 1
        for k in range(n):
            put(row, nx + k, 1.0)
            row += 1

        a_mat = sparse.csc_matrix((vals, (rows, cols)), shape=(row, nv))
        self._lo = np.zeros(row)
        self._hi = np.zeros(row)
        b = 3 + 3 * n
        for offset, bound in (
            (0, config.max_velocity),
            (n, config.max_acceleration),
            (2 * n, config.max_jerk),
        ):
            self._lo[b + offset : b + offset + n] = -bound
            self._hi[b + offset : b + offset + n] = bound

        self._solver = osqp.OSQP()
        self._solver.setup(
            sparse.diags(2.0 * p, format="csc"),
            np.zeros(nv),
            a_mat,
            self._lo,
            self._hi,
            verbose=False,
            warm_starting=True,
            polishing=True,
            eps_abs=1e-4,
            eps_rel=1e-4,
        )

    def solve(self, x0, ref):
        """Returns (pos, vel, acc, jerk) arrays, or None if infeasible."""
        q = np.zeros(self._nx + self._n)
        q[0 : self._nx : 3] = -2.0 * self._w * ref
        self._lo[0:3] = x0
        self._hi[0:3] = x0
        self._solver.update(q=q, l=self._lo, u=self._hi)
        res = self._solver.solve()
        if res.info.status_val not in (1, 2):  # solved, solved_inaccurate
            return None
        s = res.x[: self._nx].reshape(-1, 3)
        return s[:, 0], s[:, 1], s[:, 2], res.x[self._nx :]


def _eval_cubic(x, v, a, u, dt, t):
    """State of the constant-jerk cubic at an arbitrary time."""
    k = int(np.clip(int(t / dt), 0, len(u) - 1))
    tau = t - k * dt
    return (
        x[k] + v[k] * tau + 0.5 * a[k] * tau**2 + u[k] * tau**3 / 6.0,
        v[k] + a[k] * tau + 0.5 * u[k] * tau**2,
        a[k] + u[k] * tau,
    )


def _sample_cubic(x, v, a, u, dt, t_out):
    """Evaluate the cubic on a whole grid at once."""
    k = np.clip((t_out / dt).astype(int), 0, len(u) - 1)
    tau = t_out - k * dt
    return x[k] + v[k] * tau + 0.5 * a[k] * tau**2 + u[k] * tau**3 / 6.0


class TrajectoryStitcher:
    """Smooth every chunk into the one it replaces.

    Holds the live trajectory between calls, so a run is a chain of
    C2-continuous segments rather than independent chunks.

    ``keys`` are the action channel keys to solve together, in a fixed
    order. Their joint counts may differ; each channel is written back at
    its own width.
    """

    def __init__(self, config, control_frequency, keys, logger=None):
        self._log = logger
        self._config = config
        self._keys = list(keys)
        self._enabled = config is not None
        if not self._enabled:
            return
        if not _HAVE_OSQP:
            self._enabled = False
            if logger:
                logger.error(
                    "trajectory_stitch is configured but osqp is not "
                    "installed; installing chunks unchanged."
                )
            return

        self._dt_ctrl = 1.0 / control_frequency
        self._qp = {}
        self._state_lock = threading.Lock()
        self._reset_generation = 0
        # The solve currently driving the robot, kept whole rather than
        # reduced to an end state: how far it will run is only known when
        # the next chunk arrives.
        self._live = None
        # The most recent solve, held until the caller confirms it was
        # installed. A chunk can still be rejected after being solved, and
        # promoting it here would hand the next solve a trajectory the
        # robot never executed.
        self._pending = None
        self.n_solved = 0
        self.n_failed = 0
        # Handoffs read past the live window.
        self.n_overrun = 0

    @property
    def enabled(self):
        return self._enabled

    def reset(self) -> None:
        """Drop the live trajectory.

        Called when execution is paused or resumed: forcing continuity onto
        a trajectory from before the gap would command the robot back to
        wherever it was when it stopped.

        Invalidates in-flight solves without waiting for the solver. Calls
        to ``stitch`` must still be serialized by the caller.
        """
        if self._enabled:
            with self._state_lock:
                self._reset_generation += 1
                self._live = None
                self._pending = None

    def discard_pending(self) -> None:
        """Discard a rejected solve without changing the live trajectory.

        Call after ``stitch`` returns if the node rejects its result.
        """
        if self._enabled:
            with self._state_lock:
                self._pending = None

    def commit(self) -> None:
        """Promote the last solve to the live trajectory.

        Call once the chunk has actually been installed. Skipping this --
        because the chunk was dropped -- leaves the live trajectory alone,
        which is correct: the robot is still executing it.

        A chunk installed without a solve behind it (the solver failed and
        it passed through unchanged) clears the live trajectory instead.
        No cubic describes what the robot is now running, and the next
        solve is better off starting from rest than from a trajectory that
        has been superseded.
        """
        if self._enabled:
            with self._state_lock:
                self._live = self._pending
                self._pending = None

    def stitch(
        self,
        actions: dict,
        install_idx: int = 0,
        prev_ran: int = 0,
        *,
        held_actions: dict | None = None,
    ) -> dict:
        """Rewrite a chunk so it continues the live trajectory.

        ``install_idx`` is the step this chunk starts executing at, which
        need not be 0: the robot joins a chunk however far the round trip
        carried it. Only the tail from there is solved, because
        constraining step 0 pins a command that is never sent.

        Any nonempty tail can be solved, including a single control step.
        The QP uses its own integration grid; it does not require four
        action samples to obtain velocity, acceleration or jerk.

        ``prev_ran`` is how many steps the chunk being replaced actually
        got through. The handoff state is read that far into the previous
        solve. This describes the planned command, not measured feedback.

        ``held_actions`` is the exhausted chunk, with the same channel keys
        and joint ordering as ``actions``. Its final positions, in radians,
        become the initial state with zero velocity and acceleration,
        instead of continuing the old moving curve after a wait.

        Returns the action dict, stitched when possible and untouched when
        not: a solver failure degrades to installing the chunk unchanged
        rather than stopping the robot.

        Leaves the live trajectory alone. The solve is held until
        :meth:`commit`, because the caller can still reject this chunk
        after the solve has run.
        """
        if not self._enabled:
            return actions

        with self._state_lock:
            reset_generation = self._reset_generation
            live_trajectory = self._live
            self._pending = None
        cfg = self._config
        dt_qp = cfg.solver_dt

        sequences = [actions.get(key) or [] for key in self._keys]
        n_steps_in = len(sequences[0]) if sequences else 0
        if n_steps_in == 0 or any(len(s) != n_steps_in for s in sequences):
            return actions
        start = max(0, int(install_idx))
        if start >= n_steps_in:
            return actions
        widths = [len(s[0]) for s in sequences]

        full = np.hstack([np.asarray(s, dtype=float) for s in sequences])
        ref = full[start:]
        n_steps, n_j = ref.shape

        # The chunk arrives at control rate; the QP runs on its own grid,
        # rounded up so a few horizons serve every chunk and their solvers
        # get reused. Past the end of the chunk np.interp holds the final
        # position, and only t_ctrl is ever sampled back out.
        t_ctrl = np.arange(n_steps) * self._dt_ctrl
        n_qp = int(np.ceil(t_ctrl[-1] / dt_qp / cfg.horizon_quantum))
        n_qp = max(3, n_qp * cfg.horizon_quantum)
        t_qp = np.arange(n_qp + 1) * dt_qp
        ref_qp = np.column_stack(
            [np.interp(t_qp, t_ctrl, ref[:, d]) for d in range(n_j)]
        )

        # Read from the previous solve, that many steps into it. It cannot
        # be computed when that chunk was solved, because how long it would
        # run was not known yet.
        state = None
        if held_actions is not None:
            state = np.zeros((n_j, 3))
            state[:, 0] = np.concatenate(
                [held_actions[key][-1] for key in self._keys]
            )
        elif live_trajectory is not None:
            live, live_t0 = live_trajectory
            t = live_t0 + max(0, prev_ran) * self._dt_ctrl
            state = np.array(
                [_eval_cubic(*live[d], dt_qp, t) for d in range(n_j)]
            )
            # Past the window _eval_cubic extrapolates, and the state it
            # returns is imposed as this solve's initial equality.
            live_ctrl = int(round(len(live[0][3]) * dt_qp / self._dt_ctrl))
            if prev_ran > live_ctrl:
                self.n_overrun += 1
                if self._log:
                    self._log.error(
                        "Stitch handoff extrapolated: prev_ran=%d > %d "
                        "steps, %.3f rad from the chunk"
                        % (
                            prev_ran,
                            live_ctrl,
                            float(np.abs(state[:, 0] - ref_qp[0]).max()),
                        )
                    )
        if state is None:
            state = np.zeros((n_j, 3))
            state[:, 0] = ref_qp[0]

        if n_qp not in self._qp:
            self._qp[n_qp] = _PiecewiseJerkQP(n_qp, cfg)
        qp = self._qp[n_qp]

        out = np.empty_like(ref)
        solved = []
        for d in range(n_j):
            res = qp.solve(state[d], ref_qp[:, d])
            if res is None:
                self.n_failed += 1
                if self._log:
                    self._log.warning(
                        "Stitch solve failed on joint %d; installing the "
                        "chunk unchanged." % d,
                        throttle_duration_sec=5.0,
                    )
                return actions
            x, v, a, u = res
            out[:, d] = _sample_cubic(x, v, a, u, dt_qp, t_ctrl)
            solved.append(res)

        # Held, not promoted: whether this chunk is installed is decided
        # after the solve. The time origin is the instant this chunk would
        # begin executing.
        with self._state_lock:
            if reset_generation == self._reset_generation:
                self._pending = (solved, 0.0)
        self.n_solved += 1

        # Only the solved window is replaced; rewriting what came before it
        # would be a retroactive command.
        full[start:] = out
        start_col = 0
        for key, width in zip(self._keys, widths, strict=True):
            actions[key] = full[:, start_col : start_col + width].tolist()
            start_col += width
        return actions
