"""``PuzzleSolver``: the two-model portfolio for N^2-puzzles, run deterministically.

Port of ``ILP/PuzzleSolver.java`` (with ``MultiagentGraphSolverGurobiTimePuzzle``, whose
single-horizon solve is :func:`mppflow.solvers.solve_time_puzzle` and whose model is
:func:`mppflow.model.build_time_puzzle_model`).

==========================================  =================================================
mppflow                                     original
==========================================  =================================================
:class:`PuzzleSolver`                       ``PuzzleSolver`` (``extends GRBCallback``)
  ``.solve(time_limit)``                    ``protected int[][] solve(double timeLimit)``
  ``.update_reachable_sets(T)``             ``private void updateReachableSets(...)``
:func:`solve_problem`                       ``public static int[][] solve(Problem, double)``
==========================================  =================================================

The original's algorithm
------------------------
``solve`` starts at ``T = max_i |shortest path_i|`` (one ``findShortestPaths`` call: random
draws) and, for ``T, T+1, ...``, starts two threads on the same reachability-pruned model:
instance 1 maximizes the number of loopbacks (``setOptimal = false``, a max-flow), instance 2
fixes them to 1 (``setOptimal = true``, a feasibility problem). The first worker that finds a
plan of value ``n`` sets ``goodSolution`` and ``optimizationDone``; the other is aborted from
the Gurobi message callback. When a worker fails (``ObjVal < n`` or infeasible), the main
thread -- polling once a second -- sets ``optimizationDone``, waits until a callback reports
``allThreadFinished``, and moves to ``T + 1``. The static ``solve(Problem, double)`` uses this
for more than 16 robots and ``planPathsAdvanced(..., solveRelaxed=false, 0, setOptimal=true,
timeLimit)`` (:func:`mppflow.solvers.solve_time`) otherwise. It is reached only from
``Main.runN2PuzzlePerformanceMetric``, which ``Main.main`` never calls.

Deterministic emulation (both modes)
------------------------------------
The two models are solved one after the other at each horizon (``models``, default
``(1, 2)``: thread-start order). Both models have a solution of value ``n`` at exactly the
same horizons, so the race only decides *which* optimal plan the original returns
(``goodSolution``; nondeterministic). We return **instance 2's plan** (the feasibility model,
i.e. the model MINMAKESPAN's ILP also solves) when it found one, else instance 1's
(``extra["good_solution"]`` says which). ``models=(2,)`` solves only the feasibility model
(half the work, the same plans).

compat modes
------------
``compat="original"`` reproduces the original's control flow, including its deadlock: when
both workers fail at a horizon and both have finished before the main thread's poll that
notices the failure (``poll_interval`` = 1 s), no Gurobi message callback can set
``allThreadFinished`` any more and the original waits forever, ignoring its time limit
(original-bugs B17; e.g. the star swap, whose first horizon is
infeasible). We raise :class:`~mppflow.errors.OriginalWouldHang` then. Whether a worker is
still running at that poll is timing; ``race`` decides it: ``"measured"`` (default) compares
the measured backend runtimes with the poll times, ``"fast"`` assumes every worker finishes
before the first poll (true for small instances), ``"slow"`` assumes a worker is still running
(it is aborted and the loop continues with ``T + 1``). An infeasible instance makes the
original increase ``T`` forever, which we detect with
:func:`~mppflow.solvers.joint_feasibility` (and, on one-way graphs,
``solvers._original_pruning_dead``) and report as ``OriginalWouldHang``, or, with a time limit,
as the ``null`` result at once (``extra["shortcut"] = True``); an unreachable goal raises
:class:`~mppflow.errors.JavaNullPointerException` after the shortest-path draws; ``(int)``
truncation of ``ObjVal`` and ``X == 1.0`` path extraction; one ``"."`` per poll that sees a
new elapsed minute and a final empty line in the log.

Time in ``original`` mode (a virtual Java clock). The workers' horizon starts at ``h`` (the
measured elapsed time for the first horizon). The main thread polls at ``h + j * poll`` for
``j = 1, 2, ...``; at each poll it first returns ``null`` if ``elapsed > timeLimit`` (so a plan
found before that poll is thrown away: with ``timeLimit < 1`` the result is always ``null``),
then handles a failure (``failedAttempt``), then prints the minute dot, then leaves the loop
if a plan was found. A worker counts as finished at a poll when its runtime (``"measured"``;
0 for ``"fast"`` / ``"slow"``) is at most ``j * poll``; the next horizon starts at the poll
that ended this one. The backend gets the time up to the first poll past the limit (a worker
still running then no longer matters). A worker still running when the horizon ends is
aborted (its plan is lost).

The fields persist between :meth:`PuzzleSolver.solve` calls as in Java
(``optimizationDone``, ``failedAttempt``, ``allThreadFinished``, ``goodSolution``,
``flag1``/``flag2``, ``sol1``/``sol2``). Both ways of leaving ``solve`` leave
``optimizationDone == true``, so the next call skips the polling loop at its first horizon
``T`` and goes straight to its wait for ``flag1 && flag2``:

* After a success the main thread had cleared the flags, so it waits for this horizon's
  workers. They abort at their first Gurobi message (results ``null``, ``failedAttempt``
  unchanged, ``allThreadFinished`` set) and the search continues at ``T + 1``. The plan it
  returns is one step longer than optimal.
* After a ``null`` time-limit return the flags were never cleared. Each worker had set its
  flag and ``sol`` when it finished; a worker still running was aborted and delivered ``null``.
  The next call consumes those flags at once. If ``sol1`` or ``sol2`` is a plan, it returns
  the ``goodSolution`` one: the plan the timed-out call found and dropped, at the same ``T``
  (log ``"\n"``, ``extra["stale_result"]``). Its own workers abort later and leave the flags
  set with ``null`` results. A later call therefore consumes those and moves on to ``T + 1``.
  A failure noticed only by the timed-out poll leaves ``failedAttempt`` set, and the next
  horizon handles it at its first poll.

Attempt records of such a horizon have ``stale_done=True`` and status ``"interrupted"``
(workers that were waited for) or ``"abandoned"`` (workers whose results were never read).

Not emulated (nondeterministic races of the original, ``docs/notes/original-bugs.md`` B17,
item 4): a stale ``failedAttempt`` that aborts the next horizon's workers and skips a ``T``,
an aborted worker overwriting ``goodSolution``, and an aborted
worker's late messages setting ``allThreadFinished`` again after the main thread cleared it,
and the abandoned workers of a horizon whose stale flags were ``null`` and which therefore
moves on to ``T + 1``. In Java those workers run on with ``optimizationDone == false``. They
can finish and set ``optimizationDone``, which aborts the next horizon's workers and skips
another ``T``. We treat them as discarded. In the live-Java probe (seed 13, 3x3, ``solve(0.3)``,
``solve(-1)``, ``solve(-1)``) the third call gave ``T = 3`` in one run and ``T = 4`` in two;
we give ``T = 3``.
The original's Gurobi log files (``C:\\Temp\\25puzzle-instance-<k>``) are not written, so their
``GRBException`` hang (unwritable directory, original-bugs B17, item 2) cannot occur.

Differences of ``compat="fixed"``:

* No deadlock: a failed horizon always moves on to ``T + 1``.
* Infeasible instances raise :class:`~mppflow.errors.Infeasible` (joint-state search after the
  first failed horizon, or ``t_max`` exceeded); duplicate starts and out-of-range ids raise
  ``ValueError``, duplicate goals and goals unreachable from their start ``Infeasible``, all
  before any draw.
* Time limit: the measured elapsed time is tested before each model and the remaining time
  is passed to the backend; a plan found within the limit is returned (the original drops
  it unless a poll inside the limit notices it). A non-positive ``time_limit`` means no
  limit (as in the original).
* A second :meth:`PuzzleSolver.solve` call searches afresh and returns an optimal plan again.
  The flags are reset per horizon and only ``start_time_steps`` persists. After a success,
  the original's stale ``optimizationDone`` makes it return makespan ``T + 1``. After a
  ``null`` time-limit return, the original returns the stale ``sol1``/``sol2`` of the
  timed-out call without solving. That is the same horizon (and, with Gurobi ``Threads=1``,
  the same plan), but it is wrong if the instance changed in between.
* ``ObjVal`` is rounded and paths are extracted with ``X > 0.5``.
* Reachability on one-way graphs: see :func:`mppflow.network.update_reachable_sets`.
"""
from __future__ import annotations

import sys
import time
from typing import Callable, Sequence

from . import lp
from ._java import MathRandom, resolve_rng
from .errors import (
    Infeasible,
    JavaNullPointerException,
    MPPError,
    OriginalWouldHang,
)
from .graph import Graph, check_compat, java_d2i
from .model import build_time_puzzle_model
from .network import Reachability, _reversed_adjacency, update_reachable_sets
from .pathfinder import find_shortest_paths, get_distances_to_all_vertices
from .problem import Problem
from .solvers import (
    DEFAULT_FEASIBILITY_BUDGET,
    SolveResult,
    _original_pruning_dead,
    _refutes,
    _undecided_status,
    joint_feasibility,
)

__all__ = ["PuzzleSolver", "solve_problem", "PUZZLE_SOLVER_MIN_AGENTS"]

PUZZLE_SOLVER_MIN_AGENTS = 17
"""``solve(Problem, double)`` uses the two-model portfolio from this many robots on
(``p.sg[0].length > 16``)."""

_NPE_SP = 'Cannot read field "vertexList" because "<local5>[<local6>]" is null'
_NPE_SG = 'Cannot load from object array because "<parameter1>.sg" is null'


class _Out:
    def __init__(self, echo: bool):
        self.parts: list[str] = []
        self.echo = echo

    def print(self, s: str) -> None:
        if s:
            self.parts.append(s)
            if self.echo:
                sys.stdout.write(s)

    def println(self, s: str = "") -> None:
        self.print(s + "\n")

    def text(self) -> str:
        return "".join(self.parts)


class PuzzleSolver:
    """``PuzzleSolver(graph, starts, goals)``: see the module docstring.

    Args:
        graph, starts, goals: the instance (stored by reference, as in the original).
        compat: ``"fixed"`` (default) or ``"original"``.
        backend, params: LP backend (``params={"Threads": 1}`` with ``backend="gurobi"`` for
            reproducible plans).
        rng: the random stream of the shortest-path draws (``None``: the module stream).
        clock: time source in seconds (default ``time.time``).
        models: the instances solved at each horizon, in order (``1`` = max-flow,
            ``2`` = feasibility).
        race: ``"measured"``, ``"fast"`` or ``"slow"`` (``original`` only; see the module
            docstring).
        poll_interval: the original main thread's polling period (1 s).
        t_max: ``fixed`` only: never try a horizon above ``t_max`` (``Infeasible``).
        feasibility_budget: work budget of the infeasibility detection.
        echo: also print the log.

    Attributes:
        start_time_steps: the original's ``startTimeSteps`` field: the current horizon. It is
            not reset between :meth:`solve` calls (a second call starts where the first
            stopped, or at ``T_min`` if that is larger), as in the original.
        reachable_vertices: the last :class:`~mppflow.network.Reachability`.
        optimization_done, good_solution, failed_attempt, all_thread_finished, flag1, flag2,
            sol1, sol2: the original's fields after the last horizon. ``flag1``/``flag2``
            mean "set and not yet consumed by the main thread". In ``original`` mode they
            persist between :meth:`solve` calls and take effect as in Java (see the module
            docstring). ``fixed`` does not use the flag and sol fields.
    """

    def __init__(self, graph: Graph, starts: Sequence[int], goals: Sequence[int], *, compat: str = "fixed",
                 backend: str = "highs", params: dict | None = None, rng: MathRandom | None = None,
                 clock: Callable[[], float] | None = None, models: Sequence[int] = (1, 2),
                 race: str = "measured", poll_interval: float = 1.0, t_max: int | None = None,
                 feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET, echo: bool = False):
        self.orig = check_compat(compat)
        self.compat = compat
        self.graph = graph
        self.starts = starts
        self.goals = goals
        self.backend = backend
        self.params = params
        self.rng = rng
        self.clock = clock or time.time
        models = tuple(models)
        if not models or any(m not in (1, 2) for m in models) or len(set(models)) != len(models):
            raise ValueError("models must list instance 1 and/or 2, each at most once")
        self.models = models
        if race not in ("measured", "fast", "slow"):
            raise ValueError("race must be 'measured', 'fast' or 'slow'")
        self.race = race
        self.poll_interval = float(poll_interval)
        self.t_max = t_max
        self.feasibility_budget = feasibility_budget
        self.echo = echo
        self.start_time_steps = 0
        self.reachable_vertices: Reachability | None = None
        self.optimization_done = False
        self.good_solution = 0
        self.failed_attempt = False
        self.all_thread_finished = False
        self.flag1 = self.flag2 = False
        self.sol1: list[list[int]] | None = None
        self.sol2: list[list[int]] | None = None
        self._feas = None
        self._t_first = 0

    # ------------------------------------------------------------------ helpers
    def update_reachable_sets(self, T: int) -> Reachability:
        """``updateReachableSets(graph, starts, goals, T)``: identical to the Time
        solver's (:func:`~mppflow.network.update_reachable_sets`); stored in
        :attr:`reachable_vertices`."""
        self.reachable_vertices = update_reachable_sets(self.graph, self.starts, self.goals, T, self.compat)
        return self.reachable_vertices

    def _now(self):
        t = self.clock()
        return int(t * 1000) if self.orig else t

    def _elapsed(self, t0) -> float:
        d = self._now() - t0
        return d / 1000.0 if self.orig else d

    def _solve_model(self, instance: int, T: int, time_left: float | None):
        """One worker: ``MultiagentGraphSolverGurobiTimePuzzle.solve`` at horizon ``T``."""
        A = len(self.starts)
        m = build_time_puzzle_model(self.graph, self.starts, self.goals, T, True, instance == 2,
                                    self.reachable_vertices, compat=self.compat)
        sol = lp.solve(m.model, backend=self.backend, time_limit=time_left, params=self.params)
        paths = None
        if sol.has_solution:
            val = java_d2i(sol.objective) if self.orig else int(round(sol.objective))
            if val == A:
                paths = m.retrieve_paths(sol.x, self.graph)
        rec = {"T": T, "phase": "ilp", "instance": instance, "status": sol.status, "objective": sol.objective,
               "bound": sol.bound, "runtime": sol.runtime}
        return paths, sol, rec

    def _check(self) -> None:
        """``fixed``-mode input checks, before any draw."""
        g, s, gl = self.graph, list(self.starts), list(self.goals)
        if len(s) != len(gl):
            raise ValueError("starts and goals must have the same length")
        for v in s + gl:
            if not 0 <= v < g.n:
                raise ValueError(f"vertex {v} out of range 0..{g.n - 1}")
        if len(set(s)) < len(s):
            raise ValueError("two robots share a start vertex")
        if len(set(gl)) < len(gl):
            raise Infeasible("two robots share a goal vertex")
        for i, (a, b) in enumerate(zip(s, gl)):
            if get_distances_to_all_vertices(g, a, None, "fixed", unreachable=-1)[b] < 0:
                raise Infeasible(f"goal of robot {i} is unreachable from its start")

    # -------------------------------------------------------------------- solve
    def solve(self, time_limit: float = -1.0) -> SolveResult:
        """``PuzzleSolver.solve(timeLimit)``: the smallest horizon at which the models have
        a plan of value ``n`` (an optimal-makespan plan), searched upward from
        ``max(start_time_steps, T_min)``.

        ``time_limit`` in seconds; ``<= 0`` means none. Returns a
        :class:`~mppflow.solvers.SolveResult` (``paths`` ``None`` with status
        ``"time_limit"`` when the limit expired). See the module docstring for the
        deterministic emulation and the differences of ``fixed``.

        Differs from the original: (1) a plan found within the time limit is returned; the
        original tests the limit at its 1-s polls before it looks at the result, so it
        returns ``null`` whenever the poll that would notice the plan comes after the limit
        (e.g. any limit below 1 s). (2) Every call searches afresh from
        ``max(start_time_steps, T_min)``; the original leaves ``optimizationDone == true``
        after a success, so a second call skips its polling loop, both workers abort at
        their first Gurobi message, and it returns a plan of makespan ``T + 1`` (not
        optimal). After a ``null`` time-limit return, the original's next call returns the
        timed-out call's stale ``sol1``/``sol2`` plan without solving (its flags were never
        cleared). ``compat="original"`` reproduces all of these.
        """
        out = _Out(self.echo)
        try:
            return self._solve(time_limit, out)
        except MPPError as e:
            if getattr(e, "output", None) is None:
                e.output = out.text()
            raise

    def _solve(self, time_limit: float, out: _Out) -> SolveResult:
        orig = self.orig
        if not orig:
            self._check()
        t0 = self._now()
        rng = resolve_rng(self.rng)
        sp = find_shortest_paths(self.graph, self.starts, self.goals, rng,
                                 heuristic=None if orig else False, compat=self.compat)
        for i, p in enumerate(sp):
            if p is None:
                if orig:
                    raise JavaNullPointerException(_NPE_SP)
                raise Infeasible(f"goal of robot {i} is unreachable from its start")  # pragma: no cover
            if len(p) - 1 > self.start_time_steps:
                self.start_time_steps = len(p) - 1
        lower_bound = self.start_time_steps
        self._t_first = self.start_time_steps  # the proofs of _after_failure cover T >= this
        has = time_limit > 0
        attempts: list[dict] = []

        def result(paths, status, **extra):
            obj = None if paths is None else (len(paths[0]) - 1 if paths else 0)
            return SolveResult(paths=paths, status=status, objective=obj, T=self.start_time_steps,
                               lower_bound=lower_bound, log=out.text(), attempts=attempts, extra=extra)

        if orig:
            return self._solve_original(t0, time_limit, has, out, attempts, result)
        return self._solve_fixed(t0, time_limit, has, out, attempts, result)

    def _solve_fixed(self, t0, time_limit, has, out, attempts, result) -> SolveResult:
        A = len(self.starts)
        minutes = 0
        if A == 0:
            # the model has no columns; both workers report value 0 == n at once
            out.println()
            self.good_solution, self.optimization_done = 2, True
            return result([], "optimal", good_solution=2)
        if self.t_max is not None and self.start_time_steps > self.t_max:
            raise Infeasible(f"the first horizon T = {self.start_time_steps} exceeds t_max = {self.t_max}")
        while True:
            T = self.start_time_steps
            self.update_reachable_sets(T)
            self.optimization_done = False
            self.failed_attempt = False
            self.good_solution = 0
            sols: dict[int, list[list[int]] | None] = {}
            refuted, undecided = False, None
            for inst in self.models:
                left = None
                if has:
                    left = time_limit - self._elapsed(t0)
                    if left <= 0:
                        return result(None, "time_limit")
                paths, sol, rec = self._solve_model(inst, T, left)
                attempts.append(rec)
                sols[inst] = paths
                if paths is not None:
                    self.good_solution = inst
                    self.optimization_done = True
                elif sol.status != "time_limit":
                    self.failed_attempt = True
                if paths is None:
                    if _refutes(sol, len(self.starts)):
                        refuted = True
                    elif undecided is None:
                        undecided = sol.status
            new_min = int(self._elapsed(t0) // 60)
            if new_min > minutes:  # the original prints one "." per elapsed minute while polling
                out.print("." * (new_min - minutes))
                minutes = new_min
            found = sols.get(2) if sols.get(2) is not None else sols.get(1)
            if found is not None:
                out.println()
                good = 2 if sols.get(2) is not None else 1
                self.good_solution = good
                return result(found, "optimal", good_solution=good)
            if not refuted:
                # a limit (time, or a node / solution limit in params) stopped every model
                # before it decided T: T is not proved infeasible, so do not move on to T + 1
                ex = {} if undecided == "time_limit" else {"solver_status": undecided}
                return result(None, _undecided_status(None), **ex)
            self._after_failure(T, has)
            self.optimization_done = False
            self.start_time_steps += 1

    def _solve_original(self, t0, time_limit, has, out, attempts, result) -> SolveResult:
        """The original's control flow on a virtual Java clock (see "Time in original mode"
        in the module docstring)."""
        A = len(self.starts)
        poll = self.poll_interval
        vt = self._elapsed(t0)  # Java's elapsed time when the first workers start
        minutes = 0
        while True:
            T = self.start_time_steps
            self.update_reachable_sets(T)
            if self.optimization_done:
                # Stale optimizationDone of an earlier solve(): the polling loop is skipped
                # and the main thread goes straight to its wait for flag1 && flag2.
                for inst in self.models:
                    attempts.append({"T": T, "phase": "ilp", "instance": inst,
                                     "status": "abandoned" if self.flag1 and self.flag2 else "interrupted",
                                     "objective": None, "bound": None, "runtime": 0.0, "stale_done": True})
                if self.flag1 and self.flag2:
                    # The flags (and sol1/sol2) were left set by the workers of a call that
                    # returned null at its time limit: they are consumed at once, and this
                    # horizon's own workers are left running, unread.
                    self.flag1 = self.flag2 = False
                    if self.sol1 is not None or self.sol2 is not None:
                        chosen = self.sol1 if self.good_solution == 1 else self.sol2
                        # optimizationDone stays true: the abandoned workers abort at their
                        # first Gurobi message (allThreadFinished) and deliver null later
                        self.sol1 = self.sol2 = None
                        self.flag1 = self.flag2 = True
                        self.all_thread_finished = True
                        out.println()
                        if chosen is None:  # pragma: no cover - goodSolution names a null sol
                            return result(None, "time_limit", good_solution=self.good_solution,
                                          stale_result=True)
                        return result(chosen, "optimal", good_solution=self.good_solution, stale_result=True)
                    # both null: T + 1 (the abandoned workers of T race the next horizon in
                    # Java; not emulated, see the module docstring)
                else:
                    # each worker's first Gurobi message callback sets allThreadFinished and
                    # aborts it (status INTERRUPTED: failedAttempt unchanged), both results
                    # are null, and the main thread moves on to T + 1.
                    self.all_thread_finished = True
                    self.sol1 = self.sol2 = None
                self.optimization_done = False
                self.start_time_steps += 1
                continue
            h = vt
            left = None
            if has:
                # workers still running at the first poll past the limit no longer matter
                k_null = max(1, int((time_limit - h) // poll) + 1)
                left = k_null * poll
            outcomes: dict[int, tuple[float, str, list[list[int]] | None]] = {}
            for inst in self.models:
                if A == 0:
                    # no columns: the worker reports value 0 == n at once
                    paths, status, runtime = [], "optimal", 0.0
                    attempts.append({"T": T, "phase": "ilp", "instance": inst, "status": status,
                                     "objective": 0.0, "bound": 0.0, "runtime": runtime})
                else:
                    paths, sol, rec = self._solve_model(inst, T, left)
                    attempts.append(rec)
                    status, runtime = sol.status, float(sol.runtime or 0.0)
                if paths is not None:
                    kind = "ok"
                elif has and status == "time_limit":
                    kind = "running"  # not finished by the poll that returns null
                else:
                    kind = "fail"  # any status but INTERRUPTED sets failedAttempt
                outcomes[inst] = (runtime if self.race == "measured" else 0.0, kind, paths)
            j = 0
            noticed_failure = False
            while True:  # the main thread's polls, at h + j * poll
                j += 1
                tj = h + j * poll
                done = {i: o for i, o in outcomes.items() if o[1] != "running" and o[0] <= j * poll}
                if has and tj > time_limit:
                    # null without touching the workers' flags: a finished worker has set its
                    # flag and sol (and goodSolution or failedAttempt); one still running is
                    # aborted at its next message (sets allThreadFinished, delivers null)
                    self.optimization_done = True
                    oks = [i for i, o in done.items() if o[1] == "ok"]
                    if oks:
                        self.good_solution = 2 if 2 in oks else 1
                    if any(o[1] == "fail" for o in done.values()):
                        self.failed_attempt = True
                    if len(done) < len(outcomes):
                        self.all_thread_finished = True
                    self._set_sols(done, ready=True)
                    return result(None, "time_limit")
                if self.failed_attempt or any(o[1] == "fail" for o in done.values()):
                    self.failed_attempt = True
                    self.optimization_done = True
                    if self.race == "measured":
                        all_done = len(done) == len(outcomes)
                    else:
                        all_done = self.race == "fast"
                    if all_done and not self.all_thread_finished:
                        raise OriginalWouldHang(
                            f"PuzzleSolver.solve: both models failed at T = {T} before the main thread's poll "
                            "noticed it; it waits forever for a Gurobi callback that never comes "
                            "(original-bugs B17)")
                    # the main thread waited for a callback of a running worker, then cleared
                    self.all_thread_finished = False
                    self.failed_attempt = False
                    noticed_failure = True
                    break
                m = int(tj // 60)
                if m > minutes:  # one "." per poll that sees a new elapsed minute
                    out.print(".")
                    minutes = m
                if any(o[1] == "ok" for o in done.values()):
                    break
            vt = h + j * poll
            oks = {i: o[2] for i, o in done.items() if o[1] == "ok"}
            self._set_sols(done, ready=False)  # the main thread waited for the flags, cleared them
            if not noticed_failure and len(done) < len(outcomes):
                # a worker still running after a success is aborted from its next message
                # callback, which leaves allThreadFinished set
                self.all_thread_finished = True
            if oks:
                good = 2 if 2 in oks else 1
                self.good_solution = good
                self.optimization_done = True
                out.println()
                return result(oks[good], "optimal", good_solution=good)
            r = self._after_failure(T, has)
            if r is not None:
                return result(None, "time_limit", shortcut=True)
            self.optimization_done = False
            self.start_time_steps += 1

    def _set_sols(self, done: dict, ready: bool) -> None:
        """Java's ``sol1``/``sol2`` (the plan of a worker that finished with value ``n``,
        else ``null``) and ``flag1``/``flag2`` (``ready``: set and not yet consumed). A
        model not in :attr:`models` counts as a worker that delivered ``null``."""
        sols = {i: (o[2] if o[1] == "ok" else None) for i, o in done.items()}
        self.sol1, self.sol2 = sols.get(1), sols.get(2)
        self.flag1 = self.flag2 = ready

    def _after_failure(self, T: int, has: bool):
        """Detect the original's endless ``T`` loop (or, in ``fixed``, prove infeasibility /
        apply ``t_max``) after a failed horizon. Returns a marker to return the null
        result, or ``None`` to continue."""
        if not self.orig and self.t_max is not None and T + 1 > self.t_max:
            raise Infeasible(f"no solution with makespan <= t_max = {self.t_max}")
        feas = self._feas
        # keyed by the graph object itself (an ``id()`` can be reused by a new graph once
        # the old one is freed), its size, and the instance
        # An inconclusive check (budget exhausted) is retried at the 2nd, 4th, 8th, ... failed
        # horizon with the budget times the number of failed horizons (as in the solvers).
        t_first = self._t_first
        key = (self.graph.n, tuple(self.starts), tuple(self.goals), t_first)
        if feas is None or feas[0][0] is not self.graph or feas[0][1] != key:
            feas = ((self.graph, key), None, None, 0)
        ident, v, dead, nf = feas
        nf += 1
        if nf == 1 or ((nf & (nf - 1)) == 0 and (v is None or (v is not False and dead is None))):
            starts, goals = list(self.starts), list(self.goals)
            b = self.feasibility_budget * nf
            if v is None:
                v = joint_feasibility(self.graph, starts, goals, b, t_first=t_first)
            dead = False
            if v is not False and self.orig and _reversed_adjacency(self.graph) is not None:
                dead = _original_pruning_dead(self.graph, starts, goals, b, t_first=t_first)
        feas = (ident, v, dead, nf)
        self._feas = feas
        dead = dead is True
        if v is False or dead:
            if not self.orig:
                raise Infeasible("the instance has no solution for any horizon (joint-state search)")
            if not has:
                raise OriginalWouldHang(
                    "PuzzleSolver.solve: no horizon has a plan (proved by joint-state search); "
                    "the original increases T forever")
            return True
        return None


def solve_problem(p: Problem, time_limit: float = -1.0, *, compat: str = "fixed", backend: str = "highs",
                  params: dict | None = None, rng: MathRandom | None = None,
                  clock: Callable[[], float] | None = None, debug: bool = False, print_paths: bool = False,
                  echo: bool = False, **puzzle_kwargs) -> SolveResult:
    """``PuzzleSolver.solve(Problem p, double timeLimit)``.

    More than 16 robots: :meth:`PuzzleSolver.solve` (``puzzle_kwargs`` go to the
    :class:`PuzzleSolver` constructor). Otherwise ``planPathsAdvanced(g, s, g, false, 0,
    true, timeLimit)``: :func:`~mppflow.solvers.solve_time` without the LP stage, with
    ``hasTimeLimit = timeLimit > 0``, then the validity check. ``debug`` / ``print_paths``
    stand for the Time solver's static ``bDebugInfo`` / ``bPrintPaths`` (``true`` by default
    in Java).

    ``original``: ``p.sg == None`` (a failed ``readFromFile``) raises
    ``JavaNullPointerException``; ``fixed`` raises ``ValueError``.
    """
    orig = check_compat(compat)
    if p is None or p.starts is None or p.goals is None:
        if orig:
            raise JavaNullPointerException(_NPE_SG)
        raise ValueError("the problem has no starts/goals")
    if len(p.starts) >= PUZZLE_SOLVER_MIN_AGENTS:
        solver = PuzzleSolver(p.graph, p.starts, p.goals, compat=compat, backend=backend, params=params,
                              rng=rng, clock=clock, echo=echo, **puzzle_kwargs)
        return solver.solve(time_limit)
    from .heuristic import _Out as _HOut, _plan_paths_advanced

    out = _HOut(echo)
    paths, res = _plan_paths_advanced(p.graph, p.starts, p.goals, False, 0, True, time_limit, out=out, rng=rng,
                                      backend=backend, params=params, compat=compat, clock=clock,
                                      feasibility_budget=DEFAULT_FEASIBILITY_BUDGET, debug=debug,
                                      print_paths_=print_paths, catch_infeasible=False)
    if res is None:  # pragma: no cover - only with catch_infeasible
        return SolveResult(paths=None, status="infeasible", log=out.text())
    status = res.status if paths is not None else ("time_limit" if res.status == "time_limit" else
                                                   ("rejected" if res.paths is not None else res.status))
    return SolveResult(paths=paths, status=status, objective=res.objective if paths is not None else None,
                       T=res.T, lower_bound=res.lower_bound, log=out.text(), attempts=res.attempts,
                       extra=dict(res.extra))
