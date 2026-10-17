"""Solve loops: the original's ``solve`` methods, plus the horizon search of Algorithm 1
and the new min-max-distance solver.

Ports (original ``ILP/*.java``):

=====================================  ==================================================
function                               original
=====================================  ==================================================
:func:`solve_time`                     ``MultiagentGraphSolverGurobiTime.solve``
:func:`solve_time_r`                   ``MultiagentGraphSolverGurobiTimeR.solve``
:func:`solve_time_puzzle`              ``MultiagentGraphSolverGurobiTimePuzzle.solve`` (one T)
:func:`solve_total_time`               ``MultiagentGraphSolverGurobiTotalTime.solve``
:func:`solve_distance`                 ``MultiagentGraphSolverGurobiDistance.solve``
:func:`solve_max_distance`             *new* (MINMAXDIST, T-RO 2016 eqs. (11)-(13))
:func:`min_makespan`, ...              *new* convenience wrappers
:func:`joint_feasibility`              *new* exact feasibility check (joint-state BFS)
:func:`joint_optimum`                  *new* exact TT / DIST / max distance (joint-state search)
=====================================  ==================================================

Every solver returns a :class:`SolveResult`; ``result.paths`` is the original's return
value (``int[A][T+1]`` or ``None``). The text the original prints (its ``bDebugInfo`` /
``bPrintPaths`` messages, wall-clock numbers included) is collected in ``result.log``
(``debug=`` / ``print_paths=`` select the messages, ``echo=True`` also prints them);
nothing is printed by default. Gurobi's own log is not reproduced.

Backends: ``backend="highs"`` (default), ``"gurobi"`` (pass ``params={"Threads": 1}`` to
reproduce the golden fixtures exactly) or ``"builtin"``. ``compat="original"`` should be run
with ``backend="gurobi"``: its path extraction tests ``X == 1.0`` exactly, as the original
does, and HiGHS often returns binaries such as ``0.9999999999993``; the plan is then invalid
and :class:`~mppflow.model.InexactSolutionWarning` is issued. The original sets ``TimeLimit``
(``1e10`` when there is none; we pass no limit then) and, for TT/DIST, ``MIPGap``.

Randomness: :func:`solve_time`, :func:`solve_time_r` and :func:`solve_total_time` call
:func:`~mppflow.pathfinder.find_shortest_paths` exactly once at entry, as the original
does (the draws only break ties; the lengths give ``T_min``, ``sum`` of distances for TimeR,
and TT's printed lower bound). Nothing else draws. ``rng=None`` uses the module stream.

Infinite loops (``compat="original"``)
--------------------------------------
``Time.solve``/``TimeR.solve`` increase ``T`` without bound when the instance is infeasible
(no ``T_max``; ``docs/notes/original-bugs.md`` B1). After the first
failed horizon we decide feasibility with :func:`joint_feasibility` (exact BFS over joint
configurations under the model's rules, bounded by ``feasibility_budget``; plus the
certificate "two agents share a goal"). Both proofs cover the horizons from the loop's
first one, ``T_min + extra_steps``, on: without a wait loop at some goal the feasible horizons
need not be upward-closed (two robots staying at the ends of the path ``0 - 1 - 2`` without
wait loops have a plan at ``T = 0`` only), so "feasible for some ``T``" would not stop a loop
started above the feasible horizons. For TimeR the search allows one move per step when
every vertex id is below 22 (the original's one-move rows are then exact) and any number of
moves otherwise (the original rows miss arcs; still sound, but weaker). The proof covers the
horizons ``T < 10000``, on which the original's keys are exact. From ``T = 10000`` on its keys
collide (see :mod:`mppflow.model`) and its models are no longer the MPP model; they can even
become feasible (the ``compat="original"`` Time model of the 2-robot swap on one edge has value
``n`` at ``T = 10000``, the TimeR model of the triangle rotation at ``T = 10001``), so in
principle the original would stop there with a plan produced by its key collisions -- but only
after solving ~10^4 infeasible, ever larger ILPs (the triangle's TimeR model at ``T = 9998``
alone did not finish within 600 s under Gurobi). "Loops forever" is meant in this practical
sense. If the instance is proved infeasible:

* without a time limit, :class:`~mppflow.errors.OriginalWouldHang` is raised (the original
  never returns);
* with a time limit, the original keeps trying larger ``T`` until the limit expires and then
  returns ``null``; we return the ``null`` result at once (status ``"time_limit"``,
  ``extra["shortcut"] = True``), without waiting.

On a one-way graph the original can also loop forever on a **solvable** instance: its
reachability pruning measures the goal side forward from the goal (see
:func:`~mppflow.network.update_reachable_sets`) and can prune every horizon, e.g. the
one-way 4-cycle ``0 -> 1 -> 2 -> 3 -> 0`` without wait loops, ``0 -> 3`` (live Java was still
at "Trying T = 302" after 20 s). There the joint-state certificate
:func:`_original_pruning_dead` (backward search over joint configurations within the
original's time windows, ending when the windows stop binding and the sets repeat) proves
that no horizon works, and the same two outcomes apply. With wait loops at every vertex
this never happens (the windows open up for large ``T``).

When a check is inconclusive (budget exhausted) the loop continues, as in Java, and the
check is repeated at the 2nd, 4th, 8th, ... failed horizon with ``feasibility_budget`` times
the number of failed horizons. The search work therefore stays about ``2 *
feasibility_budget`` per failed horizon on average (earlier versions kept the first
inconclusive answer and tried ``T = 7, 8, ...`` forever on a 6-robot, 7-vertex instance with
840 reachable configurations). In ``fixed`` mode the checks also keep pace with the ILP work:
whenever the horizons solved so far took longer than all checks, the next failed horizon is
checked with twice the largest budget (only a proof of infeasibility is used from such a
check, so plans stay deterministic). The ILP's own infeasibility proofs get slow as ``T``
grows (a 16-vertex corridor with 5 robots that must reverse their order: under 1 s per horizon
up to ``T = 22``, then over 90 s for ``T = 23`` alone), while the search settles it in 0.2 s.

The proof is practical only when the joint state space is small, or when a polynomial
certificate of :func:`joint_feasibility` applies (robots that must pass each other on a
path component or change their cyclic order on a cycle, a robot that must cross a bridge of a
fully occupied component, or one connected component whose own search fails). An
infeasible instance outside these cases (say, many robots that must reorder on a large tree
with too few free vertices) can make a ``fixed``-mode loop without
``t_max`` or a time limit run for a very long time; use ``t_max`` or ``time_limit`` to bound
it.

With a time limit, the shortcut is not taken while the LP relaxation (``solve_relaxed_first``)
is still infeasible: the original keeps raising ``T`` there and prints a ``Trying T = ...``
line per horizon, and the relaxation of an infeasible instance typically becomes feasible
after a few horizons, so those lines are deterministic. We keep solving the relaxations
until one is feasible (then the integer loop is cut short at its first failure; it prints
nothing unless ``debug``), and cut short in the relaxed phase only when the relaxation itself
is proved infeasible for every horizon (two robots share a goal, or one robot alone has no
path in the original's pruned network), or after ``|V| + n`` further horizons (a guard for
clocks that never expire).

compat modes
------------
``compat="fixed"`` equals the original wherever the original is correct: the same
``T_min``, the same random draws, the same models in the same order (hence, under Gurobi
with the same version and ``Threads``, the same plans). Differences of ``fixed``:

* Reachability pruning on one-way graphs: the goal-side distance is ``dist(v, g)`` (BFS
  over the reversed arcs) instead of the original's ``dist(g, v)``, which prunes vertices
  a robot must pass through. The original then returns non-optimal makespans (the one-way
  4-cycle ``0 -> 3``: 5 instead of 3), proves feasible single-horizon TT/DIST models
  infeasible, or never stops (see above). On symmetric graphs both modes prune alike.
* Instance checks: duplicate starts raise ``ValueError`` (the original returns a plan that
  collides at ``t = 0``, original-bugs B4); duplicate goals and goals
  unreachable from their start (along the possibly one-way adjacency) raise
  :class:`~mppflow.errors.Infeasible` (the original loops forever, or throws
  ``NullPointerException`` after its shortest-path search).
* Makespan loop (:func:`solve_time`, :func:`solve_time_r`): proved infeasibility and
  exceeding ``t_max`` (also by the first horizon ``T_min + extra_steps``) raise
  :class:`~mppflow.errors.Infeasible` instead of looping; the
  remaining time is ``time_limit - elapsed`` (the original subtracts the cumulative elapsed
  time again at every iteration and checks ``limit - elapsed`` once more); a non-positive
  ``time_limit`` means no limit, as in the original's signature (the original returns
  ``null`` at the first failed horizon, original-bugs B2), and NaN raises ``ValueError``
  (:func:`~mppflow.planner.plan` and the ``min_*`` functions accept only ``None`` or a
  limit > 0); a feasible relaxation is accepted without a time check (the original
  returns ``null`` with "relaxed model not feasible, time limit reached." when the limit
  expired meanwhile); ``ObjVal`` is rounded (the original
  truncates with ``(int)``) and only the current horizon's value counts.
* Limits (``fixed``): a solve that stops at a node, solution or work limit given in
  ``params``, or with an incumbent outside the gap, decides nothing: the search stops with
  status ``"time_limit"`` instead of counting the horizon as infeasible (the original has no
  such limits). Model building counts against ``time_limit``. With ``backend="builtin"`` the
  makespan loop returns the plan of its joint-state search after a failed horizon (see
  ``docs/notes/solver-notes.md``, sections 4 and 5).
* Path extraction: ``X > 0.5`` (the original tests ``X == 1.0`` exactly and silently
  continues from vertex 0 when no arc matches).
* TT/DIST acceptance: a solution is accepted when the solver proves it optimal within
  ``gap``, or when its gap -- measured on the total time for TT -- is at most ``gap``. For TT
  the backend gets the gap :func:`_tt_model_gap` on the model's objective ``n*T - TT``, so
  that its "optimal" is within ``gap`` on the total time (the original passes ``gap`` itself,
  a far larger gap on the total time on long horizons). The
  original computes ``bnd/val - 1`` (TT, on the sum of goal-waiting steps) and
  ``1 - bnd/objVal`` (DIST), which is ``NaN`` for an optimal objective of 0 and rejects
  the optimum (original-bugs B3); a rejected or infeasible model is reported as
  " time out." there. A solution accepted within ``gap > 0`` gets status ``"feasible"``
  unless its bound closes (see :class:`SolveResult`).
* TT/DIST horizon: ``horizon="search"`` (the default in ``fixed``) follows Algorithm 1
  (see :func:`solve_total_time`); ``horizon="single"`` (the only choice in ``original``)
  solves at the given ``T`` once, as the original does (``docs/notes/paper-vs-code.md``,
  "TT and DIST are solved at one horizon"); in ``fixed`` mode its status is
  ``"optimal"`` only when the plan meets the lower bound.
* Input checks (``fixed``): ``gap`` must be a finite number >= 0 (``ValueError``; the
  original passes it to Gurobi, which rejects a negative one). Both modes convert numpy
  integer vertex ids to ``int`` and raise ``TypeError`` for ``bool`` / float ids, and
  ``feasibility_budget`` must be >= 1 (``ValueError``).
* Goals without a wait self-loop (``fixed``): the TT/DIST/max-distance horizon search solves
  every horizon from the optimal makespan to ``T_hi`` (:func:`_search_every`), since such
  plans do not extend by waiting.
* ``t_max`` (``fixed``) bounds the makespan in every solver: no horizon above it is solved
  (``horizon="single"`` solves at ``min(T, t_max)``) and :class:`~mppflow.errors.Infeasible`
  is raised when no plan fits.
* :func:`solve_max_distance` is new; ``original`` raises
  :class:`~mppflow.errors.NotInOriginal`.
* :func:`solve_time_puzzle` does not write the original's Gurobi log files
  (``C:\\Temp\\25puzzle-instance-<k>``) in either mode.
* Zero agents: the original's ``while (val != n)`` loop never runs and ``solve`` returns
  ``null``; TT rejects its 0/0 gap. ``fixed`` returns the empty plan.
"""
from __future__ import annotations

import math
import numbers
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Sequence

from . import lp
from ._java import MathRandom, java_double_str
from .errors import (
    Infeasible,
    JavaGRBException,
    JavaNullPointerException,
    NotInOriginal,
    OriginalWouldHang,
)
from .graph import Graph, as_ids, as_int, check_compat, java_d2i, print_vertex
from .model import (
    BuildStopped,
    MPPModel,
    build_distance_model,
    build_max_distance_model,
    build_time_model,
    build_time_puzzle_model,
    build_time_r_model,
    build_total_time_model,
)
from .network import Reachability, _distances_to, _reversed_adjacency, update_reachable_sets
from .pathfinder import find_shortest_paths, get_distances_to_all_vertices

__all__ = [
    "JavaGRBException",
    "SolveResult",
    "joint_feasibility",
    "joint_optimum",
    "solve_time",
    "solve_time_r",
    "solve_time_puzzle",
    "solve_total_time",
    "solve_distance",
    "solve_max_distance",
    "min_makespan",
    "min_total_time",
    "min_total_distance",
    "min_max_distance",
    "paths_text",
    "DEFAULT_FEASIBILITY_BUDGET",
]

DEFAULT_FEASIBILITY_BUDGET = 50000
"""Default work budget of :func:`joint_feasibility` (joint states plus partial moves)."""


# ---------------------------------------------------------------------- results
@dataclass
class SolveResult:
    """The outcome of a solve.

    Attributes:
        paths: ``paths[a][t]`` (all of length ``T + 1``), or ``None`` (the original's
            ``null``: time out, infeasible model, rejected solution).
        status: ``"optimal"``, ``"feasible"`` (accepted without an optimality proof, e.g.
            ``return_any_solution``), ``"time_limit"`` (a limit stopped the search before it
            decided: the time limit or, in ``fixed`` mode, a node, solution or work limit
            given in ``params``, or an incumbent outside the gap; ``extra["solver_status"]``
            then holds the backend's status for the makespan loop), ``"infeasible"`` (the
            model at this horizon is proved to have no solution) or ``"rejected"`` (a solution was found but the
            acceptance test failed; ``original`` only). In ``fixed`` mode ``"optimal"`` means
            proved optimal for the objective (among the plans with makespan <= ``t_max`` when
            given): a solve with ``gap > 0`` proves only its gap unless the bound closes; a
            single TT/DIST/max-distance horizon is optimal at that ``T`` only, so it reports
            ``"feasible"`` with ``extra["optimal_at_T"]`` unless it meets the lower bound; a
            makespan loop started at ``T_min + extra_steps`` reports ``"feasible"`` unless it
            ends at ``T_min``, and so does a single puzzle horizon above ``T_min``. In ``original`` mode the status is that of the last model solved.
        objective: makespan (Time/TimeR/Puzzle), total arrival time (TT), total distance
            (DIST), max distance; ``None`` without a solution.
        T: the horizon of ``paths`` (the last horizon tried when ``paths`` is ``None``).
        lower_bound: ``T_min`` (makespan), the sum of distances (TimeR, TT, DIST) or the max
            distance (max distance), when computed.
        log: the text the original would print (see the module docstring).
        attempts: one entry per model solved: ``{"T", "phase" ("lp" | "ilp"), "status",
            "objective", "bound", "runtime"}``; a makespan plan taken from the joint-state
            search (``backend="builtin"``, ``extra["proof"]``) adds one with ``phase``
            ``"joint"`` at its horizon (no model, ``runtime`` ``None``).
        extra: details (``shortcut``, ``horizon_search``, ``failed_attempt``, ...).
    """

    paths: list[list[int]] | None
    status: str
    objective: float | None = None
    T: int | None = None
    lower_bound: int | None = None
    log: str = ""
    attempts: list[dict] = field(default_factory=list)
    extra: dict = field(default_factory=dict)

    @property
    def makespan(self) -> int | None:
        if self.paths is None:
            return None
        return len(self.paths[0]) - 1 if self.paths else 0


class _Out:
    def __init__(self, echo: bool):
        self.parts: list[str] = []
        self.echo = echo

    def print(self, s: str) -> None:
        self.parts.append(s)
        if self.echo:
            sys.stdout.write(s)

    def println(self, s: str = "") -> None:
        self.print(s + "\n")

    def text(self) -> str:
        return "".join(self.parts)


def _jds(d: float) -> str:
    return java_double_str(float(d))


def paths_text(graph: Graph, paths: Sequence[Sequence[int]]) -> str:
    """What ``retrievePaths`` prints when ``bPrintPaths`` is set:
    ``"Agent a: 0:<v> 1:<v> ...\\n"`` per agent, ``<v>`` = ``Vertex.printVertex()``."""
    out = []
    for a, p in enumerate(paths):
        parts = [f"Agent {a}: 0:" + _pv(graph, p[0])]
        for t in range(1, len(p)):
            parts.append(f" {t}:" + _pv(graph, p[t]))
        out.append("".join(parts) + "\n")
    return "".join(out)


def _pv(graph: Graph, v: int) -> str:
    vid = graph.vertices[v] if 0 <= v < len(graph.vertices) else v
    return print_vertex(graph, vid)


class _Clock:
    """Elapsed time since construction: Java ``currentTimeMillis`` granularity in
    ``original`` mode, float seconds otherwise."""

    def __init__(self, clock: Callable[[], float] | None, orig: bool):
        self.f = clock or time.time
        self.orig = orig
        self.t0 = self._now()

    def _now(self):
        t = self.f()
        return int(t * 1000) if self.orig else t

    def elapsed(self) -> float:
        d = self._now() - self.t0
        return d / 1000.0 if self.orig else d


def _attempt(T, phase, sol) -> dict:
    rec = {"T": T, "phase": phase, "status": sol.status, "objective": sol.objective, "bound": sol.bound,
           "runtime": sol.runtime}
    if sol.raw_status == "iteration_cap":  # builtin: a stalled simplex, reported as "time_limit"
        rec["raw_status"] = sol.raw_status
    return rec


def _joint_attempt(T, n) -> dict:
    """The attempt record of a makespan plan taken from the joint-state search (no model was
    solved at ``T``): ``phase`` ``"joint"``, value ``n`` (every robot reaches its goal)."""
    return {"T": T, "phase": "joint", "status": "optimal", "objective": n, "bound": None, "runtime": None}


def _run(m: MPPModel, backend: str, params: dict | None, time_limit: float | None,
         mip_gap: float | None):
    tl = time_limit if (time_limit is not None and time_limit > 0) else None
    return lp.solve(m.model, backend=backend, time_limit=tl, mip_gap=mip_gap, params=params)


def _refutes(sol, n: int | None = None) -> bool:
    """``fixed``: does a makespan-type solve without a plan of value ``n`` prove that its
    horizon has no plan? Only a proved status does: ``"infeasible"``, or ``"optimal"`` with a
    smaller value. A node or solution limit (``params``), a time limit, or an incumbent below
    ``n`` without an optimality proof decides nothing, and the search must not move on to
    ``T + 1`` as if it did (it would report a too-large makespan as optimal, or never stop).
    With ``n`` given, an ``"optimal"`` whose bound still admits the value ``n`` decides
    nothing either (a tolerance in ``params``, such as Gurobi's ``MIPGapAbs``, stopped the
    solve early)."""
    if sol.status == "optimal" and n is not None and sol.bound is not None and sol.bound > n - 0.5:
        return False
    return sol.status in ("infeasible", "optimal")


def _undecided_status(sol) -> str:
    """``fixed``: the status of a solve that ended at a limit without a decision. A node,
    solution or work limit given in ``params`` is reported like a time limit
    (``"time_limit"``: a solver limit stopped the search); ``extra["solver_status"]`` of the
    result, and the attempt records, keep the backend's status."""
    return "time_limit"


def _jdiv(a: float, b: float) -> float:
    """Java ``double`` division (IEEE: x/0 = +-inf, 0/0 = NaN)."""
    if b == 0.0:
        if a == 0.0 or math.isnan(a):
            return math.nan
        neg = (a < 0) != (math.copysign(1.0, b) < 0)
        return -math.inf if neg else math.inf
    return a / b


# ----------------------------------------------------------- instance checks
def _check_budget(budget) -> None:
    """``feasibility_budget`` must be a number >= 1: with 0 or less every joint-state proof
    is inconclusive (the doubling schedule ``budget * failures`` never grows), and the
    makespan loop would try ``T = T_min, T_min + 1, ...`` forever on an infeasible
    instance. ``math.inf`` means no limit."""
    if isinstance(budget, bool) or not isinstance(budget, numbers.Real) or not budget >= 1:
        raise ValueError(f"feasibility_budget must be a number >= 1, got {budget!r}")


def _check_horizon_arg(x, what: str):
    """A horizon argument (``T``, ``t_max``, ``extra_steps``): ``None`` or an integer
    (``numpy`` integers are converted); ``bool``, floats (also NaN and integral ones) and
    other types raise :class:`TypeError`. Without the check a fractional ``t_max`` reached
    the horizon bound ``T_hi`` and crashed (or not) depending on the code path, and NaN
    silently meant no bound. A negative ``T``, ``t_max`` or ``t_start`` raises
    ``ValueError`` naming the argument (it used to give ``Infeasible``, a ``ValueError``
    about ``T`` from the network builder, or, with no robots, an "optimal" empty plan,
    depending on the path); ``extra_steps`` keeps the original's ``int``."""
    if x is None:
        return None
    x = as_int(x, what)
    if x < 0 and what != "extra_steps":
        raise ValueError(f"{what} must be >= 0, got {x}")
    return x


def _check_gap(gap) -> None:
    """``fixed``: the MIP gap must be a finite number >= 0 (a negative gap raises in Gurobi
    but only warns in HiGHS, and NaN / inf were accepted silently)."""
    if isinstance(gap, bool) or not isinstance(gap, numbers.Real) or not 0 <= gap < math.inf:
        raise ValueError(f"gap must be a finite number >= 0, got {gap!r}")


def _check_time_limit_sentinel(time_limit) -> None:
    """``fixed``, the solvers with the original's signature (``time_limit=-1.0``): a time
    limit ``<= 0`` means no limit, as in the original; it must be a number, and NaN (which
    compares false with everything, so it used to mean no limit too) raises ``ValueError``."""
    if isinstance(time_limit, bool) or not isinstance(time_limit, numbers.Real) or math.isnan(time_limit):
        raise ValueError(f"time_limit must be a number of seconds (<= 0: no limit), got {time_limit!r}")


def _check_time_limit(time_limit) -> float | None:
    """``fixed``, :func:`~mppflow.planner.plan` and the ``min_*`` functions: ``None`` is the
    only "no limit"; anything else must be a finite number of seconds ``> 0``
    (``ValueError`` otherwise; a limit of 0, a negative one or NaN used to mean no limit,
    the opposite of a tiny positive one). Returns the limit as a float, or ``None``."""
    if time_limit is None:
        return None
    if (isinstance(time_limit, bool) or not isinstance(time_limit, numbers.Real)
            or not 0 < time_limit < math.inf):
        raise ValueError(f"time_limit must be None (no limit) or a finite number of seconds > 0, "
                         f"got {time_limit!r}")
    return float(time_limit)


def _check_instance(graph: Graph, starts, goals) -> None:
    """``fixed``-mode input validation (see the module docstring)."""
    if len(starts) != len(goals):
        raise ValueError("starts and goals must have the same length")
    n = graph.n
    if any(v != i for i, v in enumerate(graph.vertices)) or len(graph.adj) != n:
        raise ValueError("the solvers need vertex ids 0..n-1 in order (the grid generators with holes "
                         "return 1-based ids with gaps): call convert_graph first")
    for v in list(starts) + list(goals):
        if not 0 <= v < n:
            raise ValueError(f"vertex {v} out of range 0..{n - 1}")
    if len(set(starts)) < len(starts):
        raise ValueError("two robots share a start vertex")
    if len(set(goals)) < len(goals):
        raise Infeasible("two robots share a goal vertex")


def _distances(graph: Graph, starts, goals) -> list[int | None]:
    """BFS distance from each robot's start to its goal along the (possibly one-way)
    adjacency (``None`` if the goal is unreachable); no random draws.

    The search starts at the start vertex: on a graph whose adjacency is not symmetric
    (:meth:`Graph.from_adjacency` adds no reverse edges) the distance from the goal back
    to the start can differ or be undefined."""
    out = []
    for s, g in zip(starts, goals):
        d = get_distances_to_all_vertices(graph, s, None, "fixed", unreachable=-1)
        out.append(None if d[g] < 0 else d[g])
    return out


def joint_feasibility(graph: Graph, starts: Sequence[int], goals: Sequence[int],
                      budget: int = DEFAULT_FEASIBILITY_BUDGET, one_move: bool = False,
                      t_first: int = 0) -> bool | None:
    """Is the compact makespan model feasible for **some** horizon ``T >= t_first``?

    Exact breadth-first search over joint configurations with the model's rules: each
    robot follows an arc of the graph (waiting needs the self-loop), the configuration at
    every ``t >= 1`` is injective (vertex capacity), and no two robots swap along an edge
    (head-on); rotations along cycles are allowed. The start configuration may repeat a
    vertex (the model does not constrain ``t = 0``). With ``one_move=True`` at most one
    robot moves per step (the TimeR model).

    ``t_first`` is the first horizon of the caller's makespan loop (``T_min +
    extra_steps``). When every goal has its self-loop, a plan for some ``T`` extends to every
    larger horizon by waiting at the goals, so ``t_first`` does not matter. Otherwise the
    feasible horizons need not be upward-closed (a robot at its goal on a one-way cycle
    without wait loops has plans only at multiples of the cycle length; the path ``0 - 1 -
    2`` without wait loops has none at ``T >= 1`` for two robots staying at ``0`` and ``2``),
    and the search runs over ``(configuration, min(t, t_first))``.

    Before the search (distinct starts), polynomial certificates can answer ``False`` on
    instances whose joint state space is far too large for it (see
    :func:`_structural_refutation`): a robot that must pass another on a component that is a
    path, robots that must change their cyclic order on a cycle, or a robot that must cross a
    bridge of a fully occupied component. With robots on several connected components, each
    component is also searched alone (``budget`` each): one without a plan decides the
    instance.

    Returns ``True`` / ``False``, or ``None`` when ``budget`` (joint states plus partial
    move assignments examined) runs out. Never draws random numbers.
    """
    return _joint_bfs(graph, starts, goals, budget, one_move, t_first)[0]


def _joint_bfs(graph, starts, goals, budget, one_move, t_first):
    """:func:`joint_feasibility` with its witness: ``(verdict, T, paths)``. When the search
    reaches the goal configuration, ``T`` is the smallest feasible horizon ``>= t_first``
    (the breadth-first depth of the first goal configuration) and ``paths`` a plan of
    makespan ``T`` (``paths[a][t]``) that follows the model's rules; otherwise both are
    ``None``. The makespan loop returns this plan instead of proving each horizon below
    ``T`` infeasible by branch and bound (exponential in ``T`` on the builtin backend)."""
    A = len(starts)
    start, goal = tuple(starts), tuple(goals)
    # every plan has T >= max(dist): a t_first up to that bound restricts nothing
    layered = (t_first > 0 and any(g not in (graph.neighbors(g) or ()) for g in goal)
               and (start == goal or t_first > max(d or 0 for d in _distances(graph, starts, goals))))
    if start == goal and not layered:
        return True, None, None
    if len(set(goal)) < A:
        return False, None, None
    dist = _distances(graph, starts, goals)
    if any(d is None for d in dist):
        return False, None, None
    if len(set(start)) == A:
        # polynomial certificates and the per-component search, before the joint search
        # (whose state space is the product of the components')
        und = _undirected(graph, start + goal)
        comp, comps = _components(und)
        if (_structural_refutation(und, comp, comps, starts, goals) is not None
                or _component_refutes(graph, comp, comps, starts, goals, budget, one_move, t_first)):
            return False, None, None
    nbrs: dict[int, list[int]] = {}

    def nb(v):
        r = nbrs.get(v)
        if r is None:
            r = list(graph.neighbors(v) or ())
            nbrs[v] = r
        return r

    work = [0]
    K = t_first if layered else 0
    seen = {(start, 0): None}  # state -> parent state
    queue = deque([(start, 0, 0)])

    def witness(st, last, t_goal):
        cfgs = [last]
        while st is not None:
            cfgs.append(st[0])
            st = seen[st]
        cfgs.reverse()
        cfgs.extend([last] * (t_goal + 1 - len(cfgs)))  # T = t_first > depth: wait at the goals
        return [[c[a] for c in cfgs] for a in range(A)]

    def successors(cfg):
        occ: dict[int, list[int]] = {}
        for i, v in enumerate(cfg):
            occ.setdefault(v, []).append(i)
        target = [0] * A
        used: set[int] = set()

        def rec(i, moved):
            work[0] += 1
            if work[0] > budget:
                raise _Budget()
            if i == A:
                yield tuple(target)
                return
            u = cfg[i]
            for w in nb(u):
                if w in used:
                    continue
                if w != u:
                    if one_move and moved:
                        continue
                    swap = False
                    for j in occ.get(w, ()):
                        if j < i and target[j] == u:
                            swap = True
                            break
                    if swap:
                        continue
                target[i] = w
                used.add(w)
                yield from rec(i + 1, moved or w != u)
                used.discard(w)

        yield from rec(0, False)

    try:
        while queue:
            cfg, k, t = queue.popleft()
            k2 = min(k + 1, K)
            for nxt in successors(cfg):
                if nxt == goal and k2 >= K:
                    T = max(t + 1, t_first)
                    return True, T, witness((cfg, k), nxt, T)
                st = (nxt, k2)
                if st not in seen:
                    seen[st] = (cfg, k)
                    work[0] += 1
                    queue.append((nxt, k2, t + 1))
    except _Budget:
        return None, None, None
    return False, None, None


class _Budget(Exception):
    pass


def _undirected(graph: Graph, extra=()) -> dict[int, set[int]]:
    """The underlying simple undirected graph (self-loops dropped, one-way arcs made
    two-way): ``{v: set of neighbours}``; ``extra`` vertices are added (isolated if absent)."""
    und: dict[int, set[int]] = {}
    for v in graph.adj:
        und.setdefault(v, set())
        for w in graph.neighbors(v) or ():
            if w != v:
                und[v].add(w)
                und.setdefault(w, set()).add(v)
    for v in extra:
        und.setdefault(v, set())
    return und


def _components(und: dict[int, set[int]]):
    """Connected components of ``und``: ``(comp, comps)`` with ``comp[v]`` the index of the
    component of ``v`` and ``comps[c]`` its vertices (ascending)."""
    comp: dict[int, int] = {}
    comps: list[list[int]] = []
    for v in sorted(und):
        if v in comp:
            continue
        c = len(comps)
        comp[v] = c
        members, stack = [v], [v]
        while stack:
            u = stack.pop()
            for w in und[u]:
                if w not in comp:
                    comp[w] = c
                    members.append(w)
                    stack.append(w)
        comps.append(sorted(members))
    return comp, comps


def _line_order(und, C, cyc: bool) -> dict[int, int]:
    """The position of every vertex of the path (``cyc=False``) or cycle component ``C``."""
    cur = C[0] if cyc else next(v for v in C if len(und[v]) == 1)
    pos = {cur: 0}
    while True:
        nxt = [w for w in und[cur] if w not in pos]
        if not nxt:
            return pos
        cur = min(nxt)
        pos[cur] = len(pos)


def _two_edge_labels(und, C) -> dict[int, int]:
    """The 2-edge-connected component of every vertex of the component ``C`` (the
    components left after removing the bridges; iterative Tarjan)."""
    disc: dict[int, int] = {}
    low: dict[int, int] = {}
    bridges: set[tuple[int, int]] = set()
    root = C[0]
    disc[root] = low[root] = 0
    stack = [(root, None, iter(sorted(und[root])))]
    while stack:
        u, parent, it = stack[-1]
        for w in it:
            if w == parent:
                continue
            if w in disc:
                low[u] = min(low[u], disc[w])
            else:
                disc[w] = low[w] = len(disc)
                stack.append((w, u, iter(sorted(und[w]))))
                break
        else:
            stack.pop()
            if stack:
                p = stack[-1][0]
                low[p] = min(low[p], low[u])
                if low[u] > disc[p]:
                    bridges.add((min(p, u), max(p, u)))
    lab: dict[int, int] = {}
    for v in C:
        if v in lab:
            continue
        lab[v] = v
        st = [v]
        while st:
            u = st.pop()
            for w in und[u]:
                if w not in lab and (min(u, w), max(u, w)) not in bridges:
                    lab[w] = v
                    st.append(w)
    return lab


def _structural_refutation(und, comp, comps, starts, goals) -> str | None:
    """Sound polynomial certificates that the model has no plan for any horizon (distinct
    starts, distinct goals, each goal reachable from its start). Returns the reason, or
    ``None`` when no certificate applies. Robots never leave the connected component of the
    underlying undirected graph they start in, and under the model's rules (vertex capacity,
    no head-on swap) two robots never pass each other on a component that is a path, and
    keep their cyclic order on one that is a cycle. On a fully occupied component every
    step is a set of rotations along cycles of length >= 3 (a chain of moves into occupied
    vertices must close, and a 2-cycle is a swap), so a robot never crosses a bridge: its
    start and goal must lie in the same 2-edge-connected component (on a tree, nothing
    moves). All of this holds with any subset of the arcs and with one move per step, so
    the certificates are sound for one-way graphs and for TimeR too."""
    robots: dict[int, list[int]] = {}
    for i, (s, g) in enumerate(zip(starts, goals)):
        if comp[s] != comp[g]:
            return f"robot {i} must leave its connected component"
        robots.setdefault(comp[s], []).append(i)
    for c in sorted(robots):
        rs = robots[c]
        if all(starts[i] == goals[i] for i in rs):
            continue
        C = comps[c]
        k = len(C)
        deg = [len(und[v]) for v in C]
        E = sum(deg) // 2
        if k >= 2 and max(deg) <= 2 and E in (k - 1, k):
            cyc = E == k
            pos = _line_order(und, C, cyc)
            order = sorted(rs, key=lambda i: pos[starts[i]])
            gp = [pos[goals[i]] for i in order]
            m = len(gp)
            if not cyc:
                for j in range(m - 1):
                    if gp[j] > gp[j + 1]:
                        return (f"robots {order[j]} and {order[j + 1]} must pass each other on a component "
                                "that is a path")
            elif m >= 3 and sum(gp[j] > gp[(j + 1) % m] for j in range(m)) > 1:
                return "the robots on a component that is a cycle must change their cyclic order"
        if len(rs) == k:
            lab = _two_edge_labels(und, C)
            for i in rs:
                if lab[starts[i]] != lab[goals[i]]:
                    return (f"robot {i} must cross a bridge of a fully occupied component (only rotations "
                            "along cycles are possible there)")
    return None


def _component_refutes(graph, comp, comps, starts, goals, budget, one_move, t_first) -> bool:
    """With robots on two or more connected components: is the joint search of some
    component alone (its own ``budget``) a proof that no horizon ``>= t_first`` has a plan?
    The components do not interact, so a plan restricted to one component is a plan of its
    robots there with the same horizon (also with one move per step)."""
    robots: dict[int, list[int]] = {}
    for i, s in enumerate(starts):
        robots.setdefault(comp[s], []).append(i)
    if len(robots) < 2:
        return False
    for c in sorted(robots):
        rs = robots[c]
        C = comps[c]
        idx = {v: j for j, v in enumerate(C)}
        sub = Graph()
        for j in range(len(C)):
            sub.xy[j] = None
        sub.adj = {idx[v]: sorted(idx[w] for w in (graph.neighbors(v) or ())) for v in C}
        sub.finish_building_graph()
        v = _joint_bfs(sub, [idx[starts[i]] for i in rs], [idx[goals[i]] for i in rs], budget, one_move,
                       t_first)[0]
        if v is False:
            return True
    return False


JOINT_SEARCH_BUDGET = 2_000_000
"""Work budget of :func:`joint_optimum` in the horizon search (joint states plus partial move
assignments; about 1-3 s of Python)."""
JOINT_OPTIMUM_MAX_ROBOTS = 30
""":func:`joint_optimum` returns ``None`` above this many robots (the JS port's robot sets
are 32-bit masks)."""
JOINT_SEARCH_MAX_CONFIGS = 1_000_000
"""The horizon search tries :func:`joint_optimum` only when ``|V|! / (|V| - n)!`` (the number
of injective configurations) is at most this."""


def _joint_moves(nb, cfg, frozen, work, budget):
    """Valid joint moves from ``cfg`` (the rules of :func:`joint_feasibility`): yields
    ``(next_cfg, moved)`` with ``moved`` the bit mask of the robots that move. Robots in the
    bit mask ``frozen`` wait (and need their self-loop)."""
    A = len(cfg)
    occ: dict[int, list[int]] = {}
    for i, v in enumerate(cfg):
        occ.setdefault(v, []).append(i)
    target = [0] * A
    used: set[int] = set()

    def rec(i, moved):
        work[0] += 1
        if work[0] > budget:
            raise _Budget()
        if i == A:
            yield tuple(target), moved
            return
        u = cfg[i]
        for w in nb(u):
            if w in used:
                continue
            if w != u:
                if (frozen >> i) & 1:
                    continue
                swap = False
                for j in occ.get(w, ()):
                    if j < i and target[j] == u:
                        swap = True
                        break
                if swap:
                    continue
            target[i] = w
            used.add(w)
            yield from rec(i + 1, moved | ((w != u) << i))
            used.discard(w)

    yield from rec(0, 0)


def joint_optimum(graph: Graph, starts: Sequence[int], goals: Sequence[int], objective: str,
                  budget: int = JOINT_SEARCH_BUDGET, upper: int | None = None):
    """Exact optimum over **all** horizons by search over joint configurations, with the
    model's rules (as :func:`joint_feasibility`). No random draws.

    ``objective``: ``"total_time"`` (A* over ``(configuration, F)``, ``F`` the robots that
    have arrived for good and wait at their goals from then on; a step costs the number of
    robots not in ``F``), ``"distance"`` (A* over configurations; a step costs the number of
    robots that move) or ``"max_distance"`` (breadth-first search over ``(configuration,
    moves per robot)`` for the bound ``upper - 1``, then lower, until no plan exists, keeping
    only the Pareto-minimal move vectors of each configuration; needs ``upper``, the value
    of a known plan). The A* searches use the sum of the remaining
    shortest-path distances (consistent) and break cost ties by the number of steps, so the
    returned plan has the smallest makespan among the optimal ones.

    Goals without a wait self-loop are allowed: such a robot cannot be frozen at its goal
    before the last step (it has no move then), so the searches return the optimum over all
    horizons for these graphs too (earlier versions returned ``None`` for them). ``None`` is
    returned when ``budget`` runs out, and for more than ``JOINT_OPTIMUM_MAX_ROBOTS`` (30)
    robots unless the start is the goal configuration or two goals coincide (the JS port
    keeps robot sets in 32-bit masks; the horizon search calls this only for far fewer
    robots). Returns ``(value, paths)`` (``paths[i][t]``, trailing idle steps removed; for
    ``"max_distance"`` the plan is ``None`` when ``upper`` is already optimal) or ``False``
    when no plan exists.
    """
    import heapq

    A = len(starts)
    start, goal = tuple(starts), tuple(goals)
    if start == goal:
        return 0, [[v] for v in starts]
    if len(set(goal)) < A:
        return False
    if A > JOINT_OPTIMUM_MAX_ROBOTS:
        return None
    # dist[i][v] = dist(v, g_i) (0 where g_i is unreachable: still a lower bound)
    rev = _reversed_adjacency(graph)
    if rev is None:
        dist = [get_distances_to_all_vertices(graph, g, None, "fixed") for g in goals]
    else:  # one-way graph: distances to the goal along the arcs
        dist = [_distances_to(rev, g, graph.n) for g in goals]
    nbrs: dict[int, list[int]] = {}

    def nb(v):
        r = nbrs.get(v)
        if r is None:
            r = list(graph.neighbors(v) or ())
            nbrs[v] = r
        return r

    work = [0]

    def paths_of(parent, st, cfg_of):
        cfgs = []
        while st is not None:
            cfgs.append(cfg_of(st))
            st = parent[st]
        cfgs.reverse()
        return _trim([[c[i] for c in cfgs] for i in range(A)])

    try:
        if objective in ("total_time", "distance"):
            tt = objective == "total_time"
            full = (1 << A) - 1

            def h(cfg, F):
                return sum(dist[i][cfg[i]] for i in range(A) if not (F >> i) & 1)

            def closures(cfg, F):
                if not tt:
                    yield F
                    return
                at = [i for i in range(A) if cfg[i] == goal[i] and not (F >> i) & 1]
                for r in range(1 << len(at)):
                    # with k robots at their goals there are 2^k closure states, and the check
                    # in _joint_moves came only after all of them were on the heap (24 robots:
                    # 2.4 GB with budget 1000): one enumeration may not exceed what is left of
                    # the budget (pushed states are counted below; the closure r = 0 comes with
                    # every move, so a search that fits its budget is unchanged)
                    if r and work[0] + r > budget:
                        raise _Budget()
                    m = F
                    for k, i in enumerate(at):
                        if (r >> k) & 1:
                            m |= 1 << i
                    yield m

            best: dict = {}
            parent: dict = {}
            pq: list = []
            tie = 0
            for F0 in closures(start, 0):
                st = (start, F0)
                best[st] = (0, 0)
                parent[st] = None
                heapq.heappush(pq, (h(start, F0), 0, tie, st))
                tie += 1
            done = set()
            while pq:
                _, t, _, st = heapq.heappop(pq)
                if st in done:
                    continue
                done.add(st)
                cfg, F = st
                c = best[st][0]
                if (tt and F == full) or (not tt and cfg == goal):
                    return c, paths_of(parent, st, lambda x: x[0])
                cost = A - bin(F).count("1")
                for nxt, moved in _joint_moves(nb, cfg, F if tt else 0, work, budget):
                    if tt:
                        nc = c + cost
                    else:
                        if moved == 0:
                            continue
                        nc = c + bin(moved).count("1")
                    for F2 in closures(nxt, F):
                        st2 = (nxt, F2)
                        key = (nc, t + 1)
                        old = best.get(st2)
                        if old is None or key < old:
                            best[st2] = key
                            parent[st2] = st
                            work[0] += 1
                            heapq.heappush(pq, (nc + h(nxt, F2), t + 1, tie, st2))
                            tie += 1
            return False
        if objective == "max_distance":
            if upper is None:
                raise ValueError("objective='max_distance' needs upper")
            lb = max(dist[i][starts[i]] for i in range(A))
            found = None
            D = int(upper) - 1
            while D >= lb:
                st0 = (start, (0,) * A)
                parent = {st0: None}
                # front[cfg]: the Pareto-minimal move vectors that reach cfg. A state whose
                # vector is dominated (componentwise >=) by another of the same configuration
                # has no better continuation, so it is not expanded (without this pruning the
                # (cfg, moves) space blows up even on 8 vertices and 5 robots).
                front = {start: [st0[1]]}
                dead = set()
                q = deque([st0])
                hit = None
                while q and hit is None:
                    st = q.popleft()
                    if st in dead:
                        continue
                    cfg, mv = st
                    for nxt, moved in _joint_moves(nb, cfg, 0, work, budget):
                        if moved == 0:
                            continue
                        m2 = tuple(mv[i] + ((moved >> i) & 1) for i in range(A))
                        if any(m2[i] + dist[i][nxt[i]] > D for i in range(A)):
                            continue
                        st2 = (nxt, m2)
                        if st2 in parent:
                            continue
                        fr = front.setdefault(nxt, [])
                        if any(all(f[i] <= m2[i] for i in range(A)) for f in fr):
                            continue
                        keep = []
                        for f in fr:
                            if all(m2[i] <= f[i] for i in range(A)):
                                dead.add((nxt, f))
                            else:
                                keep.append(f)
                        keep.append(m2)
                        front[nxt] = keep
                        parent[st2] = st
                        work[0] += 1 + len(fr)
                        if nxt == goal:
                            hit = st2
                            break
                        q.append(st2)
                if hit is None:
                    break
                found = (max(hit[1]), paths_of(parent, hit, lambda x: x[0]))
                D = found[0] - 1
            if found is None:
                return int(upper), None
            return found
        raise ValueError(f"unknown objective {objective!r}")
    except _Budget:
        return None


def _original_pruning_dead(graph: Graph, starts: Sequence[int], goals: Sequence[int],
                           budget: int = DEFAULT_FEASIBILITY_BUDGET, one_move: bool = False,
                           per_robot_only: bool = False, t_first: int = 1) -> bool | None:
    """Does the original's reachability pruning make the makespan model infeasible for
    **every** horizon ``T >= t_first`` (at least 1; the first horizon of the original's
    loop), although the instance itself may be solvable?

    Only possible on a one-way graph: the original's goal-side distance is the forward
    BFS ``dgf(v) = dist(g, v)`` (see :func:`~mppflow.network.update_reachable_sets`), so
    robot ``i`` may be at ``v`` at time ``t`` only if ``dgf_i(v) <= T - t`` (the start
    side ``dist(s_i, v) <= t`` holds on every walk from ``s_i``). Backward search over
    joint configurations with the model's rules (as :func:`joint_feasibility`: arcs incl.
    self-loops, vertex capacity at ``t >= 1``, no head-on swaps, at most one move per
    step if ``one_move``): ``B_0 = {goals}``, ``B_k`` = the injective configurations
    within the windows for ``k`` steps to go that have a valid move into ``B_(k-1)``. The
    model at horizon ``T`` is feasible iff a valid move from the start configuration (not
    pruned at ``t = 0``) lands in ``B_(T-1)``. Once ``k > max dgf`` the windows no longer
    restrict, ``B_k`` follows a fixed map and the sequence becomes periodic, which ends
    the search. A cheap per-robot pass runs first. A robot whose start is its goal is no
    exception: at ``T >= 1`` it needs a closed walk within the windows (on a one-way cycle
    without wait loops there is none).

    Returns ``True`` (infeasible for every ``T``: the original loops forever), ``False``,
    or ``None`` when ``budget`` (work units) runs out. Symmetric graphs always give
    ``False`` when the instance is solvable (``dgf`` is then the true distance). No random
    draws.

    With ``per_robot_only=True`` only the per-robot pass runs: ``True`` then means that one
    robot alone has no path in the pruned network at any horizon (so even the LP relaxation
    is infeasible for every ``T``), ``False`` that every robot has one.
    """
    import itertools

    A = len(starts)
    work = 0
    pred: dict[int, list[int]] = {}
    for u, nb in graph.adj.items():
        for w in nb:
            pred.setdefault(w, []).append(u)
    dgfs = [get_distances_to_all_vertices(graph, g, None, "fixed") for g in goals]
    k_first = max(t_first, 1) - 1     # horizon T = k + 1 >= t_first

    # per robot (exact for one robot, a necessary condition for several)
    for i in range(A):
        s, g, dgf = starts[i], goals[i], dgfs[i]
        kmax = max(max(dgf, default=0), k_first)
        out_s = set(graph.neighbors(s) or ())
        B = frozenset((g,))
        seen: set[frozenset] = set()
        k = 0
        while not (k >= k_first and out_s & B):   # out_s & B_k: horizon T = k + 1 has a path
            if k >= kmax:
                if B in seen:
                    return True
                seen.add(B)
            k += 1
            nxt = set()
            for w in B:
                for u in pred.get(w, ()):
                    work += 1
                    if dgf[u] <= k:
                        nxt.add(u)
            if work > budget:
                return None
            B = frozenset(nxt)
            if not B:
                return True
    if A <= 1 or per_robot_only:
        return False
    start, goal = tuple(starts), tuple(goals)
    if len(set(goal)) < A:
        return True

    def valid(src, dst) -> bool:
        moved = [(u, w) for u, w in zip(src, dst) if u != w]
        if one_move and len(moved) > 1:
            return False
        ms = set(moved)
        return not any((w, u) in ms for (u, w) in moved)

    first = set()                     # valid configurations at t = 1
    for dst in itertools.product(*(list(graph.neighbors(v) or ()) for v in start)):
        work += 1
        if len(set(dst)) == A and valid(start, dst):
            first.add(dst)
        if work > budget:
            return None
    kmax = max(max((max(d, default=0) for d in dgfs), default=0), k_first)
    B = frozenset((goal,))
    seen_j: set[frozenset] = set()
    k = 0
    while not (k >= k_first and first & B):
        if k >= kmax:
            if B in seen_j:
                return True
            seen_j.add(B)
        k += 1
        nxt = set()
        for cfg in B:
            for src in itertools.product(*(pred.get(w, ()) for w in cfg)):
                work += 1
                if (len(set(src)) == A and all(dgfs[i][src[i]] <= k for i in range(A))
                        and valid(src, cfg)):
                    nxt.add(src)
            if work > budget:
                return None
        B = frozenset(nxt)
        if not B:
            return True
    return False


# ------------------------------------------------------------ makespan solvers
def solve_time(graph: Graph, starts: Sequence[int], goals: Sequence[int], solve_relaxed_first: bool = False,
               set_optimal: bool = True, extra_steps: int = 0, has_time_limit: bool = False,
               time_limit: float = -1.0, *, rng: MathRandom | None = None, backend: str = "highs",
               params: dict | None = None, t_max: int | None = None, compat: str = "fixed",
               debug: bool = False, print_paths: bool = False, echo: bool = False,
               clock: Callable[[], float] | None = None,
               feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET) -> SolveResult:
    """``MultiagentGraphSolverGurobiTime.solve``: MINMAKESPAN.

    1. ``T = max_i |shortest path_i|`` (one :func:`find_shortest_paths` call: draws),
       plus ``extra_steps``.
    2. If ``solve_relaxed_first`` and ``extra_steps == 0``: solve the LP relaxation
       (:func:`~mppflow.model.build_time_model` with ``integer=False``,
       ``set_optimal=False``) for ``T, T+1, ...`` until its value exceeds ``n - 0.01``
       (a sound pruning: an LP max flow below ``n`` proves the ILP infeasible).
    3. Solve the ILP for ``T, T+1, ...`` until its value is ``n``; the paths of that
       horizon are returned (makespan ``T*``).

    Time limits: ``has_time_limit`` and ``time_limit`` (seconds) as in the original.
    ``debug`` / ``print_paths`` select the original's messages (``bDebugInfo`` /
    ``bPrintPaths``, both ``true`` by default in Java) for ``result.log``. ``t_max``
    (``fixed`` only): never solve a horizon above ``t_max``; :class:`Infeasible` is raised
    when the first horizon ``T_min + extra_steps`` already exceeds it, or when every horizon
    up to ``t_max`` failed (the returned plan therefore always has makespan <= ``t_max``).

    Differences of ``fixed`` and the ``original`` infinite loop: see the module docstring.
    In ``original`` mode an unreachable goal raises
    :class:`~mppflow.errors.JavaNullPointerException` after the shortest-path draws.
    """
    return _solve_makespan("time", graph, starts, goals, solve_relaxed_first, set_optimal, extra_steps,
                           has_time_limit, time_limit, rng=rng, backend=backend, params=params, t_max=t_max,
                           compat=compat, debug=debug, print_paths=print_paths, echo=echo, clock=clock,
                           budget=feasibility_budget)


def solve_time_r(graph: Graph, starts: Sequence[int], goals: Sequence[int], solve_relaxed_first: bool = False,
                 set_optimal: bool = True, extra_steps: int = 0, has_time_limit: bool = False,
                 time_limit: float = -1.0, *, rng: MathRandom | None = None, backend: str = "highs",
                 params: dict | None = None, t_max: int | None = None, compat: str = "fixed",
                 debug: bool = False, print_paths: bool = False, echo: bool = False,
                 clock: Callable[[], float] | None = None,
                 feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET) -> SolveResult:
    """``MultiagentGraphSolverGurobiTimeR.solve``: makespan with at most one move per
    time step (the PMG setting; *dead code* in the original). Identical to
    :func:`solve_time` except that the first horizon is the **sum** of the shortest-path
    lengths and the model is :func:`~mppflow.model.build_time_r_model`.

    Differs from the original: as :func:`solve_time`, and ``fixed`` puts every move arc in
    the one-move rows (the original omits the arcs leaving vertices with id >= 22, see
    :func:`~mppflow.model.build_time_r_model`). The original's infeasibility detection
    uses the one-move check when every vertex id is below 22 (the original rows are then
    exact) and otherwise the (weaker, still sound) unrestricted-move check, because its rows
    depend on the wrapped keys.
    """
    return _solve_makespan("time_r", graph, starts, goals, solve_relaxed_first, set_optimal, extra_steps,
                           has_time_limit, time_limit, rng=rng, backend=backend, params=params, t_max=t_max,
                           compat=compat, debug=debug, print_paths=print_paths, echo=echo, clock=clock,
                           budget=feasibility_budget)


def _time_r_rows_exact(graph: Graph) -> bool:
    """Are the original TimeR's one-move-per-step rows exact (for every ``T < 10000``)?

    The original decodes the step and ``u != v`` from ``getEdgeId(a, u, v, t) =
    a*10^12 + u*10^8 + v*10^4 + t`` (``int`` sub-products). With every vertex id in
    ``0..21`` and ``t < 10000`` nothing wraps or carries, so the decoding is exact, the
    original rows equal the fixed ones, and the one-move joint-state search decides the
    original model. Otherwise (ids >= 22) arcs drop out of the rows and only the weaker
    unrestricted-move check is sound."""
    for v in list(graph.vertices) + list(graph.adj):
        if not 0 <= v < 22:
            return False
    return all(0 <= w < 22 for nb in graph.adj.values() for w in nb)


_NPE_TIME = 'Cannot read field "vertexList" because "<local13>[<local14>]" is null'
_NPE_TT = 'Cannot read field "vertexList" because "<local14>[<local15>]" is null'


def _solve_makespan(kind, graph, starts, goals, solve_relaxed_first, set_optimal, extra_steps,
                    has_time_limit, time_limit, *, rng, backend, params, t_max, compat, debug, print_paths,
                    echo, clock, budget, t_start: int | None = None) -> SolveResult:
    orig = check_compat(compat)
    _check_budget(budget)
    t_max, t_start = _check_horizon_arg(t_max, "t_max"), _check_horizon_arg(t_start, "t_start")
    extra_steps = _check_horizon_arg(extra_steps, "extra_steps")
    starts, goals = as_ids(starts, "starts"), as_ids(goals, "goals")
    out = _Out(echo)
    A = len(starts)
    if not orig:
        _check_time_limit_sentinel(time_limit)
        _check_instance(graph, starts, goals)
    clk = _Clock(clock, orig)
    build = build_time_model if kind == "time" else build_time_r_model
    name = "MultiagentGraphSolverGurobiTime" if kind == "time" else "MultiagentGraphSolverGurobiTimeR"

    # 1. lower bound from the shortest paths (consumes the draws)
    if t_start is None:
        sp = find_shortest_paths(graph, starts, goals, rng, heuristic=None if orig else False, compat=compat)
        T = 0
        for i, p in enumerate(sp):
            if p is None:
                if orig:
                    raise JavaNullPointerException(_NPE_TIME)
                raise Infeasible(f"goal of robot {i} is unreachable from its start")
            d = len(p) - 1
            T = T + d if kind == "time_r" else max(T, d)
    else:
        T = t_start
    lower_bound = T
    T += extra_steps
    T_first = T  # the loop's first horizon: the proofs below cover the horizons >= T_first
    if not orig and A > 0 and t_max is not None and T > t_max:
        # every horizon the loop would try is above t_max (incl. T_min + extra_steps)
        raise Infeasible(f"the first horizon T = {T} (T_min = {lower_bound} plus {extra_steps} extra steps) "
                         f"exceeds t_max = {t_max}")
    if debug:
        out.println(f"\nUsing {extra_steps} extra steps.")
    has = has_time_limit if orig else (has_time_limit and time_limit > 0)
    tl = float(time_limit)
    attempts: list[dict] = []
    feas = {}

    def result(paths, status, T_, **extra):
        obj = None if paths is None else (len(paths[0]) - 1 if paths else 0)
        return SolveResult(paths=paths, status=status, objective=obj, T=T_, lower_bound=lower_bound,
                           log=out.text(), attempts=attempts, extra=extra)

    def after_failure(T_, phase="ilp"):
        """Called after a failed horizon: detect the original's endless loop (or, in fixed
        mode, prove infeasibility / apply t_max). Returns a result to return, or None.

        An inconclusive check (budget exhausted) is retried at the 2nd, 4th, 8th, ...
        failed horizon with ``budget`` times the number of failed horizons, so the total
        search work stays proportional to ``budget`` per failed horizon; ``fixed`` also
        checks in between when the models took longer than the checks (see the module
        docstring)."""
        if not orig and t_max is not None and T_ + 1 > t_max:
            raise Infeasible(f"no solution with makespan <= t_max = {t_max}")
        nf = feas["n"] = feas.get("n", 0) + 1
        due = (nf & (nf - 1)) == 0  # a power of two
        b = budget * nf
        one = kind == "time_r" and (not orig or _time_r_rows_exact(graph))

        def bfs(b_):
            t0 = time.perf_counter()
            try:
                return _joint_bfs(graph, starts, goals, b_, one, T_first)
            finally:
                feas["bfs_t"] = feas.get("bfs_t", 0.0) + time.perf_counter() - t0
                if b_ > feas.get("b_max", 0):
                    feas["b_max"] = b_

        def tmax_check(min_T):
            if not orig and min_T is not None and t_max is not None and min_T > t_max:
                raise Infeasible(f"no solution with makespan <= t_max = {t_max} (joint-state search: the "
                                 f"smallest feasible makespan is {min_T})")

        if feas.get("v", 0) is None and due and b <= feas.get("b_none", 0):
            pass  # a larger budget was inconclusive already (a check scheduled by time, below)
        elif feas.get("v", 0) is None and due or "v" not in feas:
            if A > 0 and len(set(goals)) < A and starts != goals:
                feas["v"] = False
                feas["lp_dead"] = True  # two flows end on one goal vertex: LP value <= n - 1
            else:
                feas["v"], feas["min_T"], feas["plan"] = bfs(b)
                tmax_check(feas["min_T"])
        elif (not orig and feas.get("v", 0) is None and not feas.get("known")
              and time.perf_counter() - t_loop[0] - feas.get("bfs_t", 0.0) > feas.get("bfs_t", 0.0)):
            # ``fixed``: the horizons solved since the last check took longer than all checks
            # so far (the ILP's own infeasibility proofs get slow as T grows): check again with
            # twice the largest budget, so that the search work keeps pace with the ILP work.
            # Only a proof is used: a plan found here is dropped (the scheduled checks above
            # stay deterministic, and with them the plan of joint_plan()).
            b2 = 2 * feas.get("b_max", budget)
            v2, min_T2, _ = bfs(b2)
            if v2 is False:
                feas["v"] = False
            elif v2 is True:
                feas["known"] = True
                tmax_check(min_T2)
            else:
                feas["b_none"] = b2
        if feas["v"] is not False and orig and (feas.get("dead", 0) is None and due or "dead" not in feas):
            # the original's forward goal-side BFS can prune every horizon on a one-way graph
            if _reversed_adjacency(graph) is None:
                feas["dead"] = False
            else:
                one = kind == "time_r" and _time_r_rows_exact(graph)
                feas["dead"] = _original_pruning_dead(graph, starts, goals, b, one_move=one, t_first=T_first)
                if feas["dead"] is True and not feas.get("lp_dead"):
                    # a single robot without a path in the pruned network: its flow is 0, so
                    # the LP relaxation is infeasible at every horizon, too
                    feas["lp_dead"] = _original_pruning_dead(graph, starts, goals, b, one_move=one,
                                                             per_robot_only=True, t_first=T_first) is True
            if feas["dead"] is True and not has:
                raise OriginalWouldHang(
                    f"{name}.solve: the original's reachability pruning (forward BFS from the goal on a "
                    "one-way graph) makes the model infeasible for every T; the original increases T forever")
        proved = feas.get("dead") is True or feas["v"] is False
        if not proved:
            return None
        # without wait loops at the goals a plan does not extend to longer horizons, so the
        # proof covers T >= T_first only
        later = T_first > lower_bound and any(g not in (graph.neighbors(g) or ()) for g in goals)
        if not orig:
            if later:
                raise Infeasible(f"no plan with makespan T >= {T_first} (T_min = {lower_bound} plus "
                                 f"{T_first - lower_bound} extra steps; joint-state search)")
            raise Infeasible("the instance has no solution for any horizon (joint-state search)")
        if not has:
            raise OriginalWouldHang(
                f"{name}.solve: the instance is infeasible for every T{f' >= {T_first}' if later else ''} "
                "(proved by joint-state search); the original increases T forever")
        if phase == "lp" and not feas.get("lp_dead"):
            # The original keeps raising T while the LP relaxation is infeasible, printing a
            # "Trying T = ..." line per horizon, and the relaxation of an infeasible instance
            # usually becomes feasible after a few horizons (then the integer loop runs until
            # the limit, silently unless debug). Those lines are deterministic, so keep
            # solving the relaxations; return at once only when the relaxation is proved
            # never feasible, or after |V| + n further horizons (a guard for clocks that
            # never expire, e.g. a frozen one).
            first = feas.setdefault("lp_first", T_)
            if T_ - first < graph.n + A:
                return None
        return result(None, "time_limit", T_, shortcut=True)

    if A == 0 and not orig:
        return result([], "optimal", 0)

    def joint_plan():
        """``fixed`` with ``backend="builtin"``: after a failed horizon, the plan of the
        joint-state search of ``after_failure`` when it reached the goal configuration: a
        plan at the smallest feasible horizon ``>= T_first`` (the makespan model is a
        feasibility problem, so any plan there is as good as the ILP's). The builtin depth-first
        branch and bound needs work exponential in ``T`` to prove the horizons in between
        infeasible, and about as much for the first feasible one, even when the joint state
        space is tiny (minutes for 4 robots on 8 vertices; HiGHS and Gurobi take a fraction of
        a second, and keep solving every horizon, so their plans stay those of the ILP)."""
        ps_ = None if orig or backend != "builtin" else feas.get("plan")
        if ps_ is None:
            return None
        T_ = feas["min_T"]
        if debug:
            out.println(f" joint-state search: a plan with makespan {T_}.")
        if print_paths:
            out.print(paths_text(graph, ps_))
        status = "feasible" if extra_steps > 0 and T_ != lower_bound else "optimal"
        # recorded as an attempt (phase "joint", no model): the plan's horizon is listed, and
        # PlanResult.stats says where the plan came from (also for the pieces of a split)
        attempts.append(_joint_attempt(T_, A))
        return result(ps_, status, T_, proof="joint-state search")

    def remaining():
        return tl - clk.elapsed()

    t_loop = [time.perf_counter()]  # the work since then (models and checks) paces the checks
    reach = update_reachable_sets(graph, starts, goals, T, compat)

    # 2. LP relaxation first
    if solve_relaxed_first and extra_steps == 0:
        out.println("\nSolving relaxed model first. ")
        while True:
            if orig:
                tl = tl - clk.elapsed()
                lim = tl
            else:
                lim = remaining() if has else -1.0
                if has and lim <= 0:
                    out.println("time limit reached.")
                    return result(None, "time_limit", T)
            out.print(f"Trying T = {T}" + (f" with max time limit {_jds(lim)}" if has else "") + " ... ")
            stopper, left = _build_clock(lim if has else None, orig)
            try:
                m = build(graph, starts, goals, T, False, False, reach, time_limit=lim if has else -1.0,
                          debug=debug, compat=compat, stop=stopper)
            except BuildStopped:
                out.println("time limit reached.")
                return result(None, "time_limit", T)
            if stopper is not None:
                lim = left()  # the build counts against the budget
                if lim <= 0:
                    out.println("time limit reached.")
                    return result(None, "time_limit", T)
            sol = _run(m, backend, params, lim if has else None, None)
            attempts.append(_attempt(T, "lp", sol))
            if not sol.has_solution:
                if orig:
                    raise JavaGRBException("Unable to retrieve attribute 'ObjVal'")
                if sol.status == "time_limit":
                    out.println("time limit reached.")
                    return result(None, "time_limit", T)
                if sol.status == "node_limit":  # e.g. builtin with params={"node_limit": 0}
                    out.println("limit reached.")
                    return result(None, _undecided_status(sol), T, solver_status=sol.status)
                raise RuntimeError(f"LP relaxation at T={T} returned {sol.status!r}")
            best = sol.objective
            if not orig and sol.status != "optimal" and not best + 0.01 > A:
                # a limit stopped the LP at a point below n: the relaxation is not refuted
                out.println("limit reached.")
                return result(None, _undecided_status(sol), T, solver_status=sol.status)
            if best + 0.01 > A:
                if orig and has and tl - clk.elapsed() < 0:
                    out.println("relaxed model not feasible, time limit reached.")
                    return result(None, "time_limit", T)
                out.println("relaxed model feasible.")
                break
            elif has and remaining() < 0:
                out.println("relaxed model not feasible, time limit reached.")
                return result(None, "time_limit", T)
            out.println("relaxed model not feasible, increase T...")
            r = after_failure(T, "lp") or joint_plan()
            if r is not None:
                return r
            T += 1
            reach = update_reachable_sets(graph, starts, goals, T, compat)

    # 3. the ILP loop
    val = 0
    ps = None
    T_found = None
    while val != A:
        if orig:
            tl = tl - clk.elapsed()
            lim = tl
        else:
            lim = remaining() if has else -1.0
            if has and lim <= 0:
                if debug:
                    out.println(" time out.")
                return result(None, "time_limit", T)
        if debug:
            out.print(f"Trying T = {T} for the integer model " + (f"with max time limit {_jds(lim)}" if has else "")
                      + "...")
        stopper, left = _build_clock(lim if has else None, orig)
        try:
            m = build(graph, starts, goals, T, True, set_optimal, reach, time_limit=lim if has else -1.0,
                      debug=debug, compat=compat, stop=stopper)
        except BuildStopped:
            if debug:
                out.println(" time out.")
            return result(None, "time_limit", T)
        if stopper is not None:
            lim = left()  # the build counts against the budget
            if lim <= 0:
                if debug:
                    out.println(" time out.")
                return result(None, "time_limit", T)
        sol = _run(m, backend, params, lim if has else None, None)
        attempts.append(_attempt(T, "ilp", sol))
        if not orig:
            val = 0
        if sol.has_solution:
            val = java_d2i(sol.objective) if orig else int(round(sol.objective))
        if val == A:
            if debug:
                out.println(" done.\n")
                out.println(f"Solution with optimal makespan found in {_jds(clk.elapsed())} seconds.")
            ps = m.retrieve_paths(sol.x, graph)
            if print_paths:
                out.print(paths_text(graph, ps))
            T_found = T
        else:
            if (has and remaining() < 0) or (not orig and sol.status == "time_limit"):
                if debug:
                    out.println(" time out.")
                return result(None, "time_limit", T)
            if not orig and not _refutes(sol, A):
                # a node or solution limit (params) stopped the solve: T is not proved infeasible
                if debug:
                    out.println(f" {sol.status}: horizon undecided.")
                return result(None, _undecided_status(sol), T, solver_status=sol.status)
            if debug:
                out.println(" integer model infeasible.")
            r = after_failure(T) or joint_plan()
            if r is not None:
                return r
            T += 1
            reach = update_reachable_sets(graph, starts, goals, T, compat)
    if ps is None:  # original with zero agents: the loop never runs
        return result(None, "infeasible", T)
    if not orig and extra_steps > 0 and T_found != lower_bound:
        # the loop started at T_min + extra_steps: the first feasible horizon is not proved
        # to be the optimal makespan
        return result(ps, "feasible", T_found)
    return result(ps, "optimal", T_found)


def solve_time_puzzle(graph: Graph, starts: Sequence[int], goals: Sequence[int], set_optimal: bool, T: int,
                      reach: Reachability | None = None, *, instance: int = 1, backend: str = "highs",
                      params: dict | None = None, compat: str = "fixed") -> SolveResult:
    """``MultiagentGraphSolverGurobiTimePuzzle.solve``: one ILP at the fixed horizon
    ``T`` (no loop, no LP stage, no draws), with the caller's reachability, no time limit.

    ``result.paths`` is set when the model has a solution of value ``n`` (``(int)ObjVal``
    in ``original``, rounded in ``fixed``); ``extra["good_solution"]`` is then
    ``instance`` (the original's ``callback.goodSolution``), and
    ``extra["failed_attempt"]`` is ``True`` otherwise (the solve is never interrupted
    here; the original's two-thread race lives in :mod:`mppflow.puzzle`).

    Differs from the original: no Gurobi log file is written (the original sets
    ``LogFile = "C:\\Temp\\25puzzle-instance-<instance>"``); ``fixed`` extracts with
    ``X > 0.5``, and its status is ``"optimal"`` only when ``T`` equals the lower bound
    ``max_i dist(s_i, g_i)`` (``result.lower_bound``): one solve at a larger ``T`` proves
    that a plan of makespan ``T`` exists, not that no shorter one does, so it reports
    ``"feasible"`` (:class:`~mppflow.puzzle.PuzzleSolver`'s loop, which starts at the bound,
    knows more).
    """
    orig = check_compat(compat)
    A = len(starts)
    m = build_time_puzzle_model(graph, starts, goals, T, True, set_optimal, reach, compat=compat)
    sol = _run(m, backend, params, None, None)
    paths = None
    if sol.has_solution:
        val = java_d2i(sol.objective) if orig else int(round(sol.objective))
        if val == A:
            paths = m.retrieve_paths(sol.x, graph)
    extra = {"good_solution": instance if paths is not None else None, "failed_attempt": paths is None}
    if orig:
        status = "optimal" if paths is not None else ("time_limit" if sol.status == "time_limit" else "infeasible")
    else:
        status = "optimal" if paths is not None else ("infeasible" if _refutes(sol, A) else _undecided_status(sol))
        if paths is None and status != "infeasible":
            extra["solver_status"] = sol.status
    lb = None
    if not orig and paths is not None:
        lb = max((d for d in _distances(graph, starts, goals) if d is not None), default=0)
        if T > lb:
            status = "feasible"  # optimal only if no shorter horizon works, which is not checked here
    return SolveResult(paths=paths, status=status, objective=None if paths is None else T, T=T, lower_bound=lb,
                       attempts=[_attempt(T, "ilp", sol)], extra=extra)


# --------------------------------------------------------------- TT and DIST
def _resolve_horizon(horizon, orig):
    if horizon is None:
        return "single" if orig else "search"
    if horizon not in ("single", "search"):
        raise ValueError("horizon must be 'single' or 'search'")
    if orig and horizon == "search":
        raise NotInOriginal("the horizon search of Algorithm 1 is not in the original (use horizon='single')")
    return horizon


def _trim(paths):
    """Drop trailing steps in which no robot moves (keeps the objective of TT/DIST/max
    distance)."""
    if not paths:
        return paths
    L = len(paths[0])
    while L > 1 and all(p[L - 1] == p[L - 2] for p in paths):
        L -= 1
    return [p[:L] for p in paths]


def _tt_single(graph, starts, goals, opt, gap, T, has_time_limit, time_limit, *, return_any_solution, draw, rng,
               backend, params, compat, debug, print_paths, out, clk, attempts, lower_bound=None):
    orig = compat == "original"
    A = len(starts)
    if draw:
        sp = find_shortest_paths(graph, starts, goals, rng, heuristic=None if orig else False, compat=compat)
        lb = 0
        for i, p in enumerate(sp):
            if p is None:
                if orig:
                    raise JavaNullPointerException(_NPE_TT)
                raise Infeasible(f"goal of robot {i} is unreachable from its start")
            lb += len(p) - 1
    else:
        lb = lower_bound
    if debug:
        out.println(f"\nUsing {T} time expansion horizon.")
    reach = update_reachable_sets(graph, starts, goals, T, compat)
    if debug:
        out.print(f"Trying T = {T} for the integer model "
                  + (f"with max time limit {_jds(time_limit)}" if has_time_limit else "") + "...")
    lim = time_limit if has_time_limit else -1.0
    stopper, left = _build_clock(lim, orig)
    try:
        m = build_total_time_model(graph, starts, goals, T, True, reach, time_limit=lim, gap=gap, debug=debug,
                                   compat=compat, stop=stopper)
    except BuildStopped:
        return SolveResult(paths=None, status="time_limit", T=T, lower_bound=lb, attempts=attempts)
    if stopper is not None:
        lim = left()
        if lim <= 0:
            return SolveResult(paths=None, status="time_limit", T=T, lower_bound=lb, attempts=attempts)
    out.println(" let's do this...")
    sol = _run(m, backend, params, lim, gap if orig else _tt_model_gap(gap, A, T, lb))
    attempts.append(_attempt(T, "ilp", sol))
    ps = None
    status = _single_fail_status(sol, orig)
    total = None
    if sol.has_solution:
        if orig:
            val = java_d2i(sol.objective)
            bnd = sol.bound if sol.bound is not None else math.nan
            agap = _jdiv(bnd, float(val)) - 1
            accept = agap <= gap or return_any_solution
            proven = sol.status == "optimal"
        else:
            val = int(round(sol.objective))
            # the gap on the total time: total = n*T - val, its lower bound n*T - ObjBound
            g_ = math.inf if sol.bound is None else lp.relative_gap(float(A * T - val), A * T - sol.bound)
            accept = sol.status == "optimal" or return_any_solution or g_ <= gap + 1e-9
            # a proof needs gap 0 or a closed bound: with gap > 0 the backend reports
            # "optimal" as soon as it is within the gap
            proven = _proven(sol, gap, A * T - val, None if sol.bound is None else A * T - sol.bound)
        if accept:
            total = A * T - val
            if opt is not None:
                opt[0] = total
            if debug:
                out.println(" done.\n")
                out.println(f"Solution with a total of {total} time steps found in {_jds(clk.elapsed())} seconds.")
                out.println(f"Lower bound is {lb}")
            ps = m.retrieve_paths(sol.x, graph)
            if print_paths:
                out.print(paths_text(graph, ps))
            status = "optimal" if proven else "feasible"
        else:
            status = "rejected" if orig else status
    if ps is None and debug:
        if orig or status == "time_limit":
            out.println(" time out.")
        else:
            out.println(" infeasible.")
    return SolveResult(paths=ps, status=status, objective=total, T=T, lower_bound=lb, attempts=attempts)


def _tt_model_gap(gap: float, A: int, T: int, lb: int) -> float:
    """``fixed``: the relative gap to give the backend for a total-time gap ``gap``. The
    model maximizes ``val = n*T - TT`` (the goal-waiting steps), and the backend's gap is
    relative to ``val``; on long horizons, or with many robots at their goals, ``val`` is
    much larger than ``TT``, so the same number lets ``TT`` exceed its bound by far more
    than ``gap`` (gap 0.1 accepted a total time 40% above its bound). With ``val <= n*T - lb``
    (``lb = sum(d_i) <= TT``), a backend gap of ``gap * lb / (n*T - lb)`` keeps
    ``bound - val <= gap * lb <= gap * TT``: the gap on the total time is at most ``gap``."""
    if gap == 0:
        return 0.0
    room = A * T - lb
    return gap * lb / room if room > 0 and lb > 0 else 0.0


def _dist_single(graph, starts, goals, gap, T, time_limit, *, return_any_solution, backend, params, compat, debug,
                 print_paths, out, clk, attempts, kind="distance"):
    orig = compat == "original"
    reach = update_reachable_sets(graph, starts, goals, T, compat)
    if debug:
        out.print(f"Trying T = {T} for the integer model "
                  + (f"with max time limit {_jds(time_limit)}" if time_limit != -1 else "")
                  + f", gap={_jds(gap)} ...")
    builder = build_distance_model if kind == "distance" else build_max_distance_model
    stopper, left = _build_clock(time_limit, orig)
    try:
        m = builder(graph, starts, goals, T, True, reach, time_limit=time_limit, gap=gap, compat=compat,
                    stop=stopper)
    except BuildStopped:
        return SolveResult(paths=None, status="time_limit", T=T, attempts=attempts)
    if stopper is not None:
        time_limit = left()
        if time_limit <= 0:
            return SolveResult(paths=None, status="time_limit", T=T, attempts=attempts)
    sol = _run(m, backend, params, time_limit, gap)
    attempts.append(_attempt(T, "ilp", sol))
    ps = None
    status = _single_fail_status(sol, orig)
    obj = None
    if sol.has_solution:
        objval = sol.objective
        bnd = sol.bound if sol.bound is not None else math.nan
        if orig:
            proven = sol.status == "optimal"
            agap = 1 - _jdiv(bnd, objval)
            accept = agap <= gap or return_any_solution
        else:
            rg = lp.relative_gap(objval, bnd) if sol.bound is not None else math.inf
            accept = sol.status == "optimal" or return_any_solution or rg <= gap + 1e-9
            proven = _proven(sol, gap, int(round(objval)), sol.bound)
        if accept:
            obj = objval if orig else int(round(objval))
            if debug:
                what = "optimal distance" if kind == "distance" else "optimal max distance"
                out.println(" done.\n")
                out.println(f"Solution with {what} {_jds(objval)} found in {_jds(clk.elapsed())} seconds.")
            ps = m.retrieve_paths(sol.x, graph)
            if print_paths:
                out.print(paths_text(graph, ps))
            status = "optimal" if proven else "feasible"
        else:
            status = "rejected" if orig else status
    if debug and ps is None:
        if orig or status == "time_limit":
            out.println(" time out.")
        else:
            out.println(" infeasible.")
    return SolveResult(paths=ps, status=status, objective=obj, T=T, attempts=attempts)


def _build_clock(time_limit, orig):
    """``fixed`` with a time limit (``time_limit > 0`` seconds left): ``(stop, left)``, the
    builder's ``stop`` callback and the seconds left after the build, so that building the
    model counts against the limit; ``(None, None)`` otherwise. The build is timed with
    ``time.perf_counter`` (no reading of the solver's clock, which tests replace by one that
    advances per reading)."""
    if orig or time_limit is None or time_limit <= 0:
        return None, None
    t0 = time.perf_counter()

    def left():
        return time_limit - (time.perf_counter() - t0)
    return (lambda: left() <= 0), left


def _single_fail_status(sol, orig: bool) -> str:
    """The status of a TT/DIST/max-distance horizon without an accepted plan. ``original``:
    ``"time_limit"`` for a time limit, else ``"infeasible"``. ``fixed``: ``"infeasible"`` only
    when the backend proved it; a node or solution limit, or an incumbent outside the gap
    (it is not accepted), leaves the horizon undecided (see :func:`_undecided_status`)."""
    if orig:
        return "time_limit" if sol.status == "time_limit" else "infeasible"
    return "infeasible" if sol.status == "infeasible" else _undecided_status(sol)


def _proven(sol, gap, value: int, lower: float | None) -> bool:
    """``fixed``: is the integer ``value`` of a minimization proved optimal at this horizon?
    The bound ``lower`` must close (round up to ``value``): the backend reports "optimal" as
    soon as it is within its tolerances, which are ``gap`` and any tolerance given in
    ``params`` (HiGHS ``mip_abs_gap``, Gurobi ``MIPGapAbs``, ...). Only without a bound is the
    backend's "optimal" taken as the proof, and only with ``gap == 0``."""
    if lower is not None:
        return math.ceil(lower - 1e-6) >= value
    return sol.status == "optimal" and gap == 0


def _opt_makespan(graph, starts, goals, dists, *, backend, params, deadline_left, budget, t_max=None):
    """Optimal makespan (fixed mode, no random draws)."""
    t0 = max(dists) if dists else 0
    has = deadline_left is not None
    r = _solve_makespan("time", graph, starts, goals, False, True, 0, has, deadline_left if has else -1.0,
                        rng=None, backend=backend, params=params, t_max=t_max, compat="fixed", debug=False,
                        print_paths=False, echo=False, clock=None, budget=budget, t_start=t0)
    return r


def _search(kind, graph, starts, goals, T_given, single, lb, dists, t_max, *, backend, params, time_limit, clk,
            attempts, out, budget):
    """Algorithm 1 for TT / DIST / max distance (fixed mode).

    The optimum of these objectives is non-increasing in ``T`` when every goal has its
    wait self-loop (a plan for ``T`` extends to ``T + 1`` by waiting at the goals), so it
    suffices to solve at a horizon that is
    provably large enough: ``T_hi = TT(T0) - sum(d) + max(d)`` (TT: every robot of an
    optimal plan arrives by then), ``DIST(T0)`` (DIST: an optimal plan without idle steps
    moves at least one robot per step) and ``n * MAXDIST(T0)`` (max distance, same
    argument), all capped by ``n * T_opt`` (the paper's ``T_max``). ``T0`` is ``T_given`` if
    the model is feasible there, else the optimal makespan.

    ``t_max`` bounds the makespan of the returned plan: no horizon above ``t_max`` is
    solved (``T_given`` is lowered to ``t_max``, ``T_hi`` capped by it), the result is the
    optimum among the plans with makespan <= ``t_max``, and :class:`Infeasible` is raised
    when the optimal makespan exceeds ``t_max``.

    Before the solve at ``T_hi``, instances with at most ``JOINT_SEARCH_MAX_CONFIGS``
    injective configurations are settled by :func:`joint_optimum` (budget
    ``JOINT_SEARCH_BUDGET``): if it confirms the value at ``T0``, that plan is returned;
    if it finds a better one, its plan is returned (the smallest makespan among the
    optimal plans); ``extra["horizon_search"]["proof"]`` is then ``"joint-state search"``.
    Only when the search is skipped or runs out of budget is the model at ``T_hi`` solved.
    The model at ``T_hi`` is far slower to solve than the one at ``T0`` on dense instances
    (its LP bound is weak and free waits create many equivalent solutions). When the time
    limit stops the search before or during that solve, the plan at ``T0`` is returned with
    status ``"feasible"`` and ``extra["time_limit_at_T_hi"] = True``: it is optimal at
    ``T0`` but not proved optimal for every horizon.

    Status ``"optimal"`` needs a proof over all horizons (up to ``t_max``): the lower bound
    reached, the joint-state search, or a proved solve at ``T_hi``. With ``gap > 0`` the
    solves only prove their gap, and the result is ``"feasible"`` unless the value meets the
    lower bound or the joint-state search confirms it.

    When some goal has no wait self-loop, a plan does not extend by waiting and the optimum
    need not be monotone in ``T``: :func:`_search_every` then solves every horizon from the
    optimal makespan to ``T_hi``.
    """
    if any(g not in (graph.neighbors(g) or ()) for g in goals):
        return _search_every(kind, graph, starts, goals, single, lb, dists, t_max, backend=backend,
                             params=params, time_limit=time_limit, clk=clk, attempts=attempts, budget=budget)
    A = len(starts)
    info = {"T0": None, "T_hi": None, "T_opt": None}
    if t_max is not None and T_given is not None and T_given > t_max:
        T_given = t_max

    def left():
        return None if time_limit is None else time_limit - clk.elapsed()

    def topt():
        lt = left()
        if lt is not None and lt <= 0:
            return None
        r = _opt_makespan(graph, starts, goals, dists, backend=backend, params=params,
                          deadline_left=lt, budget=budget, t_max=t_max)
        attempts.extend(r.attempts)
        if r.paths is None:
            return None
        info["T_opt"] = r.objective
        return r.objective

    def out_of_time(T_):
        # the budget is spent (e.g. by the makespan stage): no solve without a limit
        lt = left()
        if lt is not None and lt <= 0:
            return SolveResult(None, "time_limit", T=T_, lower_bound=lb, extra={"horizon_search": info})
        return None

    first_draw = True
    T0 = T_given
    if T0 is None:
        T0 = topt()
        if T0 is None:
            return SolveResult(None, "time_limit", T=None, lower_bound=lb, extra={"horizon_search": info})
    r = out_of_time(T0)
    if r is not None:
        return r
    r0 = single(T0, first_draw, left())
    first_draw = False
    if r0.paths is None and r0.status == "infeasible":
        if T_given is None:
            return r0
        t_o = topt()
        if t_o is None:
            r0.status = "time_limit"
            r0.extra["horizon_search"] = info
            return r0
        T0 = max(T0, t_o)
        r = out_of_time(T0)
        if r is not None:
            return r
        r0 = single(T0, first_draw, left())
    info["T0"] = T0
    if r0.paths is None:
        r0.extra["horizon_search"] = info
        return r0
    if r0.objective <= lb:
        r0.status = "optimal"  # the lower bound holds for every horizon
        r0.extra["horizon_search"] = info
        return r0
    T_hi = _horizon_bound(kind, r0.objective, lb, dists, A)
    if info["T_opt"] is not None:
        T_hi = min(T_hi, A * info["T_opt"])
    if t_max is not None:
        T_hi = min(T_hi, t_max)
    info["T_hi"] = T_hi
    best = r0
    if T_hi > T0:
        # On small instances an exact joint-state search settles the optimum over all
        # horizons much faster than the model at T_hi (whose LP relaxation is weak and whose
        # free waits make many equivalent solutions: minutes for 3-5 robots on 6 vertices).
        r = _joint_settle(kind, graph, starts, goals, r0, info, t_max, left())
        if r is not None:
            return r
        lt = left()
        if lt is None or lt > 0:
            r1 = single(T_hi, False, lt)
            if r1.paths is not None and r1.objective < r0.objective:
                r1.paths = _trim(r1.paths)
                r1.T = len(r1.paths[0]) - 1 if r1.paths else 0
                best = r1
            elif r1.paths is not None:
                if r1.status != "optimal":
                    best.status = "feasible"  # T_hi solved within the gap only: no proof
            else:
                # no plan at T_hi: a limit stopped the solve (the plan at T0 extends to T_hi, so
                # the model there is feasible); optimal at T0, but not proved for every T
                if r1.status == "time_limit":
                    best.extra["time_limit_at_T_hi"] = True
                best.status = "feasible"
        else:
            best.extra["time_limit_at_T_hi"] = True
            best.status = "feasible"
    best.extra["horizon_search"] = info
    return best


def _horizon_bound(kind, value, lb, dists, A) -> int:
    """``T_hi`` of :func:`_search`: every optimal plan (trailing idle steps removed) has a
    makespan of at most this when a plan of objective ``value`` exists."""
    if kind == "total_time":
        return int(value) - lb + max(dists)
    if kind == "distance":
        return int(value)
    return A * int(value)


def _joint_settle(kind, graph, starts, goals, r0, info, t_max, lt):
    """The joint-state stage of the horizon search: on small instances
    :func:`joint_optimum` settles the optimum over all horizons. Returns the result (``r0``
    proved optimal, or the better joint-state plan) or ``None`` when the search is skipped,
    runs out of budget, or finds a better plan longer than ``t_max``."""
    A = len(starts)
    if not ((lt is None or lt > 0) and _few_configs(graph.n, A)):
        return None
    jo = joint_optimum(graph, starts, goals, kind, JOINT_SEARCH_BUDGET,
                       upper=int(r0.objective) if kind == "max_distance" else None)
    if not jo:
        return None
    val, jpaths = jo
    if val >= r0.objective:
        info["proof"] = "joint-state search"
        r0.status = "optimal"
        r0.extra["horizon_search"] = info
        return r0
    if jpaths is not None and (t_max is None or len(jpaths[0]) - 1 <= t_max):
        info["proof"] = "joint-state search"
        r = SolveResult(paths=jpaths, status="optimal", objective=type(r0.objective)(val),
                        T=len(jpaths[0]) - 1 if jpaths else 0, lower_bound=r0.lower_bound,
                        log=r0.log, attempts=r0.attempts, extra=dict(r0.extra))
        r.extra["horizon_search"] = info
        return r
    return None


def _search_every(kind, graph, starts, goals, single, lb, dists, t_max, *, backend, params, time_limit, clk,
                  attempts, budget):
    """Algorithm 1 for TT / DIST / max distance (fixed mode) when some goal has no wait
    self-loop.

    Such a robot cannot wait at its goal, so a plan for ``T`` does not extend to ``T + 1``
    and the optimum need not be monotone in ``T``: it can sit strictly between the optimal
    makespan and ``T_hi`` while the model at ``T_hi`` is worse or infeasible (earlier
    versions solved only those two horizons and reported the plan at ``T0`` as optimal).
    The bound ``T_hi`` still holds (a robot without a goal loop arrives at the last step,
    and an all-wait step can be cut out of any plan). The search solves at ``T0 = T_opt``,
    the optimal makespan (the makespan loop handles horizons that are not upward-closed),
    stops there if the value meets the lower bound, tries :func:`joint_optimum` on small
    instances (as :func:`_search`), and otherwise solves **every** horizon ``T_opt + 1, ...,
    T_hi``, ``T_hi`` shrinking with the best value found. A given ``T`` does not restrict
    the search (horizons below it can be better). Among equal values the smallest horizon
    is kept, trailing idle steps removed.

    ``extra["horizon_search"]`` has ``"every_horizon": True``. When the time limit stops
    the scan, the best plan so far is returned with status ``"feasible"`` and
    ``extra["time_limit_in_scan"] = True``.
    """
    A = len(starts)
    info = {"T0": None, "T_hi": None, "T_opt": None, "every_horizon": True}

    def left():
        return None if time_limit is None else time_limit - clk.elapsed()

    def stopped():
        lt = left()
        return lt is not None and lt <= 0

    def timed_out(T_):
        return SolveResult(None, "time_limit", T=T_, lower_bound=lb, extra={"horizon_search": info})

    if stopped():
        return timed_out(None)
    r = _opt_makespan(graph, starts, goals, dists, backend=backend, params=params, deadline_left=left(),
                      budget=budget, t_max=t_max)
    attempts.extend(r.attempts)
    if r.paths is None or stopped():
        return timed_out(None if r.paths is None else r.objective)
    T_opt = info["T_opt"] = info["T0"] = r.objective
    best = single(T_opt, True, left())
    best.extra["horizon_search"] = info
    if best.paths is None:
        return best  # time limit (the model at the optimal makespan has a plan)
    best.paths = _trim(best.paths)
    best.T = len(best.paths[0]) - 1 if best.paths else 0
    if best.objective <= lb:
        best.status = "optimal"  # the lower bound holds for every horizon
        return best
    T_hi = min(_horizon_bound(kind, best.objective, lb, dists, A), A * T_opt)
    if t_max is not None:
        T_hi = min(T_hi, t_max)
    info["T_hi"] = T_hi
    if T_hi <= T_opt:
        return best
    r = _joint_settle(kind, graph, starts, goals, best, info, t_max, left())
    if r is not None:
        return r
    proven = best.status == "optimal"
    cut = False
    T = T_opt
    while T < T_hi:
        T += 1
        if stopped():
            cut = True
            break
        r = single(T, False, left())
        if r.paths is not None:
            if r.objective < best.objective:
                r.paths = _trim(r.paths)
                r.T = len(r.paths[0]) - 1 if r.paths else 0
                r.extra["horizon_search"] = info
                best = r
                if best.objective <= lb:
                    break
                T_hi = min(T_hi, _horizon_bound(kind, best.objective, lb, dists, A))
                info["T_hi"] = T_hi
            if r.status != "optimal":
                proven = False  # solved within the gap only
        elif r.status == "time_limit":
            cut = True
            break
    if best.objective <= lb:
        best.status = "optimal"
    elif cut:
        best.status = "feasible"
        best.extra["time_limit_in_scan"] = True
    else:
        best.status = "optimal" if proven else "feasible"
    return best


def _single_status(r: SolveResult, lb) -> SolveResult:
    """``fixed``, ``horizon="single"``: a plan proved optimal at the one horizon ``T`` is
    optimal for the objective only if it meets the lower bound ``lb`` (a longer horizon can
    be strictly better); otherwise its status is ``"feasible"`` and
    ``extra["optimal_at_T"]`` is ``True``."""
    if r.paths is not None and r.status == "optimal" and lb is not None and r.objective > lb:
        r.status = "feasible"
        r.extra["optimal_at_T"] = True
    return r


def _single_horizon(T: int, t_max: int | None, dists) -> int:
    """``fixed``, ``horizon="single"``: the horizon to solve, ``min(T, t_max)``.
    :class:`Infeasible` when a robot's shortest path is longer than ``t_max``."""
    if t_max is None:
        return T
    for i, d in enumerate(dists):
        if d is not None and d > t_max:
            raise Infeasible(f"the shortest path of robot {i} has length {d} > t_max = {t_max}")
    return min(T, t_max)


def _waits_at_goals(graph: Graph, goals) -> bool:
    """Does every goal have its wait self-loop? Then a plan for ``T`` extends to every
    longer horizon, and "no plan at ``T``" means "no plan with makespan <= ``T``"."""
    return all(g in (graph.neighbors(g) or ()) for g in goals)


def _single_done(r: SolveResult, T: int, T_solved: int, t_max: int | None, graph: Graph,
                 goals) -> SolveResult:
    """``fixed``, ``horizon="single"``: a horizon lowered to ``t_max`` without a plan means
    that no plan has makespan <= ``t_max`` -- when every goal has its wait loop. Without
    one, only the horizon ``t_max`` itself is proved infeasible (a shorter plan may exist),
    and the message says so."""
    if T_solved < T:
        r.extra["T_requested"] = T
        if r.paths is None and r.status == "infeasible":
            if _waits_at_goals(graph, goals):
                raise Infeasible(f"no plan with makespan <= t_max = {t_max}")
            raise Infeasible(_exact_horizon_msg(f"t_max = {t_max}"))
    return r


def _exact_horizon_msg(what: str) -> str:
    return (f"no plan with makespan exactly {what} (horizon='single'; a goal has no wait self-loop, so a "
            "shorter plan does not extend to this horizon: horizon='search' tries every horizon)")


def _few_configs(nv: int, A: int) -> bool:
    """``nv! / (nv - A)! <= JOINT_SEARCH_MAX_CONFIGS``."""
    c = 1
    for k in range(A):
        c *= nv - k
        if c > JOINT_SEARCH_MAX_CONFIGS:
            return False
    return True


def solve_total_time(graph: Graph, starts: Sequence[int], goals: Sequence[int], opt: list | None = None,
                     gap: float = 0.0, time_steps: int | None = None, has_time_limit: bool = False,
                     time_limit: float = -1.0, *, return_any_solution: bool = False, horizon: str | None = None,
                     t_max: int | None = None, rng: MathRandom | None = None, backend: str = "highs",
                     params: dict | None = None, compat: str = "fixed", debug: bool = False,
                     print_paths: bool = False, echo: bool = False, clock: Callable[[], float] | None = None,
                     feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET) -> SolveResult:
    """``MultiagentGraphSolverGurobiTotalTime.solve``: MINTOTALTIME (sum of the final
    arrival times).

    ``horizon="single"`` (the original, and the only choice in ``original`` mode): one
    :func:`~mppflow.model.build_total_time_model` at ``T = time_steps`` with
    ``MIPGap = gap``. The total time is ``n*T - (int)ObjVal``; it is stored in ``opt[0]``
    (when ``opt`` is given) and returned as ``result.objective``. A solution is accepted when
    ``ObjBound/val - 1 <= gap`` (original; ``NaN`` for an optimum of 0, rejected) or
    ``return_any_solution`` (the original's static ``bReturnAnySolution``).

    ``horizon="search"`` (the ``fixed`` default; Algorithm 1 of T-RO 2016 with
    ``T_max = n * T_opt``): solve at ``T0`` (``time_steps`` when feasible there, else the
    optimal makespan; ``time_steps=None`` means the optimal makespan); stop if the result
    equals the lower bound ``sum(d_i)``; otherwise solve once more at
    ``T_hi = TT(T0) - sum(d_i) + max(d_i)`` (capped by ``n * T_opt`` when known), which every
    optimal plan fits in, and keep the better plan (trailing idle steps removed); when a
    goal has no wait self-loop, every horizon from the optimal makespan to ``T_hi`` is
    solved instead (see :func:`_search_every`). ``t_max``
    (``fixed``) bounds the makespan: ``time_steps`` is lowered to it, ``T_hi`` is capped by
    it, the result is the optimum among the plans with makespan <= ``t_max``, and
    :class:`Infeasible` is raised when the optimal makespan exceeds it. In ``fixed`` mode a
    single horizon reports ``"optimal"`` only when the plan meets the lower bound (see
    :class:`SolveResult`). Only the first single solve draws random numbers (one
    :func:`find_shortest_paths`, as the original); the optimal makespan is computed without
    draws.

    Time limit: ``has_time_limit`` and ``time_limit`` as in the original (the TimeLimit of
    the single model; in ``search`` mode the total budget).
    """
    orig = check_compat(compat)
    hz = _resolve_horizon(horizon, orig)
    _check_budget(feasibility_budget)
    time_steps, t_max = _check_horizon_arg(time_steps, "T"), _check_horizon_arg(t_max, "t_max")
    starts, goals = as_ids(starts, "starts"), as_ids(goals, "goals")
    out = _Out(echo)
    clk = _Clock(clock, orig)
    attempts: list[dict] = []
    if not orig:
        _check_gap(gap)
        _check_time_limit_sentinel(time_limit)
        _check_instance(graph, starts, goals)
        dists = _distances(graph, starts, goals)
        for i, d in enumerate(dists):  # before the t_max check, as solve_distance and the JS engine
            if d is None:
                raise Infeasible(f"goal of robot {i} is unreachable from its start")
    if hz == "single":
        if time_steps is None:
            raise ValueError("horizon='single' needs the horizon T (time_steps)")
        has = has_time_limit if orig else (has_time_limit and time_limit > 0)
        T1 = time_steps if orig else _single_horizon(time_steps, t_max, dists)
        r = _tt_single(graph, starts, goals, opt, gap, T1, has, time_limit,
                       return_any_solution=return_any_solution, draw=True, rng=rng, backend=backend, params=params,
                       compat=compat, debug=debug, print_paths=print_paths, out=out, clk=clk, attempts=attempts)
        if not orig:
            _single_status(r, r.lower_bound)
            _single_done(r, time_steps, T1, t_max, graph, goals)
        r.log = out.text()
        return r
    lb = sum(dists)
    tl = time_limit if (has_time_limit and time_limit > 0) else None

    def single(T, draw, left):
        has = left is not None
        r = _tt_single(graph, starts, goals, None, gap, T, has, left if has else -1.0,
                       return_any_solution=return_any_solution, draw=draw, rng=rng, backend=backend, params=params,
                       compat=compat, debug=debug, print_paths=print_paths, out=out, clk=clk, attempts=attempts,
                       lower_bound=lb)
        return r

    r = _search("total_time", graph, starts, goals, time_steps, single, lb, dists, t_max, backend=backend,
                params=params, time_limit=tl, clk=clk, attempts=attempts, out=out, budget=feasibility_budget)
    if r.paths is not None and opt is not None:
        opt[0] = r.objective
    r.lower_bound = lb
    r.attempts = attempts
    r.log = out.text()
    return r


def solve_distance(graph: Graph, starts: Sequence[int], goals: Sequence[int], gap: float = 0.0,
                   expansion: int | None = None, time_limit: float = -1.0, *, return_any_solution: bool = False,
                   horizon: str | None = None, t_max: int | None = None, backend: str = "highs",
                   params: dict | None = None, compat: str = "fixed", debug: bool = False,
                   print_paths: bool = False, echo: bool = False, clock: Callable[[], float] | None = None,
                   feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET) -> SolveResult:
    """``MultiagentGraphSolverGurobiDistance.solve``: MINTOTALDIST (number of moves; waits
    are free). No random draws.

    ``horizon="single"``: one :func:`~mppflow.model.build_distance_model` at
    ``T = expansion`` (``MIPGap = gap``, ``TimeLimit = time_limit`` if positive). Accepted
    when ``1 - ObjBound/ObjVal <= gap`` (original; ``NaN`` when nobody has to move, rejected)
    or ``return_any_solution``. ``result.objective`` is ``ObjVal`` (a float in ``original``).

    ``horizon="search"`` (``fixed`` default): as :func:`solve_total_time`, with the second
    horizon ``T_hi = DIST(T0)`` (an optimal plan without idle steps has a makespan of at most
    its distance) and the lower bound ``sum(d_i)``.

    Differs from the original: see the module docstring (acceptance, horizon); ``fixed``
    raises :class:`Infeasible` for a goal unreachable from its start (the original builds and
    solves an infeasible model and returns ``null``).
    """
    orig = check_compat(compat)
    hz = _resolve_horizon(horizon, orig)
    _check_budget(feasibility_budget)
    expansion, t_max = _check_horizon_arg(expansion, "T"), _check_horizon_arg(t_max, "t_max")
    starts, goals = as_ids(starts, "starts"), as_ids(goals, "goals")
    out = _Out(echo)
    clk = _Clock(clock, orig)
    attempts: list[dict] = []
    if not orig:
        _check_gap(gap)
        _check_time_limit_sentinel(time_limit)
        _check_instance(graph, starts, goals)
        dists = _distances(graph, starts, goals)
        for i, d in enumerate(dists):
            if d is None:
                raise Infeasible(f"goal of robot {i} is unreachable from its start")
    if hz == "single":
        if expansion is None:
            raise ValueError("horizon='single' needs the horizon T (expansion)")
        T1 = expansion if orig else _single_horizon(expansion, t_max, dists)
        r = _dist_single(graph, starts, goals, gap, T1, time_limit, return_any_solution=return_any_solution,
                         backend=backend, params=params, compat=compat, debug=debug, print_paths=print_paths,
                         out=out, clk=clk, attempts=attempts)
        if not orig:
            r.lower_bound = sum(dists)
            _single_status(r, r.lower_bound)
            _single_done(r, expansion, T1, t_max, graph, goals)
        r.log = out.text()
        return r
    lb = sum(dists)
    tl = time_limit if time_limit > 0 else None

    def single(T, draw, left):
        return _dist_single(graph, starts, goals, gap, T, left if left is not None else -1.0,
                            return_any_solution=return_any_solution, backend=backend, params=params,
                            compat=compat, debug=debug, print_paths=print_paths, out=out, clk=clk,
                            attempts=attempts)

    r = _search("distance", graph, starts, goals, expansion, single, lb, dists, t_max, backend=backend,
                params=params, time_limit=tl, clk=clk, attempts=attempts, out=out, budget=feasibility_budget)
    r.lower_bound = lb
    r.attempts = attempts
    r.log = out.text()
    return r


def solve_max_distance(graph: Graph, starts: Sequence[int], goals: Sequence[int], gap: float = 0.0,
                       time_steps: int | None = None, time_limit: float = -1.0, *, horizon: str | None = None,
                       t_max: int | None = None, backend: str = "highs", params: dict | None = None,
                       compat: str = "fixed", debug: bool = False, print_paths: bool = False, echo: bool = False,
                       clock: Callable[[], float] | None = None,
                       feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET) -> SolveResult:
    """*New*: MINMAXDIST (minimize the largest number of moves of any robot), T-RO 2016
    Sec. IV-B, eqs. (11)-(13), on the compact model
    (:func:`~mppflow.model.build_max_distance_model`). No random draws.

    ``horizon="search"`` (default): as :func:`solve_distance`, with the lower bound
    ``max(d_i)`` and ``T_hi = n * MAXDIST(T0)``. ``horizon="single"``: one solve at
    ``time_steps``. ``compat="original"`` raises :class:`~mppflow.errors.NotInOriginal`
    (the original has no such solver; the paper's experiments used MINMAKESPAN instead).
    """
    if check_compat(compat):
        raise NotInOriginal("MINMAXDIST is not implemented in the original code")
    if horizon not in (None, "single", "search"):
        raise ValueError("horizon must be 'single' or 'search'")
    hz = horizon or "search"
    _check_budget(feasibility_budget)
    time_steps, t_max = _check_horizon_arg(time_steps, "T"), _check_horizon_arg(t_max, "t_max")
    _check_gap(gap)
    _check_time_limit_sentinel(time_limit)
    starts, goals = as_ids(starts, "starts"), as_ids(goals, "goals")
    out = _Out(echo)
    clk = _Clock(clock, False)
    attempts: list[dict] = []
    _check_instance(graph, starts, goals)
    dists = _distances(graph, starts, goals)
    for i, d in enumerate(dists):
        if d is None:
            raise Infeasible(f"goal of robot {i} is unreachable from its start")
    lb = max(dists) if dists else 0
    tl = time_limit if time_limit > 0 else None

    def single(T, draw, left):
        return _dist_single(graph, starts, goals, gap, T, left if left is not None else -1.0,
                            return_any_solution=False, backend=backend, params=params, compat="fixed",
                            debug=debug, print_paths=print_paths, out=out, clk=clk, attempts=attempts,
                            kind="max_distance")

    if hz == "single":
        if time_steps is None:
            raise ValueError("horizon='single' needs the horizon T (time_steps)")
        T1 = _single_horizon(time_steps, t_max, dists)
        r = _single_done(_single_status(single(T1, False, tl), lb), time_steps, T1, t_max, graph, goals)
    else:
        r = _search("max_distance", graph, starts, goals, time_steps, single, lb, dists, t_max, backend=backend,
                    params=params, time_limit=tl, clk=clk, attempts=attempts, out=out, budget=feasibility_budget)
    r.lower_bound = lb
    r.attempts = attempts
    r.log = out.text()
    return r


# ------------------------------------------------------------- convenience API
def min_makespan(graph: Graph, starts: Sequence[int], goals: Sequence[int], *, time_limit: float | None = None,
                 t_max: int | None = None, solve_relaxed_first: bool = False, backend: str = "highs",
                 params: dict | None = None, rng: MathRandom | None = None,
                 feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET) -> SolveResult:
    """Optimal makespan plan (``fixed`` mode :func:`solve_time` with ``set_optimal``).
    Raises :class:`Infeasible` when no plan exists (or none with makespan <= ``t_max``).
    ``time_limit``: seconds, ``None`` for no limit (anything but a finite number > 0 raises
    ``ValueError``). ``feasibility_budget``: the initial work budget of the infeasibility
    proof (see the module docstring)."""
    time_limit = _check_time_limit(time_limit)
    has = time_limit is not None
    return solve_time(graph, starts, goals, solve_relaxed_first, True, 0, has, time_limit if has else -1.0,
                      rng=rng, backend=backend, params=params, t_max=t_max, feasibility_budget=feasibility_budget)


def _raise_if_infeasible(r: SolveResult, T, graph: Graph, goals) -> SolveResult:
    """The ``min_*`` wrappers raise :class:`Infeasible` where a single horizon has no plan
    (without a wait loop at some goal: no plan of makespan exactly ``T``)."""
    if r.paths is None and r.status == "infeasible":
        if _waits_at_goals(graph, goals):
            raise Infeasible(f"no plan with makespan <= T = {T}")
        raise Infeasible(_exact_horizon_msg(f"T = {T}"))
    return r


_OBJ_DOC = """

    ``T``: the first horizon (``None``: the optimal makespan). ``horizon="search"``
    (default): Algorithm 1, the optimum over all horizons. ``horizon="single"``: one solve at
    ``T`` (required); the plan is optimal at ``T`` only, so its status is ``"optimal"`` only
    when it meets the lower bound, else ``"feasible"`` with ``extra["optimal_at_T"]``.
    ``t_max``: the returned plan has makespan <= ``t_max`` (no horizon above it is solved, so
    ``horizon="single"`` solves at ``min(T, t_max)``; the value is the optimum among such
    plans). ``gap > 0``: the solves stop within the MIP gap (for the total time, the gap on
    the total time), and the status is ``"feasible"`` unless the bound closes. ``time_limit`` (seconds) is the
    total budget, model building included (a backend can still overrun its own share: HiGHS
    presolve does not stop at its limit); ``None`` means no limit, and anything but a finite
    number > 0 raises ``ValueError``. ``T`` and ``t_max`` must be integers
    (``TypeError``). Raises :class:`Infeasible` when no plan exists (none with makespan <=
    ``t_max``, or none within ``T`` under ``horizon="single"``; when a goal has no wait
    self-loop, none of makespan exactly ``T``); a horizon stopped by a node or solution limit
    given in ``params`` proves nothing (status ``"time_limit"``), and a tolerance given there
    (``mip_abs_gap``, ``MIPGapAbs``) gives ``"optimal"`` only when the bound closes; ``T`` and
    ``t_max`` must be >= 0 (``ValueError``)."""


def min_total_time(graph: Graph, starts: Sequence[int], goals: Sequence[int], *, T: int | None = None,
                   gap: float = 0.0, time_limit: float | None = None, horizon: str = "search",
                   t_max: int | None = None, backend: str = "highs", params: dict | None = None,
                   rng: MathRandom | None = None,
                   feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET) -> SolveResult:
    time_limit = _check_time_limit(time_limit)
    has = time_limit is not None
    r = solve_total_time(graph, starts, goals, None, gap, T, has, time_limit if has else -1.0,
                         horizon=horizon, t_max=t_max, rng=rng, backend=backend, params=params,
                         feasibility_budget=feasibility_budget)
    return _raise_if_infeasible(r, T, graph, goals)


def min_total_distance(graph: Graph, starts: Sequence[int], goals: Sequence[int], *, T: int | None = None,
                       gap: float = 0.0, time_limit: float | None = None, horizon: str = "search",
                       t_max: int | None = None, backend: str = "highs", params: dict | None = None,
                       feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET) -> SolveResult:
    tl = _check_time_limit(time_limit)
    r = solve_distance(graph, starts, goals, gap, T, -1.0 if tl is None else tl, horizon=horizon,
                       t_max=t_max, backend=backend, params=params, feasibility_budget=feasibility_budget)
    return _raise_if_infeasible(r, T, graph, goals)


def min_max_distance(graph: Graph, starts: Sequence[int], goals: Sequence[int], *, T: int | None = None,
                     gap: float = 0.0, time_limit: float | None = None, horizon: str = "search",
                     t_max: int | None = None, backend: str = "highs", params: dict | None = None,
                     feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET) -> SolveResult:
    tl = _check_time_limit(time_limit)
    r = solve_max_distance(graph, starts, goals, gap, T, -1.0 if tl is None else tl, horizon=horizon,
                           t_max=t_max, backend=backend, params=params, feasibility_budget=feasibility_budget)
    return _raise_if_infeasible(r, T, graph, goals)


min_total_time.__doc__ = "Minimum total arrival time (``fixed`` :func:`solve_total_time`)." + _OBJ_DOC
min_total_distance.__doc__ = "Minimum total distance (``fixed`` :func:`solve_distance`)." + _OBJ_DOC
min_max_distance.__doc__ = "Minimum maximum distance (:func:`solve_max_distance`)." + _OBJ_DOC
