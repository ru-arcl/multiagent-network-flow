"""The local-repair heuristics: ``PathPlanner``'s and ``HeuristicPathPlanner``'s.

Ports (original ``ILP/PathPlanner.java`` and ``ILP/HeuristicPathPlanner.java``):

==================================================  ==========================================
mppflow                                              original
==================================================  ==========================================
:class:`LocalRepairPlanner`                          the heuristic part of ``PathPlanner``
:func:`plan_heuristic_paths`                         ``new PathPlanner().planHeuristicPaths``
  ``.plan_heuristic_paths``                          ``planHeuristicPaths`` (up to 10 attempts)
  ``.plan_heuristic_path``                           ``planHeuristicPath`` (one attempt)
  ``.create_sub_problem``                            ``createSubProblem`` (private)
  ``.move_agents_independently_when_can``            ``moveAgentsIndependentlyWhenCan`` (private)
  :func:`move_agents_independently`                  ``moveAgentsIndependently`` (*dead*)
:class:`HeuristicPathPlanner`                        ``HeuristicPathPlanner`` (subclass)
  ``.plan_heuristic_paths``                          ``planHeuristicPaths`` (3 attempts)
  ``.plan_heuristic_path``                           ``planHeuristicPath``
  ``.create_sub_problem``                            ``createSubProblem`` (private)
  :func:`exists_edge_collision`                      ``existsEdgeCollision`` (private)
  :func:`exists_vertex_collision`                    ``existsVertexCollision`` (private)
  :func:`move_agents_independently_one_step`         ``moveAgentsIndependentlyOneStep``
:func:`number_of_goals_reached`                      ``numberOfGoalsReached`` (both classes)
:func:`print_paths`                                  ``printPaths`` (both classes)
:func:`path_planner_main`                            ``PathPlanner.main``
:func:`heuristic_path_planner_main`                  ``HeuristicPathPlanner.main``
==================================================  ==========================================

Neither heuristic is in T-RO 2016. ``HeuristicPathPlanner`` is the ICRA 2013 Sec. VI-C
heuristic ("MinMakespan as a generic heuristic for locally resolving path conflicts"); only
its own ``main`` uses it. ``Main.solveProblemWithHeuristic`` uses ``PathPlanner``'s variant.

The algorithm (both variants)
-----------------------------
Single-robot shortest paths are followed until robots would collide; each conflict (two
robots swapping along an edge, or several robots entering one vertex) becomes a sub-problem
on the conflict's neighbourhood (distance 2 plus a random partial ring at distance 3), whose
robots -- every robot standing in the region -- get local sub-goals; vertex-disjoint
sub-problems are solved with the exact MINMAKESPAN solver (:func:`~mppflow.solvers.solve_time`,
LP relaxation first) and the local plans are spliced in. This repeats until every robot is
at its goal, the attempt's budget is spent, or the attempt stalled 5 times (cumulatively).
``plan_heuristic_paths`` restarts failed attempts with fresh random draws.

* ``PathPlanner``: robots advance individually (``moveAgentsIndependentlyWhenCan``: a robot
  moves if its next vertex is free at the start of the round and not just entered by a
  lower-index robot); sub-problem expansion probability 1/2; sub-goals alternate by attempt
  parity (even: greedy nearest vertex to the true goal; odd: rotate the conflicting robots'
  positions); attempt schedule by robot count (25 -> 8 x 1.1..1.8 s, 50 -> 7 x 1.7..2.9 s,
  75 -> 6 x 2.25..3.5 s, 100 -> 5 x 4.5..6.5 s, 125 -> 4 x 6.5..8 s, 150 -> 4 x 8 s,
  otherwise 10 x 3 s).
* ``HeuristicPathPlanner``: all robots advance synchronously until the first step with a
  meet or a head-on swap; expansion probability 1/4; greedy sub-goals; 3 attempts of
  ``1.5 + (5 + 4 i) n / 120`` seconds.

Results are :class:`~mppflow.solvers.SolveResult` objects: ``paths`` is the original's
``int[][]`` (possibly partial: some robots not at their goals), ``status`` is ``"feasible"``
when every robot reaches its goal (a valid plan, no optimality claim) and ``"partial"``
otherwise, ``objective`` is the makespan, ``extra["goals_reached"]`` the number of robots at
their goals and ``extra["stop"]`` why the (last) attempt ended (``"done"``, ``"stalled"``,
``"time_limit"``, ``"budget"``). ``attempts`` lists one summary per attempt
(``plan_heuristic_paths``) or one entry per sub-problem examined (``plan_heuristic_path``).
``log`` holds what the original prints ("Attempting with ... seconds...", and the sub-solver's
unconditional "Solving relaxed model first." / "Trying T = ..." lines); ``echo=True`` also
prints it.

Time budgets (both modes)
-------------------------
The original's budgets are wall-clock (``System.currentTimeMillis``), which makes it
machine-dependent (original-bugs B26). Here time is read from
``clock`` (a callable returning seconds). The default ``clock=None`` is a **virtual clock that
never advances**: the original's ``timeLeft = timeLimit - elapsed`` stays at ``timeLimit``
(also passed on, as in the original, as the sub-solver's time limit), so wall-clock budgets
never expire and the results are deterministic. Pass ``clock=time.monotonic`` to get the
original's wall-clock behaviour. In place of the wall-clock budget, ``max_subproblems``
(default :data:`DEFAULT_MAX_SUBPROBLEMS`, ``None`` = unlimited) bounds the number of
sub-problems an attempt examines; when it is exhausted the attempt returns its partial
paths exactly as the original does when its time runs out at that point (its time test sits
in the same place, before each sub-problem). Every attempt that does not stall or finish
examines at least one sub-problem per round, so attempts always terminate.

Randomness
----------
All draws come from ``rng`` (a :class:`~mppflow._java.MathRandom`; ``None`` = the module
stream) in exactly the original's order: per round, (a) ``findShortestPaths`` for all robots
(``PathPlanner``: once per ``moveAgentsIndependentlyWhenCan`` call, the last, unproductive
one included, and once more for the conflict detection); (b) the region draws of every
sub-problem, in sub-problem order (swaps first, then meets), all before any is solved: one
draw per (conflict vertex ``s``, ``u`` in ``N[s]``, ``w`` in ``N[u]``) with closed
neighbourhoods; (c) the sub-solver's ``findShortestPaths`` on each sub-graph actually solved
(sub-graph adjacency in the original's ``HashSet`` order, emulated by ``Graph.adj_order``).

compat modes
------------
``compat="original"`` reproduces the original, crashes included: a direct
``plan_heuristic_path`` call with no distance table raises
:class:`~mppflow.errors.JavaNullPointerException` at the first sub-problem (after its region
draws); zero robots raise :class:`~mppflow.errors.JavaArrayIndexOutOfBoundsException` (after
the "Attempting" line); an unreachable goal raises ``JavaNullPointerException`` in
``parsePaths`` (after the shortest-path draws); edge ids are ``i32(max*10000 + min)``; the
global flags are emulated (see below). Differences of ``compat="fixed"``:

* Distance table: ``plan_heuristic_path`` computes the robots' goal distances when the
  table is missing or was computed for another graph or other goals (the original
  dereferences ``null`` or silently uses the stale table of an earlier call; original-bugs
  B18). On a one-way graph the distance is ``dist(v, goal)`` (BFS over
  the reversed arcs) instead of the original's ``dist(goal, v)``, and a vertex that cannot
  reach the goal gets a large distance instead of 0 (so it is never chosen as the "nearest"
  sub-goal). On symmetric connected graphs the tables are identical.
* Zero robots: the empty plan ``[]`` (status ``"feasible"``) instead of
  ``ArrayIndexOutOfBoundsException`` (original-bugs B18).
* Input checks: duplicate starts or out-of-range ids raise ``ValueError``; duplicate goals
  and goals unreachable from their start raise :class:`~mppflow.errors.Infeasible` before any
  draw (the original computes garbage, or throws ``NullPointerException``).
* Edge ids are ``(max, min)`` tuples: the original's ``max*10000 + min`` collides when a
  vertex id is >= 10000 and overflows past ~214748 (HeuristicPathPlanner Javadoc: "10000
  vertex limitation"). Identical below 10000 vertices.
* Helpers (:func:`move_agents_independently`, :func:`exists_edge_collision`,
  :func:`exists_vertex_collision`, :func:`move_agents_independently_one_step`,
  :func:`number_of_goals_reached`): ``original`` raises Java's
  ``ArrayIndexOutOfBoundsException`` / ``NullPointerException`` for out-of-range steps, ragged
  or missing rows (never a Python ``IndexError`` or a negative-index wrap-around); ``fixed``
  raises ``ValueError``, and returns 0 for no robots / counts an empty path as not arrived.
* Distance-table staleness is decided by graph identity (a reference to the graph is
  kept, so a freed graph's reused ``id()`` cannot match), its vertex count and the goals.
* ``LocalRepairPlanner.plan_heuristic_path`` does not modify the caller's ``start`` list (the
  original leaves the robots' final positions in it).
* Sub-problems proved infeasible by the sub-solver (it raises ``Infeasible`` in ``fixed``)
  count as failed, like the original's ``null``.
* Exactly spent budget: the original's time test before each sub-problem is
  ``timeLeft < 0``, so ``timeLeft == 0.0`` (a millisecond clock, budgets such as 1.75 s)
  gets through, and the sub-solver then runs with ``hasTimeLimit = false``, i.e. without
  any limit (original-bugs.md, B27; ``original`` reproduces it: with a clock that reads
  exactly the budget, an infeasible sub-problem raises ``OriginalWouldHang``). ``fixed``
  ends the attempt when ``timeLeft <= 0``.
* Global state: the original permanently sets ``MultiagentGraphSolverGurobiTime.bPrintPaths``
  and ``bDebugInfo`` to ``false`` (reported in ``extra["statics_after"]`` in ``original``
  mode; nothing leaks in ``fixed``). ``PathFinder.bHeuristic`` (random-order, non-shortest
  "shortest" paths, set by ``Main.solveProblemWithHeuristic``): ``heuristic=None`` reads
  :data:`~mppflow.pathfinder.B_HEURISTIC` in ``original`` (and ``False`` in ``fixed``);
  in ``original`` an explicit ``heuristic`` value acts as the global flag for the duration of
  the call -- the sub-solvers see it too, which makes their ``T_min`` too large and the
  sub-plans non-optimal (original-bugs B19) -- and the global flag is
  restored afterwards. In ``fixed`` the flag affects only the heuristic's own path searches;
  the sub-solvers always use true BFS distances.

Quirks kept in both modes (they have no wrong outcome): the cumulative stall counter, the
``if (secondStrategy = false)`` assignment and the ``attempt != 3 && goal.length > 100``
branch that both append (no effect), the persistent ``attempt`` field (``PathPlanner``'s
loop variable: the successful attempt's number, or ``totalAttempts + 1`` after a fully failed
run; it selects the sub-goal strategy of later direct calls), robots already at
their goals being pulled into regions and moved, rotations of three or more robots producing
no sub-problem (``PathPlanner``) and partial results on infeasible instances
(``docs/notes/paper-vs-code.md``, "The local-repair heuristic is from ICRA 2013 only").
"""
from __future__ import annotations

import sys
from collections import deque
from typing import Callable, Sequence

from ._java import JavaIntHashSet, MathRandom, i32, java_double_str, resolve_rng
from .errors import (
    Infeasible,
    MPPError,
    JavaArrayIndexOutOfBoundsException,
    JavaNullPointerException,
)
from .graph import Graph, check_compat, print_vertex
from .pathfinder import B_HEURISTIC, find_shortest_paths, get_distances_to_all_vertices
from .problem import Problem, get_long_straight_with_one_garage_problem
from .solvers import DEFAULT_FEASIBILITY_BUDGET, SolveResult, solve_time

__all__ = [
    "DEFAULT_MAX_SUBPROBLEMS",
    "LocalRepairPlanner",
    "HeuristicPathPlanner",
    "plan_heuristic_paths",
    "number_of_goals_reached",
    "print_paths",
    "print_start_and_goals",
    "move_agents_independently",
    "exists_edge_collision",
    "exists_vertex_collision",
    "move_agents_independently_one_step",
    "path_planner_main",
    "heuristic_path_planner_main",
]

DEFAULT_MAX_SUBPROBLEMS = 1000
"""Default deterministic budget: sub-problems one attempt may examine (see the module
docstring)."""

_UNREACHABLE = 1 << 30
"""``fixed``-mode distance of a vertex that cannot reach the goal."""

_NPE_DIST = 'Cannot load from object array because "this.distanceToGoals" is null'
_NPE_PARSE = 'Cannot read field "vertexList" because "<parameter1>[<local2>]" is null'
_STATICS_AFTER = {"MultiagentGraphSolverGurobiTime.bPrintPaths": False,
                  "MultiagentGraphSolverGurobiTime.bDebugInfo": False}


# ------------------------------------------------------------------- output, clock
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


def _frozen_clock() -> float:
    """The default virtual clock: time never advances."""
    return 0.0


class _Timer:
    """``System.currentTimeMillis`` arithmetic in ``original`` mode, float seconds
    otherwise."""

    def __init__(self, clock: Callable[[], float], orig: bool):
        self.f, self.orig = clock, orig
        self.t0 = self._now()

    def _now(self):
        t = self.f()
        return int(t * 1000) if self.orig else t

    def elapsed(self) -> float:
        d = self._now() - self.t0
        return d / 1000.0 if self.orig else d


def _aioobe(i: int, n: int) -> JavaArrayIndexOutOfBoundsException:
    return JavaArrayIndexOutOfBoundsException(f"Index {i} out of bounds for length {n}")


def _jat(arr, i: int, what: str, orig: bool):
    """Java ``arr[i]``: ``JavaNullPointerException`` for ``null`` and
    ``JavaArrayIndexOutOfBoundsException`` out of range in ``original`` (no negative-index
    wrap-around); ``ValueError`` in ``fixed``."""
    if arr is None:
        if orig:
            raise JavaNullPointerException(f'Cannot load from array because "{what}" is null')
        raise ValueError(f"{what} is None")
    if not 0 <= i < len(arr):
        if orig:
            raise _aioobe(i, len(arr))
        raise ValueError(f"index {i} out of range for {what} of length {len(arr)}")
    return arr[i]


# --------------------------------------------------------------- static utilities
def _parse_paths(paths: Sequence[Sequence[int] | None], orig: bool) -> list[list[int]]:
    """``PathPlanner.parsePaths``: pad every path with its last vertex to the longest
    length. A ``None`` entry (unreachable goal) is ``NullPointerException`` in the original."""
    for p in paths:
        if p is None:
            if orig:
                raise JavaNullPointerException(_NPE_PARSE)
            raise Infeasible("a robot's goal is unreachable from its current vertex")
    if not paths:
        return []
    L = max(len(p) for p in paths)
    for p in paths:
        if len(p) == 0:
            raise _aioobe(-1, L) if orig else ValueError("empty path")
    return [list(p) + [p[-1]] * (L - len(p)) for p in paths]


def number_of_goals_reached(paths: Sequence[Sequence[int]], goal: Sequence[int], compat: str = "fixed") -> int:
    """``numberOfGoalsReached(paths, goal)`` (identical in ``PathPlanner`` and
    ``HeuristicPathPlanner``): the number of robots ``i < len(goal)`` whose path ends at
    ``goal[i]``. The same function as :func:`mppflow.planner.number_of_goals_reached`.

    ``original``: missing or empty rows raise
    :class:`~mppflow.errors.JavaArrayIndexOutOfBoundsException` (``null``:
    :class:`~mppflow.errors.JavaNullPointerException`). Differs from the original: ``fixed``
    raises ``ValueError`` when there are fewer paths than goals and counts an empty path as
    not arrived.
    """
    from .planner import number_of_goals_reached as _ngr

    return _ngr(paths, goal, compat)


def _pv(g: Graph, v: int) -> str:
    vid = g.vertices[v] if 0 <= v < len(g.vertices) else v
    return print_vertex(g, vid)


def print_paths(g: Graph, paths: Sequence[Sequence[int]], compat: str = "fixed", echo: bool = False) -> str:
    """``printPaths(g, paths)`` (both classes): ``"Agent a:"`` then ``" t:" +
    printVertex()`` per step, a newline per robot; returns the text (``echo``: also print).

    Every row uses ``paths[0]``'s length and the vertex is looked up as ``vertices[id]``
    (both as in the original; correct for graphs with ids ``0..n-1``). ``original`` raises
    ``JavaNullPointerException`` for ``paths=None`` and
    ``JavaArrayIndexOutOfBoundsException`` for a short row; ``fixed`` raises ``ValueError``.
    """
    orig = check_compat(compat)
    if paths is None:
        if orig:
            raise JavaNullPointerException('Cannot read the array length because "<parameter2>" is null')
        raise ValueError("paths is None")
    out = []
    for a in range(len(paths)):
        parts = [f"Agent {a}:"]
        for t in range(len(paths[0])):
            if t >= len(paths[a]):
                raise _aioobe(t, len(paths[a])) if orig else ValueError("ragged path set")
            parts.append(f" {t}:" + _pv(g, paths[a][t]))
        out.append("".join(parts) + "\n")
    text = "".join(out)
    if echo:
        sys.stdout.write(text)
    return text


def print_start_and_goals(g: Graph, start: Sequence[int], goal: Sequence[int], compat: str = "fixed", *,
                          echo: bool = False) -> str:
    """``PathPlanner.printStartAndGoals``: ``"Start and goal vertices: n pairs"``, then the
    starts and the goals, each vertex followed by a space and each line by ``" "``. The same
    function as :func:`mppflow.planner.print_start_and_goals` (``original``: an id not in
    the graph raises ``JavaNullPointerException``; differs from the original: ``fixed``
    raises ``ValueError``)."""
    from .planner import print_start_and_goals as _psg

    return _psg(g, start, goal, compat, echo=echo)


def _path_set_valid(paths, graph: Graph, starts, goals) -> tuple[bool, str]:
    """``PathPlanner.isPathSetValid`` on a rectangular plan: ``(valid, message)``."""
    L = len(paths[0]) if paths else 0
    for t in range(L):
        for a in range(len(paths)):
            if paths[a][0] != starts[a]:
                return False, f"Start location for agent {a} is incorrect in the path set."
            if paths[a][-1] != goals[a]:
                return False, f"Goal location for agent {a} is incorrect in the path set."
            if t > 0 and paths[a][t] not in (graph.adj.get(paths[a][t - 1]) or ()):
                return False, f"Path {a} is not valid at t = {t - 1}"
    for p in range(len(paths) - 1):
        for q in range(p + 1, len(paths)):
            for t in range(L):
                if paths[p][t] == paths[q][t]:
                    return False, f"Path {p} and {q} meet on vertex {paths[p][t]} at time step {t}"
                if t > 0 and paths[p][t] == paths[q][t - 1] and paths[p][t - 1] == paths[q][t]:
                    return False, f"Path {p} and {q} cross the same edge at time step {t}"
    return True, ""


def _plan_paths_advanced(g: Graph, start, goal, solve_relaxed: bool, extra_steps: int, set_optimal: bool,
                         time_limit: float, *, out: _Out, rng, backend, params, compat, clock,
                         feasibility_budget, debug: bool = False, print_paths_: bool = False,
                         catch_infeasible: bool = True) -> tuple[list[list[int]] | None, SolveResult | None]:
    """``PathPlanner.planPathsAdvanced``: :func:`~mppflow.solvers.solve_time` with
    ``hasTimeLimit = timeLimit > 0``, then the validity check (an invalid plan is reported
    and dropped). ``debug`` / ``print_paths_`` stand for the Time solver's ``bDebugInfo`` /
    ``bPrintPaths``. In ``fixed`` mode a proved-infeasible instance gives ``None`` when
    ``catch_infeasible``."""
    try:
        res = solve_time(g, start, goal, solve_relaxed, set_optimal, extra_steps, time_limit > 0, time_limit,
                         rng=rng, backend=backend, params=params, compat=compat, debug=debug,
                         print_paths=print_paths_, clock=clock, feasibility_budget=feasibility_budget)
    except Infeasible:
        if compat == "original" or not catch_infeasible:
            raise
        return None, None
    out.print(res.log)
    paths = res.paths
    if paths is not None:
        ok, msg = _path_set_valid(paths, g, start, goal)
        if not ok:
            out.println(msg)
            return None, res
    elif debug:
        out.println("Maximum allowed time reached, optimization stopped.")
    return paths, res


def move_agents_independently(paths: Sequence[Sequence[int]], compat: str = "fixed") -> int:
    """``PathPlanner.moveAgentsIndependently(paths)`` (*dead*: no caller): the last time
    index ``s - 1`` such that the steps ``1..s-1`` have no vertex collision and no two
    robots on the same unordered edge (either direction; two robots waiting on the same
    self-loop also count).

    ``original``: ``[]`` or a row shorter than ``paths[0]`` raises
    :class:`~mppflow.errors.JavaArrayIndexOutOfBoundsException`. Differs from the original:
    ``fixed`` keys edges by ``(max, min)`` tuples (the original's ``max*10000 + min``
    collides for ids >= 10000), returns 0 for no robots and raises ``ValueError`` for a
    ragged path set.
    """
    orig = check_compat(compat)
    if paths is None:
        _jat(None, 0, "paths", orig)
    n = len(paths)
    if n == 0 and not orig:
        return 0  # no robots: no step to take
    stop = 1
    L = len(_jat(paths, 0, "paths", orig))
    while stop < L:
        vset = set()
        eset = set()
        for a in range(n):
            row = _jat(paths, a, "paths", orig)
            v = _jat(row, stop, "paths[a]", orig)
            u = row[stop - 1]
            vset.add(v)
            eset.add(_edge_key(u, v, orig))
        if len(vset) < n or len(eset) < n:
            break
        stop += 1
    return stop - 1


def _edge_key(v1: int, v2: int, orig: bool):
    """The original's ``v1 > v2 ? v1*10000 + v2 : v2*10000 + v1`` (``int`` arithmetic,
    then widened to ``long``) in ``original``; a tuple in ``fixed``."""
    hi, lo = (v1, v2) if v1 > v2 else (v2, v1)
    if orig:
        return i32(i32(hi * 10000) + lo)
    return (hi, lo)


def exists_edge_collision(paths: Sequence[Sequence[int]], t: int, compat: str = "fixed") -> list[int] | None:
    """``HeuristicPathPlanner.existsEdgeCollision(paths, t)``: ``[first, a]`` for the first
    robot ``a`` (index order) whose unordered edge ``{paths[a][t], paths[a][t+1]}`` robot
    ``first`` already uses (a head-on swap), else ``None``.

    ``original``: ``t`` or ``t + 1`` out of a row's range raises
    :class:`~mppflow.errors.JavaArrayIndexOutOfBoundsException`. Differs from the original:
    ``fixed`` keys edges by tuples (see the module docstring) and raises ``ValueError`` for an
    out-of-range step.
    """
    orig = check_compat(compat)
    seen: dict = {}
    for a in range(len(_jat_paths(paths, orig))):
        row = _jat(paths, a, "paths", orig)
        k = _edge_key(_jat(row, t, "paths[a]", orig), _jat(row, t + 1, "paths[a]", orig), orig)
        if k in seen:
            return [seen[k], a]
        seen[k] = a
    return None


def _jat_paths(paths, orig: bool):
    if paths is None:
        if orig:
            raise JavaNullPointerException('Cannot read the array length because "<parameter1>" is null')
        raise ValueError("paths is None")
    return paths


def exists_vertex_collision(paths: Sequence[Sequence[int]], t: int, compat: str = "fixed") -> list[int] | None:
    """``HeuristicPathPlanner.existsVertexCollision(paths, t)``: ``[first, a]`` for the
    first robot ``a`` whose vertex ``paths[a][t+1]`` robot ``first`` also enters, else
    ``None``. ``original``: ``t + 1`` out of a row's range raises
    :class:`~mppflow.errors.JavaArrayIndexOutOfBoundsException`; differs from the original:
    ``fixed`` raises ``ValueError`` there."""
    orig = check_compat(compat)
    seen: dict[int, int] = {}
    for a in range(len(_jat_paths(paths, orig))):
        v = _jat(_jat(paths, a, "paths", orig), t + 1, "paths[a]", orig)
        if v in seen:
            return [seen[v], a]
        seen[v] = a
    return None


def move_agents_independently_one_step(path_array: list[list[int]], paths: Sequence[Sequence[int]], t: int,
                                       compat: str = "fixed") -> None:
    """``HeuristicPathPlanner.moveAgentsIndependentlyOneStep(pathArray, paths, t)``: append
    ``paths[a][t]`` to every ``path_array[a]`` (in place; all robots move in lockstep).

    ``original``: a ``path_array`` shorter than ``paths`` or ``t`` out of a row's range
    raises :class:`~mppflow.errors.JavaArrayIndexOutOfBoundsException` (Java evaluates
    ``pathArray[a]`` before ``paths[a][t]``; the robots before it have been appended to).
    Differs from the original: ``fixed`` raises ``ValueError`` there, before appending
    anything."""
    orig = check_compat(compat)
    n = len(_jat_paths(paths, orig))
    if not orig:
        for a in range(n):
            if _jat(path_array, a, "path_array", orig) is None:
                raise ValueError("path_array[a] is None")
            _jat(_jat(paths, a, "paths", orig), t, "paths[a]", orig)
    for a in range(n):
        target = _jat(path_array, a, "pathArray", orig)
        v = _jat(_jat(paths, a, "paths", orig), t, "paths[a]", orig)
        if target is None:
            raise JavaNullPointerException(
                'Cannot invoke "projects.multipath.advanced.Path.addVertex(int)" because "<parameter1>[<local5>]" '
                'is null')
        target.append(v)


# ------------------------------------------------------------- distance tables
def _goal_distances(graph: Graph, goal: Sequence[int], orig: bool) -> list[list[int]]:
    """``distanceToGoals``: per robot, the BFS distance of every vertex to its goal.

    ``original``: ``getDistancesToAllVertices(graph, goal[a])`` (forward BFS from the goal,
    0 for unreachable vertices). ``fixed``: ``dist(v, goal)`` over the reversed arcs (equal
    on symmetric graphs) and :data:`_UNREACHABLE` for vertices that cannot reach it."""
    n = graph.n
    if orig:
        return [get_distances_to_all_vertices(graph, goal[a], [0] * n, "original") for a in range(len(goal))]
    rev: dict[int, list[int]] = {v: [] for v in range(n)}
    for u, nb in graph.adj.items():
        for w in nb:
            if 0 <= w < n and 0 <= u < n:
                rev[w].append(u)
    out = []
    for gl in goal:
        d = [_UNREACHABLE] * n
        d[gl] = 0
        q = deque([gl])
        while q:
            v = q.popleft()
            for u in rev[v]:
                if d[u] == _UNREACHABLE:
                    d[u] = d[v] + 1
                    q.append(u)
        out.append(d)
    return out


def _validate(graph: Graph, start, goal) -> None:
    """``fixed``-mode input checks (see the module docstring); no draws."""
    if len(start) != len(goal):
        raise ValueError("start and goal must have the same length")
    n = graph.n
    for v in list(start) + list(goal):
        if not 0 <= v < n:
            raise ValueError(f"vertex {v} out of range 0..{n - 1}")
    if len(set(start)) < len(start):
        raise ValueError("two robots share a start vertex")
    if len(set(goal)) < len(goal):
        raise Infeasible("two robots share a goal vertex")
    for i, (s, g) in enumerate(zip(start, goal)):
        d = get_distances_to_all_vertices(graph, s, None, "fixed", unreachable=-1)
        if d[g] < 0:
            raise Infeasible(f"goal of robot {i} is unreachable from its start")


# ------------------------------------------------------------------- planners
class LocalRepairPlanner:
    """``PathPlanner``'s local-repair heuristic (``planHeuristicPaths`` and helpers).

    The instance holds the original's instance fields: :attr:`distance_to_goals`
    (``distanceToGoals``), :attr:`attempt` and :attr:`second_strategy`. They persist between
    calls, as in Java (``attempt`` selects the sub-goal strategy of
    :meth:`create_sub_problem`: even = greedy, odd = rotate).

    Args:
        compat: ``"fixed"`` (default) or ``"original"`` (see the module docstring).
        backend, params: LP backend for the sub-problems (``params={"Threads": 1}`` with
            ``backend="gurobi"`` reproduces the original's sub-plans).
        rng: the random stream (``None``: the module stream).
        clock: time source in seconds (``None``: the virtual clock that never advances).
        max_subproblems: sub-problems one attempt may examine (``None``: unlimited).
        heuristic: ``PathFinder.bHeuristic`` (see the module docstring).
        feasibility_budget: passed to the sub-solver (its infeasibility detection).
        echo: also print the log.
    """

    variant = "PathPlanner"
    expand_threshold = 0.5

    def __init__(self, *, compat: str = "fixed", backend: str = "highs", params: dict | None = None,
                 rng: MathRandom | None = None, clock: Callable[[], float] | None = None,
                 max_subproblems: int | None = DEFAULT_MAX_SUBPROBLEMS, heuristic: bool | None = None,
                 feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET, echo: bool = False):
        self.orig = check_compat(compat)
        self.compat = compat
        self.backend = backend
        self.params = params
        self.rng = rng
        self.clock = clock if clock is not None else _frozen_clock
        self.max_subproblems = max_subproblems
        self.heuristic = heuristic
        self.feasibility_budget = feasibility_budget
        self.echo = echo
        self.distance_to_goals: list[list[int]] | None = None
        self.attempt = 0
        self.second_strategy = False
        self._dist_key = None
        self._out = _Out(echo)

    # ---------------------------------------------------------------- plumbing
    def _rng(self) -> MathRandom:
        return resolve_rng(self.rng)

    def _h(self) -> bool:
        """The ``bHeuristic`` value of this planner's own path searches."""
        if self.heuristic is not None:
            return bool(self.heuristic)
        return bool(B_HEURISTIC) if self.orig else False

    def _shortest(self, g, start, goal):
        return find_shortest_paths(g, start, goal, self._rng(), heuristic=self._h(), compat=self.compat)

    def _with_flag(self, fn):
        """Run ``fn`` with :data:`B_HEURISTIC` set to this planner's flag (``original``
        mode with an explicit ``heuristic``: the sub-solvers read the global)."""
        try:
            if not (self.orig and self.heuristic is not None):
                return fn()
            old = B_HEURISTIC.value
            B_HEURISTIC.set(bool(self.heuristic))
            try:
                return fn()
            finally:
                B_HEURISTIC.set(old)
        except MPPError as e:
            # what the original printed before it threw / hung
            if getattr(e, "output", None) is None:
                e.output = self._out.text()
            raise

    def _set_distances(self, graph: Graph, goal) -> None:
        self.distance_to_goals = _goal_distances(graph, goal, self.orig)
        # A strong reference compared with ``is`` (an ``id()`` could be reused by a new
        # graph once the old one is freed), plus the vertex count as a cheap guard against
        # in-place growth.
        self._dist_key = (graph, graph.n, tuple(goal))

    def _dist_stale(self, graph: Graph, goal) -> bool:
        k = self._dist_key
        return (self.distance_to_goals is None or k is None or k[0] is not graph or k[1] != graph.n
                or k[2] != tuple(goal))

    def _ensure_distances(self, graph: Graph, goal) -> None:
        """``fixed``: (re)compute a missing or stale table."""
        if not self.orig and self._dist_stale(graph, goal):
            self._set_distances(graph, goal)

    def _dist(self, a: int, v: int) -> int:
        d = self.distance_to_goals
        if d is None:
            raise JavaNullPointerException(_NPE_DIST)
        if not 0 <= a < len(d):
            raise _aioobe(a, len(d))
        row = d[a]
        if not 0 <= v < len(row):
            raise _aioobe(v, len(row))
        return row[v]

    def _nb(self, g: Graph, v: int) -> list[int]:
        nb = g.neighbors(v)
        if nb is None:
            if self.orig:
                raise JavaNullPointerException(
                    'Cannot invoke "java.util.Set.toArray(Object[])" because the return value of '
                    '"java.util.Map.get(Object)" is null')
            raise ValueError(f"vertex {v} has no adjacency entry")
        return nb

    def _result(self, paths, goal, stop, attempts, extra=None) -> SolveResult:
        k = number_of_goals_reached(paths, goal, self.compat) if paths is not None else 0
        ex = {"goals_reached": k, "stop": stop}
        if self.orig:
            ex["statics_after"] = dict(_STATICS_AFTER)
        if extra:
            ex.update(extra)
        status = "feasible" if paths is not None and k == len(goal) else "partial"
        obj = None if paths is None else (len(paths[0]) - 1 if paths else 0)
        return SolveResult(paths=paths, status=status, objective=obj, T=obj, log=self._out.text(),
                           attempts=attempts, extra=ex)

    # ------------------------------------------------------- sub-problem region
    def _region(self, g: Graph, svs: Sequence[int], distance: int):
        """The ``involved`` / ``unOccupied`` sets of ``createSubProblem`` (with the
        region draws)."""
        rng = self._rng()
        involved = JavaIntHashSet()
        unocc = JavaIntHashSet()
        for s in svs:
            involved.add(s)
            unocc.add(s)
            for u in list(self._nb(g, s)):
                involved.add(u)
                if distance > 1:
                    for w in list(self._nb(g, u)):
                        involved.add(w)
                        if distance > 2 or rng.random() > self.expand_threshold:
                            for x in list(self._nb(g, w)):
                                involved.add(x)
        return involved, unocc

    def _sub_graph(self, g: Graph, involved: JavaIntHashSet):
        """The sub-graph on ``involved`` (new ids in ``HashSet`` order; ``HashSet``
        adjacency: the vertex itself first, then its neighbours in ``g``'s order)."""
        vs = involved.to_list()
        vid_new = {v: i for i, v in enumerate(vs)}
        new_vid = {i: v for i, v in enumerate(vs)}
        sub = Graph()
        for i in range(len(vs)):
            sub.xy[i] = None
        for i, v in enumerate(vs):
            sub.set_hashset_adjacency(i, [i] + [vid_new[u] for u in self._nb(g, v) if u in vid_new])
        sub.finish_building_graph()
        return sub, vid_new, new_vid

    def _greedy_goals(self, paths, ct, involved, unocc, vid_new):
        """Greedy sub-goals: per robot in the region (index order), the vertex of
        ``unOccupied`` (= the region) nearest to its goal; ties go to the **last** such
        vertex in ``HashSet`` order (``TreeMap.put`` overwrites); then it is removed."""
        for x in involved.to_list():
            unocc.add(x)
        new_start, new_goal = [], []
        for a in range(len(paths)):
            v = paths[a][ct]
            if v in vid_new:
                new_start.append(vid_new[v])
                best: dict[int, int] = {}
                for v2 in unocc.to_list():
                    best[self._dist(a, v2)] = v2
                if not best:  # TreeMap.firstKey() on an empty map
                    raise ValueError("more robots than vertices in the region")
                gid = best[min(best)]
                new_goal.append(vid_new[gid])
                unocc.remove(gid)
        return new_start, new_goal

    def _problem(self, sub, vid_new, new_vid, new_start, new_goal) -> Problem:
        p = Problem(sub, new_start, new_goal[:len(new_start)])
        p.vid_new_id_map = vid_new
        p.new_id_vid_map = new_vid
        return p

    def create_sub_problem(self, g: Graph, paths: Sequence[Sequence[int]], svs: Sequence[int],
                           goal: Sequence[int], distance: int = 2) -> Problem:
        """``PathPlanner.createSubProblem(g, paths, svs, goal, distance)``.

        The region is ``svs`` plus everything within distance 2 (``distance > 1``), each
        2-hop vertex ``w`` being expanded by one more ring when ``distance > 2`` or a draw
        is ``> 0.5`` (one draw per ``(s, u, w)`` triple). The robots are those with
        ``paths[a][0]`` in the region (index order). Sub-goals: :attr:`attempt` even ->
        greedy nearest-to-goal (``distance_to_goals``); odd -> a robot standing on
        ``svs[i]`` goes to ``svs[i+1]`` (cyclically), the others stay. Returns a
        :class:`~mppflow.problem.Problem` with ``vid_new_id_map`` / ``new_id_vid_map``; the
        sub-graph adjacency carries the original's ``HashSet`` order (``adj_order``).

        ``original``: with no distance table (even attempt) ``JavaNullPointerException``
        after the region draws, and ``goal`` is unused. ``fixed`` computes the table from
        ``goal`` when it is missing or stale.
        """
        svs = list(svs)
        if self.attempt % 2 == 0:
            self._ensure_distances(g, goal)
        involved, unocc = self._region(g, svs, distance)
        sub, vid_new, new_vid = self._sub_graph(g, involved)
        if self.attempt % 2 == 0:
            new_start, new_goal = self._greedy_goals(paths, 0, involved, unocc, vid_new)
        else:
            new_start, new_goal = [], []
            for a in range(len(paths)):
                v = paths[a][0]
                if v in vid_new:
                    new_start.append(vid_new[v])
                    if v in unocc:
                        for i, sv in enumerate(svs):
                            if v == sv:
                                new_goal.append(vid_new[svs[0] if i == len(svs) - 1 else svs[i + 1]])
                    else:
                        new_goal.append(vid_new[v])
        return self._problem(sub, vid_new, new_vid, new_start, new_goal)

    # ------------------------------------------------------ independent moves
    def move_agents_independently_when_can(self, g: Graph, start: list[int], goal: Sequence[int],
                                           path_array: list[list[int]]) -> bool:
        """``PathPlanner.moveAgentsIndependentlyWhenCan(g, start, goal, pathArray)``.

        Computes shortest paths from ``start`` (draws), then advances in rounds: in each
        round every robot not at the end of its path moves to its next vertex if that
        vertex was not occupied at the start of the round and was not just entered by a
        lower-index robot, otherwise it waits; every robot gets one vertex appended per
        round (so the rows stay equal in length). Stops when no robot can move. Updates
        ``start`` and ``path_array`` in place (both modes: this is the helper's contract)
        and returns whether anything was appended.
        """
        paths = _parse_paths(self._shortest(g, start, goal), self.orig)
        if not paths:
            if self.orig:
                raise _aioobe(0, 0)
            return False
        if len(paths[0]) == 1:
            return False
        length = len(path_array[0])
        n = len(paths)
        cur = [0] * n
        count = 5
        while count > 0:
            count = 0
            occ = {paths[a][cur[a]] for a in range(n)}
            movable = any(cur[a] < len(paths[a]) - 1 and paths[a][cur[a] + 1] not in occ for a in range(n))
            if not movable:
                break
            for a in range(n):
                if cur[a] < len(paths[a]) - 1:
                    nv = paths[a][cur[a] + 1]
                    if nv not in occ:
                        path_array[a].append(nv)
                        occ.add(nv)
                        cur[a] += 1
                        count += 1
                    else:
                        path_array[a].append(paths[a][cur[a]])
                else:
                    path_array[a].append(paths[a][cur[a]])
        for a in range(n):
            start[a] = paths[a][cur[a]]
        return len(path_array[0]) - length > 0

    # --------------------------------------------------- conflict sub-problems
    def _conflicts(self, g, paths, ct, goal, make):
        """The sub-problems of one round, in the original's order: swaps (in the order of
        the second robot of each pair), then meets (in the order of the second robot
        entering each vertex). ``make(svs)`` builds one (with its draws)."""
        n = len(paths)
        subs = []
        eset = set()
        for a in range(n):
            v1, v2 = paths[a][ct], paths[a][ct + 1]
            k = _edge_key(v1, v2, self.orig)
            if k in eset:
                subs.append(make([v1, v2]))
            else:
                eset.add(k)
        vset, processed = set(), set()
        for a in range(n):
            v = paths[a][ct + 1]
            if v in vset and v not in processed:
                svs = [paths[aa][ct] for aa in range(n) if paths[aa][ct + 1] == v]
                subs.append(make(svs))
                processed.add(v)
            else:
                vset.add(v)
        return subs

    def _solve_subs(self, g, subs, path_array, pos_of, timer, time_limit, state, on_solved):
        """The sub-problem loop of one round. Returns ``None`` normally, or the stop reason
        (``"time_limit"`` / ``"budget"``) when the attempt must return its partial paths."""
        progress = False
        to_add = 0
        start_length = len(path_array[0])
        used: set[int] = set()
        for p in subs:
            state["examined"] += 1
            if self.max_subproblems is not None and state["examined"] > self.max_subproblems:
                return "budget"
            time_left = time_limit - timer.elapsed()
            # original: ``timeLeft < 0`` lets an exactly spent budget (0.0) through, and the
            # sub-solver then runs without any limit (hasTimeLimit = timeLeft > 0 is false;
            # original-bugs.md, "heuristic-zero-time-left"); fixed stops at 0, too
            if time_left < 0 or (not self.orig and time_left <= 0):
                return "time_limit"
            region = [p.new_id_vid_map[v] for v in p.graph.vertices]
            if any(v in used for v in region):
                state["log"].append({"round": state["round"], "region": len(region), "agents": len(p.starts),
                                     "status": "skipped"})
                continue
            sub_ps, res = _plan_paths_advanced(
                p.graph, p.starts, p.goals, True, 0, True, time_left, out=self._out, rng=self._rng(),
                backend=self.backend, params=self.params, compat=self.compat, clock=self.clock,
                feasibility_budget=self.feasibility_budget)
            state["log"].append({"round": state["round"], "region": len(region), "agents": len(p.starts),
                                 "status": "solved" if sub_ps is not None else "failed",
                                 "T": None if sub_ps is None else len(sub_ps[0]) - 1})
            if sub_ps is not None:
                agent_of = {}
                for i, s in enumerate(p.starts):
                    agent_of[s] = i
                for a in range(len(path_array)):
                    v = pos_of(a)
                    if v in p.vid_new_id_map:
                        k = agent_of[p.vid_new_id_map[v]]
                        for t in range(1, len(sub_ps[0])):
                            path_array[a].append(p.new_id_vid_map[sub_ps[k][t]])
                        on_solved(a)
                progress = True
                used.update(region)
                state["solved"] += 1
                to_add = max(to_add, len(sub_ps[0]) - 1)
        if progress:
            exp = start_length + to_add
            for row in path_array:
                if len(row) < exp:
                    row.extend([row[-1]] * (exp - len(row)))
        return None

    # -------------------------------------------------------------- one attempt
    def plan_heuristic_path(self, g: Graph, start: list[int], goal: Sequence[int], time_limit: float) -> SolveResult:
        """``PathPlanner.planHeuristicPath(g, start, goal, timeLimit)``: one attempt of the
        local-repair heuristic (see the module docstring).

        Each round: move robots independently while possible
        (:meth:`move_agents_independently_when_can`, repeated), recompute shortest paths
        (done if everyone is at its goal), build the swap and meet sub-problems
        (:meth:`create_sub_problem`, ``distance = 2``), solve the vertex-disjoint ones in
        order (``planPathsAdvanced(sub, LP first, setOptimal, timeLeft)``), splice their
        plans in and pad all rows; a round that adds nothing counts as a stall, and the 5th
        stall (cumulative) ends the attempt. ``time_limit`` is checked before each
        sub-problem (see "Time budgets" in the module docstring; with the default virtual
        clock it never expires).

        ``original`` mutates ``start`` (it ends as the robots' final positions); ``fixed``
        works on a copy. Other differences: see the module docstring.
        """
        def run():
            return self._plan_heuristic_path(g, start, goal, time_limit)
        return self._with_flag(run)

    def _begin(self, g, start, goal):
        if not self.orig:
            _validate(g, start, goal)
            self._ensure_distances(g, goal)

    def _plan_heuristic_path(self, g, start, goal, time_limit, out_reset=True):
        if out_reset:
            self._out = _Out(self.echo)
        timer = _Timer(self.clock, self.orig)
        goal = list(goal)
        self._begin(g, start, goal)
        if not self.orig:
            start = list(start)
        n = len(start)
        if n == 0:
            if self.orig:
                raise _aioobe(0, 0)
            return self._result([], goal, "done", [], {"rounds": 0, "examined": 0, "solved": 0})
        path_array = [[v] for v in start]
        stalled = 0
        state = {"examined": 0, "solved": 0, "round": 0, "log": []}

        def finish(stop):
            return self._result(_parse_paths(path_array, self.orig), goal, stop, state["log"],
                                {"rounds": state["round"], "examined": state["examined"], "solved": state["solved"]})

        while True:
            state["round"] += 1
            begin = len(path_array[0])
            while self.move_agents_independently_when_can(g, start, goal, path_array):
                pass
            paths = _parse_paths(self._shortest(g, start, goal), self.orig)
            if len(paths[0]) == 1:
                return finish("done")

            def make(svs):
                return self.create_sub_problem(g, paths, svs, goal, 2)

            subs = self._conflicts(g, paths, 0, goal, make)

            def on_solved(a):
                start[a] = path_array[a][-1]

            stop = self._solve_subs(g, subs, path_array, lambda a: start[a], timer, time_limit, state, on_solved)
            if stop is not None:
                return finish(stop)
            if len(path_array[0]) == begin:
                stalled += 1
                if stalled == 5:
                    return finish("stalled")

    # ------------------------------------------------------------- attempts
    @staticmethod
    def attempt_schedule(n_agents: int) -> tuple[int, float, float]:
        """``(totalAttempts, attemptTimeOut, increment)`` of ``PathPlanner.planHeuristicPaths``
        by robot count; attempt ``k`` (1-based) gets ``attemptTimeOut + k*increment`` s."""
        table = {25: (8, 1.0, 0.1), 50: (7, 1.5, 0.2), 75: (6, 2.0, 0.25), 100: (5, 4.0, 0.5),
                 125: (4, 6.0, 0.5), 150: (4, 8.0, 0.0)}
        return table.get(n_agents, (10, 3.0, 0.0))

    def plan_heuristic_paths(self, graph: Graph, start: Sequence[int], goal: Sequence[int]) -> SolveResult:
        """``PathPlanner.planHeuristicPaths(graph, start, goal)``: compute the goal distance
        table, then run up to :meth:`attempt_schedule` attempts (:meth:`plan_heuristic_path`
        on copies of ``start`` / ``goal``, printing ``"Attempting with X seconds..."``),
        returning the first one that reaches every goal, else the last (partial) one.

        The loop variable is the instance field :attr:`attempt`, as in Java: it stays at the
        successful attempt's number, or ends at ``totalAttempts + 1`` (11 by default, odd:
        rotation sub-goals) when every attempt fails; later direct :meth:`create_sub_problem`
        / :meth:`plan_heuristic_path` calls see that value (both modes). Differences of ``fixed``: see the module docstring.
        """
        return self._with_flag(lambda: self._plan_heuristic_paths(graph, start, goal))

    def _attempt_limits(self, n):
        total, timeout, inc = self.attempt_schedule(n)
        return [(k, timeout + k * inc) for k in range(1, total + 1)]

    def _plan_heuristic_paths(self, graph, start, goal):
        self._out = _Out(self.echo)
        start, goal = list(start), list(goal)
        if not self.orig:
            _validate(graph, start, goal)
        self._set_distances(graph, goal)
        self.second_strategy = False
        attempts = []
        res = None
        limits = self._attempt_limits(len(start))
        for k, tl in limits:
            self._set_attempt(k)
            self._out.println(f"Attempting with {java_double_str(tl)} seconds...")
            res = self._plan_heuristic_path(graph, list(start), list(goal), tl, out_reset=False)
            attempts.append({"attempt": k, "time_limit": tl, "stop": res.extra["stop"],
                             "goals_reached": res.extra["goals_reached"], "makespan": res.objective,
                             "examined": res.extra["examined"], "solved": res.extra["solved"]})
            if res.paths is not None and res.extra["goals_reached"] == len(goal):
                break
            self._after_attempt()
        else:
            # Java's ``for (attempt = 1; attempt <= totalAttempts; attempt++)`` runs on the
            # field: after the last failed attempt it is incremented once more.
            self._set_attempt(limits[-1][0] + 1 if limits else 1)
        out = self._result(res.paths, goal, res.extra["stop"], attempts,
                           {"rounds": res.extra["rounds"], "examined": res.extra["examined"],
                            "solved": res.extra["solved"]})
        return out

    def _set_attempt(self, k: int) -> None:
        self.attempt = k

    def _after_attempt(self) -> None:
        # ``if (secondStrategy = false) {...}`` assigns false and never runs its body
        self.second_strategy = False


class HeuristicPathPlanner(LocalRepairPlanner):
    """``HeuristicPathPlanner`` (extends ``PathPlanner``): the ICRA 2013 Sec. VI-C
    heuristic with synchronous advancing. Same constructor arguments as
    :class:`LocalRepairPlanner`; :attr:`attempt` and :attr:`second_strategy` are inherited
    but unused, as in the original."""

    variant = "HeuristicPathPlanner"
    expand_threshold = 0.75

    def _attempt_limits(self, n):
        return [(i, 1.5 + ((5 + i * 4) * n) / 120.0) for i in range(3)]

    def _set_attempt(self, k: int) -> None:
        pass  # the inherited field is not used

    def _after_attempt(self) -> None:
        pass

    def plan_heuristic_paths(self, graph: Graph, start: Sequence[int], goal: Sequence[int]) -> SolveResult:
        """``HeuristicPathPlanner.planHeuristicPaths(graph, start, goal)``: the goal
        distance table, then 3 attempts with budgets ``1.5 + (5 + 4 i) n / 120`` s
        (``i = 0, 1, 2``, printed as ``"Attempting with X seconds..."``); the first attempt
        that reaches every goal is returned, else the last one."""
        return self._with_flag(lambda: self._plan_heuristic_paths(graph, start, goal))

    def create_sub_problem(self, g: Graph, paths: Sequence[Sequence[int]], ct: int, svs: Sequence[int],
                           goal: Sequence[int], distance: int = 2) -> Problem:
        """``HeuristicPathPlanner.createSubProblem(g, paths, ct, svs, goal, distance)``.

        As :meth:`LocalRepairPlanner.create_sub_problem` with the robots' positions taken
        at step ``ct`` (``paths[a][ct]``), expansion threshold ``0.75`` (probability 1/4)
        and always greedy sub-goals."""
        svs = list(svs)
        self._ensure_distances(g, goal)
        involved, unocc = self._region(g, svs, distance)
        sub, vid_new, new_vid = self._sub_graph(g, involved)
        new_start, new_goal = self._greedy_goals(paths, ct, involved, unocc, vid_new)
        return self._problem(sub, vid_new, new_vid, new_start, new_goal)

    def plan_heuristic_path(self, g: Graph, s: Sequence[int], goal: Sequence[int], time_limit: float) -> SolveResult:
        """``HeuristicPathPlanner.planHeuristicPath(g, s, goal, timeLimit)``: one attempt.

        Each round: shortest paths from the current positions (done if everyone is at its
        goal); advance **all** robots along them in lockstep until the first step ``ct``
        with a head-on swap (:func:`exists_edge_collision`) or a meet
        (:func:`exists_vertex_collision`), returning at once if the paths run out without
        one; build the conflict sub-problems at ``ct`` (:meth:`create_sub_problem`), solve
        the vertex-disjoint ones, splice and pad; stall rule and budgets as in
        :meth:`LocalRepairPlanner.plan_heuristic_path`. ``s`` is not modified (both modes).
        """
        return self._with_flag(lambda: self._plan_heuristic_path(g, s, goal, time_limit))

    def _plan_heuristic_path(self, g, s, goal, time_limit, out_reset=True):
        if out_reset:
            self._out = _Out(self.echo)
        timer = _Timer(self.clock, self.orig)
        goal = list(goal)
        self._begin(g, s, goal)
        start = list(s)
        n = len(start)
        if n == 0:
            if self.orig:
                raise _aioobe(0, 0)
            return self._result([], goal, "done", [], {"rounds": 0, "examined": 0, "solved": 0})
        path_array = [[v] for v in start]
        stalled = 0
        state = {"examined": 0, "solved": 0, "round": 0, "log": []}

        def finish(stop):
            return self._result(_parse_paths(path_array, self.orig), goal, stop, state["log"],
                                {"rounds": state["round"], "examined": state["examined"], "solved": state["solved"]})

        while True:
            state["round"] += 1
            begin = len(path_array[0])
            paths = _parse_paths(self._shortest(g, start, goal), self.orig)
            if len(paths[0]) == 1:
                return finish("done")
            ct = 0
            L = len(paths[0])
            while (ct < L and exists_edge_collision(paths, ct, self.compat) is None
                   and exists_vertex_collision(paths, ct, self.compat) is None):
                ct += 1
                move_agents_independently_one_step(path_array, paths, ct, self.compat)
                if ct == L - 1:
                    return finish("done")

            cur_ct = ct

            def make(svs):
                return self.create_sub_problem(g, paths, cur_ct, svs, goal, 2)

            subs = self._conflicts(g, paths, ct, goal, make)
            stop = self._solve_subs(g, subs, path_array, lambda a: paths[a][cur_ct], timer, time_limit, state,
                                    lambda a: None)
            if stop is not None:
                return finish(stop)
            if len(path_array[0]) == begin:
                stalled += 1
                if stalled == 5:
                    return finish("stalled")
            for a in range(n):
                start[a] = path_array[a][-1]


# ------------------------------------------------------------- module-level entry
def plan_heuristic_paths(graph: Graph, start: Sequence[int], goal: Sequence[int], *, compat: str = "fixed",
                         backend: str = "highs", params: dict | None = None, rng: MathRandom | None = None,
                         clock: Callable[[], float] | None = None,
                         max_subproblems: int | None = DEFAULT_MAX_SUBPROBLEMS, heuristic: bool | None = None,
                         feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET, echo: bool = False) -> SolveResult:
    """``new PathPlanner().planHeuristicPaths(graph, start, goal)``: a fresh
    :class:`LocalRepairPlanner` (keyword arguments as its constructor) running
    :meth:`LocalRepairPlanner.plan_heuristic_paths`. Returns its
    :class:`~mppflow.solvers.SolveResult` (``.paths``, ``.log`` = the printed text). This
    is the entry point :func:`mppflow.planner.path_planner_main` calls by default."""
    pl = LocalRepairPlanner(compat=compat, backend=backend, params=params, rng=rng, clock=clock,
                            max_subproblems=max_subproblems, heuristic=heuristic,
                            feasibility_budget=feasibility_budget, echo=echo)
    return pl.plan_heuristic_paths(graph, start, goal)


# ------------------------------------------------------------------------ mains
def _main(planner_cls, *, compat, backend, params, rng, clock, max_subproblems, echo) -> str:
    out = _Out(echo)
    p = get_long_straight_with_one_garage_problem()
    out.print(print_start_and_goals(p.graph, p.starts, p.goals, compat))
    start, goal = list(p.starts), list(p.goals)
    pl = planner_cls(compat=compat, backend=backend, params=params, rng=rng, clock=clock,
                     max_subproblems=max_subproblems)
    res = pl.plan_heuristic_paths(p.graph, p.starts, p.goals)
    out.print(res.log)
    if res.paths is not None:
        out.print(print_paths(p.graph, res.paths, compat))
        ok, msg = _path_set_valid(res.paths, p.graph, start, goal)
        if ok:
            out.println("Path set is valid.")
        else:
            out.println(msg)
    return out.text()


def path_planner_main(*, compat: str = "fixed", backend: str = "highs", params: dict | None = None,
                      rng: MathRandom | None = None, clock: Callable[[], float] | None = None,
                      max_subproblems: int | None = DEFAULT_MAX_SUBPROBLEMS, echo: bool = False) -> str:
    """``PathPlanner.main``: :meth:`LocalRepairPlanner.plan_heuristic_paths` on
    ``getLongStraightWithOneGarageProblem`` (a path 0..8 with a garage vertex 9 on 4, two
    robots swapping ends); returns the printed text (start/goals, the log, the paths and
    "Path set is valid.")."""
    return _main(LocalRepairPlanner, compat=compat, backend=backend, params=params, rng=rng, clock=clock,
                 max_subproblems=max_subproblems, echo=echo)


def heuristic_path_planner_main(*, compat: str = "fixed", backend: str = "highs", params: dict | None = None,
                                rng: MathRandom | None = None, clock: Callable[[], float] | None = None,
                                max_subproblems: int | None = DEFAULT_MAX_SUBPROBLEMS, echo: bool = False) -> str:
    """``HeuristicPathPlanner.main``: as :func:`path_planner_main` with
    :class:`HeuristicPathPlanner`."""
    return _main(HeuristicPathPlanner, compat=compat, backend=backend, params=params, rng=rng, clock=clock,
                 max_subproblems=max_subproblems, echo=echo)
