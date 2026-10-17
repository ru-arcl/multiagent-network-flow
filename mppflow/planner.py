"""Path planning entry points, the k-way time split, and path utilities (port of
``ILP/PathPlanner.java``, without the local-repair heuristic).

Ports
-----

=================================================  =============================================
mppflow                                            original (``PathPlanner``)
=================================================  =============================================
:meth:`PathPlanner.plan_paths_advanced`            ``planPathsAdvanced`` (MINMAKESPAN)
:meth:`PathPlanner.plan_paths_advanced_pmg`        ``planPathsAdvancedPMG`` (one move per step)
:meth:`PathPlanner.plan_paths_advanced_tt`         ``planPathsAdvancedTT`` (MINTOTALTIME)
:meth:`PathPlanner.plan_paths_advanced_td`         ``planPathsAdvancedTD`` (MINTOTALDIST)
:meth:`PathPlanner.split_paths`                    ``splitPaths`` (intermediate goals, T-RO Fig. 8)
:meth:`PathPlanner.plan_paths_advanced_split`      ``planPathsAdvancedSplit`` (recursive split)
:meth:`PathPlanner.multi_threaded_split_planning`  ``multiThreadedSplitPlanning`` (k-way split)
:meth:`PathPlanner.plan_paths_advanced_split_no_cycle`  ``planPathsAdvancedSplitNoCycle``
:meth:`PathPlanner.plan_paths_advanced_split_dist` ``planPathsAdvancedSplitDist``
:meth:`PathPlanner.get_longest_path_length`        ``getLongestPathLength``
:meth:`PathPlanner.split_path_by_size`             ``splitPathBySize`` (*dead*)
:meth:`PathPlanner.plan_paths_advanced_split_by_size`   ``planPathsAdvancedSplitBySize`` (*dead*)
:meth:`PathPlanner.plan_paths_advanced_split_by_size_no_cycle`  ``...SplitBySizeNoCycle`` (*dead*)
:func:`merge_paths`, :func:`parse_paths`           ``mergePaths``, ``parsePaths``
:func:`is_path_set_valid`, :func:`path_set_error`  ``isPathSetValid`` (bool / its message)
:func:`has_cycles`, :func:`number_of_goals_reached`  ``hasCycles``, ``numberOfGoalsReached``
:func:`print_paths`, :func:`print_path_to_string`  ``printPaths``, ``printPathToString``
:func:`print_start_and_goals`                      ``printStartAndGoals``
:func:`path_planner_main`                          ``PathPlanner.main`` (demo)
:func:`plan`                                       *new* high-level API (:class:`PlanResult`)
=================================================  =============================================

The local-repair heuristic of ``PathPlanner`` (``planHeuristicPaths``, ``planHeuristicPath``,
``createSubProblem``, ``moveAgentsIndependently[WhenCan]``) is in :mod:`mppflow.heuristic`
(:class:`~mppflow.heuristic.LocalRepairPlanner`); :func:`path_planner_main` runs it.

State, output and randomness
----------------------------
The Java methods are instance methods of a planner that reads and writes the solvers'
static flags. :class:`PathPlanner` keeps those flags in ``planner.flags``
(:class:`SolverFlags`: all ``False`` by default, ``SolverFlags.java_defaults()`` gives the
original's defaults); changes the original makes to them (``bPrintPaths``) persist on the
planner instance, not globally. Every text the original prints (the solvers' messages,
"Maximum allowed time reached, optimization stopped.", the ``isPathSetValid`` messages,
``printPaths`` of split results) is collected in ``planner.log`` (``echo=True`` also
prints it); stack traces the original prints go to ``planner.stderr``. Each solve's
:class:`~mppflow.solvers.SolveResult` is appended to ``planner.solves``.

Randomness: ``planner.rng`` (a :class:`~mppflow._java.MathRandom`) stands in for
``Math.random()``. Draws are consumed exactly as in the original: one
:func:`~mppflow.pathfinder.find_shortest_paths` at the start of every
``splitPaths``/``splitPathBySize``/``getLongestPathLength`` and of every solve, plus
``n`` draws for a random agent order (``splitPaths(..., true)``). The recursive variants
consume them depth first (parent split, left subtree, right subtree).
``multiThreadedSplitPlanning`` computes every split in the calling thread (level by level)
and then runs one thread per piece; here the pieces run **sequentially, in piece order**,
so their draws are consumed in thread-index order. The plans are the same as those of
every Java run that does not lose a piece (the threads' draws only break ties among
shortest paths); the generator state after the call differs from a particular Java run,
which is itself not deterministic (original-bugs B14). Java's workers also store their
pieces in an unsynchronized ``HashMap``:
workers that finish at about the same moment can lose a ``put``, and the call then returns
``null`` silently (original-bugs.md, B14 item 4; 5-24% of the runs on a 2-robot instance
whose pieces are one step long). This timing-dependent failure is not reproduced in either
mode: the port always returns the plan. Each piece still gets the full time limit,
as each Java thread does, so the wall time can be up to ``pieces`` times the limit.

Infinite loops and crashes (``compat="original"``)
---------------------------------------------------
* ``splitPaths`` widens its distance window forever when every vertex it may pick is
  already some earlier agent's intermediate goal (original-bugs B10); we
  raise :class:`~mppflow.errors.OriginalWouldHang` once both distance rings cover the
  whole component (the candidates can no longer change). ``splitPathBySize`` likewise
  once its lower bound reached 0 (original-bugs B15).
* ``planPathsAdvancedSplitNoCycle`` retries forever at top level (``tries-- > 0 || top``)
  while the result has rotations (original-bugs B12). From retry
  ``retry_analysis_after`` (20) on we decide, by enumerating every intermediate-goal
  assignment the random re-split can produce and solving the resulting pieces, whether
  any retry can end the loop; if none can, :class:`OriginalWouldHang` is raised. The
  analysis runs in installments of at most ``analysis_budget`` (200) new leaf solves, one
  per retry, and keeps what it computed (the leaves go to the call's leaf cache, which
  the loop reuses), so it always decides once all its leaves are solved: an 8-robot 3x3
  instance (``splits = 1``) with 182 possible re-splits is decided at retry 21 (364
  leaves), an 11-robot 3x4 instance with ``splits = 2`` (about 3000 re-splits of the
  retried half, some 6000 leaves) after about 400 s with Gurobi. It examines the
  re-splits likeliest first (ties in discovery order), so the retry at which it decides
  never depends on ``PYTHONHASHSEED`` or on memory addresses. It is exact when the
  pieces' solves are deterministic (no binding time limit). It gives up for good (the
  loop then simply continues, as in Java) only when enumerating the re-splits alone
  exceeds its step bound (``_ANALYSIS_STEPS`` per installment). The number of re-splits
  grows quickly with the number of robots, so on large dense instances deciding can take
  far longer than these examples. A retried piece that comes back ``null`` makes the next
  ``hasCycles`` throw (:class:`~mppflow.errors.JavaNullPointerException`, original-bugs
  B12). The same analysis covers ``planPathsAdvancedSplitBySizeNoCycle``.
* ``planPathsAdvancedSplitBySize[NoCycle]`` recurse forever when a sub-instance repeats
  on the call stack; Java then throws ``StackOverflowError``
  (:class:`JavaStackOverflowError`).
* ``multiThreadedSplitPlanning`` waits forever when a worker throws (``mdone[i]`` is never
  set, original-bugs B14): :class:`OriginalWouldHang`. Its ``mpath`` map is
  an instance field that is never cleared (original-bugs B14); ``planner.mpath``
  reproduces that.
* ``planPathsAdvancedSplitDist`` dereferences a ``null`` makespan result
  (:class:`JavaNullPointerException`, original-bugs B22).

compat modes
------------
``compat="fixed"`` equals the original wherever the original is correct (same
intermediate goals, same pieces, same random draws, same models). Differences of
``fixed``:

* ``splitPaths``: a robot's own start and goal may be its intermediate goal (each BFS
  ring also holds its centre at distance 0). The original can never pick them, so a
  robot at its goal, or one step from it, is sent on a detour (original-bugs B11:
  makespan 0 -> 2, 1 -> 3 or 6 after one split). The
  windows, their widening, the ``HashSet`` order and the draws are the original's, so the
  choice only changes once a window reaches an endpoint (``dMin == 0`` and ``dMax`` at
  least the start-goal distance; for a robot with ``L <= 2`` at once). Where the window
  can no longer produce a free vertex the original loops forever; fixed then uses the
  start, else the goal, else any free vertex that reaches the goal (with the endpoints
  eligible the saturated window is the whole component, which always has a free vertex,
  so this is only a guard). Two more corrections of the windows (see :class:`_SplitWindows`):
  the goal side is measured to the goal, ``dist(v, goal)`` (over the reversed arcs of a
  one-way graph; the original's forward BFS from the goal can offer a vertex that cannot
  reach the goal, as in its reachability pruning, original-bugs B8), and for an odd distance
  ``D`` the first candidates are the middle vertices of the shortest paths (the paper's
  middle node; the original's first window ``ceil(D/2)`` from both ends holds only vertices
  one step off every shortest path, which gave a lone robot a detour on 8-connected grids).
  On symmetric bipartite graphs (4-connected grids) neither changes the choice.
  ``planner._split_endpoints = False`` restores the original's rules (tests use it to show
  that these are the only differences on the golden split records).
* Split variants: a piece that is infeasible makes the split return ``None`` (``fixed``
  solvers raise :class:`~mppflow.errors.Infeasible`; the original loops forever on it), and
  so does a split that finds no intermediate goal (the failure is recorded in
  ``planner.failures``); only a goal unreachable from its start raises.
* ``multiThreadedSplitPlanning``: ``time_limit <= 0`` means no limit (the original always
  passes ``hasTimeLimit=true``, so with ``-1`` every piece that is infeasible at its first
  horizon fails, original-bugs B14); the pieces of each call are kept
  apart (no stale ``mpath``); a worker exception propagates (no hang).
* ``planPathsAdvancedSplitNoCycle``: the top-level retries run past ``3^(4-splits)`` as
  in the original (so a loop that ends in the original gives the same plan, solves and
  draws), but stop (a) when the endless-loop analysis (above, run in installments from
  retry ``retry_analysis_after`` on, or from the first retry beyond the original's
  bounded tries if that comes first, with a leaf cache of its own) shows that no retry can
  give a plan without rotations; (b) beyond the original's bounded tries, once the
  loop's retries and its analysis have spent ``max_retry_solves`` (1000) solves (the
  first split and the bounded tries are the original's own work and always run; with
  ``splits = 3`` they can by themselves take a few thousand solves); or
  (c) beyond those tries, once a positive ``time_limit`` has elapsed (the original
  ignores the time in this loop). The plan with rotations is then returned. With
  ``splits >= 2``, once the top-level retries of the first half stop with rotations, the
  second half is not retried (the plan keeps rotations whatever it is). A
  retried piece that comes back ``None`` where the original's ``hasCycles`` would throw
  stops the retries and the last complete pair of pieces is kept (the original throws
  ``NullPointerException``, or returns ``null`` when retries run out on such a pair);
  ``bPrintPaths`` is restored on every exit (the original leaves it ``false`` after
  ``splits == 0`` and after a failed first split, original-bugs B12).
* ``planPathsAdvancedSplitDist``: ``time_limit <= 0`` means no limit (the original then
  always returns ``null`` after solving the first half, original-bugs B13); a failed
  piece makespan returns ``None`` (original:
  ``NullPointerException``); the distance solve of a piece gets the time that is left
  after its makespan solve (the original gives it the full limit again).
* ``getLongestPathLength`` returns the largest number of vertices on a shortest path (the
  original compares edges but stores vertices and can return one less, original-bugs B15).
* ``splitPathBySize``: the goal-side window uses edges (``D - d``, the original the vertex
  count, off by one), the endpoints are eligible at distance 0, the window widens on both
  sides, and the saturation fallback of ``splitPaths`` applies (the original's ``dMax``
  never grows and the loop can spin forever). ``planPathsAdvancedSplitBySize[NoCycle]``
  solve a sub-instance that repeats on the call stack directly instead of recursing
  forever; ``...NoCycle`` keeps the last complete pair when a retry fails, and its
  top-level retries go past the 5 tries and stop as those of
  ``planPathsAdvancedSplitNoCycle``.
* ``planPathsAdvancedTT`` / ``planPathsAdvancedTD``: ``horizon=None`` follows the solvers'
  default, Algorithm 1's horizon search in ``fixed`` (see :func:`~mppflow.solvers.solve_total_time`);
  pass ``horizon="single"`` for the original's single horizon.
* Utilities: ``None``, empty or ragged inputs that make the original throw raise
  ``ValueError`` (or are handled, see each function); :func:`print_paths` and
  :func:`print_path_to_string` look vertices up by id (the original indexes
  ``vertices[]`` by id, which prints the wrong vertex on graphs whose ids are not
  ``0..n-1``); :func:`merge_paths` checks that the pieces join; :func:`path_set_error`
  rejects ragged path sets and a wrong number of paths, and reports a missing adjacency as
  an invalid move. :func:`is_path_set_valid` returns the verdict and does not print unless
  ``echo=True``.
* Any solver difference of :mod:`mppflow.solvers` applies (e.g. duplicate starts raise
  ``ValueError``).

Not reproduced in either mode: the interleaving of the threads' console output, Gurobi's
own log, and the wall-clock races of the original (time limits are honoured, but which
piece times out under load is machine dependent).
"""
from __future__ import annotations

import math
import sys
import time
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Callable, ClassVar, Sequence

from ._java import JavaIntHashSet, MathRandom, resolve_rng
from .errors import (
    Infeasible,
    JavaArrayIndexOutOfBoundsException,
    JavaException,
    JavaNegativeArraySizeException,
    JavaNullPointerException,
    JavaStackOverflowError,
    NotInOriginal,
    OriginalWouldHang,
)
from .graph import Graph, as_ids, as_int, check_compat, print_vertex, print_vertex_to_buffer
from .network import _reversed_adjacency
from .pathfinder import (
    B_HEURISTIC,
    find_goals_in_distance_range,
    find_shortest_paths,
    get_distances_to_all_vertices,
)
from .solvers import (
    DEFAULT_FEASIBILITY_BUDGET,
    JavaGRBException,
    SolveResult,
    _check_budget,
    _check_gap,
    _check_instance,
    _check_time_limit,
    _distances,
    _trim,
    joint_feasibility,
    solve_distance,
    solve_max_distance,
    solve_time,
    solve_time_r,
    solve_total_time,
)

__all__ = [
    "JavaStackOverflowError",
    "SolverFlags",
    "PathPlanner",
    "PlanResult",
    "plan",
    "plan_metrics",
    "print_paths",
    "print_path_to_string",
    "print_start_and_goals",
    "number_of_goals_reached",
    "parse_paths",
    "has_cycles",
    "path_set_error",
    "is_path_set_valid",
    "validate",
    "merge_paths",
    "split_paths",
    "split_path_by_size",
    "get_longest_path_length",
    "plan_paths_advanced",
    "plan_paths_advanced_pmg",
    "plan_paths_advanced_tt",
    "plan_paths_advanced_td",
    "plan_paths_advanced_split",
    "plan_paths_advanced_split_dist",
    "plan_paths_advanced_split_no_cycle",
    "plan_paths_advanced_split_by_size",
    "plan_paths_advanced_split_by_size_no_cycle",
    "multi_threaded_split_planning",
    "path_planner_main",
]


# ------------------------------------------------------------------ Java helpers
def _aioobe(i: int, n: int) -> JavaArrayIndexOutOfBoundsException:
    return JavaArrayIndexOutOfBoundsException(f"Index {i} out of bounds for length {n}")


def _at(arr, i: int, what: str = "<array>"):
    """Java ``arr[i]``: NPE for ``null``, AIOOBE out of range."""
    if arr is None:
        raise JavaNullPointerException(f'Cannot load from array because "{what}" is null')
    if i < 0 or i >= len(arr):
        raise _aioobe(i, len(arr))
    return arr[i]


def _jpow_int(base: float, e: int) -> int:
    """Java ``(int) Math.pow(base, e)``."""
    v = float(base) ** e
    if v >= 2147483647:
        return 2147483647
    return int(v)


@dataclass
class SolverFlags:
    """The original's static solver flags, as seen by one :class:`PathPlanner`.

    Defaults are all ``False`` (quiet); :meth:`java_defaults` gives the values the
    original declares. Field -> Java static: ``time_debug`` ->
    ``MultiagentGraphSolverGurobiTime.bDebugInfo``, ``time_print_paths`` -> ``...Time.bPrintPaths``,
    ``time_r_*`` -> ``...TimeR.*``, ``tt_*`` -> ``...TotalTime.*``, ``dist_*`` ->
    ``...Distance.*``, ``*_return_any_solution`` -> ``bReturnAnySolution``.
    """

    time_debug: bool = False
    time_print_paths: bool = False
    time_r_debug: bool = False
    time_r_print_paths: bool = False
    tt_debug: bool = False
    tt_print_paths: bool = False
    tt_return_any_solution: bool = False
    dist_debug: bool = False
    dist_print_paths: bool = False
    dist_return_any_solution: bool = False

    _JAVA: ClassVar[dict] = {
        "MultiagentGraphSolverGurobiTime.bDebugInfo": "time_debug",
        "MultiagentGraphSolverGurobiTime.bPrintPaths": "time_print_paths",
        "MultiagentGraphSolverGurobiTimeR.bDebugInfo": "time_r_debug",
        "MultiagentGraphSolverGurobiTimeR.bPrintPaths": "time_r_print_paths",
        "MultiagentGraphSolverGurobiTotalTime.bDebugInfo": "tt_debug",
        "MultiagentGraphSolverGurobiTotalTime.bPrintPaths": "tt_print_paths",
        "MultiagentGraphSolverGurobiTotalTime.bReturnAnySolution": "tt_return_any_solution",
        "MultiagentGraphSolverGurobiDistance.bDebugInfo": "dist_debug",
        "MultiagentGraphSolverGurobiDistance.bPrintPaths": "dist_print_paths",
        "MultiagentGraphSolverGurobiDistance.bReturnAnySolution": "dist_return_any_solution",
    }

    @classmethod
    def java_defaults(cls) -> "SolverFlags":
        """The values the original declares (everything printed, except
        ``Distance.bPrintPaths``; no ``bReturnAnySolution``)."""
        return cls(True, True, True, True, True, True, False, True, False, False)

    @classmethod
    def all(cls, value: bool) -> "SolverFlags":
        """Every debug/print flag set to ``value`` (``bReturnAnySolution`` off)."""
        return cls(value, value, value, value, value, value, False, value, value, False)

    def apply_java(self, statics: dict) -> "SolverFlags":
        """Set flags from ``{"MultiagentGraphSolverGurobiTime.bDebugInfo": False, ...}``
        (unknown names are ignored); returns ``self``."""
        for k, v in statics.items():
            name = self._JAVA.get(k)
            if name is not None:
                setattr(self, name, bool(v))
        return self


# --------------------------------------------------------------------- printing
def _emit_text(text: str, echo: bool) -> str:
    if echo:
        sys.stdout.write(text)
    return text


def _fail_with_output(e: Exception, parts: list, echo: bool):
    e.output = "".join(parts)  # type: ignore[attr-defined]
    if echo:
        sys.stdout.write(e.output)  # type: ignore[attr-defined]
    raise e


def _vertex_by_index(g: Graph, paths, a: int, t: int) -> int:
    """Java ``g.vertices[paths[a][t]]`` (the vertex id stored at that array index)."""
    row = _at(paths, a, "<parameter2>")
    v = _at(row, t, "<parameter2>[<local2>]")
    return _at(g.vertices, v, "g.vertices")


def print_paths(g: Graph, paths, compat: str = "fixed", *, echo: bool = False) -> str:
    """``printPaths(g, paths)``: ``"Agent a: 0:<v> 1:<v> ...\\n"`` per robot, ``<v>`` =
    ``Vertex.printVertex()`` (``id`` or ``id(x,y)``). Returns the text (``echo`` prints
    it).

    ``original``: every row uses ``paths[0].length`` and ``g.vertices[paths[a][t]]``
    (index = id), so ``null`` raises :class:`JavaNullPointerException` and a ragged array
    :class:`JavaArrayIndexOutOfBoundsException` (``exception.output`` holds the text
    printed before).

    Differs from the original: ``fixed`` prints each row to its own length, looks the
    vertex up by id (``ValueError`` for an unknown id) and raises ``ValueError`` for
    ``None``.
    """
    orig = check_compat(compat)
    parts: list[str] = []
    if paths is None:
        if orig:
            _fail_with_output(JavaNullPointerException('Cannot read the array length because "<parameter2>" is null'),
                              parts, echo)
        raise ValueError("paths is None")
    try:
        for a in range(len(paths)):
            parts.append(f"Agent {a}:")
            L = len(_at(paths, 0, "<parameter2>")) if orig else len(paths[a])
            for t in range(L):
                parts.append(f" {t}:")
                if orig:
                    vid = _vertex_by_index(g, paths, a, t)
                else:
                    vid = paths[a][t]
                    if vid not in g.xy:
                        raise ValueError(f"vertex {vid} is not in the graph")
                parts.append(print_vertex(g, vid))
            parts.append("\n")
    except JavaException as e:
        _fail_with_output(e, parts, echo)
    return _emit_text("".join(parts), echo)


def print_path_to_string(g: Graph, paths, compat: str = "fixed") -> str:
    """``printPathToString(g, paths)``: ``"Agent  1: 0:(x,y) 1:(x,y) ...\\n"`` per robot
    (1-based numbers, padded to two characters below 10). The vertex text is
    ``Vertex.printVertex(StringBuffer)``: the point **without the id**, or nothing for a
    vertex without coordinates (the original's format, kept in both modes).

    Crash and lookup behaviour as :func:`print_paths` (the original builds the string in
    a buffer, so nothing is output before a crash).
    """
    orig = check_compat(compat)
    if paths is None:
        if orig:
            raise JavaNullPointerException('Cannot read the array length because "<parameter2>" is null')
        raise ValueError("paths is None")
    parts: list[str] = []
    for a in range(len(paths)):
        parts.append("Agent " + (" " if a < 9 else "") + f"{a + 1}:")
        L = len(_at(paths, 0, "<parameter2>")) if orig else len(paths[a])
        for t in range(L):
            parts.append(f" {t}:")
            if orig:
                vid = _vertex_by_index(g, paths, a, t)
            else:
                vid = paths[a][t]
                if vid not in g.xy:
                    raise ValueError(f"vertex {vid} is not in the graph")
            parts.append(print_vertex_to_buffer(g, vid))
        parts.append("\n")
    return "".join(parts)


def print_start_and_goals(g: Graph, start: Sequence[int], goal: Sequence[int], compat: str = "fixed", *,
                          echo: bool = False) -> str:
    """``printStartAndGoals(g, start, goal)``: ``"Start and goal vertices: n pairs\\n"``,
    then the starts and then the goals, each as ``printVertex() + " "`` per vertex and a
    final ``" \\n"``. Returns the text.

    ``original``: an id that is not in the graph raises :class:`JavaNullPointerException`
    (``output`` = the text printed before). Differs from the original: ``fixed`` raises
    ``ValueError`` instead.
    """
    orig = check_compat(compat)
    parts = [f"Start and goal vertices: {len(start)} pairs\n"]
    for seq in (start, goal):
        for v in seq:
            if v not in g.xy:
                if orig:
                    _fail_with_output(JavaNullPointerException(
                        'Cannot invoke "projects.multipath.advanced.Vertex.printVertex()" because "v" is null'),
                        parts, echo)
                raise ValueError(f"vertex {v} is not in the graph")
            parts.append(print_vertex(g, v) + " ")
        parts.append(" \n")
    return _emit_text("".join(parts), echo)


# ---------------------------------------------------------------- path utilities
def number_of_goals_reached(paths, goal: Sequence[int], compat: str = "fixed") -> int:
    """``numberOfGoalsReached(paths, goal)``: the number of robots ``i < len(goal)`` whose
    path ends at ``goal[i]``.

    ``original``: missing or empty rows raise :class:`JavaArrayIndexOutOfBoundsException`
    (``null``: :class:`JavaNullPointerException`). Differs from the original: ``fixed``
    raises ``ValueError`` when there are fewer paths than goals and counts an empty path
    as not arrived.
    """
    orig = check_compat(compat)
    if orig:
        n = 0
        for i in range(len(goal)):
            row = _at(paths, i, "<parameter1>")
            if _at(row, len(row) - 1, "<parameter1>[<local3>]") == goal[i]:
                n += 1
        return n
    if paths is None or len(paths) < len(goal):
        raise ValueError("fewer paths than goals")
    return sum(1 for i in range(len(goal)) if paths[i] and paths[i][-1] == goal[i])


def parse_paths(paths, compat: str = "fixed") -> list[list[int]]:
    """``parsePaths(Path[] paths)``: a rectangular array, each path padded with its last
    vertex up to the longest one.

    ``original``: a ``null`` path raises :class:`JavaNullPointerException` (an unreachable
    goal of :func:`~mppflow.pathfinder.find_shortest_paths`), an empty path among longer
    ones :class:`JavaArrayIndexOutOfBoundsException`. Differs from the original: ``fixed``
    raises ``ValueError`` for both.
    """
    orig = check_compat(compat)
    if paths is None:
        if orig:
            raise JavaNullPointerException('Cannot read the array length because "<parameter1>" is null')
        raise ValueError("paths is None")
    max_len = 0
    for i, p in enumerate(paths):
        if p is None:
            if orig:
                raise JavaNullPointerException('Cannot read field "vertexList" because "<parameter1>[<local2>]" is null')
            raise ValueError(f"path {i} is None (unreachable goal)")
        max_len = max(max_len, len(p))
    if not orig and any(len(p) == 0 for p in paths) and max_len > 0:
        raise ValueError("empty path")
    out = []
    for p in paths:
        row = []
        for t in range(max_len):
            if len(p) > t:
                row.append(p[t])
            else:
                row.append(_at(row, t - 1))
        out.append(row)
    return out


def has_cycles(paths, compat: str = "fixed") -> int:
    """``hasCycles(paths)``: the number of rotations, summed over time steps.

    At each step ``i`` the map ``paths[a][i] -> paths[a][i+1]`` (later robots overwrite)
    is followed from every robot's vertex; a chain that returns to its first vertex counts
    once (two-robot swaps included). Waiting robots and chains ending at a free vertex do
    not count. Deterministic (order independent).

    ``original``: ``[]`` raises :class:`JavaArrayIndexOutOfBoundsException`, ``None`` (or
    a ``None`` row) :class:`JavaNullPointerException` (original-bugs B12), a ragged
    array may raise AIOOBE. Differs from the original: ``fixed`` returns 0 for ``[]`` and
    raises ``ValueError`` for ``None`` and ragged input.
    """
    orig = check_compat(compat)
    if paths is None:
        if orig:
            raise JavaNullPointerException('Cannot load from object array because "<parameter1>" is null')
        raise ValueError("paths is None")
    if orig:
        L = len(_at(paths, 0, "<parameter1>"))
    else:
        if not paths:
            return 0
        if any(p is None for p in paths) or len({len(p) for p in paths}) > 1:
            raise ValueError("ragged path set")
        L = len(paths[0])
    n = len(paths)
    num = 0
    for i in range(L - 1):
        nxt: dict[int, int] = {}
        for v in range(n):
            row = _at(paths, v) if orig else paths[v]
            nxt[_at(row, i) if orig else row[i]] = _at(row, i + 1) if orig else row[i + 1]
        for s in range(n):
            vid = _at(paths[s], i) if orig else paths[s][i]
            if vid in nxt:
                if nxt[vid] == vid:
                    del nxt[vid]
                    continue
            else:
                continue
            while vid in nxt:
                vid = nxt.pop(vid)
            if vid == paths[s][i]:
                num += 1
    return num


def _adjacent(graph: Graph, u: int, v: int) -> bool | None:
    nb = graph.adj.get(u)
    if nb is None:
        return None
    lo, hi = 0, len(nb)
    while lo < hi:
        mid = (lo + hi) // 2
        if nb[mid] < v:
            lo = mid + 1
        else:
            hi = mid
    return lo < len(nb) and nb[lo] == v


def _first_collision(paths) -> str | None:
    """The first meet / head-on violation in the original's order (p, q, t, meet first),
    for a rectangular path set."""
    n = len(paths)
    L = len(paths[0]) if n else 0
    best = None
    for t in range(L):
        at: dict[int, list[int]] = {}
        for a in range(n):
            at.setdefault(paths[a][t], []).append(a)
        for v, ags in at.items():
            if len(ags) > 1:
                c = (ags[0], ags[1], t, 0, v)
                if best is None or c[:4] < best[:4]:
                    best = c
        if t > 0:
            moves: dict[tuple[int, int], list[int]] = {}
            for a in range(n):
                u, w = paths[a][t - 1], paths[a][t]
                if u != w:
                    moves.setdefault((u, w), []).append(a)
            for (u, w), ags in moves.items():
                rev = moves.get((w, u))
                if rev:
                    for p in ags:
                        for q in rev:
                            c = (min(p, q), max(p, q), t, 1, None)
                            if best is None or c[:4] < best[:4]:
                                best = c
    if best is None:
        return None
    p, q, t, kind, v = best
    if kind == 0:
        return f"Path {p} and {q} meet on vertex {v} at time step {t}"
    return f"Path {p} and {q} cross the same edge at time step {t}"


def _path_set_error_literal(paths, graph: Graph, starts, goals) -> str | None:
    """``isPathSetValid`` statement by statement (original mode, irregular input)."""
    L0 = len(_at(paths, 0, "<parameter1>"))
    for t in range(L0):
        for a in range(len(paths)):
            row = _at(paths, a)
            if _at(row, 0) != _at(starts, a, "<parameter3>"):
                return f"Start location for agent {a} is incorrect in the path set."
            if _at(row, len(row) - 1) != _at(goals, a, "<parameter4>"):
                return f"Goal location for agent {a} is incorrect in the path set."
            if t > 0:
                last_v = _at(row, t - 1)
                cur_v = _at(row, t)
                ok = _adjacent(graph, last_v, cur_v)
                if ok is None:
                    raise JavaNullPointerException(
                        'Cannot invoke "java.util.Set.contains(Object)" because the return value of '
                        '"java.util.Map.get(Object)" is null')
                if not ok:
                    return f"Path {a} is not valid at t = {t - 1}"
    for p in range(len(paths) - 1):
        for q in range(p + 1, len(paths)):
            for t in range(L0):
                if _at(paths[p], t) == _at(paths[q], t):
                    return f"Path {p} and {q} meet on vertex {paths[p][t]} at time step {t}"
                if t > 0 and _at(paths[p], t) == _at(paths[q], t - 1) and _at(paths[p], t - 1) == _at(paths[q], t):
                    return f"Path {p} and {q} cross the same edge at time step {t}"
    return None


def path_set_error(paths, graph: Graph, starts: Sequence[int], goals: Sequence[int],
                   compat: str = "fixed") -> str | None:
    """The message ``isPathSetValid`` prints for an invalid path set, or ``None`` if it is
    valid. Checked in the original's order: for each ``t`` and robot ``a``, the start, the
    goal and the move from ``t-1`` (waits are allowed, the adjacency contains the vertex
    itself); then for each pair ``p < q`` and ``t``, a meet on a vertex, then a head-on
    swap. Trains (following) and rotations of three or more robots are allowed (T-RO's
    motion model). Messages: ``"Start location for agent a is incorrect in the path
    set."``, ``"Goal location for agent a is incorrect in the path set."``, ``"Path a is
    not valid at t = t-1"``, ``"Path p and q meet on vertex v at time step t"``, ``"Path p
    and q cross the same edge at time step t"``.

    ``original``: ``[]`` raises :class:`JavaArrayIndexOutOfBoundsException`, ``None``
    :class:`JavaNullPointerException`, a ragged array uses ``paths[0].length`` throughout
    (AIOOBE, or rows beyond it unchecked), a vertex without adjacency entry raises NPE;
    ``[[], []]`` is "valid".

    Differs from the original: ``fixed`` raises ``ValueError`` for ``None``, reports a
    wrong number of paths, ragged or empty rows and a move from a vertex without
    adjacency as invalid, and accepts ``[]`` for zero robots.
    """
    orig = check_compat(compat)
    if paths is None:
        if orig:
            raise JavaNullPointerException('Cannot load from object array because "<parameter1>" is null')
        raise ValueError("paths is None")
    n = len(paths)
    if not orig:
        if n != len(starts) or n != len(goals):
            return f"The path set has {n} paths for {len(starts)} robots."
        if n == 0:
            return None
        if any(p is None or len(p) == 0 for p in paths):
            return "The path set contains an empty path."
        if len({len(p) for p in paths}) > 1:
            return "The paths of the path set have different lengths."
    else:
        regular = (n > 0 and all(p is not None for p in paths) and len({len(p) for p in paths}) == 1
                   and len(paths[0]) > 0 and len(starts) >= n and len(goals) >= n
                   and all(paths[a][t] in graph.adj for a in range(n) for t in range(len(paths[0]) - 1)))
        if not regular:
            return _path_set_error_literal(paths, graph, starts, goals)
    L = len(paths[0])
    for a in range(n):
        if paths[a][0] != starts[a]:
            return f"Start location for agent {a} is incorrect in the path set."
        if paths[a][-1] != goals[a]:
            return f"Goal location for agent {a} is incorrect in the path set."
    for t in range(1, L):
        for a in range(n):
            if not _adjacent(graph, paths[a][t - 1], paths[a][t]):
                return f"Path {a} is not valid at t = {t - 1}"
    return _first_collision(paths)


def is_path_set_valid(paths, graph: Graph, starts: Sequence[int], goals: Sequence[int],
                      compat: str = "fixed", *, echo: bool = False) -> bool:
    """``isPathSetValid(paths, graph, starts, goals)``: ``True`` iff
    :func:`path_set_error` finds nothing. The original prints the message; here it is
    printed only with ``echo=True`` (use :func:`path_set_error` to get it)."""
    msg = path_set_error(paths, graph, starts, goals, compat)
    if msg is not None and echo:
        print(msg)
    return msg is None


def validate(problem, paths, compat: str = "fixed") -> str | None:
    """Check ``paths`` against ``problem`` (a :class:`~mppflow.problem.Problem` or a tuple
    ``(graph, starts, goals)``): ``None`` if the plan is valid, else the reason, as
    :func:`path_set_error` (the same checks as :func:`is_path_set_valid`)."""
    if isinstance(problem, tuple):
        graph, starts, goals = problem
    else:
        graph, starts, goals = problem.graph, problem.starts, problem.goals
    return path_set_error(paths, graph, starts, goals, compat)


def merge_paths(path_from_start, path_from_middle, compat: str = "fixed"):
    """``mergePaths(A, B)``: ``A`` followed by ``B`` without its first column (the shared
    boundary configuration), or ``None`` if either is ``None``. The makespan of the result
    is the sum of the pieces' makespans (no trimming).

    ``original``: no continuity check; widths come from ``A[0]`` and ``B[0]`` (ragged or
    shorter ``B`` raise :class:`JavaArrayIndexOutOfBoundsException`, ``A == []`` too).
    Differs from the original: ``fixed`` raises ``ValueError`` when the pieces do not join
    (``A[i][-1] != B[i][0]``), have different numbers of rows or are ragged, and merges
    two empty path sets into ``[]``.
    """
    orig = check_compat(compat)
    if path_from_start is None or path_from_middle is None:
        return None
    A, B = path_from_start, path_from_middle
    if orig:
        la = len(_at(A, 0, "<local1>"))
        lb = len(_at(B, 0, "<local2>"))
        if la + lb - 1 < 0:
            raise JavaNegativeArraySizeException(str(la + lb - 1))
        out = []
        for i in range(len(A)):
            row = [0] * (la + lb - 1)
            for j in range(la):
                row[j] = _at(A[i], j)
            for j in range(1, lb):
                row[la + j - 1] = _at(_at(B, i), j)
            out.append(row)
        return out
    if len(A) != len(B):
        raise ValueError("the pieces have different numbers of robots")
    out = []
    for i, (ra, rb) in enumerate(zip(A, B)):
        if not ra or not rb:
            raise ValueError("empty path in a piece")
        if ra[-1] != rb[0]:
            raise ValueError(f"the pieces do not join for robot {i}: {ra[-1]} != {rb[0]}")
        out.append(list(ra) + list(rb[1:]))
    if len({len(r) for r in out}) > 1:
        raise ValueError("ragged pieces")
    return out


# -------------------------------------------------------------- split windows
def _ecc(g: Graph, c: int) -> int:
    """Largest BFS layer reachable from ``c`` (self-loops ignored), as seen by
    ``findGoalsInDistanceRange``."""
    dist = {c: 0}
    q = deque([c])
    m = 0
    while q:
        v = q.popleft()
        for u in g.neighbors(v) or ():
            if u != v and u not in dist:
                dist[u] = dist[v] + 1
                m = max(m, dist[u])
                q.append(u)
    return m


def _hops(nbrs, c: int) -> dict[int, int]:
    """BFS hop distances from ``c`` over ``nbrs(v)`` (self-loops ignored)."""
    dist = {c: 0}
    q = deque([c])
    while q:
        v = q.popleft()
        for u in nbrs(v) or ():
            if u not in dist:
                dist[u] = dist[v] + 1
                q.append(u)
    return dist


def _hops_to(g: Graph, c: int) -> dict[int, int]:
    """``fixed``: ``dist(v, c)`` for every ``v`` that reaches ``c`` along the (possibly
    one-way) arcs (BFS over the reversed arcs; on a symmetric graph the forward BFS)."""
    rev = _reversed_adjacency(g)
    if rev is None:
        return _hops(g.neighbors, c)
    return _hops(rev.get, c)


class _SplitWindows:
    """The candidate lists ``splitPaths`` examines for one robot, computed lazily.

    The original's loop computes, for the window ``[dMin, dMax]`` (start ``d = L/2`` for
    ``L`` path vertices), the vertices at those BFS distances from both the start and the
    goal (``findGoalsInDistanceRange``, which never contains the centre), in the
    ``HashSet`` order of their intersection, takes the first one no earlier robot has
    taken, and otherwise widens the window (``dMin--`` floored at 0, then ``dMax++``,
    alternately). Identical windows are stored once; once ``dMin <= 1`` and ``dMax`` is at
    least the eccentricity of both endpoints the list can no longer change
    (``saturated``).

    With ``endpoints`` (the ``fixed`` rules) three things change:

    * each ring also holds its centre at distance 0, so the robot's start and goal are
      candidates as soon as the window reaches them (``dMin == 0`` and ``dMax >=
      dist(start, goal)``); a robot at its goal (``L == 1``) then gets its start first, an
      adjacent one (``L == 2``) its start or goal;
    * the goal side is measured to the goal, ``dist(v, goal)`` (over the reversed arcs of a
      one-way graph), not from it: the original's ``dist(goal, v)`` can offer a vertex from
      which the goal cannot be reached;
    * for an odd distance ``D = L - 1`` the first list holds the middle vertices of the
      shortest paths, ``dist(start, v) in {floor(D/2), ceil(D/2)}`` and
      ``dist(start, v) + dist(v, goal) = D`` (T-RO Sec. V-B: "the middle node is set as the
      intermediate goal"); the original's first window ``d = L/2 = ceil(D/2)`` from both
      ends holds only vertices one step off every shortest path (none on a bipartite
      graph such as a 4-connected grid, where the next window gives this same list, so
      the choice is unchanged there).

    Otherwise the windows are the original's."""

    __slots__ = ("g", "s", "goal", "d_min", "d_max", "flipped", "lists", "done", "ecc", "_last", "endpoints",
                 "_to_goal")

    def __init__(self, g: Graph, s: int, goal: int, L: int, endpoints: bool = False):
        self.g, self.s, self.goal = g, s, goal
        self.d_min = self.d_max = L // 2
        self.flipped = True
        self.lists: list[list[int]] = []
        self.done = False
        self._last = None
        self.endpoints = endpoints
        if endpoints:
            self._to_goal = _hops_to(g, goal)
            self.ecc = max(_ecc(g, s), max(self._to_goal.values()))
            D = L - 1
            if D % 2 == 1:
                ds = _hops(g.neighbors, s)
                to = self._to_goal
                mid = [v for v in _ring(g, s, D // 2, D // 2 + 1) if v in to and ds.get(v, -1) + to[v] == D]
                self.lists.append(JavaIntHashSet(mid).to_list())
        else:
            self._to_goal = None
            self.ecc = max(_ecc(g, s), _ecc(g, goal))

    def _next(self) -> None:
        lo = 0 if self.endpoints and self.d_min <= 0 else max(self.d_min, 1)
        key = (lo, self.d_max)
        if key != self._last:
            if self.endpoints:
                S = _ring(self.g, self.s, self.d_min, self.d_max)
                d_lo, d_hi = self.d_min, self.d_max
                G = {v for v, d in self._to_goal.items() if d_lo <= d <= d_hi}
            else:
                S = find_goals_in_distance_range(self.g, self.s, self.d_min, self.d_max)
                G = set(find_goals_in_distance_range(self.g, self.goal, self.d_min, self.d_max))
            self.lists.append(JavaIntHashSet(v for v in S if v in G).to_list())
            self._last = key
        if key[0] == (0 if self.endpoints else 1) and self.d_max >= self.ecc:
            self.done = True
        if self.flipped:
            self.d_min = max(self.d_min - 1, 0)
        else:
            self.d_max += 1
        self.flipped = not self.flipped

    def choose(self, used) -> int | None:
        """The vertex the original picks given the used set, or ``None`` if it would
        widen the window forever."""
        j = 0
        while True:
            while j >= len(self.lists):
                if self.done:
                    return None
                self._next()
            for v in self.lists[j]:
                if v not in used:
                    return v
            j += 1


def _reaches(g: Graph, src: int, dst: int) -> bool:
    seen = {src}
    q = deque([src])
    while q:
        v = q.popleft()
        if v == dst:
            return True
        for u in g.neighbors(v) or ():
            if u not in seen:
                seen.add(u)
                q.append(u)
    return False


class _SplitStuck(Infeasible):
    """``fixed``: no free vertex can serve as a robot's intermediate goal. A failure of the
    split heuristic, not a proof that the instance has no solution."""


def _split_fallback(g: Graph, s: int, goal: int, used, agent: int) -> int:
    """``fixed``: an intermediate goal when every vertex of the window is taken."""
    for v in (s, goal):
        if v not in used:
            return v
    seen = {s}
    q = deque([s])
    while q:
        v = q.popleft()
        if v not in used and _reaches(g, v, goal):
            return v
        for u in g.neighbors(v) or ():
            if u not in seen:
                seen.add(u)
                q.append(u)
    raise _SplitStuck(f"no free vertex can serve as the intermediate goal of robot {agent}")


def _ring(g: Graph, c: int, lo: int, hi: int) -> list[int]:
    """``fixed`` ``splitPathBySize`` ring: BFS distances in ``[lo, hi]`` from ``c``,
    the centre included at distance 0."""
    if hi < 0 or lo > hi:
        return []
    out = [c] if lo <= 0 else []
    if hi >= 1:
        out += find_goals_in_distance_range(g, c, lo, hi)
    return out


def _bfs_dists(g: Graph, s: int) -> list[int]:
    return get_distances_to_all_vertices(g, s, None, "fixed", unreachable=-1)


# ------------------------------------------------------------------- the planner
@dataclass
class _LeafEntry:
    paths: list | None
    exc: BaseException | None
    log: str
    err: str
    solves: list
    cyclic: bool | None = None
    """``has_cycles(paths) > 0``, filled in by the retry analysis."""


class _Unknown(Exception):
    """The retry analysis exceeded its budget."""


_NPE = ("X", "JavaNullPointerException")
_NULL = ("N",)
_ANALYSIS_STEPS = 200000


class _OSet(dict):
    """An insertion-ordered set (the keys of a dict). The retry analysis collects its
    outcomes (tuples of strings) in these instead of ``set``s: a ``set`` of strings
    iterates in an order that depends on ``PYTHONHASHSEED``, and the analysis's leaf
    solves, its budget installments and hence the retry at which it decides follow the
    iteration order."""

    __slots__ = ()

    @classmethod
    def of(cls, items) -> _OSet:
        return cls.fromkeys(items)

    def add(self, x) -> None:
        self[x] = None

    def __ior__(self, items):
        for x in items:
            self[x] = None
        return self


def _uniq(items) -> tuple:
    """The distinct ``items`` in first-seen order (a frozen :class:`_OSet`)."""
    return tuple(dict.fromkeys(items))
MAX_SPLIT = 1024
"""Largest ``split`` that :func:`plan` (and ``mppflow solve --split``) accepts. Time and memory
of the k-way split grow linearly in k (k pieces, k + 1 configurations of all robots), and
pieces beyond the makespan are only waits, so a larger k is a mistake (``1 << 40`` would
allocate 2**40 configurations); it raises ``ValueError`` at once."""
DEFAULT_MAX_RETRY_SOLVES = 1000
"""``fixed``: bound on the solves of a top-level no-cycle retry loop (its retries' leaf
solves plus those of its endless-loop analysis) beyond the original's bounded tries; the
original retries forever."""


class _TopRun:
    """``fixed``: the work and time of one top-level no-cycle retry loop."""

    def __init__(self, pl: PathPlanner, tl: float):
        self.pl, self.tl = pl, tl
        self.t0 = pl._now()
        self.solves0 = len(pl.solves)
        self.analysis = 0

    def start(self) -> None:
        """The retries start here (the first split's solves do not count)."""
        self.solves0 = len(self.pl.solves)

    def work(self) -> int:
        return len(self.pl.solves) - self.solves0 + self.analysis


class PathPlanner:
    """A port of ``PathPlanner`` (one instance = one Java planner object).

    Args:
        backend, params: the LP/MILP backend (``"highs"``, ``"gurobi"``, ``"builtin"``) and
            its parameters (``{"Threads": 1}`` to reproduce the golden Gurobi runs).
        rng: the ``Math.random()`` stream (``None``: the module stream).
        compat: ``"fixed"`` or ``"original"`` (see the module docstring).
        flags: the solvers' static flags (:class:`SolverFlags`, default all quiet).
        echo: also print what is collected in :attr:`log`.
        clock: wall clock in seconds (default ``time.time``), for the time limits.
        feasibility_budget: passed to the solvers' infeasibility detection.
        retry_analysis_after, analysis_budget: from which top-level retry on, and with how
            many new leaf solves per retry, the top-level retry loops of the no-cycle splits
            are analysed for an endless loop (see the module docstring and
            :class:`_RetryWatch`).
        max_retry_solves: ``fixed``: the most solves (retries' leaf solves plus the
            analysis's) a top-level no-cycle retry loop may spend beyond the original's
            bounded tries (the original has no bound); ``None``: no bound.
    """

    def __init__(self, *, backend: str = "highs", params: dict | None = None, rng: MathRandom | None = None,
                 compat: str = "fixed", flags: SolverFlags | None = None, echo: bool = False,
                 clock: Callable[[], float] | None = None,
                 feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET, retry_analysis_after: int = 20,
                 analysis_budget: int = 200, max_retry_solves: int | None = DEFAULT_MAX_RETRY_SOLVES):
        self.orig = check_compat(compat)
        self.compat = compat
        self.backend = backend
        self.params = params
        self.rng = resolve_rng(rng)
        self.flags = flags if flags is not None else SolverFlags()
        self.echo = echo
        self.clock = clock or time.time
        self.feasibility_budget = feasibility_budget
        self.retry_analysis_after = retry_analysis_after
        self.analysis_budget = analysis_budget
        self.max_retry_solves = max_retry_solves
        self.solves: list[SolveResult] = []
        """Every solver result, in call order."""
        self.failures: list[str] = []
        """``fixed``: why split pieces failed."""
        self.time_limit_hit = False
        """``fixed``: a piece of :meth:`plan_paths_advanced_split_dist` failed because its time
        limit ran out between two solves (no solve reports ``"time_limit"`` then)."""
        # multiThreadedSplitPlanning's instance fields
        self.mpieces = 0
        self.msg: list[list[int]] | None = None
        self.mdone: list[bool] | None = None
        self.mg: Graph | None = None
        self.mpath: dict[int, list[list[int]]] = {}
        self.mtime = 0.0
        self.dist_mids: list[list[int]] | None = None
        """:meth:`plan_paths_advanced_split_dist`: the intermediate configurations of its last
        call, in time order (the k - 1 configurations between the pieces of a complete plan)."""
        self._log: list[str] = []
        self._err: list[str] = []
        self._capture: tuple[list, list, list] | None = None
        self._depth = 0
        self._wcache: dict | None = None
        self._cache: dict | None = None
        # fixed: a robot's start and goal may be its intermediate goal (False: the original's rule)
        self._split_endpoints = not self.orig

    # ------------------------------------------------------------ output
    @property
    def log(self) -> str:
        """Everything the original would have printed to ``System.out``."""
        return "".join(self._log)

    @property
    def stderr(self) -> str:
        """Stack traces the original prints to ``System.err``."""
        return "".join(self._err)

    def clear_log(self) -> None:
        self._log.clear()
        self._err.clear()

    def _emit(self, s: str) -> None:
        if not s:
            return
        if self._capture is not None:
            self._capture[0].append(s)
            return
        self._log.append(s)
        if self.echo:
            sys.stdout.write(s)

    def _emit_err(self, s: str) -> None:
        if not s:
            return
        if self._capture is not None:
            self._capture[1].append(s)
            return
        self._err.append(s)
        if self.echo:
            sys.stderr.write(s)

    def _record(self, r: SolveResult) -> None:
        self._emit(r.log)
        if self._capture is not None:
            self._capture[2].append(r)
        else:
            self.solves.append(r)

    def _now(self) -> float:
        t = self.clock()
        return int(t * 1000) / 1000.0 if self.orig else t

    @contextmanager
    def _scope(self, leaf_cache: bool = False):
        outer = self._depth == 0
        if outer:
            self._wcache = {}
            self._cache = {} if leaf_cache else None
        self._depth += 1
        try:
            yield
        finally:
            self._depth -= 1
            if outer:
                self._wcache = None
                self._cache = None

    def _windows(self, g: Graph, s: int, goal: int, L: int) -> _SplitWindows:
        ep = self._split_endpoints
        if self._wcache is None:
            return _SplitWindows(g, s, goal, L, ep)
        # The entry keeps its graph and is used only for that same object: an ``id()``
        # can be reused by a new graph once the old one is freed.
        key = (id(g), s, goal, L)
        ent = self._wcache.get(key)
        if ent is None or ent[0] is not g:
            ent = self._wcache[key] = (g, _SplitWindows(g, s, goal, L, ep))
        return ent[1]

    # ------------------------------------------------------------ solver wrappers
    def _check_result(self, paths, graph, start, goal):
        if paths is not None:
            err = path_set_error(paths, graph, start, goal, self.compat)
            if err is not None:
                self._emit(err + "\n")
                return None
        elif self.flags.time_debug:
            self._emit("Maximum allowed time reached, optimization stopped.\n")
        return paths

    def _grb(self, e: JavaGRBException) -> None:
        self._emit_err(f"{e.java_class}: {e}\n")

    def _makespan(self, kind, graph, start, goal, solve_relaxed, extra, set_optimal, has, tl, *, worker=False):
        f = solve_time if kind == "time" else solve_time_r
        if kind == "time":
            dbg, pp = self.flags.time_debug, self.flags.time_print_paths
        else:
            dbg, pp = self.flags.time_r_debug, self.flags.time_r_print_paths
        try:
            r = f(graph, list(start), list(goal), solve_relaxed, set_optimal, extra, has, tl, rng=self.rng,
                  backend=self.backend, params=self.params, compat=self.compat, debug=dbg, print_paths=pp,
                  clock=self.clock, feasibility_budget=self.feasibility_budget)
        except JavaGRBException as e:
            if not self.orig or worker:
                raise
            self._grb(e)
            return None
        self._record(r)
        return self._check_result(r.paths, graph, start, goal)

    def plan_paths_advanced(self, graph: Graph, start: Sequence[int], goal: Sequence[int],
                            solve_relaxed: bool = False, extra_steps: int = 0, set_optimal: bool = True,
                            time_limit: float = -1.0):
        """``planPathsAdvanced``: MINMAKESPAN with :func:`~mppflow.solvers.solve_time`
        (``hasTimeLimit = time_limit > 0``). Returns ``paths`` or ``None`` (time out, or a
        path set that fails :func:`path_set_error`, whose message is logged). With
        ``time_debug`` a ``None`` result logs "Maximum allowed time reached, optimization
        stopped.". ``original`` catches the solver's ``GRBException`` (logged to
        :attr:`stderr`) and returns ``None``; other exceptions propagate.
        """
        return self._makespan("time", graph, start, goal, solve_relaxed, extra_steps, set_optimal,
                              time_limit > 0, float(time_limit))

    def plan_paths_advanced_pmg(self, graph: Graph, start: Sequence[int], goal: Sequence[int],
                                solve_relaxed: bool = False, extra_steps: int = 0, set_optimal: bool = True,
                                time_limit: float = -1.0):
        """``planPathsAdvancedPMG`` (*dead*): as :meth:`plan_paths_advanced` with the
        one-move-per-step solver :func:`~mppflow.solvers.solve_time_r` (its own
        ``time_r_*`` flags; the "Maximum allowed time" message uses ``time_debug``, as the
        original)."""
        return self._makespan("time_r", graph, start, goal, solve_relaxed, extra_steps, set_optimal,
                              time_limit > 0, float(time_limit))

    def plan_paths_advanced_tt(self, graph: Graph, start: Sequence[int], goal: Sequence[int], opt: list | None = None,
                               gap: float = 0.0, time_steps: int | None = None, time_limit: float = -1.0, *,
                               horizon: str | None = None):
        """``planPathsAdvancedTT``: MINTOTALTIME (:func:`~mppflow.solvers.solve_total_time`),
        ``opt[0]`` receives the total time. The original solves one horizon
        ``time_steps``; ``fixed`` defaults to the horizon search (``horizon="single"`` for
        the original's behaviour). Result handling as :meth:`plan_paths_advanced`."""
        try:
            r = solve_total_time(graph, list(start), list(goal), opt, gap, time_steps, time_limit > 0,
                                 float(time_limit), return_any_solution=self.flags.tt_return_any_solution,
                                 horizon=horizon, rng=self.rng, backend=self.backend, params=self.params,
                                 compat=self.compat, debug=self.flags.tt_debug, print_paths=self.flags.tt_print_paths,
                                 clock=self.clock, feasibility_budget=self.feasibility_budget)
        except JavaGRBException as e:
            if not self.orig:
                raise
            self._grb(e)
            return None
        self._record(r)
        return self._check_result(r.paths, graph, start, goal)

    def plan_paths_advanced_td(self, graph: Graph, start: Sequence[int], goal: Sequence[int], gap: float = 0.0,
                               time_steps: int | None = None, time_limit: float = -1.0, *,
                               horizon: str | None = None):
        """``planPathsAdvancedTD``: MINTOTALDIST (:func:`~mppflow.solvers.solve_distance`)
        at horizon ``time_steps`` (``fixed`` default: the horizon search). Result handling
        as :meth:`plan_paths_advanced`."""
        try:
            r = solve_distance(graph, list(start), list(goal), gap, time_steps, float(time_limit),
                               return_any_solution=self.flags.dist_return_any_solution, horizon=horizon,
                               backend=self.backend, params=self.params, compat=self.compat,
                               debug=self.flags.dist_debug, print_paths=self.flags.dist_print_paths,
                               clock=self.clock, feasibility_budget=self.feasibility_budget)
        except JavaGRBException as e:
            if not self.orig:
                raise
            self._grb(e)
            return None
        self._record(r)
        return self._check_result(r.paths, graph, start, goal)

    # ------------------------------------------------------------ leaves
    def _run_leaf_captured(self, g, s, goal, set_optimal, tl, rng) -> _LeafEntry:
        saved_rng, saved_cap = self.rng, self._capture
        cap: tuple[list, list, list] = ([], [], [])
        self.rng, self._capture = rng, cap
        paths, exc = None, None
        try:
            paths = self._makespan("time", g, s, goal, False, 0, set_optimal, tl > 0, tl)
        except (JavaException, OriginalWouldHang, Infeasible) as e:
            exc = e
        finally:
            self.rng, self._capture = saved_rng, saved_cap
        return _LeafEntry(paths, exc, "".join(cap[0]), "".join(cap[1]), cap[2])

    def _leaf(self, g, s, goal, set_optimal, tl, inner):
        """A split leaf: ``planPathsAdvanced(g, s, goal, false, 0, setOptimal, tl)``."""
        if self._cache is not None:
            key = (tuple(s), tuple(goal), set_optimal, tl)
            ent = self._cache.get(key)
            if ent is None:
                ent = self._cache[key] = self._run_leaf_captured(g, s, goal, set_optimal, tl, self.rng)
            else:
                # the solve's only random draws (they break ties; the result is cached)
                find_shortest_paths(g, list(s), list(goal), self.rng, heuristic=None, compat="original")
            self._emit(ent.log)
            self._emit_err(ent.err)
            self.solves.extend(ent.solves)
            if ent.exc is not None:
                raise ent.exc
            return None if ent.paths is None else [list(r) for r in ent.paths]
        if self.orig or not inner:
            return self.plan_paths_advanced(g, s, goal, False, 0, set_optimal, tl)
        try:
            return self.plan_paths_advanced(g, s, goal, False, 0, set_optimal, tl)
        except Infeasible as e:
            self.failures.append(f"piece {list(s)} -> {list(goal)}: {e}")
            return None

    def _concat(self, A, B):
        return merge_paths(A, B, self.compat)

    # ------------------------------------------------------------ split goals
    def _lengths(self, g, start, goal) -> list[int]:
        paths = find_shortest_paths(g, list(start), list(goal), self.rng, heuristic=None if self.orig else False,
                                    compat=self.compat)
        L = []
        for i, p in enumerate(paths):
            if p is None:
                if self.orig:
                    raise JavaNullPointerException(
                        'Cannot read field "vertexList" because "<local5>[<local7>]" is null')
                raise Infeasible(f"goal of robot {i} is unreachable from its start")
            L.append(len(p))
        return L

    def _order(self, L: list[int], random: bool) -> list[int]:
        n = len(L)
        order = sorted(range(n), key=lambda i: -L[i])  # TreeMap over -size: longest first, ties by index
        if random:
            vec = list(range(n))
            order = [vec.pop(int(len(vec) * self.rng.random())) for _ in range(n)]
        return order

    def _assign(self, g, start, goal, L, order) -> list[int]:
        middle = [0] * len(start)
        used: set[int] = set()
        for ag in order:
            v = self._windows(g, start[ag], goal[ag], L[ag]).choose(used)
            if v is None:
                if self.orig:
                    raise OriginalWouldHang(
                        f"splitPaths: every vertex at any distance from robot {ag}'s start and goal is already "
                        f"another robot's intermediate goal; the original widens its window forever")
                v = _split_fallback(g, start[ag], goal[ag], used, ag)
            middle[ag] = v
            used.add(v)
        return middle

    def split_paths(self, g: Graph, start: Sequence[int], goal: Sequence[int], random: bool = False) -> list[int]:
        """``splitPaths(g, start, goal, random)``: one intermediate goal per robot (T-RO
        Sec. V-B, Fig. 8).

        Robots are processed longest shortest path first (ties by index), or in a uniform
        random order when ``random`` (``n`` draws, ``vec.remove((int)(size * R))``, after the
        one :func:`~mppflow.pathfinder.find_shortest_paths` call whose path lengths ``L``
        are used). Each robot takes the first vertex, in Java ``HashSet`` order, at BFS
        distance ``L/2`` from both its start and its goal that no earlier robot took,
        widening the window as described in :class:`_SplitWindows`. The paper's Fig. 8
        instance gives ``[5, 10]``.

        ``original``: the endpoints are never chosen (a stationary or adjacent robot gets a
        detour); raises :class:`OriginalWouldHang` where the original widens the window
        forever, :class:`JavaNullPointerException` for an unreachable goal.
        Differs from the original: in ``fixed`` the robot's start and goal are candidates
        once the window reaches them (so ``[4] -> [4]`` gives ``[4]``, not ``[1]``, on the
        3x3 grid), the goal side is ``dist(v, goal)``, an odd distance first offers the
        middle vertices of the shortest paths (see :class:`_SplitWindows`), a saturated
        window falls back to the start (else the goal, else any free vertex that reaches
        the goal), and an unreachable goal raises :class:`Infeasible`.
        """
        with self._scope():
            start, goal = list(start), list(goal)
            L = self._lengths(g, start, goal)
            return self._assign(g, start, goal, L, self._order(L, random))

    def get_longest_path_length(self, g: Graph, start: Sequence[int], goal: Sequence[int]) -> int:
        """``getLongestPathLength`` (used by the dead split-by-size code): one
        :func:`~mppflow.pathfinder.find_shortest_paths` call (draws).

        ``original``: ``length = 0``; for each robot, if ``size - 1 > length`` then
        ``length = size`` (``size`` = vertices on its path), so a path one edge longer than
        the stored value is skipped and the result is the largest vertex count or one less
        (0 when every robot is at its goal). Differs from the original: ``fixed`` returns
        the largest vertex count.
        """
        L = self._lengths(g, start, goal)
        if self.orig:
            length = 0
            for s in L:
                if s - 1 > length:
                    length = s
            return length
        return max(L) if L else 0

    def split_path_by_size(self, g: Graph, start: Sequence[int], goal: Sequence[int], random: bool = False,
                           first_size: int = 2) -> list[int]:
        """``splitPathBySize(g, start, goal, random, firstSize)`` (*dead*): an intermediate
        goal about ``first_size - 1`` steps from each start. Order and draws as
        :meth:`split_paths`.

        ``original``: start window ``[dMin, dMax] = [firstSize-1, firstSize-1]``, goal
        window ``[L - dMax, L - dMin]`` (``L`` = vertex count: off by one); on failure only
        ``dMin`` drops (to 0), so the loop can spin forever: :class:`OriginalWouldHang`.
        Differs from the original: ``fixed`` uses ``[D - dMax, D - dMin]`` with ``D = L - 1``
        edges, ``k = min(firstSize - 1, D)``, admits the endpoints at distance 0, widens
        both sides alternately and falls back as :meth:`split_paths` does.
        """
        with self._scope():
            start, goal = list(start), list(goal)
            L = self._lengths(g, start, goal)
            order = self._order(L, random)
            return self._assign_by_size(g, start, goal, L, order, first_size)

    def _assign_by_size(self, g, start, goal, L, order, first_size) -> list[int]:
        middle = [0] * len(start)
        used: set[int] = set()
        for ag in order:
            s, t, Li = start[ag], goal[ag], L[ag]
            if self.orig:
                d_min = d_max = first_size - 1
                d_min_goal = Li - d_max
                d_max_goal = Li - d_min
                while True:
                    S = find_goals_in_distance_range(g, s, d_min, d_max)
                    G = set(find_goals_in_distance_range(g, t, d_min_goal, d_max_goal))
                    v = next((v for v in JavaIntHashSet(v for v in S if v in G) if v not in used), None)
                    if v is not None:
                        break
                    if d_min == 0:
                        raise OriginalWouldHang(
                            f"splitPathBySize: no free vertex for robot {ag} and the window no longer changes "
                            "(dMax never grows); the original loops forever")
                    d_min = max(d_min - 1, 0)
                    d_max_goal = Li - d_min
            else:
                D = Li - 1
                k = min(max(first_size - 1, 0), D)
                d_min = d_max = k
                flipped = True
                to_goal = _hops_to(g, t)  # dist(v, t): over the reversed arcs of a one-way graph
                ecc = max(_ecc(g, s), max(to_goal.values()), D)
                while True:
                    S = _ring(g, s, d_min, d_max)
                    G = {v for v, d in to_goal.items() if D - d_max <= d <= D - d_min}
                    v = next((v for v in JavaIntHashSet(v for v in S if v in G) if v not in used), None)
                    if v is not None:
                        break
                    if d_min == 0 and d_max >= ecc:
                        v = _split_fallback(g, s, t, used, ag)
                        break
                    if flipped:
                        d_min = max(d_min - 1, 0)
                    else:
                        d_max += 1
                    flipped = not flipped
            middle[ag] = v
            used.add(v)
        return middle

    # ------------------------------------------------------------ recursive split
    def plan_paths_advanced_split(self, g: Graph, start: Sequence[int], goal: Sequence[int],
                                  set_optimal: bool = True, time_limit: float = -1.0, splits: int = 1):
        """``planPathsAdvancedSplit``: ``2^splits`` pieces, solved depth first.

        With ``splits > 0``: ``middle = split_paths(start, goal)``, then both halves
        recursively with ``time_limit / 2`` each (the second half is solved even if the
        first failed), concatenated with :func:`merge_paths`. With ``splits == 0``: the
        leaf ``planPathsAdvanced(g, start, goal, false, 0, setOptimal, time_limit)`` (no LP
        stage). ``Main`` uses ``(false, -1, 2)`` (a 4-way split) to get the horizon of
        MINTOTALTIME / MINTOTALDIST.

        Differs from the original: as :meth:`split_paths`; ``fixed`` returns ``None`` when
        a piece is infeasible (the original loops forever there).
        """
        with self._scope():
            return self._split(g, list(start), list(goal), set_optimal, float(time_limit), splits, False)

    def _middle(self, g, s, goal, inner):
        """``split_paths`` for the split variants. ``fixed``: a failure of the heuristic (no
        free vertex for an intermediate goal, or, in an inner piece, a goal its robot cannot
        reach) is recorded in :attr:`failures` and gives ``None`` (status ``"failed"``), not
        :class:`Infeasible`; a top-level goal unreachable from its start still raises."""
        if self.orig:
            return self.split_paths(g, s, goal, False)
        try:
            return self.split_paths(g, s, goal, False)
        except Infeasible as e:
            if not inner and not isinstance(e, _SplitStuck):
                raise
            self.failures.append(f"split {list(s)} -> {list(goal)}: {e}")
            return None

    def _split(self, g, s, goal, so, tl, splits, inner):
        if splits > 0:
            middle = self._middle(g, s, goal, inner)
            if middle is None:
                return None
            A = self._split(g, s, middle, so, tl / 2, splits - 1, True)
            B = self._split(g, middle, goal, so, tl / 2, splits - 1, True)
            if A is not None and B is not None:
                return self._concat(A, B)
            return None
        return self._leaf(g, s, goal, so, tl, inner)

    # ------------------------------------------------------------ split + distance
    def plan_paths_advanced_split_dist(self, g: Graph, start: Sequence[int], goal: Sequence[int], gap: float = 0.0,
                                       time_limit: float = -1.0, splits: int = 1, *, leaf_horizon: str = "single"):
        """``planPathsAdvancedSplitDist``: the time split with MINTOTALDIST pieces (TOR
        Sec. V-B remark).

        ``splits > 0``: ``middle = split_paths``; the first half gets ``time_limit`` (half
        of it when ``dist_return_any_solution``), the elapsed time is subtracted, and (in
        ``original``) the second half is solved only ``if time_limit > 0`` with what is left. A leaf solves
        MINMAKESPAN (``setOptimal = false``) and then MINTOTALDIST at that makespan (the
        same limit).

        ``original``: with ``time_limit <= 0`` and ``splits > 0`` it always returns
        ``None`` (after solving the first half); a failed leaf makespan raises
        :class:`JavaNullPointerException`.

        Differs from the original: ``fixed`` treats ``time_limit <= 0`` as no limit,
        returns ``None`` for a failed piece, gives every piece the whole ``time_limit`` (the
        original gives the second half what the first left over) and each piece's distance
        solve the time left after its makespan solve (when none is left, the piece fails and
        :attr:`time_limit_hit` is set, so that :func:`plan` reports ``"time_limit"``);
        ``leaf_horizon`` (``"single"``, the original's choice, or ``"search"``) selects the
        distance solve's horizon.
        """
        self.dist_mids = []
        with self._scope():
            return self._split_dist(g, list(start), list(goal), gap, float(time_limit), splits, False, leaf_horizon)

    def _split_dist(self, g, s, goal, gap, tl, splits, inner, hz):
        if splits > 0:
            middle = self._middle(g, s, goal, inner)
            if middle is None:
                return None
            t0 = self._now()
            first = tl / 2 if self.flags.dist_return_any_solution else tl
            A = self._split_dist(g, s, middle, gap, first, splits - 1, True, hz)
            if self.dist_mids is not None:  # after the first half's own middles: time order
                self.dist_mids.append(list(middle))
            el = self._now() - t0
            if self.orig:
                tl -= el
                if tl > 0:
                    t0 = self._now()
                    B = self._split_dist(g, middle, goal, gap, tl, splits - 1, True, hz)
                    tl -= self._now() - t0
                    if A is not None and B is not None:
                        return self._concat(A, B)
                return None
            # fixed: each piece gets the whole limit, as in the makespan split (plan(split=k)
            # documents a per-piece limit; the pieces run one after another)
            B = self._split_dist(g, middle, goal, gap, tl, splits - 1, True, hz)
            if A is not None and B is not None:
                return self._concat(A, B)
            return None
        t0 = self._now()
        tp = self._leaf(g, s, goal, False, tl, inner)
        if tp is None:
            if self.orig:
                raise JavaNullPointerException('Cannot load from object array because "<local9>" is null')
            return None
        if self.orig:
            return self.plan_paths_advanced_td(g, s, goal, gap, len(_at(tp, 0, "<local9>")) - 1, tl)
        rem = tl
        if tl > 0:
            rem = tl - (self._now() - t0)
            if rem <= 0:
                self.failures.append("time limit reached after a piece's makespan solve")
                self.time_limit_hit = True
                return None
        T = len(tp[0]) - 1 if tp else 0
        try:
            return self.plan_paths_advanced_td(g, s, goal, gap, T, rem, horizon=hz)
        except Infeasible as e:
            if not inner:
                raise
            self.failures.append(f"piece {s} -> {goal}: {e}")
            return None

    # ------------------------------------------------------------ split, no cycles
    def plan_paths_advanced_split_no_cycle(self, g: Graph, start: Sequence[int], goal: Sequence[int],
                                           set_optimal: bool = True, time_limit: float = -1.0, splits: int = 1,
                                           top: bool = True):
        """``planPathsAdvancedSplitNoCycle``: :meth:`plan_paths_advanced_split` that
        re-plans pieces whose paths contain rotations (:func:`has_cycles`), up to
        ``tries = (int) 3^(4 - splits)`` times (27, 9, 3, 1, then 0). At ``splits == 1``
        both halves are re-planned with a random re-split (:meth:`split_paths` with
        ``random=True``); above, the same sub-instance is re-solved (first the left half,
        then the right, sharing ``tries``). Each retry gets ``time_limit / 2`` again. With
        ``time_print_paths`` set and ``top``, the flag is turned off during the call and the
        final paths are logged once (:func:`print_paths`).

        ``original``: at top level the retries never stop (``tries-- > 0 || top``):
        :class:`OriginalWouldHang` when no retry can succeed (see the module docstring); a
        retried piece that is ``None`` raises :class:`JavaNullPointerException` in the next
        ``hasCycles``; ``bPrintPaths`` stays ``false`` after ``splits <= 0`` and after a
        failed first split. Differs from the original: ``fixed`` also retries without the
        bound at top level, but stops (returning the plan with rotations, reason in
        :attr:`failures`) when the analysis shows that no retry can give a plan without
        rotations, once the retries and the analysis have spent ``max_retry_solves``
        solves, or once a positive ``time_limit`` has elapsed; see the module docstring
        for the rest.
        """
        with self._scope(leaf_cache=self.orig):
            s, goal = list(start), list(goal)
            if self.orig:
                return self._nc_orig(g, s, goal, set_optimal, float(time_limit), splits, top)
            return self._nc_fixed(g, s, goal, set_optimal, float(time_limit), splits, top)

    def _nc_orig(self, g, s, goal, so, tl, splits, top):
        F = self.flags
        pp = False
        if F.time_print_paths and top:
            F.time_print_paths = False
            pp = True
        tries = _jpow_int(3.0, 4 - splits)
        if splits > 0:
            middle = self.split_paths(g, s, goal, False)
            A = self._nc_orig(g, s, middle, so, tl / 2, splits - 1, False)
            B = self._nc_orig(g, middle, goal, so, tl / 2, splits - 1, False)
            if A is None or B is None:
                return None
            if splits == 1:
                it = 0
                watch = None
                while has_cycles(A, "original") > 0 or has_cycles(B, "original") > 0:
                    ok = tries > 0
                    tries -= 1
                    if not (ok or top):
                        break
                    it += 1
                    if top and it >= self.retry_analysis_after:
                        if watch is None:
                            watch = self._retry_watch("nc1", g, s, goal, so, tl)
                        self._retry_check(watch)
                    middle = self.split_paths(g, s, goal, True)
                    A = self._nc_orig(g, s, middle, so, tl / 2, 0, False)
                    B = self._nc_orig(g, middle, goal, so, tl / 2, 0, False)
            else:
                for side in ("A", "B"):
                    it = 0
                    watch = None
                    while has_cycles(A if side == "A" else B, "original") > 0:
                        ok = tries > 0
                        tries -= 1
                        if not (ok or top):
                            break
                        it += 1
                        if top and it >= self.retry_analysis_after:
                            if watch is None:
                                ends = (s, middle) if side == "A" else (middle, goal)
                                watch = self._retry_watch("ncK", g, ends[0], ends[1], so, tl / 2, k=splits - 1)
                            self._retry_check(watch)
                        if side == "A":
                            A = self._nc_orig(g, s, middle, so, tl / 2, splits - 1, False)
                        else:
                            B = self._nc_orig(g, middle, goal, so, tl / 2, splits - 1, False)
            if A is not None and B is not None:
                full = self._concat(A, B)
                if pp and top:
                    self._emit(print_paths(g, full, "original"))
                    F.time_print_paths = True
                return full
        else:
            return self._leaf(g, s, goal, so, tl, True)
        if pp and top:
            F.time_print_paths = True
        return None

    def _top_retry_stop(self, it, ok, run: _TopRun, watch: _RetryWatch | None) -> bool:
        """``fixed``: whether the top-level retry ``it`` (1-based) should not run.

        From retry ``retry_analysis_after`` on, or from the first retry beyond the
        original's bounded tries if that comes first, each retry advances the endless-loop
        analysis (``watch``, :class:`_RetryWatch`); once it shows that no retry can give a
        plan without rotations, the retries stop (the original would retry forever).
        Otherwise retries within the original's bounded ``tries`` (``ok``) always run, as
        in the original. Beyond them (the original retries forever at top level) fixed
        stops once the solves of this loop's retries plus those of its analysis reach
        ``max_retry_solves``, or once a positive ``time_limit`` has elapsed."""
        if watch is not None:
            before = watch.solves
            # the analysis never takes the loop past max_retry_solves
            hopeless = watch.advance(None if self.max_retry_solves is None else self.max_retry_solves - run.work())
            run.analysis += watch.solves - before
            if hopeless:
                self.failures.append("no retry can give a plan without rotations; keeping the plan with rotations")
                return True
        if ok:
            return False
        work = run.work()
        if self.max_retry_solves is not None and work >= self.max_retry_solves:
            self.failures.append(f"stopped after {work} solves in the top-level retries (max_retry_solves = "
                                 f"{self.max_retry_solves})")
            return True
        if run.tl > 0 and self._now() - run.t0 >= run.tl:
            self.failures.append("time limit reached in the top-level retries")
            return True
        return False

    def _nc_fixed(self, g, s, goal, so, tl, splits, top):
        F = self.flags
        if splits <= 0:
            return self._leaf(g, s, goal, so, tl, not top)
        pp = F.time_print_paths and top
        if pp:
            F.time_print_paths = False
        run = _TopRun(self, tl)
        try:
            tries = _jpow_int(3.0, 4 - splits)
            middle = self.split_paths(g, s, goal, False)
            A = self._nc_fixed(g, s, middle, so, tl / 2, splits - 1, False)
            B = self._nc_fixed(g, middle, goal, so, tl / 2, splits - 1, False)
            if A is None or B is None:
                return None
            if splits == 1:
                best = (A, B)
                cur = (A, B)
                it = 0
                watch = None
                run.start()
                while True:
                    a, b = cur
                    # the original's condition hasCycles(A) > 0 || hasCycles(B) > 0, where it
                    # would throw (A null, or A acyclic and B null) the retries stop
                    if a is None or (has_cycles(a) == 0 and (b is None or has_cycles(b) == 0)):
                        break
                    ok = tries > 0
                    tries -= 1
                    if not (ok or top):
                        break
                    it += 1
                    if top and watch is None and (it >= self.retry_analysis_after or not ok):
                        watch = self._retry_watch("nc1", g, s, goal, so, tl)
                    if top and self._top_retry_stop(it, ok, run, watch):
                        break
                    m2 = self.split_paths(g, s, goal, True)
                    cur = (self._nc_fixed(g, s, m2, so, tl / 2, 0, False),
                           self._nc_fixed(g, m2, goal, so, tl / 2, 0, False))
                    if cur[0] is not None and cur[1] is not None:
                        best = cur
                if cur[0] is None or cur[1] is None:
                    self.failures.append("a re-planned piece failed; keeping the last complete pair")
                    cur = best
                A, B = cur
            else:
                run.start()
                for side in ("A", "B"):
                    cur = A if side == "A" else B
                    best = cur
                    it = 0
                    watch = None
                    if top and side == "B" and has_cycles(A) > 0:
                        # the retries of A stopped with rotations: the plan keeps them whatever B is
                        if has_cycles(B) > 0:
                            self.failures.append("second half not retried: the first keeps its rotations")
                        break
                    while cur is not None and has_cycles(cur) > 0:
                        ok = tries > 0
                        tries -= 1
                        if not (ok or top):
                            break
                        it += 1
                        if top:
                            if watch is None and (it >= self.retry_analysis_after or not ok):
                                ends = (s, middle) if side == "A" else (middle, goal)
                                watch = self._retry_watch("ncK", g, ends[0], ends[1], so, tl / 2, k=splits - 1)
                            if self._top_retry_stop(it, ok, run, watch):
                                break
                        if side == "A":
                            cur = self._nc_fixed(g, s, middle, so, tl / 2, splits - 1, False)
                        else:
                            cur = self._nc_fixed(g, middle, goal, so, tl / 2, splits - 1, False)
                        if cur is not None:
                            best = cur
                    if cur is None:
                        self.failures.append("a re-planned piece failed; keeping the last complete one")
                        cur = best
                    if side == "A":
                        A = cur
                    else:
                        B = cur
            full = self._concat(A, B)
            if pp:
                F.time_print_paths = True
                self._emit(print_paths(g, full, "fixed"))
            return full
        finally:
            if pp:
                F.time_print_paths = True

    # ------------------------------------------------------------ split by size (dead)
    def plan_paths_advanced_split_by_size(self, g: Graph, start: Sequence[int], goal: Sequence[int],
                                          set_optimal: bool = True, time_limit: float = -1.0, size: int = 2):
        """``planPathsAdvancedSplitBySize`` (*dead*): split (by
        :meth:`split_path_by_size`) while :meth:`get_longest_path_length` exceeds ``size``,
        solving the leaves as :meth:`plan_paths_advanced_split` does.

        ``original``: hangs inside ``splitPathBySize`` (:class:`OriginalWouldHang`), or
        recurses forever when a sub-instance repeats on the stack
        (:class:`JavaStackOverflowError`). Differs from the original: ``fixed`` uses the
        corrected :meth:`get_longest_path_length` / :meth:`split_path_by_size`, solves a
        repeated sub-instance directly and returns ``None`` for an infeasible piece.
        """
        with self._scope():
            return self._bs(g, list(start), list(goal), set_optimal, float(time_limit), size, set(), False)

    def _bs(self, g, s, goal, so, tl, size, stack, inner):
        key = (tuple(s), tuple(goal))
        if key in stack:
            if self.orig:
                raise JavaStackOverflowError(
                    "planPathsAdvancedSplitBySize recurses forever on the same sub-instance")
            return self._leaf(g, s, goal, so, tl, inner)
        L = self.get_longest_path_length(g, s, goal)
        if L > size:
            middle = self.split_path_by_size(g, s, goal, False, size)
            stack.add(key)
            try:
                A = self._bs(g, s, middle, so, tl / 2, size, stack, True)
                B = self._bs(g, middle, goal, so, tl / 2, size, stack, True)
            finally:
                stack.discard(key)
            if A is not None and B is not None:
                return self._concat(A, B)
            return None
        return self._leaf(g, s, goal, so, tl, inner)

    def plan_paths_advanced_split_by_size_no_cycle(self, g: Graph, start: Sequence[int], goal: Sequence[int],
                                                   set_optimal: bool = True, time_limit: float = -1.0,
                                                   size: int = 2, top: bool = True):
        """``planPathsAdvancedSplitBySizeNoCycle`` (*dead*): :meth:`plan_paths_advanced_split_by_size`
        with up to 5 retries (unbounded at top level in the original) while a half has
        rotations; a retry re-splits with :meth:`split_paths` (``random=True``, halving, not
        by size). A ``None`` half ends the retries and the call returns ``None``.

        ``original``: hangs and stack overflows as the other split-by-size variant, plus
        the endless top-level retry (:class:`OriginalWouldHang`, see the module
        docstring). Differs from the original: ``fixed`` stops the top-level retries as
        :meth:`plan_paths_advanced_split_no_cycle` does and keeps the last complete pair
        when a retry fails; a sub-instance whose longest path is not shorter than its
        parent's (a retry's random re-split can give one) is solved directly instead of
        split and retried again, so the recursion depth is at most
        :meth:`get_longest_path_length` (without this the descent never ended and raised
        ``RecursionError``); otherwise as :meth:`plan_paths_advanced_split_by_size`.
        """
        with self._scope(leaf_cache=self.orig):
            return self._bsnc(g, list(start), list(goal), set_optimal, float(time_limit), size, top, set(), False,
                              None)

    def _bsnc(self, g, s, goal, so, tl, size, top, stack, inner, parent_L):
        key = (tuple(s), tuple(goal))
        if key in stack:
            if self.orig:
                raise JavaStackOverflowError(
                    "planPathsAdvancedSplitBySizeNoCycle recurses forever on the same sub-instance")
            return self._leaf(g, s, goal, so, tl, inner)
        L = self.get_longest_path_length(g, s, goal)
        tries = 5
        if L <= size:
            return self._leaf(g, s, goal, so, tl, inner)
        if not self.orig and parent_L is not None and L >= parent_L:
            # fixed: a retry's random re-split (split_paths) can give a half that is no shorter
            # than its parent; splitting and retrying it again recursed without bound (new
            # middles, so no exact repeat on the stack) until RecursionError. Each level must
            # shrink the longest path, so the depth is at most L; such a half is solved directly.
            return self._leaf(g, s, goal, so, tl, inner)
        run = None if self.orig or not top else _TopRun(self, tl)
        middle = self.split_path_by_size(g, s, goal, False, size)
        stack.add(key)
        try:
            A = self._bsnc(g, s, middle, so, tl / 2, size, False, stack, True, L)
            B = self._bsnc(g, middle, goal, so, tl / 2, size, False, stack, True, L)
            best = (A, B)
            it = 0
            watch = None
            if run is not None:
                run.start()
            while A is not None and B is not None and (has_cycles(A, self.compat) > 0
                                                       or has_cycles(B, self.compat) > 0):
                ok = tries > 0
                tries -= 1
                if not (ok or top):
                    break
                it += 1
                if top and watch is None and (it >= self.retry_analysis_after or (not ok and not self.orig)):
                    watch = self._retry_watch("bsnc", g, s, goal, so, tl, size=size, stack=frozenset(stack))
                if self.orig:
                    if watch is not None:
                        self._retry_check(watch)
                elif top and self._top_retry_stop(it, ok, run, watch):
                    break
                middle = self.split_paths(g, s, goal, True)
                A = self._bsnc(g, s, middle, so, tl / 2, size, False, stack, True, L)
                B = self._bsnc(g, middle, goal, so, tl / 2, size, False, stack, True, L)
                if A is not None and B is not None:
                    best = (A, B)
            if not self.orig and (A is None or B is None) and best[0] is not None and best[1] is not None:
                self.failures.append("a re-planned piece failed; keeping the last complete pair")
                A, B = best
        finally:
            stack.discard(key)
        if A is not None and B is not None:
            return self._concat(A, B)
        return None

    # ------------------------------------------------------------ k-way split
    def multi_threaded_split_planning(self, g: Graph, start: Sequence[int], goal: Sequence[int],
                                      time_limit: float = -1.0, splits: int = 1):
        """``multiThreadedSplitPlanning``: the paper's k-way split (``k = 2^splits``
        pieces, the experiments' "k-way" runs).

        All intermediate configurations are computed first, level by level
        (``msg[s*step + step/2] = split_paths(msg[s*step], msg[(s+1)*step])``,
        ``step = 2^(splits-i)``); they equal those of the recursive split. Then each piece
        ``i`` is solved with ``solve_time(msg[i], msg[i+1], solveRelaxedFirst=false,
        setOptimal=true)`` and the full ``time_limit`` (the original runs one thread per
        piece; here the pieces run sequentially in index order, see the module docstring).
        A valid piece is stored in :attr:`mpath` (``planner.msg`` holds the
        configurations); the result is ``mpath[0]`` merged with ``mpath[1..]``. With
        ``time_print_paths`` the flag is off during the call and the result is logged once.

        ``original``: ``hasTimeLimit`` is always ``true``, so a non-positive limit makes
        every piece fail that is infeasible at its first horizon; ``mpath`` is never
        cleared, so a failed piece reuses the previous call's piece ``i``; a worker
        exception (e.g. :class:`JavaNullPointerException` for an unreachable goal) leaves
        the main thread waiting forever: :class:`OriginalWouldHang`. Differs from the
        original: see the module docstring.
        """
        with self._scope():
            return self._mt(g, list(start), list(goal), float(time_limit), splits)

    def _mt(self, g, start, goal, tl, splits):
        if not self.orig and splits < 0:
            raise ValueError("splits must be >= 0")
        if splits > MAX_SPLIT.bit_length() - 1:  # both modes: 2**splits configurations are allocated
            raise ValueError(f"splits must be <= {MAX_SPLIT.bit_length() - 1} (at most {MAX_SPLIT} pieces)")
        F = self.flags
        self.mg = g
        self.mtime = tl
        self.mpieces = mp = _jpow_int(2.0, splits)
        n = len(start)
        msg = [[0] * n for _ in range(mp + 1)]
        for i in range(n):
            msg[0][i] = start[i]
            msg[mp][i] = goal[i]
        self.msg = msg
        self.mdone = [False] * mp
        pp = False
        if F.time_print_paths:
            F.time_print_paths = False
            pp = True
        try:
            for i in range(splits):
                num = _jpow_int(2.0, i)
                step = _jpow_int(2.0, splits - i)
                for s_ in range(num):
                    middle = self._middle(g, msg[s_ * step], msg[(s_ + 1) * step], i > 0)
                    if middle is None:  # fixed: the split heuristic failed
                        self.mpath = {}
                        self.msg = None
                        return None
                    msg[s_ * step + step // 2] = list(middle)
            if not self.orig:
                self.mpath = {}  # fresh per call (the original never clears it)
            pieces = self.mpath
            for i in range(mp):
                has = True if self.orig else tl > 0
                try:
                    paths = self._makespan("time", g, msg[i], msg[i + 1], False, 0, True, has, tl, worker=True)
                except JavaException as e:
                    if not self.orig:
                        raise
                    self._emit_err(f'Exception in thread "Thread-{i}" {e.java_class}: {e}\n')
                    raise OriginalWouldHang(
                        f"multiThreadedSplitPlanning: worker {i} threw {e.java_class}; mdone[{i}] is never set "
                        "and the main thread waits forever") from e
                except OriginalWouldHang as e:
                    raise OriginalWouldHang(f"multiThreadedSplitPlanning: worker {i} never finishes: {e}") from e
                except Infeasible as e:
                    if mp == 1:
                        raise
                    self.failures.append(f"piece {i}: {e}")
                    paths = None
                if paths is not None:
                    pieces[i] = paths
                self.mdone[i] = True
            allp = pieces.get(0)
            for i in range(1, mp):
                allp = merge_paths(allp, pieces.get(i), self.compat)
        finally:
            if pp:
                F.time_print_paths = True
        if pp and allp is not None:
            self._emit(print_paths(g, allp, self.compat))
        return allp

    # ------------------------------------------------------------ retry analysis
    def _retry_watch(self, kind, g, s, goal, so, tl, **kw) -> _RetryWatch:
        """The endless-loop analysis of one top-level retry loop (see :class:`_RetryWatch`)."""
        return _RetryWatch(self, kind, g, s, goal, so, tl, kw)

    def _retry_check(self, watch: _RetryWatch) -> None:
        """``original``, at every top-level retry from ``retry_analysis_after`` on: advance
        the analysis by up to ``analysis_budget`` leaf solves and raise
        :class:`OriginalWouldHang` once it shows that no retry can end the loop."""
        if watch.advance():
            raise OriginalWouldHang(
                "the top-level retry loop (tries-- > 0 || top) never ends: every plan a retry can produce "
                "contains rotations (hasCycles > 0)")


class _RetryWatch:
    """The endless-loop analysis of one top-level no-cycle retry loop, run in installments.

    The set of plans a retry can produce is finite (every robot order of the random
    re-split, each leaf solved once), so the analysis always decides once all of its
    leaves are solved. Each :meth:`advance` solves at most ``analysis_budget`` new leaves
    and keeps everything it computed, so a loop that the original never leaves is
    recognised after about ``(number of leaves) / analysis_budget`` retries. The verdict:

    * ``original``: no retry can end the loop (every outcome has rotations; ``null``, an
      exception or a rotation-free pair would end it). The analysis shares the call's
      leaf cache, so its solves are not repeated by the loop.
    * ``fixed``: no retry can give a rotation-free plan (then the loop can only end, in the
      original, by hanging or crashing). Its leaves are solved in a cache of its own (the
      fixed loop solves its leaves afresh) and counted in :attr:`solves`.

    The analysis gives up for good (:attr:`verdict` stays ``None``) when an installment
    makes no progress (the enumeration alone exceeds the step bound), or when the
    heuristic hook is set. It never touches the planner's random stream.
    """

    def __init__(self, pl: PathPlanner, kind, g, s, goal, so, tl, kw):
        self.pl = pl
        self.kind, self.g, self.s, self.goal, self.so, self.tl, self.kw = kind, g, tuple(s), tuple(goal), so, tl, kw
        self.verdict: bool | None = None
        self.gave_up = False
        self.solves = 0
        self.cache: dict | None = None if pl.orig else {}
        self.oracle: _RetryOracle | None = None

    def advance(self, limit: int | None = None) -> bool:
        """One installment of at most ``analysis_budget`` (and ``limit``) new leaf solves;
        ``True`` once the loop is shown to be endless (hopeless)."""
        if self.verdict is not None or self.gave_up:
            return bool(self.verdict)
        pl = self.pl
        budget = pl.analysis_budget if limit is None else min(pl.analysis_budget, limit)
        if budget <= 0 < pl.analysis_budget:
            return False
        cache = pl._cache if pl.orig else self.cache
        if bool(B_HEURISTIC) or cache is None:
            self.gave_up = True
            return False
        if self.oracle is None:
            self.oracle = _RetryOracle(pl, self.g, self.so, budget, cache)
        o = self.oracle
        o.new, o.steps, o.budget = 0, 0, budget
        try:
            if self.kind == "nc1":
                self.verdict = o.nc1_top_hangs(self.s, self.goal, self.tl)
            elif self.kind == "ncK":
                self.verdict = o.nck_top_hangs(self.s, self.goal, self.kw["k"], self.tl)
            else:
                self.verdict = o.bsnc_top_hangs(self.s, self.goal, self.tl, self.kw["size"], self.kw["stack"])
        except _Unknown:
            if o.new == 0:
                self.gave_up = True
        except ValueError:
            self.gave_up = True
        finally:
            self.solves += o.new
        return bool(self.verdict)


class _RetryOracle:
    """Outcome sets of the original's split procedures over every random choice.

    An outcome is ``("P", cyclic)`` (a path set; only whether it has rotations matters,
    and rotations of a concatenation are those of its pieces, :func:`has_cycles` being
    a sum over time steps), ``("N",)`` (``null``) or ``("X", name)`` (an exception or a
    hang that ends the call). Leaf solves are stored in ``cache`` (solved without
    touching the planner's random stream); everything else is computed from BFS
    lengths. ``fixed``: the verdicts ask whether a rotation-free plan is reachable, and
    a second half that cannot matter for that (the first is not rotation-free) is not
    solved but taken as ``("P", True)``, which can only add outcomes.
    """

    def __init__(self, pl: PathPlanner, g: Graph, so: bool, budget: int, cache: dict):
        self.pl, self.g, self.so = pl, g, so
        self.fixed = not pl.orig
        self.budget = budget
        self.cache = cache
        self.new = 0
        self.steps = 0
        self.memo: dict = {}

    def tick(self, k: int = 1) -> None:
        self.steps += k
        if self.steps > _ANALYSIS_STEPS:
            raise _Unknown()

    # -- primitives
    def leaf(self, s, goal, tl):
        key = (s, goal, self.so, tl)
        ent = self.cache.get(key)
        if ent is None:
            if self.new >= self.budget:
                raise _Unknown()
            self.new += 1
            ent = self.pl._run_leaf_captured(self.g, list(s), list(goal), self.so, tl, MathRandom(0))
            self.cache[key] = ent
        if ent.exc is not None:
            return ("X", type(ent.exc).__name__)
        if ent.paths is None:
            return _NULL
        if ent.cyclic is None:
            ent.cyclic = has_cycles(ent.paths, self.pl.compat) > 0
        return ("P", ent.cyclic)

    @staticmethod
    def has_cyc(o) -> bool:
        return o[1]

    @staticmethod
    def acyclic(o) -> bool:
        return o[0] == "P" and not o[1]

    def lengths(self, s, goal):
        L = []
        for a, b in zip(s, goal):
            d = _bfs_dists(self.g, a)
            if b >= len(d) or d[b] < 0:
                return None
            L.append(d[b] + 1)
        return L

    @staticmethod
    def final(a, b):
        if a[0] == "P" and b[0] == "P":
            return ("P", a[1] or b[1])
        return _NULL

    def split_det(self, s, goal):
        key = ("sd", s, goal)
        if key not in self.memo:
            L = self.lengths(s, goal)
            if L is None:
                self.memo[key] = _NPE
            else:
                order = sorted(range(len(L)), key=lambda i: -L[i])
                self.memo[key] = self._assign(s, goal, L, order)
        return self.memo[key]

    def _assign(self, s, goal, L, order):
        used: set[int] = set()
        m = [0] * len(s)
        for ag in order:
            self.tick()
            v = self.pick(self.pl._windows(self.g, s[ag], goal[ag], L[ag]), s[ag], goal[ag], used, ag)
            if isinstance(v, tuple):
                return v
            m[ag] = v
            used.add(v)
        return ("M", tuple(m))

    def pick(self, w, s, goal, used, ag):
        """The intermediate goal ``splitPaths`` gives robot ``ag`` (``fixed``: with its
        fallback), or the outcome that ends the call."""
        v = w.choose(used)
        if v is not None:
            return v
        if not self.fixed:
            return ("X", "OriginalWouldHang")
        try:
            return _split_fallback(self.g, s, goal, used, ag)
        except Infeasible:
            return ("X", "Infeasible")

    def split_rand(self, s, goal):
        """Every outcome of ``splitPaths(s, goal, true)`` over all robot orders, likeliest
        first (the share of the ``n!`` equally likely orders that give it; ties in
        discovery order). The order is deterministic (it never depends on hashing), and
        the analysis examines, and solves the leaves of, the outcomes a retry is likeliest
        to produce first."""
        key = ("sr", s, goal)
        if key in self.memo:
            return self.memo[key]
        L = self.lengths(s, goal)
        if L is None:
            self.memo[key] = (_NPE,)
            return self.memo[key]
        n = len(L)
        W = [self.pl._windows(self.g, s[a], goal[a], L[a]) for a in range(n)]
        # Breadth first over partial assignments (robot -> vertex, -1 = not yet placed):
        # the vertex a robot gets depends only on the vertices already used, so two orders
        # that placed the same robots on the same vertices continue identically and are
        # merged. (Enumerating ordered choice sequences instead grows like n!.) ``level``
        # maps each partial assignment of ``k`` robots to the number of order prefixes
        # that reach it; an outcome weighs the number of full orders that give it.
        out: dict = {}
        level: dict = {tuple([-1] * n): 1}
        for k in range(n):
            nxt: dict = {}
            rest = math.factorial(n - k - 1)
            for m, c in level.items():
                used = {v for v in m if v >= 0}
                for ag in range(n):
                    if m[ag] >= 0:
                        continue
                    self.tick()
                    v = self.pick(W[ag], s[ag], goal[ag], used, ag)
                    if isinstance(v, tuple):
                        out[v] = out.get(v, 0) + c * rest
                        continue
                    m2 = m[:ag] + (v,) + m[ag + 1:]
                    nxt[m2] = nxt.get(m2, 0) + c
            level = nxt
        for m, c in level.items():
            out[("M", m)] = out.get(("M", m), 0) + c
        ranked = sorted(enumerate(out.items()), key=lambda e: (-e[1][1], e[0]))
        self.memo[key] = r = tuple(o for _, (o, _) in ranked)
        return r

    @staticmethod
    def all_cyclic_of(oracle, outcomes) -> bool:
        return all(o[0] == "P" and oracle.has_cyc(o) for o in outcomes)

    def all_cyclic(self, outcomes) -> bool:
        return self.all_cyclic_of(self, outcomes)

    # -- planPathsAdvancedSplitNoCycle (non-top)
    def cond1(self, a, b):
        if a[0] == "N":
            return "NPE"
        if self.has_cyc(a):
            return True
        if b[0] == "N":
            return "NPE"
        return self.has_cyc(b)

    def nc1_option(self, s, mm, goal, tl):
        """What one retry with intermediate goals ``mm`` (a ``split_rand`` outcome) gives:
        ``("S", a, b)`` or the exception that ends the call."""
        if mm[0] == "X":
            return mm
        a = self.leaf(s, mm[1], tl / 2)
        if a[0] == "X":
            return a
        if self.fixed and not self.acyclic(a):
            return ("S", a, ("P", True))
        b = self.leaf(mm[1], goal, tl / 2)
        return b if b[0] == "X" else ("S", a, b)

    def nc1_options(self, s, goal, tl):
        key = ("o1", s, goal, tl)
        if key not in self.memo:
            self.memo[key] = _uniq(self.nc1_option(s, mm, goal, tl) for mm in self.split_rand(s, goal))
        return self.memo[key]

    def nc1_top_hangs(self, s, goal, tl) -> bool:
        """``original``: no retry can end the loop (every one gives a pair with rotations).
        ``fixed``: no retry gives a pair without rotations. Stops at the first retry
        that decides otherwise."""
        for mm in self.split_rand(s, goal):
            o = self.nc1_option(s, mm, goal, tl)
            if self.fixed:
                if o[0] == "S" and self.acyclic(o[1]) and self.acyclic(o[2]):
                    return False
            elif o[0] == "X" or self.cond1(o[1], o[2]) is not True:
                return False
        return True

    def nck_top_hangs(self, s, goal, k, tl) -> bool:
        """The top-level loop that re-solves one half (``planPathsAdvancedSplitNoCycle``
        with ``splits = k + 1``): ``original``: every outcome of the half has rotations;
        ``fixed``: no outcome is free of them."""
        outs = self.nc(s, goal, k, tl)
        if self.fixed:
            return not any(self.acyclic(o) for o in outs)
        return self.all_cyclic(outs)

    def nc(self, s, goal, k, tl):
        key = ("nc", s, goal, k, tl)
        if key in self.memo:
            return self.memo[key]
        if k <= 0:
            r = (self.leaf(s, goal, tl),)
            self.memo[key] = r
            return r
        m0 = self.split_det(s, goal)
        if m0[0] == "X":
            self.memo[key] = (m0,)
            return self.memo[key]
        m = m0[1]
        PA = self.nc(s, m, k - 1, tl / 2)
        PB = None
        out = _OSet()
        tries = _jpow_int(3.0, 4 - k)
        for a in PA:
            if a[0] == "X":
                out.add(a)
                continue
            if PB is None:
                PB = self.nc(m, goal, k - 1, tl / 2)
            for b in PB:
                if b[0] == "X":
                    out.add(b)
                elif a[0] == "N" or b[0] == "N":
                    out.add(_NULL)
                elif k == 1:
                    out |= self._loop1(s, goal, tl, a, b, tries)
                else:
                    out |= self._loopk(s, goal, m, k, tl, a, b, tries)
        self.memo[key] = r = tuple(out)
        return r

    def _loop1(self, s, goal, tl, a, b, tries):
        out = _OSet()
        seen = set()
        stack = [(a, b, tries)]
        while stack:
            st = stack.pop()
            if st in seen:
                continue
            seen.add(st)
            self.tick()
            a, b, t = st
            c = self.cond1(a, b)
            if c == "NPE":
                out.add(_NPE)
                continue
            if c and t > 0:
                for o in self.nc1_options(s, goal, tl):
                    if o[0] == "X":
                        out.add(o)
                    else:
                        stack.append((o[1], o[2], t - 1))
                continue
            out.add(self.final(a, b))
        return out

    def _loopk(self, s, goal, m, k, tl, a, b, tries):
        out = _OSet()
        seen = set()
        stack = [("A", a, b, tries)]
        PA = PB = None
        while stack:
            st = stack.pop()
            if st in seen:
                continue
            seen.add(st)
            self.tick()
            ph, a, b, t = st
            cur = a if ph == "A" else b
            if cur[0] == "N":
                out.add(_NPE)
                continue
            if self.has_cyc(cur):
                if t > 0:
                    if ph == "A":
                        PA = PA if PA is not None else self.nc(s, m, k - 1, tl / 2)
                        nxt = [("A", x, b, t - 1) for x in PA if x[0] != "X"]
                        out |= [x for x in PA if x[0] == "X"]
                    else:
                        PB = PB if PB is not None else self.nc(m, goal, k - 1, tl / 2)
                        nxt = [("B", a, x, t - 1) for x in PB if x[0] != "X"]
                        out |= [x for x in PB if x[0] == "X"]
                    stack.extend(nxt)
                    continue
                t -= 1
            if ph == "A":
                stack.append(("B", a, b, t))
            else:
                out.add(self.final(a, b))
        return out

    # -- planPathsAdvancedSplitBySizeNoCycle
    def lpl(self, s, goal):
        L = self.lengths(s, goal)
        if L is None:
            return None
        if not self.pl.orig:
            return max(L) if L else 0
        length = 0
        for x in L:
            if x - 1 > length:
                length = x
        return length

    def split_by_size_det(self, s, goal, size):
        key = ("sbs", s, goal, size)
        if key not in self.memo:
            L = self.lengths(s, goal)
            if L is None:
                self.memo[key] = _NPE
            else:
                order = sorted(range(len(L)), key=lambda i: -L[i])
                try:
                    m = self.pl._assign_by_size(self.g, list(s), list(goal), L, order, size)
                    self.memo[key] = ("M", tuple(m))
                except (OriginalWouldHang, Infeasible) as e:
                    self.memo[key] = ("X", type(e).__name__)
        return self.memo[key]

    def bsnc(self, s, goal, tl, size, stack: frozenset):
        if (s, goal) in stack:
            # fixed solves a repeated sub-instance directly
            return (self.leaf(s, goal, tl) if self.fixed else ("X", "JavaStackOverflowError"),)
        key = ("bsnc", s, goal, tl, size, stack)
        if key in self.memo:
            return self.memo[key]
        self.tick()
        L = self.lpl(s, goal)
        if L is None:
            r = (_NPE,)
        elif L > size:
            m0 = self.split_by_size_det(s, goal, size)
            if m0[0] == "X":
                r = (m0,)
            else:
                st2 = stack | {(s, goal)}
                PA = self.bsnc(s, m0[1], tl / 2, size, st2)
                out = _OSet()
                PB = None
                for a in PA:
                    if a[0] == "X":
                        out.add(a)
                        continue
                    if PB is None:
                        PB = self.bsnc(m0[1], goal, tl / 2, size, st2)
                    for b in PB:
                        if b[0] == "X":
                            out.add(b)
                        else:
                            out |= self._loop_bs(s, goal, tl, size, st2, a, b, 5)
                r = tuple(out)
        else:
            r = (self.leaf(s, goal, tl),)
        self.memo[key] = r
        return r

    def bs_option(self, s, mm, goal, tl, size, st2):
        """The outcomes of one by-size retry with intermediate goals ``mm``."""
        if mm[0] == "X":
            return (mm,)
        opts = _OSet()
        PA = self.bsnc(s, mm[1], tl / 2, size, st2)
        if self.fixed and not any(self.acyclic(a) for a in PA):
            PB = (("P", True),)
        else:
            PB = None
        for a in PA:
            if a[0] == "X":
                opts.add(a)
                continue
            if PB is None:
                PB = self.bsnc(mm[1], goal, tl / 2, size, st2)
            for b in PB:
                opts.add(b if b[0] == "X" else ("S", a, b))
        return tuple(opts)

    def bs_options(self, s, goal, tl, size, st2):
        key = ("obs", s, goal, tl, size, st2)
        if key not in self.memo:
            out = _OSet()
            for mm in self.split_rand(s, goal):
                out |= self.bs_option(s, mm, goal, tl, size, st2)
            self.memo[key] = tuple(out)
        return self.memo[key]

    def _bs_cond(self, a, b) -> bool:
        return a[0] == "P" and b[0] == "P" and (self.has_cyc(a) or self.has_cyc(b))

    def _loop_bs(self, s, goal, tl, size, st2, a, b, tries):
        out = _OSet()
        seen = set()
        stack = [(a, b, tries)]
        while stack:
            st = stack.pop()
            if st in seen:
                continue
            seen.add(st)
            self.tick()
            a, b, t = st
            if self._bs_cond(a, b) and t > 0:
                for o in self.bs_options(s, goal, tl, size, st2):
                    if o[0] == "X":
                        out.add(o)
                    else:
                        stack.append((o[1], o[2], t - 1))
                continue
            out.add(self.final(a, b))
        return out

    def bsnc_top_hangs(self, s, goal, tl, size, stack) -> bool:
        """As :meth:`nc1_top_hangs`, for ``planPathsAdvancedSplitBySizeNoCycle``."""
        st2 = frozenset(stack) | {(s, goal)}
        for mm in self.split_rand(s, goal):
            for o in self.bs_option(s, mm, goal, tl, size, st2):
                if self.fixed:
                    if o[0] == "S" and self.acyclic(o[1]) and self.acyclic(o[2]):
                        return False
                elif o[0] == "X" or not self._bs_cond(o[1], o[2]):
                    return False
        return True


# ------------------------------------------------------------ module-level wrappers
def _planner(compat, backend, params, rng, flags, echo, clock=None) -> PathPlanner:
    return PathPlanner(backend=backend, params=params, rng=rng, compat=compat, flags=flags, echo=echo, clock=clock)


def split_paths(g: Graph, start: Sequence[int], goal: Sequence[int], random: bool = False, *,
                rng: MathRandom | None = None, compat: str = "fixed") -> list[int]:
    """:meth:`PathPlanner.split_paths` on a fresh planner."""
    return _planner(compat, "highs", None, rng, None, False).split_paths(g, start, goal, random)


def split_path_by_size(g: Graph, start: Sequence[int], goal: Sequence[int], random: bool = False,
                       first_size: int = 2, *, rng: MathRandom | None = None, compat: str = "fixed") -> list[int]:
    """:meth:`PathPlanner.split_path_by_size` on a fresh planner."""
    return _planner(compat, "highs", None, rng, None, False).split_path_by_size(g, start, goal, random, first_size)


def get_longest_path_length(g: Graph, start: Sequence[int], goal: Sequence[int], *, rng: MathRandom | None = None,
                            compat: str = "fixed") -> int:
    """:meth:`PathPlanner.get_longest_path_length` on a fresh planner."""
    return _planner(compat, "highs", None, rng, None, False).get_longest_path_length(g, start, goal)


def _wrap(method: str, doc_target: str):
    def f(*args, rng: MathRandom | None = None, backend: str = "highs", params: dict | None = None,
          compat: str = "fixed", flags: SolverFlags | None = None, echo: bool = False, **kw):
        pl = _planner(compat, backend, params, rng, flags, echo)
        return getattr(pl, method)(*args, **kw)

    f.__name__ = method
    f.__qualname__ = method
    f.__doc__ = (f":meth:`PathPlanner.{method}` on a fresh planner (``{doc_target}``); keyword arguments "
                 "``rng``, ``backend``, ``params``, ``compat``, ``flags``, ``echo`` configure the planner. "
                 "Returns ``paths`` or ``None``.")
    return f


plan_paths_advanced = _wrap("plan_paths_advanced", "planPathsAdvanced")
plan_paths_advanced_pmg = _wrap("plan_paths_advanced_pmg", "planPathsAdvancedPMG")
plan_paths_advanced_tt = _wrap("plan_paths_advanced_tt", "planPathsAdvancedTT")
plan_paths_advanced_td = _wrap("plan_paths_advanced_td", "planPathsAdvancedTD")
plan_paths_advanced_split = _wrap("plan_paths_advanced_split", "planPathsAdvancedSplit")
plan_paths_advanced_split_dist = _wrap("plan_paths_advanced_split_dist", "planPathsAdvancedSplitDist")
plan_paths_advanced_split_no_cycle = _wrap("plan_paths_advanced_split_no_cycle", "planPathsAdvancedSplitNoCycle")
plan_paths_advanced_split_by_size = _wrap("plan_paths_advanced_split_by_size", "planPathsAdvancedSplitBySize")
plan_paths_advanced_split_by_size_no_cycle = _wrap("plan_paths_advanced_split_by_size_no_cycle",
                                                   "planPathsAdvancedSplitBySizeNoCycle")
multi_threaded_split_planning = _wrap("multi_threaded_split_planning", "multiThreadedSplitPlanning")


# --------------------------------------------------------------------- metrics
def _arrival(p: Sequence[int], g: int) -> int | None:
    if not p or p[-1] != g:
        return None
    t = len(p) - 1
    while t > 0 and p[t - 1] == g:
        t -= 1
    return t


def plan_metrics(paths: Sequence[Sequence[int]], goals: Sequence[int]) -> dict:
    """The four T-RO objectives of a plan: ``makespan`` (``len - 1``), ``total_time``
    (sum of the arrival times, the first step after which a robot stays at its goal),
    ``total_distance`` (number of moves; waits are free) and ``max_distance`` (the most
    moves of any robot). ``total_time`` is ``None`` if a robot does not end at its goal.

    ``paths`` needs one row per goal, all of the same length >= 1 (``[]`` with no goals is
    the empty plan, all metrics 0); otherwise :class:`ValueError` is raised (``None``, a row
    count other than ``len(goals)``, an empty row, rows of different lengths)."""
    if paths is None:
        raise ValueError("plan_metrics needs a plan, got None")
    paths = [as_ids(p, "path entries") for p in paths]  # also a numpy array
    goals = as_ids(goals, "goals")
    if len(paths) != len(goals):
        raise ValueError(f"{len(paths)} paths for {len(goals)} goals")
    if not paths:
        return {"makespan": 0, "total_time": 0, "total_distance": 0, "max_distance": 0}
    if any(len(p) != len(paths[0]) for p in paths) or not paths[0]:
        raise ValueError("the paths must be non-empty and of equal length")
    arr = [_arrival(p, g) for p, g in zip(paths, goals)]
    dist = [sum(1 for i in range(len(p) - 1) if p[i] != p[i + 1]) for p in paths]
    return {
        "makespan": len(paths[0]) - 1,
        "total_time": None if any(a is None for a in arr) else sum(arr),
        "total_distance": sum(dist),
        "max_distance": max(dist),
    }


@dataclass
class PlanResult:
    """The outcome of :func:`plan`.

    Attributes:
        paths: ``paths[a][t]`` (all rows of length ``makespan + 1``), or ``None``.
        status: ``"optimal"`` (proved optimal for the objective), ``"feasible"`` (a plan
            without optimality proof: a split, a gap that did not close, a single horizon
            above the lower bound, or the original's single horizon),
            ``"time_limit"`` or ``"failed"`` (no plan; see ``extra["failures"]``).
        objective: the objective that was optimized.
        value: its value for ``paths`` (``None`` without a plan).
        makespan, total_time, total_distance, max_distance: the four metrics of ``paths``
            (:func:`plan_metrics`).
        lower_bounds: ``makespan`` / ``max_distance`` = the longest shortest path,
            ``total_time`` / ``total_distance`` = the sum of shortest paths (BFS, no draws).
        split: ``k`` (number of pieces); ``intermediate_goals``: the configurations between
            the pieces (``msg[1..k-1]`` of the makespan k-way split; the middles of a
            total-distance split, in time order), or ``None``.
        time: wall-clock seconds.
        stats: ``solves`` (models solved), ``lp_solves``, ``ilp_solves``, ``solver_time``
            (seconds reported by the backend), ``horizons`` (the ``T`` of every model, in
            order) and ``phases`` (per horizon: ``"lp"`` for an LP relaxation, ``"ilp"`` for
            an ILP, ``"joint"`` for a makespan plan, or a split's piece, that the makespan loop
            took from its joint-state search with ``backend="builtin"``: no model was solved
            at that horizon, and it is not counted in ``solves``).
        log: the original's console text (see :class:`PathPlanner`).
        compat: the mode.
        extra: details (``failures``, ``mpath_keys``, ``horizon``: the one horizon solved with
            ``horizon="single"`` or ``compat="original"``; the horizon search sets none, ...).
    """

    paths: list[list[int]] | None
    status: str
    objective: str
    value: float | None
    makespan: int | None
    total_time: int | None
    total_distance: int | None
    max_distance: int | None
    lower_bounds: dict
    split: int
    intermediate_goals: list[list[int]] | None
    time: float
    stats: dict
    log: str
    compat: str
    extra: dict = field(default_factory=dict)

    @property
    def metrics(self) -> dict:
        return {"makespan": self.makespan, "total_time": self.total_time, "total_distance": self.total_distance,
                "max_distance": self.max_distance}


_OBJECTIVES = ("makespan", "total_time", "total_distance", "max_distance")


def _lower_bounds(g: Graph, starts, goals) -> dict:
    d = []
    for s, t in zip(starts, goals):
        dist = _bfs_dists(g, s)
        d.append(dist[t] if 0 <= t < len(dist) and dist[t] >= 0 else None)
    if any(x is None for x in d):
        return {k: None for k in _OBJECTIVES}
    mx = max(d) if d else 0
    sm = sum(d)
    return {"makespan": mx, "total_time": sm, "total_distance": sm, "max_distance": mx}


def plan(problem, objective: str = "makespan", split: int = 1, *, backend: str = "highs",
         time_limit: float | None = None, compat: str = "fixed", gap: float = 0.0, rng: MathRandom | None = None,
         params: dict | None = None, solve_relaxed_first: bool = True, horizon: str | None = None,
         debug: bool = False, echo: bool = False,
         feasibility_budget: int = DEFAULT_FEASIBILITY_BUDGET) -> PlanResult:
    """Plan paths for ``problem`` (a :class:`~mppflow.problem.Problem` or a tuple
    ``(graph, starts, goals)``).

    ``objective``:

    * ``"makespan"``: ``split == 1``: :meth:`PathPlanner.plan_paths_advanced` (optimal,
      LP relaxation first when ``solve_relaxed_first``, as ``Main.solveProblem``);
      ``split = k > 1``: the k-way split :meth:`PathPlanner.multi_threaded_split_planning`
      (``k`` in 2, 4, 8, 16, ... gives the original's split levels 1, 2, 3, 4).
    * ``"total_time"`` / ``"total_distance"`` (``split == 1``): ``fixed`` runs
      Algorithm 1's horizon search (:func:`~mppflow.solvers.solve_total_time` /
      :func:`~mppflow.solvers.solve_distance`; ``horizon="single"`` solves at the
      optimal makespan only); ``original`` solves at the makespan of the original's
      sequential 4-way split, as ``Main.solveProblemTotalTime`` / ``solveProblemDistance``
      do (without ``Main``'s time accounting). ``gap`` is the MIP gap.
    * ``"total_distance"`` with ``split = k > 1``:
      :meth:`PathPlanner.plan_paths_advanced_split_dist` (``Main.solveProblemDistanceSplit``).
    * ``"max_distance"``: :func:`~mppflow.solvers.solve_max_distance` (new; ``original``
      raises :class:`~mppflow.errors.NotInOriginal`); ``horizon="single"`` solves at the
      optimal makespan, as for TT / DIST.

    ``horizon``: ``None`` / ``"search"`` or ``"single"`` (anything else raises
    ``ValueError``); it is ignored for the makespan and for ``split > 1``. A single horizon
    is optimal at that ``T`` only, so its status is ``"feasible"`` unless the plan meets the
    lower bound. ``time_limit`` is in seconds (``None``: unlimited; with ``split = k`` it
    applies to each piece); in ``fixed`` mode it must be ``None`` or a finite number > 0
    (``ValueError`` otherwise, as for ``gap``; ``original`` keeps the original's rule that a
    limit <= 0 means no limit). ``debug`` turns on the solvers' messages in ``result.log``.
    Splitting is only defined for the makespan and the total distance (``ValueError``
    otherwise), with ``split <= MAX_SPLIT`` (1024; ``ValueError`` above, in both modes); it is a heuristic and can fail (status ``"failed"``) on a solvable instance
    whose intermediate configuration is unreachable, e.g. on a cycle
    (``docs/notes/paper-vs-code.md``). Before a split, ``fixed`` runs the instance checks of
    the unsplit solve (duplicate starts: ``ValueError``; duplicate goals, a goal unreachable
    from its start, or no plan for any horizon by :func:`~mppflow.solvers.joint_feasibility`
    within ``feasibility_budget``: :class:`~mppflow.errors.Infeasible`), so ``"failed"``
    means that the heuristic failed on an instance that is solvable or whose feasibility the
    budget did not decide. ``compat="original"`` has no horizon search:
    ``horizon="search"`` raises :class:`~mppflow.errors.NotInOriginal` there, as in the
    solvers (``None`` and ``"single"`` solve at the split's makespan).
    """
    if objective not in _OBJECTIVES:
        raise ValueError(f"objective must be one of {_OBJECTIVES}")
    if horizon not in (None, "single", "search"):
        raise ValueError("horizon must be 'single' or 'search'")
    split = as_int(split, "split")
    if split < 1 or split & (split - 1):
        raise ValueError("split must be a power of two (1, 2, 4, 8, 16, ...)")
    if split > MAX_SPLIT:
        raise ValueError(f"split must be at most {MAX_SPLIT} (pieces beyond the makespan are only waits)")
    splits = split.bit_length() - 1
    if split > 1 and objective not in ("makespan", "total_distance"):
        raise ValueError("splitting is defined for the makespan and the total distance only")
    orig = check_compat(compat)
    _check_budget(feasibility_budget)
    if not orig:
        _check_gap(gap)
        time_limit = _check_time_limit(time_limit)
    if orig and objective == "max_distance":
        raise NotInOriginal("MINMAXDIST is not implemented in the original code")
    if orig and horizon == "search" and objective in ("total_time", "total_distance"):
        raise NotInOriginal("the horizon search of Algorithm 1 is not in the original (use horizon='single')")
    if isinstance(problem, tuple):
        graph, starts, goals = problem
    else:
        graph, starts, goals = problem.graph, problem.starts, problem.goals
    starts, goals = as_ids(starts, "starts"), as_ids(goals, "goals")
    if split > 1 and not orig:
        # the unsplit solve's instance checks: "failed" is then a failure of the heuristic only
        _check_instance(graph, starts, goals)
        for i, d in enumerate(_distances(graph, starts, goals)):
            if d is None:
                raise Infeasible(f"goal of robot {i} is unreachable from its start")
        if joint_feasibility(graph, starts, goals, feasibility_budget) is False:
            raise Infeasible("the instance has no solution for any horizon (joint-state search)")
    tl = float(time_limit) if time_limit is not None and time_limit > 0 else -1.0
    flags = SolverFlags.all(debug)
    pl = PathPlanner(backend=backend, params=params, rng=rng, compat=compat, flags=flags, echo=echo,
                     feasibility_budget=feasibility_budget)
    t0 = time.time()
    status = None
    inter = None
    extra: dict = {}
    out_of_time = False  # the budget ran out before the horizon solves started

    def remaining():
        nonlocal out_of_time
        rem = tl if tl <= 0 else tl - (time.time() - t0)
        if tl > 0 and rem <= 0:
            out_of_time = True
        return rem

    if objective == "makespan":
        if split == 1:
            paths = pl.plan_paths_advanced(graph, starts, goals, solve_relaxed_first, 0, True, tl)
            if paths is not None:
                status = "optimal"
        else:
            paths = pl.multi_threaded_split_planning(graph, starts, goals, tl, splits)
            inter = [list(c) for c in pl.msg[1:-1]] if pl.msg else None
            extra["mpath_keys"] = sorted(pl.mpath)
    elif objective in ("total_time", "total_distance") and split == 1:
        T = None
        if orig or horizon == "single":
            if orig:
                tp = pl.plan_paths_advanced_split(graph, starts, goals, False, -1.0, 2)
            else:
                tp = pl.plan_paths_advanced(graph, starts, goals, solve_relaxed_first, 0, True, tl)
            T = None if tp is None else (len(tp[0]) - 1 if tp else 0)
            extra["horizon"] = T
        if (orig or horizon == "single") and T is None:
            paths = None
        else:
            hz = "single" if (orig or horizon == "single") else horizon
            rem = remaining()
            if tl > 0 and rem <= 0:
                paths = None
            elif objective == "total_time":
                opt = [0]
                paths = pl.plan_paths_advanced_tt(graph, starts, goals, opt, gap, T, rem, horizon=hz)
            else:
                paths = pl.plan_paths_advanced_td(graph, starts, goals, gap, T, rem, horizon=hz)
            if paths is not None and not orig and pl.solves:
                # fixed-mode solver statuses carry the proof: "optimal" needs gap 0 or a closed
                # bound, and a single horizon is optimal only when it meets the lower bound
                status = "optimal" if pl.solves[-1].status == "optimal" else "feasible"
            if paths is not None:
                # the original's horizon (the 4-way split's makespan) leaves trailing steps in
                # which every robot waits at its goal: drop them, so that the makespan is the
                # latest arrival (T-RO Sec. II-B); a single solved horizon stays in extra["horizon"]
                paths = _trim(paths)
                v, lbv = plan_metrics(paths, goals)[objective], _lower_bounds(graph, starts, goals)[objective]
                if orig and v is not None and lbv is not None and v <= lbv and path_set_error(paths, graph, starts, goals) is None:
                    status = "optimal"  # a valid plan that meets the lower bound: optimal for every horizon
    elif objective == "total_distance":
        paths = pl.plan_paths_advanced_split_dist(graph, starts, goals, gap, tl, splits)
        inter = [list(c) for c in pl.dist_mids] if pl.dist_mids else None
    else:
        T = None
        r = None
        if horizon == "single":  # as for TT / DIST: solve at the optimal makespan
            tp = pl.plan_paths_advanced(graph, starts, goals, solve_relaxed_first, 0, True, tl)
            T = None if tp is None else (len(tp[0]) - 1 if tp else 0)
            extra["horizon"] = T
        rem = remaining()
        if (horizon == "single" and T is None) or (tl > 0 and rem <= 0):
            paths = None
        else:
            r = solve_max_distance(graph, starts, goals, gap, T, rem, horizon=horizon, backend=backend,
                                   params=params, debug=debug, echo=echo, feasibility_budget=feasibility_budget)
            pl.solves.append(r)
            pl._log.append(r.log)
            paths = r.paths
        if paths is not None:
            status = "optimal" if r.status == "optimal" else "feasible"
    elapsed = time.time() - t0
    if paths is None:
        status = ("time_limit" if out_of_time or pl.time_limit_hit or any(r.status == "time_limit" for r in pl.solves)
                  else "failed")
    elif status is None:
        status = "feasible"
    m = plan_metrics(paths, goals) if paths is not None else dict.fromkeys(_OBJECTIVES)
    attempts = [a for r in pl.solves for a in r.attempts]
    models = [a for a in attempts if a.get("phase") != "joint"]
    stats = {
        "solves": len(models),
        "lp_solves": sum(1 for a in models if a.get("phase") == "lp"),
        "ilp_solves": sum(1 for a in models if a.get("phase") != "lp"),
        "solver_time": sum(a.get("runtime") or 0.0 for a in models),
        "horizons": [a.get("T") for a in attempts],
        "phases": [a.get("phase") if a.get("phase") in ("lp", "joint") else "ilp" for a in attempts],
    }
    if pl.failures:
        extra["failures"] = list(pl.failures)
    return PlanResult(paths=paths, status=status, objective=objective, value=m[objective], makespan=m["makespan"],
                      total_time=m["total_time"], total_distance=m["total_distance"],
                      max_distance=m["max_distance"], lower_bounds=_lower_bounds(graph, starts, goals), split=split,
                      intermediate_goals=inter, time=elapsed, stats=stats, log=pl.log, compat=compat, extra=extra)


# ------------------------------------------------------------------------ main
def path_planner_main(*, heuristic: Callable | None = None, backend: str = "highs", params: dict | None = None,
                      rng: MathRandom | None = None, compat: str = "fixed", echo: bool = False) -> str:
    """``PathPlanner.main``: on ``Problem.getLongStraightWithOneGarageProblem()`` (a line
    0..8 with a garage 9 at vertex 4, two robots swapping ends), print the start and goal
    vertices, run the local-repair heuristic ``planHeuristicPaths``, print the paths and
    "Path set is valid." if they are (else the message ``isPathSetValid`` prints). Returns
    the text (``echo`` prints it).

    By default (``heuristic=None``) this is :func:`mppflow.heuristic.path_planner_main`,
    which runs :meth:`mppflow.heuristic.LocalRepairPlanner.plan_heuristic_paths` with
    ``rng``, ``backend``, ``params`` and ``compat``. ``heuristic(graph, starts, goals)``
    replaces the heuristic: it returns the paths (or ``None``), or an object with
    ``.paths`` and optionally ``.log`` (the text it prints).
    """
    if heuristic is None:
        from .heuristic import path_planner_main as _heuristic_main

        return _heuristic_main(compat=compat, backend=backend, params=params, rng=rng, echo=echo)

    from .problem import get_long_straight_with_one_garage_problem

    p = get_long_straight_with_one_garage_problem()
    parts = [print_start_and_goals(p.graph, p.starts, p.goals, compat)]
    if echo:
        sys.stdout.write(parts[0])
    start, goal = list(p.starts), list(p.goals)
    r = heuristic(p.graph, list(p.starts), list(p.goals))
    paths = getattr(r, "paths", r)
    text = getattr(r, "log", "") or ""
    parts.append(text)
    if echo and text:
        sys.stdout.write(text)
    if paths is not None:
        t = print_paths(p.graph, paths, compat)
        err = path_set_error(paths, p.graph, start, goal, compat)
        t += "Path set is valid.\n" if err is None else err + "\n"
        parts.append(t)
        if echo:
            sys.stdout.write(t)
    return "".join(parts)
