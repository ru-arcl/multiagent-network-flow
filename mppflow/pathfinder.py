"""Single-robot searches, distances and lower bounds (port of ``ILP/PathFinder.java``).

* :func:`get_distances_to_all_vertices` -- BFS hop distances (no randomness).
* :func:`find_goals_in_distance_range` -- vertices in a BFS layer range, in Java
  ``HashSet`` iteration order (it drives the k-way split's middle-vertex choice).
* :func:`find_shortest_paths` / :func:`find_one_shortest_path` -- BFS with a random
  neighbour order, consuming ``Math.random()`` draws exactly as the original.
* :func:`get_makespan_lower_bound`, :func:`get_total_time_lower_bound`,
  :func:`get_total_distance`, :func:`get_total_path_length`.

Randomized BFS, exactly as the original (both modes)
-----------------------------------------------------
``find_one_shortest_path(g, s, t)`` returns ``[s]`` without drawing when ``s == t``.
Otherwise it runs a FIFO search from ``s``; ``s`` is **not** marked visited, so it is
rediscovered by its first expanded neighbour, re-enqueued and expanded a second time.
Expanding ``v`` takes its non-self neighbours in iteration order and removes them in
random order, one draw ``(int)(len * R)`` per removal; an unvisited neighbour gets its
parent set and is enqueued (and marked visited), and the search stops right after the draw
that picks the goal. The result is a shortest path; the draws only break ties. We keep the
start re-expansion in both modes because it determines how many draws are consumed, and
therefore every later random instance and tie-break.

With ``heuristic=True`` (the original's global ``PathFinder.bHeuristic``) each enqueue
inserts at a random position ``(int)(R * len(queue))`` (one more draw, the root
included), so the search is no longer breadth-first and paths may be longer.

compat modes
------------
Differences of ``compat="fixed"`` from the original:

* ``heuristic`` is an explicit parameter (default ``False``). In ``original`` mode,
  ``heuristic=None`` (the default) reads the module-level flag :data:`B_HEURISTIC`,
  which emulates the original's global ``bHeuristic`` (``Main.solveProblemWithHeuristic``
  sets it and never resets it, so it leaks into later exact solves).
* :func:`get_makespan_lower_bound` / :func:`get_total_time_lower_bound` raise
  :class:`~mppflow.errors.Infeasible` when a goal is unreachable (``original``:
  :class:`~mppflow.errors.JavaNullPointerException`). The draws consumed before that
  point are the same.
* :func:`get_distances_to_all_vertices` treats a vertex without an adjacency entry as
  having no neighbours and raises ``ValueError`` for a start outside the array
  (``original``: ``NullPointerException`` / ``ArrayIndexOutOfBoundsException``).
* :func:`find_one_shortest_path` does not mutate the caller's ``already_visited`` set
  (the original adds every discovered vertex to it).

Unchanged in both modes (documented quirks, not bugs): unreachable vertices keep distance
0 (or the caller's prefilled value) in :func:`get_distances_to_all_vertices`, as the
reachability pruning of the models expects (use ``unreachable=`` to choose another
value); :func:`get_makespan_lower_bound` returns the longest shortest path as a **vertex
count** (length + 1), which every caller in the original compensates with ``- 1``.
"""
from __future__ import annotations

from typing import Sequence

from ._java import JavaIntHashSet, MathRandom, resolve_rng
from .errors import (
    Infeasible,
    JavaArrayIndexOutOfBoundsException,
    JavaNullPointerException,
    OriginalWouldHang,
)
from .graph import Graph, check_compat

__all__ = [
    "GlobalFlag",
    "B_HEURISTIC",
    "get_distances_to_all_vertices",
    "find_goals_in_distance_range",
    "find_shortest_paths",
    "find_one_shortest_path",
    "add_to_queue",
    "de_queue",
    "get_total_distance",
    "get_total_path_length",
    "get_makespan_lower_bound",
    "get_total_time_lower_bound",
]


class GlobalFlag:
    """A mutable boolean standing in for a Java ``static boolean`` field."""

    __slots__ = ("value",)

    def __init__(self, value: bool = False):
        self.value = bool(value)

    def __bool__(self) -> bool:
        return self.value

    def set(self, value: bool) -> None:
        self.value = bool(value)

    def __repr__(self) -> str:
        return f"GlobalFlag({self.value})"


B_HEURISTIC = GlobalFlag(False)
"""Emulation of the original's global ``PathFinder.bHeuristic``; read only in
``compat="original"`` when ``heuristic`` is ``None``."""


def _resolve_heuristic(heuristic: bool | None, orig: bool) -> bool:
    if heuristic is not None:
        return bool(heuristic)
    return bool(B_HEURISTIC) if orig else False


# ----------------------------------------------------------------------- distances
def get_distances_to_all_vertices(g: Graph, start: int, input_distances: list[int] | None = None,
                                  compat: str = "fixed", *, unreachable: int | None = None) -> list[int]:
    """``getDistancesToAllVertices(g, start, inputDistances)``: BFS hop distances from
    ``start`` to every vertex ``0..n-1``.

    Writes into ``input_distances`` (in place, and returns the same list) when given,
    else into a fresh list of ``n`` zeros. Unreachable vertices keep their prior value --
    0 for a fresh list, as in the original -- unless ``unreachable`` is given, in which
    case they are set to it. No randomness.

    Differs from the original: ``fixed`` skips vertices without an adjacency entry and
    raises ``ValueError`` for an out-of-range start/neighbour (``original``:
    :class:`JavaNullPointerException` / :class:`JavaArrayIndexOutOfBoundsException`).
    """
    orig = check_compat(compat)
    dist = input_distances if input_distances is not None else [0] * g.n
    size = len(dist)

    def oob(i):
        if orig:
            raise JavaArrayIndexOutOfBoundsException(f"Index {i} out of bounds for length {size}")
        raise ValueError(f"vertex {i} out of range 0..{size - 1}")

    if not 0 <= start < size:
        oob(start)
    dist[start] = 0
    visited: set[int] = set()
    queue = [start]
    head = 0
    while head < len(queue):
        v = queue[head]
        head += 1
        nb = g.neighbors(v)
        if nb is None:
            if orig:
                raise JavaNullPointerException(
                    'Cannot invoke "java.util.Set.toArray(Object[])" because the return value of "java.util.Map.get(Object)" is null')
            continue
        dv = dist[v] + 1
        for u in nb:
            if not 0 <= u < size:
                oob(u)  # first sight of u (an out-of-range id is never marked visited)
            if u not in visited:
                visited.add(u)
                queue.append(u)
                if u != start:
                    dist[u] = dv
            elif dist[u] > dv:  # pragma: no cover - never fires for FIFO BFS; kept for fidelity
                dist[u] = dv
    if unreachable is not None:
        for i in range(size):
            if i != start and i not in visited:
                dist[i] = unreachable
    return dist


def find_goals_in_distance_range(g: Graph, start: int, d_min: int, d_max: int,
                                 compat: str = "fixed") -> list[int]:
    """``findGoalsInDistanceRange(g, start, dMin, dMax)``: the vertices whose BFS layer
    ``d`` satisfies ``max(d_min, 1) <= d <= d_max`` (``start`` is never included).

    Layer-synchronous BFS ignoring self-loops; it stops early once ``n`` vertices have
    been visited. Returns the result **in the iteration order of the original's
    ``HashSet<Integer>``** (the k-way split takes the first suitable element).
    Identical in both modes; no randomness.
    """
    check_compat(compat)
    vset = JavaIntHashSet()
    visited = JavaIntHashSet([start])
    current = JavaIntHashSet([start])
    tobe = JavaIntHashSet()
    distance = 1
    n = g.n
    while distance <= d_max and len(visited) < n:
        for v in current.to_list():
            nb = g.neighbors(v)
            if nb is None:
                continue
            for u in list(nb):
                if u != v and u not in visited:
                    tobe.add(u)
                    if distance >= d_min:
                        vset.add(u)
        distance += 1
        current = tobe
        tobe = JavaIntHashSet()
        for u in current:
            visited.add(u)
    return vset.to_list()


# ------------------------------------------------------------- randomized BFS
def add_to_queue(queue_map: dict[int, list[int]], v_id: int, rng: MathRandom | None = None,
                 heuristic: bool = False) -> None:
    """``addToQueue``: append ``v_id`` to the list of key 0 (the "priority" is the
    constant 0); with ``heuristic`` insert it at ``(int)(R * len(list))`` instead (one
    draw, also when the list is new and empty)."""
    lst = queue_map.get(0)
    if lst is None:
        lst = []
        queue_map[0] = lst
    if not heuristic:
        lst.append(v_id)
    else:
        rng = resolve_rng(rng)
        lst.insert(int(rng.random() * len(lst)), v_id)


def de_queue(queue_map: dict[int, list[int]]) -> int:
    """``deQueue``: pop the head of the smallest key's list, dropping the key when its list
    becomes empty."""
    k = min(queue_map)
    lst = queue_map[k]
    v = lst.pop(0)
    if not lst:
        del queue_map[k]
    return v


def find_one_shortest_path(g: Graph, start: int, goal: int, already_visited: set[int] | None = None,
                           rng: MathRandom | None = None, *, heuristic: bool | None = None,
                           compat: str = "fixed") -> list[int] | None:
    """``findOneShortestPath(g, start, goal, alreadyVisitedVertices)``: a random shortest
    path ``[start, ..., goal]``, or ``None`` when ``goal`` is unreachable.

    Draw consumption is described in the module docstring (identical in both modes).
    ``already_visited`` pre-marks vertices as visited (never used by the original's
    callers).

    Differs from the original: ``heuristic`` is explicit (``original`` with ``None``
    reads :data:`B_HEURISTIC`); ``fixed`` copies ``already_visited`` instead of adding to
    it. The original's path recovery would spin forever on a missing parent, which cannot
    happen: every vertex other than ``start`` gets its parent once, when it is first
    discovered, from a vertex dequeued (hence discovered) before it, so the parent chain
    from ``goal`` reaches ``start``. The guard that raises :class:`OriginalWouldHang`
    there is therefore unreachable and untested.
    """
    orig = check_compat(compat)
    h = _resolve_heuristic(heuristic, orig)
    if start == goal:
        return [start]
    rng = resolve_rng(rng)
    if already_visited is None:
        visited: set[int] = set()
    else:
        visited = already_visited if orig else set(already_visited)
    queue_map: dict[int, list[int]] = {}
    parent: dict[int, int] = {}
    add_to_queue(queue_map, start, rng, h)
    found = False
    while queue_map:
        v = de_queue(queue_map)
        nb = g.neighbors(v)
        if nb is None:
            continue
        nvec = [u for u in nb if u != v]
        while nvec:
            idx = int(len(nvec) * rng.random())
            u = nvec.pop(idx)
            if u not in visited:
                parent[u] = v
                if u == goal:
                    queue_map.clear()
                    found = True
                    break
                add_to_queue(queue_map, u, rng, h)
                visited.add(u)
    if not found:
        return None
    path = [goal]
    last = goal
    steps = 0
    limit = len(parent) + 1
    while True:
        par = parent.get(last)
        if par is None or steps > limit:  # pragma: no cover - unreachable, see the docstring
            raise OriginalWouldHang("findOneShortestPath: path recovery loops forever (missing parent)")
        path.append(par)
        last = par
        steps += 1
        if last == start:
            break
    path.reverse()
    return path


def find_shortest_paths(g: Graph, starts: Sequence[int], goals: Sequence[int],
                        rng: MathRandom | None = None, *, heuristic: bool | None = None,
                        compat: str = "fixed") -> list[list[int] | None]:
    """``findShortestPaths(g, start, goal)``: :func:`find_one_shortest_path` for each
    robot in index order (so the draws are consumed robot by robot). An entry is ``None``
    when that robot's goal is unreachable."""
    orig = check_compat(compat)
    h = _resolve_heuristic(heuristic, orig)
    rng = resolve_rng(rng)
    return [find_one_shortest_path(g, starts[i], goals[i], None, rng, heuristic=h, compat=compat)
            for i in range(len(starts))]


# ---------------------------------------------------------------- metrics, bounds
def get_total_distance(paths: Sequence[Sequence[int]]) -> int:
    """``getTotalDistance(int[][] paths)``: the number of moves, i.e. of ``(a, t)`` with
    ``paths[a][t] != paths[a][t+1]`` (waits are not counted)."""
    d = 0
    for p in paths:
        for i in range(len(p) - 1):
            if p[i] != p[i + 1]:
                d += 1
    return d


def get_total_path_length(paths: Sequence[Sequence[int]]) -> int:
    """``getTotalDistance(Path[] hp)`` (*dead* overload): ``sum(len(path) - 1)``, which
    counts waits too."""
    return sum(len(p) - 1 for p in paths)


def _unreachable(orig: bool, i: int):
    if orig:
        raise JavaNullPointerException('Cannot read field "vertexList" because "<local3>[<local5>]" is null')
    raise Infeasible(f"goal of robot {i} is unreachable from its start")


def get_makespan_lower_bound(g: Graph, starts: Sequence[int], goals: Sequence[int],
                             rng: MathRandom | None = None, *, heuristic: bool | None = None,
                             compat: str = "fixed") -> int:
    """``getMakespanLowerBound(g, start, goal)``: the maximum over robots of the
    **number of vertices** on a shortest path (= makespan lower bound + 1; 0 for no
    robots). Calls :func:`find_shortest_paths`, so it consumes draws.

    Differs from the original: an unreachable goal raises :class:`Infeasible`
    (``original``: :class:`JavaNullPointerException`, as the Java).
    """
    orig = check_compat(compat)
    paths = find_shortest_paths(g, starts, goals, rng, heuristic=heuristic, compat=compat)
    lb = 0
    for i, p in enumerate(paths):
        if p is None:
            _unreachable(orig, i)
        if len(p) > lb:
            lb = len(p)
    return lb


def get_total_time_lower_bound(g: Graph, starts: Sequence[int], goals: Sequence[int],
                               rng: MathRandom | None = None, *, heuristic: bool | None = None,
                               compat: str = "fixed") -> int:
    """``getTotalTimeLowerBound(g, start, goal)``: the sum of shortest-path lengths (also
    used by the original as the total-distance lower bound). Consumes draws.

    Differs from the original: an unreachable goal raises :class:`Infeasible`
    (``original``: :class:`JavaNullPointerException`).
    """
    orig = check_compat(compat)
    paths = find_shortest_paths(g, starts, goals, rng, heuristic=heuristic, compat=compat)
    tt = 0
    for i, p in enumerate(paths):
        if p is None:
            _unreachable(orig, i)
        tt += len(p) - 1
    return tt
