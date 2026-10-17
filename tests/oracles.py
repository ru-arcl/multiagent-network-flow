"""Independent oracles for multi-robot path planning on graphs (MPP) on tiny instances.

Everything here is written from the problem definition of our T-RO 2016 paper (Yu and LaValle),
Sec. II-A, and does **not** import or reuse any ``mppflow`` code:

* A graph is an undirected simple graph given as neighbour lists ``nbrs[v]`` (without
  ``v`` itself; every robot may always stay where it is).
* Wait loops (optional, keyword ``loops`` of the search functions): the set of vertices
  where a robot may stay; ``None`` (the default) is every vertex. A robot on any other
  vertex must move in every step. In the total arrival time, a robot "stays at its goal
  until ``T``" only from the last step if its goal has no wait loop, so it arrives at ``T``
  (or at 0 in the plan with no steps).
* A plan is ``paths[i][t]``, ``t = 0..T``. In one step every robot either stays or moves
  along one edge. Two robots collide when they are on the same vertex at the same time
  (meet), or when they traverse the same edge in opposite directions in the same step
  (head-on swap). Nothing else is forbidden, so robots may rotate along a cycle of three
  or more vertices, even a fully occupied one, and a robot may follow another into the
  vertex it is just leaving.
* Objectives of a plan (robots end at their goals at ``T``):
  - makespan ``T``;
  - total arrival time ``sum_i t_i``, where ``t_i`` is the first time from which robot
    ``i`` stays at its goal until ``T`` (``0`` when it never leaves it);
  - total distance: the number of moves (waits are free);
  - max distance: the largest number of moves of any robot.

Oracles (exact; exponential in the number of robots, meant for tiny instances):

* :func:`min_makespan` -- BFS over joint configurations.
* :func:`min_total_time`, :func:`min_total_distance`, :func:`min_max_distance` -- the
  optimum over **all** horizons (Dijkstra / BFS over joint configurations, with extra state
  where the objective needs it); ``*_at_horizon`` -- the optimum among plans of exactly
  ``T`` steps (dynamic programming over ``(configuration, t)``).
* :func:`full_network_ilp` -- an ILP on the paper's full time-expanded network (Fig. 5:
  vertex arcs ``v(t) -> v(t)'`` of capacity 1, holdover arcs ``v(t)' -> v(t+1)`` and one
  gadget ``u(t)', v(t)' -> w1 -> w2 -> u(t+1), v(t+1)`` per edge, with capacity 1 on
  ``w1 -> w2``), multi-commodity with one commodity per robot (gurobipy or scipy).
* :func:`validate_plan` -- a plan checker that returns the first violation.

All functions return ``None`` for "infeasible".
"""
from __future__ import annotations

import heapq
import itertools
from collections import deque
from typing import Iterable, Sequence

INF = float("inf")


# ------------------------------------------------------------------ graphs
def normalize(nbrs: Sequence[Iterable[int]]) -> list[list[int]]:
    """Symmetric, loop-free, sorted neighbour lists."""
    n = len(nbrs)
    s = [set() for _ in range(n)]
    for u in range(n):
        for v in nbrs[u]:
            if v != u:
                s[u].add(v)
                s[v].add(u)
    return [sorted(x) for x in s]


def path_graph(n: int) -> list[list[int]]:
    return normalize([[v + 1] if v + 1 < n else [] for v in range(n)])


def cycle_graph(n: int) -> list[list[int]]:
    return normalize([[(v + 1) % n] for v in range(n)])


def star_graph(leaves: int) -> list[list[int]]:
    return normalize([list(range(1, leaves + 1))] + [[] for _ in range(leaves)])


def random_tree(n: int, rnd) -> list[list[int]]:
    return normalize([[] if v == 0 else [rnd.randrange(v)] for v in range(n)])


def grid_graph(rows: int, cols: int, holes: Iterable[int] = ()) -> list[list[int]]:
    """4-connected ``rows x cols`` grid (cell ``r*cols + c``) without the ``holes``,
    relabelled ``0..n-1`` in increasing cell order."""
    holes = set(holes)
    cells = [c for c in range(rows * cols) if c not in holes]
    idx = {c: i for i, c in enumerate(cells)}
    out = [[] for _ in cells]
    for c in cells:
        r, k = divmod(c, cols)
        for rr, kk in ((r + 1, k), (r, k + 1)):
            if rr < rows and kk < cols and rr * cols + kk in idx:
                out[idx[c]].append(idx[rr * cols + kk])
    return normalize(out)


def bfs_distances(nbrs: Sequence[Sequence[int]], src: int) -> list[float]:
    d = [INF] * len(nbrs)
    d[src] = 0
    q = deque([src])
    while q:
        u = q.popleft()
        for v in nbrs[u]:
            if d[v] == INF:
                d[v] = d[u] + 1
                q.append(v)
    return d


def goal_distances(nbrs, goals) -> list[list[float]]:
    """``D[i][v]`` = distance from ``v`` to robot ``i``'s goal."""
    return [bfs_distances(nbrs, g) for g in goals]


# --------------------------------------------------------------- validator
def validate_plan(nbrs: Sequence[Sequence[int]], starts: Sequence[int], goals: Sequence[int],
                  paths, *, loops=None) -> str | None:
    """``None`` if ``paths`` is a collision-free plan from ``starts`` to ``goals`` (waiting
    only on the vertices of ``loops``, if given), else a description of the first violation."""
    A = len(starts)
    if paths is None:
        return "no plan"
    if len(paths) != A:
        return f"{len(paths)} paths for {A} robots"
    if A == 0:
        return None
    L = len(paths[0])
    if L < 1:
        return "empty path"
    adj = [set(x) for x in nbrs]
    for i, p in enumerate(paths):
        if len(p) != L:
            return f"path {i} has length {len(p)}, path 0 has {L}"
        if any(not (isinstance(v, int) and 0 <= v < len(nbrs)) for v in p):
            return f"path {i} has an invalid vertex"
        if p[0] != starts[i]:
            return f"robot {i} starts at {p[0]}, not {starts[i]}"
        if p[-1] != goals[i]:
            return f"robot {i} ends at {p[-1]}, not {goals[i]}"
        for t in range(L - 1):
            if p[t] != p[t + 1] and p[t + 1] not in adj[p[t]]:
                return f"robot {i} jumps {p[t]} -> {p[t + 1]} at t={t}"
            if p[t] == p[t + 1] and loops is not None and p[t] not in loops:
                return f"robot {i} waits at {p[t]} (no wait loop) at t={t}"
    for t in range(L):
        seen = {}
        for i in range(A):
            v = paths[i][t]
            if v in seen:
                return f"robots {seen[v]} and {i} meet at {v} at t={t}"
            seen[v] = i
    for t in range(L - 1):
        for i in range(A):
            for j in range(i + 1, A):
                if (paths[i][t] == paths[j][t + 1] and paths[i][t + 1] == paths[j][t]
                        and paths[i][t] != paths[i][t + 1]):
                    return f"robots {i} and {j} swap along {paths[i][t]}-{paths[i][t + 1]} at t={t}"
    return None


def plan_metrics(paths, goals) -> dict:
    """makespan, total_time, total_distance, max_distance of a (valid) plan."""
    if not paths:
        return {"makespan": 0, "total_time": 0, "total_distance": 0, "max_distance": 0}
    T = len(paths[0]) - 1
    tt = 0
    moves = []
    for p, g in zip(paths, goals):
        t = T
        while t > 0 and p[t - 1] == g:
            t -= 1
        tt += t
        moves.append(sum(1 for k in range(T) if p[k] != p[k + 1]))
    return {"makespan": T, "total_time": tt, "total_distance": sum(moves), "max_distance": max(moves)}


# ----------------------------------------------------------- joint moves
def joint_successors(nbrs, cfg: tuple, frozen: frozenset = frozenset(), loops=None):
    """All configurations reachable from ``cfg`` in one collision-free step
    (``frozen`` robots must stay; a robot may stay only on a vertex of ``loops``, any vertex
    if ``None``). Yields ``(next_cfg, n_moved, moved_mask)``."""
    A = len(cfg)
    where = {v: i for i, v in enumerate(cfg)}
    nxt = [None] * A
    used = set()

    def rec(i, moved, mask):
        if i == A:
            yield tuple(nxt), moved, mask
            return
        u = cfg[i]
        stay = [u] if loops is None or u in loops else []
        opts = stay if i in frozen else stay + list(nbrs[u])
        for w in opts:
            if w in used:  # meet collision at t+1
                continue
            if w != u:
                j = where.get(w)
                # head-on: robot j (on w) goes to u while i goes from u to w
                if j is not None and j < i and nxt[j] == u:
                    continue
            nxt[i] = w
            used.add(w)
            yield from rec(i + 1, moved + (w != u), mask | ((w != u) << i))
            used.discard(w)
        nxt[i] = None

    yield from rec(0, 0, 0)


def _check_instance(nbrs, starts, goals):
    A = len(starts)
    if len(goals) != A:
        raise ValueError("starts/goals length")
    if len(set(starts)) < A:
        raise ValueError("duplicate starts")


def _trivially_infeasible(nbrs, starts, goals):
    if len(set(goals)) < len(goals):
        return True
    for s, g in zip(starts, goals):
        if bfs_distances(nbrs, s)[g] == INF:
            return True
    return False


def _reconstruct(parent, end):
    seq = [end]
    while parent[seq[-1]] is not None:
        seq.append(parent[seq[-1]])
    seq.reverse()
    return seq


def _cfgs_to_paths(cfgs, A):
    return [[c[i] for c in cfgs] for i in range(A)]


# ----------------------------------------------------------------- makespan
def min_makespan(nbrs, starts, goals, max_states: int = 2_000_000, *, loops=None):
    """``(T*, paths)`` with ``T*`` the optimal makespan, or ``None`` (infeasible)."""
    _check_instance(nbrs, starts, goals)
    A = len(starts)
    s, g = tuple(starts), tuple(goals)
    if s == g:
        return 0, [[v] for v in starts]
    if _trivially_infeasible(nbrs, starts, goals):
        return None
    parent = {s: None}
    depth = {s: 0}
    q = deque([s])
    while q:
        c = q.popleft()
        for n2, _, _ in joint_successors(nbrs, c, loops=loops):
            if n2 in parent:
                continue
            parent[n2] = c
            depth[n2] = depth[c] + 1
            if n2 == g:
                return depth[n2], _cfgs_to_paths(_reconstruct(parent, n2), A)
            if len(parent) > max_states:
                raise RuntimeError("state budget exceeded")
            q.append(n2)
    return None


def reachable_configs(nbrs, starts, limit: int | None = None, *, loops=None) -> int:
    """Number of joint configurations reachable from ``starts`` (for sizing); the count
    stops at ``limit + 1`` when ``limit`` is given."""
    s = tuple(starts)
    seen = {s}
    q = deque([s])
    while q:
        c = q.popleft()
        for n2, _, _ in joint_successors(nbrs, c, loops=loops):
            if n2 not in seen:
                seen.add(n2)
                if limit is not None and len(seen) > limit:
                    return len(seen)
                q.append(n2)
    return len(seen)


# ------------------------------------------------------ total distance (all T)
def min_total_distance(nbrs, starts, goals, *, loops=None):
    """``(opt, paths)``: minimum number of moves over plans of any length (Dijkstra on
    configurations, step cost = number of robots that move), or ``None``."""
    _check_instance(nbrs, starts, goals)
    A = len(starts)
    s, g = tuple(starts), tuple(goals)
    if s == g:
        return 0, [[v] for v in starts]
    if _trivially_infeasible(nbrs, starts, goals):
        return None
    D = goal_distances(nbrs, goals)
    best = {s: 0}
    parent = {s: None}
    done = set()
    tie = itertools.count()
    pq = [(sum(D[i][s[i]] for i in range(A)), 0, next(tie), s)]
    while pq:
        _, d, _, c = heapq.heappop(pq)
        if c in done:
            continue
        done.add(c)
        if c == g:
            return d, _cfgs_to_paths(_reconstruct(parent, c), A)
        for n2, k, _ in joint_successors(nbrs, c, loops=loops):
            if k == 0:
                continue
            nd = d + k
            if nd < best.get(n2, INF):
                best[n2] = nd
                parent[n2] = c
                h = sum(D[i][n2[i]] for i in range(A))  # admissible: each robot needs >= D moves
                heapq.heappush(pq, (nd + h, nd, next(tie), n2))
    return None


# ----------------------------------------------------- total arrival time (all T)
def min_total_time(nbrs, starts, goals, *, loops=None):
    """``(opt, paths)``: minimum total arrival time over plans of any length, or ``None``.

    Dijkstra over ``(configuration, F)``, ``F`` = robots that have arrived for good (they
    sit at their goals from now on). A step costs the number of robots not in ``F``; a robot
    may join ``F`` whenever it is at its goal (and the goal has a wait loop); the search ends
    when every robot is at its goal."""
    _check_instance(nbrs, starts, goals)
    A = len(starts)
    s, g = tuple(starts), tuple(goals)
    if _trivially_infeasible(nbrs, starts, goals):
        return None
    full = (1 << A) - 1
    D = goal_distances(nbrs, goals)

    def closures(cfg, F):
        at = [i for i in range(A) if cfg[i] == goals[i] and not (F >> i) & 1
              and (loops is None or goals[i] in loops)]
        for r in range(len(at) + 1):
            for sub in itertools.combinations(at, r):
                m = F
                for i in sub:
                    m |= 1 << i
                yield m

    def h(cfg, F):
        return sum(D[i][cfg[i]] for i in range(A) if not (F >> i) & 1)

    tie = itertools.count()
    best = {}
    parent = {}
    pq = []
    for F0 in closures(s, 0):
        st = (s, F0)
        best[st] = 0
        parent[st] = None
        heapq.heappush(pq, (h(s, F0), 0, next(tie), st))
    done = set()
    while pq:
        _, d, _, st = heapq.heappop(pq)
        if st in done:
            continue
        done.add(st)
        c, F = st
        if F == full or (loops is not None and c == g):
            cfgs = [x[0] for x in _reconstruct(parent, st)]
            return d, _cfgs_to_paths(cfgs, A)
        frozen = frozenset(i for i in range(A) if (F >> i) & 1)
        cost = A - len(frozen)
        for n2, _, _ in joint_successors(nbrs, c, frozen, loops):
            for F2 in closures(n2, F):
                st2 = (n2, F2)
                nd = d + cost
                if nd < best.get(st2, INF):
                    best[st2] = nd
                    parent[st2] = st
                    heapq.heappush(pq, (nd + h(n2, F2), nd, next(tie), st2))
    return None


# ------------------------------------------------------- max distance (all T)
def _max_distance_feasible(nbrs, starts, goals, Dmax, D, loops=None):
    """A plan where every robot moves at most ``Dmax`` times, or ``None``."""
    A = len(starts)
    s, g = tuple(starts), tuple(goals)
    st0 = (s, (0,) * A)
    parent = {st0: None}
    q = deque([st0])
    while q:
        st = q.popleft()
        c, m = st
        if c == g:
            return _cfgs_to_paths([x[0] for x in _reconstruct(parent, st)], A)
        for n2, k, mask in joint_successors(nbrs, c, loops=loops):
            if k == 0:
                continue
            m2 = tuple(m[i] + ((mask >> i) & 1) for i in range(A))
            if any(m2[i] + D[i][n2[i]] > Dmax for i in range(A)):
                continue
            st2 = (n2, m2)
            if st2 not in parent:
                parent[st2] = st
                q.append(st2)
    return None


def min_max_distance(nbrs, starts, goals, upper: int | None = None, *, loops=None):
    """``(opt, paths)``: minimum over plans of any length of the largest number of moves of
    a robot, or ``None``. ``upper`` (e.g. the optimal makespan, which is always an upper
    bound) ends the search; it is computed when not given."""
    _check_instance(nbrs, starts, goals)
    A = len(starts)
    if tuple(starts) == tuple(goals):
        return 0, [[v] for v in starts]
    if _trivially_infeasible(nbrs, starts, goals):
        return None
    if upper is None:
        r = min_makespan(nbrs, starts, goals, loops=loops)
        if r is None:
            return None
        upper = r[0]
    D = goal_distances(nbrs, goals)
    lo = max(D[i][starts[i]] for i in range(A))
    for Dm in range(int(lo), int(upper) + 1):
        p = _max_distance_feasible(nbrs, starts, goals, Dm, D, loops)
        if p is not None:
            return Dm, p
    raise AssertionError("no plan within the makespan bound")  # cannot happen


# ------------------------------------------------------------ fixed horizon
def min_total_distance_at_horizon(nbrs, starts, goals, T: int, *, loops=None):
    """``(opt, paths)``: fewest moves among plans of exactly ``T`` steps, or ``None``."""
    _check_instance(nbrs, starts, goals)
    A = len(starts)
    s, g = tuple(starts), tuple(goals)
    if len(set(goals)) < A:
        return None
    D = goal_distances(nbrs, goals)
    if any(D[i][starts[i]] > T for i in range(A)):
        return None
    layers = [{s: (0, None)}]
    for t in range(T):
        cur = layers[-1]
        nxt: dict = {}
        rem = T - t - 1
        for c, (d, _) in cur.items():
            for n2, k, _ in joint_successors(nbrs, c, loops=loops):
                if any(D[i][n2[i]] > rem for i in range(A)):
                    continue
                nd = d + k
                if n2 not in nxt or nd < nxt[n2][0]:
                    nxt[n2] = (nd, c)
        layers.append(nxt)
    if g not in layers[T]:
        return None
    cfgs = [g]
    for t in range(T, 0, -1):
        cfgs.append(layers[t][cfgs[-1]][1])
    cfgs.reverse()
    return layers[T][g][0], _cfgs_to_paths(cfgs, A)


def min_total_time_at_horizon(nbrs, starts, goals, T: int, *, loops=None):
    """``(opt, paths)``: least total arrival time among plans of exactly ``T`` steps."""
    _check_instance(nbrs, starts, goals)
    A = len(starts)
    s, g = tuple(starts), tuple(goals)
    if len(set(goals)) < A:
        return None
    D = goal_distances(nbrs, goals)
    if any(D[i][starts[i]] > T for i in range(A)):
        return None
    full = (1 << A) - 1

    def closures(cfg, F):
        at = [i for i in range(A) if cfg[i] == goals[i] and not (F >> i) & 1
              and (loops is None or goals[i] in loops)]
        for r in range(len(at) + 1):
            for sub in itertools.combinations(at, r):
                m = F
                for i in sub:
                    m |= 1 << i
                yield m

    layers = [{(s, F0): (0, None) for F0 in closures(s, 0)}]
    for t in range(T):
        cur = layers[-1]
        nxt: dict = {}
        rem = T - t - 1
        for st, (d, _) in cur.items():
            c, F = st
            frozen = frozenset(i for i in range(A) if (F >> i) & 1)
            cost = A - len(frozen)
            for n2, _, _ in joint_successors(nbrs, c, frozen, loops):
                if any(D[i][n2[i]] > rem for i in range(A)):
                    continue
                for F2 in closures(n2, F):
                    st2 = (n2, F2)
                    nd = d + cost
                    if st2 not in nxt or nd < nxt[st2][0]:
                        nxt[st2] = (nd, st)
        layers.append(nxt)
    ends = [st for st in layers[T] if st[0] == g] if loops is not None else [(g, full)]
    ends = [st for st in ends if st in layers[T]]
    if not ends:
        return None
    end = min(ends, key=lambda st: layers[T][st][0])
    sts = [end]
    for t in range(T, 0, -1):
        sts.append(layers[t][sts[-1]][1])
    sts.reverse()
    return layers[T][end][0], _cfgs_to_paths([x[0] for x in sts], A)


def min_max_distance_at_horizon(nbrs, starts, goals, T: int, *, loops=None):
    """``(opt, paths)``: least max-moves among plans of exactly ``T`` steps."""
    _check_instance(nbrs, starts, goals)
    A = len(starts)
    s, g = tuple(starts), tuple(goals)
    if len(set(goals)) < A:
        return None
    D = goal_distances(nbrs, goals)
    if any(D[i][starts[i]] > T for i in range(A)):
        return None
    lo = int(max(D[i][starts[i]] for i in range(A))) if A else 0
    for Dm in range(lo, T + 1):
        layer = {(s, (0,) * A): None}
        hist = [layer]
        for t in range(T):
            rem = T - t - 1
            nxt = {}
            for st in layer:
                c, m = st
                for n2, _, mask in joint_successors(nbrs, c, loops=loops):
                    if any(D[i][n2[i]] > rem for i in range(A)):
                        continue
                    m2 = tuple(m[i] + ((mask >> i) & 1) for i in range(A))
                    if any(m2[i] + D[i][n2[i]] > Dm for i in range(A)):
                        continue
                    st2 = (n2, m2)
                    if st2 not in nxt:
                        nxt[st2] = st
            layer = nxt
            hist.append(layer)
        ends = [st for st in layer if st[0] == g]
        if ends:
            sts = [ends[0]]
            for t in range(T, 0, -1):
                sts.append(hist[t][sts[-1]])
            sts.reverse()
            return Dm, _cfgs_to_paths([x[0] for x in sts], A)
    return None


def feasible_at_horizon(nbrs, starts, goals, T: int, *, loops=None) -> bool:
    return min_total_distance_at_horizon(nbrs, starts, goals, T, loops=loops) is not None


# ---------------------------------------------------- full-network ILP (Fig. 5)
def full_network_ilp(nbrs, starts, goals, T: int, objective: str = "feasible", backend: str | None = None):
    """Solve the paper's full time-expanded network (Fig. 5) at horizon ``T``.

    Nodes: ``v(t)`` and ``v(t)'`` for ``t = 0..T``; arcs: ``v(t) -> v(t)'`` (capacity 1),
    ``v(t)' -> v(t+1)`` (holdover), and per edge ``{u, v}`` and ``t < T`` the gadget
    ``u(t)' -> w1, v(t)' -> w1, w1 -> w2`` (capacity 1), ``w2 -> u(t+1), w2 -> v(t+1)``.
    Commodity ``i`` sends one unit from ``s_i(0)`` to ``g_i(T)'``. ``objective``:
    ``"feasible"`` (returns paths or ``None``), ``"distance"`` (number of gadget middle
    arcs whose entry and exit differ, i.e. real moves) -> ``(value, paths)``.

    The gadget lets a robot enter and leave on the same side (``u -> w1 -> w2 -> u``); that
    is a wait which blocks the edge, harmless for both objectives (it is never better than
    the holdover arc, and moves are counted by entry != exit).
    """
    A = len(starts)
    n = len(nbrs)
    edges = sorted({(min(u, v), max(u, v)) for u in range(n) for v in nbrs[u]})
    arcs = []  # (tail, head, group or None, cost-move)
    # node names
    def V(v, t):
        return ("v", v, t)

    def Vp(v, t):
        return ("vp", v, t)

    cap_groups = []
    for t in range(T + 1):
        for v in range(n):
            arcs.append((V(v, t), Vp(v, t), ("cap", v, t), None))
    for t in range(T):
        for v in range(n):
            arcs.append((Vp(v, t), V(v, t + 1), None, None))
        for (u, v) in edges:
            w1, w2 = ("w1", u, v, t), ("w2", u, v, t)
            arcs.append((Vp(u, t), w1, None, ("in", u)))
            arcs.append((Vp(v, t), w1, None, ("in", v)))
            arcs.append((w1, w2, ("mid", u, v, t), None))
            arcs.append((w2, V(u, t + 1), None, ("out", u)))
            arcs.append((w2, V(v, t + 1), None, ("out", v)))
    nodes = sorted({a[0] for a in arcs} | {a[1] for a in arcs}, key=repr)
    nid = {x: k for k, x in enumerate(nodes)}
    groups = {}
    for k, a in enumerate(arcs):
        if a[2] is not None:
            groups.setdefault(a[2], []).append(k)
    # move indicator: per robot, per gadget, entry side != exit side; model with extra
    # binaries m[i,gadget] >= in_u + out_v - 1 and >= in_v + out_u - 1.
    gadget_arcs = {}
    for k, a in enumerate(arcs):
        if a[3] is not None:
            key = a[0][1:] if a[0][0] == "w2" else a[1][1:]
            gadget_arcs.setdefault(key, {})[a[3]] = k
    nA = len(arcs)
    nG = len(gadget_arcs)
    gkeys = sorted(gadget_arcs)
    nvar = A * nA + (A * nG if objective == "distance" else 0)

    def x(i, k):
        return i * nA + k

    def m(i, gi):
        return A * nA + i * nG + gi

    rows, lo, hi = [], [], []
    # conservation
    for i in range(A):
        src, snk = V(starts[i], 0), Vp(goals[i], T)
        out_of = [[] for _ in nodes]
        into = [[] for _ in nodes]
        for k, a in enumerate(arcs):
            out_of[nid[a[0]]].append(k)
            into[nid[a[1]]].append(k)
        for node in nodes:
            r = {}
            for k in out_of[nid[node]]:
                r[x(i, k)] = r.get(x(i, k), 0) + 1
            for k in into[nid[node]]:
                r[x(i, k)] = r.get(x(i, k), 0) - 1
            b = (1 if node == src else 0) - (1 if node == snk else 0)
            rows.append(r)
            lo.append(b)
            hi.append(b)
    for gname, ks in groups.items():
        r = {x(i, k): 1 for i in range(A) for k in ks}
        rows.append(r)
        lo.append(-INF)
        hi.append(1)
    cost = [0.0] * nvar
    if objective == "distance":
        for i in range(A):
            for gi, key in enumerate(gkeys):
                u, v, _t = key
                ga = gadget_arcs[key]
                for a_, b_ in ((("in", u), ("out", v)), (("in", v), ("out", u))):
                    rows.append({m(i, gi): 1, x(i, ga[a_]): -1, x(i, ga[b_]): -1})
                    lo.append(-1)
                    hi.append(INF)
                cost[m(i, gi)] = 1.0
    sol = _solve_binary(nvar, cost, rows, lo, hi, backend)
    if sol is None:
        return None
    # extract paths: robot i at time t is the v with x[v(t)->v(t)'] = 1
    paths = [[None] * (T + 1) for _ in range(A)]
    for i in range(A):
        for k, a in enumerate(arcs):
            if a[2] is not None and a[2][0] == "cap" and sol[x(i, k)] > 0.5:
                _, v, t = a[2]
                paths[i][t] = v
    if objective == "distance":
        val = round(sum(sol[m(i, gi)] for i in range(A) for gi in range(nG)))
        return val, paths
    return paths


def _solve_binary(nvar, cost, rows, lo, hi, backend):
    """Minimize ``cost`` over binaries subject to ``lo <= rows <= hi``: the optimal ``x``,
    or ``None`` if infeasible. ``backend``: ``"gurobi"``, ``"scipy"`` or ``None`` (Gurobi
    when it works here, else scipy)."""
    if backend in (None, "gurobi"):
        try:
            return _solve_binary_gurobi(nvar, cost, rows, lo, hi)
        except Exception:
            if backend == "gurobi":
                raise
    return _solve_binary_scipy(nvar, cost, rows, lo, hi)


def _solve_binary_gurobi(nvar, cost, rows, lo, hi):
    import gurobipy as gp
    from gurobipy import GRB

    env = gp.Env(empty=True)
    env.setParam("OutputFlag", 0)
    env.start()
    try:
        mdl = gp.Model(env=env)
        mdl.Params.Threads = 1
        xs = mdl.addVars(nvar, vtype=GRB.BINARY, obj=cost)
        for r, l, h in zip(rows, lo, hi):
            e = gp.quicksum(c * xs[j] for j, c in r.items())
            if l == h:
                mdl.addConstr(e == l)
            else:
                if l != -INF:
                    mdl.addConstr(e >= l)
                if h != INF:
                    mdl.addConstr(e <= h)
        mdl.optimize()
        if mdl.Status in (GRB.INFEASIBLE, GRB.INF_OR_UNBD):  # binaries: never unbounded
            return None
        if mdl.Status != GRB.OPTIMAL:
            raise RuntimeError(f"gurobi status {mdl.Status}")
        out = [xs[j].X for j in range(nvar)]
        mdl.dispose()
        return out
    finally:
        env.dispose()


def _solve_binary_scipy(nvar, cost, rows, lo, hi):
    import numpy as np
    from scipy.optimize import Bounds, LinearConstraint, milp
    from scipy.sparse import lil_matrix

    M = lil_matrix((len(rows), nvar))
    for k, r in enumerate(rows):
        for j, c in r.items():
            M[k, j] = c
    res = milp(c=np.array(cost), constraints=LinearConstraint(M.tocsr(), np.array(lo), np.array(hi)),
               integrality=np.ones(nvar), bounds=Bounds(0, 1))
    if res.status == 2:
        return None
    if res.status != 0 or res.x is None:
        raise RuntimeError(f"scipy milp: {res.message}")
    return list(res.x)
