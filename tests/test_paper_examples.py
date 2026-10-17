"""Regression tests for the worked examples in the papers behind mppflow.

* TOR 2016: J. Yu, S. M. LaValle, "Optimal Multirobot Path Planning on Graphs: Complete
  Algorithms and Effective Heuristics", IEEE T-RO 32(5), 2016.
* ICRA 2013: J. Yu, S. M. LaValle, "Planning Optimal Paths for Multiple Robots on Graphs".
* STAR 2013: J. Yu, S. M. LaValle, "Multi-agent Path Planning and Network Flow"
  (Algorithmic Foundations of Robotics X, STAR 86).

The data in ``tests/data/paper/`` was transcribed from the papers and verified
independently (see ``tests/data/paper/README.md``). Each test checks that mppflow reproduces
an example, and also re-derives the expected value independently, using the joint-state
search, the validators and the small ILP defined in this file. Those use only the paper's
motion model and none of mppflow's solver code. Paper errors that the tests assert against
are documented in ``docs/notes/paper-errata.md``.

Conventions (as in the data files): a grid vertex id is ``row * cols + col``, with row 0 at
the top of the figure; robot ``i`` is the paper's label ``i + 1``; ``paths[i][t]`` is the
vertex of robot ``i`` at time ``t``.
"""
from __future__ import annotations

import heapq
import itertools
import json
import math
import os
import random
from collections import deque
from functools import lru_cache

import pytest

from mppflow import lp
from mppflow import solvers as S
from mppflow._java import MathRandom
from mppflow.graph import Graph, create_2d_grid_graph
from mppflow.model import build_full_network_model, build_time_model
from mppflow.network import CompactNetwork, FullNetwork
from mppflow.pathfinder import get_makespan_lower_bound
from mppflow.planner import PathPlanner, is_path_set_valid, path_set_error, plan, plan_metrics, split_paths
from mppflow.problem import get_a9_puzzle

DATA = os.path.join(os.path.dirname(__file__), "data", "paper")
from helpers import needs_gurobi  # noqa: E402  (gurobipy and a full-size license)

BACKENDS = lp.available_backends()  # gurobi cases skip without a full-size license (tests/conftest.py)
GRB = {"Threads": 1}


def params(backend):
    return GRB if backend == "gurobi" else None


@lru_cache(maxsize=None)
def _load(name):
    with open(os.path.join(DATA, name + ".json"), encoding="utf-8") as f:
        return json.load(f)


def load(name):
    return json.loads(json.dumps(_load(name)))  # a fresh copy per test


# =====================================================================================
# Independent helpers, written from the paper's definitions (Sec. II-A of TOR 2016):
# in each step every robot stays or moves along one edge; two robots may not end on the
# same vertex (meet) or swap along an edge (head-on). Following into a vertex that is being
# vacated is allowed, and so is rotating along a fully occupied cycle of length >= 3.
# Adjacency here is a list of neighbour lists without the vertex itself.
# =====================================================================================
def grid_adj(rows, cols):
    adj = []
    for r in range(rows):
        for c in range(cols):
            nb = []
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < rows and 0 <= cc < cols:
                    nb.append(rr * cols + cc)
            adj.append(sorted(nb))
    return adj


def grid_adj_removed(rows, cols, removed):
    """A grid with vertices removed, relabelled to ``0..n-1``; returns ``(adj, old->new)``."""
    keep = [v for v in range(rows * cols) if v not in set(removed)]
    idx = {v: i for i, v in enumerate(keep)}
    adj = []
    for v in keep:
        r, c = divmod(v, cols)
        nb = []
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < rows and 0 <= cc < cols and rr * cols + cc in idx:
                nb.append(idx[rr * cols + cc])
        adj.append(sorted(nb))
    return adj, idx


def edges_adj(n, edges):
    adj = [set() for _ in range(n)]
    for u, v in edges:
        adj[u].add(v)
        adj[v].add(u)
    return [sorted(a) for a in adj]


def to_graph(adj):
    """The mppflow graph with the same vertex ids (every vertex gets its wait self-loop)."""
    return Graph.from_adjacency([list(nb) for nb in adj])


def bfs_dist(adj, src):
    d = {src: 0}
    q = deque([src])
    while q:
        u = q.popleft()
        for w in adj[u]:
            if w not in d:
                d[w] = d[u] + 1
                q.append(w)
    return d


def step_error(adj, before, after):
    """Why the synchronous step ``before -> after`` is illegal, or ``None``."""
    n = len(before)
    for i in range(n):
        if before[i] != after[i] and after[i] not in adj[before[i]]:
            return "jump"
    if len(set(after)) < n:
        return "meet"
    for i in range(n):
        for j in range(i + 1, n):
            if before[i] != after[i] and before[i] == after[j] and before[j] == after[i]:
                return "head-on"
    return None


def plan_error(adj, paths, starts, goals):
    if [p[0] for p in paths] != list(starts):
        return "start"
    if [p[-1] for p in paths] != list(goals):
        return "goal"
    if len({len(p) for p in paths}) != 1:
        return "ragged"
    cfgs = configs(paths)
    if len(set(cfgs[0])) < len(cfgs[0]):
        return "meet at t=0"
    for a, b in zip(cfgs, cfgs[1:]):
        e = step_error(adj, a, b)
        if e:
            return e
    return None


def configs(paths):
    return [tuple(p[t] for p in paths) for t in range(len(paths[0]))]


def to_paths(cfgs):
    return [[c[i] for c in cfgs] for i in range(len(cfgs[0]))]


def successors(adj, cfg, fixed=frozenset()):
    """All legal next configurations (robots in ``fixed`` must stay). Backtracking: robot
    ``i`` picks a target that is not taken and that does not swap with an earlier robot."""
    n = len(cfg)
    pos = {v: i for i, v in enumerate(cfg)}
    tgt = [None] * n
    taken = set()
    out = []

    def rec(i):
        if i == n:
            out.append(tuple(tgt))
            return
        u = cfg[i]
        opts = (u,) if i in fixed else (u, *adj[u])
        for w in opts:
            if w in taken:
                continue
            j = pos.get(w)
            if w != u and j is not None and j < i and tgt[j] == u:
                continue  # head-on with an earlier robot
            tgt[i] = w
            taken.add(w)
            rec(i + 1)
            taken.discard(w)
        tgt[i] = None

    rec(0)
    return out


def joint_bfs(adj, starts, goals, limit=60):
    """Optimal makespan by breadth-first search over joint configurations."""
    s, g = tuple(starts), tuple(goals)
    if s == g:
        return 0
    seen = {s}
    frontier = [s]
    for d in range(1, limit + 1):
        nxt = []
        for c in frontier:
            for c2 in successors(adj, c):
                if c2 not in seen:
                    if c2 == g:
                        return d
                    seen.add(c2)
                    nxt.append(c2)
        frontier = nxt
        if not frontier:
            return None
    return None


def layered(adj, src, depth):
    """BFS layers from ``src`` up to ``depth``: ``{cfg: (dist, parents)}``."""
    info = {src: (0, [])}
    frontier = [src]
    for d in range(1, depth + 1):
        nxt = []
        for c in frontier:
            for c2 in successors(adj, c):
                e = info.get(c2)
                if e is None:
                    info[c2] = (d, [c])
                    nxt.append(c2)
                elif e[0] == d:
                    e[1].append(c)
        frontier = nxt
    return info


def chains(info, c):
    """All shortest sequences from the BFS source to ``c``."""
    d, parents = info[c]
    if d == 0:
        return [[c]]
    return [p + [c] for par in parents for p in chains(info, par)]


def unlabeled_bfs(adj, starts, goals, limit=60):
    """Optimal makespan of the unlabeled (permutation-invariant) problem."""
    s, g = tuple(sorted(starts)), tuple(sorted(goals))
    if s == g:
        return 0
    seen = {s}
    frontier = [s]
    for d in range(1, limit + 1):
        nxt = []
        for c in frontier:
            for c2 in successors(adj, c):
                k = tuple(sorted(c2))
                if k not in seen:
                    if k == g:
                        return d
                    seen.add(k)
                    nxt.append(k)
        frontier = nxt
    return None


def arrival_times(paths, goals):
    out = []
    for p, g in zip(paths, goals):
        t = len(p) - 1
        while t > 0 and p[t - 1] == g:
            t -= 1
        out.append(t)
    return out


def moves(paths):
    return [sum(1 for a, b in zip(p, p[1:]) if a != b) for p in paths]


def eq15_literal(paths, goals):
    """TOR eq. (15) evaluated on a plan: ``sum_i (T - sum_{t=1..T} y_i^t)`` with
    ``y_i^t = 1`` iff robot ``i`` is at its goal at every time ``t..T`` (eq. (14), blue arcs
    exist for ``t = 1..T`` only)."""
    T = len(paths[0]) - 1
    total = 0
    for p, g in zip(paths, goals):
        y = sum(1 for t in range(1, T + 1) if all(p[k] == g for k in range(t, T + 1)))
        total += T - y
    return total


def joint_dp(adj, starts, goals, T, cost):
    """Exact minimum over all plans of length exactly ``T`` of an additive objective, by
    dynamic programming over (configuration, set of robots that have finished).

    A robot may declare itself finished at time ``t`` only on its goal; from then on it
    stays. ``cost`` is ``"time"`` (sum of the arrival times: each step ``t -> t+1`` costs the
    number of unfinished robots), ``"eq15"`` (TOR eq. (15) read literally: the same, but the
    step ``0 -> 1`` is free) or ``"dist"`` (the number of moves). Small instances only."""
    n = len(starts)
    g = tuple(goals)

    def closures(cfg, done):
        if cost == "dist":  # moves do not depend on the finishing times
            yield done
            return
        at = [i for i in range(n) if cfg[i] == g[i] and i not in done]
        for k in range(len(at) + 1):
            for sub in itertools.combinations(at, k):
                yield done | frozenset(sub)

    layer = {}
    for d in closures(tuple(starts), frozenset()):
        layer[(tuple(starts), d)] = 0
    for t in range(T):
        nxt = {}
        for (cfg, done), c in layer.items():
            for c2 in successors(adj, cfg, done):
                if cost == "dist":
                    step = sum(1 for a, b in zip(cfg, c2) if a != b)
                elif cost == "time":
                    step = n - len(done)
                else:
                    step = 0 if t == 0 else n - len(done)
                for d2 in closures(c2, done):
                    k = (c2, d2)
                    if c + step < nxt.get(k, math.inf):
                        nxt[k] = c + step
        layer = nxt
    return min((c for (cfg, d), c in layer.items() if cfg == g and (cost == "dist" or len(d) == n)), default=None)


def max_flow(n_nodes, arcs, s, t):
    """Edmonds-Karp on ``arcs = [(u, v, cap)]``."""
    cap = {}
    nb = [set() for _ in range(n_nodes)]
    for u, v, c in arcs:
        cap[(u, v)] = cap.get((u, v), 0) + c
        cap.setdefault((v, u), 0)
        nb[u].add(v)
        nb[v].add(u)
    flow = 0
    while True:
        par = {s: None}
        q = deque([s])
        while q and t not in par:
            u = q.popleft()
            for v in nb[u]:
                if v not in par and cap[(u, v)] > 0:
                    par[v] = u
                    q.append(v)
        if t not in par:
            return flow
        b, v = math.inf, t
        while par[v] is not None:
            b = min(b, cap[(par[v], v)])
            v = par[v]
        v = t
        while par[v] is not None:
            cap[(par[v], v)] -= b
            cap[(v, par[v])] += b
            v = par[v]
        flow += b


def lp_max_flow(n_nodes, arcs, s, t, backend):
    """The same max flow as an LP solved by an mppflow backend."""
    m = lp.LinearModel("max")
    x = [m.add_column(0.0, float(c), "C") for (_, _, c) in arcs]
    for v in range(n_nodes):
        if v in (s, t):
            continue
        terms = [(x[j], 1.0) for j, (a, b, _) in enumerate(arcs) if b == v]
        terms += [(x[j], -1.0) for j, (a, b, _) in enumerate(arcs) if a == v]
        if terms:
            m.add_row(terms, "==", 0.0)
    m.set_objective([(x[j], 1.0) for j, (a, _, _) in enumerate(arcs) if a == s]
                    + [(x[j], -1.0) for j, (_, b, _) in enumerate(arcs) if b == s], "max")
    sol = lp.solve(m, backend)
    assert sol.status == "optimal"
    return round(sol.objective)


def visual_to_cfg(layout):
    """Rows of robot labels (top to bottom) -> ``cfg[label - 1] = vertex``."""
    cols = len(layout[0])
    cfg = [None] * (len(layout) * cols)
    for r, row in enumerate(layout):
        for c, lab in enumerate(row):
            cfg[lab - 1] = r * cols + c
    return tuple(cfg)


def apply_arrows(cfg, arrows):
    """One step given as arrows ``(from, to)``: the robot on ``from`` moves to ``to``."""
    where = {v: i for i, v in enumerate(cfg)}
    out = list(cfg)
    for a, b in arrows:
        out[where[a]] = b
    return tuple(out)


def rc_arrows(step, cols):
    return [(a[0] * cols + a[1], b[0] * cols + b[1]) for a, b in step]


def rotation_cycles(arrows):
    """Decompose a step into directed cycles (every arrow's head must be another's tail)."""
    nxt = dict(arrows)
    assert len(nxt) == len(arrows), "two arrows leave one vertex"
    assert sorted(nxt) == sorted(nxt.values()), "not a set of rotations"
    seen, cyc = set(), []
    for v in nxt:
        if v in seen:
            continue
        c = [v]
        seen.add(v)
        while nxt[c[-1]] != v:
            c.append(nxt[c[-1]])
            seen.add(c[-1])
        cyc.append(c)
    return cyc


# =====================================================================================
# 9-puzzle: TOR Fig. 1 / Fig. 3 (= ICRA Fig. 2 / Fig. 3)
# =====================================================================================
def nine():
    d = load("tor16_fig1_fig3_9puzzle")
    return d, create_2d_grid_graph(3, 3), grid_adj(3, 3)


def optimal_9puzzle_plans():
    d = load("tor16_fig1_fig3_9puzzle")
    return [to_paths([visual_to_cfg(c) for c in p]) for p in d["expected"]["all_optimal_plans_visual"]]


def test_9puzzle_instance_transcription():
    d, g, adj = nine()
    icra = load("icra_9puzzle_fig2_fig3_fig7")
    assert icra["fig2_start_rowmajor"] == d["visual_layout_start"]
    assert list(visual_to_cfg(d["visual_layout_start"])) == d["starts"]
    assert list(visual_to_cfg(d["visual_layout_goal"])) == d["goals"] == list(range(9))
    assert g.adj == {v: sorted([v] + adj[v]) for v in range(9)}


def test_9puzzle_independent_bfs_four_optimal_plans():
    """Bidirectional BFS: the optimum is 4 and exactly 4 optimal plans exist (the fixture's)."""
    d, _, adj = nine()
    s, g = tuple(d["starts"]), tuple(d["goals"])
    F, B = layered(adj, s, 2), layered(adj, g, 2)  # the move set is closed under reversal
    meet = [c for c in F if c in B]
    best = min(F[c][0] + B[c][0] for c in meet)
    assert best == d["expected"]["optimal_makespan"] == 4
    plans = set()
    for c in meet:
        if F[c][0] == 2 and B[c][0] == 2:
            for a in chains(F, c):
                for b in chains(B, c):
                    plans.add(tuple(a + b[::-1][1:]))
    assert len(plans) == d["expected"]["number_of_optimal_plans"] == 4
    fixture = {tuple(configs(p)) for p in optimal_9puzzle_plans()}
    assert plans == fixture
    for p in plans:
        assert plan_error(adj, to_paths(list(p)), s, g) is None


def test_9puzzle_fig3_plan_valid():
    d, g, adj = nine()
    icra = load("icra_9puzzle_fig2_fig3_fig7")
    cfg = tuple(d["starts"])
    seq = [cfg]
    for k, step in enumerate(icra["fig3_steps_as_directed_cycles"]):
        arrows = rc_arrows(step, 3)
        assert sorted(arrows) == sorted(map(tuple, d["fig3_moves_vertex"][k]))
        cyc = rotation_cycles(arrows)
        assert len(cyc) == 1 and len(cyc[0]) == icra["fig3_cycle_lengths"][k]
        cfg = apply_arrows(cfg, arrows)
        seq.append(cfg)
    assert [list(c) for c in seq] == d["fig3_configs_robot_to_vertex"]
    assert [visual_to_cfg(v) for v in d["fig3_configs_visual"]] == seq
    paths = to_paths(seq)
    assert plan_error(adj, paths, d["starts"], d["goals"]) is None
    for compat in ("fixed", "original"):
        assert is_path_set_valid(paths, g, d["starts"], d["goals"], compat)
    e = d["expected"]
    assert [optimal_9puzzle_plans().index(paths)] == e["fig3_plan_index_in_all_optimal_plans"]
    m = plan_metrics(paths, d["goals"])
    assert m["makespan"] == e["fig3_plan_makespan"] == 4
    assert m["total_time"] == e["fig3_total_arrival_time"] == sum(e["fig3_arrival_times"]) == 29
    assert m["total_distance"] == e["fig3_total_distance"] == sum(e["fig3_distances"]) == 24
    assert arrival_times(paths, d["goals"]) == e["fig3_arrival_times"]
    assert moves(paths) == e["fig3_distances"]


@pytest.mark.parametrize("backend", [pytest.param(b, marks=pytest.mark.slow) if b == "builtin" else b
                                     for b in BACKENDS])  # builtin: about 30 s
def test_9puzzle_optimal_makespan_every_backend(backend):
    d, g, adj = nine()
    r = S.min_makespan(g, d["starts"], d["goals"], backend=backend, params=params(backend))
    assert r.status == "optimal" and r.makespan == 4
    assert is_path_set_valid(r.paths, g, d["starts"], d["goals"])
    assert plan_error(adj, r.paths, d["starts"], d["goals"]) is None
    assert r.paths in optimal_9puzzle_plans()  # there are only 4 optimal plans


def test_9puzzle_full_network_max_flow():
    """The paper's full network (eqs. (8)-(10)): max flow 8 < 9 at T = 3 and 9 at T = 4."""
    d, g, adj = nine()
    exp = d["expected"]["ilp_independent"]
    for T, val in ((3, exp["full_T3_maxflow"]), (4, exp["full_T4_maxflow"])):
        fm = build_full_network_model(g, d["starts"], d["goals"], T)
        sol = lp.solve(fm.model, "highs")
        assert sol.status == "optimal" and round(sol.objective) == val, T
    assert exp["compact_T3_feasible"] is False and exp["compact_T4_feasible"] is True
    for T, ok in ((3, False), (4, True)):
        m = build_time_model(g, d["starts"], d["goals"], T, set_optimal=True)
        assert (lp.solve(m.model, "highs").status == "optimal") is ok
    fm = build_full_network_model(g, d["starts"], d["goals"], 4, set_optimal=True)
    sol = lp.solve(fm.model, "highs")
    paths = fm.retrieve_paths(sol.x)
    assert plan_error(adj, paths, d["starts"], d["goals"]) is None and paths in optimal_9puzzle_plans()


@pytest.mark.parametrize("compat", ["fixed", "original"])
def test_9puzzle_plan_paths_advanced(compat):
    d, g, _ = nine()
    pl = PathPlanner(backend="highs", compat=compat, rng=MathRandom(1))
    paths = pl.plan_paths_advanced(g, d["starts"], d["goals"], False, 0, True, -1.0)
    assert len(paths[0]) - 1 == load("tor16_java_original_outputs")["fig1_fig3_9puzzle"]["makespan"] == 4
    assert paths in optimal_9puzzle_plans()
    r = plan((g, d["starts"], d["goals"]))
    assert r.makespan == 4 and r.lower_bounds["makespan"] == 4 and r.status == "optimal"


def test_9puzzle_lower_bound():
    d, g, adj = nine()
    lb = get_makespan_lower_bound(g, d["starts"], d["goals"], MathRandom(1)) - 1  # vertex count - 1
    dist = [bfs_dist(adj, s)[t] for s, t in zip(d["starts"], d["goals"])]
    assert lb == max(dist) == d["expected"]["makespan_lower_bound_max_shortest_path"] == 4
    assert dist.index(4) == 8  # robot 9 is four steps away (Sec. II-B)


def test_code_builtin_9puzzle_is_not_fig1():
    """The original's ``getA9Puzzle`` is a different instance (optimum 5, lower bound 3)."""
    d, g, adj = nine()
    c = d["code_builtin_9puzzle"]
    p = get_a9_puzzle()
    assert p.starts == c["starts"] != d["starts"] and p.goals == c["goals"]
    assert joint_bfs(adj, c["starts"], c["goals"]) == c["optimal_makespan"] == 5
    assert max(bfs_dist(adj, s)[t] for s, t in zip(c["starts"], c["goals"])) == c["lower_bound"] == 3
    r = S.min_makespan(p.graph, p.starts, p.goals)
    assert r.makespan == 5 and is_path_set_valid(r.paths, p.graph, p.starts, p.goals)
    # regression (B1): the docstring used to call this "the TOR Fig. 1 9-puzzle"
    doc = " ".join(get_a9_puzzle.__doc__.split())
    assert "the TOR Fig. 1 9-puzzle" not in doc and "*not* the TOR Fig. 1 instance" in doc


# =====================================================================================
# 25-puzzle: TOR Fig. 14 (instance) / Fig. 15 (7-step solution)
# =====================================================================================
def fig15_plan():
    d = load("tor16_fig14_fig15_25puzzle")
    cfg = tuple(d["starts"])
    seq = [cfg]
    for step in d["fig15_moves_rowcol"]:
        cfg = apply_arrows(cfg, rc_arrows(step, 5))
        seq.append(cfg)
    return d, seq


def test_25puzzle_fig15_plan_valid_and_optimal():
    d, seq = fig15_plan()
    g, adj = create_2d_grid_graph(5, 5), grid_adj(5, 5)
    assert list(visual_to_cfg(d["visual_layout_start"])) == d["starts"]
    assert d["goals"] == list(range(25))
    assert [len(s) for s in d["fig15_moves_rowcol"]] == d["fig15_arrow_counts_per_panel"]
    for step in d["fig15_moves_rowcol"]:
        assert all(len(c) >= 3 for c in rotation_cycles(rc_arrows(step, 5)))  # disjoint rotations
    assert [list(c) for c in seq] == d["fig15_configs_robot_to_vertex"]
    paths = to_paths(seq)
    assert paths[0][-1] == 0 and [p[-1] for p in paths] == d["goals"]  # ends exactly at the goal
    assert plan_error(adj, paths, d["starts"], d["goals"]) is None
    for compat in ("fixed", "original"):
        assert is_path_set_valid(paths, g, d["starts"], d["goals"], compat)
    e = d["expected"]
    m = plan_metrics(paths, d["goals"])
    assert m["makespan"] == e["fig15_plan_makespan"] == 7
    assert m["total_distance"] == e["fig15_total_distance"] == 134
    assert m["total_time"] == e["fig15_total_arrival_time"] == 167
    assert moves(paths) == e["fig15_distances"]
    assert arrival_times(paths, d["goals"]) == e["fig15_arrival_times"]
    # optimal: the lower bound (robot 10's distance) equals the plan's makespan
    dist = [bfs_dist(adj, s)[t] for s, t in zip(d["starts"], d["goals"])]
    assert dist == e["shortest_path_lengths"]
    assert max(dist) == 7 and [i + 1 for i, x in enumerate(dist) if x == 7] == e["robots_attaining_lower_bound"]
    assert sum(dist) == e["total_distance_lower_bound"] == 78
    assert get_makespan_lower_bound(g, d["starts"], d["goals"], MathRandom(1)) - 1 == 7
    assert plan_metrics(paths, d["goals"])["makespan"] == e["optimal_makespan"]


@pytest.mark.slow
@pytest.mark.parametrize("backend", [pytest.param("gurobi", marks=[pytest.mark.gurobi, needs_gurobi]), "highs"])
def test_25puzzle_solved_makespan_7(backend):
    d = load("tor16_fig14_fig15_25puzzle")
    g = create_2d_grid_graph(5, 5)
    r = S.min_makespan(g, d["starts"], d["goals"], backend=backend, solve_relaxed_first=True)
    assert r.status == "optimal" and r.makespan == d["expected"]["optimal_makespan"] == 7
    assert load("tor16_java_original_outputs")["fig14_fig15_25puzzle"]["makespan"] == 7
    assert is_path_set_valid(r.paths, g, d["starts"], d["goals"])
    assert plan_error(grid_adj(5, 5), r.paths, d["starts"], d["goals"]) is None


# =====================================================================================
# TOR Fig. 2: the motion model
# =====================================================================================
@pytest.mark.parametrize("case", load("tor16_fig2_moves")["cases"], ids=lambda c: c["case"])
def test_fig2_move_cases(case):
    n = 1 + max(max(e) for e in case["edges"])
    adj = edges_adj(n, case["edges"])
    g = to_graph(adj)
    paths = [[a, b] for a, b in zip(case["before"], case["after"])]
    for compat in ("fixed", "original"):
        assert is_path_set_valid(paths, g, case["before"], case["after"], compat) is case["expected_valid"]
    err = step_error(adj, case["before"], case["after"])
    assert (err is None) is case["expected_valid"]
    msg = path_set_error(paths, g, case["before"], case["after"])
    if not case["expected_valid"]:
        kind = "head-on" if "head-on" in case["violations"][0] else "meet"
        assert err == kind
        assert ("cross the same edge" if kind == "head-on" else "meet on vertex") in msg
    else:
        assert msg is None


def test_icra_fig7_exchange_of_8_and_9():
    """ICRA Fig. 7: three rotations on a 2x3 grid exchange robots 8 and 9."""
    d = load("icra_9puzzle_fig2_fig3_fig7")
    labels = sorted(x for row in d["fig7_configs"][0] for x in row)
    cfgs = []
    for layout in d["fig7_configs"]:
        where = {lab: r * 3 + c for r, row in enumerate(layout) for c, lab in enumerate(row)}
        cfgs.append(tuple(where[lab] for lab in labels))
    adj = grid_adj(2, 3)
    paths = to_paths(cfgs)
    assert plan_error(adj, paths, cfgs[0], cfgs[-1]) is None
    assert is_path_set_valid(paths, to_graph(adj), cfgs[0], cfgs[-1])
    i8, i9 = labels.index(8), labels.index(9)
    assert cfgs[-1][i8] == cfgs[0][i9] and cfgs[-1][i9] == cfgs[0][i8]
    assert all(cfgs[-1][i] == cfgs[0][i] for i in range(len(labels)) if i not in (i8, i9))
    assert joint_bfs(adj, cfgs[0], cfgs[-1]) == 3  # and three steps are needed
    assert S.min_makespan(to_graph(adj), list(cfgs[0]), list(cfgs[-1])).makespan == 3


# =====================================================================================
# TOR Fig. 4-7: time-expanded networks on the Fig. 4(a) graph
# =====================================================================================
def fig4():
    d = load("tor16_fig4_fig7_network")
    adj = edges_adj(6, d["graph"]["edges"])
    return d, to_graph(adj), adj


def test_fig5_full_network_counts():
    d, g, _ = fig4()
    exp = d["expected"]["fig5_full_network_T2"]
    net = FullNetwork.build(g, d["starts"], d["goals"], 2)
    assert net.counts() == exp["built"]
    f = net.formula_counts()
    assert {k: f[k] for k in f if k in exp["formula"]} == {k: v for k, v in exp["formula"].items() if k in f}
    # n*T goal blue arcs ("the four bold blue arcs")
    goal_blue = [j for j, (a, _, k) in enumerate(net.arcs)
                 if k == "blue" and net.nodes[a][1] in d["goals"]]
    assert len(goal_blue) == exp["formula"]["goal_blue_arcs_x_i^t (bold blue in Fig.5)"] == 4
    # general formulas |V'| = |V|(2T+1) + 2|E|T, |E'| = 2|V|T + 5|E|T + n
    for T in range(0, 6):
        c = FullNetwork.build(g, d["starts"], d["goals"], T).counts()
        assert c["vertices_total"] == 6 * (2 * T + 1) + 2 * 5 * T
        assert c["arcs_total"] == 2 * 6 * T + 5 * 5 * T + 2


def test_fig6_fig7_compact_network_counts():
    d, g, _ = fig4()
    exp = d["expected"]["fig6_compact_network_T2"]
    c = CompactNetwork.build(g, d["starts"], d["goals"], 2).counts()
    assert {k: c[k] for k in exp} == exp
    for T in range(0, 6):
        c = CompactNetwork.build(g, d["starts"], d["goals"], T).counts()
        assert c["vertices"] == 6 * (T + 1) and c["arcs_total"] == 6 * T + 2 * 5 * T
    assert 2 * 5 * 2 / exp["move_arcs"] == 1  # the 2-arc gadget: two move arcs per edge and step


def test_fig4a_full_network_max_flow_by_T():
    """Max multiflow on the full network (eqs. (8)-(10)) for T = 0..5: the T = 2 network drawn
    in Fig. 5 admits no flow; the instance needs T = 4. Independently: the largest set of
    robots that can be routed in T steps (the others removed), by joint BFS."""
    d, g, adj = fig4()
    exp = {int(k): v for k, v in d["expected"]["full_network_maxflow_by_T"].items()}
    n = len(d["starts"])
    for T, val in exp.items():
        fm = build_full_network_model(g, d["starts"], d["goals"], T)
        sol = lp.solve(fm.model, "highs")
        assert sol.status == "optimal" and round(sol.objective) == val, T
        best = 0
        for k in range(n, 0, -1):
            if any(joint_bfs(adj, [d["starts"][i] for i in sub], [d["goals"][i] for i in sub], T) is not None
                   for sub in itertools.combinations(range(n), k)):
                best = k
                break
        assert best == val, T
        assert d["expected"]["feasible_by_T"][str(T)] is (val == n)


def test_fig4a_optimal_makespan_and_compact_model_sizes():
    d, g, adj = fig4()
    assert joint_bfs(adj, d["starts"], d["goals"]) == d["expected"]["optimal_makespan"] == 4
    assert [bfs_dist(adj, s)[t] for s, t in zip(d["starts"], d["goals"])] == d["expected"]["shortest_path_lengths"]
    r = S.min_makespan(g, d["starts"], d["goals"])
    assert r.makespan == 4 and is_path_set_valid(r.paths, g, d["starts"], d["goals"])
    ex = d["expected"]["optimal_plan_example"]
    assert plan_error(adj, to_paths([tuple(c) for c in ex]), d["starts"], d["goals"]) is None
    # model sizes and statuses of the original's makespan model (reachability-pruned)
    java = load("tor16_java_original_outputs")["fig4a_network"]
    assert java["makespan"] == 4 and java["total_distance"] == 6
    for T, info in java["time_model_size_by_T"].items():
        m = build_time_model(g, d["starts"], d["goals"], int(T))
        assert (m.model.num_columns, m.model.num_rows) == (info["cols"], info["rows"]), T
        sol = lp.solve(m.model, "highs")
        assert sol.status == info["status"].lower(), T
        if info["status"] == "OPTIMAL":
            assert round(sol.objective) == info["obj"]


def _count_dag_paths(net, src, dst):
    out = net.out_arcs()

    @lru_cache(maxsize=None)
    def cnt(v):
        if v == dst:
            return 1
        return sum(cnt(net.arcs[j][1]) for j in out[v] if net.arcs[j][2] != "loop")

    return cnt(src)


def _count_walks(adj, s, g, T):
    ways = {s: 1}
    for _ in range(T):
        nxt = {}
        for v, c in ways.items():
            for w in [v] + adj[v]:
                nxt[w] = nxt.get(w, 0) + c
        ways = nxt
    return ways.get(g, 0)


def test_theorem1_flows_versus_paths():
    """Theorem 1's "bijection" is not injective: a robot can stay at u through a gadget."""
    d, g, adj = fig4()
    exp = d["expected"]["theorem1_flows_vs_paths_single_robot"]
    s, t = d["starts"][0], d["goals"][0]
    for T in (1, 2, 3, 4):
        net = FullNetwork.build(g, [s], [t], T)
        flows = _count_dag_paths(net, net.copy_node(s, 0), net.copy_node(t, T, prime=True))
        assert flows == exp[str(T)]["single_robot_flows_r1"], T
        assert _count_walks(adj, s, t, T) == exp[str(T)]["single_robot_paths_r1"], T
    # ICRA Theorem 2 / TOR Theorem 1 on one edge, T = 1: two flows, one (stay) path
    b = load("icra_bijection_duplicate_flows")
    g1 = to_graph([[1], [0]])
    net = FullNetwork.build(g1, [0], [0], b["T"])
    assert _count_dag_paths(net, net.copy_node(0, 0), net.copy_node(0, 1, prime=True)) \
        == b["num_unit_flows_u0_to_uTprime"] == exp["single_edge_stay_T1"]["flows"] == 2
    assert _count_walks([[1], [0]], 0, 0, 1) == b["num_paths_in_MPP"] == 1


def _all_plans(adj, starts, goals, T):
    out = []

    def rec(seq):
        if len(seq) == T + 1:
            if seq[-1] == tuple(goals):
                out.append(list(seq))
            return
        for c in successors(adj, seq[-1]):
            seq.append(c)
            rec(seq)
            seq.pop()

    rec([tuple(starts)])
    return out


def test_eq16_eq12_constant_with_unit_cost_waits():
    """With the paper's costs (green wait arcs and gadget middle arcs cost 1), eqs. (16) and
    (12) are constant (n*T and T); with free waits they give the true optima (6 and 3)."""
    d, g, adj = fig4()
    n = len(d["starts"])
    for T in (4, 5, 6):
        exp = d["expected"]["eq16_with_literal_paper_costs"][str(T)]
        vals = []
        for sense in ("min", "max"):
            fm = build_full_network_model(g, d["starts"], d["goals"], T, objective="distance", paper_costs=True)
            fm.model.set_objective(list(fm.model.objective_terms), sense)
            sol = lp.solve(fm.model, "highs")
            vals.append(round(sol.objective))
        assert vals == [exp["eq16_min_with_paper_costs"], exp["eq16_max_with_paper_costs"]] == [n * T, n * T]
        fm = build_full_network_model(g, d["starts"], d["goals"], T, objective="distance")
        assert round(lp.solve(fm.model, "highs").objective) == exp["true_min_total_distance"] == 6
        fm = build_full_network_model(g, d["starts"], d["goals"], T, objective="max_distance", paper_costs=True)
        assert round(lp.solve(fm.model, "highs").objective) == T
        fm = build_full_network_model(g, d["starts"], d["goals"], T, objective="max_distance")
        assert round(lp.solve(fm.model, "highs").objective) == 3
        r = S.solve_distance(g, d["starts"], d["goals"], 0.0, T, horizon="single")
        assert r.objective == 6
        r = S.solve_max_distance(g, d["starts"], d["goals"], 0.0, T, horizon="single")
        assert r.objective == 3
    # independent: enumerate every plan at T = 4 and price it with the paper's costs (a wait
    # step costs 1 (green arc), a move costs 1 (gadget middle arc)): always n*T
    plans = _all_plans(adj, d["starts"], d["goals"], 4)
    assert plans
    paper = {sum((1 if a == b else 0) + (1 if b in adj[a] else 0) for p in to_paths(c) for a, b in zip(p, p[1:]))
             for c in plans}
    assert paper == {n * 4}
    assert min(sum(moves(to_paths(c))) for c in plans) == 6
    assert min(max(moves(to_paths(c))) for c in plans) == 3


def test_fig4a_total_time_and_distance():
    d, g, adj = fig4()
    icra = load("icra_fig4a_fig5_network")
    assert icra["network_T2"]["num_nodes"] == 50
    assert icra["network_T2"]["num_arcs_including_2_loopbacks"] == 76
    assert {int(k): v for k, v in icra["max_multiflow_value_by_T"].items()} == \
        {int(k): v for k, v in d["expected"]["full_network_maxflow_by_T"].items() if k != "0"}
    r = S.min_total_time(g, d["starts"], d["goals"])
    assert r.objective == icra["min_total_arrival_time"] == 7
    assert plan_metrics(r.paths, d["goals"])["total_time"] == 7
    r = S.min_total_distance(g, d["starts"], d["goals"])
    assert r.objective == icra["min_total_distance"] == 6
    for T in (4, 5, 6):
        assert joint_dp(adj, d["starts"], d["goals"], T, "time") == 7
        assert joint_dp(adj, d["starts"], d["goals"], T, "dist") == 6


# =====================================================================================
# TOR Fig. 8: the 2-way split (Sec. V-B)
# =====================================================================================
def test_fig8_split_goals_and_child_horizons():
    """Fig. 8: the intermediate goals are (1,1) = 5 and (2,2) = 10. The paper says both
    children need 2 steps; in fact child 1 needs 3 (both robots' only 2-step paths pass
    (2,1) at t = 1), so this split gives 3 + 2 = 5 > 4. See docs/notes/paper-errata.md."""
    d = load("tor16_fig8_split")
    g, adj = create_2d_grid_graph(4, 4), grid_adj(4, 4)
    s, t = d["starts"], d["goals"]
    e = d["expected"]
    java = load("tor16_java_original_outputs")["fig8_split"]
    for compat in ("fixed", "original"):
        mid = split_paths(g, s, t, rng=MathRandom(1), compat=compat)
        assert mid == d["intermediate_goals"] == java["splitPaths_middles"] == [5, 10]
    mid = [5, 10]
    for i in range(2):  # the intermediate goals lie on shortest paths
        ds, dt = bfs_dist(adj, s[i]), bfs_dist(adj, t[i])
        assert ds[mid[i]] + dt[mid[i]] == ds[t[i]] == 4
        assert ds[mid[i]] == 2
    assert [bfs_dist(adj, a)[b] for a, b in zip(s, t)] == e["shortest_path_lengths"]
    # independent joint BFS
    assert joint_bfs(adj, s, t) == e["optimal_makespan_unsplit"] == java["exact_makespan"] == 4
    assert joint_bfs(adj, s, mid) == e["child1_optimal_makespan"] == 3  # not 2 as the paper says
    assert joint_bfs(adj, mid, t) == e["child2_optimal_makespan"] == 2
    assert e["two_way_split_total_makespan"] == 5
    for pair in e["intermediate_pairs_on_shortest_paths_giving_2_plus_2"]:
        alt = [r * 4 + c for r, c in pair]
        assert joint_bfs(adj, s, alt) == 2 and joint_bfs(adj, alt, t) == 2
    # mppflow
    r1 = S.min_makespan(g, s, mid)
    r2 = S.min_makespan(g, mid, t)
    assert (r1.makespan, r2.makespan) == (java["child1_makespan"], java["child2_makespan"]) == (3, 2)
    assert S.min_makespan(g, s, t).makespan == 4
    for compat in ("fixed", "original"):
        pl = PathPlanner(compat=compat, rng=MathRandom(1))
        paths = pl.plan_paths_advanced_split(g, s, t, True, -1.0, 1)
        assert len(paths[0]) - 1 == java["planPathsAdvancedSplit_splits1_makespan"] == 5
        assert is_path_set_valid(paths, g, s, t, compat)
        assert [p[3] for p in paths] == mid
        pl = PathPlanner(compat=compat, rng=MathRandom(1))
        paths = pl.multi_threaded_split_planning(g, s, t, 60.0, 1)  # the Java run used 60 s
        assert len(paths[0]) - 1 == java["multiThreadedSplitPlanning_splits1_makespan"] == 5
        assert is_path_set_valid(paths, g, s, t, compat)
        # without a limit the original gives up on child 1 at its lower bound T = 2
        # (docs/notes/original-bugs.md: non-positive time limit in multiThreadedSplitPlanning)
        paths = PathPlanner(compat=compat, rng=MathRandom(1)).multi_threaded_split_planning(g, s, t, -1.0, 1)
        assert (paths is None) if compat == "original" else (len(paths[0]) - 1 == 5)
    r = plan((g, s, t), split=2, rng=MathRandom(1))
    assert r.makespan == 5 and r.intermediate_goals == [mid]
    assert plan_error(adj, r.paths, s, t) is None
    # the reversed instance
    rev = java["reversed_instance (starts [0,7] goals [13,8]) middles"]
    assert split_paths(g, t, s, rng=MathRandom(1)) == rev == [5, 10]
    assert [joint_bfs(adj, t, rev), joint_bfs(adj, rev, s)] == java["reversed child makespans"] == [2, 3]


# =====================================================================================
# TOR eq. (14)-(15): total arrival time
# =====================================================================================
def test_eq15_on_fig3_plan_and_hand_example():
    d, g, adj = nine()
    b = load("tor16_bounds_objectives")
    paths = optimal_9puzzle_plans()[d["expected"]["fig3_plan_index_in_all_optimal_plans"][0]]
    e = b["eq15_on_fig3_plan"]
    assert arrival_times(paths, d["goals"]) == e["arrival_times"]
    assert plan_metrics(paths, d["goals"])["total_time"] == e["sum_tf"] == 29
    assert eq15_literal(paths, d["goals"]) == e["paper_eq15_value"] == e["sum_max(tf-1,0)"] == 20
    h = b["eq15_hand_example"]
    adj = edges_adj(5, h["edges"])
    paths = to_paths([tuple(c) for c in h["plan"]])
    assert plan_error(adj, paths, h["starts"], h["goals"]) is None
    assert is_path_set_valid(paths, to_graph(adj), h["starts"], h["goals"])
    assert arrival_times(paths, h["goals"]) == h["arrival_times"]
    assert plan_metrics(paths, h["goals"])["total_time"] == h["true_total"] == 6
    assert eq15_literal(paths, h["goals"]) == h["paper_eq15"] == 4
    assert joint_bfs(adj, h["starts"], h["goals"]) == h["optimal_makespan"] == 3
    # the recorded plan illustrates the off-by-one; it is not total-time optimal: robot 1
    # can follow robot 2 back into vertex 1 one step earlier (arrival times [2, 3])
    r = S.solve_total_time(to_graph(adj), h["starts"], h["goals"], None, 0.0, 3, horizon="single")
    assert r.objective == joint_dp(adj, h["starts"], h["goals"], 3, "time") == 5 < h["true_total"]
    assert arrival_times(r.paths, h["goals"]) == [2, 3]
    assert eq15_literal(r.paths, h["goals"]) == 1 + 2
    assert joint_dp(adj, h["starts"], h["goals"], 3, "eq15") == 3


class IndependentILP:
    """A compact time-expanded ILP written from the motion model only (not mppflow's model):
    position ``p[i,v,t]``, move ``m[i,u,w,t]`` (``w = u`` is a wait) and
    ``z[i,t]`` = "robot i is at its goal at every time t..T" (exact, both directions).
    True total time = ``nT - sum_{t<T} z``; eq. (15) read literally = ``nT - sum_{t>=1} z``.
    Pure binary, solved with scipy's HiGHS."""

    def __init__(self, adj, starts, goals, T):
        self.n, self.T = len(starts), T
        V = range(len(adj))
        self.col = {}
        rows = []

        def var(k):
            return self.col.setdefault(k, len(self.col))

        for i in range(self.n):
            for t in range(T + 1):
                for v in V:
                    var(("p", i, v, t))
                var(("z", i, t))
            for t in range(T):
                for u in V:
                    for w in [u] + adj[u]:
                        var(("m", i, u, w, t))
        for i, (s, g) in enumerate(zip(starts, goals)):
            rows.append(({var(("p", i, s, 0)): 1}, 1, 1))
            rows.append(({var(("p", i, g, T)): 1}, 1, 1))
            for t in range(T + 1):
                rows.append(({var(("p", i, v, t)): 1 for v in V}, 1, 1))
            for t in range(T):
                for u in V:
                    r = {var(("m", i, u, w, t)): 1 for w in [u] + adj[u]}
                    r[var(("p", i, u, t))] = -1
                    rows.append((r, 0, 0))
                for w in V:
                    r = {var(("m", i, u, w, t)): 1 for u in [w] + adj[w]}
                    r[var(("p", i, w, t + 1))] = -1
                    rows.append((r, 0, 0))
            rows.append(({var(("z", i, T)): 1, var(("p", i, g, T)): -1}, 0, 0))
            for t in range(T):
                z, z1, p = var(("z", i, t)), var(("z", i, t + 1)), var(("p", i, g, t))
                rows.append(({z: 1, p: -1}, -math.inf, 0))
                rows.append(({z: 1, z1: -1}, -math.inf, 0))
                rows.append(({z: 1, p: -1, z1: -1}, -1, math.inf))
        for t in range(T + 1):
            for v in V:
                rows.append(({var(("p", i, v, t)): 1 for i in range(self.n)}, -math.inf, 1))
        for t in range(T):
            for u in V:
                for w in adj[u]:
                    if u < w:
                        r = {}
                        for i in range(self.n):
                            r[var(("m", i, u, w, t))] = 1
                            r[var(("m", i, w, u, t))] = 1
                        rows.append((r, -math.inf, 1))
        self.rows = rows

    def z_sum(self, t_range):
        return {self.col[("z", i, t)]: 1 for i in range(self.n) for t in t_range}

    def true_obj(self):  # minimize n*T - sum_{t=0..T-1} z
        return self.z_sum(range(self.T)), self.n * self.T

    def eq15_obj(self):  # minimize n*T - sum_{t=1..T} z
        return self.z_sum(range(1, self.T + 1)), self.n * self.T

    def solve(self, obj, sense="min", extra=()):
        np = pytest.importorskip("numpy")
        opt = pytest.importorskip("scipy.optimize")
        sparse = pytest.importorskip("scipy.sparse")
        zs, const = obj
        nc = len(self.col)
        c = np.zeros(nc)
        for j in zs:
            c[j] = 1.0  # min (const - sum z) == max sum z
        if sense == "min":
            c = -c
        rows = list(self.rows) + list(extra)
        data, ri, ci, lo, hi = [], [], [], [], []
        for k, (r, a, b) in enumerate(rows):
            for j, v in r.items():
                ri.append(k)
                ci.append(j)
                data.append(v)
            lo.append(a)
            hi.append(b)
        A = sparse.csr_matrix((data, (ri, ci)), shape=(len(rows), nc))
        res = opt.milp(c, constraints=opt.LinearConstraint(A, lo, hi), integrality=np.ones(nc),
                       bounds=opt.Bounds(0, 1))
        assert res.status == 0, res.message
        val = round(const - sum(res.x[j] for j in zs))
        return val, res.x

    def bound(self, obj, value):
        """Row ``obj <= value`` (i.e. ``sum z >= const - value``)."""
        zs, const = obj
        return ({j: 1 for j in zs}, const - value, math.inf)


def _eq15_lexicographic_full_network(g, starts, goals, T):
    """On mppflow's full-network model (Sec. III, corrected total-time objective), add the
    paper's literal eq. (14) variables ``y_i^t`` (t = 1..T, on the goal blue arcs) and solve
    (a) min eq. (15), then (b) min / max of the true total time among eq. (15) minimisers."""
    fm = build_full_network_model(g, starts, goals, T, objective="total_time")
    m, net = fm.model, fm.network
    n = len(starts)
    blue = {}
    for j, (a, _, kind) in enumerate(net.arcs):
        if kind == "blue":
            _, v, t, _ = net.nodes[a]
            blue[(v, t)] = j
    lit = []
    for i, gl in enumerate(goals):
        ys = {t: m.add_column(0.0, 1.0, "B") for t in range(1, T + 1)}
        for t in range(1, T + 1):
            x = fm.x[i][blue[(gl, t)]]
            if t == T:
                m.add_row([(ys[t], 1.0), (x, -1.0)], "==", 0.0)
            else:  # y_t = x_t AND y_{t+1}
                m.add_row([(ys[t], 1.0), (x, -1.0)], "<=", 0.0)
                m.add_row([(ys[t], 1.0), (ys[t + 1], -1.0)], "<=", 0.0)
                m.add_row([(ys[t], 1.0), (x, -1.0), (ys[t + 1], -1.0)], ">=", -1.0)
        lit += list(ys.values())
    # mppflow's corrected y[i,t] (t = 0..T-1) are only bounded above (enough when their sum
    # is maximised); for the "max true total time" stage, also bound them below:
    # y_t >= at_goal(t) + y_{t+1} - 1 (at_goal(0) = [start == goal], y_T = at_goal(T) = 1).
    for i, yi in enumerate(fm.y):
        gl, s = goals[i], starts[i]
        for t in range(T):
            nxt = [(yi[t + 1], -1.0)] if t + 1 < T else []
            if t == 0:
                if s == gl:
                    m.add_row([(yi[0], 1.0)] + nxt, ">=", 0.0 if nxt else 1.0)
                # else y[i,0] has upper bound 0
            else:
                x = fm.x[i][blue[(gl, t)]]
                m.add_row([(yi[t], 1.0), (x, -1.0)] + nxt, ">=", -1.0 if nxt else 0.0)
    m.set_objective([(c, 1.0) for c in lit], "max")
    sol = lp.solve(m, "highs")
    assert sol.status == "optimal"
    out = {"eq15": n * T - round(sol.objective)}
    m.add_row([(c, 1.0) for c in lit], ">=", float(n * T - out["eq15"]))
    true_terms = [(c, 1.0) for yi in fm.y for c in yi]
    for sense, obj_sense in (("min", "max"), ("max", "min")):
        m.set_objective(true_terms, obj_sense)
        sol = lp.solve(m, "highs")
        assert sol.status == "optimal"
        out["true_" + sense] = n * T - round(sol.objective)
        paths = fm.retrieve_paths(sol.x)
        assert is_path_set_valid(paths, g, starts, goals)
        assert eq15_literal(paths, goals) == out["eq15"]
        assert plan_metrics(paths, goals)["total_time"] == out["true_" + sense]
    return out


@pytest.mark.parametrize("which", [pytest.param("strict_counterexample", marks=pytest.mark.slow), "tie_counterexample"])
def test_eq15_counterexample(which):
    """mppflow's total-time objective is exactly sum t_f (21 on the strict counterexample);
    the paper's literal eq. (15) has minimum 17 and every minimiser has sum t_f = 22."""
    c = load("tor16_bounds_objectives")["eq15_search"][which]
    adj, idx = grid_adj_removed(c["rows"], c["cols"], c["removed"])
    s = [idx[v] for v in c["starts"]]
    t = [idx[v] for v in c["goals"]]
    T = c["T"]
    g = to_graph(adj)
    # mppflow
    r = S.solve_total_time(g, s, t, None, 0.0, T, horizon="single")
    # optimal at this T (one horizon: status "feasible" unless the lower bound is met)
    assert r.extra.get("optimal_at_T") or r.status == "optimal"
    assert r.objective == c["true_total_time_opt"]
    assert plan_metrics(r.paths, t)["total_time"] == c["true_total_time_opt"]
    assert is_path_set_valid(r.paths, g, s, t) and plan_error(adj, r.paths, s, t) is None
    fm = build_full_network_model(g, s, t, T, objective="total_time")
    sol = lp.solve(fm.model, "highs")
    assert len(s) * T - round(sol.objective) == c["true_total_time_opt"]
    lex = _eq15_lexicographic_full_network(g, s, t, T)
    assert lex == {"eq15": c["paper15_opt"], "true_min": c["min_true_total_among_paper15_optimal"],
                   "true_max": c["max_true_total_among_paper15_optimal"]}
    assert lex["true_min"] > c["true_total_time_opt"] or which == "tie_counterexample"
    assert joint_bfs(adj, s, t) == c["optimal_makespan"] == S.min_makespan(g, s, t).makespan
    # independent ILP
    ilp = IndependentILP(adj, s, t, T)
    assert ilp.solve(ilp.true_obj())[0] == c["true_total_time_opt"]
    e15, _ = ilp.solve(ilp.eq15_obj())
    assert e15 == c["paper15_opt"]
    row = ilp.bound(ilp.eq15_obj(), e15)
    assert ilp.solve(ilp.true_obj(), "min", [row])[0] == c["min_true_total_among_paper15_optimal"]
    assert ilp.solve(ilp.true_obj(), "max", [row])[0] == c["max_true_total_among_paper15_optimal"]
    # the recorded plans
    worst = to_paths([tuple(idx[v] for v in cfg) for cfg in c["paper15_worst_plan"]])
    assert plan_error(adj, worst, s, t) is None and is_path_set_valid(worst, g, s, t)
    assert eq15_literal(worst, t) == c["paper15_opt"]
    assert plan_metrics(worst, t)["total_time"] == c["max_true_total_among_paper15_optimal"]
    for key in ("true_total_optimal_plan", "paper15_optimal_plan"):
        if key in c:
            p = c[key]
            paths = to_paths([tuple(idx[v] for v in cfg) for cfg in p["plan"]])
            assert is_path_set_valid(paths, g, s, t) is p["valid"] is True
            assert arrival_times(paths, t) == p["arrival_times"]
            assert plan_metrics(paths, t)["total_time"] == p["sum_tf"]
            assert eq15_literal(paths, t) == p["paper_eq15_value_recomputed"]


# =====================================================================================
# TOR Sec. IV-E: T_min / T_max bounds
# =====================================================================================
def test_tmin_lower_bounds():
    """T_min = max_i |shortest path_i|: tight on Fig. 1, Fig. 14 and Fig. 8; 3 < 4 on Fig. 4(a)."""
    cases = []
    d = load("tor16_fig1_fig3_9puzzle")
    cases.append((grid_adj(3, 3), d["starts"], d["goals"], 4, 4))
    d = load("tor16_fig14_fig15_25puzzle")
    cases.append((grid_adj(5, 5), d["starts"], d["goals"], 7, 7))
    d = load("tor16_fig8_split")
    cases.append((grid_adj(4, 4), d["starts"], d["goals"], 4, 4))
    d = load("tor16_fig4_fig7_network")
    cases.append((edges_adj(6, d["graph"]["edges"]), d["starts"], d["goals"], 3, 4))
    for adj, s, t, tmin, topt in cases:
        g = to_graph(adj)
        lb = max(bfs_dist(adj, a)[b] for a, b in zip(s, t))
        assert lb == tmin
        assert get_makespan_lower_bound(g, s, t, MathRandom(1)) - 1 == tmin
        if len(s) < 25:  # the 25-puzzle is solved in test_25puzzle_solved_makespan_7
            assert plan((g, s, t)).lower_bounds["makespan"] == tmin
            assert joint_bfs(adj, s, t) == topt
            assert S.min_makespan(g, s, t).makespan == topt


def test_garage_corridor_tmax():
    """T_max = n T_opt: on the garage corridor (the original's
    getLongStraightWithOneGarageProblem) the minimum total distance (18) and total time (19)
    are the same for every T from T_opt = 10 to n T_opt = 20."""
    gc = load("tor16_bounds_objectives")["garage_corridor"]
    adj = edges_adj(10, gc["edges"])
    g = to_graph(adj)
    s, t = gc["starts"], gc["goals"]
    assert max(bfs_dist(adj, a)[b] for a, b in zip(s, t)) == gc["T_min_LB"] == 8
    assert get_makespan_lower_bound(g, s, t, MathRandom(1)) - 1 == 8
    assert joint_bfs(adj, s, t) == gc["optimal_makespan"] == 10
    assert S.min_makespan(g, s, t).makespan == 10
    assert gc["n*T_opt"] == 20
    for T, exp in gc["by_T"].items():
        T = int(T)
        assert S.solve_distance(g, s, t, 0.0, T, horizon="single").objective == exp["min_total_distance"] == 18
        assert S.solve_total_time(g, s, t, None, 0.0, T, horizon="single").objective \
            == exp["min_total_arrival_time"] == 19
        assert joint_dp(adj, s, t, T, "dist") == 18
        assert joint_dp(adj, s, t, T, "time") == 19
        assert joint_dp(adj, s, t, T, "eq15") == exp["min_paper_eq15"] == 17
    assert S.min_total_distance(g, s, t).objective == 18
    assert S.min_total_time(g, s, t).objective == 19
    assert all(joint_dp(adj, s, t, T, "time") is None for T in (8, 9))  # below T_opt


def test_star_graph_main_case3():
    c = load("tor16_bounds_objectives")["star_main_case3"]
    adj = edges_adj(4, c["edges"])
    g = to_graph(adj)
    assert max(bfs_dist(adj, a)[b] for a, b in zip(c["starts"], c["goals"])) == c["T_min_LB"] == 2
    assert joint_bfs(adj, c["starts"], c["goals"]) == c["optimal_makespan"] == 4
    r = S.min_makespan(g, c["starts"], c["goals"])
    assert r.makespan == 4 and is_path_set_valid(r.paths, g, c["starts"], c["goals"])
    paths = to_paths([tuple(x) for x in c["plan"]])
    assert plan_error(adj, paths, c["starts"], c["goals"]) is None
    assert is_path_set_valid(paths, g, c["starts"], c["goals"])


# =====================================================================================
# N^2-puzzle state spaces and branching factors (TOR Sec. VI-B, ICRA Sec. VI-A)
# =====================================================================================
def test_npuzzle_state_space_sizes():
    d = load("icra_npuzzle_state_space")
    for k, v in d["state_counts"].items():
        assert math.isclose(math.factorial(int(k)), v, rel_tol=1e-12)
    assert 1e25 <= math.factorial(25) < 2e25  # "about 10^25"
    assert f"{math.factorial(36):.1e}" == "3.7e+41"


@pytest.mark.parametrize("shape", ["2x2", "2x3", "3x3", "3x4", "4x4"])
def test_npuzzle_legal_joint_moves(shape):
    """Legal joint moves of a fully occupied grid: 951 for the 16-puzzle (the paper's "over
    1000" is wrong). Every enumerated move is accepted by mppflow's validator; on small grids
    every other assignment of targets is rejected."""
    d = load("icra_npuzzle_state_space")
    rows, cols = map(int, shape.split("x"))
    adj = grid_adj(rows, cols)
    g = to_graph(adj)
    cfg = tuple(range(rows * cols))
    succ = successors(adj, cfg)
    assert len(succ) == d["legal_moves_including_all_stay"][shape]
    br = load("tor16_puzzle_branching")["counts"]
    if shape in br:
        assert len(succ) == br[shape]["legal_joint_moves_incl_identity"]
    for c in succ:
        assert is_path_set_valid([[a, b] for a, b in zip(cfg, c)], g, cfg, c)
    if rows * cols <= 6:
        legal = set(succ)
        for c in itertools.product(*[[v] + adj[v] for v in cfg]):
            ok = is_path_set_valid([[a, b] for a, b in zip(cfg, c)], g, cfg, c)
            assert ok is (c in legal) is (step_error(adj, cfg, c) is None)
    else:
        rng = random.Random(2016)
        legal = set(succ)
        for _ in range(3000):
            c = tuple(rng.choice([v] + adj[v]) for v in cfg)
            ok = is_path_set_valid([[a, b] for a, b in zip(cfg, c)], g, cfg, c)
            assert ok is (c in legal)
    if shape == "4x4":
        assert len(succ) - 1 == d["branching_factor_excluding_all_stay"]["16-puzzle"] == 950 < 1000


@pytest.mark.slow
@pytest.mark.parametrize("backend", [pytest.param("gurobi", marks=[pytest.mark.gurobi, needs_gurobi]), "highs"])
@pytest.mark.parametrize("k", [0, 4])
def test_16puzzle_random_instances(k, backend):
    """Seeded random 16-puzzles (a check of "generally 6 steps"; not a paper figure)."""
    inst = load("tor16_16puzzle_random_check")["instances"][k]
    g, adj = create_2d_grid_graph(4, 4), grid_adj(4, 4)
    goals = list(range(16))
    assert max(bfs_dist(adj, a)[b] for a, b in zip(inst["starts"], goals)) == inst["lower_bound"]
    r = S.min_makespan(g, inst["starts"], goals, backend=backend, solve_relaxed_first=True)
    assert r.makespan == inst["optimal_makespan"]
    assert plan_error(adj, r.paths, inst["starts"], goals) is None


# =====================================================================================
# ICRA 2013 Fig. 1: time-optimal versus distance-optimal
# =====================================================================================
def icra_fig1_graph(t):
    names = ["S1", "S2", "G1", "G2", "L", "R"]
    straight = ["L"] + [f"p{i}" for i in range(1, t)] + ["R"]
    arc = ["L"] + [f"q{i}" for i in range(1, int(1.5 * t))] + ["R"]
    names += straight[1:-1] + arc[1:-1]
    ix = {v: i for i, v in enumerate(names)}
    E = [("S2", "L"), ("L", "G1"), ("S1", "R"), ("R", "G2")]
    E += list(zip(straight, straight[1:])) + list(zip(arc, arc[1:]))
    adj = edges_adj(len(names), [(ix[a], ix[b]) for a, b in E])
    return adj, [ix["S1"], ix["S2"]], [ix["G1"], ix["G2"]]


def _lex_dijkstra(adj, s, g, key):
    """Lexicographic optimum over joint states: ``key="time"`` minimises (steps, moves),
    ``"dist"`` minimises (moves, steps)."""
    s, g = tuple(s), tuple(g)
    best = {s: (0, 0)}
    pq = [((0, 0), s)]
    while pq:
        c, cfg = heapq.heappop(pq)
        if best.get(cfg) != c:
            continue
        if cfg == g:
            return c
        for c2 in successors(adj, cfg):
            mv = sum(1 for a, b in zip(cfg, c2) if a != b)
            if mv == 0:
                continue
            nc = (c[0] + 1, c[1] + mv) if key == "time" else (c[0] + mv, c[1] + 1)
            if nc < best.get(c2, (math.inf, 0)):
                best[c2] = nc
                heapq.heappush(pq, (nc, c2))
    return None


@pytest.mark.parametrize("t", [2, 4])
def test_icra_fig1_time_versus_distance(t):
    d = load("icra_fig1_time_vs_distance")
    assert t in d["verified_t_values"]
    adj, s, gl = icra_fig1_graph(t)
    g = to_graph(adj)
    mk_time, d_time = int(1.5 * t + 2), int(2.5 * t + 4)
    mk_dist, d_dist = 2 * t + 3, 2 * t + 4
    assert _lex_dijkstra(adj, s, gl, "time") == (mk_time, d_time)
    assert _lex_dijkstra(adj, s, gl, "dist") == (d_dist, mk_dist)
    assert joint_dp(adj, s, gl, mk_time, "dist") == d_time
    assert joint_dp(adj, s, gl, mk_dist - 1, "dist") > d_dist
    assert joint_dp(adj, s, gl, mk_dist, "dist") == d_dist
    r = S.min_makespan(g, s, gl)
    assert r.makespan == mk_time
    assert S.solve_distance(g, s, gl, 0.0, mk_time, horizon="single").objective == d_time
    assert S.solve_distance(g, s, gl, 0.0, mk_dist - 1, horizon="single").objective > d_dist
    r = S.solve_distance(g, s, gl, 0.0, mk_dist, horizon="single")
    assert r.objective == d_dist and plan_error(adj, r.paths, s, gl) is None
    assert S.min_total_distance(g, s, gl).objective == d_dist
    if t == d["java_original_check"]["t"]:
        j = d["java_original_check"]
        assert (mk_time, d_time, d_dist) == (j["makespan"], j["min_total_distance_at_T=1.5t+2"],
                                             j["min_total_distance_at_T=2t+3"])


def test_icra_fig1_single_horizon_is_not_optimal():
    """t = 2: at the optimal makespan T* = 5 the best total distance is 9 and the best max
    distance 5, while longer horizons reach 8 and 4. ``plan(..., horizon="single")`` returns
    the T* optimum with status "feasible"; the default horizon search finds the optimum."""
    adj, s, gl = icra_fig1_graph(2)
    g = to_graph(adj)
    for obj, at_T, best in (("total_distance", 9, 8), ("max_distance", 5, 4), ("total_time", 9, 9)):
        one = plan((g, s, gl), obj, horizon="single")
        assert (one.value, one.makespan, one.extra["horizon"]) == (at_T, 5, 5)
        assert one.status == ("optimal" if at_T == one.lower_bounds[obj] else "feasible") == "feasible"
        full = plan((g, s, gl), obj)
        assert (full.value, full.status) == (best, "optimal")
        assert plan_error(adj, full.paths, s, gl) is None


def test_icra_fig1_horizon_search_adopts_the_plan_at_T_hi(monkeypatch):
    """Algorithm 1 without the joint-state shortcut: the plan at T0 = T* (distance 9, max
    distance 5) is beaten at T_hi, and the search keeps the T_hi plan, trimmed (8 at T = 9;
    4 at T = 10). The total time has T_hi = T0 here (9 - 8 + 4 = 5)."""
    adj, s, gl = icra_fig1_graph(2)
    g = to_graph(adj)
    monkeypatch.setattr(S, "JOINT_SEARCH_MAX_CONFIGS", 0)
    for f, want, T0, T_hi in ((S.min_total_distance, 8, 5, 9), (S.min_max_distance, 4, 5, 10),
                              (S.min_total_time, 9, 5, 5)):
        r = f(g, s, gl)
        info = r.extra["horizon_search"]
        assert (r.objective, r.status, info["T0"], info["T_hi"]) == (want, "optimal", T0, T_hi), f.__name__
        assert "proof" not in info and plan_error(adj, r.paths, s, gl) is None
        assert r.T == len(r.paths[0]) - 1 and (r.T > T0) == (T_hi > T0)
    assert _lex_dijkstra(adj, s, gl, "dist")[0] == 8


def test_icra_fig1_gap_without_proof_is_feasible(monkeypatch):
    """``plan(..., gap > 0)``: a solve that stops within the gap proves nothing, so the status
    is "feasible" (unless the lower bound or the joint-state search confirms the value; the
    latter is switched off here). The backend is made to report "optimal" without a bound,
    as a MIP solver does when it stops within the gap; with ``gap = 0`` its "optimal" is a
    proof."""
    adj, s, gl = icra_fig1_graph(2)
    g = to_graph(adj)
    monkeypatch.setattr(S, "JOINT_SEARCH_MAX_CONFIGS", 0)
    run = S._run

    def within_gap(m, backend, params, time_limit, mip_gap):
        sol = run(m, backend, params, time_limit, mip_gap)
        if mip_gap and sol.status == "optimal":
            sol.bound = None
            sol.mip_gap = mip_gap
        return sol

    monkeypatch.setattr(S, "_run", within_gap)
    for obj, best in (("total_time", 9), ("total_distance", 8), ("max_distance", 4)):
        r = plan((g, s, gl), obj, gap=0.5)
        assert r.paths is not None and r.status == "feasible" and r.value >= best, obj
        r0 = plan((g, s, gl), obj)
        assert (r0.status, r0.value) == ("optimal", best), obj


# =====================================================================================
# STAR 2013
# =====================================================================================
def test_star_fig1_escape():
    """STAR Fig. 1 (the escape problem): (a) all 10 evaders escape, (b) only 10 of 11."""
    d = load("star_fig1_escape")
    N = 6

    def network(ev):
        node = {}

        def nd(k):
            return node.setdefault(k, len(node))

        arcs = []
        src, snk = nd("S"), nd("T")
        for r in range(N):
            for c in range(N):
                arcs.append((nd(("in", r, c)), nd(("out", r, c)), 1))
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    if 0 <= r + dr < N and 0 <= c + dc < N:
                        arcs.append((nd(("out", r, c)), nd(("in", r + dr, c + dc)), 1))
                if r in (0, N - 1) or c in (0, N - 1):
                    arcs.append((nd(("out", r, c)), snk, 1))
        for r, c in ev:
            arcs.append((src, nd(("in", r, c)), 1))
        return len(node), arcs, src, snk

    a, b = [tuple(p) for p in d["a_evaders"]], [tuple(p) for p in d["b_evaders"]]
    for backend in ("highs", "builtin"):
        assert lp_max_flow(*network(a), backend) == len(a) == 10
        assert lp_max_flow(*network(b), backend) == d["b_max_escapes"] == 10 < len(b)
    assert max_flow(*network(a)) == 10 and max_flow(*network(b)) == 10
    paths = [[tuple(p) for p in q] for q in d["a_bold_paths"]]
    assert sorted(q[0] for q in paths) == sorted(a)
    used = [p for q in paths for p in q]
    assert len(used) == len(set(used))
    for q in paths:
        assert q[-1][0] in (0, N - 1) or q[-1][1] in (0, N - 1)
        assert all(abs(u[0] - v[0]) + abs(u[1] - v[1]) == 1 for u, v in zip(q, q[1:]))


def test_star_fig2_dynamic_flow():
    """STAR Fig. 2: the time-expanded network of a dynamic flow (transit times 0, 1, 2)."""
    d = load("star_fig2_dynamic_flow")
    V = d["vertices"]

    def expand(T):
        node = {}

        def nd(k):
            return node.setdefault(k, len(node))

        arcs, black, green = [], 0, 0
        for u, v, c in d["static_edges"]:
            for t in range(T + 1 - c):
                arcs.append((nd((u, t)), nd((v, t + c)), 1))
                black += 1
        for v in V:
            for t in range(T):
                arcs.append((nd((v, t)), nd((v, t + 1)), 1))
                green += 1
        src, snk = nd("SRC"), nd("SNK")
        arcs.append((src, nd(("s+", 0)), 10 ** 6))
        for t in range(T + 1):
            arcs.append((nd(("s-", t)), snk, 10 ** 6))
        return (len(node), arcs, src, snk), black, green

    _, b, g = expand(4)
    assert (b, g) == (d["fig2b_T4"]["black_arcs"], d["fig2b_T4"]["green_arcs"]) == (19, 16)
    for T, val in d["max_flow_by_T"].items():
        net = expand(int(T))[0]
        assert max_flow(*net) == val
        assert lp_max_flow(*net, "highs") == val


def test_star_theorem8_bound():
    """T = n + ell - 1 suffices for the unlabeled problem (tight on a family and on Fig. 3)
    but is not necessary. Labeled optima from mppflow are upper bounds of the unlabeled ones."""
    d = load("star_theorem8_bound")
    c = d["counterexample_to_necessity"]
    adj = edges_adj(5, c["edges"])

    def ell(adj, S_, G_):
        return max(bfs_dist(adj, u)[v] for u in S_ for v in G_)

    assert ell(adj, c["starts"], c["goals"]) == c["ell"] == 3
    assert len(c["starts"]) + c["ell"] - 1 == c["bound"] == 4
    assert unlabeled_bfs(adj, c["starts"], c["goals"]) == c["unlabeled_min_makespan"] == 1
    assert S.min_makespan(to_graph(adj), c["starts"], c["goals"]).makespan == 1
    for case in d["tight_family"]["cases"]:
        n, L = case["n"], case["L"]
        names = ["h", "h2"] + [f"c{i}" for i in range(1, L + 1)] + [f"s{i}" for i in range(n)] + [f"g{i}" for i in range(n)]
        ix = {v: i for i, v in enumerate(names)}
        chain = ["h"] + [f"c{i}" for i in range(1, L + 1)] + ["h2"]
        E = list(zip(chain, chain[1:])) + [("h", f"s{i}") for i in range(n)] + [("h2", f"g{i}") for i in range(n)]
        adj = edges_adj(len(names), [(ix[a], ix[b]) for a, b in E])
        S_ = [ix[f"s{i}"] for i in range(n)]
        G_ = [ix[f"g{i}"] for i in range(n)]
        e = ell(adj, S_, G_)
        assert e == L + 3
        u = unlabeled_bfs(adj, S_, G_)
        assert u == n + e - 1, case
        assert S.min_makespan(to_graph(adj), S_, G_).makespan >= u
    f3 = d["fig3_instance_attains_bound"]
    d4, g4, adj4 = fig4()
    assert ell(adj4, d4["starts"], d4["goals"]) == f3["ell"] == 3
    assert unlabeled_bfs(adj4, d4["starts"], d4["goals"]) == f3["unlabeled_min_makespan"] == f3["bound"] == 4
    assert S.min_makespan(g4, d4["starts"], d4["goals"]).makespan == 4
    assert load("icra_fig4a_fig5_network")["unlabeled_STAR_problem2"]["unlabeled_min_makespan"] == 4


def test_star_cug_radius():
    """STAR Sec. 2: discs moving on non-conflicting unit grid edges stay sqrt(2)/2 apart
    (L-shaped following at the edge midpoints), so the radius must be below sqrt(2)/4."""
    d = load("star_cug_radius")
    pts = [(x, y) for x in range(3) for y in range(3)]
    dedges = [(p, q) for p in pts for q in pts if abs(p[0] - q[0]) + abs(p[1] - q[1]) == 1]
    best, arg = math.inf, None
    for (a, b), (c, e) in itertools.combinations(dedges, 2):
        if a == c or b == e or (a == e and b == c):
            continue  # same tail, meet or head-on
        # |P(s) - Q(s)|^2 is a quadratic in s on [0, 1]; check the ends and the vertex
        dx0, dy0 = a[0] - c[0], a[1] - c[1]
        vx, vy = (b[0] - a[0]) - (e[0] - c[0]), (b[1] - a[1]) - (e[1] - c[1])
        cand = [0.0, 1.0]
        if vx or vy:
            s = -(dx0 * vx + dy0 * vy) / (vx * vx + vy * vy)
            if 0 < s < 1:
                cand.append(s)
        for s in cand:
            dist = math.hypot(dx0 + s * vx, dy0 + s * vy)
            if dist < best - 1e-12:
                best, arg = dist, s
    assert math.isclose(best, d["min_center_distance_over_all_nonconflicting_edge_pairs_on_grid"])
    assert math.isclose(best / 2, d["radius_limit"]) and math.isclose(best / 2, math.sqrt(2) / 4)
    assert arg == 0.5
