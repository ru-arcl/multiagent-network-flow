"""Parity fixtures for the browser engine (``docs/js/engine.js`` and ``docs/js/lp.js``).

``python -m mppflow.parity`` regenerates ``tests/parity/*.json``: inputs plus the outputs of
this package for every function the JavaScript engine ports (fixed mode, ``backend="builtin"``).
``npm test`` (``docs/js/test/parity.test.js``) recomputes every case in JavaScript and requires
**exact** equality: the same integers, the same strings, and bit-identical floats (the builtin
simplex is a line-by-line port, so it performs the same pivots in the same order).
``tests/test_parity.py`` checks that the committed fixtures are current (it regenerates them in
memory and compares) and, when Node.js is available, runs ``npm test``.

Fixture files (each ``{"meta": ..., "cases": {group: [case, ...]}}``):

==============  ========  ===================================================================
file            tier      contents
==============  ========  ===================================================================
java.json       fast      MathRandom streams, ``nextInt``, ``JavaIntHashSet`` iteration orders
                          (treeified bins included), ``Double.toString``, ``round``
graphs.json     fast      grids, grids with holes, ``create_grid_problem``,
                          ``create_n2_puzzle``, ``from_adjacency`` (with draw counts)
pathfinder.json fast      BFS distances, ``find_goals_in_distance_range``, randomized shortest
                          paths, lower bounds
network.json    fast      reachability, compact arcs, full / compact network JSON
models.json     fast      canonical model text (SHA-256, sizes; the full text of small models)
                          for every objective, including the 24x18 demo instance
lp.json         fast      builtin LP/MILP solutions of small demo models
planners.json   fast      split goals (incl. the 16-way split of the 24x18 instance), the four
                          optimal planners, the joint-state searches (feasibility, optimum),
                          the k-way split, validation, rotations, metrics
lp_slow.json    slow      builtin solutions of the 9-puzzle and 16-puzzle models
planners_slow   slow      the 9-puzzle planned with k = 1, 2
==============  ========  ===================================================================

The slow tier takes a few minutes in Python (the builtin solver is pure Python); the same
cases take seconds in Node.js. ``python -m mppflow.parity --fast`` rewrites the fast files only.

Non-finite floats are written as the strings ``"__Infinity__"``, ``"__-Infinity__"`` and
``"__NaN__"`` (JSON has no literal for them).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import random
import sys
import time

from ._java import JavaIntHashSet, JavaRandom, MathRandom, java_double_str
from .graph import Graph, convert_graph, create_2d_grid_graph, create_generic_2d_grid_graph_with_holes, \
    create_main_case3_graph
from .lp import builtin
from .lp.linear import golden_number
from .model import (
    build_distance_model,
    build_max_distance_model,
    build_time_model,
    build_time_r_model,
    build_total_time_model,
)
from .network import CompactNetwork, FullNetwork, compact_arcs, update_reachable_sets
from .pathfinder import (
    find_goals_in_distance_range,
    find_shortest_paths,
    get_distances_to_all_vertices,
    get_makespan_lower_bound,
    get_total_time_lower_bound,
)
from .planner import (
    PathPlanner,
    _lower_bounds,
    has_cycles,
    path_set_error,
    plan,
    plan_metrics,
    split_paths,
)
from .problem import create_grid_problem, create_n2_puzzle
from .solvers import (JOINT_SEARCH_BUDGET, joint_feasibility, joint_optimum, min_makespan, min_max_distance,
                      min_total_distance, min_total_time)

__all__ = ["FAST", "SLOW", "FILES", "generate", "dumps", "write", "main", "INSTANCES"]

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(os.path.dirname(HERE), "tests", "parity")
"""``tests/parity`` of the checkout this module runs from (see :func:`default_out`)."""
VERSION = 1

_TIMES: list = []
"""(label, seconds) of the slower cases generated in this process (printed by ``main``)."""

TEXT_LIMIT = 6000
"""Canonical model texts up to this many characters are stored in full (else only the hash)."""


# ------------------------------------------------------------------------ encoding
def _clean(x):
    """JSON-able copy: tuples -> lists, non-finite floats -> strings, dict keys -> str."""
    if isinstance(x, float):
        if math.isnan(x):
            return "__NaN__"
        if math.isinf(x):
            return "__Infinity__" if x > 0 else "__-Infinity__"
        return x
    if isinstance(x, (list, tuple)):
        return [_clean(v) for v in x]
    if isinstance(x, dict):
        return {str(k): _clean(v) for k, v in x.items()}
    return x


def dumps(obj) -> str:
    """The fixture text: compact JSON, one top-level group per line."""
    obj = _clean(obj)
    parts = []
    for k, v in obj.items():
        parts.append(json.dumps(k) + ":" + json.dumps(v, separators=(",", ":"), allow_nan=False))
    return "{\n" + ",\n".join(parts) + "\n}\n"


def _sha(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _graph_json(g: Graph) -> dict:
    return {"n": g.n, "adj": [list(g.adj[v]) for v in range(g.n)],
            "xy": [None if g.xy.get(v) is None else [g.xy[v][0], g.xy[v][1]] for v in range(g.n)]}


# ------------------------------------------------------------------------ instances
def _fig4_graph() -> Graph:
    edges = [(0, 2), (1, 2), (2, 4), (3, 4), (4, 5)]
    adj = [[] for _ in range(6)]
    for u, w in edges:
        adj[u].append(w)
        adj[w].append(u)
    xy = [(0, 0), (0, 2), (1, 1), (4, 0), (3, 1), (4, 2)]
    return Graph.from_adjacency(adj, xy)


def _instances() -> dict:
    """The named instances shared by the fixtures: ``{name: (graph, starts, goals)}``."""
    out = {}
    out["fig1_9puzzle"] = (create_2d_grid_graph(3, 3), [2, 4, 5, 1, 8, 6, 7, 3, 0], list(range(9)))
    out["fig4a_tree"] = (_fig4_graph(), [3, 5], [0, 1])
    out["fig8_split"] = (create_2d_grid_graph(4, 4), [13, 8], [0, 7])
    star = create_main_case3_graph()
    out["star_swap"] = (Graph.from_adjacency([star.adj[v] for v in range(4)]), [0, 3], [3, 0])
    p = create_n2_puzzle(4, MathRandom(2016))
    out["puzzle16_s2016"] = (p.graph, p.starts, p.goals)
    p = create_grid_problem(5, 5, 0.2, 4, MathRandom(3))
    out["grid5x5_s3"] = (p.graph, p.starts, p.goals)
    p = create_grid_problem(6, 6, 0.1, 8, MathRandom(7))
    out["grid6x6_s7"] = (p.graph, p.starts, p.goals)
    p = create_grid_problem(4, 6, 0.0, 6, MathRandom(5))
    out["grid4x6_s5"] = (p.graph, p.starts, p.goals)
    p = create_grid_problem(8, 8, 0.0, 40, MathRandom(11))
    out["crowded8x8_s11"] = (p.graph, p.starts, p.goals)
    p = create_grid_problem(24, 18, 0.25, 50, MathRandom(1))
    out["grid24x18_s1"] = (p.graph, p.starts, p.goals)
    # a one-way graph (fixed mode prunes with reversed-arc BFS distances)
    out["oneway_cycle"] = (Graph.from_adjacency([[1], [2], [3], [0]]), [0], [3])
    return out


INSTANCES = None


def _inst():
    global INSTANCES
    if INSTANCES is None:
        INSTANCES = _instances()
    return INSTANCES


def _instance_json(name: str) -> dict:
    g, s, t = _inst()[name]
    d = _graph_json(g)
    return {"graph": d, "starts": list(s), "goals": list(t)}


# ------------------------------------------------------------------------ java.json
def _java() -> dict:
    rng_cases = []
    for seed in (0, 1, 7, 42, 2016, -1, 123456789012, 2 ** 47 + 5):
        r = MathRandom(seed)
        vals = [r.random() for _ in range(40)]
        jr = JavaRandom(seed)
        ints = [jr.next_int(b) for b in (1, 2, 3, 7, 10, 16, 100, 1000, 2 ** 30, 2 ** 31 - 1, 5, 64)]
        ints.append(jr.next_int())
        rng_cases.append({"seed": seed, "random": vals, "draws": r.draws, "state": r.state(),
                          "next_int": ints, "state_after_ints": jr.state()})
    hs_cases = []
    pools = {
        "small": lambda R: R.randrange(0, 200),
        "mult64": lambda R: 64 * R.randrange(0, 60),
        "mult16": lambda R: 16 * R.randrange(0, 200),
        "spread": lambda R: R.randrange(-2 ** 31, 2 ** 31),
        "collide": lambda R: (R.randrange(0, 40) << 16) ^ R.randrange(0, 4),
        "mixed": lambda R: R.choice((64 * R.randrange(0, 90), R.randrange(0, 5000), 1024 * R.randrange(0, 30))),
    }
    for name, draw in pools.items():
        for seed in range(3):
            R = random.Random(f"{name}-{seed}")
            hs = JavaIntHashSet()
            ops = []
            snaps = []
            for step in range(400):
                k = draw(R)
                op = "add" if R.random() < 0.7 else "remove"
                if op == "remove" and len(hs) and R.random() < 0.6:
                    lst = hs.to_list()
                    k = lst[R.randrange(len(lst))]
                ret = hs.add(k) if op == "add" else hs.remove(k)
                ops.append([0 if op == "add" else 1, k, 1 if ret else 0])
                if step % 50 == 49:
                    snaps.append([step, hs.to_list()])
            hs_cases.append({"name": f"{name}-{seed}", "ops": ops, "snapshots": snaps, "final": hs.to_list(),
                             "size": len(hs)})
    vals = [0.0, -0.0, 1.0, -1.0, 0.5, 0.1, 1e-3, 1.5e-3, 9.99e-4, 1e-4, 1234567.0, 1e7, 12345678.9, 1e15,
            1.25e16, 3.141592653589793, -2.718281828459045, 1 / 3, 2 / 3, 100.0, 0.30000000000000004,
            123456.789, 9999999.999, 1e-300, 5e-324, 1.7976931348623157e308, float("inf"), float("-inf"),
            float("nan")]
    dbl = [[v, java_double_str(v), golden_number(v) if not math.isnan(v) else None] for v in vals]
    rounds = [[v, round(v)] for v in (0.5, 1.5, 2.5, -0.5, -1.5, 2.4999999, 2.5000001, 3.0, -2.5, 7.5, 1e15 + 0.5)]
    return {"rng": rng_cases, "hashset": hs_cases, "doubles": dbl, "round": rounds}


# ------------------------------------------------------------------------ graphs.json
def _graphs() -> dict:
    grids = []
    for r, c in ((1, 1), (1, 5), (5, 1), (3, 3), (4, 4), (2, 7), (6, 5)):
        grids.append({"rows": r, "cols": c, "expected": _graph_json(create_2d_grid_graph(r, c))})
    holes = []
    for r, c, pct, seed in ((5, 5, 0.2, 1), (8, 8, 0.25, 2), (6, 9, 0.3, 3), (24, 18, 0.25, 4), (3, 7, 0.1, 5),
                            (10, 10, 0.0, 6), (12, 12, 0.35, 7)):
        rng = MathRandom(seed)
        g = convert_graph(create_generic_2d_grid_graph_with_holes(r, c, pct, rng))
        holes.append({"rows": r, "cols": c, "percent": pct, "seed": seed, "expected": _graph_json(g),
                      "draws": rng.draws})
    problems = []
    for col, row, frac, n, seed in ((24, 18, 0.25, 50, 1), (24, 18, 0.25, 50, 2), (24, 18, 0.1, 100, 3),
                                    (8, 8, 0.0, 40, 11), (15, 15, 0.2, 15, 4), (6, 6, 0.1, 8, 7),
                                    (5, 5, 0.2, 4, 3), (4, 6, 0.0, 6, 5), (16, 16, 0.2, 60, 9),
                                    (32, 32, 0.2, 30, 10)):
        rng = MathRandom(seed)
        p = create_grid_problem(col, row, frac, n, rng)
        problems.append({"col": col, "row": row, "frac": frac, "n": n, "seed": seed,
                         "expected": {"graph": _graph_json(p.graph), "starts": p.starts, "goals": p.goals},
                         "draws": rng.draws})
    puzzles = []
    for n, seed in ((3, 1), (3, 9), (4, 2016), (4, 5), (5, 25), (6, 3)):
        rng = MathRandom(seed)
        p = create_n2_puzzle(n, rng)
        puzzles.append({"n": n, "seed": seed, "expected": {"graph": _graph_json(p.graph), "starts": p.starts,
                                                           "goals": p.goals}, "draws": rng.draws})
    froms = []
    for adj, xy in (([[1], [0, 2], [1]], None), ([[1], [2], [3], [0]], None), ([[3, 1], [], [0, 0, 2]], [[0, 0], None, [2.5, 1]]),
                    ([[1, 2, 3], [0], [0], [0]], [[1, 1], [0, 1], [2, 1], [1, 0]])):
        g = Graph.from_adjacency(adj, xy)
        froms.append({"adj": adj, "xy": xy, "expected": _graph_json(g)})
    return {"grid": grids, "holes": holes, "grid_problem": problems, "n2_puzzle": puzzles, "from_adjacency": froms}


# ------------------------------------------------------------------------ pathfinder.json
def _pathfinder() -> dict:
    inst = _inst()
    bfs, ranges, sps, lbs = [], [], [], []
    for name in ("fig4a_tree", "fig8_split", "grid6x6_s7", "grid24x18_s1", "oneway_cycle", "star_swap"):
        g, s, t = inst[name]
        for v in sorted(set([0, g.n - 1, g.n // 2] + list(s[:3]))):
            bfs.append({"instance": name, "start": v, "expected": get_distances_to_all_vertices(g, v),
                        "expected_unreachable": get_distances_to_all_vertices(g, v, unreachable=-1)})
            for dmin, dmax in ((0, 0), (1, 1), (0, 2), (2, 3), (1, 5), (3, 3), (0, 100)):
                ranges.append({"instance": name, "start": v, "dmin": dmin, "dmax": dmax,
                               "expected": find_goals_in_distance_range(g, v, dmin, dmax)})
        for seed in (1, 2, 99):
            rng = MathRandom(seed)
            paths = find_shortest_paths(g, s, t, rng)
            d1 = rng.draws
            mlb = tlb = None
            try:
                mlb = get_makespan_lower_bound(g, s, t, rng)
                d2 = rng.draws
                tlb = get_total_time_lower_bound(g, s, t, rng)
            except Exception:  # pragma: no cover - all fixture instances are solvable
                d2 = rng.draws
            sps.append({"instance": name, "seed": seed, "expected": paths, "draws": d1, "makespan_lb": mlb,
                        "draws_after_mlb": d2, "total_time_lb": tlb, "draws_after_tlb": rng.draws})
        lbs.append({"instance": name, "expected": _lower_bounds(g, s, t)})
    return {"bfs": bfs, "goals_in_range": ranges, "shortest_paths": sps, "lower_bounds": lbs}


# ------------------------------------------------------------------------ network.json
def _network() -> dict:
    inst = _inst()
    reach, arcs, nets = [], [], []
    for name, Ts in (("fig4a_tree", (2, 4, 5)), ("fig8_split", (4, 5)), ("fig1_9puzzle", (4,)),
                     ("grid6x6_s7", (6,)), ("oneway_cycle", (3, 4)), ("grid24x18_s1", (30,))):
        g, s, t = inst[name]
        for T in Ts:
            r = update_reachable_sets(g, s, t, T)
            reach.append({"instance": name, "T": T, "dist_start": r.dist_start, "dist_goal": r.dist_goal})
            a = list(compact_arcs(g, s, r))
            txt = json.dumps(a, separators=(",", ":"))
            entry = {"instance": name, "T": T, "count": len(a), "sha256": _sha(txt)}
            if len(a) <= 400:
                entry["arcs"] = a
            arcs.append(entry)
    for name, T in (("fig4a_tree", 2), ("fig4a_tree", 4), ("fig8_split", 4), ("star_swap", 3), ("oneway_cycle", 3)):
        g, s, t = inst[name]
        try:
            nets.append({"instance": name, "T": T, "variant": "full",
                         "expected": FullNetwork.build(g, s, t, T).to_json()})
        except ValueError as e:  # the full network needs an undirected graph with wait loops
            nets.append({"instance": name, "T": T, "variant": "full", "error": str(e)})
        nets.append({"instance": name, "T": T, "variant": "compact", "prune": True,
                     "expected": CompactNetwork.build(g, s, t, T, prune=True).to_json()})
        nets.append({"instance": name, "T": T, "variant": "compact", "prune": False,
                     "expected": CompactNetwork.build(g, s, t, T).to_json()})
    counts = []
    for name, T in (("fig1_9puzzle", 4), ("grid6x6_s7", 6), ("puzzle16_s2016", 5)):
        g, s, t = inst[name]
        counts.append({"instance": name, "T": T, "full": FullNetwork.build(g, s, t, T).counts(),
                       "compact": CompactNetwork.build(g, s, t, T, prune=True).counts()})
    return {"reachability": reach, "compact_arcs": arcs, "network": nets, "network_counts": counts}


# ------------------------------------------------------------------------ models.json
_BUILDERS = {
    "makespan": lambda g, s, t, T, integer, so: build_time_model(g, s, t, T, integer, so),
    "time_r": lambda g, s, t, T, integer, so: build_time_r_model(g, s, t, T, integer, so),
    "total_time": lambda g, s, t, T, integer, so: build_total_time_model(g, s, t, T, integer),
    "total_distance": lambda g, s, t, T, integer, so: build_distance_model(g, s, t, T, integer),
    "max_distance": lambda g, s, t, T, integer, so: build_max_distance_model(g, s, t, T, integer),
}


def _model(name, kind, T, integer=True, set_optimal=True):
    g, s, t = _inst()[name]
    return _BUILDERS[kind](g, s, t, T, integer, set_optimal)


def _models() -> dict:
    cases = []
    spec = [
        ("fig4a_tree", (3, 4, 5)), ("fig8_split", (4, 5, 8)), ("star_swap", (3, 4, 5)),
        ("fig1_9puzzle", (3, 4, 5)), ("puzzle16_s2016", (5, 6)), ("grid5x5_s3", (4, 6)),
        ("grid6x6_s7", (5, 7)), ("grid4x6_s5", (5,)), ("crowded8x8_s11", (8,)), ("oneway_cycle", (3, 4)),
        ("grid24x18_s1", (27,)),
    ]
    for name, Ts in spec:
        for T in Ts:
            variants = [("makespan", True, True), ("makespan", False, False), ("makespan", True, False),
                        ("total_time", True, True), ("total_distance", True, True), ("max_distance", True, True)]
            if name in ("fig8_split", "star_swap", "fig4a_tree"):
                variants += [("time_r", True, True), ("total_time", False, True), ("total_distance", False, True)]
            if name == "grid24x18_s1":
                variants = [("makespan", True, True), ("total_distance", True, True)]
            for kind, integer, so in variants:
                m = _model(name, kind, T, integer, so)
                text = m.canonical_text()
                e = {"instance": name, "kind": kind, "T": T, "integer": integer, "set_optimal": so,
                     "ncols": m.model.num_columns, "nrows": m.model.num_rows, "sha256": _sha(text)}
                if len(text) <= TEXT_LIMIT:
                    e["text"] = text
                cases.append(e)
    return {"models": cases}


# ------------------------------------------------------------------------ lp.json
def _sol_json(sol) -> dict:
    return {"status": sol.status, "objective": sol.objective, "x": sol.x, "bound": sol.bound,
            "mip_gap": sol.mip_gap, "sol_count": sol.sol_count, "node_count": sol.node_count,
            "iterations": sol.iterations, "raw_status": sol.raw_status}


def _lp_cases(spec) -> list:
    out = []
    for name, kind, T, integer, so, opts in spec:
        m = _model(name, kind, T, integer, so)
        t0 = time.perf_counter()
        sol = builtin.solve(m.model, **opts)
        out.append({"instance": name, "kind": kind, "T": T, "integer": integer, "set_optimal": so,
                    "opts": opts, "ncols": m.model.num_columns, "nrows": m.model.num_rows,
                    "expected": _sol_json(sol)})
        _TIMES.append((f"lp {name} {kind} T={T}", time.perf_counter() - t0))
    return out


LP_FAST = [
    ("fig4a_tree", "makespan", 3, True, True, {}),
    ("fig4a_tree", "makespan", 4, True, True, {}),
    ("fig4a_tree", "makespan", 3, False, False, {}),
    ("fig4a_tree", "makespan", 4, False, False, {}),
    ("fig4a_tree", "total_time", 4, True, True, {}),
    ("fig4a_tree", "total_time", 5, True, True, {}),
    ("fig4a_tree", "total_distance", 5, True, True, {}),
    ("fig4a_tree", "max_distance", 5, True, True, {}),
    ("fig8_split", "makespan", 4, True, True, {}),
    ("fig8_split", "makespan", 4, False, False, {}),
    ("fig8_split", "total_time", 4, True, True, {}),
    ("fig8_split", "total_time", 5, True, True, {}),
    ("fig8_split", "total_distance", 5, True, True, {}),
    ("fig8_split", "max_distance", 5, True, True, {}),
    ("fig8_split", "time_r", 8, True, True, {}),
    ("fig8_split", "total_distance", 8, True, True, {"node_limit": 1}),
    ("star_swap", "makespan", 3, True, True, {}),
    ("star_swap", "makespan", 4, True, True, {}),
    ("star_swap", "makespan", 5, False, False, {}),
    ("star_swap", "total_time", 5, True, True, {}),
    ("star_swap", "total_distance", 5, True, True, {}),
    ("star_swap", "max_distance", 5, True, True, {}),
    ("grid5x5_s3", "makespan", 4, True, True, {}),
    ("grid5x5_s3", "makespan", 4, False, False, {}),
    ("grid5x5_s3", "total_distance", 4, True, True, {}),
    ("grid5x5_s3", "max_distance", 4, True, True, {}),
    ("grid4x6_s5", "makespan", 5, False, False, {}),
    ("grid4x6_s5", "makespan", 5, True, True, {"work_limit": 300}),
    ("oneway_cycle", "makespan", 3, True, True, {}),
    ("fig1_9puzzle", "makespan", 3, False, False, {}),
    ("fig1_9puzzle", "makespan", 4, False, False, {}),
    ("fig1_9puzzle", "makespan", 4, True, True, {"node_limit": 3}),
    ("fig1_9puzzle", "total_time", 4, True, True, {"work_limit": 2000}),
    ("grid6x6_s7", "makespan", 5, False, False, {}),
    ("grid6x6_s7", "makespan", 5, True, True, {}),
]

LP_SLOW = [
    ("fig1_9puzzle", "makespan", 4, True, True, {}),
    ("puzzle16_s2016", "makespan", 5, False, False, {"work_limit": 5000}),
    ("fig1_9puzzle", "total_distance", 4, True, True, {"node_limit": 2}),
]


# ------------------------------------------------------------------------ planners
def _solve_json(r, rng=None) -> dict:
    d = {"paths": r.paths, "status": r.status, "objective": r.objective, "T": r.T, "lower_bound": r.lower_bound,
         "attempts": [{k: a[k] for k in ("T", "phase", "status", "objective", "bound")} for a in r.attempts],
         "extra": r.extra}
    if rng is not None:
        d["draws"] = rng.draws
    return d


def _plan_json(r, rng) -> dict:
    return {"paths": r.paths, "status": r.status, "intermediate_goals": r.intermediate_goals,
            "metrics": {"makespan": r.makespan, "totalTime": r.total_time, "totalDistance": r.total_distance,
                        "maxDistance": r.max_distance},
            "lower_bounds": r.lower_bounds, "horizons": r.stats["horizons"], "phases": r.stats["phases"],
            "solves": r.stats["solves"],
            "failures": r.extra.get("failures", []), "draws": rng.draws}


def _split_levels(g, s, t, splits, seed):
    """The intermediate configurations of the k-way split (``_mt``'s split loop, no solves)."""
    rng = MathRandom(seed)
    pl = PathPlanner(backend="builtin", rng=rng)
    mp = 2 ** splits
    msg = [[0] * len(s) for _ in range(mp + 1)]
    msg[0], msg[mp] = list(s), list(t)
    with pl._scope():
        for i in range(splits):
            num, step = 2 ** i, 2 ** (splits - i)
            for k in range(num):
                msg[k * step + step // 2] = list(pl.split_paths(g, msg[k * step], msg[(k + 1) * step], False))
    return msg, rng.draws


PLAN_FAST = [
    ("fig8_split", (1, 2, 4)), ("fig4a_tree", (1, 2)), ("star_swap", (1, 2)), ("grid5x5_s3", (1, 2)),
    ("oneway_cycle", (1,)),
]
SOLVE_FAST = ["fig8_split", "fig4a_tree", "star_swap", "grid5x5_s3", "oneway_cycle"]
PLAN_SLOW = [("fig1_9puzzle", (1, 2))]


def _graph_solves() -> list:
    """Objective solves on small graphs: goals without a wait self-loop (the horizon search
    solves every horizon; ``kwargs["joint_search"] = False`` disables the joint-state stage,
    ``solvers.JOINT_SEARCH_MAX_CONFIGS = 0``, to reach the scan) and the order of the instance
    checks (an unreachable goal is reported before ``t_max``). ``kwargs["node_limit"]`` is
    the builtin ``params={"node_limit": ...}`` (JS ``lp.nodeLimit``): a horizon stopped by it is
    undecided, never infeasible. The tree ``gt`` (a 2-way split piece) fails at ``T = 2``
    and is feasible only at ``T = 6``: the makespan loop returns the joint-state plan."""
    from . import solvers

    g2 = create_2d_grid_graph(2, 5)
    for v in (0, 1, 3, 4, 5, 6, 7, 9):  # wait loops only at 2 and 8
        g2.adj[v].remove(v)
    g6 = Graph()  # robot 0's goal 0 has no wait loop (nor has 5)
    for v, nb in enumerate([[3], [1, 2, 4, 5], [2, 3], [0, 2, 3, 4], [1, 2, 3, 4], [1, 4]]):
        g6.adj[v] = nb
        g6.xy[v] = None
    g6.vertices = list(range(6))
    gu = Graph.from_adjacency([[4], [2, 5], [1, 4], [], [0, 2], [1]])  # vertex 3 is unreachable
    fns = {"min_total_time": min_total_time, "min_total_distance": min_total_distance,
           "min_max_distance": min_max_distance, "min_makespan": min_makespan}
    p5 = create_grid_problem(5, 5, 0.1, 10, MathRandom(1))
    gt = Graph.from_adjacency([[1, 7], [0, 2, 6], [1, 3, 5], [2, 4], [3], [2], [1], [0]])
    cases = []
    for fn in fns:
        kw = {"node_limit": 0} if fn == "min_makespan" else {"node_limit": 0, "horizon": "single", "T": 8}
        cases.append((p5.graph, p5.starts, p5.goals, fn, kw))
    cases.append((gt, [3, 6, 0, 1], [2, 6, 0, 7], "min_makespan", {}))
    for fn in ("min_total_time", "min_total_distance", "min_max_distance"):
        for kw in ({}, {"joint_search": False}, {"T": 6}, {"t_max": 4}):
            cases.append((g2, [6, 8], [8, 5], fn, kw))
        cases.append((gu, [1, 4, 3, 0], [4, 3, 1, 2], fn, {"horizon": "single", "T": 5, "t_max": 0}))
    # (its scan up to T_hi = 12 takes minutes with the builtin solver: joint-state stage only)
    cases.append((g6, [1, 0, 2, 5], [0, 5, 2, 4], "min_total_time", {}))
    out = []
    for g, s, t, fn, kw in cases:
        rng = MathRandom(4)
        args = {k: v for k, v in kw.items() if k not in ("joint_search", "node_limit")}
        if "node_limit" in kw:
            args["params"] = {"node_limit": kw["node_limit"]}
        if fn in ("min_total_time", "min_makespan"):
            args["rng"] = rng
        old = solvers.JOINT_SEARCH_MAX_CONFIGS
        if kw.get("joint_search") is False:
            solvers.JOINT_SEARCH_MAX_CONFIGS = 0
        try:
            exp = _solve_json(fns[fn](g, s, t, backend="builtin", **args), rng)
        except Exception as e:  # Infeasible
            exp = {"error": type(e).__name__, "message": str(e), "draws": rng.draws}
        finally:
            solvers.JOINT_SEARCH_MAX_CONFIGS = old
        out.append({"graph": _graph_json(g), "starts": s, "goals": t, "fn": fn, "kwargs": kw, "seed": 4,
                    "expected": exp})
    return out


def _planners(slow: bool) -> dict:
    inst = _inst()
    out = {}
    if not slow:
        sg = []
        for name in ("fig8_split", "fig4a_tree", "star_swap", "fig1_9puzzle", "puzzle16_s2016", "grid6x6_s7",
                     "crowded8x8_s11", "grid24x18_s1", "grid4x6_s5"):
            g, s, t = inst[name]
            for seed in (1, 5):
                for rnd in (False, True):
                    rng = MathRandom(seed)
                    mid = split_paths(g, s, t, rnd, rng=rng)
                    sg.append({"instance": name, "seed": seed, "random": rnd, "expected": mid, "draws": rng.draws})
            for splits in (1, 2, 3, 4):
                msg, draws = _split_levels(g, s, t, splits, 3)
                sg.append({"instance": name, "seed": 3, "splits": splits, "levels": msg, "draws": draws})
        # a robot already at its goal, and adjacent robots (the fixed-mode endpoint rule)
        g4 = create_2d_grid_graph(3, 3)
        for s, t in (([4], [4]), ([4, 0], [4, 1]), ([0, 8, 2], [8, 0, 6]), ([0, 1, 2, 3, 4, 5, 6, 7], [7, 6, 5, 4, 3, 2, 1, 0])):
            rng = MathRandom(2)
            sg.append({"graph": _graph_json(g4), "starts": s, "goals": t, "seed": 2, "random": False,
                       "expected": split_paths(g4, s, t, False, rng=rng), "draws": rng.draws})
        # an odd distance on a non-bipartite graph (the middle vertex), one-way graphs (dist(v, goal))
        from .graph import create_2d_grid_graph_8connected
        for g5, s, t in ((create_2d_grid_graph_8connected(3, 4), [0], [3]),
                         (create_2d_grid_graph_8connected(3, 4), [0, 4, 11], [3, 7, 8]),
                         (Graph.from_adjacency([[2, 3], [0, 2], [1, 3], []]), [0], [2]),
                         (Graph.from_adjacency([[1, 3], [0, 2, 3], [], [2]]), [0, 1, 3], [1, 3, 2])):
            for rnd in (False, True):
                rng = MathRandom(2)
                sg.append({"graph": _graph_json(g5), "starts": s, "goals": t, "seed": 2, "random": rnd,
                           "expected": split_paths(g5, s, t, rnd, rng=rng), "draws": rng.draws})
        out["split_goals"] = sg
        solves = []
        for name in SOLVE_FAST:
            g, s, t = inst[name]
            for fn, kw in (("min_makespan", {}), ("min_makespan", {"solve_relaxed_first": True}),
                           ("min_total_time", {}), ("min_total_distance", {}), ("min_max_distance", {}),
                           ("min_total_time", {"horizon": "single", "T": None}),
                           ("min_total_distance", {"horizon": "single", "T": None}),
                           ("min_total_time", {"t_max": 0}), ("min_total_distance", {"t_max": -1}),
                           ("min_max_distance", {"t_max": 1}), ("min_total_distance", {"gap": 0.5}),
                           ("min_total_time", {"horizon": "single", "T": None, "t_max": 0}),
                           ("min_total_distance", {"horizon": "single", "T": None, "t_max": -1}),
                           ("min_max_distance", {"horizon": "single", "T": None, "t_max": 0})):
                kw = dict(kw)
                if kw.get("T", 0) is None:
                    kw["T"] = _lower_bounds(g, s, t)["makespan"] + 1
                if "t_max" in kw:  # relative to the makespan lower bound
                    kw["t_max"] += _lower_bounds(g, s, t)["makespan"]
                rng = MathRandom(4)
                f = {"min_makespan": min_makespan, "min_total_time": min_total_time,
                     "min_total_distance": min_total_distance, "min_max_distance": min_max_distance}[fn]
                if fn in ("min_makespan", "min_total_time"):
                    kw["rng"] = rng
                try:
                    r = f(g, s, t, backend="builtin", **kw)
                    exp = _solve_json(r, rng)
                except Exception as e:  # Infeasible
                    exp = {"error": type(e).__name__, "message": str(e), "draws": rng.draws}
                kw.pop("rng", None)
                solves.append({"instance": name, "fn": fn, "kwargs": kw, "seed": 4, "expected": exp})
        # total time with a gap > 0 on a single horizon: the backend gets the gap on the model's
        # objective (solvers._tt_model_gap; the gap itself there gives the bound 3.5, not 3)
        g, s, t = inst["star_swap"]
        rng = MathRandom(4)
        kw = {"horizon": "single", "T": 5, "gap": 0.2}
        solves.append({"instance": "star_swap", "fn": "min_total_time", "kwargs": kw, "seed": 4,
                       "expected": _solve_json(min_total_time(g, s, t, backend="builtin", rng=rng, **kw), rng)})
        # infeasible: two robots swap on a path; duplicate goals
        for adj, s, t in (([[1], [0, 2], [1]], [0, 2], [2, 0]), ([[1], [0, 2], [1]], [0, 1], [2, 2]),
                          ([[1], [0]], [0], [1])):
            g = Graph.from_adjacency(adj)
            rng = MathRandom(4)
            try:
                r = min_makespan(g, s, t, backend="builtin", rng=rng)
                exp = _solve_json(r, rng)
            except Exception as e:
                exp = {"error": type(e).__name__, "message": str(e), "draws": rng.draws}
            solves.append({"graph": _graph_json(g), "starts": s, "goals": t, "fn": "min_makespan", "kwargs": {},
                           "seed": 4, "expected": exp})
        solves.extend(_graph_solves())
        out["solvers"] = solves
        # joint_feasibility from a first horizon (graphs without wait loops)
        jf = []
        for adj, s, t, tf in (([[1], [0, 2], [1]], [0, 2], [0, 2], 0), ([[1], [0, 2], [1]], [0, 2], [0, 2], 1),
                              ([[1], [2], []], [0], [2], 2), ([[1], [2], []], [0], [2], 3),
                              ([[1], [2], [3], [4], [5], [0]], [5, 0, 2], [5, 0, 2], 1),
                              ([[1], [2], [3], [4], [5], [0]], [0, 2], [1, 3], 3),
                              ([[0, 1], [0, 1, 2], [1, 2]], [0, 2], [2, 0], 0)):
            g = Graph()
            for v, nb in enumerate(adj):
                g.adj[v] = list(nb)
                g.xy[v] = None
            g.vertices = list(range(len(adj)))
            jf.append({"graph": _graph_json(g), "starts": s, "goals": t, "t_first": tf,
                       "expected": joint_feasibility(g, s, t, t_first=tf)})
        # robots on two components, proved only by the search of the first one alone (the whole
        # instance exceeds the budget): a 3x3 grid minus two cells (840 configurations,
        # infeasible) and a 6x6 grid with 20 robots
        adj = [[1, 2], [0, 3], [0, 3, 5], [1, 2, 4], [3, 6], [2], [4]]
        s, t = [1, 0, 2, 5, 3, 4], [6, 3, 0, 2, 4, 1]
        grid = create_2d_grid_graph(6, 6)
        adj += [[7 + w for w in grid.adj[v] if w != v] for v in range(36)]
        s, t = s + [7 + i for i in range(20)], t + [7 + 35 - i for i in range(20)]
        g = Graph.from_adjacency(adj)
        jf.append({"graph": _graph_json(g), "starts": s, "goals": t, "t_first": 0, "budget": 10**6,
                   "expected": joint_feasibility(g, s, t, budget=10**6, t_first=0)})
        out["joint_feasibility"] = jf
        # joint_optimum: the max-distance search with a bound above the optimum + 1 (its descent and
        # the Pareto pruning of the move vectors), and the two A* searches
        nbrs = [[0, 1, 2], [0, 1, 3, 4, 7], [0, 2, 5], [1, 3, 6], [1, 4, 7], [2, 5], [3, 6], [1, 4, 7]]
        g = Graph.from_adjacency(nbrs)
        s, t = [1, 0, 3, 7, 5], [7, 4, 0, 1, 3]
        jo = []
        for objective, upper in (("max_distance", 7), ("max_distance", 8), ("max_distance", 10),
                                 ("total_time", None), ("distance", None)):
            jo.append({"graph": _graph_json(g), "starts": s, "goals": t, "objective": objective, "upper": upper,
                       "budget": JOINT_SEARCH_BUDGET,
                       "expected": joint_optimum(g, s, t, objective, JOINT_SEARCH_BUDGET, upper)})
        out["joint_optimum"] = jo
    plans = []
    for name, ks in (PLAN_SLOW if slow else PLAN_FAST):
        g, s, t = inst[name]
        for k in ks:
            rng = MathRandom(6)
            t0 = time.perf_counter()
            r = plan((g, s, t), "makespan", k, backend="builtin", rng=rng)
            e = _plan_json(r, rng)
            plans.append({"instance": name, "k": k, "seed": 6, "expected": e})
            _TIMES.append((f"plan {name} k={k}", time.perf_counter() - t0))
    if not slow:
        from .graph import create_2d_grid_graph_8connected
        for g5, s, t, ks in ((create_2d_grid_graph_8connected(3, 4), [0], [3], (2,)),
                             (Graph.from_adjacency([[1, 3], [0, 2, 3], [], [2]]), [0, 1, 3], [1, 3, 2], (2, 4)),
                             (Graph.from_adjacency([[1], [0, 2], [1]]), [0, 2], [2, 0], (2, 2048, 2 ** 40)),
                             (Graph.from_adjacency([[1], [0, 2], [1]]), [0, 1], [2, 2], (2,)),
                             (Graph.from_adjacency([[1, 7], [0, 2, 6], [1, 3, 5], [2, 4], [3], [2], [1], [0]]),
                              [4, 6, 1, 3], [2, 6, 0, 7], (2,))):
            for k in ks:
                rng = MathRandom(6)
                try:
                    e = _plan_json(plan((g5, s, t), "makespan", k, backend="builtin", rng=rng), rng)
                except Exception as ex:  # Infeasible: the instance checks; ValueError: k > MAX_SPLIT
                    e = {"error": type(ex).__name__, "draws": rng.draws}
                plans.append({"graph": _graph_json(g5), "starts": s, "goals": t, "k": k, "seed": 6, "expected": e})
    out["plan_split"] = plans
    if not slow:
        checks = []
        g = create_2d_grid_graph(3, 3)
        S, G = [0, 2], [2, 0]
        sets = [
            [[0, 1, 2], [2, 5, 8]], [[0, 1, 2], [2, 1, 0]], [[0, 1, 1], [2, 1, 0]], [[0, 4], [2, 1]],
            [[0, 3, 4, 1, 2], [2, 1, 0, 3, 4]], [[0, 1], [2]], [], [[1, 2], [2, 0]],
            [[0, 1, 2, 2], [2, 5, 4, 0]], [[0, 0, 0, 1, 2], [2, 2, 1, 0, 0]],
        ]
        def metrics_or_error(P, goals):
            try:
                return plan_metrics(P, goals)
            except ValueError:
                return {"error": "ValueError"}

        for P in sets:
            checks.append({"paths": P, "starts": S if len(P) != 0 else [], "goals": G if len(P) != 0 else [],
                           "error": path_set_error(P, g, S if P else [], G if P else []),
                           "cycles": has_cycles(P) if P and len({len(p) for p in P}) == 1 else None,
                           "metrics": metrics_or_error(P, G if P else [])})
        # rotations on the 3x3 ring
        ring = [0, 1, 2, 5, 8, 7, 6, 3]
        P = [[ring[i], ring[(i + 1) % 8], ring[(i + 2) % 8]] for i in range(7)]
        checks.append({"paths": P, "starts": [p[0] for p in P], "goals": [p[-1] for p in P],
                       "error": path_set_error(P, g, [p[0] for p in P], [p[-1] for p in P]),
                       "cycles": has_cycles(P), "metrics": plan_metrics(P, [p[-1] for p in P])})
        P = [[ring[i], ring[(i + 1) % 8], ring[(i + 2) % 8]] for i in range(8)] + [[4, 4, 4]]
        checks.append({"paths": P, "starts": [p[0] for p in P], "goals": [p[-1] for p in P],
                       "error": path_set_error(P, g, [p[0] for p in P], [p[-1] for p in P]),
                       "cycles": has_cycles(P), "metrics": plan_metrics(P, [p[-1] for p in P])})
        out["checks"] = [{**c, "graph": _graph_json(g)} for c in checks]
    return out


# ------------------------------------------------------------------------ driver
FAST = {
    "java": _java,
    "graphs": _graphs,
    "pathfinder": _pathfinder,
    "network": _network,
    "models": _models,
    "lp": lambda: {"lp": _lp_cases(LP_FAST)},
    "planners": lambda: _planners(False),
}
SLOW = {
    "lp_slow": lambda: {"lp": _lp_cases(LP_SLOW)},
    "planners_slow": lambda: _planners(True),
}
FILES = {**FAST, **SLOW}


def generate(name: str) -> dict:
    """The fixture ``name`` (a key of :data:`FILES`) as a JSON-able dict."""
    cases = FILES[name]()
    used = sorted({c["instance"] for grp in cases.values() for c in grp if isinstance(c, dict) and "instance" in c})
    return {"meta": {"version": VERSION, "generator": "python -m mppflow.parity", "file": name + ".json",
                     "tier": "slow" if name in SLOW else "fast"},
            "instances": {k: _instance_json(k) for k in used},
            "cases": cases}


def default_out() -> str | None:
    """:data:`DEFAULT_OUT` when this module runs from a checkout of the repository (with
    ``tests/`` and ``docs/js/`` next to the package), else ``None``: an installed package
    would otherwise write into ``site-packages/tests/parity``."""
    root = os.path.dirname(HERE)
    if os.path.isdir(os.path.join(root, "tests")) and os.path.isdir(os.path.join(root, "docs", "js")):
        return DEFAULT_OUT
    return None


def write(out_dir: str | None = None, names=None, verbose: bool = True) -> None:
    """Write the fixtures ``names`` (default: all) to ``out_dir`` (default:
    :func:`default_out`; ``ValueError`` outside a checkout)."""
    if out_dir is None:
        out_dir = default_out()
        if out_dir is None:
            raise ValueError("not run from a checkout of the repository: give an output directory")
    os.makedirs(out_dir, exist_ok=True)
    for name in names or FILES:
        t0 = time.perf_counter()
        text = dumps(generate(name))
        path = os.path.join(out_dir, name + ".json")
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        if verbose:
            print(f"{path}: {len(text) / 1024:.0f} KiB in {time.perf_counter() - t0:.1f} s")
            for label, sec in _TIMES:
                if sec >= 1.0:
                    print(f"    {label}: {sec:.1f} s")
            _TIMES.clear()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m mppflow.parity", description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", default=None,
                    help="output directory (default: tests/parity of the checkout; required for an installed package)")
    ap.add_argument("--fast", action="store_true", help="only the fast fixtures")
    ap.add_argument("names", nargs="*", help="fixture names (default: all)")
    a = ap.parse_args(argv)
    names = a.names or (list(FAST) if a.fast else list(FILES))
    for n in names:
        if n not in FILES:
            ap.error(f"unknown fixture {n!r}; choose from {', '.join(FILES)}")
    out = a.out or default_out()
    if out is None:
        ap.error("not run from a checkout of the repository (no tests/ and docs/js/ next to the package): "
                 "pass --out DIR")
    write(out, names)
    return 0


if __name__ == "__main__":
    sys.exit(main())
