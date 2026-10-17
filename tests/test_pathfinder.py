"""mppflow.pathfinder against the live original (fixtures recorded by the Java harness).

Fixtures: tests/data/core/pathfinder.json.gz, recorded by ``tools/golden/core/record_core.sh``
(``CoreGolden.java``) against the original classes with Math.random seeded by reflection;
treebins.json.gz by a one-off probe (see tools/golden/README.md, "Core fixtures").
Randomized searches are checked on the returned paths and on the next random value after
the call (i.e. the exact number of draws), with ``PathFinder.bHeuristic`` off and on.
"""
from __future__ import annotations

import os

import pytest

from mppflow import graph as G
from mppflow import pathfinder as PF
from mppflow._java import MathRandom
from mppflow.errors import Infeasible, JavaException, JavaNullPointerException
from mppflow.problem import create_grid_problem, create_grid_problem_8connected
from helpers import case_id, java_class, load_core

DATA = os.path.join(os.path.dirname(__file__), "data", "core")


CASES = load_core("pathfinder")


def by_kind(*kinds):
    out = [c for c in CASES if c["kind"] in kinds]
    assert out, kinds
    return out


def hash_grid(rows, cols):
    """A converted grid whose adjacency sets are Java HashSets (self first, then ascending),
    like the planners' sub-graphs."""
    g = G.create_2d_grid_graph(rows, cols, True)
    for v in range(g.n):
        g.set_hashset_adjacency(v, [v] + [u for u in g.adj[v] if u != v])
    return g


def disconnected():
    return G.Graph.from_adjacency([[1], [0], [3], [2]])


def graph(name):
    return {
        "star": G.create_main_case3_graph,
        "tree": G.create_main_case6_tree,
        "grid3": lambda: G.create_2d_grid_graph(3, 3, True),
        "grid5": lambda: G.create_2d_grid_graph(5, 5, True),
        "grid7": lambda: G.create_2d_grid_graph(7, 7, True),
        "grid8": lambda: G.create_2d_grid_graph(8, 8, True),
        "grid6x7": lambda: G.create_2d_grid_graph(6, 7, True),
        "hash6x6": lambda: hash_grid(6, 6),
        "disconnected": disconnected,
        "unconverted2x2": lambda: G.create_2d_grid_graph(2, 2, False),
    }[name]()


# ---------------------------------------------------------------------- distances
@pytest.mark.parametrize("case", by_kind("dist"), ids=case_id)
def test_distances(case):
    a = case["args"]
    g = graph(a["graph"])
    if case["exception"]:
        with pytest.raises(JavaException) as ei:
            PF.get_distances_to_all_vertices(g, a["start"], None, "original")
        assert java_class(ei.value) == case["exception"]
        with pytest.raises(ValueError):
            PF.get_distances_to_all_vertices(g, a["start"])
        return
    for compat in ("original", "fixed"):
        assert PF.get_distances_to_all_vertices(g, a["start"], None, compat) == case["result"]


def test_distances_prefilled():
    c = by_kind("dist_prefilled")[0]
    arr = [99] * 4
    res = PF.get_distances_to_all_vertices(disconnected(), 0, arr)
    assert res is arr and res == c["result"]["result"] and c["result"]["same"]
    assert PF.get_distances_to_all_vertices(disconnected(), 0, unreachable=-1) == [0, 1, -1, -1]


def test_distances_problem():
    c = by_kind("dist_problem")[0]
    a = c["args"]
    rng = MathRandom(c["seed"])
    p = create_grid_problem(a["col"], a["row"], a["p"], a["n"], rng)
    got = []
    for i in range(a["n"]):
        got.append(PF.get_distances_to_all_vertices(p.graph, p.starts[i]))
        got.append(PF.get_distances_to_all_vertices(p.graph, p.goals[i]))
    assert got == c["result"]
    assert rng.random() == c["next"]  # no draws


def test_distances_missing_adjacency():
    g = G.Graph.from_adjacency([[1], [0, 2], [1]])
    del g.adj[2]
    with pytest.raises(JavaNullPointerException):
        PF.get_distances_to_all_vertices(g, 0, compat="original")
    assert PF.get_distances_to_all_vertices(g, 0) == [0, 1, 2]


# ------------------------------------------------------------------ distance range
@pytest.mark.parametrize("case", by_kind("range"), ids=case_id)
def test_goals_in_distance_range(case):
    a = case["args"]
    g = graph(a["graph"])
    for compat in ("original", "fixed"):
        assert PF.find_goals_in_distance_range(g, a["start"], a["dmin"], a["dmax"], compat) == case["result"]


@pytest.fixture(scope="module")
def problem32():
    return create_grid_problem(32, 32, 0.2, 1, MathRandom(3))


@pytest.mark.parametrize("case", by_kind("range_problem32"), ids=case_id)
def test_goals_in_distance_range_large(case, problem32):
    a = case["args"]
    assert case["seed"] == 3
    assert PF.find_goals_in_distance_range(problem32.graph, a["start"], a["dmin"], a["dmax"]) == case["result"]


# --------------------------------------------------------------- randomized BFS
@pytest.mark.parametrize("case", by_kind("one_path"), ids=case_id)
@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_one_shortest_path(case, compat):
    a = case["args"]
    g = graph(a["graph"])
    rng = MathRandom(case["seed"])
    p = PF.find_one_shortest_path(g, a["start"], a["goal"], None, rng, heuristic=a["heuristic"], compat=compat)
    assert p == case["result"]
    assert rng.random() == case["next"]


@pytest.mark.parametrize("case", by_kind("one_path")[:24], ids=case_id)
def test_global_flag_emulation(case):
    """compat="original" with heuristic=None reads the emulated global bHeuristic."""
    a = case["args"]
    g = graph(a["graph"])
    rng = MathRandom(case["seed"])
    PF.B_HEURISTIC.set(a["heuristic"])
    try:
        p = PF.find_one_shortest_path(g, a["start"], a["goal"], None, rng, compat="original")
        # fixed mode ignores the global flag
        q = PF.find_one_shortest_path(g, a["start"], a["goal"], None, MathRandom(case["seed"]))
    finally:
        PF.B_HEURISTIC.set(False)
    assert p == case["result"]
    assert rng.random() == case["next"]
    plain = next(c for c in by_kind("one_path")
                 if c["args"] == dict(a, heuristic=False) and c["seed"] == case["seed"])
    assert q == plain["result"]


@pytest.mark.parametrize("case", by_kind("paths_problem"), ids=case_id)
def test_shortest_paths_problem(case):
    a = case["args"]
    rng = MathRandom(case["seed"])
    p = create_grid_problem(a["col"], a["row"], a["p"], a["n"], rng)
    assert p.sg == case["result"]["sg"]
    paths = PF.find_shortest_paths(p.graph, p.starts, p.goals, rng, heuristic=a["heuristic"])
    assert paths == case["result"]["paths"]
    assert rng.random() == case["next"]


@pytest.mark.parametrize("case", by_kind("lb_problem"), ids=case_id)
@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_lower_bounds_problem(case, compat):
    a = case["args"]
    rng = MathRandom(case["seed"])
    p = create_grid_problem(a["col"], a["row"], a["p"], a["n"], rng, compat)
    m = PF.get_makespan_lower_bound(p.graph, p.starts, p.goals, rng, heuristic=a["heuristic"], compat=compat)
    t = PF.get_total_time_lower_bound(p.graph, p.starts, p.goals, rng, heuristic=a["heuristic"], compat=compat)
    assert {"makespan_lb": m, "total_time_lb": t} == case["result"]
    assert rng.random() == case["next"]


@pytest.mark.parametrize("case", by_kind("lb"), ids=case_id)
def test_lower_bounds_small(case):
    a = case["args"]
    g = graph(a["graph"])
    rng = MathRandom(case["seed"])
    m = PF.get_makespan_lower_bound(g, a["starts"], a["goals"], rng)
    t = PF.get_total_time_lower_bound(g, a["starts"], a["goals"], rng)
    assert [m, t] == case["result"]
    assert rng.random() == case["next"]


@pytest.mark.parametrize("case", by_kind("lb_makespan", "lb_total_time"), ids=case_id)
def test_lower_bounds_unreachable(case):
    a = case["args"]
    f = PF.get_makespan_lower_bound if case["kind"] == "lb_makespan" else PF.get_total_time_lower_bound
    rng = MathRandom(case["seed"])
    with pytest.raises(JavaNullPointerException):
        f(disconnected(), a["starts"], a["goals"], rng, compat="original")
    assert rng.random() == case["next"]
    rng = MathRandom(case["seed"])
    with pytest.raises(Infeasible):
        f(disconnected(), a["starts"], a["goals"], rng)
    assert rng.random() == case["next"]


def test_total_distance():
    for c in by_kind("total_distance"):
        assert PF.get_total_distance(c["args"]["paths"]) == c["result"]
    c = by_kind("total_path_length")[0]
    rng = MathRandom(c["seed"])
    paths = PF.find_shortest_paths(graph("star"), [0, 3], [3, 0], rng)
    assert PF.get_total_path_length(paths) == c["result"]
    assert rng.random() == c["next"]


# ---------------------------------------------------------------------- misc
def test_queue_helpers():
    q: dict = {}
    PF.add_to_queue(q, 5)
    PF.add_to_queue(q, 6)
    assert PF.de_queue(q) == 5 and PF.de_queue(q) == 6 and q == {}
    rng = MathRandom(1)
    PF.add_to_queue(q, 1, rng, heuristic=True)
    assert rng.draws == 1 and q == {0: [1]}


def test_already_visited_not_mutated_in_fixed():
    g = G.create_2d_grid_graph(3, 3, True)
    av = {7}
    p = PF.find_one_shortest_path(g, 0, 8, av, MathRandom(1))
    assert av == {7} and p[0] == 0 and p[-1] == 8 and 7 not in p and len(p) == 5
    av2 = {7}
    q = PF.find_one_shortest_path(g, 0, 8, av2, MathRandom(1), compat="original")
    assert q == p and len(av2) > 1


def test_start_equals_goal_no_draw():
    rng = MathRandom(3)
    assert PF.find_one_shortest_path(G.create_2d_grid_graph(3, 3, True), 4, 4, None, rng) == [4]
    assert rng.draws == 0


def test_unreachable_returns_none():
    rng = MathRandom(3)
    assert PF.find_one_shortest_path(disconnected(), 0, 3, None, rng) is None
    assert PF.find_shortest_paths(disconnected(), [0, 2], [1, 0], MathRandom(1)) [1] is None


# ------------------------------------------- treeified HashMap bins
# tests/data/core/treebins.json.gz, recorded live (JDK 17) by a Java probe:
# * "range": findGoalsInDistanceRange on grids whose BFS layers put 9+ ids in one bucket
#   (probe cases "range4|8 col row p start dmin dmax seed": createGridProblem
#   [8Connected](col, row, p, 1), start % n), with the next Math.random() value, and the
#   SHA-256 of all results on 16 x {31,33,47,49,63,64} grids from every start (dmin=1,
#   dmax=100);
# * "hashset": java.util.HashSet<Integer>/<Long> add/remove sequences with colliding keys
#   (HSProbe.java): each call's return value, a hash of the iteration order after each
#   call, and the final iteration order.
TREEBINS = load_core("treebins")


def _range_case_id(c):
    return c["case"]


@pytest.mark.parametrize("case", TREEBINS["range"]["cases"], ids=_range_case_id)
@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_goals_in_distance_range_treeified(case, compat):
    k, col, row, p, st, a, b, seed = case["case"].split()
    rng = MathRandom(int(seed))
    f = create_grid_problem if k == "range4" else create_grid_problem_8connected
    g = f(int(col), int(row), float(p), 1, rng, compat).graph
    assert PF.find_goals_in_distance_range(g, int(st) % g.n, int(a), int(b), compat) == case["result"]
    assert rng.random() == case["next"]


@pytest.mark.parametrize("cols", [31, 33] + [pytest.param(c, marks=pytest.mark.slow) for c in (47, 49, 63, 64)])
def test_goals_in_distance_range_all_starts(cols):
    import hashlib
    g = G.create_2d_grid_graph(16, cols)
    h = hashlib.sha256()
    for st in range(g.n):
        res = PF.find_goals_in_distance_range(g, st, 1, 100)
        h.update((",".join(map(str, res)) + "\n").encode())
    assert h.hexdigest() == TREEBINS["range"]["sweep_rows16_d1_100_sha256"][str(cols)]


def _java_order_hash(items, is_long):
    from mppflow._java import i32
    h = 17
    for o in items:
        u = o & 0xFFFFFFFFFFFFFFFF
        hc = i32(u ^ (u >> 32)) if is_long else i32(o)
        h = i32(h * 31 + hc + i32(u >> 7))
    return h


@pytest.mark.parametrize("idx", range(len(TREEBINS["hashset"])))
def test_java_hashset_treeified_bins(idx):
    from mppflow._java import JavaIntHashSet
    from mppflow.graph import _JavaLongHashSet
    c = TREEBINS["hashset"][idx]
    is_long = c["type"] == "L"
    s = _JavaLongHashSet() if is_long else JavaIntHashSet()
    for i, (op, k) in enumerate(zip(c["ops"], c["keys"])):
        r = s.add(k) if op == "a" else s.remove(k)
        assert ("1" if r else "0") == c["ret"][i], i
        assert _java_order_hash(s, is_long) == c["hash"][i], i
    assert s.to_list() == c["order"]
    assert len(s) == len(set(c["order"]))
    assert all(k in s for k in c["order"])


def test_goals_in_distance_range_example():
    # Reproduction: 16 x 33 grid, start 48, range [14, 31].
    g = create_grid_problem(16, 33, 0.0, 1, MathRandom(786451)).graph
    for compat in ("original", "fixed"):
        res = PF.find_goals_in_distance_range(g, 48, 14, 31, compat)
        assert len(res) == len(set(res)) > 0


def test_missing_adjacency_entries_are_skipped():
    # findGoalsInDistanceRange and findOneShortestPath test the adjacency set for null
    # and skip the vertex (PathFinder.java:74, :150), in both modes.
    g = G.Graph.from_adjacency([[1], [0, 2], [1, 3], [2]])
    del g.adj[2]
    for compat in ("original", "fixed"):
        assert PF.find_goals_in_distance_range(g, 0, 1, 5, compat) == [1, 2]
        rng = MathRandom(3)
        assert PF.find_one_shortest_path(g, 0, 3, None, rng, heuristic=False, compat=compat) is None
        assert PF.find_one_shortest_path(g, 0, 2, None, MathRandom(3), heuristic=False, compat=compat) == [0, 1, 2]
    assert repr(PF.GlobalFlag(True)) == "GlobalFlag(True)"
