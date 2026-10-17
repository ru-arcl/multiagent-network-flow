"""Tests of mppflow.network: reachability (golden), compact arcs (column order), the paper's
full (Fig. 5) and compact (Fig. 6/7) networks (counts from the verified paper fixture),
and the JSON export."""
from __future__ import annotations

import json
import os
from functools import lru_cache

import pytest

from mppflow import network as N
from mppflow.errors import JavaArrayIndexOutOfBoundsException, JavaNegativeArraySizeException
from mppflow.graph import Graph, create_2d_grid_graph, create_main_case3_graph
from mppflow.model import build_time_model
from helpers import from_edges, golden_records, graph_from_json

HERE = os.path.dirname(__file__)
GOLDEN = os.path.join(HERE, "golden")


# The Fig. 4(a) instance and its counts, checked by hand against T-RO 2016 Figs. 4 and 7.
FIG4_EDGES = [(0, 2), (1, 2), (2, 4), (3, 4), (4, 5)]
FIG4_STARTS, FIG4_GOALS = [3, 5], [0, 1]
FIG5_BUILT = {"vertices_total": 50, "arcs_total": 76,
              "by_kind": {"loop": 2, "green": 12, "blue": 12, "g_in": 20, "g_mid": 10, "g_out": 20}}
FIG5_FORMULA = {"vertex_copies": 5, "copy_vertices": 30, "gadget_vertices": 20, "vertices_total": 50,
                "green_arcs": 12, "blue_arcs": 12, "gadget_arcs": 50, "gadgets": 10,
                "arcs_without_loopbacks": 74, "loopback_arcs": 2, "arcs_total": 76, "x_variables_eq8": 152}
FIG6_COMPACT = {"vertices": 18, "stay_arcs": 12, "move_arcs": 20, "arcs_total": 32, "loopback_arcs": 2,
                "arcs_total_with_loopbacks": 34, "crossing_pairs_per_step": 5}


# ----------------------------------------------------------------- reachability
@pytest.mark.parametrize("rec", golden_records("models", "updateReachableSets"), ids=lambda r: r["id"])
@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_reachable_sets_golden(rec, compat):
    a = rec["args"]
    g = graph_from_json(a["graph"])
    reach = N.update_reachable_sets(g, a["starts"], a["goals"], a["timeSteps"], compat)
    assert reach.vertex_lists() == rec["result"]
    m = reach.matrix()
    for ag, per_t in enumerate(rec["result"]):
        for t, vs in enumerate(per_t):
            assert [v for v in range(g.n) if m[ag][v][t]] == vs
            assert all(reach(ag, v, t) == (v in vs) for v in range(g.n))


def test_reachability_unreached_distance_zero():
    # two components {0,1} and {2,3}: vertices the BFS does not reach keep distance 0
    g = Graph.from_adjacency([[1], [0], [3], [2]])
    r = N.update_reachable_sets(g, [0], [1], 2)
    assert r.dist_start[0] == [0, 1, 0, 0]
    assert r.vertex_lists() == [[[0, 2, 3], [0, 1, 2, 3], [1, 2, 3]]]


def test_reachability_bad_inputs():
    g = create_main_case3_graph()
    with pytest.raises(ValueError):
        N.update_reachable_sets(g, [7], [0], 3)
    with pytest.raises(JavaArrayIndexOutOfBoundsException):
        N.update_reachable_sets(g, [7], [0], 3, compat="original")
    with pytest.raises(ValueError):
        N.update_reachable_sets(g, [0], [3], -1)
    assert N.update_reachable_sets(g, [0], [3], -1, compat="original").T == -1
    with pytest.raises(JavaNegativeArraySizeException):
        N.update_reachable_sets(g, [0], [3], -2, compat="original")


# ----------------------------------------------------------------- compact arcs
def test_compact_arcs_star_order():
    """Main case 3's star, starts {0,3}, goals {3,0}, T=3: the compact arcs in generation order."""
    g = create_main_case3_graph()
    r = N.update_reachable_sets(g, [0, 3], [3, 0], 3)
    arcs = list(N.compact_arcs(g, [0, 3], r))
    assert arcs == [(0, 0, 0, 0), (0, 0, 1, 0), (0, 0, 1, 1), (0, 1, 1, 1), (0, 1, 3, 1), (0, 1, 3, 2),
                    (0, 3, 3, 2), (1, 3, 1, 0), (1, 3, 3, 0), (1, 1, 0, 1), (1, 1, 1, 1), (1, 3, 1, 1),
                    (1, 0, 0, 2), (1, 1, 0, 2)]


def test_compact_arcs_hashset_order_differs_from_sorted():
    """Frontier ids >= 16 follow Java HashSet bucket order, not ascending order."""
    g = create_2d_grid_graph(6, 6, True)
    r = N.update_reachable_sets(g, [0], [35], 12)
    arcs = list(N.compact_arcs(g, [0], r))
    tails_by_t = {}
    for (_, u, _, t) in arcs:
        lst = tails_by_t.setdefault(t, [])
        if u not in lst:
            lst.append(u)
    assert any(lst != sorted(lst) for lst in tails_by_t.values())
    # the order is the one the (golden-verified) model builder uses
    m = build_time_model(g, [0], [35], 12)
    assert m.arcs == arcs


def test_compact_arcs_missing_adjacency():
    g = Graph.from_adjacency([[1], [0]])
    del g.adj[1]
    r = N.Reachability(1, [[0, 1]], [[1, 0]])
    with pytest.raises(ValueError):
        list(N.compact_arcs(g, [1], r))
    from mppflow.errors import JavaNullPointerException
    with pytest.raises(JavaNullPointerException):
        list(N.compact_arcs(g, [1], r, compat="original"))


# --------------------------------------------------------------- full network
def fig4_graph():
    return from_edges(6, FIG4_EDGES)


def test_fig5_full_network_counts():
    net = N.FullNetwork.build(fig4_graph(), FIG4_STARTS, FIG4_GOALS, 2)
    assert net.counts() == FIG5_BUILT
    f = net.formula_counts()
    for k, v in FIG5_FORMULA.items():
        assert f[k] == v, k
    # loopbacks first, in robot order: g_i(T)' -> s_i(0)
    for i, (s, g) in enumerate(zip(FIG4_STARTS, FIG4_GOALS)):
        a, b, kind = net.arcs[i]
        assert kind == "loop"
        assert net.nodes[a] == ("v", g, 2, 1) and net.nodes[b] == ("v", s, 0, 0)
    # costs: the paper's literal ones vs the corrected distance costs
    assert sum(net.paper_cost) == 12 + 10 and sum(net.arc_cost) == 10


@pytest.mark.parametrize("rows,cols,T,n", [(2, 3, 1, 1), (3, 3, 3, 2), (4, 2, 5, 3)])
def test_full_network_formula_matches_built(rows, cols, T, n):
    g = create_2d_grid_graph(rows, cols, True)
    net = N.FullNetwork.build(g, list(range(n)), list(range(n)), T)
    f = net.formula_counts()
    c = net.counts()
    assert c["vertices_total"] == f["vertices_total"]
    assert c["arcs_total"] == f["arcs_total"]
    assert c["by_kind"]["green"] == f["green_arcs"] and c["by_kind"]["blue"] == f["blue_arcs"]
    assert c["by_kind"]["g_mid"] == f["gadgets"]
    # every node except time 0 / T' has in- and out-arcs; flow is conserved structurally
    ins, outs = net.in_arcs(), net.out_arcs()
    for i, key in enumerate(net.nodes):
        if key[0] in ("m", "s"):
            assert len(ins[i]) == (2 if key[0] == "m" else 1)
            assert len(outs[i]) == (1 if key[0] == "m" else 2)


def test_theorem1_flows_vs_paths():
    """The full network's single-robot flows outnumber the paths (fixture: 12 flows vs 4
    paths for r1 of Fig. 4(a) at T=4; 2 vs 1 on a single edge at T=1)."""

    def count(g, s, t_, T):
        net = N.FullNetwork.build(g, [s], [t_], T)
        out = {}
        for (a, b, kind) in net.arcs:
            if kind != "loop":
                out.setdefault(a, []).append(b)
        target = net.copy_node(t_, T, prime=T > 0)

        @lru_cache(maxsize=None)
        def flows(i):
            if i == target:
                return 1
            return sum(flows(j) for j in out.get(i, ()))

        @lru_cache(maxsize=None)
        def walks(v, t):
            if t == T:
                return 1 if v == t_ else 0
            return sum(walks(w, t + 1) for w in g.adj[v])

        return flows(net.copy_node(s, 0)), walks(s, 0)

    g = fig4_graph()
    assert count(g, 3, 0, 3) == (1, 1)
    assert count(g, 3, 0, 4) == (12, 4)
    assert count(from_edges(2, [(0, 1)]), 0, 0, 1) == (2, 1)


# ------------------------------------------------------------- compact network
def test_fig6_compact_network_counts():
    net = N.CompactNetwork.build(fig4_graph(), FIG4_STARTS, FIG4_GOALS, 2)
    assert net.counts() == FIG6_COMPACT


def test_compact_network_pruned_equals_model_columns():
    g = create_2d_grid_graph(3, 3, True)
    s, t = [0, 2, 6], [8, 6, 1]
    net = N.CompactNetwork.build(g, s, t, 5, prune=True)
    m = build_time_model(g, s, t, 5)
    assert [arc for per in net.agent_arcs for arc in per] == m.arcs
    full = net.counts()
    assert full["arcs_total"] == 9 * 5 + 2 * 12 * 5
    assert all(k < full["arcs_total"] for k in full["pruned_arcs_per_agent"])


def test_fig4_pruning_example():
    """Fig. 4(a) Time model sizes after pruning (setOptimal): T=2: 2 columns
    (loopbacks only), T=3: 8, T=4: 22, T=5: 50."""
    g = fig4_graph()
    sizes = {T: build_time_model(g, FIG4_STARTS, FIG4_GOALS, T).model.num_columns for T in (2, 3, 4, 5)}
    rows = {T: build_time_model(g, FIG4_STARTS, FIG4_GOALS, T).model.num_rows for T in (2, 3, 4, 5)}
    assert sizes == {2: 2, 3: 8, 4: 22, 5: 50}
    assert rows == {2: 4, 3: 11, 4: 24, 5: 53}


# ----------------------------------------------------------------- JSON export
def test_json_export():
    g = fig4_graph()
    full = N.FullNetwork.build(g, FIG4_STARTS, FIG4_GOALS, 2)
    d = json.loads(full.to_json_str())
    assert d["type"] == "full" and len(d["nodes"]) == 50 and len(d["arcs"]) == 76
    assert d["counts"] == FIG5_BUILT
    ids = {n["id"] for n in d["nodes"]}
    assert all(a["tail"] in ids and a["head"] in ids for a in d["arcs"])
    assert sorted({n["kind"] for n in d["nodes"]}) == ["copy", "copy'", "merge", "split"]
    assert [a["robot"] for a in d["arcs"] if a["kind"] == "loop"] == [0, 1]
    comp = N.CompactNetwork.build(g, FIG4_STARTS, FIG4_GOALS, 4, prune=True)
    d = json.loads(comp.to_json_str())
    assert d["type"] == "compact" and len(d["nodes"]) == 30
    assert [a["robot"] for a in d["arcs"] if a["kind"] == "loop"] == [0, 1]
    assert sum(len(x) for x in d["agent_arcs"]) == 20  # 22 columns minus 2 loopbacks
    for arcs_a in d["agent_arcs"]:
        for (u, w) in arcs_a:
            assert d["nodes"][w]["t"] == d["nodes"][u]["t"] + 1


def test_undirected_edges():
    g = create_main_case3_graph()
    assert N.undirected_edges(g) == [(0, 1), (1, 2), (1, 3)]


def test_reachability_one_way_goal_side_uses_reversed_arcs():
    """Regression: on a one-way graph the goal-side distance must be dist(v, g).

    The original runs the BFS forward from the goal (dist(g, v)) and prunes the only route
    of the one-way 4-cycle 0 -> 1 -> 2 -> 3 -> 0 at T = 3; fixed keeps it."""
    g = Graph.from_adjacency([[1], [2], [3], [0]])
    fx = N.update_reachable_sets(g, [0], [3], 3)
    og = N.update_reachable_sets(g, [0], [3], 3, compat="original")
    assert fx.dist_start == og.dist_start == [[0, 1, 2, 3]]
    assert fx.dist_goal == [[3, 2, 1, 0]]
    assert og.dist_goal == [[1, 2, 3, 0]]
    assert fx.vertex_lists() == [[[0], [1], [2], [3]]]
    assert og.vertex_lists() == [[[0], [0, 1], [0], [3]]]
    arcs = list(N.compact_arcs(g, [0], fx))
    assert arcs == [(0, 0, 1, 0), (0, 1, 2, 1), (0, 2, 3, 2)]
    # original: the robot can only wander around 0 and never reaches 3 at T
    assert list(N.compact_arcs(g, [0], og, compat="original")) == [(0, 0, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)]
    # a vertex that cannot reach the goal keeps distance 0 (as the forward BFS leaves it)
    g2 = Graph.from_adjacency([[1, 2], [], [0]])  # 1 is a sink
    assert N.update_reachable_sets(g2, [0], [2], 2).dist_goal == [[1, 0, 0]]


def test_reachability_symmetric_graph_modes_equal():
    """On symmetric graphs (all the original can build) both modes run the same BFS."""
    g = create_2d_grid_graph(3, 4)
    starts, goals = [0, 5, 11], [11, 2, 4]
    fx = N.update_reachable_sets(g, starts, goals, 6)
    og = N.update_reachable_sets(g, starts, goals, 6, compat="original")
    assert (fx.dist_start, fx.dist_goal) == (og.dist_start, og.dist_goal)
    assert N._reversed_adjacency(g) is None
    assert N._reversed_adjacency(Graph.from_adjacency([[1], [0, 2], [1]])) is None
    assert N._reversed_adjacency(Graph.from_adjacency([[1], [2], [0]])) is not None


def test_full_network_needs_undirected_graph_with_wait_loops():
    """The two-way gadget would let a robot travel against a one-way arc, and its same-side
    exit would let it wait at a vertex without a self-loop: the full network (T-RO Sec. III,
    undirected graphs with waiting) rejects both graphs instead of returning invalid plans."""
    from mppflow.model import build_full_network_model
    one_way = Graph.from_adjacency([[1], []])
    with pytest.raises(ValueError, match="one-way"):
        N.FullNetwork.build(one_way, [1], [0], 1)
    with pytest.raises(ValueError, match="one-way"):
        build_full_network_model(one_way, [1], [0], 1)
    g = Graph.from_adjacency([[1], [0, 2], [1]])
    for v in list(g.adj):
        g.adj[v] = [w for w in g.adj[v] if w != v]
    with pytest.raises(ValueError, match="wait loop at every vertex; vertex 0"):
        N.FullNetwork.build(g, [0], [1], 2)
    assert N.CompactNetwork.build(one_way, [0], [1], 1).counts()["move_arcs"] == 1
