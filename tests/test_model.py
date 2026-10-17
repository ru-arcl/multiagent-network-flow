"""Tests of mppflow.model: golden model dumps (canonical text / SHA-256, both modes), key
helpers, the original's int32 key quirks, the new max-distance and full-network models."""
from __future__ import annotations

import os

import pytest

from mppflow import lp
from mppflow import model as M
from mppflow.errors import NotInOriginal
from mppflow.graph import Graph, create_2d_grid_graph, create_main_case3_graph
from mppflow.network import update_reachable_sets
from helpers import graph_from_json, load_golden

HERE = os.path.dirname(__file__)
GOLDEN = os.path.join(HERE, "golden")


MODEL_RECS = [r for r in load_golden("models") if r["fn"].endswith(".prepareModel")]
# fixed mode differs from the original only here (TimeR's id-overflow bug, ids >= 22)
FIXED_DIFFERS = {"models/TimeR/6x6_swap_T10"}


def build_from_record(r, compat):
    a = r["args"]
    g = graph_from_json(a["graph"])
    cls = r["fn"].split(".")[0]
    s, t, T = a["starts"], a["goals"], a["timeSteps"]
    if cls == "MultiagentGraphSolverGurobiTime":
        return M.build_time_model(g, s, t, T, a["integer"], a["setOptimal"], time_limit=a["timeLimit"], compat=compat)
    if cls == "MultiagentGraphSolverGurobiTimeR":
        return M.build_time_r_model(g, s, t, T, a["integer"], a["setOptimal"], time_limit=a["timeLimit"],
                                    compat=compat)
    if cls == "MultiagentGraphSolverGurobiTimePuzzle":
        return M.build_time_puzzle_model(g, s, t, T, a["integer"], a["setOptimal"], compat=compat)
    if cls == "MultiagentGraphSolverGurobiTotalTime":
        return M.build_total_time_model(g, s, t, T, a["integer"], time_limit=a["timeLimit"], gap=a.get("gap"),
                                        compat=compat)
    if cls == "MultiagentGraphSolverGurobiDistance":
        return M.build_distance_model(g, s, t, T, a["integer"], time_limit=a["timeLimit"], gap=a.get("gap"),
                                      compat=compat)
    raise AssertionError(cls)


# ------------------------------------------------------------------ golden dumps
@pytest.mark.parametrize("rec", MODEL_RECS, ids=lambda r: r["id"])
@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_golden_model_dump(rec, compat):
    m = build_from_record(rec, compat)
    s = m.summary()
    gold = rec["result"]
    if compat == "fixed" and rec["id"] in FIXED_DIFFERS:
        assert s["sha256"] != gold["sha256"]
        return
    for k in ("ncols", "nrows", "nnz", "sense", "edgeVarMap_size"):
        assert s[k] == gold[k], k
    if "dump" in gold:
        assert s["dump"] == gold["dump"]
    assert s["sha256"] == gold["sha256"]
    for k in ("TimeLimit", "MIPGap", "OutputFlag"):
        assert s["params"][k] == gold["params"][k], k
    assert not any(m.key_collisions.values())


def test_canonical_text_example():
    """README example (models/Time/star_T3)."""
    g = create_main_case3_graph()
    text = M.build_time_model(g, [0, 3], [3, 0], 3).canonical_text()
    lines = text.splitlines()
    assert lines[:4] == ["SENSE max", "NCOLS 16", "NROWS 18", "C0 0 1 B 0 x[0,0,0,0]"]
    assert "C14 1 1 B 1 x[0,3,0,3]" in lines and "C15 1 1 B 1 x[1,0,3,3]" in lines
    assert lines[3 + 16] == "R0 <= 1 : (1,1) (7,1)"
    assert "R4 = 0 : (0,1) (1,1) (14,-1)" in lines
    assert text.endswith("\n")


def test_canonical_text_entry_points_agree():
    """LinearModel.to_canonical_text(names) is the golden format; model.canonical_text and
    MPPModel.canonical_text/sha256 are wrappers around it."""
    rec = next(r for r in MODEL_RECS if r["id"] == "models/Time/star_T3")
    m = build_from_record(rec, "original")
    names = m.column_names()
    text = m.model.to_canonical_text(names)
    assert text == m.canonical_text() == M.canonical_text(m.model, names)
    assert m.model.sha256(names) == m.sha256() == rec["result"]["sha256"]
    assert M.canonical_text(m.model) == m.model.to_canonical_text()


def test_canonical_number_format():
    lm = lp.LinearModel("min")
    lm.add_column(-0.0, lp.INF, "C", 0.5)
    lm.add_column(0, 1e100, "I", 2)
    lm.add_row([(0, 1.0), (1, 2.0), (0, -1.0)], ">=", -1.25)
    lm.add_row([(0, 1.0), (0, -1.0)], "==", 0)
    text = M.canonical_text(lm, ["a", "b"])
    assert text == ("SENSE min\nNCOLS 2\nNROWS 2\nC0 0 inf C 0.5 a\nC1 0 inf I 2 b\n"
                    "R0 >= -1.25 : (1,2)\nR1 = 0 :\n")


# ----------------------------------------------------------------- key helpers
@pytest.mark.parametrize("rec", [r for r in load_golden("models") if r["fn"].endswith("keyHelpers")],
                         ids=lambda r: r["id"])
def test_key_helpers_golden(rec):
    time_r = "TimeR" in rec["fn"]
    got = [M.key_helpers(*tup, time_r=time_r) for tup in rec["args"]["tuples"]]
    assert got == rec["result"]


def test_key_helper_overflows():
    assert M.edge_id(0, 22, 21, 3) == -2094757293
    assert M.time_from_edge_id(M.edge_id(0, 22, 21, 3)) == -7293
    assert M.time_from_edge_id(M.edge_id(1, 22, 21, 3)) == 2707
    assert M.edge_time_id(3, 4, 0) == -1294927296 == M.edge_time_id(4, 3, 0)
    assert M.edge_time_id(2, 3, 0) == 2000030000
    assert M.vertex_time_id(0, 10000) == M.vertex_time_id(1, 0)


# ------------------------------------------------------------ int32 key quirks
def test_head_on_rows_in_wrapped_key_order():
    """R3 rows follow the TreeMap order of the wrapped int keys, not (u, v, t) order."""
    g = create_2d_grid_graph(4, 4, True)
    m = M.build_time_model(g, [0, 15, 3], [15, 0, 12], 8)
    lm = m.model
    last_eq = max(i for i in range(lm.num_rows) if lm.row_sense(i) == "==")
    heads = []
    for i in range(last_eq + 1, lm.num_rows):
        ks = {(min(m.arcs[c][1:3]), max(m.arcs[c][1:3]), m.arcs[c][3]) for c, _ in lm.row(i)}
        assert len(ks) == 1 and all(m.arcs[c][1] != m.arcs[c][2] for c, _ in lm.row(i))
        heads.append(ks.pop())
    assert len(heads) > 5
    keys = [M.edge_time_id(*k) for k in heads]
    assert keys == sorted(keys)
    assert heads != sorted(heads)


def test_vertex_time_key_collision_t10000():
    """T >= 10000: v*10000+t collides between (0, 10000) and (1, 0). The original merges
    agent 0's loopback (into (1, 0)) into the capacity row of (0, 10000) and its flow
    rows; fixed keeps them apart. (The HiGHS solve of these 40000 columns takes about 7 s on
    scipy 1.15.3 but did not finish in minutes on 1.15.0-1.15.2, hence the floor 1.15.3.)"""
    g = Graph.from_adjacency([[1], [0]])
    mo = M.build_time_model(g, [1], [0], 10000, compat="original")
    mf = M.build_time_model(g, [1], [0], 10000, compat="fixed")
    assert mo.key_collisions["vertexTime"] == 1 and mo.key_collisions["agentVertexTime"] == 1
    assert mf.key_collisions == mo.key_collisions
    assert mo.model.num_columns == mf.model.num_columns

    def rows_with(m, col):
        lm = m.model
        return [(lm.row_sense(i), len(lm.row(i))) for i in range(lm.num_rows) if any(c == col for c, _ in lm.row(i))]

    assert rows_with(mo, mo.loopbacks[0]) == [("<=", 3), ("==", 6), ("==", 6)]
    assert rows_with(mf, mf.loopbacks[0]) == [("==", 3), ("==", 3)]
    # the fixed model is the T >= 10000 analogue of a short-horizon model: feasible
    sol = lp.solve(mf.model, backend="highs")
    assert sol.status == "optimal" and round(sol.objective) == 1


def test_time_r_rows_fixed_include_high_ids():
    g = create_2d_grid_graph(6, 6, True)
    mo = M.build_time_r_model(g, [30, 35], [35, 30], 10, compat="original")
    mf = M.build_time_r_model(g, [30, 35], [35, 30], 10, compat="fixed")
    T = 10
    moves = [j for j, (a, u, v, t) in enumerate(mf.arcs) if u != v]
    rows_f = [set(c for c, _ in mf.model.row(i)) for i in range(T + 1)]
    rows_o = [set(c for c, _ in mo.model.row(i)) for i in range(T + 1)]
    assert set().union(*rows_f) == set(moves)
    for t in range(T + 1):
        assert rows_f[t] == {j for j in moves if mf.arcs[j][3] == t}
        # the original keeps only arcs with tail < 22
        assert rows_o[t] == {j for j in rows_f[t] if mf.arcs[j][1] < 22}
    assert not rows_o[0] and rows_f[0]


# ------------------------------------------------------------ model structure
def test_total_time_columns_and_chain():
    g = Graph.from_adjacency([[1], [0, 2], [1]])
    m = M.build_total_time_model(g, [0], [2], 4)
    names = m.column_names()
    assert names[len(m.arcs):] == ["x[0,2,0,4]", "yT[0]", "y[0,3]", "y[0,2]", "y[0,1]", "y[0,0]"]
    assert m.model.objective_terms == [(m.y_total[0], 1.0)]
    for t in (2, 3):
        assert m.edge_var(0, 2, 2, t) is not None
    assert m.edge_var(0, 2, 2, 1) is None
    sol = lp.solve(m.model, backend="highs", mip_gap=0)
    assert round(sol.objective) == 2  # waits at the goal from t=2 to 4: total time 4*1 - 2 = 2


def test_distance_objective_and_loopbacks():
    g = create_main_case3_graph()
    m = M.build_distance_model(g, [0, 3], [3, 0], 4)
    lm = m.model
    assert lm.sense == "min"
    assert [j for j, _ in lm.objective_terms] == [j for j, (a, u, v, t) in enumerate(m.arcs) if u != v]
    assert all(lm.column(j)[0] == 1.0 for j in m.loopbacks)
    lp_m = M.build_distance_model(g, [0, 3], [3, 0], 4, integer=False)
    assert all(lp_m.model.column(j)[0] == 0.0 and lp_m.model.column(j)[2] == "C" for j in lp_m.loopbacks)


def test_max_distance_model():
    g = create_main_case3_graph()
    with pytest.raises(NotInOriginal):
        M.build_max_distance_model(g, [0, 3], [3, 0], 4, compat="original")
    m = M.build_max_distance_model(g, [0, 3], [3, 0], 6)
    d = M.build_distance_model(g, [0, 3], [3, 0], 6)
    assert m.model.num_columns == d.model.num_columns + 1 and m.x_max == m.model.num_columns - 1
    assert m.model.num_rows == d.model.num_rows + 2
    assert m.column_names()[-1] == "x_max"
    sol = lp.solve(m.model, backend="highs")
    assert round(sol.objective) == 4  # one robot hides in leaf 2: 4 moves
    paths = m.retrieve_paths(sol.x, g)
    assert max(sum(p[i] != p[i + 1] for i in range(len(p) - 1)) for p in paths) == 4


def test_relaxed_time_model():
    g = create_main_case3_graph()
    m = M.build_time_model(g, [0, 3], [3, 0], 3, integer=False, set_optimal=False)
    assert set(m.model.vtypes) == {"C"} and all(m.model.column(j)[0] == 0.0 for j in m.loopbacks)
    sol = lp.solve(m.model, backend="highs")
    assert sol.objective > 2 - 0.01  # the relaxation is feasible at T=3 (the ILP is not)
    ilp = lp.solve(M.build_time_model(g, [0, 3], [3, 0], 3).model, backend="highs")
    assert ilp.status == "infeasible"


def test_reach_argument_checked():
    g = create_main_case3_graph()
    r = update_reachable_sets(g, [0, 3], [3, 0], 3)
    assert M.build_time_puzzle_model(g, [0, 3], [3, 0], 3, reach=r).sha256() == \
        M.build_time_model(g, [0, 3], [3, 0], 3).sha256()
    with pytest.raises(ValueError):
        M.build_time_model(g, [0, 3], [3, 0], 4, reach=r)
    with pytest.raises(ValueError):
        M.build_time_model(g, [0, 3], [3, 0], 3, compat="java")


def test_retrieve_paths_modes():
    g = create_main_case3_graph()
    m = M.build_time_model(g, [0, 3], [3, 0], 4)
    sol = lp.solve(m.model, backend="highs")
    x = [1.0 if v > 0.5 else 0.0 for v in sol.x]
    p = m.retrieve_paths(x, g)
    noisy = [v - 1e-9 if v == 1.0 else v for v in x]
    assert m.retrieve_paths(noisy, g) == p  # fixed: X > 0.5
    mo = M.build_time_model(g, [0, 3], [3, 0], 4, compat="original")
    assert mo.retrieve_paths(x, g) == p
    with pytest.warns(M.InexactSolutionWarning, match="backend='gurobi'"):
        bad = mo.retrieve_paths(noisy, g)  # original: X == 1.0 fails, falls back to vertex 0
    assert bad == [[0, 0, 0, 0, 0], [3, 0, 0, 0, 0]]
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("error")  # exact binaries: no warning
        assert mo.retrieve_paths(x, g) == p
    with pytest.raises(RuntimeError):
        m.retrieve_paths([0.0] * len(x), g)


# ----------------------------------------------------------- full network model
FIG4 = dict(edges=[(0, 2), (1, 2), (2, 4), (3, 4), (4, 5)], starts=[3, 5], goals=[0, 1])


def fig4_graph():
    adj = [[] for _ in range(6)]
    for u, v in FIG4["edges"]:
        adj[u].append(v)
        adj[v].append(u)
    return Graph.from_adjacency(adj)


def test_full_network_maxflow_by_T():
    """Fixture full_network_maxflow_by_T = {0:0, 1:0, 2:0, 3:1, 4:2, 5:2}."""
    g = fig4_graph()
    got = {}
    for T in range(6):
        fm = M.build_full_network_model(g, FIG4["starts"], FIG4["goals"], T)
        got[T] = round(lp.solve(fm.model, backend="highs").objective)
    assert got == {0: 0, 1: 0, 2: 0, 3: 1, 4: 2, 5: 2}
    fm = M.build_full_network_model(g, FIG4["starts"], FIG4["goals"], 2)
    assert fm.model.num_columns == 152  # n * |E'| variables of eq. (8)


@pytest.mark.parametrize("T", [4, 5])
def test_full_network_objectives(T):
    """eq. (16) with the paper's costs is the constant n*T; corrected: total distance 6,
    max distance 3; total time 7 (= the compact TT model)."""
    g = fig4_graph()
    s, t = FIG4["starts"], FIG4["goals"]

    def val(obj, **kw):
        fm = M.build_full_network_model(g, s, t, T, objective=obj, **kw)
        sol = lp.solve(fm.model, backend="highs", mip_gap=0)
        paths = fm.retrieve_paths(sol.x)
        assert [p[0] for p in paths] == s and [p[-1] for p in paths] == t
        return sol.objective, paths

    assert val("distance", paper_costs=True)[0] == pytest.approx(2 * T)
    d, paths = val("distance")
    assert d == pytest.approx(6)
    assert sum(p[i] != p[i + 1] for p in paths for i in range(T)) == 6
    assert val("max_distance")[0] == pytest.approx(3)
    tt, _ = val("total_time")
    assert 2 * T - tt == pytest.approx(7)
    m = M.build_total_time_model(g, s, t, T)
    assert 2 * T - lp.solve(m.model, backend="highs", mip_gap=0).objective == pytest.approx(7)
    names = M.build_full_network_model(g, s, t, 1).column_names()
    assert names[0].startswith("x[0,loop:")
    with pytest.raises(ValueError):
        M.build_full_network_model(g, s, t, 1, objective="nope")


# ------------------------------------------- regressions: colliding-key names
LINE3 = [[1], [0, 2], [1]]  # 0 - 1 - 2 with self-loops (the recorded graph)
# sha256 of the live Java dumps (tools/golden/Harness.java dumpModel, Gurobi 13.0.2);
# every model has colliding getEdgeId keys (t >= 10000 carries into the v2 field)
BIG_COLLISION_SHA = {
    ("time", 10000, (1,), (0,)): "94ceab7e6da261a641e14631bac4129b80cde98beae288af9b6ffa13dd7da753",
    ("time", 10001, (0, 2), (2, 1)): "77cccc5cbc73f4521d94448d7eb05ea80276bb57fdf69e338d11c1e49d2092ac",
    ("time", 10003, (2, 0), (0, 1)): "7ebd1eae95e4fba87cfb8fdfd7be306a5b0c858b64f583c8778d0602ae178409",
    ("total_time", 10000, (1,), (0,)): "a5e9170aaa052a5498efb4d4752667ad820c87e2f2e5649297c5c0b40af252c4",
    ("total_time", 10001, (0, 2), (2, 1)): "c29b58c2def9be07ef7959feca02b3a9a86aa4691d61d2e58965dfe5483a772e",
    ("total_time", 10003, (2, 0), (0, 1)): "c0cd67c3ccbce8a3167bf9bd5b069b1a1fc716037e5a4dbad9253cfbb39de743",
    ("distance", 10000, (1,), (0,)): "5f9a116e3b5e27ee79d4fd422a6b5022055966a0fe978ff73d8b7b0bde26a455",
    ("distance", 10001, (0, 2), (2, 1)): "6a17df756bf612201b66c3cbe60fa505221ff15677cc9c3096b0bc2c2ac1daa2",
    ("distance", 10003, (2, 0), (0, 1)): "798c5f93b9833b35bc3fb92126932956477fe19a4744e700ebfdcf7a93c861d3",
    ("time_r", 10000, (1,), (0,)): "18c2948a7e0a4a77789118e9624b33ec2d645385726607f6be398cdd9d50737b",
    ("time_r", 10001, (0, 2), (2, 1)): "4bd85324fec245a7d56670536a7018701e135e7e813b5b07f4092342ed168ed7",
    ("time_r", 10003, (2, 0), (0, 1)): "ac925e753b4061a55afb1dc97893f4057131d1a1dcdb21e89e7d67d2b5e8427c",
}
_BUILDERS = {"time": M.build_time_model, "time_r": M.build_time_r_model,
             "total_time": M.build_total_time_model, "distance": M.build_distance_model}


def test_collision_names_follow_dump_writer():
    """Time, T = 10001: the dump writer joins the names of tuples sharing a key and writes
    ?<j> for a column whose edgeVarMap entry was overwritten (live Java dump lines)."""
    g = Graph.from_adjacency(LINE3)
    m = M.build_time_model(g, [0, 2], [2, 1], 10001, compat="original")
    names = m.column_names()
    assert names[1] == "x[0,0,1,0]|x[0,0,0,10000]"
    assert names[69994] == "?69994"
    text = m.canonical_text().splitlines()
    assert text[4] == "C1 0 1 B 0 x[0,0,1,0]|x[0,0,0,10000]"
    assert m.sha256() == BIG_COLLISION_SHA[("time", 10001, (0, 2), (2, 1))]
    # fixed mode keeps the groups apart and names every column by its own tuple
    mf = M.build_time_model(g, [0, 2], [2, 1], 10001)
    assert "|" not in "".join(mf.column_names()) and "?" not in "".join(mf.column_names())


def test_no_collision_names_unchanged():
    """Without collisions the dump-writer recovery gives the semantic names (ids >= 22
    exercise the recovery path, T small)."""
    g = create_2d_grid_graph(6, 6, True)
    m = M.build_total_time_model(g, [0, 35], [35, 0], 12, compat="original")
    assert not m._edge_keys_injective()
    semantic = M.MPPModel.column_names(m.__class__(**{**m.__dict__, "graph": None}))
    assert m._golden_names() == semantic == m.column_names()


@pytest.mark.slow
@pytest.mark.parametrize("key", sorted(BIG_COLLISION_SHA), ids=lambda k: f"{k[0]}-{k[1]}")
def test_collision_models_match_java_dumps(key):
    kind, T, s, t = key
    g = Graph.from_adjacency(LINE3)
    m = _BUILDERS[kind](g, list(s), list(t), T, compat="original")
    assert m.sha256() == BIG_COLLISION_SHA[key]
