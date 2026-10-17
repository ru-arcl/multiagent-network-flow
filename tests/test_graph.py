"""mppflow.graph against the live original (fixtures recorded by the Java harness).

Fixtures: tests/data/core/graph.json.gz, recorded by ``tools/golden/core/record_core.sh``
(``CoreGolden.java``) with JDK 17 against the original classes, Math.random seeded by
reflection. The ``graph`` records of tests/golden are checked in ``test_golden_core.py``. Each case holds
the result (or the Java exception class), the captured stdout and the next Math.random()
value after the call, so the random-stream position is checked too.
"""
from __future__ import annotations

import os

import pytest

from mppflow import graph as G
from mppflow._java import MathRandom
from mppflow.errors import (
    JavaException,
    JavaIndexOutOfBoundsException,
    JavaNullPointerException,
    OriginalWouldHang,
)
from helpers import case_id, java_class, load_core

DATA = os.path.join(os.path.dirname(__file__), "data", "core")


CASES = load_core("graph")


def by_kind(*kinds):
    out = [c for c in CASES if c["kind"] in kinds]
    assert out, kinds
    return out


def dump(g: G.Graph) -> dict:
    d = g.to_dict()
    return d


def assert_graph(py: G.Graph, java: dict, exact_order: bool | None = None):
    d = dump(py)
    assert d["adj"] == java["adj"]
    assert d["xy"] == java["xy"]
    assert d["rows"] == java["rows"] and d["cols"] == java["cols"]
    if exact_order is None:
        # Unconverted graphs: the original's vertices[] is in identity-hash order, which
        # is not reproducible (and never observable); we use ascending order.
        exact_order = java["vertices"] == sorted(java["vertices"])
    if exact_order:
        assert d["vertices"] == java["vertices"]
    else:
        assert sorted(d["vertices"]) == sorted(java["vertices"])
        assert d["vertices"] == sorted(d["vertices"])


def run(case, fn, compat="original"):
    """Run fn(rng) under the case's seed; return (result, exception class, rng)."""
    rng = MathRandom(case["seed"]) if case["seed"] >= 0 else MathRandom(12345)
    try:
        res = fn(rng)
        exc = None
    except JavaException as e:
        res, exc = None, java_class(e)
    if case["seed"] >= 0:
        assert rng.random() == case["next"], "random stream position differs"
    return res, exc


# ---------------------------------------------------------------- grid builders
@pytest.mark.parametrize("case", by_kind("grid4", "grid8"), ids=case_id)
@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_grid(case, compat):
    a = case["args"]
    f = G.create_2d_grid_graph if case["kind"] == "grid4" else G.create_2d_grid_graph_8connected
    g = f(a["rows"], a["cols"], a["update"], compat)
    buggy = (case["kind"] == "grid4" and a["cols"] == 1 and a["rows"] > 1) or \
            (case["kind"] == "grid8" and a["cols"] <= 2)
    if compat == "original" or not buggy:
        assert_graph(g, case["result"])
    else:
        # fixed: the true grid neighbourhood, symmetric, with self-loops
        d = dump(g)
        assert d != case["result"]
        for k, nb in g.adj.items():
            assert k in nb
            for u in nb:
                assert k in g.adj[u]


def test_grid_fixed_narrow_shapes():
    g = G.create_2d_grid_graph(3, 1, True)
    assert g.adj == {0: [0, 1], 1: [0, 1, 2], 2: [1, 2]}
    g = G.create_2d_grid_graph_8connected(2, 2, True)
    assert g.adj == {v: [0, 1, 2, 3] for v in range(4)}
    g = G.create_2d_grid_graph_8connected(3, 1, False)
    assert g.adj == {1: [1, 2], 2: [1, 2, 3], 3: [2, 3]}
    # wider grids are identical in both modes
    for r, c in [(3, 3), (4, 5), (1, 4)]:
        assert G.create_2d_grid_graph_8connected(r, c, True).adj == \
            G.create_2d_grid_graph_8connected(r, c, True, "original").adj


# ---------------------------------------------------------------- hole generators
@pytest.mark.parametrize("case", by_kind("holes4", "holes8"), ids=case_id)
def test_holes_original(case):
    a = case["args"]
    f = (G.create_generic_2d_grid_graph_with_holes if case["kind"] == "holes4"
         else G.create_generic_2d_grid_graph_with_holes_8connected)
    res, exc = run(case, lambda rng: f(a["rows"], a["cols"], a["p"], rng, "original"))
    assert exc == case["exception"]
    if exc is None:
        assert_graph(res, case["result"])


@pytest.mark.parametrize("case", by_kind("holes4", "holes8"), ids=case_id)
def test_holes_fixed_equal_where_correct(case):
    a = case["args"]
    narrow = a["cols"] == 1 and a["rows"] > 1 if case["kind"] == "holes4" else a["cols"] <= 2
    if narrow:
        pytest.skip("narrow grid: the original's adjacency is wrong here")
    f = (G.create_generic_2d_grid_graph_with_holes if case["kind"] == "holes4"
         else G.create_generic_2d_grid_graph_with_holes_8connected)
    res, exc = run(case, lambda rng: f(a["rows"], a["cols"], a["p"], rng, "fixed"))
    assert exc is None
    assert_graph(res, case["result"])


def test_holes_too_many():
    with pytest.raises(OriginalWouldHang):
        G.create_generic_2d_grid_graph_with_holes(3, 3, 1.5, MathRandom(1), "original")
    with pytest.raises(ValueError):
        G.create_generic_2d_grid_graph_with_holes(3, 3, 1.5, MathRandom(1))
    g = G.create_generic_2d_grid_graph_with_holes(3, 3, 1.0, MathRandom(1))
    assert g.xy == {} and g.vertices == []


def test_holes_fixed_narrow_connected():
    for seed in range(5):
        g = G.create_generic_2d_grid_graph_with_holes_8connected(7, 2, 0.4, MathRandom(seed))
        cg = G.convert_graph(g)
        from mppflow.pathfinder import get_distances_to_all_vertices
        d = get_distances_to_all_vertices(cg, 0, unreachable=-1)
        assert all(x >= 0 for x in d)


def test_holes_8c_cols1_npe():
    # The original's 8-connected 3x1 grid has empty adjacency sets; the first removal
    # deletes them all, and the next attempt dereferences a missing set.
    with pytest.raises(JavaNullPointerException):
        G.create_generic_2d_grid_graph_with_holes_8connected(3, 1, 0.67, MathRandom(1), "original")
    g = G.create_generic_2d_grid_graph_with_holes_8connected(3, 1, 0.67, MathRandom(1))
    assert len(g.xy) == 1


# ----------------------------------------------------------------- start / goal
@pytest.mark.parametrize("case", by_kind("random_sg_holes"), ids=case_id)
@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_random_sg_holes(case, compat):
    a = case["args"]

    def f(rng):
        g = G.create_generic_2d_grid_graph_with_holes(a["rows"], a["cols"], a["p"], rng, compat)
        sg = G.get_random_start_goal(g, a["n"], rng, compat)
        return g, sg

    (g, sg), exc = run(case, f)
    assert exc is None
    assert_graph(g, case["result"]["graph"])
    assert sg == case["result"]["sg"]


@pytest.mark.parametrize("case", by_kind("random_sg_converted"), ids=case_id)
def test_random_sg_converted(case):
    a = case["args"]
    res, exc = run(case, lambda rng: G.get_random_start_goal(
        G.create_2d_grid_graph(a["rows"], a["cols"], True), a["n"], rng, "original"))
    assert res == case["result"]


@pytest.mark.parametrize("case", by_kind("random_sg_many_grid", "random_sg_many_tree"), ids=case_id)
@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_random_sg_many(case, compat):
    a = case["args"]
    if case["kind"] == "random_sg_many_tree":
        mk = G.create_main_case6_tree
        number = a["number"]
    else:
        mk = lambda: G.create_2d_grid_graph(a["n"], a["n"], True)  # noqa: E731
        number = a["number"]
    if compat == "fixed" and case["exception"]:
        with pytest.raises(ValueError):
            G.get_random_start_goal_many(mk(), number, MathRandom(case["seed"]))
        return
    res, exc = run(case, lambda rng: G.get_random_start_goal_many(mk(), number, rng, compat))
    assert exc == case["exception"]
    assert res == case["result"]


def test_random_sg_hang():
    g = G.create_2d_grid_graph(2, 2, False)
    assert G.get_random_start_goal(g, 4, MathRandom(1)) == [[2, 4, 1, 3], [1, 4, 3, 2]]
    with pytest.raises(OriginalWouldHang):
        G.get_random_start_goal(g, 5, MathRandom(1), "original")
    with pytest.raises(ValueError):
        G.get_random_start_goal(g, 5, MathRandom(1))


# --------------------------------------------------------------------- convert
@pytest.mark.parametrize("case", by_kind("convert"), ids=case_id)
@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_convert(case, compat):
    a = case["args"]

    def f(rng):
        g = G.create_generic_2d_grid_graph_with_holes(a["rows"], a["cols"], a["p"], rng, compat)
        s, t = G.get_random_start_goal(g, a["n"], rng, compat)
        before = [list(s), list(t)]
        og = G.convert_graph(g, s, t, compat)
        return before, [s, t], og

    (before, after, og), exc = run(case, f)
    assert before == case["result"]["before"]
    assert after == case["result"]["after"]
    assert_graph(og, case["result"]["graph"], exact_order=True)


def _path_graph(*ids):
    g = G.Graph()
    for i, v in enumerate(ids):
        nb = []
        if i > 0:
            nb.append(ids[i - 1])
        if i + 1 < len(ids):
            nb.append(ids[i + 1])
        g.add_vertex(v, nb)
    g.finish_building_graph()
    return g


def test_convert_errors():
    c = by_kind("convert_missing_vertex")[0]
    g = _path_graph(1, 2, 3)
    g.adj[3].append(7)
    with pytest.raises(JavaNullPointerException):
        G.convert_graph(g, [], [], "original")
    assert c["exception"] == "java.lang.NullPointerException"
    with pytest.raises(ValueError):
        G.convert_graph(g, [], [])
    c = by_kind("convert_bad_start")[0]
    assert c["exception"] == "java.lang.NullPointerException"
    with pytest.raises(JavaNullPointerException):
        G.convert_graph(_path_graph(1, 2, 3), [5], [1], "original")
    with pytest.raises(ValueError):
        G.convert_graph(_path_graph(1, 2, 3), [5], [1])


# ------------------------------------------------------------ connectivity tests
def _star():
    return G.create_main_case3_graph()


@pytest.mark.parametrize("case", by_kind("conn_path", "conn_grid_noself", "conn_star", "conn_missing", "conn_isolated"), ids=case_id)
def test_is_connected_without_vertex(case):
    v = case["args"]["v"]
    k = case["kind"]
    if k == "conn_path":
        g = _path_graph(1, 2, 3)
    elif k == "conn_grid_noself":
        g = G.create_2d_grid_graph(3, 3, True)
        G.update_adj_list(g)
    elif k in ("conn_star", "conn_missing"):
        g = _star()
    else:
        g = G.Graph()
        g.add_vertex(2, [])
        g.finish_building_graph()
    before = g.copy()
    for compat in ("original", "fixed"):
        h = before.copy()
        try:
            r = h.is_connected_without_vertex(v, compat)
            exc = None
        except JavaException as e:
            r, exc = None, java_class(e)
        except ValueError:
            r, exc = None, "ValueError"
        if compat == "original":
            assert exc == case["exception"]
            if exc is None:
                assert r == case["result"]["result"]
                assert_graph(h, case["result"]["graph"])
        else:
            if case["exception"]:
                assert exc == "ValueError"
            else:
                assert r == case["result"]["result"]
                assert h.adj == before.adj  # fixed: no mutation


@pytest.mark.parametrize("case", by_kind("conn_without_v3_grid3", "conn_without_v3_star"), ids=case_id)
def test_are_vertices_connected_without_v3(case):
    a = case["args"]
    g = G.create_2d_grid_graph(3, 3, True) if case["kind"].endswith("grid3") else _star()
    assert g.are_vertices_connected_without_v3(a["a"], a["b"], a["c"]) == case["result"]


def test_quirk_documented_examples():
    g = _path_graph(1, 2, 3)
    assert g.is_connected_without_vertex(1, "original") is True
    assert g.adj[1] == [2]  # self-loop lost
    g = G.create_2d_grid_graph(3, 3, True)
    G.update_adj_list(g)
    g.is_connected_without_vertex(2 - 1, "original")
    assert g.adj[1] == [0, 1, 2, 4]  # self-loop added back


# ------------------------------------------------------------------- removal
@pytest.mark.parametrize("case", by_kind("remove_vertex"), ids=case_id)
def test_remove_vertex(case):
    v = case["args"]["v"]
    n = 2 if v == 0 else 3
    g = G.create_2d_grid_graph(n, n, True)
    g.remove_vertex(v, "original")
    assert_graph(g, case["result"], exact_order=True)  # vertices stays stale
    h = G.create_2d_grid_graph(n, n, True)
    h.remove_vertex(v)
    assert h.adj == g.adj and v not in h.vertices


@pytest.mark.parametrize("case", by_kind("remove_vertex_xy", "remove_vertex_xy_empty"), ids=case_id)
def test_remove_vertex_xy(case):
    a = case["args"]
    f = G.create_2d_grid_graph if case["kind"] == "remove_vertex_xy" else G.create_2d_grid_graph_8connected
    g = f(a["rows"], a["cols"], False, "original")
    g.remove_vertex_xy(a["x"], a["y"], "original")
    assert_graph(g, case["result"])


# ------------------------------------------------------------------- edge ids
def test_edge_ids():
    c = by_kind("edge_id")[0]
    py = [G.get_edge_id(5, 3), G.get_edge_id(3, 5), G.get_edge_id(0, 0), G.get_edge_id(70000, 123456),
          G.get_first_vertex_of_edge(300000005), G.get_second_vertex_of_edge(300000005),
          G.get_first_vertex_of_edge(-300000005), G.get_second_vertex_of_edge(-300000005),
          G.get_first_vertex_of_edge(900000000000000007)]
    assert py == c["result"]


@pytest.mark.parametrize("case", by_kind("edge_set_garage", "edge_set_grid", "edge_set_unconverted"), ids=case_id)
def test_edge_set(case):
    from mppflow.problem import get_long_straight_with_one_garage_problem
    if case["kind"] == "edge_set_garage":
        g = get_long_straight_with_one_garage_problem().graph
    elif case["kind"] == "edge_set_grid":
        g = G.create_2d_grid_graph(6, 7, True)
    else:
        g = G.create_2d_grid_graph(2, 2, False)
    if case["exception"]:
        with pytest.raises(JavaNullPointerException):
            g.get_edge_set("original")
        with pytest.raises(ValueError):
            g.get_edge_set()
    else:
        assert g.get_edge_set("original") == case["result"]
        assert g.get_edge_set() == case["result"]


def test_update_adj_list():
    c = by_kind("update_adj_list")[0]
    g = G.create_2d_grid_graph(2, 3, True)
    G.update_adj_list(g)
    assert_graph(g, c["result"], exact_order=True)


# ------------------------------------------------------------------- printing
def _print_graph(name):
    from mppflow.problem import create_grid_problem, get_long_straight_with_one_garage_problem
    return {
        "star": _star,
        "tree": G.create_main_case6_tree,
        "grid3": lambda: G.create_2d_grid_graph(3, 3, True),
        "garage": lambda: get_long_straight_with_one_garage_problem().graph,
        "problem15": lambda: create_grid_problem(15, 15, 0.2, 15, MathRandom(1)).graph,
        "unconverted": lambda: G.create_2d_grid_graph(2, 2, False),
    }[name]()


@pytest.mark.parametrize("case", by_kind("print_adj"), ids=case_id)
def test_print_adjacency_list(case, capsys):
    g = _print_graph(case["args"]["graph"])
    if case["exception"]:
        with pytest.raises(JavaNullPointerException) as ei:
            g.print_graph_as_adjacency_list("original")
        assert ei.value.output == case["stdout"]
        # fixed prints each vertex under its own id
        text = g.print_graph_as_adjacency_list()
        assert text.startswith("1: 2, 3, \n")
    else:
        assert g.print_graph_as_adjacency_list("original") == case["stdout"]
        assert g.print_graph_as_adjacency_list() == case["stdout"]
        g.print_graph_as_adjacency_list(echo=True)
        assert capsys.readouterr().out == case["stdout"]
        assert g.print_graph_as_adjaceny_list() == case["stdout"]


@pytest.mark.parametrize("case", by_kind("print_vertex"), ids=case_id)
def test_print_vertex(case):
    g = _print_graph(case["args"]["graph"])
    out = "".join(G.vertex_print(g, v) + "|" + G.print_vertex(g, v) + "|" for v in g.vertices)
    buf = "".join(G.print_vertex_to_buffer(g, v) + "|" for v in g.vertices)
    assert out == case["stdout"]
    assert buf == case["result"]


@pytest.mark.parametrize("case", by_kind("print_path"), ids=case_id)
def test_print_path(case):
    g = _print_graph(case["args"]["graph"])
    path = case["args"]["path"]
    if case["exception"]:
        with pytest.raises(JavaNullPointerException) as ei:
            G.print_path(g, path, "original")
        assert ei.value.output == case["stdout"]
        assert G.print_path(g, path) == "0(0,0)|1(1,0)|42|5(2,1)|\n"
    else:
        assert G.print_path(g, path, "original") == case["stdout"]
        assert G.print_path(g, path) == case["stdout"]


# ---------------------------------------------------------------- test fixtures
def test_v4_and_bridge_fixtures():
    c = by_kind("v4_test_graph")[0]
    assert_graph(G.get_v4_test_graph(), c["result"]["graph"])
    assert G.SG == c["result"]["SG"]
    c = by_kind("bridge_test_graph")[0]
    assert_graph(G.get_test_graph_for_bridge(), c["result"], exact_order=True)


def test_manual_graphs():
    assert_graph(_star(), by_kind("manual_star")[0]["result"], exact_order=True)
    assert_graph(G.create_main_case6_tree(), by_kind("manual_tree")[0]["result"], exact_order=True)
    g = G.Graph.from_adjacency([[1], [0, 2, 3], [1], [1]])
    assert g.adj == _star().adj and g.vertices == [0, 1, 2, 3]


def test_hashset_adjacency_order():
    g = G.create_2d_grid_graph(5, 5, True)
    g.set_hashset_adjacency(11, [11, 6, 10, 12, 16])
    assert g.adj[11] == [6, 10, 11, 12, 16]
    assert g.neighbors(11) == [16, 6, 10, 11, 12]
    assert g.neighbors(0) == [0, 1, 5]


def test_ix_truncation():
    g = G.Graph()
    g.add_vertex_point(0, (-0.5, 2.7))
    g.add_vertex(1, [])
    assert g.ix(0) == 0 and g.iy(0) == 2
    with pytest.raises(JavaNullPointerException):
        g.ix(1, "original")
    with pytest.raises(ValueError):
        g.iy(1)


def test_compat_validation():
    with pytest.raises(ValueError):
        G.create_2d_grid_graph(2, 2, True, "bogus")


def test_index_error_class():
    assert issubclass(JavaIndexOutOfBoundsException, JavaException)


# ------------------------------------------------- corner cases (recorded live, JDK 17)
# Hole generators asked for more holes than vertices, and narrow 8-connected grids.
# [probe case "kind rows cols p seed", Java exception (HANG = still running after 4 s),
# next Math.random() after the call]. Recorded with a Java probe against the original classes.
HOLES_JAVA = [
    ["holes8 1 1 1.2 775783", None, 0.4533942671925869],
    ["holes8 1 1 1.5 243389", None, 0.5635152289651454],
    ["holes8 1 1 2.0 411628", "HANG", None],
    ["holes8 1 1 3.0 588297", "HANG", None],
    ["holes8 2 1 1.2 419132", "java.lang.NullPointerException", 0.8282928962769344],
    ["holes8 2 1 1.5 180735", "java.lang.NullPointerException", 0.47157397942160173],
    ["holes8 2 1 2.0 507116", "java.lang.NullPointerException", 0.34359384830253803],
    ["holes8 2 1 3.0 828131", "java.lang.NullPointerException", 0.23644649465903855],
    ["holes8 3 1 1.2 272128", "java.lang.NullPointerException", 0.4850457794354336],
    ["holes8 3 1 1.5 908819", "java.lang.NullPointerException", 0.32344422727176436],
    ["holes8 3 1 2.0 640052", "java.lang.NullPointerException", 0.553839799872339],
    ["holes8 3 1 3.0 345656", "java.lang.NullPointerException", 0.02719015736137309],
    ["holes8 4 1 1.2 750835", "java.lang.NullPointerException", 0.4377392479527218],
    ["holes8 4 1 1.5 233117", "java.lang.NullPointerException", 0.399729143026459],
    ["holes8 4 1 2.0 271337", "java.lang.NullPointerException", 0.779186964275245],
    ["holes8 4 1 3.0 639579", "java.lang.NullPointerException", 0.9633724924420631],
    ["holes8 5 1 1.2 741018", "java.lang.NullPointerException", 0.10852661411610065],
    ["holes8 5 1 1.5 256091", "java.lang.NullPointerException", 0.7555333386507006],
    ["holes8 5 1 2.0 884780", "java.lang.NullPointerException", 0.4477325086162539],
    ["holes8 5 1 3.0 692941", "java.lang.NullPointerException", 0.6840748128846641],
    ["holes8 6 1 1.2 32017", "java.lang.NullPointerException", 0.08835280804995438],
    ["holes8 6 1 1.5 893147", "java.lang.NullPointerException", 0.7567755036558088],
    ["holes8 6 1 2.0 943082", "java.lang.NullPointerException", 0.27633567793603364],
    ["holes8 6 1 3.0 910616", "java.lang.NullPointerException", 0.6730764281129658],
    ["holes4 1 1 1.5 282162", None, 0.42951148008033935],
    ["holes4 1 1 2.5 199125", "HANG", None],
    ["holes4 2 1 1.5 76063", "HANG", None],
    ["holes4 2 1 2.5 656289", "HANG", None],
    ["holes4 3 1 1.5 767896", "HANG", None],
    ["holes4 3 1 2.5 173675", "HANG", None],
    ["holes4 2 2 1.5 913365", "HANG", None],
    ["holes8 2 2 1.5 607279", "HANG", None],
    ["holes4 1 3 1.5 465123", "HANG", None],
    ["holes8 1 3 1.5 609676", "HANG", None],
    ["holes4 4 2 1.5 957501", "HANG", None],
    ["holes8 4 2 1.5 978251", "HANG", None],
    ["holes4 2 2 1.0 1", None, 0.11596708803265776],
    ["holes8 2 2 1.0 2", None, 0.4082850955300814],
    ["holes4 1 1 1.0 3", None, 0.06712000939049956],
    ["holes8 1 1 1.0 4", None, 0.9186071189908658],
    ["holes4 3 3 1.0 5", None, 0.19043113981915427],
    ["holes8 3 3 1.0 6", None, 0.614067595532978],
    ["holes4 1 3 1.0 7", None, 0.31301371517872],
    ["holes8 4 1 1.0 8", "java.lang.NullPointerException", 0.559230256306747],
    ["holes4 3 1 1.0 9", None, 0.11744872475171408],
    ["holes8 2 1 1.0 10", "java.lang.NullPointerException", 0.8188090228552316],
]


@pytest.mark.parametrize("case,jexc,jnext", HOLES_JAVA, ids=[c[0] for c in HOLES_JAVA])
def test_holes_too_many_original(case, jexc, jnext):
    k, r, c, p, seed = case.split()
    r, c, p = int(r), int(c), float(p)
    f = G.create_generic_2d_grid_graph_with_holes if k == "holes4" else \
        G.create_generic_2d_grid_graph_with_holes_8connected
    rng = MathRandom(int(seed))
    exc = None
    try:
        g = f(r, c, p, rng, "original")
    except OriginalWouldHang:
        exc = "HANG"
    except JavaException as e:
        exc = java_class(e)
    assert exc == jexc
    if exc != "HANG":
        assert rng.random() == jnext
    if exc is None:
        assert len(g.xy) == r * c - int(r * c * p)
    # fixed: more holes than vertices is refused before any draw
    rng = MathRandom(int(seed))
    if int(r * c * p) > r * c:
        with pytest.raises(ValueError):
            f(r, c, p, rng)
        assert rng.draws == 0


def test_holes_too_many_grid_problem_8c():
    # createGridProblem8Connected(col, 1, frac > 1, 0): NullPointerException, not a hang.
    from mppflow.problem import create_grid_problem_8connected
    for col, frac, seed, nxt in ((2, 2.0, 422155, 0.5347186789271519), (3, 1.5, 973076, 0.14263313697899171),
                                 (4, 1.5, 978451, 0.3481760822500808), (5, 1.2, 823564, 0.42602707098583525)):
        rng = MathRandom(seed)
        with pytest.raises(JavaNullPointerException):
            create_grid_problem_8connected(col, 1, frac, 0, rng, "original")
        assert rng.random() == nxt
        with pytest.raises(ValueError):
            create_grid_problem_8connected(col, 1, frac, 0, MathRandom(seed))


# convertGraph error paths: "vertices;id:nbrs ...;starts;goals" -> Java result
# (live, JDK 17, a Java probe). Entries: (vertices, adjacency, starts,
# goals, Java exception or adjacency after conversion, starts after, goals after).
CONVERT_JAVA = [
    ([3, 1, 2], {1: [1, 2], 2: [1, 2, 3], 3: [2, 3]}, [3, 1], [1, 2],
     {0: [0, 1], 1: [0, 1, 2], 2: [1, 2]}, [2, 0], [0, 1]),
    ([5, 5, 7], {5: [5, 7], 7: [5, 7]}, [5], [7], "java.lang.ArrayIndexOutOfBoundsException", [5], [7]),
    ([5, 7], {5: [5, 7]}, [5], [7], "java.lang.NullPointerException", [5], [7]),
    ([5, 7], {5: [5, 7, 9], 7: [5, 7]}, [5], [7], "java.lang.NullPointerException", [5], [7]),
    ([5, 7], {5: [5, 7], 7: [5, 7]}, [9], [7], "java.lang.NullPointerException", [9], [7]),
    ([5, 7], {5: [5, 7], 7: [5, 7]}, [5], [9], "java.lang.NullPointerException", [0], [9]),
    ([5, 7], {5: [5, 7], 7: [5, 7]}, [5, 7], [5], "java.lang.ArrayIndexOutOfBoundsException", [0, 1], [0]),
    ([5, 7], {5: [5, 7], 7: []}, [7, 5], [5, 7], {0: [0, 1], 1: []}, [1, 0], [0, 1]),
    ([], {}, [], [], {}, [], []),
]


@pytest.mark.parametrize("compat", ["original", "fixed"])
@pytest.mark.parametrize("case", CONVERT_JAVA, ids=[str(i) for i in range(len(CONVERT_JAVA))])
def test_convert_error_paths(case, compat):
    vs, adj, s, t, jres, js, jt = case
    g = G.Graph()
    g.vertices = list(vs)
    for v in vs:
        g.xy[v] = (v, 0)
    g.adj = {k: list(nb) for k, nb in adj.items()}
    s, t = list(s), list(t)
    if isinstance(jres, str):
        if compat == "original":
            with pytest.raises(JavaException) as ei:
                G.convert_graph(g, s, t, compat)
            assert java_class(ei.value) == jres
            assert (s, t) == (js, jt)  # renumbered in place up to the crash
        else:
            with pytest.raises(ValueError):
                G.convert_graph(g, s, t, compat)
        return
    og = G.convert_graph(g, s, t, compat)
    assert og.adj == jres and og.vertices == list(range(len(vs)))
    assert (s, t) == (js, jt)


def test_random_sg_negative_number():
    from mppflow.errors import JavaNegativeArraySizeException
    g = G.create_2d_grid_graph(3, 3, False)
    for fn in (G.get_random_start_goal, G.get_random_start_goal_many):
        rng = MathRandom(1)
        with pytest.raises(JavaNegativeArraySizeException):  # new int[2][number]
            fn(g, -1, rng, "original")
        with pytest.raises(ValueError):
            fn(g, -1, rng)
        assert rng.draws == 0


def test_hashset_adjacency_mutation():
    # remove_vertex / the original's self-loop edits keep adj_order in Java HashSet order.
    from mppflow._java import JavaIntHashSet
    g = G.create_2d_grid_graph(5, 5, True)
    for v in range(g.n):
        g.set_hashset_adjacency(v, [v] + [u for u in g.adj[v] if u != v][::-1])
    ref = {v: JavaIntHashSet([v] + [u for u in g.adj[v] if u != v][::-1]) for v in range(g.n)}
    g.remove_vertex(12)
    for k in (7, 11, 13, 17):
        ref[k].remove(12)
        assert g.neighbors(k) == ref[k].to_list() and g.adj[k] == sorted(ref[k])
    assert g._set_remove(7, 7) and not g._set_remove(7, 7)
    ref[7].remove(7)
    assert g.neighbors(7) == ref[7].to_list()
    assert g._set_add(7, 7) and not g._set_add(7, 7)
    ref[7].add(7)
    assert g.neighbors(7) == ref[7].to_list() and g.adj[7] == sorted(ref[7])
    assert g._set_add(0, 24)  # an id that lands in another bucket
    ref[0].add(24)
    assert g.neighbors(0) == ref[0].to_list()


def test_hashset_adjacency_remove_empties():
    g = G.Graph()
    g.add_vertex(0, [1])
    g.add_vertex(1, [0])
    g.finish_building_graph()
    g.set_hashset_adjacency(0, [1])
    g.set_hashset_adjacency(1, [0])
    g.remove_vertex(1)
    assert g.adj == {} and g.adj_order == {}


def test_convert_graph_renumbers_the_lists_in_place():
    """docs/API.md: starts and goals are renumbered in place (as the Java convertGraph);
    ``fixed`` rejects tuples (TypeError), unequal lengths and unknown ids (ValueError)
    without touching the lists."""
    from mppflow import MathRandom
    g = G.create_generic_2d_grid_graph_with_holes(4, 4, 0.3, MathRandom(2))
    ids = sorted(g.adj)
    s, t = [ids[-1], ids[-2]], [ids[0], ids[1]]
    og = G.convert_graph(g, s, t)
    assert (s, t) == ([len(ids) - 1, len(ids) - 2], [0, 1]) and og.vertices == list(range(len(ids)))
    with pytest.raises(TypeError, match="in place"):
        G.convert_graph(g, (ids[-1],), (ids[0],))
    for a, b in [([ids[-1]], None), ([ids[-1]], [ids[0], ids[1]]), ([ids[-1], 10**6], [ids[0], ids[1]]),
                 ([ids[-1], ids[-2]], [ids[0], 10**6])]:
        a0, b0 = list(a), None if b is None else list(b)
        with pytest.raises(ValueError):
            G.convert_graph(g, a, b)
        assert (a, b) == (a0, b0)
