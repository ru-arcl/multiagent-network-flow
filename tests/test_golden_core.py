"""The ``graph``, ``problem`` and ``pathfinder`` records of ``tests/golden`` against the port
(``compat="original"``): result (or exception class, or hang), printed text, mutated
arguments, files written, random draws and the next ``Math.random()`` value.

The records are recorded live from the original Java code by ``tools/golden/record.py``.
Each record runs through the same Python-side runner as the differential tests
(``tools/golden/differential.py``: :func:`run_python`, :func:`compare`); the functions that
runner does not cover (the dead helpers, the file writers, the batch generators) are mapped
here.
"""

from __future__ import annotations

import base64
import contextlib
import hashlib
import io
import os
import sys

import pytest

from mppflow import graph as G
from mppflow import pathfinder as PF
from mppflow import problem as PR
from mppflow.errors import JavaNullPointerException
from helpers import load_golden

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools", "golden"))
import differential as D  # noqa: E402

C = "original"
GROUPS = ("graph", "problem", "pathfinder")
RECORDS = [r for g in GROUPS for r in load_golden(g)]
BATCH = {  # Problem.<fn> -> generate_benchmark_files families
    "Problem.createN2Puzzles": ["n2puzzle"],
    "Problem.createCrowdedHardProblems": ["crowded"],
    "Problem.create32x32Problems": ["32x32"],
    "Problem.create32x32Problems8Connected": ["32x32-8c"],
    "Problem.create24x18PerformanceTestingProblems": ["24x18"],
    "Problem.createCrowdedHardProblems8Connected": ["crowded-8c"],
}
SLOW_BATCH = {"Problem.create24x18PerformanceTestingProblems", "Problem.main"}


# ------------------------------------------------------------------ encodings
def dec_graph(d) -> G.Graph:
    """A golden graph, with the original's stale ``vertices`` array: ``idVertexMap_extra``
    (ids in the id map only) and ``idVertexMap_missing`` (ids in the array only)."""
    g = D.graph_from_json(d)
    for e in d.get("idVertexMap_extra", ()):
        g.xy[e[0]] = (e[1], e[2]) if len(e) == 3 else None
    for v in d.get("idVertexMap_missing", ()):
        g._ghost_xy[v] = g.xy.pop(v)
    return g


def enc_graph(g: G.Graph) -> dict:
    d = D.graph_to_json(g)
    arr = set(g.vertices)
    missing = [v for v in g.vertices if v not in g.xy]
    extra = [[v] if g.xy[v] is None else [v, D._num(g.xy[v][0]), D._num(g.xy[v][1])]
             for v in sorted(g.xy) if v not in arr]
    if missing:
        d["idVertexMap_missing"] = missing
    if extra:
        d["idVertexMap_extra"] = extra
    return d


def dec_problem(d) -> PR.Problem:
    return PR.Problem.from_sg(dec_graph(d["graph"]), d["sg"])


def enc_problem(p: PR.Problem) -> dict:
    return {"graph": None if p.graph is None else enc_graph(p.graph), "sg": p.sg}


@contextlib.contextmanager
def printed(out: dict):
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            yield
    finally:
        out["stdout"] = buf.getvalue()


def _manual(a):
    g = G.Graph()
    for v, nb in a["neighbors"]:
        g.add_vertex(v, nb)
    g.finish_building_graph()
    return g


# fn -> f(args, rng, out): the result in the golden encoding; ``out`` takes "stdout" and
# "mutated" (as differential._py_call)
def _mutating(f, key="g"):
    def run(a, rng, out):
        g = dec_graph(a[key])
        r = f(g, a)
        out["mutated"] = {key: enc_graph(g)}  # the harness records it after a normal return
        return r
    return run


def _vertex(g, v):
    """The harness passes ``g.idVertexMap.get(vId)``: ``null`` for an absent id, so the
    original's ``removeVertex(Vertex)`` dereferences null."""
    if v not in g.xy:
        raise JavaNullPointerException('Cannot read field "id" because "<parameter1>" is null')
    return v


def _convert(a, rng, out):
    s, t = list(a["start"]), list(a["goal"])
    r = enc_graph(G.convert_graph(dec_graph(a["graph"]), s, t, C))
    if s or t:
        out["mutated"] = {"start": s, "goal": t}
    return r


def _print_graph(a, rng, out):
    text = []
    try:
        text.append(dec_graph(a["g"]).print_graph_as_adjacency_list(C))
    except Exception as e:
        text.append(getattr(e, "output", None) or "")
        raise
    finally:
        out["stdout"] = "".join(text)


def _print_path(a, rng, out):
    out["stdout"] = G.print_path(dec_graph(a["g"]), a["path"], C)


EXTRA = {
    "Graph.manual": lambda a, rng, out: enc_graph(_manual(a)),
    "Graph.getv4TestGraph": lambda a, rng, out: enc_graph(G.get_v4_test_graph()),
    "Graph.getTestGraphForBridge": lambda a, rng, out: enc_graph(G.get_test_graph_for_bridge()),
    "Graph.convertGraph": _convert,
    "Graph.getEdgeSet": lambda a, rng, out: dec_graph(a["g"]).get_edge_set(C),
    "Graph.getEdgeId": lambda a, rng, out: [
        [e, G.get_first_vertex_of_edge(e), G.get_second_vertex_of_edge(e)]
        for e in (G.get_edge_id(u, v) for u, v in a["pairs"])],
    "Graph.removeVertex(Vertex)": _mutating(lambda g, a: g.remove_vertex(_vertex(g, a["vId"]), C)),
    "Graph.removeVertex(int,int)": _mutating(lambda g, a: g.remove_vertex_xy(a["x"], a["y"], C)),
    "Graph.addVertex(Vertex)": _mutating(
        lambda g, a: g.add_vertex_point(a["id"], None if a.get("x") is None else (a["x"], a["y"]))),
    "Graph.addVertex(int,int[])": _mutating(lambda g, a: g.add_vertex(a["vId"], a["neighbors"])),
    "Graph.finishBuildingGraph": _mutating(lambda g, a: g.finish_building_graph()),
    "Graph.getId": lambda a, rng, out: dec_graph(a["g"]).get_id(a["x"], a["y"]),
    "Graph.updateAdjList": _mutating(lambda g, a: G.update_adj_list(g)),
    "Graph.isConnectedWithoutVertex": _mutating(lambda g, a: g.is_connected_without_vertex(a["vId"], C)),
    "Graph.areVerticesConnectedWithoutV3": lambda a, rng, out: dec_graph(a["g"]).are_vertices_connected_without_v3(
        a["vId1"], a["vId2"], a["vId3"]),
    "Graph.printGraphAsAdjacenyList": _print_graph,
    "Vertex.print": lambda a, rng, out: out.update(stdout=G.vertex_print(dec_graph(a["g"]), a["vId"])),
    "Vertex.printVertex": lambda a, rng, out: out.update(stdout=G.print_vertex(dec_graph(a["g"]), a["vId"])),
    "Vertex.printVertex(StringBuffer)": lambda a, rng, out: G.print_vertex_to_buffer(dec_graph(a["g"]), a["vId"]),
    "Path.printPath": _print_path,
    "Problem.createN2M1Puzzle": lambda a, rng, out: enc_problem(PR.create_n2m1_puzzle(a["n"], rng, C)),
    "Problem.getA9Puzzle": lambda a, rng, out: enc_problem(PR.get_a9_puzzle()),
    "Problem.getLongStraightWithOneGarageProblem":
        lambda a, rng, out: enc_problem(PR.get_long_straight_with_one_garage_problem()),
    "PathFinder.getTotalDistance(int[][])": lambda a, rng, out: PF.get_total_distance(a["paths"]),
    "PathFinder.getTotalDistance(Path[])": lambda a, rng, out: PF.get_total_path_length(a["paths"]),
}
"""Functions that :func:`differential._py_call` does not map."""

FILES = {  # the writers: (writer, file name in the record)
    "Problem.writeToFile(String)": (PR.write_txt, "out/problem.txt"),
    "Problem.writeToFile(Problem,String)": (PR.write_dat, "problem.dat"),
}


def _file_entry(path: str) -> dict:
    with open(path, "rb") as f:
        b = f.read()
    return {"size": len(b), "sha256": hashlib.sha256(b).hexdigest()}


def batch_digest(folder: str, samples: int) -> dict:
    """``Harness.batchDigest``: counts and SHA-256 digests of a generated folder."""
    names = sorted(os.listdir(folder))

    def digest(sel):
        h = hashlib.sha256()
        for n in sel:
            with open(os.path.join(folder, n), "rb") as f:
                b = f.read()
            h.update(n.encode() + b"\0" + str(len(b)).encode() + b"\0" + b)
        return h.hexdigest()

    dat = [n for n in names if n.endswith(".dat")]
    txt = [n for n in names if not n.endswith(".dat")]
    h = hashlib.sha256()
    for n in names:
        h.update(n.encode() + b"\n")
    smp = {}
    for n in names:
        if n.endswith(".txt") and len(smp) < samples:
            with open(os.path.join(folder, n), "rb") as f:
                smp[n] = f.read().decode("utf-8", errors="replace")  # as Java's new String(b, UTF_8)
    return {"nfiles": len(names), "ntxt": len(txt), "ndat": len(dat), "sha256_txt": digest(txt),
            "sha256_dat": digest(dat), "sha256_names": h.hexdigest(), "first_names": names[:6], "samples": smp}


def _params():
    out = []
    for r in RECORDS:
        slow = r["fn"] in SLOW_BATCH
        out.append(pytest.param(r, id=r["id"], marks=pytest.mark.slow if slow else ()))
    return out


# --------------------------------------------------------------------- the test
@pytest.mark.parametrize("rec", _params())
def test_golden_record(rec, tmp_path, monkeypatch):
    fn = rec["fn"]
    monkeypatch.chdir(tmp_path)
    if fn in BATCH or fn == "Problem.main":
        _check_batch(rec, tmp_path)
        return
    if fn in FILES or fn in ("Problem.roundTrip", "Problem.readFromFile"):
        _check_io(rec, tmp_path)
        return
    orig = D._py_call

    def py_call(case, rng, out):
        f = EXTRA.get(case["fn"])
        return f(case["args"], rng, out) if f is not None else orig(case, rng, out)

    monkeypatch.setattr(D, "_py_call", py_call)
    p = D.run_python(rec, timeout=120)
    assert "py_error" not in p, p.get("py_traceback") or p["py_error"]
    if "mutated" in p and "mutated" not in rec and "exception" not in rec:
        # arguments the original leaves unchanged are not recorded
        a = rec["args"]
        assert all(_same(v, a.get(k)) for k, v in p["mutated"].items()), p["mutated"]
        del p["mutated"]
    assert D.compare(rec, rec, p) == []


def _same(enc, given) -> bool:
    if isinstance(given, dict) and "vertices" in given:
        return D._jnum_eq(enc, enc_graph(dec_graph(given)))
    return D._jnum_eq(enc, given)


def _check_io(rec, tmp_path):
    """The file writers and readers: result or exception class, files written, stderr."""
    fn, a = rec["fn"], rec["args"]
    err = io.StringIO()
    got_exc = None
    result = None
    with contextlib.redirect_stderr(err):
        try:
            if fn in FILES:
                writer, name = FILES[fn]
                writer(dec_problem(a["p"]), name, C)
            elif fn == "Problem.roundTrip":
                PR.write_dat(dec_problem(a["p"]), "rt.dat", C)
                result = enc_problem(PR.read_dat("rt.dat", C))
            else:  # readFromFile
                if a["source"] == "text":
                    with open("in.dat", "w", encoding="utf-8") as f:
                        f.write(a["data"])
                result = enc_problem(PR.read_dat("in.dat", C))
        except Exception as e:  # noqa: BLE001 - compared with the record
            got_exc = e
    if "exception" in rec:
        assert got_exc is not None and getattr(got_exc, "java_class", None) == rec["exception"]["class"]
    else:
        assert got_exc is None, got_exc
        assert D._jnum_eq(rec.get("result"), result)
    for name, f in (rec.get("files") or {}).items():
        path = tmp_path / name
        assert path.exists(), name
        got = _file_entry(str(path))
        assert (got["size"], got["sha256"]) == (f["size"], f["sha256"]), name
        if "base64" in f:
            assert base64.b64encode(path.read_bytes()).decode() == f["base64"]
        if "text" in f:
            assert path.read_text(encoding="utf-8") == f["text"]
    if rec.get("stderr"):
        java = rec["stderr"].splitlines()[0].replace("$CASEDIR/", "")
        assert err.getvalue().splitlines()[0] == java


def _check_batch(rec, tmp_path):
    """The batch generators (``folder``: ``"case"`` / ``"none"`` = a fresh folder, as
    ``FILE_FOLDER`` or ``Problem.main``'s argument; ``"default"`` = the original's
    ``FILE_FOLDER``, a Windows path)."""
    a = rec["args"]
    folder = str(tmp_path / "batch") + "/"
    os.makedirs(folder)
    rng = D.MathRandom(rec["seed"])
    fams = BATCH.get(rec["fn"], PR.BENCHMARK_FAMILIES)  # Problem.main: every family in order
    if a.get("folder") == "default":
        with pytest.raises(JavaNullPointerException) as e:
            PR.generate_benchmark_files(None, rng, fams, C)
        assert e.value.java_class == rec["exception"]["class"]
    else:
        PR.generate_benchmark_files(folder, rng, fams, C)
        assert batch_digest(folder, a.get("samples", 1)) == rec["result"]
    assert rng.draws == rec["draws"] and rng.next_double() == rec["rng_next"]


def test_every_record_is_covered():
    """Every function of the three groups is compared above (none is format-only)."""
    seen = {r["fn"] for r in RECORDS}
    differential = {"Graph.create2DGridGraph", "Graph.create2DGridGraph8Connected",
                    "Graph.createGeneric2DGridGraphWithHoles", "Graph.createGeneric2DGridGraphWithHoles8Connected",
                    "Graph.getRandomStartGoal", "Graph.getRandomStartGoalMany", "Problem.createGridProblem",
                    "Problem.createGridProblem8Connected", "Problem.createN2Puzzle",
                    "PathFinder.getDistancesToAllVertices", "PathFinder.findGoalsInDistanceRange",
                    "PathFinder.findShortestPaths", "PathFinder.findOneShortestPath",
                    "PathFinder.getMakespanLowerBound", "PathFinder.getTotalTimeLowerBound"}
    mapped = differential | set(EXTRA) | set(FILES) | set(BATCH) | {"Problem.main", "Problem.roundTrip",
                                                                    "Problem.readFromFile"}
    assert seen <= mapped, seen - mapped
    assert len(RECORDS) == 174
