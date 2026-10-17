"""mppflow.problem against the live original (fixtures recorded by the Java harness).

Fixtures: tests/data/core/problem.json.gz and families.json.gz, recorded by
``tools/golden/core/record_core.sh`` (``CoreGolden.java``) against the original classes
with Math.random seeded by reflection; dat_streams, dat_rows and writers.json.gz by
one-off probes (see tools/golden/README.md, "Core fixtures"), each case with its input. Checked: generated instances, the next random value after each
generator, ``.dat`` bytes (byte for byte, or SHA-256 for large instances), ``.txt`` text,
the Java read-back of the ``.dat`` files, the writer/reader crash behaviour, and the
batch generator (``Problem.main`` families: file names and contents).
"""
from __future__ import annotations

import base64
import hashlib
import json
import os

import pytest

from mppflow import problem as P
from mppflow._java import MathRandom
from mppflow.errors import JavaArrayIndexOutOfBoundsException, JavaException, JavaNullPointerException
from mppflow.graph import Graph, convert_graph, create_generic_2d_grid_graph_with_holes, get_random_start_goal
from helpers import case_id, load_core

DATA = os.path.join(os.path.dirname(__file__), "data", "core")


CASES = load_core("problem")
FAMILIES = load_core("families")


def by_kind(*kinds):
    out = [c for c in CASES if c["kind"] in kinds]
    assert out, kinds
    return out


def gdump(g):
    return None if g is None else g.to_dict()


def pdump(p):
    return {"graph": gdump(p.graph), "sg": p.sg}


def sha(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def make(case, compat, rng):
    a, k = case["args"], case["kind"]
    if k == "grid_problem":
        return P.create_grid_problem(a["col"], a["row"], a["p"], a["n"], rng, compat)
    if k == "grid_problem_8c":
        return P.create_grid_problem_8connected(a["col"], a["row"], a["p"], a["n"], rng, compat)
    if k == "n2_puzzle":
        return P.create_n2_puzzle(a["n"], rng, compat)
    if k == "n2m1_puzzle":
        return P.create_n2m1_puzzle(a["n"], rng, compat)
    raise KeyError(k)


# --------------------------------------------------------------------- generators
@pytest.mark.parametrize("case", by_kind("grid_problem", "grid_problem_8c", "n2_puzzle"), ids=case_id)
@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_generated_problem_files(case, compat, tmp_path):
    rng = MathRandom(case["seed"])
    p = make(case, compat, rng)
    if compat == "fixed" and case["kind"] == "grid_problem_8c" and case["args"]["row"] <= 2:
        # 8-connected grid with <= 2 columns: the original's adjacency is wrong here
        assert pdump(p) != case["result"]["problem"]
        return
    assert rng.random() == case["next"]
    r = case["result"]
    assert pdump(p) == r["problem"]
    dat = P.problem_to_dat_bytes(p, compat)
    assert sha(dat) == r["dat_sha"] and len(dat) == r["dat_len"]
    if r["dat_b64"] is not None:
        assert dat == base64.b64decode(r["dat_b64"])
    txt = P.problem_to_txt(p, compat=compat).encode()
    assert sha(txt) == r["txt_sha"]
    if r["txt"] is not None:
        assert txt.decode() == r["txt"]
    # files on disk, and the read-back equals the Java read-back
    P.write_dat(p, str(tmp_path / "x.dat"), compat)
    P.write_txt(p, str(tmp_path / "t" / "x.txt"), compat)
    assert (tmp_path / "x.dat").read_bytes() == dat
    assert (tmp_path / "t" / "x.txt").read_bytes() == txt
    q = P.read_dat(str(tmp_path / "x.dat"), compat)
    assert pdump(q) == r["readback"]
    t = P.read_txt(str(tmp_path / "t" / "x.txt"))
    assert pdump(t) == r["problem"]


@pytest.mark.parametrize("case", by_kind("n2m1_puzzle"), ids=case_id)
@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_n2m1(case, compat):
    rng = MathRandom(case["seed"])
    p = make(case, compat, rng)
    assert rng.random() == case["next"]
    assert pdump(p) == case["result"]


def test_fixed_problems():
    assert pdump(P.get_a9_puzzle()) == by_kind("a9_puzzle")[0]["result"]
    assert pdump(P.get_long_straight_with_one_garage_problem()) == by_kind("garage")[0]["result"]


def test_as_named_layout():
    p = P.create_grid_problem(24, 18, 0.0, 5, MathRandom(1))
    xs = [x for x, _ in p.graph.xy.values()]
    assert max(xs) == 17  # 24 rows x 18 columns, as the original
    q = P.create_grid_problem(24, 18, 0.0, 5, MathRandom(1), as_named=True)
    assert max(x for x, _ in q.graph.xy.values()) == 23
    assert max(y for _, y in q.graph.xy.values()) == 17


def test_tor_fig9_generator_fixture():
    """Seed-1 Fig. 9 instance (an independently recorded fixture of the original's output)."""
    path = os.path.join(os.path.dirname(__file__), "data", "paper", "tor16_java_original_outputs.json")
    with open(path, encoding="utf-8") as f:
        fx = json.load(f)["fig9_generator"]
    rng = MathRandom(1)
    p = P.create_grid_problem(24, 18, 0.25, 50, rng)
    assert len(p.graph.vertices) == fx["vertices"] == 324
    assert p.starts == fx["seed1_starts"] and p.goals == fx["seed1_goals"]
    assert [list(p.graph.xy[v]) for v in p.graph.vertices] == [list(c) for c in fx["seed1_coords_xy"]]
    assert rng.random() == fx["seed1_next_random_after"]


# ------------------------------------------------------------------ writer crashes
def test_dat_write_null_point(tmp_path):
    c = by_kind("dat_write_null_point")[0]["result"]
    p = P.get_long_straight_with_one_garage_problem()
    fn = tmp_path / "np.dat"
    with pytest.raises(JavaNullPointerException):
        P.write_dat(p, str(fn), "original")
    assert c["exception"] == "java.lang.NullPointerException"
    assert fn.read_bytes() == base64.b64decode(c["b64"]) and len(fn.read_bytes()) == c["len"] == 227
    fn2 = tmp_path / "np2.dat"
    with pytest.raises(ValueError):
        P.write_dat(p, str(fn2))
    assert not fn2.exists()


def test_txt_write_null_point(tmp_path):
    c = by_kind("txt_write_null_point")[0]["result"]
    p = P.get_long_straight_with_one_garage_problem()
    fn = tmp_path / "npt" / "np.txt"
    with pytest.raises(JavaNullPointerException):
        P.write_txt(p, str(fn), "original")
    assert fn.exists() and fn.stat().st_size == c["len"] == 0
    fn2 = tmp_path / "npt2" / "np.txt"
    with pytest.raises(ValueError):
        P.write_txt(p, str(fn2))
    assert not fn2.exists()


def test_txt_write_no_parent(tmp_path, monkeypatch):
    assert by_kind("txt_write_no_parent")[0]["exception"] == "java.lang.NullPointerException"
    monkeypatch.chdir(tmp_path)
    p = P.create_n2_puzzle(3, MathRandom(1))
    with pytest.raises(JavaNullPointerException):
        P.write_txt(p, "noparent.txt", "original")
    assert not (tmp_path / "noparent.txt").exists()
    P.write_txt(p, "noparent.txt")
    assert (tmp_path / "noparent.txt").read_text() == P.problem_to_txt(p)


def _star_problem(k):
    g = Graph()
    pts = [(0, 0)] + [(i, 1) for i in range(1, k + 1)]
    for i, pt in enumerate(pts):
        g.add_vertex_point(i, pt)
    g.adj[0] = list(range(k + 1))
    for i in range(1, k + 1):
        g.adj[i] = [0, i]
    g.finish_building_graph()
    return P.Problem(g, [1, 2], [2, 1])


@pytest.mark.parametrize("case", by_kind("dat_write_star"), ids=case_id)
def test_dat_write_star(case, tmp_path):
    k = case["args"]["k"]
    r = case["result"]
    p = _star_problem(k)
    fn = tmp_path / "star.dat"
    if r["exception"]:
        with pytest.raises(JavaArrayIndexOutOfBoundsException):
            P.write_dat(p, str(fn), "original")
        assert fn.read_bytes() == base64.b64decode(r["b64"])
        # fixed writes a longer row, which reads back (also by the original's reader)
        P.write_dat(p, str(fn))
        q = P.read_dat(str(fn))
        assert q.graph.adj == p.graph.adj and q.sg == p.sg
    else:
        P.write_dat(p, str(fn), "original")
        assert fn.read_bytes() == base64.b64decode(r["b64"])
        assert P.problem_to_dat_bytes(p) == base64.b64decode(r["b64"])


def test_dat_write_edge_cases():
    b = base64.b64decode(by_kind("dat_write_empty_graph")[0]["result"]["b64"])
    g = Graph()
    assert P.problem_to_dat_bytes(P.Problem(g, [], []), "original") == b
    b = base64.b64decode(by_kind("dat_write_null_sg")[0]["result"]["b64"])
    p = P.create_n2_puzzle(2, MathRandom(0))
    p.sg = None
    assert P.problem_to_dat_bytes(p, "original") == b
    b = base64.b64decode(by_kind("dat_write_null_graph")[0]["result"]["b64"])
    assert P.problem_to_dat_bytes(P.Problem(None, [1], [2]), "original") == b


def test_dat_unconverted_holes(tmp_path):
    c = by_kind("dat_unconverted_holes")[0]
    rng = MathRandom(c["seed"])
    g = create_generic_2d_grid_graph_with_holes(4, 5, 0.2, rng, "original")
    s, t = get_random_start_goal(g, 3, rng, "original")
    assert rng.random() == c["next"]
    java = c["result"]
    assert [s, t] == java["problem"]["sg"]
    # The original writes vertices[] in its identity-hash order; with that order our
    # bytes are identical.
    g.vertices = list(java["problem"]["graph"]["vertices"])
    b = P.problem_to_dat_bytes(P.Problem(g, s, t), "original")
    assert b == base64.b64decode(java["b64"])
    fn = tmp_path / "u.dat"
    fn.write_bytes(b)
    assert pdump(P.read_dat(str(fn), "original")) == java["readback"]
    assert P.read_dat(str(fn)).graph.rows == 4


# -------------------------------------------------------------------------- readers
def test_read_missing(tmp_path, capsys):
    c = by_kind("read_missing")[0]
    p = P.read_dat(str(tmp_path / "nope.dat"), "original")
    assert pdump(p) == c["result"]
    assert "FileNotFoundException" in capsys.readouterr().err
    with pytest.raises(FileNotFoundError):
        P.read_dat(str(tmp_path / "nope.dat"))


def test_read_garbage(tmp_path, capsys):
    c = by_kind("read_garbage")[0]
    fn = tmp_path / "g.dat"
    fn.write_bytes(b"0:0:0 1:1:0")
    assert pdump(P.read_dat(str(fn), "original")) == c["result"]
    assert "StreamCorruptedException" in capsys.readouterr().err
    with pytest.raises(ValueError):
        P.read_dat(str(fn))


@pytest.mark.parametrize("kind,cut", [("read_truncated", -20), ("read_truncated_graph", 400)])
def test_read_truncated(kind, cut, tmp_path):
    c = by_kind(kind)[0]
    b = P.problem_to_dat_bytes(P.create_n2_puzzle(3, MathRandom(5)))
    fn = tmp_path / "tr.dat"
    fn.write_bytes(b[:cut] if cut > 0 else b[:len(b) + cut])
    assert pdump(P.read_dat(str(fn), "original")) == c["result"]
    with pytest.raises(ValueError):
        P.read_dat(str(fn))


def test_txt_crlf_and_reader(tmp_path):
    p = P.create_grid_problem(3, 2, 0, 2, MathRandom(1))
    assert P.problem_to_txt(p) == ("0:0:0 1:1:0 2:0:1 3:1:1 4:0:2 5:1:2\n"
                                  "0:0 0:1 0:2 1:0 1:1 1:3 2:0 2:2 2:3 2:4 3:1 3:2 3:3 3:5 4:2 4:4 4:5 5:3 5:4 5:5\n"
                                  "3:0 1:5")
    fn = tmp_path / "w.txt"
    P.write_txt(p, str(fn), newline="\r\n")
    assert b"\r\n" in fn.read_bytes()
    q = P.read_txt(str(fn))
    assert pdump(q) == pdump(p)


def test_problem_sg_view():
    p = P.Problem(None, [1, 2], [3, 4])
    p.sg[0][1] = 7
    assert p.starts == [1, 7]
    p.sg = None
    assert p.starts is None and p.goals is None and p.sg is None
    q = P.Problem.from_sg(None, [[0], [1]])
    assert q.starts == [0] and q.goals == [1] and q.n_agents == 1


# -------------------------------------------------------------- batch generation
def _family_digest(files: dict[str, bytes]):
    md = hashlib.sha256()
    for name in sorted(files):
        md.update(name.encode() + b"\0" + sha(files[name]).encode() + b"\0")
    return md.hexdigest()


FAST_FAMILIES = {"n2puzzle", "crowded", "crowded-8c", "32x32", "32x32-8c"}


@pytest.mark.parametrize("case", [pytest.param(c, marks=() if c["args"]["family"] in FAST_FAMILIES else pytest.mark.slow,
                                               id=c["args"]["family"]) for c in FAMILIES])
def test_family_original(case, tmp_path):
    fam = case["args"]["family"]
    folder = str(tmp_path) + "/"
    rng = MathRandom(case["seed"])
    written = P.generate_benchmark_files(folder, rng, [fam], "original")
    assert rng.random() == case["next"]
    files = {n: (tmp_path / n).read_bytes() for n in os.listdir(tmp_path)}
    r = case["result"]
    assert len(files) == r["count"] == len(written)
    assert sorted(files)[:6] == r["first"]
    assert _family_digest(files) == r["sha"]


@pytest.mark.parametrize("fam", ["n2puzzle", "crowded"])
def test_family_fixed(fam, tmp_path):
    case = next(c for c in FAMILIES if c["args"]["family"] == fam)
    rng = MathRandom(case["seed"])
    written = P.generate_benchmark_files(str(tmp_path), rng, [fam])
    assert rng.random() == case["next"]
    files = {}
    for fn in written:
        rel = os.path.relpath(fn, str(tmp_path))
        sub, name = os.path.split(rel)
        with open(fn, "rb") as fh:
            files["\\" + sub + "\\" + name] = fh.read()
    assert _family_digest(files) == case["result"]["sha"]


def test_generate_errors(tmp_path):
    with pytest.raises(ValueError):
        P.generate_benchmark_files(None, MathRandom(1), ["n2puzzle"])
    with pytest.raises(ValueError):
        P.generate_benchmark_files(str(tmp_path), MathRandom(1), ["bogus"])
    with pytest.raises(JavaNullPointerException):
        P.generate_benchmark_files("out", MathRandom(1), ["n2puzzle"], "original")


def test_crowded_8c_fixed_writes_dat(tmp_path):
    written = P.generate_benchmark_files(str(tmp_path), MathRandom(106), ["crowded-8c"])
    assert len(written) == 310 and all(w.endswith(".dat") for w in written)
    p = P.read_dat(written[0])
    assert p.n_agents == 10


def test_dat_write_no_dir(tmp_path, capsys):
    assert by_kind("dat_write_no_dir")[0]["result"] is False
    p = P.create_n2_puzzle(2, MathRandom(1))
    fn = tmp_path / "nodir" / "x.dat"
    P.write_dat(p, str(fn), "original")  # printed and swallowed, as the original
    assert not fn.exists()
    assert "FileNotFoundException" in capsys.readouterr().err
    with pytest.raises(FileNotFoundError):
        P.write_dat(p, str(fn))


def test_txt_write_unopenable(tmp_path):
    assert by_kind("txt_write_unopenable")[0]["result"] is True
    p = P.create_n2_puzzle(2, MathRandom(1))
    d = tmp_path / "isdir" / "sub"
    d.mkdir(parents=True)
    P.write_txt(p, str(d), "original")  # swallowed
    assert d.is_dir()
    with pytest.raises(OSError):
        P.write_txt(p, str(d))


def test_read_wrong_class(tmp_path):
    c = by_kind("read_wrong_class")[0]
    assert c["exception"] == "java.lang.ClassCastException"
    fn = tmp_path / "wc.dat"
    w = P._Writer()  # an int[][] {{1}, {2}} as ObjectOutputStream writes it
    w.int2_array([[1], [2]])
    fn.write_bytes(bytes(w.out))
    with pytest.raises(P.JavaClassCastException):
        P.read_dat(str(fn), "original")
    with pytest.raises(ValueError):
        P.read_dat(str(fn))


# ------------------------------------------------ hand-built .dat streams
# tests/data/core/dat_streams.json.gz: hand-built serialization streams (unknown classes
# in either slot, unknown array classes, InvalidClassException, wrong array types,
# negative lengths, null descriptors, split/long block data, corrupt headers/handles,
# and more), each with the live result of the original's Problem.readFromFile
# (JDK 17, a Java probe): "graph=set|null
# sg=null|<rows>:<len of row 0>" or "THROWS <class>", and the first line it printed.
STREAMS = load_core("dat_streams")


def _read_outcome(path, compat):
    try:
        p = P.read_dat(path, compat)
    except JavaException as e:
        return "THROWS " + e.java_class
    sg = "null" if p.sg is None else f"{len(p.sg)}:{-1 if p.sg[0] is None else len(p.sg[0])}"
    return f"graph={'null' if p.graph is None else 'set'} sg={sg}"


@pytest.mark.parametrize("name", sorted(STREAMS))
def test_read_dat_streams(name, tmp_path, capsys):
    c = STREAMS[name]
    fn = tmp_path / (name + ".dat")
    fn.write_bytes(base64.b64decode(c["b64"]))
    assert _read_outcome(str(fn), "original") == c["java"]
    err = capsys.readouterr().err.split("\n")[0]
    # Same exception class and, for ClassNotFound/InvalidClass/StreamCorrupted, the
    # same message (EOFException messages are not reproduced).
    assert err.split(":")[0] == c["stderr"].split(":")[0]
    if "EOFException" not in err:
        assert err == c["stderr"]
    if c["java"].startswith("graph=") and not c["stderr"]:
        assert _read_outcome(str(fn), "fixed") == c["java"]
    else:
        with pytest.raises(ValueError):
            P.read_dat(str(fn))


def test_read_dat_unknown_class_keeps_graph(tmp_path, capsys):
    # Example: a valid Graph followed by an object of a missing class.
    fn = tmp_path / "u.dat"
    fn.write_bytes(base64.b64decode(STREAMS["review_unknown_sg"]["b64"]))
    p = P.read_dat(str(fn), "original")
    assert p.graph is not None and p.graph.n == 4 and p.sg is None
    assert capsys.readouterr().err.startswith("java.lang.ClassNotFoundException: zz.Nope")


# tests/data/core/dat_rows.json.gz: streams probing Graph.readObject's per-row loop
# (a degree v[i][3] < -1 must read no entries; the rows are processed
# inside readObject, before the two readInt calls, and for a Graph in any position).
# Degrees <= 0 / exact / too big, rows of length 0..4, null rows, duplicate and negative
# ids, a bad Graph followed by EOF / no block / a short block, a Graph (good or bad) as
# the sg object, inside the sg int[][] and as its own vertex array. Each with the live
# Problem.readFromFile result (JDK 17, a Java probe): vertex ids in vertices[] order, idVertexMap points, the adjacency
# sets, rows/columns and sg, or "THROWS <class>: <message>"; and the first stderr line.
ROWS = load_core("dat_rows")


def _rows_outcome(path, compat):
    try:
        p = P.read_dat(path, compat)
    except JavaException as e:
        return f"THROWS {e.java_class}: {e}"
    if p.graph is None:
        s = "graph=null"
    else:
        g = p.graph
        s = "vertices=" + "".join(f"{v}," for v in g.vertices)
        s += " pts=" + "".join(f"{k}({g.xy[k][0]},{g.xy[k][1]});" for k in sorted(g.xy))
        s += " adj=" + "".join(f"{k}:[{', '.join(map(str, g.adj[k]))}];" for k in sorted(g.adj))
        s += f" rc={g.rows},{g.cols}"
    sg = "null" if p.sg is None else "[" + ", ".join(
        "null" if r is None else "[" + ", ".join(map(str, r)) + "]" for r in p.sg) + "]"
    return s + " sg=" + sg


@pytest.mark.parametrize("name", sorted(ROWS))
def test_read_dat_graph_rows(name, tmp_path, capsys):
    c = ROWS[name]
    fn = tmp_path / (name + ".dat")
    fn.write_bytes(base64.b64decode(c["b64"]))
    # Java appends module/loader details to ClassCastException messages.
    assert _rows_outcome(str(fn), "original") == c["java"].split(" (projects.")[0]
    err = capsys.readouterr().err.split("\n")[0]
    assert err.split(":")[0] == c["stderr"].split(":")[0]
    if c["java"].startswith(("graph=", "vertices=")) and not c["stderr"]:
        assert _rows_outcome(str(fn), "fixed") == c["java"]
    else:
        with pytest.raises(ValueError):
            P.read_dat(str(fn))


def test_read_dat_negative_degree_reads_no_entries(tmp_path):
    # Reproduction: v[0][3] = -5 with trailing entries 7..15.
    fn = tmp_path / "neg.dat"
    fn.write_bytes(base64.b64decode(ROWS["review_negdeg5"]["b64"]))
    for compat in ("fixed", "original"):
        g = P.read_dat(str(fn), compat).graph
        assert g.adj == {0: [], 1: [0, 1]}


def test_read_dat_bad_row_before_int_block(tmp_path, capsys):
    # A short row followed by EOF, and a bad Graph in the sg slot: Java throws the
    # AIOOBE from Graph.readObject, not an EOFException or a ClassCastException.
    for name in ("short_then_eof", "null_then_badgraph"):
        fn = tmp_path / (name + ".dat")
        fn.write_bytes(base64.b64decode(ROWS[name]["b64"]))
        with pytest.raises(JavaArrayIndexOutOfBoundsException, match="Index 5 out of bounds for length 5"):
            P.read_dat(str(fn), "original")
        with pytest.raises(ValueError):
            P.read_dat(str(fn))
    assert capsys.readouterr().err == ""


# ----------------------------------------------- writer corner cases
# tests/data/core/writers.json.gz: Problem.writeToFile(Problem, String) and the private
# writeToFile(String) on a converted 2x2 grid, sg {{0,3},{3,0}}, with: graph null, sg[1]
# the same array as sg[0], sg null, sg[0] null, the adjacency entry of vertex 2 removed
# (live, JDK 17, a Java probe). Result and file bytes (hex).
WRITERS = load_core("writers")


def _writer_problem(name):
    g = create_generic_2d_grid_graph_with_holes(2, 2, 0.0, MathRandom(1))
    p = P.Problem(convert_graph(g), [0, 3], [3, 0])
    if name.endswith("nullgraph"):
        p.graph = None
    if name == "shared":
        p.goals = p.starts  # one array object, written once and then referenced
    if name.endswith("nullsg"):
        p.sg = None
    if name == "nullrow":
        p.starts = None
    if name.endswith("missingadj"):
        del p.graph.adj[2]
    return p


@pytest.mark.parametrize("name", sorted(WRITERS))
@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_writer_corner_cases(name, compat, tmp_path):
    c = WRITERS[name]
    p = _writer_problem(name)
    fn = tmp_path / (name + (".txt" if name.startswith("txt") else ".dat"))
    write = P.write_txt if name.startswith("txt") else P.write_dat
    if compat == "fixed" and c["java"] != "ok":
        with pytest.raises(ValueError):
            write(p, str(fn))
        return
    try:
        write(p, str(fn), compat)
        res = "ok"
    except JavaException as e:
        res = "THROWS " + e.java_class
    assert res == c["java"]
    assert fn.read_bytes().hex() == c["hex"]
    if res == "ok" and not name.startswith("txt"):
        q = P.read_dat(str(fn), compat)
        assert q.sg == p.sg and (q.graph is None) == (p.graph is None)


def test_writer_helpers():
    w = P._Writer()
    a = [[1], [2]]
    w.int2_array(None)
    w.int2_array(a)
    w.int2_array(a)  # back-reference
    w.int_array(None)
    assert bytes(w.out[4:5]) == b"\x70" and bytes(w.out[-6:-5]) == b"\x71" and bytes(w.out[-1:]) == b"\x70"
    assert "Problem(graph=None" in repr(P.Problem())


def test_txt_writer_errors(tmp_path):
    p = P.create_n2_puzzle(2, MathRandom(1))
    q = P.Problem(p.graph)  # no starts/goals
    for compat, exc in (("original", JavaNullPointerException), ("fixed", ValueError)):
        with pytest.raises(exc):
            P.problem_to_txt(q, compat=compat)
        with pytest.raises(exc):
            P.problem_to_txt(P.Problem(None, [0], [1]), compat=compat)
    # original: a parent that cannot be created is swallowed like the unopenable file
    blocker = tmp_path / "afile"
    blocker.write_text("x")
    P.write_txt(p, str(blocker / "sub" / "p.txt"), "original")
    with pytest.raises(OSError):
        P.write_txt(p, str(blocker / "sub" / "p.txt"))


def test_read_txt_short_file(tmp_path):
    fn = tmp_path / "s.txt"
    fn.write_text("0:0:0 1:1:0\n0:1 1:0")
    p = P.read_txt(str(fn))
    assert p.graph.adj == {0: [1], 1: [0]} and p.starts == [] and p.goals == []


def test_writer_temporaries_are_not_back_references():
    """Regression (det-4): back-references were keyed by ``id(arr)`` without keeping ``arr``
    alive, so a freed temporary's id could be reused by a later, distinct array and that array
    was written as a false TC_REFERENCE. Write many distinct temporaries and check that each
    one is a new array (TC_ARRAY 0x75) and reads back with its own contents."""
    w = P._Writer()
    offsets = []
    for i in range(200):
        offsets.append(len(w.out))
        w.int_array([i, i + 1, i + 2])  # a temporary, freed right after the call
    assert all(bytes(w.out[o:o + 1]) == b"\x75" for o in offsets)
    w2 = P._Writer()
    for i in range(50):
        w2.int2_array([[i], [i + 1]])
    allowed = {P._BASE_HANDLE + h for h in w2.classdescs.values()}
    data = bytes(w2.out)
    refs = [int.from_bytes(data[k + 1:k + 5], "big") for k in range(len(data) - 4)
            if data[k:k + 3] == b"\x71\x00\x7e"]
    assert refs and set(refs) <= allowed  # only class descriptors are ever back-referenced


def test_writers_reject_non_integral_coordinates(tmp_path):
    """Regression (C-FILE-IO-TRUNCATES-COORDS): both file formats store integer coordinates;
    write_txt / write_dat truncated (0.5, 1.25) to (0, 1) silently. fixed raises ValueError
    (integral floats are fine); original still truncates, as the Java code does."""
    g = Graph.from_adjacency([[1], [0]], xy=[(0.5, 1.25), (2, 3)])
    p = P.Problem(g, [0], [1])
    for write in (P.write_txt, P.write_dat):
        with pytest.raises(ValueError, match="non-integral"):
            write(p, str(tmp_path / "x"))
    assert not os.path.exists(str(tmp_path / "x"))
    assert P.problem_to_txt(p, compat="original").startswith("0:0:1 1:2:3")
    q = P.Problem(Graph.from_adjacency([[1], [0]], xy=[(1.0, 2), (2, 3)]), [0], [1])
    P.write_txt(q, str(tmp_path / "q.txt"))
    assert P.read_txt(str(tmp_path / "q.txt")).graph.xy == {0: (1, 2), 1: (2, 3)}
