"""Problems (graph + starts + goals): generators, ``.dat``/``.txt`` I/O, batch generation.

Port of ``projects.multipath.advanced.Problem``.

* :class:`Problem` holds ``graph``, ``starts`` and ``goals`` (robot ``i`` goes
  ``starts[i] -> goals[i]``); ``sg`` gives the original's ``int[2][n]`` view.
* Generators: :func:`create_grid_problem`, :func:`create_grid_problem_8connected`,
  :func:`create_n2_puzzle`, :func:`create_n2m1_puzzle`, :func:`get_a9_puzzle`,
  :func:`get_long_straight_with_one_garage_problem`.
* I/O: :func:`write_dat` / :func:`read_dat` (Java object serialization, byte-identical to
  the original's ``writeToFile(Problem, String)`` / ``readFromFile``), :func:`write_txt` /
  :func:`problem_to_txt` (the private text writer) and :func:`read_txt` (a parser for that
  text format; the original has none).
* :func:`generate_benchmark_files`: ``Problem.main``, i.e. the original's data-set
  generator, with the same files, names, contents and random stream.

Grid size convention (both modes): ``create_grid_problem(col, row, ...)`` builds a grid
with **``col`` rows and ``row`` columns** -- the original passes its parameter ``col`` as
the row count. The paper's "24x18" instances therefore have 24 rows and 18 columns. We
keep this in both modes because every experiment of the paper was generated this way;
pass ``as_named=True`` to get ``row`` rows and ``col`` columns instead.

Differences of ``compat="fixed"`` from the original:

* :func:`read_dat` raises on a missing or corrupt file (the original prints the stack
  trace and returns a ``Problem`` whose ``graph``/``sg`` are ``None``).
* :func:`write_dat` raises on I/O errors (the original prints and swallows them); a vertex
  with more than 8 neighbours is written with a longer row (the original's fixed
  13-column rows overflow); a vertex without coordinates raises ``ValueError`` before the
  file is opened (the original throws ``NullPointerException`` and leaves a truncated
  file).
* :func:`write_txt` raises ``ValueError`` for a vertex without coordinates before opening
  the file (the original leaves an empty file behind its ``NullPointerException``), and
  creates the parent directory with ``os.makedirs`` (the original throws
  ``NullPointerException`` when the name has no parent directory).
* :func:`generate_benchmark_files` joins paths with ``os.path.join`` (the original
  concatenates Windows backslashes, which on other systems become part of the file
  *name*), and writes the dead ``createCrowdedHardProblems8Connected`` family as ``.dat``
  files (the original writes serialized data under ``.txt`` names, without a separator
  after the folder).
* The generators inherit the fixes of :mod:`mppflow.graph` (narrow grids, too many holes
  or agents).
"""
from __future__ import annotations

import os
import struct
import sys
from typing import Iterable, Sequence

from ._java import MathRandom, resolve_rng
from .errors import (
    JavaArrayIndexOutOfBoundsException,
    JavaArrayStoreException,
    JavaClassCastException,
    JavaClassNotFoundException,
    JavaEOFException,
    JavaException,
    JavaInvalidClassException,
    JavaNegativeArraySizeException,
    JavaNullPointerException,
    JavaStreamCorruptedException,
)
from .graph import (
    Graph,
    check_compat,
    convert_graph,
    create_2d_grid_graph,
    create_generic_2d_grid_graph_with_holes,
    create_generic_2d_grid_graph_with_holes_8connected,
    get_random_start_goal,
    get_random_start_goal_many,
    java_d2i,
)

__all__ = [
    "Problem",
    "FILE_FOLDER",
    "create_grid_problem",
    "create_grid_problem_8connected",
    "create_n2_puzzle",
    "create_n2m1_puzzle",
    "get_a9_puzzle",
    "get_long_straight_with_one_garage_problem",
    "problem_to_txt",
    "write_txt",
    "read_txt",
    "problem_to_dat_bytes",
    "write_dat",
    "read_dat",
    "BENCHMARK_FAMILIES",
    "generate_benchmark_files",
    "create_n2_puzzles",
    "create_crowded_hard_problems",
    "create_32x32_problems",
    "create_32x32_problems_8connected",
    "create_24x18_performance_testing_problems",
    "create_crowded_hard_problems_8connected",
    "JavaClassCastException",
    "JavaStreamCorruptedException",
    "JavaArrayStoreException",
]

FILE_FOLDER = "D:\\temp\\data\\dmrpp\\"
"""The original's default output folder (``Problem.FILE_FOLDER``)."""


# Former local names (the classes now live in mppflow.errors).
_JavaEOFException = JavaEOFException
_JavaInvalidClassException = JavaInvalidClassException
_JavaClassNotFoundException = JavaClassNotFoundException


class Problem:
    """A multi-robot path planning instance (port of ``advanced.Problem``).

    ``starts``/``goals`` are lists of vertex ids. ``sg`` is the original's
    ``[starts, goals]`` view (the same list objects); it is ``None`` when both are
    ``None``, as after a failed ``readFromFile`` in ``original`` mode.
    ``vid_new_id_map``/``new_id_vid_map`` are set by the planners' sub-problem
    construction (original id <-> sub-graph id); they are never serialized.
    """

    def __init__(self, graph: Graph | None = None, starts: Sequence[int] | None = None,
                 goals: Sequence[int] | None = None):
        self.graph = graph
        self.starts = None if starts is None else list(starts)
        self.goals = None if goals is None else list(goals)
        self.vid_new_id_map: dict[int, int] | None = None
        self.new_id_vid_map: dict[int, int] | None = None

    @classmethod
    def from_sg(cls, graph: Graph | None, sg: Sequence[Sequence[int]] | None) -> "Problem":
        """``new Problem(graph, sg)``."""
        p = cls(graph)
        p.sg = sg
        return p

    @property
    def sg(self):
        if self.starts is None and self.goals is None:
            return None
        return [self.starts, self.goals]

    @sg.setter
    def sg(self, value) -> None:
        if value is None:
            self.starts = self.goals = None
        else:
            self.starts = None if value[0] is None else list(value[0])
            self.goals = None if value[1] is None else list(value[1])

    @property
    def n_agents(self) -> int:
        return 0 if self.starts is None else len(self.starts)

    def __repr__(self) -> str:
        return f"Problem(graph={self.graph!r}, starts={self.starts}, goals={self.goals})"


# ------------------------------------------------------------------------ generators
def create_grid_problem(col: int, row: int, frac_obs: float, n: int, rng: MathRandom | None = None,
                        compat: str = "fixed", *, as_named: bool = False) -> Problem:
    """``createGridProblem(col, row, fracObs, n)``: a 4-connected grid with obstacles and
    ``n`` random start/goal pairs, converted to ids ``0..|V|-1``.

    **The grid has ``col`` rows and ``row`` columns** (the original passes ``col`` as the
    row count; kept in both modes, see the module docstring). ``as_named=True`` builds
    ``row`` rows and ``col`` columns instead (different instances for the same seed).

    Random draws, in order: the hole draws (``y`` then ``x`` per attempt), then the
    start/goal draws (``x`` then ``y`` per vertex) of :func:`get_random_start_goal`.
    """
    rows, cols = (row, col) if as_named else (col, row)
    g = create_generic_2d_grid_graph_with_holes(rows, cols, frac_obs, rng, compat)
    starts, goals = get_random_start_goal(g, n, rng, compat)
    g = convert_graph(g, starts, goals, compat)
    return Problem(g, starts, goals)


def create_grid_problem_8connected(col: int, row: int, frac_obs: float, n: int,
                                   rng: MathRandom | None = None, compat: str = "fixed", *,
                                   as_named: bool = False) -> Problem:
    """``createGridProblem8Connected``: :func:`create_grid_problem` on an 8-connected grid
    (same row/column convention and draw order)."""
    rows, cols = (row, col) if as_named else (col, row)
    g = create_generic_2d_grid_graph_with_holes_8connected(rows, cols, frac_obs, rng, compat)
    starts, goals = get_random_start_goal(g, n, rng, compat)
    g = convert_graph(g, starts, goals, compat)
    return Problem(g, starts, goals)


def create_n2_puzzle(n: int, rng: MathRandom | None = None, compat: str = "fixed") -> Problem:
    """``createN2Puzzle(n)``: the ``n x n`` grid fully occupied, starts ``0..n^2-1``,
    goals a random permutation (``n^2`` draws). No solvability check (TOR Sec. VI-B: for
    ``n >= 3`` every instance is solvable)."""
    g = create_2d_grid_graph(n, n, True, compat)
    starts, goals = get_random_start_goal_many(g, n * n, rng, compat)
    return Problem(g, starts, goals)


def create_n2m1_puzzle(n: int, rng: MathRandom | None = None, compat: str = "fixed") -> Problem:
    """``createN2M1Puzzle(n)`` (*dead*): as :func:`create_n2_puzzle` with ``n^2 - 1``
    robots."""
    g = create_2d_grid_graph(n, n, True, compat)
    starts, goals = get_random_start_goal_many(g, n * n - 1, rng, compat)
    return Problem(g, starts, goals)


def get_a9_puzzle() -> Problem:
    """``getA9Puzzle()`` (*dead*): the original's hand-coded 9-puzzle on a hand-built 3x3
    grid (no coordinates), starts ``[3,0,2,8,1,4,7,5,6]``, goals ``[0..8]``.

    This is *not* the TOR Fig. 1 instance (whose starts are ``[2,4,5,1,8,6,7,3,0]`` and whose
    optimal makespan is 4): this instance has optimal makespan 5 and lower bound 3."""
    g = Graph()
    g.add_vertex(0, [0, 1, 3])
    g.add_vertex(1, [0, 1, 2, 4])
    g.add_vertex(2, [1, 2, 5])
    g.add_vertex(3, [0, 3, 4, 6])
    g.add_vertex(4, [1, 3, 4, 5, 7])
    g.add_vertex(5, [2, 4, 5, 8])
    g.add_vertex(6, [3, 6, 7])
    g.add_vertex(7, [4, 6, 7, 8])
    g.add_vertex(8, [5, 7, 8])
    g.finish_building_graph()
    return Problem(g, [3, 0, 2, 8, 1, 4, 7, 5, 6], [0, 1, 2, 3, 4, 5, 6, 7, 8])


def get_long_straight_with_one_garage_problem() -> Problem:
    """``getLongStraightWithOneGarageProblem()``: the path 0-1-...-8 with a garage vertex 9
    attached to 4 (no coordinates); two robots swap ends (``0 -> 8``, ``8 -> 0``)."""
    g = Graph()
    g.add_vertex(0, [0, 1])
    g.add_vertex(1, [0, 1, 2])
    g.add_vertex(2, [1, 2, 3])
    g.add_vertex(3, [2, 3, 4])
    g.add_vertex(4, [3, 4, 5, 9])
    g.add_vertex(5, [4, 5, 6])
    g.add_vertex(6, [5, 6, 7])
    g.add_vertex(7, [6, 7, 8])
    g.add_vertex(8, [7, 8])
    g.add_vertex(9, [4, 9])
    g.finish_building_graph()
    return Problem(g, [0, 8], [8, 0])


# ------------------------------------------------------------------------- text I/O
def _npe_point() -> JavaNullPointerException:
    return JavaNullPointerException('Cannot invoke "java.awt.geom.Point2D.getX()" because "this.point" is null')


def _check_integral_point(v, p) -> None:
    """``fixed``: both file formats store integer coordinates (the original truncates with
    ``(int)``); a non-integral one would come back changed, so it raises ``ValueError``."""
    for c in p[:2]:
        try:
            ok = float(c).is_integer()
        except (TypeError, ValueError, OverflowError):
            ok = False
        if not ok:
            raise ValueError(f"vertex {v} has the non-integral coordinate {c!r}; the file formats store "
                             f"integers (round the coordinates first)")


def _check_writable(p: Problem, orig: bool) -> None:
    """Raise the exception the original's writers hit first (NPE on a missing point or
    adjacency entry), in vertex order."""
    g = p.graph
    if g is None:
        if orig:
            raise JavaNullPointerException('Cannot read field "vertices" because "this.graph" is null')
        raise ValueError("problem has no graph")
    for v in g.vertices:
        if g.point(v) is None:
            if orig:
                raise _npe_point()
            raise ValueError(f"vertex {v} has no coordinates; the file format requires them")
        if not orig:
            _check_integral_point(v, g.point(v))
        if g.neighbors(v) is None:
            if orig:
                raise JavaNullPointerException(
                    'Cannot invoke "java.util.Set.toArray(Object[])" because the return value of "java.util.Map.get(Object)" is null')
            raise ValueError(f"vertex {v} has no adjacency entry")


def problem_to_txt(p: Problem, newline: str = "\n", compat: str = "fixed") -> str:
    """The text the original's private ``writeToFile(String)`` writes.

    Line 1: ``id:x:y`` for each vertex in ``vertices`` order. Line 2: ``id:nbr`` for each
    vertex and each neighbour in iteration order (self-loop included). Line 3:
    ``start:goal`` per robot, **without a trailing newline**. Items are separated by
    single spaces; ``newline`` is the platform separator ``println`` uses (``"\\n"`` on
    Linux, ``"\\r\\n"`` on the Windows machine the original data came from).
    """
    orig = check_compat(compat)
    _check_writable(p, orig)
    g = p.graph
    verts = " ".join(f"{v}:{g.ix(v)}:{g.iy(v)}" for v in g.vertices)
    edges = " ".join(f"{v}:{u}" for v in g.vertices for u in g.neighbors(v))
    if p.sg is None:
        if orig:
            raise JavaNullPointerException('Cannot load from object array because "this.sg" is null')
        raise ValueError("problem has no starts/goals")
    starts, goals = p.starts, p.goals
    sgs = " ".join(f"{starts[i]}:{goals[i]}" for i in range(len(starts)))
    return verts + newline + edges + newline + sgs


def write_txt(p: Problem, file_name: str, compat: str = "fixed", newline: str = "\n") -> None:
    """The original's private ``Problem.writeToFile(String fileName)``: write
    :func:`problem_to_txt` to ``file_name``, creating the parent directories.

    ``original`` mode: a name without a parent directory raises
    :class:`JavaNullPointerException` (``getParentFile()`` is ``null``); a vertex without
    coordinates leaves an empty file and raises :class:`JavaNullPointerException`; an
    unopenable file is silently skipped.

    Differs from the original: ``fixed`` uses ``os.makedirs`` on the parent (if any),
    raises ``ValueError`` before opening the file for a vertex without coordinates or with
    a non-integral one (the format stores integers; the original truncates), and lets I/O
    errors propagate.
    """
    _write_txt(p, file_name, compat, newline)


def _write_txt(p: Problem, file_name: str, compat: str = "fixed", newline: str = "\n") -> bool:
    """:func:`write_txt`; returns ``False`` when ``original`` mode silently skipped an
    unopenable file, else ``True``."""
    orig = check_compat(compat)
    parent = os.path.dirname(file_name)
    if orig:
        if not parent:
            raise JavaNullPointerException(
                'Cannot invoke "java.io.File.mkdirs()" because the return value of "java.io.File.getParentFile()" is null')
        try:
            os.makedirs(parent, exist_ok=True)
        except OSError:
            pass
        try:
            f = open(file_name, "w", encoding="utf-8", newline="")
        except OSError:
            return False  # FileNotFoundException is swallowed
        try:
            text = problem_to_txt(p, newline, compat)
        except JavaException:
            f.close()  # the original never flushes: the file stays empty
            raise
        with f:
            f.write(text)
        return True
    text = problem_to_txt(p, newline, compat)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(file_name, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    return True


def read_txt(file_name: str) -> Problem:
    """Parse the text format of :func:`write_txt` (helper added in the port; the original
    has no text reader).

    ``vertices`` keeps the file order, the points are the integer ``x:y`` values, and each
    adjacency is the ascending set of the listed neighbours. ``rows``/``cols`` are 0.
    """
    with open(file_name, "r", encoding="utf-8", newline="") as f:
        text = f.read()
    lines = text.replace("\r\n", "\n").split("\n")
    while len(lines) < 3:
        lines.append("")
    g = Graph()
    for tok in lines[0].split():
        vid, x, y = (int(t) for t in tok.split(":"))
        g.vertices.append(vid)
        g.xy[vid] = (x, y)
    nbrs: dict[int, set[int]] = {}
    for tok in lines[1].split():
        a, b = (int(t) for t in tok.split(":"))
        nbrs.setdefault(a, set()).add(b)
    for v, s in nbrs.items():
        g.adj[v] = sorted(s)
    starts, goals = [], []
    for tok in lines[2].split():
        s, t = (int(t) for t in tok.split(":"))
        starts.append(s)
        goals.append(t)
    return Problem(g, starts, goals)


# -------------------------------------------------------------- Java serialization
_GRAPH_CLASS = "projects.multipath.advanced.Graph"
_GRAPH_SUID = 1
_INT2_SUID = 0x17F7E44F198F893C  # [[I
_INT1_SUID = 0x4DBA602676EAB2A5  # [I
_BASE_HANDLE = 0x7E0000


def _utf(s: str) -> bytes:
    b = s.encode("utf-8")
    return struct.pack(">H", len(b)) + b


class _Writer:
    """Just enough of ``java.io.ObjectOutputStream`` for a Problem's Graph and sg."""

    def __init__(self) -> None:
        self.out = bytearray(b"\xac\xed\x00\x05")
        self.next_handle = 0
        self.classdescs: dict[str, int] = {}
        # handle of each written array by ``id()``; ``keep`` holds the arrays so that no id
        # is reused by a new object while the stream is being written
        self.objs: dict[int, int] = {}
        self.keep: list = []

    def _assign(self) -> int:
        h = self.next_handle
        self.next_handle += 1
        return h

    def _ref(self, h: int) -> None:
        self.out += b"\x71" + struct.pack(">i", _BASE_HANDLE + h)

    def array_classdesc(self, name: str, suid: int) -> None:
        if name in self.classdescs:
            self._ref(self.classdescs[name])
            return
        self.out += b"\x72" + _utf(name) + struct.pack(">q", suid)
        self.classdescs[name] = self._assign()
        self.out += b"\x02" + struct.pack(">H", 0) + b"\x78\x70"

    def int_array(self, arr) -> None:
        if arr is None:
            self.out += b"\x70"
            return
        if id(arr) in self.objs:
            self._ref(self.objs[id(arr)])
            return
        self.out += b"\x75"
        self.array_classdesc("[I", _INT1_SUID)
        self.objs[id(arr)] = self._assign()
        self.keep.append(arr)
        self.out += struct.pack(">i", len(arr))
        self.out += struct.pack(f">{len(arr)}i", *arr) if arr else b""

    def int2_array(self, arr) -> None:
        if arr is None:
            self.out += b"\x70"
            return
        if id(arr) in self.objs:
            self._ref(self.objs[id(arr)])
            return
        self.out += b"\x75"
        self.array_classdesc("[[I", _INT2_SUID)
        self.objs[id(arr)] = self._assign()
        self.keep.append(arr)
        self.out += struct.pack(">i", len(arr))
        for row in arr:
            self.int_array(row)

    def graph_header(self) -> None:
        """TC_OBJECT + the Graph class descriptor (everything the original flushes before
        calling ``Graph.writeObject``)."""
        self.out += b"\x73\x72" + _utf(_GRAPH_CLASS) + struct.pack(">q", _GRAPH_SUID)
        self.classdescs[_GRAPH_CLASS] = self._assign()
        self.out += b"\x03" + struct.pack(">H", 6)
        self.out += b"I" + _utf("columns")
        self.out += b"I" + _utf("rows")
        for code, fname, ftype in (("L", "adjacencySetMap", "Ljava/util/Map;"),
                                   ("L", "idVertexMap", "Ljava/util/SortedMap;"),
                                   ("L", "verticeSet", "Ljava/util/Set;"),
                                   ("[", "vertices", "[Lprojects/multipath/advanced/Vertex;")):
            self.out += code.encode() + _utf(fname) + b"\x74" + _utf(ftype)
            self._assign()  # the type string
        self.out += b"\x78\x70"
        self._assign()  # the Graph object


def _graph_rows(g: Graph, orig: bool) -> list[list[int]]:
    rows = []
    for v in g.vertices:
        p = g.point(v)
        if p is None:
            if orig:
                raise _npe_point()
            raise ValueError(f"vertex {v} has no coordinates; the .dat format requires them")
        if not orig:
            _check_integral_point(v, p)
        nb = g.neighbors(v)
        if nb is None:
            if orig:
                raise JavaNullPointerException(
                    'Cannot invoke "java.util.Set.toArray(Object[])" because the return value of "java.util.Map.get(Object)" is null')
            raise ValueError(f"vertex {v} has no adjacency entry")
        width = 13 if orig else max(13, 4 + len(nb))
        row = [0] * width
        row[0] = v
        row[1] = java_d2i(p[0])
        row[2] = java_d2i(p[1])
        row[3] = len(nb)
        for j, u in enumerate(nb):
            if 4 + j >= width:
                raise JavaArrayIndexOutOfBoundsException(f"Index {4 + j} out of bounds for length {width}")
            row[4 + j] = u
        rows.append(row)
    return rows


def problem_to_dat_bytes(p: Problem, compat: str = "fixed") -> bytes:
    """The bytes of the original's ``writeToFile(Problem, String)``: Java serialization
    (protocol 2) of ``p.graph`` (custom ``writeObject``: an ``int[n][13]`` of
    ``[id, x, y, deg, adj..., 0...]`` rows, then ``rows`` and ``columns``) followed by
    ``p.sg``.

    ``original`` mode raises like the Java (see :func:`write_dat`). In ``fixed`` mode a
    vertex with more than 9 adjacency entries (self included) gets a longer row, which
    the original's reader accepts.
    """
    orig = check_compat(compat)
    w = _Writer()
    if p.graph is None:
        w.out += b"\x70"
    else:
        w.graph_header()
        rows = _graph_rows(p.graph, orig)
        w.int2_array(rows)
        w.out += b"\x77\x08" + struct.pack(">ii", p.graph.rows, p.graph.cols)
        w.out += b"\x78"
    sg = p.sg
    if sg is None:
        w.out += b"\x70"
    else:
        # Keep list identity (a shared starts/goals list is written as a back-reference).
        w.out += b"\x75"
        w.array_classdesc("[[I", _INT2_SUID)
        w._assign()
        w.out += struct.pack(">i", 2)
        w.int_array(p.starts)
        w.int_array(p.goals)
    return bytes(w.out)


def _partial_dat_bytes() -> bytes:
    w = _Writer()
    w.graph_header()
    return bytes(w.out)


def write_dat(p: Problem, file_name: str, compat: str = "fixed") -> None:
    """``Problem.writeToFile(Problem p, String fileName)``: write
    :func:`problem_to_dat_bytes` to ``file_name`` (byte-identical to the original).

    ``original`` mode: an unopenable file (e.g. a missing directory) prints the Java
    stack-trace line to stderr and returns; a vertex without coordinates
    (:class:`JavaNullPointerException`) or with more than 9 adjacency entries
    (:class:`JavaArrayIndexOutOfBoundsException`) leaves the 227-byte prefix (stream
    header and the Graph class descriptor) in the file and raises.

    Differs from the original: ``fixed`` raises on I/O errors, raises ``ValueError``
    before opening the file for a vertex without coordinates or with a non-integral one
    (the rows store integers; the original truncates), and writes longer rows for
    high-degree vertices.
    """
    _write_dat(p, file_name, compat)


def _write_dat(p: Problem, file_name: str, compat: str) -> bool:
    """:func:`write_dat`; returns ``False`` when ``original`` mode printed and swallowed the
    ``FileNotFoundException`` (nothing written), else ``True``."""
    orig = check_compat(compat)
    if not orig:
        data = problem_to_dat_bytes(p, compat)
        with open(file_name, "wb") as f:
            f.write(data)
        return True
    try:
        f = open(file_name, "wb")
    except OSError as e:
        print(f"java.io.FileNotFoundException: {file_name} ({e.strerror})", file=sys.stderr)
        return False
    with f:
        try:
            data = problem_to_dat_bytes(p, compat)
        except JavaException:
            if p.graph is not None:
                f.write(_partial_dat_bytes())
            raise
        f.write(data)
    return True


class _Reader:
    """Just enough of ``java.io.ObjectInputStream`` to read the original's ``.dat``.

    ``orig`` selects how ``Graph.readObject``'s per-row work fails (see
    :func:`_graph_from_rows`); that work runs wherever a Graph object occurs in the
    stream, as in Java."""

    def __init__(self, b: bytes, orig: bool = False):
        self.b, self.p, self.handles = b, 0, []
        self.orig = orig

    def _need(self, n: int) -> None:
        if self.p + n > len(self.b):
            raise _JavaEOFException("unexpected end of stream")

    def u1(self) -> int:
        self._need(1)
        v = self.b[self.p]
        self.p += 1
        return v

    def u2(self) -> int:
        self._need(2)
        (v,) = struct.unpack_from(">H", self.b, self.p)
        self.p += 2
        return v

    def i4(self) -> int:
        self._need(4)
        (v,) = struct.unpack_from(">i", self.b, self.p)
        self.p += 4
        return v

    def i8(self) -> int:
        self._need(8)
        (v,) = struct.unpack_from(">q", self.b, self.p)
        self.p += 8
        return v

    def utf(self) -> str:
        n = self.u2()
        self._need(n)
        s = self.b[self.p:self.p + n].decode("utf-8")
        self.p += n
        return s

    def handle(self, h: int):
        idx = h - _BASE_HANDLE
        if not 0 <= idx < len(self.handles):
            raise JavaStreamCorruptedException(f"invalid handle value: {h:08X}")
        return self.handles[idx]

    def classdesc(self):
        tc = self.u1()
        if tc == 0x70:
            return None
        if tc == 0x71:
            d = self.handle(self.i4())
            if _unresolved(d) is not None:
                return None  # handles.lookupObject is null for a failed handle
            if not (isinstance(d, dict) and "fields" in d):
                raise JavaClassCastException(
                    f"class {_java_type_name(d)} cannot be cast to class java.io.ObjectStreamClass")
            return d
        if tc != 0x72:
            raise JavaStreamCorruptedException(f"invalid type code: {tc:02X}")
        name = self.utf()
        d = {"name": name, "uid": self.i8()}
        self.handles.append(d)
        d["flags"] = self.u1()
        fields = []
        for _ in range(self.u2()):
            t = chr(self.u1())
            fname = self.utf()
            if t in "L[":
                self.content()
            fields.append((t, fname))
        d["fields"] = fields
        while True:  # class annotations
            tc = self.u1()
            if tc == 0x78:
                break
            self.p -= 1
            self.content()
        d["super"] = self.classdesc()
        return d

    def content(self):
        tc = self.u1()
        if tc == 0x70:
            return None
        if tc == 0x71:
            return self.handle(self.i4())
        if tc == 0x74 or tc == 0x7C:  # TC_STRING, TC_LONGSTRING
            if tc == 0x74:
                n = self.u2()
            else:
                n = max(self.i8(), 0)  # readLongUTF: a negative length reads ""
            self._need(n)
            s = self.b[self.p:self.p + n].decode("utf-8")
            self.p += n
            self.handles.append(s)
            return s
        if tc == 0x75:
            return self._array()
        if tc == 0x73:
            return self._object()
        raise JavaStreamCorruptedException(f"invalid type code: {tc:02X}")

    def _array(self):
        cd = self.classdesc()
        n = self.i4()
        if cd is None:
            raise JavaNullPointerException(
                'Cannot invoke "java.io.ObjectStreamClass.forClass()" because "desc" is null')
        name = cd["name"]
        arr = _JArr(name)
        arr.unresolved = _missing_class(name)
        if arr.unresolved is None and n < 0:
            raise JavaNegativeArraySizeException(str(n))
        self.handles.append(arr)
        elem = name[1:2]
        if arr.unresolved is not None or elem == "L" or elem == "[":
            # An unresolved array class reads its elements as objects, as Java does.
            for _ in range(max(n, 0)):
                x = self.content()
                if x is not None:
                    missing = _unresolved(x)  # such an element is read as null
                    if (missing is None and name == "[[I"
                            and not (isinstance(x, _JArr) and x.cls == "[I")):
                        raise JavaArrayStoreException(_java_type_name(x))
                    if arr.unresolved is None:
                        arr.unresolved = missing
                arr.append(x)
        else:  # a primitive array (name is "[" + one primitive code)
            size = _PRIM_SIZES[elem]
            self._need(size * n)
            if elem == "I":
                arr.extend(struct.unpack_from(f">{n}i", self.b, self.p))
            else:
                arr.extend([0] * n)  # contents never used: only int arrays are read
            self.p += size * n
        return arr

    def _skip_custom_data(self, left: int = 0) -> None:
        """``ObjectInputStream.skipCustomData``: the ``left`` bytes remaining in the
        current block, then block data and objects up to ``TC_ENDBLOCKDATA``."""
        self._need(left)
        self.p += left
        while True:
            tc = self.u1()
            if tc == 0x78:
                return
            if tc == 0x77:
                self._need(n := self.u1())
                self.p += n
            elif tc == 0x7A:
                n = self.i4()
                if n < 0:
                    raise JavaStreamCorruptedException(f"illegal block data header length: {n}")
                self._need(n)
                self.p += n
            else:
                self.p -= 1
                self.content()

    def _object(self):
        cd = self.classdesc()
        if cd is None:
            raise JavaNullPointerException(
                'Cannot invoke "java.io.ObjectStreamClass.checkDeserialize()" because "desc" is null')
        name = cd["name"]
        if name in _ORIGINAL_CLASSES:
            if name not in _ORIGINAL_SERIALIZABLE:
                raise _JavaInvalidClassException(f"{name}; class invalid for deserialization")
            if cd["uid"] != 1:  # both serializable classes declare serialVersionUID = 1L
                raise _JavaInvalidClassException(
                    f"{name}; local class incompatible: stream classdesc serialVersionUID = "
                    f"{cd['uid']}, local class serialVersionUID = 1")
            if name == _GRAPH_CLASS:
                return self._graph()
            raise JavaClassCastException(f"class {name} cannot be cast to class {_GRAPH_CLASS}")
        if name.startswith("[") and _missing_class(name) is None:
            # An array class named in an ordinary object: not instantiable.
            raise _JavaInvalidClassException(f"{name}; no valid constructor")
        if _resolvable(name):
            # Approximation: assumes a serializable class whose data deserializes.
            raise JavaClassCastException(f"class {name} cannot be cast to class {_GRAPH_CLASS}")
        # An unknown class: the original's stream reads (and discards) the object's data
        # with the stream's own descriptors, then readObject throws
        # ClassNotFoundException.
        obj = _Unresolved(name)
        self.handles.append(obj)
        chain = []
        d = cd
        while d is not None:
            chain.append(d)
            d = d["super"]
        if cd["flags"] & 0x04:  # SC_EXTERNALIZABLE
            if cd["flags"] & 0x08:  # SC_BLOCK_DATA
                self._skip_custom_data()
            return obj
        for d in reversed(chain):  # superclass data first
            prim = sum(_PRIM_SIZES.get(t, 0) for t, _ in d["fields"])
            self._need(prim)
            self.p += prim
            for t, _ in d["fields"]:
                if t in "L[":
                    self.content()
            if d["flags"] & 0x01:  # SC_WRITE_METHOD
                self._skip_custom_data()
        return obj

    def _graph(self):
        obj = {"kind": "graph"}
        self.handles.append(obj)
        v = self.content()  # Graph.readObject: (int[][]) in.readObject()
        missing = _unresolved(v)
        if missing is not None:
            raise _JavaClassNotFoundException(missing)
        if v is not None and not (isinstance(v, _JArr) and v.cls == "[[I"):
            raise JavaClassCastException(
                f"class {_java_type_name(v)} cannot be cast to class [[I")
        # Graph.readObject builds the vertices from v right away: before the two
        # readInt calls, and for every Graph in the stream (not only the first object),
        # so a bad row fails before a truncated int block or an outer cast does.
        g = _graph_from_rows(v, self.orig)
        # in.readInt() twice, in block-data mode (the ints may span blocks), then
        # skipCustomData() up to TC_ENDBLOCKDATA.
        raw = self._block_bytes(8)
        g.rows, g.cols = struct.unpack(">ii", raw)
        self._skip_custom_data(self._block_left)
        obj["graph"] = g
        return obj

    def _block_bytes(self, k: int) -> bytes:
        out = b""
        self._block_left = 0
        while len(out) < k:
            if self._block_left == 0:
                if self.p >= len(self.b):
                    raise _JavaEOFException("unexpected end of stream")
                tc = self.b[self.p]
                if tc == 0x77:
                    self.p += 1
                    self._block_left = self.u1()
                elif tc == 0x7A:
                    self.p += 1
                    self._block_left = self.i4()
                    if self._block_left < 0:
                        raise JavaStreamCorruptedException(
                            f"illegal block data header length: {self._block_left}")
                else:  # end of the custom data: readInt hits EOF
                    raise _JavaEOFException("end of block data")
                continue
            take = min(k - len(out), self._block_left)
            self._need(take)
            out += self.b[self.p:self.p + take]
            self.p += take
            self._block_left -= take
        return out


_PRIM_SIZES = {"B": 1, "C": 2, "D": 8, "F": 4, "I": 4, "J": 8, "S": 2, "Z": 1}

# The original's classes (packages projects.multipath.*) and the serializable ones among them.
_ORIGINAL_CLASSES = frozenset(
    ["projects.multipath.advanced." + c for c in ("Graph", "Path", "Problem", "Vertex")]
    + ["projects.multipath.ILP." + c for c in (
        "HeuristicPathPlanner", "Main", "MultiagentGraphSolverGurobiDistance",
        "MultiagentGraphSolverGurobiTime", "MultiagentGraphSolverGurobiTimePuzzle",
        "MultiagentGraphSolverGurobiTimeR", "MultiagentGraphSolverGurobiTotalTime",
        "PathFinder", "PathPlanner", "PathPlanner$1", "PuzzleSolver", "PuzzleSolver$1")])
_ORIGINAL_SERIALIZABLE = frozenset([_GRAPH_CLASS, "projects.multipath.advanced.Problem"])
_PLATFORM_PREFIXES = ("java.", "javax.", "jdk.", "sun.", "com.sun.", "gurobi.")


def _missing_class(name: str) -> str | None:
    """For an array class name: the class ``Class.forName`` reports missing, or None."""
    base = name.lstrip("[")
    if len(base) == 1 and base in _PRIM_SIZES and base != name:
        return None
    if base.startswith("L") and base.endswith(";") and base != name:
        return None if _resolvable(base[1:-1]) else base[1:-1]
    return name


def _resolvable(name: str) -> bool:
    """Whether the original's class path has the (non-array) class ``name``: its own
    classes, and (approximately) every JDK or Gurobi class name."""
    return name in _ORIGINAL_CLASSES or name.startswith(_PLATFORM_PREFIXES)


class _JArr(list):
    """A deserialized array: its elements plus its class name (``[[I``, ``[I``, ...)."""

    __slots__ = ("cls", "unresolved")

    def __init__(self, cls: str):
        super().__init__()
        self.cls = cls
        self.unresolved: str | None = None


class _Unresolved:
    """An object of a class the original's class path lacks (``ClassNotFoundException``)."""

    __slots__ = ("name",)

    def __init__(self, name: str):
        self.name = name


def _unresolved(x) -> str | None:
    """The missing class an object depends on (``ObjectInputStream``'s handle
    dependencies), or None."""
    if isinstance(x, _Unresolved):
        return x.name
    if isinstance(x, _JArr):
        return x.unresolved
    return None


def _java_type_name(x) -> str:
    if isinstance(x, _JArr):
        return x.cls
    if isinstance(x, str):
        return "java.lang.String"
    if isinstance(x, dict):
        return _GRAPH_CLASS
    return "java.lang.Object"


def _graph_from_rows(v, orig: bool) -> Graph:
    """The per-row loop of ``Graph.readObject`` (``rows``/``cols`` are set by the caller).

    Row ``i`` is read as Java does: ``v[i][1]``, ``v[i][2]``, ``v[i][0]``, ``v[i][3]``,
    then ``v[i][4 + j]`` for ``0 <= j < v[i][3]`` (none when ``v[i][3] <= 0``). A null
    ``v`` or null row raises :class:`~mppflow.errors.JavaNullPointerException` and a
    row too short for these reads raises
    :class:`~mppflow.errors.JavaArrayIndexOutOfBoundsException` (first missing index)
    in ``original`` mode, ``ValueError`` in ``fixed`` mode.
    """
    if v is None:
        if orig:
            raise JavaNullPointerException('Cannot read the array length because "<local2>" is null')
        raise ValueError("corrupt .dat: no vertex array")
    g = Graph()
    for row in v:
        if row is None:
            if orig:
                raise JavaNullPointerException(
                    'Cannot load from int array because "<local2>[<local3>]" is null')
            raise ValueError("corrupt .dat: null vertex row")
        n = len(row)
        deg = max(row[3], 0) if n >= 4 else 0
        if n < 4 or n < 4 + deg:
            if orig:  # the first index read out of range: 1, 2, 0, 3, then 4, 5, ...
                bad = 1 if n < 2 else 2 if n < 3 else 3 if n < 4 else n
                raise JavaArrayIndexOutOfBoundsException(f"Index {bad} out of bounds for length {n}")
            raise ValueError("corrupt .dat: short vertex row")
        vid = row[0]
        g.vertices.append(vid)
        g.xy[vid] = (row[1], row[2])
        g.adj[vid] = sorted(set(row[4:4 + deg]))
    return g


def read_dat(file_name: str, compat: str = "fixed") -> Problem:
    """``Problem.readFromFile(fileName)``: read a ``.dat`` written by the original (or by
    :func:`write_dat`).

    ``vertices`` keeps the file order, points are the stored integers, each adjacency is
    the ascending set of the stored entries, and ``rows``/``cols`` are the stored values.

    ``original`` mode follows ``ObjectInputStream`` and the original's ``catch``
    clauses. A missing, truncated or corrupt file, a Graph with another
    ``serialVersionUID`` or a non-serializable class of the original
    (``InvalidClassException``), and an object or array of a class that does not exist
    (``ClassNotFoundException``, raised after the object's data has been read with the
    stream's descriptors) print a Java-style line to stderr and return a
    :class:`Problem` with ``graph`` and/or ``sg`` ``None`` (``graph`` stays set when only
    ``sg`` fails). Runtime exceptions, which the original does not catch, are raised: an
    existing class of the wrong type (:class:`JavaClassCastException`), a non-``int[]``
    element of an ``int[][]`` (:class:`JavaArrayStoreException`), a negative array
    length (:class:`~mppflow.errors.JavaNegativeArraySizeException`), and the failures of
    ``Graph.readObject``'s row loop: a null vertex array or row
    (:class:`~mppflow.errors.JavaNullPointerException`) or a row too short for its
    stored degree (:class:`~mppflow.errors.JavaArrayIndexOutOfBoundsException`). That
    loop runs as soon as a Graph's ``int[][]`` is read, before its two ints, and for a
    Graph anywhere in the stream (also in the ``sg`` slot), so it takes precedence over
    a later EOF or cast failure. A stored degree ``<= 0`` reads no adjacency entries
    (both modes). Approximation: any ``java.*``, ``javax.*``, ``jdk.*``, ``sun.*``,
    ``com.sun.*`` or ``gurobi.*`` class name is taken to exist and to deserialize (so it
    gives ``ClassCastException``).

    Differs from the original: ``fixed`` raises (``FileNotFoundError``, ``ValueError``).
    """
    orig = check_compat(compat)
    p = Problem()
    try:
        with open(file_name, "rb") as f:
            data = f.read()
    except OSError as e:
        if orig:
            print(f"java.io.FileNotFoundException: {file_name} ({e.strerror})", file=sys.stderr)
            return p
        raise
    r = _Reader(data, orig)
    try:
        if len(data) < 4:
            raise _JavaEOFException("short stream header")
        if r.u2() != 0xACED or r.u2() != 5:
            raise JavaStreamCorruptedException(f"invalid stream header: {data[:4].hex().upper()}")
        obj = r.content()
        missing = _unresolved(obj)
        if missing is not None:
            raise _JavaClassNotFoundException(missing)
        if obj is not None:
            if not isinstance(obj, dict) or obj.get("kind") != "graph":
                raise JavaClassCastException(
                    f"class {_java_type_name(obj)} cannot be cast to class {_GRAPH_CLASS}")
            p.graph = obj["graph"]
        sg = r.content()
        missing = _unresolved(sg)
        if missing is not None:
            raise _JavaClassNotFoundException(missing)
        if sg is not None and not (isinstance(sg, _JArr) and sg.cls == "[[I"):
            raise JavaClassCastException(f"class {_java_type_name(sg)} cannot be cast to class [[I")
        if sg is None:
            p.sg = None
        else:
            p.starts = None if len(sg) < 1 or sg[0] is None else list(sg[0])
            p.goals = None if len(sg) < 2 or sg[1] is None else list(sg[1])
    except (JavaStreamCorruptedException, _JavaEOFException, _JavaInvalidClassException,
            _JavaClassNotFoundException, UnicodeDecodeError) as e:
        if orig:  # IOException / ClassNotFoundException: printed and swallowed
            cls = getattr(e, "java_class", "java.io.UTFDataFormatException")
            print(f"{cls}: {e}", file=sys.stderr)
            return p
        raise ValueError(f"{file_name}: not a valid Problem .dat file ({_cls_name(e)}: {e})") from None
    except (JavaClassCastException, JavaArrayStoreException, JavaNegativeArraySizeException,
            JavaNullPointerException, JavaArrayIndexOutOfBoundsException):
        if orig:
            raise  # a RuntimeException: not caught by the original
        raise ValueError(f"{file_name}: not a Problem .dat file") from None
    return p


def _cls_name(e: BaseException) -> str:
    return getattr(e, "java_class", type(e).__name__).rsplit(".", 1)[-1]


# ----------------------------------------------------------------- batch generation
BENCHMARK_FAMILIES = ("n2puzzle", "crowded", "32x32", "32x32-8c", "24x18")
"""The families ``Problem.main`` generates, in its order. ``"crowded-8c"``
(``createCrowdedHardProblems8Connected``, *dead*) can be requested explicitly."""


class _Out:
    def __init__(self, folder: str, compat: str, write_text: bool, write_binary: bool):
        self.folder = folder
        self.compat = compat
        self.orig = check_compat(compat)
        self.write_text = write_text
        self.write_binary = write_binary
        self.files: list[str] = []
        self.failed: list[str] = []

    def name(self, sub: str, base: str) -> str:
        if self.orig:
            return self.folder + "\\" + sub + "\\" + base
        return os.path.join(self.folder, sub, base)

    def dat(self, p: Problem, fn: str) -> None:
        """Write ``fn`` and list it in ``files``, or in ``failed`` if ``original`` mode could
        not open it (the original prints the exception and goes on)."""
        (self.files if _write_dat(p, fn, self.compat) else self.failed).append(fn)

    def emit(self, p: Problem, sub: str, base: str) -> None:
        if self.write_text:
            fn = self.name(sub, base + ".txt")
            (self.files if _write_txt(p, fn, self.compat) else self.failed).append(fn)
        if self.write_binary:
            fn = self.name(sub, base + ".dat")
            parent = os.path.dirname(fn)
            if not self.orig:
                os.makedirs(parent, exist_ok=True)
            elif not self.write_text and parent:
                # the original creates the directory in the text writer (Problem.java:46,
                # getParentFile().mkdirs(), whose failure is silent); write_text=False (new)
                # skips that writer
                try:
                    os.makedirs(parent, exist_ok=True)
                except OSError:
                    pass
            self.dat(p, fn)


def create_n2_puzzles(out: _Out, rng: MathRandom) -> None:
    """``createN2Puzzles``: ``i = 0..99``, ``n = 3, 4, 5``: ``n2puzzle/{n*n}-puzzle-{1001+i}``."""
    for i in range(100):
        for n in range(3, 6):
            p = create_n2_puzzle(n, rng, out.compat)
            out.emit(p, "n2puzzle", f"{n * n}-puzzle-{1001 + i}")


def create_crowded_hard_problems(out: _Out, rng: MathRandom) -> None:
    """``createCrowdedHardProblems``: 8x8 (10..60 robots) then 16x16 (10..250), no
    obstacles, 10 instances each (instance index outermost)."""
    for i in range(10):
        for a in range(10, 61, 10):
            p = create_grid_problem(8, 8, 0, a, rng, out.compat)
            out.emit(p, "8x8-grid", f"{a}-agts-{1001 + i}")
    for i in range(10):
        for a in range(10, 251, 10):
            p = create_grid_problem(16, 16, 0, a, rng, out.compat)
            out.emit(p, "16x16-grid", f"{a}-agts-{1001 + i}")


def create_32x32_problems(out: _Out, rng: MathRandom) -> None:
    """``create32x32Problems``: 32x32, 20% obstacles, 10..200 robots."""
    for i in range(10):
        for n in range(10, 201, 10):
            p = create_grid_problem(32, 32, 0.2, n, rng, out.compat)
            out.emit(p, "32x32-grid", f"20-pct-obs-{n}-agts-{1001 + i}")


def create_32x32_problems_8connected(out: _Out, rng: MathRandom) -> None:
    """``create32x32Problems8Connected``: 8-connected 32x32, 20% obstacles, 10..400 robots."""
    for i in range(10):
        for n in range(10, 401, 10):
            p = create_grid_problem_8connected(32, 32, 0.2, n, rng, out.compat)
            out.emit(p, "32x32-grid-8c", f"20-pct-obs-{n}-agts-{1001 + i}")


def create_24x18_performance_testing_problems(out: _Out, rng: MathRandom) -> None:
    """``create24x18PerformanceTestingProblems``: obstacles 0..30% (step 5), 10..300
    robots, 10 instances (instance index innermost). 24 rows x 18 columns."""
    for obs in range(0, 35, 5):
        for n in range(10, 301, 10):
            for i in range(10):
                p = create_grid_problem(24, 18, obs / 100.0, n, rng, out.compat)
                out.emit(p, "24x18-grid", f"{n}-agt-{obs}-pct-obs-{1001 + i}")


def create_crowded_hard_problems_8connected(out: _Out, rng: MathRandom) -> None:
    """``createCrowdedHardProblems8Connected`` (*dead*): 8-connected 8x8 and 16x16.

    ``original`` mode writes serialized data to ``folder + "8x8-grid-8c\\\\" + name +
    ".txt"`` (no separator after the folder, no text file, no directory creation), as
    the Java does. ``fixed`` writes ``folder/8x8-grid-8c/name.dat``.
    """
    for sub, size, top in (("8x8-grid-8c", 8, 60), ("16x16-grid-8c", 16, 250)):
        for i in range(10):
            for a in range(10, top + 1, 10):
                p = create_grid_problem_8connected(size, size, 0, a, rng, out.compat)
                base = f"{a}-agts-{1001 + i}"
                if out.orig:
                    fn = out.folder + sub + "\\" + base + ".txt"
                    if out.write_binary:
                        out.dat(p, fn)
                else:
                    if out.write_binary:
                        fn = os.path.join(out.folder, sub, base + ".dat")
                        os.makedirs(os.path.dirname(fn), exist_ok=True)
                        out.dat(p, fn)


_FAMILY_FUNCS = {
    "n2puzzle": create_n2_puzzles,
    "crowded": create_crowded_hard_problems,
    "32x32": create_32x32_problems,
    "32x32-8c": create_32x32_problems_8connected,
    "24x18": create_24x18_performance_testing_problems,
    "crowded-8c": create_crowded_hard_problems_8connected,
}


def generate_benchmark_files(folder: str | None = None, rng: MathRandom | None = None,
                             families: Iterable[str] = BENCHMARK_FAMILIES, compat: str = "fixed",
                             *, write_text: bool = True, write_binary: bool = True,
                             failed: list[str] | None = None) -> list[str]:
    """``Problem.main(argv)``: generate the original's benchmark data set.

    Runs the requested ``families`` in the given order (default: all five, in
    ``Problem.main``'s order) on one random stream ``rng``, writing for each instance the
    text file (:func:`write_txt`) and then the ``.dat`` file (:func:`write_dat`), with the
    original's file names. Returns the written paths in order. With the full default
    family list and ``MathRandom(s)``, the files equal those of the original run with
    ``Math.random`` seeded ``s``; a subset of families consumes the stream from the
    first selected family on (each family's draws depend on everything generated
    before it).

    ``original`` mode: the name is ``folder + "\\\\<sub>\\\\" + name`` exactly as the Java
    builds it (``folder`` defaults to :data:`FILE_FOLDER`); on systems where ``\\\\`` is
    not a separator the backslashes become part of the file name, and a ``folder``
    without a ``/`` raises :class:`JavaNullPointerException`, as the original does on
    Linux.

    ``original`` mode, as the Java: a ``.dat`` file that cannot be opened (e.g. its directory
    is missing) prints ``java.io.FileNotFoundException`` to stderr and is skipped, and so is
    (silently) a text file. Such paths are not in the returned list; pass a list as ``failed`` to collect them. The original
    creates each directory in the text writer; with ``write_text=False`` (new) the ``.dat``
    writer creates it instead, so the ``.dat`` files are written as with the text files.

    Differs from the original: ``fixed`` requires ``folder`` and joins
    ``folder/<sub>/<name>`` with ``os.path.join``, creating the directories.
    """
    orig = check_compat(compat)
    if folder is None:
        if not orig:
            raise ValueError("folder is required")
        folder = FILE_FOLDER
    rng = resolve_rng(rng)
    out = _Out(folder, compat, write_text, write_binary)
    for fam in families:
        if fam not in _FAMILY_FUNCS:
            raise ValueError(f"unknown family {fam!r}; choose from {sorted(_FAMILY_FUNCS)}")
        _FAMILY_FUNCS[fam](out, rng)
    if failed is not None:
        failed.extend(out.failed)
    return out.files
