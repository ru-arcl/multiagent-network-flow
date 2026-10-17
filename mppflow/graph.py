"""Graphs: the data structure, grid generators, manual construction and printing.

Port of ``projects.multipath.advanced.Graph`` (plus the non-GUI parts of ``Vertex`` and
``Path``) from the original Java code.

Data model
----------
A :class:`Graph` mirrors the four Java fields that matter:

* ``vertices`` -- the Java ``vertices[]`` array, as a list of vertex ids in array order.
  After :func:`convert_graph` or :meth:`Graph.finish_building_graph` the ids are
  ``0..n-1`` in ascending order, which is what every solver assumes.
* ``xy`` -- the Java ``idVertexMap``: vertex id -> point ``(x, y)`` (``x`` = column,
  ``y`` = row), or ``None`` for a vertex without coordinates (``new Vertex(int)``).
* ``adj`` -- the Java ``adjacencySetMap``: vertex id -> **ascending** list of neighbour
  ids, **including the vertex itself** (the "wait" self-loop). It is a ``dict`` because
  graphs before conversion have 1-based ids with gaps; for converted graphs the keys are
  exactly ``0..n-1``.
* ``rows``, ``cols`` -- the Java ``rows``/``columns`` (only set on unconverted grids;
  :func:`convert_graph` resets them to 0, as the original does).

Adjacency iteration order is the Java ``TreeSet`` order (ascending). Sub-graphs built by
the planner heuristics use ``HashSet<Integer>`` adjacency in the original; their iteration
order is modelled by ``Graph.adj_order`` (vertex -> list in iteration order, see
:meth:`Graph.set_hashset_adjacency`). Every routine that iterates neighbours in the
original's order calls :meth:`Graph.neighbors`, which honours ``adj_order``.

compat modes
------------
Every ported function takes ``compat="fixed"`` (default) or ``compat="original"``.
``fixed`` gives the same results, in the same order, with the same random draws, wherever
the original is correct. Differences of ``fixed`` from the original:

* ``create_2d_grid_graph``: a grid with ``cols == 1`` has its vertical edges (the
  original deletes them because ``id +- 1 == id +- cols``).
* ``create_2d_grid_graph_8connected``: grids with ``cols <= 2`` get their correct
  8-neighbourhood (the original loses horizontal and diagonal edges, makes the adjacency
  asymmetric, and with ``cols == 1`` even deletes the self-loops).
* ``create_generic_2d_grid_graph_with_holes[_8connected]``: asking for more holes than
  vertices raises ``ValueError`` before any draw. The original usually loops forever
  (``original`` simulates it and raises :class:`~mppflow.errors.OriginalWouldHang` once
  no vertex is left that could be removed); on the 8-connected ``cols == 1`` grid it
  throws ``NullPointerException`` instead, which ``original`` reproduces.
* ``Graph.is_connected_without_vertex``: leaves the adjacency unchanged (the original
  drops the self-loop in the single-neighbour case and adds a missing one otherwise).
* ``Graph.remove_vertex``: also removes the id from ``vertices`` (the original leaves the
  array stale).
* ``get_random_start_goal``: ``number`` larger than the number of eligible vertices raises
  ``ValueError`` (the original loops forever).
* ``get_random_start_goal_many``: ``number > |V|`` raises ``ValueError`` before any draw
  (the original throws ``IndexOutOfBoundsException`` after one more draw).
* ``convert_graph``, ``Graph.print_graph_as_adjacency_list``, ``print_path``: inputs the
  original dereferences as ``null`` raise ``ValueError`` (``original`` raises
  :class:`~mppflow.errors.JavaNullPointerException`); the adjacency printer prints the
  real vertex ids when they are not ``0..n-1``; ``print_path`` prints a bare id for a
  vertex that is not in the graph.

Not reproduced in either mode: the identity-hash order of the original ``verticeSet`` (it
leaks into the raw ``vertices[]`` of unconverted grids only, and no reachable output
depends on it). We use ascending id order there.

Dead code of the original (no callers) is ported and marked *dead* in its docstring:
:meth:`Graph.get_edge_set`, :func:`get_edge_id`, :func:`get_first_vertex_of_edge`,
:func:`get_second_vertex_of_edge`, :func:`update_adj_list`, the public
:meth:`Graph.remove_vertex`, :func:`get_v4_test_graph`, :func:`get_test_graph_for_bridge`,
:func:`print_path`. GUI drawing methods are not ported.
"""
from __future__ import annotations

import bisect
import math
import operator
from collections import deque
from typing import Iterable, Sequence

from ._java import JavaIntHashSet, MathRandom, i32, i64, resolve_rng
from .errors import (
    JavaArrayIndexOutOfBoundsException,
    JavaIndexOutOfBoundsException,
    JavaNegativeArraySizeException,
    JavaNullPointerException,
    OriginalWouldHang,
)

__all__ = [
    "Graph",
    "check_compat",
    "java_d2i",
    "create_2d_grid_graph",
    "create_2d_grid_graph_8connected",
    "create_generic_2d_grid_graph_with_holes",
    "create_generic_2d_grid_graph_with_holes_8connected",
    "get_random_start_goal",
    "get_random_start_goal_many",
    "convert_graph",
    "update_adj_list",
    "EDGE_MULTIPLIER",
    "get_edge_id",
    "get_first_vertex_of_edge",
    "get_second_vertex_of_edge",
    "SG",
    "BRIDGE_TEST_VERTICES",
    "get_v4_test_graph",
    "get_test_graph_for_bridge",
    "create_main_case3_graph",
    "create_main_case6_tree",
    "vertex_print",
    "print_vertex",
    "print_vertex_to_buffer",
    "print_path",
]

_COMPAT = ("fixed", "original")


def as_int(x, what: str = "a vertex id") -> int:
    """``x`` as a plain ``int``: integer types such as ``numpy.int64`` are converted
    (``operator.index``); ``bool``, floats and other non-integers raise ``TypeError``.
    The Java-key emulation (:func:`~mppflow._java.i64`) needs plain ints."""
    if isinstance(x, bool):
        raise TypeError(f"{what} must be an integer, got bool {x!r}")
    try:
        return operator.index(x)
    except TypeError:
        raise TypeError(f"{what} must be an integer, got {type(x).__name__} {x!r}") from None


def as_ids(seq, what: str = "vertex ids") -> list[int]:
    """A sequence of vertex ids (a list, tuple, ``numpy`` array, ...) as a list of plain
    ``int`` (see :func:`as_int`)."""
    return [as_int(x, f"each of the {what}") for x in seq]


def check_compat(compat: str) -> bool:
    """Validate ``compat`` and return ``True`` for ``"original"``."""
    if compat not in _COMPAT:
        raise ValueError(f"compat must be 'fixed' or 'original', got {compat!r}")
    return compat == "original"


def java_d2i(d: float) -> int:
    """Java ``(int) d``: truncation toward zero, NaN -> 0, saturating at the int range."""
    if math.isnan(d):
        return 0
    if d >= 2147483647:
        return 2147483647
    if d <= -2147483648:
        return -2147483648
    return int(d)


def _npe(msg: str, output: str | None = None) -> JavaNullPointerException:
    e = JavaNullPointerException(msg)
    e.output = output
    return e


class Graph:
    """A graph with explicit wait self-loops (port of ``advanced.Graph``).

    See the module docstring for the meaning of ``vertices``, ``xy``, ``adj``,
    ``rows``/``cols`` and ``adj_order``.
    """

    def __init__(self) -> None:
        self.vertices: list[int] = []
        self.xy: dict[int, tuple | None] = {}
        self.adj: dict[int, list[int]] = {}
        self.rows: int = 0
        self.cols: int = 0
        self.adj_order: dict[int, list[int]] | None = None
        # Points of vertices removed from ``xy`` that a stale ``vertices`` array still
        # references (the original's Vertex objects keep their point).
        self._ghost_xy: dict[int, tuple | None] = {}

    # ------------------------------------------------------------------ basics
    @property
    def n(self) -> int:
        """Number of entries in ``vertices`` (Java ``vertices.length``)."""
        return len(self.vertices)

    def __len__(self) -> int:
        return len(self.vertices)

    def __repr__(self) -> str:
        return f"Graph(n={self.n}, rows={self.rows}, cols={self.cols})"

    def copy(self) -> "Graph":
        """A deep copy."""
        g = Graph()
        g.vertices = list(self.vertices)
        g.xy = dict(self.xy)
        g.adj = {k: list(v) for k, v in self.adj.items()}
        g.rows, g.cols = self.rows, self.cols
        g.adj_order = None if self.adj_order is None else {k: list(v) for k, v in self.adj_order.items()}
        g._ghost_xy = dict(self._ghost_xy)
        return g

    def neighbors(self, v: int) -> list[int] | None:
        """Neighbours of ``v`` (self included) in the original's iteration order.

        This is ``adj_order[v]`` when an override is set (``HashSet`` sub-graphs), else
        the ascending ``adj[v]``. Returns ``None`` when ``v`` has no adjacency entry
        (Java ``adjacencySetMap.get(v) == null``).
        """
        if self.adj_order is not None:
            o = self.adj_order.get(v)
            if o is not None:
                return o
        return self.adj.get(v)

    def adjacency_lists(self) -> list[list[int]]:
        """``[adj[0], ..., adj[n-1]]`` for a graph with ids ``0..n-1``."""
        return [self.adj[v] for v in range(self.n)]

    def point(self, v: int):
        """The point of vertex ``v`` (``None`` if it has no coordinates)."""
        if v in self.xy:
            return self.xy[v]
        return self._ghost_xy.get(v)

    def ix(self, v: int, compat: str = "fixed") -> int:
        """``Vertex.getIX()``: ``(int) x``, truncated toward zero.

        A vertex without coordinates raises ``ValueError`` (``original``:
        :class:`JavaNullPointerException`).
        """
        p = self.point(v)
        if p is None:
            if check_compat(compat):
                raise _npe('Cannot invoke "java.awt.geom.Point2D.getX()" because "this.point" is null')
            raise ValueError(f"vertex {v} has no coordinates")
        return java_d2i(p[0])

    def iy(self, v: int, compat: str = "fixed") -> int:
        """``Vertex.getIY()``: ``(int) y``, truncated toward zero."""
        p = self.point(v)
        if p is None:
            if check_compat(compat):
                raise _npe('Cannot invoke "java.awt.geom.Point2D.getY()" because "this.point" is null')
            raise ValueError(f"vertex {v} has no coordinates")
        return java_d2i(p[1])

    def get_id(self, x: int, y: int) -> int:
        """The pre-conversion grid id ``y * cols + x + 1`` (Java ``getId``, 1-based)."""
        return i32(y * self.cols + x + 1)

    def to_dict(self) -> dict:
        """A JSON-friendly snapshot (used by tests)."""
        return {
            "vertices": list(self.vertices),
            "xy": {str(k): (None if p is None else [p[0], p[1]]) for k, p in sorted(self.xy.items())},
            "adj": {str(k): list(v) for k, v in sorted(self.adj.items())},
            "rows": self.rows,
            "cols": self.cols,
        }

    @classmethod
    def from_adjacency(cls, adj, xy=None) -> "Graph":
        """Build a graph with ids ``0..n-1`` from neighbour lists.

        ``adj`` is a list (or a dict keyed ``0..n-1``) of neighbour iterables; each vertex
        gets its self-loop added, as ``addVertex(int, int[])`` does. Reverse edges are
        **not** added. ``xy`` optionally gives a point per vertex. Equivalent to
        ``add_vertex`` for each vertex followed by ``finish_building_graph``.
        """
        g = cls()
        items = adj.items() if isinstance(adj, dict) else enumerate(adj)
        for v, nb in items:
            g.add_vertex(v, list(nb))
            if xy is not None:
                p = xy[v]
                g.xy[as_int(v)] = None if p is None else tuple(p)
        g.finish_building_graph()
        return g

    # ---------------------------------------------------- manual construction
    def add_vertex(self, v_id: int, neighbors: Sequence[int] = ()) -> None:
        """``addVertex(int vId, int[] neighbors)``.

        Adds a vertex **without coordinates** and sets its adjacency to
        ``{v_id} | neighbors`` (ascending). Reverse edges are not added, and
        ``vertices`` is not updated until :meth:`finish_building_graph`.
        """
        v_id = as_int(v_id)
        self.xy[v_id] = None
        self.adj[v_id] = sorted({v_id, *as_ids(neighbors, "neighbours")})

    def add_vertex_point(self, v_id: int, xy) -> None:
        """``addVertex(Vertex v)`` with ``v.id = v_id`` and ``v.point = xy``.

        Adds the vertex to the id map only (no adjacency entry). Used by the dead test
        fixtures.
        """
        self.xy[v_id] = None if xy is None else tuple(xy)

    def finish_building_graph(self) -> None:
        """``finishBuildingGraph()``: ``vertices`` = all ids in ascending order."""
        self.vertices = sorted(self.xy)

    def set_hashset_adjacency(self, v: int, insertion_order: Iterable[int]) -> None:
        """Give ``v`` a ``HashSet<Integer>`` adjacency, as the planner sub-graphs have.

        ``insertion_order`` lists the ids in the order the original adds them to the
        set. ``adj[v]`` becomes the ascending list and ``adj_order[v]`` the Java
        ``HashSet`` iteration order (emulated with
        :class:`~mppflow._java.JavaIntHashSet`).
        """
        hs = JavaIntHashSet(insertion_order)
        if self.adj_order is None:
            self.adj_order = {}
        self.adj[v] = sorted(hs)
        self.adj_order[v] = hs.to_list()

    # ------------------------------------------------------------- mutation
    def _set_remove(self, k: int, v: int) -> bool:
        lst = self.adj[k]
        i = bisect.bisect_left(lst, v)
        if i < len(lst) and lst[i] == v:
            del lst[i]
            if self.adj_order is not None and k in self.adj_order:
                self.adj_order[k].remove(v)
            return True
        return False

    def _set_add(self, k: int, v: int) -> bool:
        lst = self.adj[k]
        i = bisect.bisect_left(lst, v)
        if i < len(lst) and lst[i] == v:
            return False
        lst.insert(i, v)
        if self.adj_order is not None and k in self.adj_order:
            hs = JavaIntHashSet(self.adj_order[k])
            hs.add(v)
            self.adj_order[k] = hs.to_list()
        return True

    def remove_vertex(self, v_id: int, compat: str = "fixed") -> None:
        """``removeVertex(Vertex v)`` (public; *dead* in the original).

        Removes ``v_id`` from the id map and the adjacency map, removes it from every
        adjacency set, and deletes every adjacency entry whose set becomes (or already
        is) empty.

        Differs from the original: ``fixed`` also removes ``v_id`` from ``vertices``;
        the original leaves the array stale.
        """
        orig = check_compat(compat)
        if v_id in self.xy:
            self._ghost_xy[v_id] = self.xy.pop(v_id)
        self.adj.pop(v_id, None)
        if self.adj_order is not None:
            self.adj_order.pop(v_id, None)
        for k in list(self.adj):
            self._set_remove(k, v_id)
            if not self.adj[k]:
                del self.adj[k]
                if self.adj_order is not None:
                    self.adj_order.pop(k, None)
        if not orig:
            self.vertices = [u for u in self.vertices if u != v_id]

    def remove_vertex_xy(self, x: int, y: int, compat: str = "fixed") -> None:
        """The private ``removeVertex(int x, int y)``: removes grid vertex ``get_id(x, y)``.

        Same semantics (and the same ``fixed`` difference) as :meth:`remove_vertex`.
        """
        self.remove_vertex(self.get_id(x, y), compat)

    # ---------------------------------------------------------- connectivity
    def are_vertices_connected_without_v3(self, v1: int, v2: int, v3: int) -> bool:
        """``areVerticesConnectedWithoutV3``: is ``v2`` reachable from ``v1`` avoiding ``v3``?

        BFS over the (possibly directed) adjacency, skipping ids without an adjacency
        entry. As in the original, ``v1`` itself is not marked visited, and ``v1 == v2``
        is only reported when ``v1`` is reachable from itself (e.g. through its
        self-loop).
        """
        visited = {v3}
        queue = deque([v1])
        adj = self.adj
        while queue:
            v = queue.popleft()
            nb = adj.get(v)
            if nb is None:
                continue
            for u in nb:
                if u not in visited:
                    if u == v2:
                        return True
                    queue.append(u)
                    visited.add(u)
        return False

    def is_removable(self, v_id: int) -> bool:
        """Side-effect-free value of :meth:`is_connected_without_vertex` (``v_id`` must
        have an adjacency entry)."""
        nb = [u for u in self.adj[v_id] if u != v_id]
        if len(nb) == 1:
            return True
        for i in range(len(nb) - 1):
            if not self.are_vertices_connected_without_v3(nb[i], nb[i + 1], v_id):
                return False
        return True

    def is_connected_without_vertex(self, v_id: int, compat: str = "fixed") -> bool:
        """``isConnectedWithoutVertex``: do the other neighbours of ``v_id`` stay
        pairwise connected (along the ascending chain) when ``v_id`` is removed?

        In ``original`` mode the adjacency set of ``v_id`` is mutated as in Java: its
        self-loop is removed, and it is **not** restored when ``v_id`` has exactly one
        other neighbour; in the other cases the self-loop is added back (or added, if it
        was missing). A missing adjacency entry raises
        :class:`JavaNullPointerException`.

        Differs from the original: ``fixed`` leaves the adjacency unchanged (and raises
        ``ValueError`` for a missing entry). The return value is the same.
        """
        orig = check_compat(compat)
        if v_id not in self.adj:
            if orig:
                raise _npe('Cannot invoke "java.util.Set.remove(Object)" because "neighborSet" is null')
            raise ValueError(f"vertex {v_id} has no adjacency entry")
        if not orig:
            return self.is_removable(v_id)
        self._set_remove(v_id, v_id)
        nb = list(self.adj[v_id])
        if len(nb) == 1:
            return True  # quirk: the self-loop is not restored
        for i in range(len(nb) - 1):
            if not self.are_vertices_connected_without_v3(nb[i], nb[i + 1], v_id):
                self._set_add(v_id, v_id)
                return False
        self._set_add(v_id, v_id)
        return True

    # --------------------------------------------------------------- edges
    def get_edge_set(self, compat: str = "fixed") -> list[int]:
        """``getEdgeSet()`` (*dead*): edge ids ``get_edge_id(i, j)`` of all non-loop
        adjacencies of vertices ``0..n-1``, in Java ``HashSet<Long>`` iteration order.

        A missing adjacency entry raises ``ValueError`` (``original``:
        :class:`JavaNullPointerException`).
        """
        orig = check_compat(compat)
        hs = _JavaLongHashSet()
        for i in range(self.n):
            nb = self.adj.get(i)
            if nb is None:
                if orig:
                    raise _npe('Cannot invoke "java.util.Set.iterator()" because the return value of "java.util.Map.get(Object)" is null')
                raise ValueError(f"vertex {i} has no adjacency entry")
            for j in self.neighbors(i):
                if i == j:
                    continue
                hs.add(get_edge_id(i, j))
        return hs.to_list()

    # ------------------------------------------------------------ printing
    def print_graph_as_adjacency_list(self, compat: str = "fixed", echo: bool = False) -> str:
        """``printGraphAsAdjacenyList()`` (sic): the exact text the original prints.

        One line per vertex index ``i`` in ``0..n-1``: ``"i: "`` followed by
        ``"u, "`` for every neighbour ``u != i`` in iteration order, then ``"\\n"``.
        Returns the text; ``echo=True`` also prints it.

        In ``original`` mode an index without an adjacency entry raises
        :class:`JavaNullPointerException` whose ``output`` attribute holds the text
        printed before the crash.

        Differs from the original: when the ids are not ``0..n-1``, ``fixed`` prints
        each vertex of ``vertices`` under its own id instead of crashing (for ids
        ``0..n-1`` the output is identical).
        """
        orig = check_compat(compat)
        out: list[str] = []
        if orig:
            ids = range(self.n)
        else:
            ids = range(self.n) if all(i in self.adj for i in range(self.n)) else list(self.vertices)
        for i in ids:
            out.append(f"{i}: ")
            nb = self.neighbors(i)
            if nb is None:
                text = "".join(out)
                if echo:
                    print(text, end="")
                if orig:
                    raise _npe('Cannot invoke "java.util.Set.toArray(Object[])" because the return value of "java.util.Map.get(Object)" is null', text)
                raise ValueError(f"vertex {i} has no adjacency entry")
            for u in nb:
                if u != i:
                    out.append(f"{u}, ")
            out.append("\n")
        text = "".join(out)
        if echo:
            print(text, end="")
        return text

    # Keep the original's (misspelled) name as an alias.
    print_graph_as_adjaceny_list = print_graph_as_adjacency_list


class _JavaLongHashSet(JavaIntHashSet):
    """``HashSet<Long>`` order: ``Long.hashCode = (int)(v ^ (v >>> 32))``."""

    __slots__ = ()

    @staticmethod
    def _h(k: int) -> int:
        u = k & 0xFFFFFFFFFFFFFFFF
        h = (u ^ (u >> 32)) & 0xFFFFFFFF
        return h ^ (h >> 16)


# ---------------------------------------------------------------------------- grids
def _grid_vertices(g: Graph, rows: int, cols: int) -> None:
    for i in range(rows):
        for j in range(cols):
            g.xy[g.get_id(j, i)] = (j, i)


def create_2d_grid_graph(rows: int, cols: int, update: bool = True, compat: str = "fixed") -> Graph:
    """``create2DGridGraph(rows, cols, update)``: a 4-connected ``rows x cols`` grid.

    Vertex ``(x, y)`` (``x`` = column, ``y`` = row) gets the pre-conversion id
    ``y * cols + x + 1``. With ``update=True`` the graph is passed through
    :func:`convert_graph` (ids ``y * cols + x``, ``rows = cols = 0``).

    Differs from the original: for ``cols == 1`` the original deletes every vertical edge
    (``id + 1 == id + cols``), leaving only self-loops; ``fixed`` builds the path graph.
    The raw ``vertices`` order before conversion is ascending (the original's is an
    identity-hash order).
    """
    orig = check_compat(compat)
    g = Graph()
    g.rows, g.cols = rows, cols
    _grid_vertices(g, rows, cols)
    for i in range(rows):
        for j in range(cols):
            vid = g.get_id(j, i)
            if orig:
                nb = {vid, vid - 1, vid + 1, vid - cols, vid + cols}
                if i == 0:
                    nb.discard(vid - cols)
                if j == 0:
                    nb.discard(vid - 1)
                if i == rows - 1:
                    nb.discard(vid + cols)
                if j == cols - 1:
                    nb.discard(vid + 1)
            else:
                nb = {vid}
                for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    x, y = j + dx, i + dy
                    if 0 <= x < cols and 0 <= y < rows:
                        nb.add(g.get_id(x, y))
            g.adj[vid] = sorted(nb)
    g.vertices = sorted(g.xy)
    if not update:
        return g
    return convert_graph(g, [], [], compat)


def create_2d_grid_graph_8connected(rows: int, cols: int, update: bool = True, compat: str = "fixed") -> Graph:
    """``create2DGridGraph8Connected(rows, cols, update)``: an 8-connected grid.

    Same ids and conversion as :func:`create_2d_grid_graph`.

    Differs from the original: for ``cols <= 2`` the original's id arithmetic aliases
    neighbours (``id + cols - 1 == id + 1`` for ``cols == 2``, ``id +- cols -+ 1 == id``
    for ``cols == 1``), so it loses horizontal and diagonal edges, makes the adjacency
    asymmetric, and for ``cols == 1`` removes even the self-loops; ``fixed`` builds the
    true 8-neighbourhood. For ``cols >= 3`` both modes are identical.
    """
    orig = check_compat(compat)
    g = Graph()
    g.rows, g.cols = rows, cols
    _grid_vertices(g, rows, cols)
    for i in range(rows):
        for j in range(cols):
            vid = g.get_id(j, i)
            if orig:
                c = cols
                nb = {vid, vid - 1, vid + 1, vid - c, vid + c, vid - c - 1, vid - c + 1, vid + c - 1, vid + c + 1}
                if i == 0:
                    for r in (vid - c, vid - c - 1, vid - c + 1):
                        nb.discard(r)
                if j == 0:
                    for r in (vid - 1, vid - c - 1, vid + c - 1):
                        nb.discard(r)
                if i == rows - 1:
                    for r in (vid + c, vid + c - 1, vid + c + 1):
                        nb.discard(r)
                if j == cols - 1:
                    for r in (vid + 1, vid - c + 1, vid + c + 1):
                        nb.discard(r)
            else:
                nb = {vid}
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        x, y = j + dx, i + dy
                        if 0 <= x < cols and 0 <= y < rows:
                            nb.add(g.get_id(x, y))
            g.adj[vid] = sorted(nb)
    g.vertices = sorted(g.xy)
    if not update:
        return g
    return convert_graph(g, [], [], compat)


def _holes(g: Graph, rows: int, cols: int, percent_obstacle: float, rng: MathRandom | None, orig: bool) -> Graph:
    rng = resolve_rng(rng)
    compat = "original" if orig else "fixed"
    to_remove = java_d2i(float(i32(rows * cols)) * percent_obstacle)
    if to_remove > len(g.xy) and not orig:
        raise ValueError(f"cannot remove {to_remove} vertices from a graph with {len(g.xy)} "
                         "vertices (percent_obstacle too large)")
    # ``original`` simulates even when to_remove > |V|: the Java usually ends up stuck
    # (detected below), but on grids whose removals leave vertices without an adjacency
    # entry (8-connected cols == 1) its next draw of such a vertex throws
    # NullPointerException instead.
    # Reverse index (who lists u as a neighbour) so that a removal does not scan every
    # adjacency set; the effect is exactly that of the original's brute-force scan.
    rev: dict[int, set[int]] = {}
    for k, nb in g.adj.items():
        for u in nb:
            rev.setdefault(u, set()).add(k)
    empties = {k for k, nb in g.adj.items() if not nb}
    failures = 0
    check_every = 20 * max(1, rows * cols) + 100
    while to_remove > 0:
        y = int(rng.random() * rows)
        x = int(rng.random() * cols)
        vid = g.get_id(x, y)
        removed = False
        if vid in g.xy:
            if g.is_connected_without_vertex(vid, compat):
                # == g.remove_vertex_xy(x, y): drop vid everywhere, delete empty sets.
                g._ghost_xy[vid] = g.xy.pop(vid)
                g.adj.pop(vid, None)
                for k in rev.pop(vid, ()):
                    nb = g.adj.get(k)
                    if nb is None:
                        continue
                    g._set_remove(k, vid)
                    if not nb:
                        empties.add(k)
                for k in empties:
                    if k in g.adj and not g.adj[k]:
                        del g.adj[k]
                empties.clear()
                to_remove -= 1
                removed = True
            elif orig and vid in g.adj and vid in g.adj[vid]:
                rev.setdefault(vid, set()).add(vid)
        if removed:
            failures = 0
            continue
        failures += 1
        if failures % check_every == 0:
            # Deterministic hang detection: no remaining vertex can ever be removed (or
            # none remains), and none lacks an adjacency entry (drawing one would throw).
            if not any(v in g.adj and g.is_removable(v) for v in g.xy) and not any(v not in g.adj for v in g.xy):
                msg = ("no vertex is left to remove" if not g.xy else
                       "no remaining vertex can be removed without disconnecting the graph")
                if orig:
                    raise OriginalWouldHang("createGeneric2DGridGraphWithHoles loops forever: " + msg)
                # Unreachable in ``fixed``: its grids are connected (a connected graph
                # always has a removable vertex) and to_remove <= |V| was checked.
                raise RuntimeError(msg)  # pragma: no cover
    g.vertices = sorted(g.xy)
    return g


def create_generic_2d_grid_graph_with_holes(rows: int, cols: int, percent_obstacle: float,
                                            rng: MathRandom | None = None, compat: str = "fixed") -> Graph:
    """``createGeneric2DGridGraphWithHoles``: a 4-connected grid with
    ``(int)(rows * cols * percent_obstacle)`` random vertices removed, keeping it connected.

    Each attempt draws ``y = (int)(R * rows)`` then ``x = (int)(R * cols)`` (two draws, also
    for cells already removed and for cut vertices); vertex ``get_id(x, y)`` is removed
    when :meth:`Graph.is_connected_without_vertex` holds. The result is **not**
    converted: ids stay 1-based and ``rows``/``cols`` stay set.

    Differs from the original: asking for more holes than vertices raises ``ValueError``
    before any draw. ``original`` runs the loop as the Java does and raises
    :class:`OriginalWouldHang` when it is stuck for good (no vertex left, or none that
    can be removed), i.e. where the Java loops forever, or the Java's
    ``NullPointerException`` where a remaining vertex has lost its adjacency entry. The
    stuck test runs every ``20 * rows * cols + 100`` consecutive failed attempts. Grids
    with ``cols == 1`` are built correctly (see :func:`create_2d_grid_graph`).
    """
    orig = check_compat(compat)
    g = create_2d_grid_graph(rows, cols, False, compat)
    return _holes(g, rows, cols, percent_obstacle, rng, orig)


def create_generic_2d_grid_graph_with_holes_8connected(rows: int, cols: int, percent_obstacle: float,
                                                       rng: MathRandom | None = None,
                                                       compat: str = "fixed") -> Graph:
    """``createGeneric2DGridGraphWithHoles8Connected``: as
    :func:`create_generic_2d_grid_graph_with_holes` on an 8-connected grid.

    Differs from the original: as :func:`create_generic_2d_grid_graph_with_holes`, and the
    narrow-grid fix of :func:`create_2d_grid_graph_8connected`. (On the original's broken
    ``cols == 1`` 8-connected grid, the first removal deletes the other, empty, adjacency
    entries and the next attempt throws ``NullPointerException``; ``original`` reproduces
    that.)
    """
    orig = check_compat(compat)
    g = create_2d_grid_graph_8connected(rows, cols, False, compat)
    return _holes(g, rows, cols, percent_obstacle, rng, orig)


# ----------------------------------------------------------------- starts and goals
def _int_draw_values(m: int) -> range:
    """Values of ``(int)(Math.random() * m)``."""
    if m > 0:
        return range(m)
    if m == 0:
        return range(0, 1)
    return range(m + 1, 1)


def get_random_start_goal(g: Graph, number: int, rng: MathRandom | None = None,
                          compat: str = "fixed") -> list[list[int]]:
    """``getRandomStartGoal(g, number)``: ``[starts, goals]`` with pairwise distinct
    starts and pairwise distinct goals (a start may equal any goal).

    Unconverted grid (``cols != 0``): per attempt draw ``x = (int)(R * cols)``,
    ``y = (int)(R * rows)`` for the start; if it is taken or not a vertex, retry;
    otherwise draw the goal the same way; if that fails, retry **both** (so a failed
    attempt costs 2 or 4 draws). Other graphs: draw ``id = (int)(R * n)`` for the start
    and then for the goal, with the same retry rule.

    Differs from the original: when ``number`` exceeds the number of vertices that can
    be drawn, ``fixed`` raises ``ValueError`` and ``original`` raises
    :class:`OriginalWouldHang` (the Java loops forever), both before any draw.
    """
    orig = check_compat(compat)
    rng = resolve_rng(rng)
    if number < 0:
        if orig:
            raise JavaNegativeArraySizeException(str(number))
        raise ValueError("number must be >= 0")
    if g.cols != 0:
        eligible = {i32(y * g.cols + x + 1) for x in _int_draw_values(g.cols) for y in _int_draw_values(g.rows)}
    else:
        eligible = set(_int_draw_values(g.n))
    eligible_count = sum(1 for v in eligible if v in g.xy)
    if number > eligible_count:
        msg = f"cannot draw {number} distinct starts/goals from {eligible_count} vertices"
        if orig:
            raise OriginalWouldHang("getRandomStartGoal loops forever: " + msg)
        raise ValueError(msg)
    start_set: set[int] = set()
    goal_set: set[int] = set()
    starts: list[int] = []
    goals: list[int] = []
    for _ in range(number):
        while True:
            if g.cols != 0:
                x = java_d2i(rng.random() * g.cols)
                y = java_d2i(rng.random() * g.rows)
                vid = g.get_id(x, y)
                if vid in start_set or vid not in g.xy:
                    continue
                x2 = java_d2i(rng.random() * g.cols)
                y2 = java_d2i(rng.random() * g.rows)
                vid2 = g.get_id(x2, y2)
                if vid2 in goal_set or vid2 not in g.xy:
                    continue
            else:
                vid = java_d2i(rng.random() * g.n)
                if vid in start_set or vid not in g.xy:
                    continue
                vid2 = java_d2i(rng.random() * g.n)
                if vid2 in goal_set or vid2 not in g.xy:
                    continue
            start_set.add(vid)
            goal_set.add(vid2)
            starts.append(vid)
            goals.append(vid2)
            break
    return [starts, goals]


def get_random_start_goal_many(g: Graph, number: int, rng: MathRandom | None = None,
                               compat: str = "fixed") -> list[list[int]]:
    """``getRandomStartGoalMany(g, number)``: starts are ``vertices[0..number-1]`` in
    order; goals are drawn without replacement, one draw each:
    ``goal_i = pool.pop((int)(R * len(pool)))``.

    (The original's variable names are swapped -- its "startList" supplies the random
    goals -- which does not change the behaviour.)

    Differs from the original: ``number > n`` raises ``ValueError`` before any draw;
    ``original`` makes the draws for the first ``n`` agents and one more, then raises
    :class:`~mppflow.errors.JavaIndexOutOfBoundsException`, as the Java does.
    """
    orig = check_compat(compat)
    rng = resolve_rng(rng)
    if number < 0:
        if orig:
            raise JavaNegativeArraySizeException(str(number))
        raise ValueError("number must be >= 0")
    if number > g.n and not orig:
        raise ValueError(f"cannot assign {number} agents on {g.n} vertices")
    pool = list(g.vertices)
    order = list(g.vertices)
    starts: list[int] = []
    goals: list[int] = []
    for _ in range(number):
        idx = java_d2i(rng.random() * len(pool))
        if idx >= len(pool):
            raise JavaIndexOutOfBoundsException(f"Index: {idx}, Size: {len(pool)}")
        goals.append(pool.pop(idx))
        if not order:
            raise JavaIndexOutOfBoundsException("Index: 0, Size: 0")
        starts.append(order.pop(0))
    return [starts, goals]


# ----------------------------------------------------------------------- conversion
def convert_graph(graph: Graph, start: list[int] | None = None, goal: list[int] | None = None,
                  compat: str = "fixed") -> Graph:
    """``convertGraph(graph, start, goal)``: renumber the vertices ``0..n-1`` by ascending
    old id.

    Returns a new graph whose vertex ``i`` keeps the point of the ``i``-th smallest old
    id, with adjacency mapped to the new ids (ascending), and ``rows = cols = 0`` (as in
    the original). ``start`` and ``goal`` (lists) are renumbered **in place**, for
    ``k < len(start)``: the caller's lists then hold the new ids, so pass the lists the
    solver will get and do not map them again; the old-to-new map is ``i``-th smallest old
    id -> ``i``.

    In ``original`` mode an adjacency entry that is missing, or that names a vertex not
    in ``vertices``, and a start/goal that is not a vertex, raise
    :class:`JavaNullPointerException`; duplicate ids in ``vertices`` raise
    :class:`JavaArrayIndexOutOfBoundsException`; a ``goal`` shorter than ``start`` raises
    :class:`JavaArrayIndexOutOfBoundsException` after renumbering up to it.

    Differs from the original: those inputs raise ``ValueError`` in ``fixed`` mode, as does
    a ``goal`` whose length differs from ``start`` (pass both or neither); an immutable
    ``start`` or ``goal`` (e.g. a tuple) raises ``TypeError``; and nothing is renumbered
    when an error is raised.
    """
    orig = check_compat(compat)
    start = [] if start is None else start
    goal = [] if goal is None else goal
    if not orig:
        for name, x in (("start", start), ("goal", goal)):
            if len(x) and not hasattr(x, "__setitem__"):
                raise TypeError(f"{name} is renumbered in place, so it must be a list, got {type(x).__name__}")
        if len(start) != len(goal):
            raise ValueError(f"start and goal must have the same length (pass both or neither), "
                             f"got {len(start)} and {len(goal)}")

    def fail(msg, exc=JavaNullPointerException):
        if orig:
            raise exc(msg)
        raise ValueError(msg)

    vs = sorted(set(graph.vertices))
    n = len(graph.vertices)
    if len(vs) < n:
        fail(f"Index {len(vs)} out of bounds for length {len(vs)}", JavaArrayIndexOutOfBoundsException)
    index = {v: i for i, v in enumerate(vs)}
    og = Graph()
    og.vertices = list(range(n))
    for i, old in enumerate(vs):
        og.xy[i] = graph.point(old)
        nb = graph.neighbors(old)
        if nb is None:
            fail(f"vertex {old} has no adjacency entry")
        new_nb = []
        for a in nb:
            if a not in index:
                fail(f"neighbour {a} of vertex {old} is not a vertex")
            new_nb.append(index[a])
        og.adj[i] = sorted(set(new_nb))
    if not orig:  # check first, so that an error leaves the lists unchanged
        for name, x in (("start", start), ("goal", goal)):
            for v in x:
                if v not in index:
                    fail(f"{name} {v} is not a vertex")
    for k in range(len(start)):
        if start[k] not in index:
            fail(f"start {start[k]} is not a vertex")
        start[k] = index[start[k]]
        if k >= len(goal):
            fail(f"Index {k} out of bounds for length {len(goal)}", JavaArrayIndexOutOfBoundsException)
        if goal[k] not in index:
            fail(f"goal {goal[k]} is not a vertex")
        goal[k] = index[goal[k]]
    return og


def update_adj_list(g: Graph) -> None:
    """``updateAdjList(g)`` (*dead*): remove every self-loop from the adjacency sets."""
    for k in list(g.adj):
        g._set_remove(k, k)


# ----------------------------------------------------------------- edge ids (dead)
EDGE_MULTIPLIER = 100000000
"""``Graph.EDGE_MULTIPLIER`` (*dead*)."""


def get_edge_id(u: int, v: int) -> int:
    """``getEdgeId(u, v)`` (*dead*): ``min * 10^8 + max`` as a Java ``long``."""
    return i64(v * EDGE_MULTIPLIER + u) if u > v else i64(u * EDGE_MULTIPLIER + v)


def _ldiv(a: int, b: int) -> int:
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b > 0) else -q


def get_first_vertex_of_edge(edge: int) -> int:
    """``getFirstVertexOfEdge(e)`` (*dead*): ``(int)(e / 10^8)``."""
    return i32(_ldiv(edge, EDGE_MULTIPLIER))


def get_second_vertex_of_edge(edge: int) -> int:
    """``getSecondVertexOfEdge(e)`` (*dead*): ``(int)(e % 10^8)`` (Java remainder)."""
    return i32(edge - _ldiv(edge, EDGE_MULTIPLIER) * EDGE_MULTIPLIER)


# --------------------------------------------------------------- test fixtures
SG = [[0, 3], [3, 0]]
"""``Graph.SG`` (*dead*): the start/goal pair used with :func:`get_v4_test_graph`."""

BRIDGE_TEST_VERTICES = [
    [0, 0], [0, 1], [1, 0], [1, 1], [2, 1], [2, 2], [3, 0],
    [3, 1], [4, 0], [4, 1], [4, 2], [5, 1], [5, 2], [6, 1],
]
"""``Graph.BRIDGE_TEST_VERTICES`` (*dead*)."""


def get_v4_test_graph() -> Graph:
    """``getv4TestGraph()`` (*dead*): a T-shaped graph, vertices 0..3 at
    (0,0), (0,1), (1,1), (0,2), adjacency ``{0:[0,1], 1:[0,1,2,3], 2:[1,2], 3:[1,3]}``.

    ``vertices`` is ascending (the original's is an identity-hash order).
    """
    g = Graph()
    for vid, p in enumerate([(0, 0), (0, 1), (1, 1), (0, 2)]):
        g.add_vertex_point(vid, p)
    g.adj = {0: [0, 1], 1: [0, 1, 2, 3], 2: [1, 2], 3: [1, 3]}
    g.vertices = sorted(g.xy)
    return g


def get_test_graph_for_bridge() -> Graph:
    """``getTestGraphForBridge()`` (*dead*): 14 vertices at ``BRIDGE_TEST_VERTICES``,
    adjacent when their Manhattan distance is below 2 (self included)."""
    g = Graph()
    for i, p in enumerate(BRIDGE_TEST_VERTICES):
        g.add_vertex_point(i, p)
    for v, (vx, vy) in enumerate(BRIDGE_TEST_VERTICES):
        g.adj[v] = sorted(u for u, (ux, uy) in enumerate(BRIDGE_TEST_VERTICES)
                          if abs(vx - ux) + abs(vy - uy) < 2)
    g.finish_building_graph()
    return g


def create_main_case3_graph() -> Graph:
    """The star graph built by hand in ``Main`` case 3 (0-1, 1-2, 1-3; no points).

    Helper added in the port (not a function of the original).
    """
    g = Graph()
    g.add_vertex(0, [1])
    g.add_vertex(1, [0, 2, 3])
    g.add_vertex(2, [1])
    g.add_vertex(3, [1])
    g.finish_building_graph()
    return g


def create_main_case6_tree(branches: int = 4, depth: int = 4) -> Graph:
    """The tree built by hand in ``Main`` case 6: a root 0 with ``branches`` paths of
    ``depth`` vertices each (no points).

    Helper added in the port (not a function of the original).
    """
    t = Graph()
    t.add_vertex(0, [i * depth + 1 for i in range(branches)])
    for i in range(1, branches + 1):
        for j in range(depth):
            offset = (i - 1) * depth
            if j == depth - 1:
                t.add_vertex(offset + j + 1, [offset + j])
            else:
                t.add_vertex(offset + j + 1, [0 if j == 0 else offset + j, offset + j + 2])
    t.finish_building_graph()
    return t


# ------------------------------------------------------------- Vertex / Path text
def vertex_print(g: Graph, v: int) -> str:
    """``Vertex.print()``: the id."""
    return f"{v}"


def print_vertex(g: Graph, v: int) -> str:
    """``Vertex.printVertex()``: ``"id(x,y)"``, or ``"id"`` for a vertex without
    coordinates. Returns the text (the original prints it without a newline)."""
    p = g.point(v)
    if p is None:
        return f"{v}"
    return f"{v}({java_d2i(p[0])},{java_d2i(p[1])})"


def print_vertex_to_buffer(g: Graph, v: int) -> str:
    """``Vertex.printVertex(StringBuffer)``: ``"(x,y)"`` -- **without the id** -- or
    ``""`` for a vertex without coordinates (quirk kept in both modes: it is the
    original's output format, used by ``PathPlanner.printPathToString``)."""
    p = g.point(v)
    if p is None:
        return ""
    return f"({java_d2i(p[0])},{java_d2i(p[1])})"


def print_path(g: Graph, path: Sequence[int], compat: str = "fixed", echo: bool = False) -> str:
    """``Path.printPath(g)`` (*dead*): ``print_vertex(v) + "|"`` for each vertex, then a
    newline.

    In ``original`` mode an id that is not in the graph raises
    :class:`JavaNullPointerException` (``output`` = the text printed before).

    Differs from the original: ``fixed`` prints the bare id for such a vertex.
    """
    orig = check_compat(compat)
    out = []
    for v in path:
        if v not in g.xy:
            if orig:
                text = "".join(out)
                if echo:
                    print(text, end="")
                raise _npe('Cannot invoke "projects.multipath.advanced.Vertex.printVertex()"', text)
            out.append(f"{v}|")
            continue
        out.append(print_vertex(g, v) + "|")
    out.append("\n")
    text = "".join(out)
    if echo:
        print(text, end="")
    return text
