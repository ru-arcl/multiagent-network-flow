"""Time-expanded networks: reachability pruning, the paper's full network, the compact
network, and the per-agent arc sets the ILP models are built on.

Port of ``updateReachableSets`` and of the column-generation loop shared by the
``prepareModel`` methods of ``MultiagentGraphSolverGurobi{Time,TimeR,TimePuzzle,TotalTime,
Distance}`` and ``PuzzleSolver`` (original ``ILP/*.java``), plus new data structures for
the networks of our T-RO 2016 paper (Sec. III and Sec. V-A).

* :func:`update_reachable_sets` -- ``reach[a][v][t] = dist(s_a, v) <= t and
  dist(v, g_a) <= T - t`` (BFS hop distances; a vertex the BFS does not reach gets
  distance 0, exactly as ``PathFinder.getDistancesToAllVertices`` leaves it). The
  original computes the goal side as ``dist(g_a, v)`` (a forward BFS from the goal),
  which equals ``dist(v, g_a)`` only on symmetric graphs; see *compat modes*. Returned as
  a :class:`Reachability` (distances plus the horizon) instead of a ``boolean[A][n][T+1]``;
  :meth:`Reachability.matrix` gives the Java array.
* :func:`compact_arcs` -- the arcs ``(a, u, v, t)`` of the pruned compact network in the
  original's **column order**: agent-major; for ``t = 1..T`` the frontier set ``rVs`` (a
  Java ``HashSet<Integer>``, emulated by :class:`~mppflow._java.JavaIntHashSet`) is
  snapshotted, and for each ``curV`` of the snapshot and each ``nextV`` of
  ``adj[curV]`` (the graph's iteration order: ascending ``TreeSet``, or the ``HashSet``
  order of planner sub-graphs) with ``reach[a][nextV][t]`` the arc
  ``curV@(t-1) -> nextV@t`` is emitted and ``nextV`` is added to ``rVs``; afterwards
  ``curV`` is removed from ``rVs`` if ``not reach[a][curV][t]``.
* :class:`FullNetwork` -- the network ``G'`` of Sec. III / Fig. 5: ``2T+1`` copies
  ``v(0), v(1), v(1)', ..., v(T), v(T)'`` of every vertex (``v(0) = v(0)'``), blue arcs
  ``v(t) -> v(t)'`` (``t = 1..T``), green (wait) arcs ``v(t)' -> v(t+1)``, one merge-split
  gadget per edge and step with 5 arcs (``u(t)' -> m``, ``w(t)' -> m``, ``m -> s``,
  ``s -> u(t+1)``, ``s -> w(t+1)``), and one loopback ``g_i(T)' -> s_i(0)`` per robot.
  ``|V'| = |V|(2T+1) + 2|E|T`` and ``|E'| = 2|V|T + 5|E|T + n``.
* :class:`CompactNetwork` -- the compact network of Sec. V-A / Fig. 6-7: copies
  ``v(0..T)``, stay arcs ``v(t) -> v(t+1)``, two move arcs per edge and step, and the
  loopbacks; ``|V|(T+1)`` nodes and ``|V|T + 2|E|T`` arcs (+ ``n``). With
  ``prune=True`` it also holds the per-agent arc sets after reachability pruning, i.e.
  exactly the arc columns of the models in :mod:`mppflow.model`.

Both networks export a JSON-able dict (:meth:`FullNetwork.to_json`,
:meth:`CompactNetwork.to_json`) for the browser demo.

compat modes
------------
:func:`update_reachable_sets` and :func:`compact_arcs` take ``compat``. They behave the same
in both modes on every graph the original can build (its graphs are always symmetric);
the differences are:

* on a one-way (asymmetric) graph, e.g. from :meth:`Graph.from_adjacency`, ``fixed``
  takes the goal-side distance ``dist(v, g_a)`` from a BFS over the reversed arcs.
  ``original`` keeps the forward BFS from the goal (``dist(g_a, v)``), which prunes
  vertices a robot must pass through, so its models can be infeasible at the optimal
  horizon (suboptimal makespans, and no solution at all on a one-way cycle without
  wait loops);

* a vertex id outside ``0..n-1`` in a start, goal or adjacency list raises ``ValueError``
  (``original``: :class:`~mppflow.errors.JavaArrayIndexOutOfBoundsException`, as the
  Java array access does);
* a frontier vertex without an adjacency entry raises ``ValueError`` (``original``:
  :class:`~mppflow.errors.JavaNullPointerException`).

:class:`FullNetwork` and :class:`CompactNetwork` are new (the original never builds the
full network, and builds the compact one only implicitly); they have no ``compat`` switch.
Paper notes reproduced in their docstrings: the full network's flows do not map
one-to-one to paths (a robot can "stay" through a gadget), and the paper's unit cost on
green arcs makes the distance objectives constant (:attr:`FullNetwork.arc_cost` holds the
corrected costs, :attr:`FullNetwork.paper_cost` the literal ones).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Iterator, Sequence

from ._java import JavaIntHashSet
from .errors import JavaArrayIndexOutOfBoundsException, JavaNullPointerException
from .graph import Graph, check_compat
from .pathfinder import get_distances_to_all_vertices

__all__ = [
    "Reachability",
    "update_reachable_sets",
    "compact_arcs",
    "undirected_edges",
    "FullNetwork",
    "CompactNetwork",
]


# --------------------------------------------------------------------- reachability
@dataclass
class Reachability:
    """Reachability of each vertex by each agent at each time (Sec. V-A pruning).

    ``reachable(a, v, t)`` is ``dist_start[a][v] <= t and dist_goal[a][v] <= T - t``,
    the original's ``reachableVertices[a][v][t]``. For a vertex ``v`` the reachable times
    form the interval ``[dist_start[a][v], T - dist_goal[a][v]]``.
    """

    T: int
    dist_start: list[list[int]]
    dist_goal: list[list[int]]

    @property
    def n_agents(self) -> int:
        return len(self.dist_start)

    @property
    def n(self) -> int:
        return len(self.dist_start[0]) if self.dist_start else 0

    def reachable(self, a: int, v: int, t: int) -> bool:
        return self.dist_start[a][v] <= t and self.dist_goal[a][v] <= self.T - t

    __call__ = reachable

    def matrix(self) -> list[list[list[bool]]]:
        """The Java ``boolean[A][n][T+1]`` array."""
        T = self.T
        return [[[ds[v] <= t and dg[v] <= T - t for t in range(T + 1)] for v in range(len(ds))]
                for ds, dg in zip(self.dist_start, self.dist_goal)]

    def vertex_lists(self) -> list[list[list[int]]]:
        """``[a][t] -> ascending list of the vertices reachable at t`` (golden format)."""
        T = self.T
        return [[[v for v in range(len(ds)) if ds[v] <= t and dg[v] <= T - t] for t in range(T + 1)]
                for ds, dg in zip(self.dist_start, self.dist_goal)]


def update_reachable_sets(graph: Graph, starts: Sequence[int], goals: Sequence[int], T: int,
                          compat: str = "fixed") -> Reachability:
    """``updateReachableSets(graph, starts, goals, T)`` (identical in Time, TimeR,
    TotalTime, Distance and PuzzleSolver).

    One deterministic BFS from each start and each goal (no random draws). Vertices the
    BFS does not reach keep distance 0, as in the original (harmless: the frontier only
    grows inside the start's component, and on a one-way graph a vertex that cannot reach
    the goal only adds columns no feasible flow can use).

    Differs from the original: on an asymmetric adjacency ``fixed`` computes
    ``dist_goal[a][v] = dist(v, g_a)`` by a BFS over the reversed arcs; ``original`` runs
    the BFS forward from the goal (``dist(g_a, v)``), which wrongly prunes vertices on
    every shortest route (e.g. the one-way 4-cycle ``0 -> 1 -> 2 -> 3 -> 0`` with
    ``0 -> 3``: the original's models are infeasible at ``T = 3, 4``). On symmetric
    graphs both modes run the same forward BFS. ``fixed`` also raises ``ValueError`` for a start or goal outside
    ``0..n-1`` (``original``: :class:`JavaArrayIndexOutOfBoundsException`) and for a
    negative ``T`` (``original``: ``T = -1`` gives an empty array and ``T < -1``
    :class:`~mppflow.errors.JavaNegativeArraySizeException`, as ``new boolean[A][n][T+1]``).
    """
    orig = check_compat(compat)
    if T < 0 and not orig:
        raise ValueError(f"horizon T must be >= 0, got {T}")
    if T < -1:
        from .errors import JavaNegativeArraySizeException
        raise JavaNegativeArraySizeException(str(T + 1))
    # fixed, one-way graph: the goal side needs dist(v, g), a BFS over reversed arcs
    rev = None if orig else _reversed_adjacency(graph)
    ds_all, dg_all = [], []
    for a in range(len(starts)):
        ds_all.append(get_distances_to_all_vertices(graph, starts[a], None, compat))
        if rev is None:
            dg_all.append(get_distances_to_all_vertices(graph, goals[a], None, compat))
        else:
            dg_all.append(_distances_to(rev, goals[a], graph.n))
    return Reachability(T, ds_all, dg_all)


def _reversed_adjacency(graph: Graph) -> dict[int, list[int]] | None:
    """The predecessor lists of ``graph``, or ``None`` when its adjacency is symmetric
    (then ``dist(v, g) = dist(g, v)`` and the original's forward BFS is already right).

    Neighbours outside ``0..n-1`` are left to the forward BFS, which raises the
    documented ``ValueError`` (the graph is reported as symmetric then).
    """
    n = graph.n
    arcs = set()
    for u, nb in graph.adj.items():
        for w in nb:
            if not 0 <= w < n:
                return None
            arcs.add((u, w))
    if all((w, u) in arcs for (u, w) in arcs):
        return None
    rev: dict[int, list[int]] = {}
    for u, w in sorted(arcs):
        rev.setdefault(w, []).append(u)
    return rev


def _distances_to(rev: dict[int, list[int]], goal: int, n: int) -> list[int]:
    """BFS hop distances ``dist(v, goal)`` over the predecessor lists ``rev``; vertices
    that cannot reach ``goal`` keep 0, like the forward BFS of the original."""
    if not 0 <= goal < n:
        raise ValueError(f"vertex {goal} out of range 0..{n - 1}")
    dist = [0] * n
    seen = {goal}
    queue = [goal]
    head = 0
    while head < len(queue):
        v = queue[head]
        head += 1
        for u in rev.get(v, ()):
            if u not in seen:
                seen.add(u)
                dist[u] = dist[v] + 1
                queue.append(u)
    return dist


def _oob(orig: bool, i: int, n: int):
    if orig:
        return JavaArrayIndexOutOfBoundsException(f"Index {i} out of bounds for length {n}")
    return ValueError(f"vertex {i} out of range 0..{n - 1}")


def compact_arcs(graph: Graph, starts: Sequence[int], reach: Reachability,
                 compat: str = "fixed") -> Iterator[tuple[int, int, int, int]]:
    """Yield the arcs ``(a, u, v, t)`` (agent ``a`` from ``u`` at time ``t`` to ``v`` at
    ``t + 1``; ``u == v`` is a wait) of the reachability-pruned compact network, in the
    original's column order (see the module docstring).

    Differs from the original only on inputs it crashes on (see the module docstring).
    """
    orig = check_compat(compat)
    T = reach.T
    for a in range(len(starts)):
        ds, dg = reach.dist_start[a], reach.dist_goal[a]
        n = len(ds)
        rvs = JavaIntHashSet((starts[a],))
        for t in range(1, T + 1):
            lim = T - t
            for cur in rvs.to_list():
                nb = graph.neighbors(cur)
                if nb is None:
                    if orig:
                        raise JavaNullPointerException(
                            'Cannot invoke "java.util.Set.toArray(Object[])" because the return value of '
                            '"java.util.Map.get(Object)" is null')
                    raise ValueError(f"vertex {cur} has no adjacency entry")
                for nxt in nb:
                    if not 0 <= nxt < n:
                        raise _oob(orig, nxt, n)
                    if ds[nxt] <= t and dg[nxt] <= lim:
                        rvs.add(nxt)
                        yield (a, cur, nxt, t - 1)
                if not 0 <= cur < n:
                    raise _oob(orig, cur, n)
                if not (ds[cur] <= t and dg[cur] <= lim):
                    rvs.remove(cur)


def undirected_edges(graph: Graph) -> list[tuple[int, int]]:
    """The edges ``{u, w}`` (``u < w``, self-loops excluded) of ``graph``, ascending.

    An edge listed in only one direction of an asymmetric adjacency is still one edge.
    """
    es = set()
    for u, nb in graph.adj.items():
        for w in nb:
            if w != u:
                es.add((u, w) if u < w else (w, u))
    return sorted(es)


def _vertex_ids(graph: Graph) -> list[int]:
    return list(graph.vertices) if graph.vertices else sorted(graph.adj)


# --------------------------------------------------------------------- full network
@dataclass
class FullNetwork:
    """The time-expanded network ``G'`` of T-RO 2016 Sec. III (Fig. 5).

    Nodes (``nodes[i]`` is a tuple):

    * ``("v", v, t, 0)`` = ``v(t)``; ``("v", v, t, 1)`` = ``v(t)'`` for ``t >= 1``
      (``v(0)`` and ``v(0)'`` are one node, ``("v", v, 0, 0)``);
    * ``("m", u, w, t)`` / ``("s", u, w, t)`` = the merge / split node of the gadget of
      edge ``{u, w}`` (``u < w``) between ``t`` and ``t + 1``.

    Arcs (``arcs[j] = (tail, head, kind)``, node indices; loopbacks first, robot order;
    then for each ``t``: the green arcs of all vertices, then the 5 arcs of each gadget;
    then the blue arcs for ``t = 1..T``):

    ``"loop"`` ``g_i(T)' -> s_i(0)``, ``"green"`` ``v(t)' -> v(t+1)``,
    ``"blue"`` ``v(t) -> v(t)'``, ``"g_in"``, ``"g_mid"``, ``"g_out"``.

    The network is defined for an undirected graph on which every vertex allows waiting
    (Sec. II: the robots move on an undirected graph and may stay): :meth:`build` raises
    ``ValueError`` for an asymmetric (one-way) adjacency or a vertex without its self-loop.
    The two-way gadget of an edge would let a robot travel against a one-way arc, and its
    same-side exit ``u(t)' -> m -> s -> u(t+1)`` would let it wait where the graph does not
    allow it, so the full ILP would return plans that are invalid on the graph (every
    generator and :meth:`Graph.from_adjacency` on a symmetric list satisfy the condition;
    the compact models handle both cases).

    ``paper_cost[j]`` is the cost the paper states (green and gadget middle arcs 1,
    others 0); with it eqs. (12)/(16) are constant (every unit flow costs ``T``).
    ``arc_cost[j]`` is the corrected distance cost (only the gadget middle arc costs 1),
    which is what the original code (``MultiagentGraphSolverGurobiDistance``) charges.

    Theorem 1 is not a bijection on this network: a robot can stay at ``u`` by
    ``u(t)' -> m -> s -> u(t+1)`` through any incident gadget, so several flows map to one
    path (12 flows vs 4 paths for robot 1 of Fig. 4(a) at ``T = 4``).
    """

    graph: Graph
    starts: list[int]
    goals: list[int]
    T: int
    nodes: list[tuple] = field(default_factory=list)
    arcs: list[tuple[int, int, str]] = field(default_factory=list)
    paper_cost: list[int] = field(default_factory=list)
    arc_cost: list[int] = field(default_factory=list)
    arc_edge: list[tuple[int, int] | None] = field(default_factory=list)
    index: dict = field(default_factory=dict)

    @classmethod
    def build(cls, graph: Graph, starts: Sequence[int], goals: Sequence[int], T: int) -> "FullNetwork":
        if T < 0:
            raise ValueError(f"horizon T must be >= 0, got {T}")
        net = cls(graph, list(starts), list(goals), T)
        V = _vertex_ids(graph)
        E = undirected_edges(graph)
        for v in V:
            nb = graph.adj.get(v, ())
            if v not in nb:
                raise ValueError(f"the full network needs a wait loop at every vertex; vertex {v} has none "
                                 "(use the compact models)")
            for w in nb:
                if v not in graph.adj.get(w, ()):
                    raise ValueError(f"the full network needs an undirected graph; the arc {v} -> {w} is "
                                     "one-way (use the compact models)")

        def node(key):
            i = net.index.get(key)
            if i is None:
                i = len(net.nodes)
                net.index[key] = i
                net.nodes.append(key)
            return i

        def cp(v, t, prime):
            return node(("v", v, 0, 0) if t == 0 else ("v", v, t, prime))

        for v in V:
            cp(v, 0, 0)
        for t in range(1, T + 1):
            for v in V:
                cp(v, t, 0)
                cp(v, t, 1)
        for t in range(T):
            for (u, w) in E:
                node(("m", u, w, t))
                node(("s", u, w, t))

        def arc(a, b, kind, pc, c, e=None):
            net.arcs.append((a, b, kind))
            net.paper_cost.append(pc)
            net.arc_cost.append(c)
            net.arc_edge.append(e)

        for s, g in zip(net.starts, net.goals):
            arc(cp(g, T, 1), cp(s, 0, 0), "loop", 0, 0)
        for t in range(T):
            for v in V:
                nb = graph.adj.get(v, ())
                if v in nb:
                    arc(cp(v, t, 1), cp(v, t + 1, 0), "green", 1, 0)
            for (u, w) in E:
                m, s = net.index[("m", u, w, t)], net.index[("s", u, w, t)]
                arc(cp(u, t, 1), m, "g_in", 0, 0, (u, w))
                arc(cp(w, t, 1), m, "g_in", 0, 0, (u, w))
                arc(m, s, "g_mid", 1, 1, (u, w))
                arc(s, cp(u, t + 1, 0), "g_out", 0, 0, (u, w))
                arc(s, cp(w, t + 1, 0), "g_out", 0, 0, (u, w))
        for t in range(1, T + 1):
            for v in V:
                arc(cp(v, t, 0), cp(v, t, 1), "blue", 0, 0)
        return net

    # ------------------------------------------------------------------ queries
    def node_id(self, key: tuple) -> int:
        return self.index[key]

    def copy_node(self, v: int, t: int, prime: bool = False) -> int:
        """Index of ``v(t)`` (or ``v(t)'``)."""
        return self.index[("v", v, 0, 0) if t == 0 else ("v", v, t, 1 if prime else 0)]

    def counts(self) -> dict:
        """Node and arc counts (compare with the paper's formulas, :meth:`formula_counts`)."""
        by_kind: dict[str, int] = {}
        for _, _, k in self.arcs:
            by_kind[k] = by_kind.get(k, 0) + 1
        return {"vertices_total": len(self.nodes), "arcs_total": len(self.arcs),
                "by_kind": {k: by_kind.get(k, 0) for k in ("loop", "green", "blue", "g_in", "g_mid", "g_out")}}

    def formula_counts(self) -> dict:
        """The paper's counts: ``|V|(2T+1) + 2|E|T`` nodes, ``2|V|T + 5|E|T + n`` arcs."""
        V = len(_vertex_ids(self.graph))
        E = len(undirected_edges(self.graph))
        T, n = self.T, len(self.starts)
        return {"vertex_copies": 2 * T + 1, "copy_vertices": V * (2 * T + 1), "gadget_vertices": 2 * E * T,
                "vertices_total": V * (2 * T + 1) + 2 * E * T, "green_arcs": V * T, "blue_arcs": V * T,
                "gadget_arcs": 5 * E * T, "gadgets": E * T, "arcs_without_loopbacks": 2 * V * T + 5 * E * T,
                "loopback_arcs": n, "arcs_total": 2 * V * T + 5 * E * T + n,
                "x_variables_eq8": n * (2 * V * T + 5 * E * T + n)}

    def in_arcs(self) -> list[list[int]]:
        out: list[list[int]] = [[] for _ in self.nodes]
        for j, (_, b, _) in enumerate(self.arcs):
            out[b].append(j)
        return out

    def out_arcs(self) -> list[list[int]]:
        out: list[list[int]] = [[] for _ in self.nodes]
        for j, (a, _, _) in enumerate(self.arcs):
            out[a].append(j)
        return out

    def to_json(self) -> dict:
        """A JSON-able description (for the browser demo).

        Nodes carry ``col`` (layout column: ``2t`` for ``v(t)``, ``2t + 0.5`` for ``v(t)'``,
        ``2t + 1`` and ``2t + 1.5`` for the merge and split nodes) and ``row`` (the vertex,
        or the mean of the edge's endpoints for gadget nodes).
        """
        nodes = []
        for i, key in enumerate(self.nodes):
            if key[0] == "v":
                _, v, t, p = key
                nodes.append({"id": i, "kind": "copy'" if p else "copy", "v": v, "t": t,
                              "col": 2 * t + (0.5 if p else 0.0), "row": v})
            else:
                k, u, w, t = key
                nodes.append({"id": i, "kind": "merge" if k == "m" else "split", "edge": [u, w], "t": t,
                              "col": 2 * t + (1.0 if k == "m" else 1.5), "row": (u + w) / 2})
        arcs = []
        for j, (a, b, k) in enumerate(self.arcs):
            d = {"id": j, "tail": a, "head": b, "kind": k, "cost": self.arc_cost[j],
                 "paper_cost": self.paper_cost[j]}
            if self.arc_edge[j] is not None:
                d["edge"] = list(self.arc_edge[j])
            if k == "loop":
                d["robot"] = j
            arcs.append(d)
        return {"type": "full", "T": self.T, "starts": self.starts, "goals": self.goals,
                "vertices": _vertex_ids(self.graph), "edges": [list(e) for e in undirected_edges(self.graph)],
                "nodes": nodes, "arcs": arcs, "counts": self.counts()}

    def to_json_str(self, **kw) -> str:
        return json.dumps(self.to_json(), **kw)


# ------------------------------------------------------------------ compact network
@dataclass
class CompactNetwork:
    """The compact time-expanded network of T-RO 2016 Sec. V-A (Fig. 6 / Fig. 7).

    ``nodes[i] = (v, t)`` for ``t = 0..T`` (time-major, vertex order of the graph);
    ``arcs[j] = (u, w, t, kind)`` from ``u(t)`` to ``w(t+1)`` with ``kind`` ``"stay"``
    (``u == w``) or ``"move"``, for ``t = 0..T-1`` in vertex then neighbour order,
    followed by one ``(g_i, s_i, T, "loop")`` per robot. Each undirected edge gives two
    move arcs per step (the 2-arc gadget of Fig. 7) instead of the 5-arc gadget.

    With ``prune=True``, :attr:`agent_arcs` holds, per agent, the arcs that survive the
    reachability pruning, as ``(a, u, w, t)`` in the original's column order
    (:func:`compact_arcs`); these are exactly the arc columns of the models in
    :mod:`mppflow.model`.
    """

    graph: Graph
    starts: list[int]
    goals: list[int]
    T: int
    nodes: list[tuple[int, int]] = field(default_factory=list)
    arcs: list[tuple[int, int, int, str]] = field(default_factory=list)
    agent_arcs: list[list[tuple[int, int, int, int]]] | None = None
    reach: Reachability | None = None

    @classmethod
    def build(cls, graph: Graph, starts: Sequence[int], goals: Sequence[int], T: int,
              prune: bool = False, compat: str = "fixed") -> "CompactNetwork":
        if T < 0:
            raise ValueError(f"horizon T must be >= 0, got {T}")
        net = cls(graph, list(starts), list(goals), T)
        V = _vertex_ids(graph)
        for t in range(T + 1):
            for v in V:
                net.nodes.append((v, t))
        for t in range(T):
            for u in V:
                for w in graph.neighbors(u) or ():
                    net.arcs.append((u, w, t, "stay" if u == w else "move"))
        for s, g in zip(net.starts, net.goals):
            net.arcs.append((g, s, T, "loop"))
        if prune:
            net.reach = update_reachable_sets(graph, starts, goals, T, compat)
            per: list[list[tuple[int, int, int, int]]] = [[] for _ in starts]
            for arc in compact_arcs(graph, starts, net.reach, compat):
                per[arc[0]].append(arc)
            net.agent_arcs = per
        return net

    def counts(self) -> dict:
        stay = sum(1 for a in self.arcs if a[3] == "stay")
        move = sum(1 for a in self.arcs if a[3] == "move")
        loop = sum(1 for a in self.arcs if a[3] == "loop")
        out = {"vertices": len(self.nodes), "stay_arcs": stay, "move_arcs": move,
               "arcs_total": stay + move, "loopback_arcs": loop,
               "arcs_total_with_loopbacks": stay + move + loop,
               "crossing_pairs_per_step": len(undirected_edges(self.graph))}
        if self.agent_arcs is not None:
            out["pruned_arcs_per_agent"] = [len(x) for x in self.agent_arcs]
        return out

    def to_json(self) -> dict:
        """A JSON-able description (for the browser demo); node ``id`` = index in
        :attr:`nodes`, arcs refer to node ids."""
        idx = {k: i for i, k in enumerate(self.nodes)}
        nodes = [{"id": i, "v": v, "t": t, "col": t, "row": v} for i, (v, t) in enumerate(self.nodes)]
        arcs = []
        for j, (u, w, t, k) in enumerate(self.arcs):
            if k == "loop":
                d = {"id": j, "tail": idx[(u, t)], "head": idx[(w, 0)], "kind": k, "robot": j - (len(self.arcs) - len(self.starts))}
            else:
                d = {"id": j, "tail": idx[(u, t)], "head": idx[(w, t + 1)], "kind": k, "cost": 1 if k == "move" else 0}
            arcs.append(d)
        out = {"type": "compact", "T": self.T, "starts": self.starts, "goals": self.goals,
               "vertices": _vertex_ids(self.graph), "edges": [list(e) for e in undirected_edges(self.graph)],
               "nodes": nodes, "arcs": arcs, "counts": self.counts()}
        if self.agent_arcs is not None:
            out["agent_arcs"] = [[[idx[(u, t)], idx[(w, t + 1)]] for (_, u, w, t) in arcs_a]
                                 for arcs_a in self.agent_arcs]
        return out

    def to_json_str(self, **kw) -> str:
        return json.dumps(self.to_json(), **kw)
