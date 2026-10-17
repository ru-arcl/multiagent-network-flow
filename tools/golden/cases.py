"""Deterministic golden case specifications for the original Java code.

The cases are generated in two phases:

* ``phase1()`` returns cases whose results are *instances* (graphs and problems built by the
  original generators from a seeded ``Math.random()``).  They are recorded like every other case.
* ``phase2(inst)`` receives the phase-1 results (``{case_id: result}``) and returns all other
  cases, with the instances embedded explicitly in their arguments, so every recorded case is
  self-contained.

A case is a dict with keys ``id``, ``group``, ``fn``, ``args``, ``seed`` and optionally
``statics`` (static fields set after the reset to the declared defaults), ``budget`` (seconds
before the harness declares a hang), ``tags``, ``note``, ``nondet`` (record fields that are not
reproducible run to run, e.g. because of thread scheduling) and ``normalize`` (whether
wall-clock numbers in stdout/files are replaced by ``<t>``; default True).

Nothing here uses randomness: re-running the generator yields the same case list.
"""
from __future__ import annotations

import copy
from collections import deque

GROUPS = ["graph", "problem", "pathfinder", "models", "solvers", "planner", "heuristic", "main"]

# Main.main sets these two flags before doing anything; most cases do the same.
QUIET = {
    "MultiagentGraphSolverGurobiTime.bDebugInfo": False,
    "MultiagentGraphSolverGurobiTime.bPrintPaths": False,
}
ALL_QUIET = {
    **QUIET,
    "MultiagentGraphSolverGurobiTimeR.bDebugInfo": False,
    "MultiagentGraphSolverGurobiTimeR.bPrintPaths": False,
    "MultiagentGraphSolverGurobiTotalTime.bDebugInfo": False,
    "MultiagentGraphSolverGurobiTotalTime.bPrintPaths": False,
    "MultiagentGraphSolverGurobiDistance.bDebugInfo": False,
}
BIG = 1.0e6  # time limit (s) that never binds


class Cases:
    def __init__(self):
        self.items: list[dict] = []
        self.ids: set[str] = set()

    def add(self, group, cid, fn, args=None, seed=0, statics=None, budget=None, tags=(), note=None,
            nondet=None, normalize=None):
        assert group in GROUPS, group
        cid = f"{group}/{cid}"
        assert cid not in self.ids, cid
        self.ids.add(cid)
        c = {"id": cid, "group": group, "fn": fn, "args": copy.deepcopy(args or {}), "seed": seed}
        if statics:
            c["statics"] = dict(statics)
        if budget is not None:
            c["budget"] = budget
        if tags:
            c["tags"] = list(tags)
        if note:
            c["note"] = note
        if nondet:
            c["nondet"] = list(nondet)
        if normalize is not None:
            c["normalize"] = normalize
        self.items.append(c)
        return cid


# ----------------------------------------------------------------------------------------------
# Hand-built graphs, in exactly the form Graph.addVertex(int, int[]) + finishBuildingGraph()
# produces (vertex added to its own set; neighbour lists NOT symmetrised; no coordinates).
# ----------------------------------------------------------------------------------------------

def manual(nbrs: dict[int, list[int]]) -> dict:
    ids = sorted(nbrs)
    return {
        "vertices": [[i] for i in ids],
        "adj": [[i, sorted(set(nbrs[i]) | {i})] for i in ids],
        "adj_type": "TreeSet",
        "rows": 0,
        "cols": 0,
    }


def undirected(n: int, edges: list[tuple[int, int]]) -> dict:
    nb = {i: [] for i in range(n)}
    for u, v in edges:
        nb[u].append(v)
        nb[v].append(u)
    return manual(nb)


def line(n):
    return undirected(n, [(i, i + 1) for i in range(n - 1)])


STAR = manual({0: [1], 1: [0, 2, 3], 2: [1], 3: [1]})  # Main case 3
LINE2, LINE3, LINE5, LINE7, LINE8 = line(2), line(3), line(5), line(7), line(8)
CYCLE3 = undirected(3, [(0, 1), (1, 2), (2, 0)])
CYCLE5 = undirected(5, [(i, (i + 1) % 5) for i in range(5)])
TWOCOMP = undirected(4, [(0, 1), (2, 3)])
SQUARE = undirected(4, [(0, 1), (0, 2), (1, 3), (2, 3)])  # the 2x2 grid of original-bugs B10


def spider(branches=4, depth=4):
    """Main case 6's tree, built with the same addVertex calls."""
    nb = {0: [i * depth + 1 for i in range(branches)]}
    for i in range(1, branches + 1):
        off = (i - 1) * depth
        for j in range(depth):
            if j == depth - 1:
                nb[off + j + 1] = [off + j]
            else:
                nb[off + j + 1] = [0 if j == 0 else off + j, off + j + 2]
    return manual(nb)


SPIDER = spider()


def comb():
    """The 'comb' graph (exercises the HashSet
    bin-overflow resize in the column order)."""
    n = 160
    nb = {i: set() for i in range(n)}
    for k in range(9):
        nb[16 * k].add(16 * (k + 1))
        nb[16 * (k + 1)].add(16 * k)
    prev = 0
    for i in range(1, n):
        if i % 16 != 0 or i > 144:
            nb[prev].add(i)
            nb[i].add(prev)
            prev = i
    return manual({i: sorted(s) for i, s in nb.items()})


COMB = comb()


def with_points(g: dict, coords: dict[int, tuple[int, int]]) -> dict:
    g = copy.deepcopy(g)
    g["vertices"] = [[v[0], coords[v[0]][0], coords[v[0]][1]] for v in g["vertices"]]
    return g


# K1,9 with coordinates: Problem.writeToFile overflows its 13-int row (original-bugs B24).
K19 = with_points(undirected(10, [(0, i) for i in range(1, 10)]), {i: (i, 0) for i in range(10)})
K18 = with_points(undirected(9, [(0, i) for i in range(1, 9)]), {i: (i, 0) for i in range(9)})


# ----------------------------------------------------------------------------------------------
# Small graph utilities (case selection only)
# ----------------------------------------------------------------------------------------------

def adjacency(g: dict) -> dict[int, list[int]]:
    return {a[0]: a[1] for a in g["adj"]}


def bfs(g: dict, s: int) -> dict[int, int]:
    adj = adjacency(g)
    d = {s: 0}
    q = deque([s])
    while q:
        u = q.popleft()
        for v in adj.get(u, []):
            if v not in d:
                d[v] = d[u] + 1
                q.append(v)
    return d


def makespan_lb(g, starts, goals):
    return max(bfs(g, s)[t] for s, t in zip(starts, goals))


def problem_parts(p):
    return p["graph"], p["sg"][0], p["sg"][1]


# ----------------------------------------------------------------------------------------------
# Phase 1: instances
# ----------------------------------------------------------------------------------------------

GRID4 = [(1, 1, True), (1, 3, True), (2, 2, True), (2, 3, False), (2, 3, True), (3, 1, False),
         (3, 1, True), (3, 2, True), (3, 3, False), (3, 3, True), (3, 4, False), (4, 4, True),
         (5, 5, True), (6, 6, True), (8, 8, True)]
GRID8 = [(1, 3, True), (2, 2, False), (2, 2, True), (3, 1, False), (3, 2, False), (3, 3, True),
         (4, 4, True), (5, 5, True)]
HOLES4 = [(6, 6, 0.2, 1), (6, 6, 0.2, 2), (5, 4, 0.3, 3), (8, 8, 0.25, 4), (4, 4, 0.0, 1),
          (3, 3, 1.0, 1), (3, 3, 0.99, 1), (3, 1, 0.34, 1)]
HOLES8 = [(6, 6, 0.2, 1), (5, 5, 0.3, 2), (8, 8, 0.2, 3)]
GRID_PROBLEMS = {
    # name: (col, row, fracObs, n, seed)
    "p8x8_0_10_s1": (8, 8, 0.0, 10, 1),
    "p6x6_10_5_s2": (6, 6, 0.1, 5, 2),
    "p5x5_20_4_s3": (5, 5, 0.2, 4, 3),
    "p8x8_20_8_s4": (8, 8, 0.2, 8, 4),
    "p10x10_20_12_s5": (10, 10, 0.2, 12, 5),
    "p4x4_0_3_s8": (4, 4, 0.0, 3, 8),
    "p3x3_0_3_s9": (3, 3, 0.0, 3, 9),
    "p8x8_0_20_s7": (8, 8, 0.0, 20, 7),
    "p8x8_0_40_s1": (8, 8, 0.0, 40, 1),
    "p8x8_0_40_s100": (8, 8, 0.0, 40, 100),
    "p15x15_20_15_s1": (15, 15, 0.2, 15, 1),
    "p24x18_25_50_s1": (24, 18, 0.25, 50, 1),  # TOR Fig. 9 generator call
    "p8x8_10_10_s11": (8, 8, 0.1, 10, 11),
    "p6x6_0_6_s1": (6, 6, 0.0, 6, 1),
}
GRID8_PROBLEMS = {
    "p8c_6x6_10_6_s1": (6, 6, 0.1, 6, 1),
    "p8c_8x8_20_10_s2": (8, 8, 0.2, 10, 2),
}
PUZZLES = {
    # name: (fn, n, seed)
    "n2p3_s0": ("Problem.createN2Puzzle", 3, 0),
    "n2p3_s1": ("Problem.createN2Puzzle", 3, 1),
    "n2p3_s11": ("Problem.createN2Puzzle", 3, 11),  # splitPaths hangs on it (original-bugs B10)
    "n2p4_s1": ("Problem.createN2Puzzle", 4, 1),
    "n2p5_s1": ("Problem.createN2Puzzle", 5, 1),
    "n2m1p3_s0": ("Problem.createN2M1Puzzle", 3, 0),
    "n2m1p3_s2": ("Problem.createN2M1Puzzle", 3, 2),
    "n2m1p4_s1": ("Problem.createN2M1Puzzle", 4, 1),
}


def phase1() -> list[dict]:
    c = Cases()
    for r, k, u in GRID4:
        c.add("graph", f"grid4/{r}x{k}{'' if u else '-raw'}", "Graph.create2DGridGraph",
              {"rows": r, "cols": k, "update": u},
              tags=["defect:grid4-cols1-no-edges"] if k == 1 else ())
    for r, k, u in GRID8:
        c.add("graph", f"grid8/{r}x{k}{'' if u else '-raw'}", "Graph.create2DGridGraph8Connected",
              {"rows": r, "cols": k, "update": u},
              tags=["defect:grid8-narrow-wrap"] if k <= 2 else ())
    for r, k, p, s in HOLES4:
        c.add("graph", f"holes4/{r}x{k}_{p}_s{s}", "Graph.createGeneric2DGridGraphWithHoles",
              {"rows": r, "cols": k, "percentObstacle": p}, seed=s)
    for r, k, p, s in HOLES8:
        c.add("graph", f"holes8/{r}x{k}_{p}_s{s}", "Graph.createGeneric2DGridGraphWithHoles8Connected",
              {"rows": r, "cols": k, "percentObstacle": p}, seed=s)
    c.add("graph", "manual/star", "Graph.manual",
          {"neighbors": [[0, [1]], [1, [0, 2, 3]], [2, [1]], [3, [1]]]}, note="Main case 3 graph")
    c.add("graph", "manual/spider", "Graph.manual",
          {"neighbors": [[v[0], [x for x in v[1] if x != v[0]]] for v in SPIDER["adj"]]},
          note="Main case 6 tree")
    c.add("graph", "getv4TestGraph", "Graph.getv4TestGraph")
    c.add("graph", "getTestGraphForBridge", "Graph.getTestGraphForBridge")
    for name, (col, row, f, n, s) in GRID_PROBLEMS.items():
        c.add("problem", f"inst/{name}", "Problem.createGridProblem",
              {"col": col, "row": row, "fracObs": f, "n": n}, seed=s)
    for name, (col, row, f, n, s) in GRID8_PROBLEMS.items():
        c.add("problem", f"inst/{name}", "Problem.createGridProblem8Connected",
              {"col": col, "row": row, "fracObs": f, "n": n}, seed=s)
    for name, (fn, n, s) in PUZZLES.items():
        c.add("problem", f"inst/{name}", fn, {"n": n}, seed=s)
    c.add("problem", "inst/a9", "Problem.getA9Puzzle", note="TOR Fig. 1/3 9-puzzle")
    c.add("problem", "inst/garage", "Problem.getLongStraightWithOneGarageProblem")
    return c.items


# ----------------------------------------------------------------------------------------------
# Phase 2: everything else
# ----------------------------------------------------------------------------------------------

def phase2(inst: dict[str, object]) -> list[dict]:
    c = Cases()
    g = lambda name: inst[f"graph/{name}"]                    # noqa: E731
    P = lambda name: inst[f"problem/inst/{name}"]             # noqa: E731
    G3, G2, G4, G6, G8 = g("grid4/3x3"), g("grid4/2x2"), g("grid4/4x4"), g("grid4/6x6"), g("grid4/8x8")
    TS = ([0, 2, 6], [8, 6, 1])  # 3x3 grid instance used by many cases below
    _graph_cases(c, g)
    _problem_cases(c, g, P)
    _pathfinder_cases(c, g, P, G3, TS)
    _model_cases(c, g, P, G3, G6, TS)
    _solver_cases(c, g, P, G3, G6, TS)
    _planner_cases(c, g, P, G3, G2, G4, G8, TS)
    _heuristic_cases(c, g, P, G3, TS)
    _main_cases(c, g, P, G3, TS)
    return c.items


def _graph_cases(c: Cases, g):
    raw33, raw23, raw34 = g("grid4/3x3-raw"), g("grid4/2x3-raw"), g("grid4/3x4-raw")
    h66 = g("holes4/6x6_0.2_s1")
    G3, G2 = g("grid4/3x3"), g("grid4/2x2")
    # random start/goal generators
    c.add("graph", "getRandomStartGoal/holes6x6_n5", "Graph.getRandomStartGoal", {"g": h66, "number": 5}, seed=3,
          note="columns != 0 branch (x/y sampling)")
    c.add("graph", "getRandomStartGoal/holes8_6x6_n6", "Graph.getRandomStartGoal",
          {"g": g("holes8/6x6_0.2_s1"), "number": 6}, seed=4)
    c.add("graph", "getRandomStartGoal/converted4x4_n6", "Graph.getRandomStartGoal",
          {"g": g("grid4/4x4"), "number": 6}, seed=5, note="columns == 0 branch (converted graph)")
    c.add("graph", "getRandomStartGoal/raw2x3_n6_full", "Graph.getRandomStartGoal", {"g": raw23, "number": 6}, seed=1,
          note="number == |V| terminates")
    c.add("graph", "getRandomStartGoal/raw3x3_n0", "Graph.getRandomStartGoal", {"g": raw33, "number": 0}, seed=1)
    c.add("graph", "getRandomStartGoal/raw2x2_n5_hang", "Graph.getRandomStartGoal",
          {"g": g("grid8/2x2-raw"), "number": 5}, seed=1, budget=3,
          tags=["hang", "defect:random-sg-infinite-loop"], note="number > |V|: rejection loop never ends")
    c.add("graph", "getRandomStartGoal/converted2x2_n5_hang", "Graph.getRandomStartGoal", {"g": G2, "number": 5},
          seed=1, budget=3, tags=["hang", "defect:random-sg-infinite-loop"])
    c.add("graph", "getRandomStartGoalMany/3x3_n9", "Graph.getRandomStartGoalMany", {"g": G3, "number": 9}, seed=5)
    c.add("graph", "getRandomStartGoalMany/3x3_n8", "Graph.getRandomStartGoalMany", {"g": G3, "number": 8}, seed=6)
    c.add("graph", "getRandomStartGoalMany/spider_n8", "Graph.getRandomStartGoalMany", {"g": SPIDER, "number": 8},
          seed=6, note="Main case 6")
    c.add("graph", "getRandomStartGoalMany/2x2_n5_ioobe", "Graph.getRandomStartGoalMany", {"g": G2, "number": 5},
          seed=2, tags=["crash", "defect:random-sg-many-ioobe"])
    c.add("graph", "createGeneric2DGridGraphWithHoles/3x3_1.5_hang", "Graph.createGeneric2DGridGraphWithHoles",
          {"rows": 3, "cols": 3, "percentObstacle": 1.5}, seed=1, budget=3,
          tags=["hang", "defect:random-sg-infinite-loop"])
    c.add("graph", "createGeneric2DGridGraphWithHoles8Connected/2x2_0.5", "Graph.createGeneric2DGridGraphWithHoles8Connected",
          {"rows": 2, "cols": 2, "percentObstacle": 0.5}, seed=2, budget=10,
          note="8-connected grids with cols <= 2 have asymmetric adjacency (original-bugs B25)")
    # convertGraph (mutates start/goal)
    ids = [v[0] for v in h66["vertices"]]
    c.add("graph", "convertGraph/holes6x6", "Graph.convertGraph",
          {"graph": h66, "start": ids[:3], "goal": ids[-3:][::-1]})
    c.add("graph", "convertGraph/raw3x3", "Graph.convertGraph", {"graph": raw33, "start": [1, 9], "goal": [9, 5]})
    c.add("graph", "convertGraph/raw3x3_none", "Graph.convertGraph", {"graph": raw33, "start": [], "goal": []})
    c.add("graph", "convertGraph/raw3x3_badstart_npe", "Graph.convertGraph",
          {"graph": raw33, "start": [42], "goal": [1]}, tags=["crash"], note="unknown id -> null unboxing")
    # edges
    c.add("graph", "getEdgeSet/3x3", "Graph.getEdgeSet", {"g": G3})
    c.add("graph", "getEdgeSet/star", "Graph.getEdgeSet", {"g": STAR})
    c.add("graph", "getEdgeId", "Graph.getEdgeId", {"pairs": [[1, 2], [5, 3], [0, 0], [100000, 99999], [7, 123456789]]},
          note="returns [edgeId, getFirstVertexOfEdge, getSecondVertexOfEdge] per pair")
    # mutators
    c.add("graph", "removeVertex(Vertex)/3x3_4", "Graph.removeVertex(Vertex)", {"g": G3, "vId": 4})
    c.add("graph", "removeVertex(Vertex)/line3_leaf", "Graph.removeVertex(Vertex)", {"g": LINE3, "vId": 2},
          note="neighbour sets that become empty are removed from adjacencySetMap")
    c.add("graph", "removeVertex(Vertex)/absent_npe", "Graph.removeVertex(Vertex)", {"g": STAR, "vId": 9},
          tags=["crash"])
    c.add("graph", "removeVertex(int,int)/raw3x3_center", "Graph.removeVertex(int,int)", {"g": raw33, "x": 1, "y": 1})
    c.add("graph", "addVertex(Vertex)/star", "Graph.addVertex(Vertex)", {"g": STAR, "id": 4, "x": 3, "y": 2})
    c.add("graph", "addVertex(Vertex)/nopoint", "Graph.addVertex(Vertex)", {"g": STAR, "id": 5, "x": None})
    c.add("graph", "addVertex(int,int[])/star", "Graph.addVertex(int,int[])", {"g": STAR, "vId": 4, "neighbors": [3, 1]})
    c.add("graph", "finishBuildingGraph/stale", "Graph.finishBuildingGraph",
          {"g": {**STAR, "vertices": [[0], [1]], "idVertexMap_extra": [[2], [3]]}},
          note="'vertices' rebuilt from idVertexMap (input: stale array of 2, map of 4)")
    c.add("graph", "getId/raw3x4", "Graph.getId", {"g": raw34, "x": 2, "y": 1})
    c.add("graph", "getId/converted", "Graph.getId", {"g": G3, "x": 2, "y": 1}, note="columns == 0 after conversion")
    c.add("graph", "updateAdjList/star", "Graph.updateAdjList", {"g": STAR})
    c.add("graph", "isConnectedWithoutVertex/line5_mid", "Graph.isConnectedWithoutVertex", {"g": LINE5, "vId": 2})
    c.add("graph", "isConnectedWithoutVertex/line5_end", "Graph.isConnectedWithoutVertex", {"g": LINE5, "vId": 0})
    c.add("graph", "isConnectedWithoutVertex/3x3_center", "Graph.isConnectedWithoutVertex", {"g": G3, "vId": 4})
    c.add("graph", "isConnectedWithoutVertex/cycle5", "Graph.isConnectedWithoutVertex", {"g": CYCLE5, "vId": 0})
    c.add("graph", "isConnectedWithoutVertex/edgeless", "Graph.isConnectedWithoutVertex",
          {"g": g("grid4/3x1"), "vId": 1})
    c.add("graph", "isConnectedWithoutVertex/absent_npe", "Graph.isConnectedWithoutVertex", {"g": STAR, "vId": 7},
          tags=["crash"])
    c.add("graph", "areVerticesConnectedWithoutV3/line5", "Graph.areVerticesConnectedWithoutV3",
          {"g": LINE5, "vId1": 0, "vId2": 4, "vId3": 2})
    c.add("graph", "areVerticesConnectedWithoutV3/3x3", "Graph.areVerticesConnectedWithoutV3",
          {"g": G3, "vId1": 0, "vId2": 8, "vId3": 4})
    c.add("graph", "areVerticesConnectedWithoutV3/twocomp", "Graph.areVerticesConnectedWithoutV3",
          {"g": TWOCOMP, "vId1": 0, "vId2": 3, "vId3": 1})
    # printing
    c.add("graph", "printGraphAsAdjacenyList/star", "Graph.printGraphAsAdjacenyList", {"g": STAR})
    c.add("graph", "printGraphAsAdjacenyList/3x3", "Graph.printGraphAsAdjacenyList", {"g": G3})
    c.add("graph", "printGraphAsAdjacenyList/raw_npe", "Graph.printGraphAsAdjacenyList", {"g": raw23},
          tags=["crash"], note="ids start at 1 on unconverted grids")
    c.add("graph", "Vertex.print", "Vertex.print", {"g": G3, "vId": 5})
    c.add("graph", "Vertex.printVertex/point", "Vertex.printVertex", {"g": G3, "vId": 5})
    c.add("graph", "Vertex.printVertex/nopoint", "Vertex.printVertex", {"g": STAR, "vId": 2})
    c.add("graph", "Vertex.printVertex(StringBuffer)/point", "Vertex.printVertex(StringBuffer)", {"g": G3, "vId": 7})
    c.add("graph", "Vertex.printVertex(StringBuffer)/nopoint", "Vertex.printVertex(StringBuffer)", {"g": STAR, "vId": 2})
    c.add("graph", "Path.printPath/3x3", "Path.printPath", {"g": G3, "path": [0, 1, 2, 5]})
    c.add("graph", "Path.printPath/star", "Path.printPath", {"g": STAR, "path": [0, 1, 3]})


def _problem_cases(c: Cases, g, P):
    p3 = P("n2p3_s0")
    c.add("problem", "writeToFile(String)/n2p3", "Problem.writeToFile(String)", {"p": p3},
          note="private text writer: vertices line, edges line, start:goal line")
    c.add("problem", "writeToFile(String)/grid5x5", "Problem.writeToFile(String)", {"p": P("p5x5_20_4_s3")})
    c.add("problem", "writeToFile(String)/star_npe", "Problem.writeToFile(String)",
          {"p": {"graph": STAR, "sg": [[0, 3], [3, 0]]}}, tags=["crash", "defect:dat-write-null-point-npe"],
          note="vertices without coordinates: NPE, empty file left behind")
    c.add("problem", "writeToFile(Problem,String)/n2p3", "Problem.writeToFile(Problem,String)", {"p": p3},
          note="Java serialization (.dat); bytes in files['problem.dat'].base64")
    c.add("problem", "writeToFile(Problem,String)/p8c_6x6", "Problem.writeToFile(Problem,String)",
          {"p": P("p8c_6x6_10_6_s1")})
    c.add("problem", "writeToFile(Problem,String)/K18", "Problem.writeToFile(Problem,String)",
          {"p": {"graph": K18, "sg": [[1], [2]]}}, tags=["defect:dat-write-degree-aioobe"])
    c.add("problem", "writeToFile(Problem,String)/K19_aioobe", "Problem.writeToFile(Problem,String)",
          {"p": {"graph": K19, "sg": [[1], [2]]}}, tags=["crash", "defect:dat-write-degree-aioobe"])
    c.add("problem", "writeToFile(Problem,String)/star_npe", "Problem.writeToFile(Problem,String)",
          {"p": {"graph": STAR, "sg": [[0, 3], [3, 0]]}}, tags=["crash", "defect:dat-write-null-point-npe"])
    c.add("problem", "roundTrip/n2p3", "Problem.roundTrip", {"p": p3},
          note="Problem.writeToFile(p, f) then Problem.readFromFile(f) (Graph.writeObject/readObject)")
    c.add("problem", "roundTrip/p6x6", "Problem.roundTrip", {"p": P("p6x6_10_5_s2")})
    c.add("problem", "readFromFile/missing", "Problem.readFromFile", {"source": "missing"},
          tags=["defect:readfromfile-swallow"], note="exception printed and swallowed; graph and sg stay null")
    c.add("problem", "readFromFile/garbage", "Problem.readFromFile", {"source": "text", "data": "hello"},
          tags=["defect:readfromfile-swallow"])
    # batch generators (Problem.main and its helpers); FILE_FOLDER points into the case dir
    for fn, seed in [("createN2Puzzles", 1), ("createCrowdedHardProblems", 2), ("create32x32Problems", 3),
                     ("create32x32Problems8Connected", 4), ("create24x18PerformanceTestingProblems", 5),
                     ("createCrowdedHardProblems8Connected", 6)]:
        c.add("problem", f"batch/{fn}", f"Problem.{fn}", {"folder": "case", "samples": 1}, seed=seed, budget=900,
              note="digest of all generated files (names contain literal backslashes)")
    c.add("problem", "batch/main", "Problem.main", {"argv": ["$BATCH"], "folder": "none", "samples": 2}, seed=7,
          budget=900, note="all five generators in sequence, then sleeps 5 s")
    c.add("problem", "batch/createN2Puzzles_default_folder_npe", "Problem.createN2Puzzles", {"folder": "default"},
          seed=1, tags=["crash", "defect:problem-main-linux-npe"],
          note="default FILE_FOLDER has no '/', so getParentFile() is null on Linux")
    c.add("problem", "createGridProblem/2x2_n5_hang", "Problem.createGridProblem",
          {"col": 2, "row": 2, "fracObs": 0.0, "n": 5}, seed=1, budget=3,
          tags=["hang", "defect:random-sg-infinite-loop"])
    c.add("problem", "createN2Puzzle/2", "Problem.createN2Puzzle", {"n": 2}, seed=3)
    c.add("problem", "createN2M1Puzzle/2", "Problem.createN2M1Puzzle", {"n": 2}, seed=3)


def _pathfinder_cases(c: Cases, g, P, G3, TS):
    p8 = P("p8x8_0_10_s1")
    pg, ps, pt = problem_parts(p8)
    ph, phs, pht = problem_parts(P("p10x10_20_12_s5"))
    for v in (0, 4):
        c.add("pathfinder", f"getDistancesToAllVertices/3x3_{v}", "PathFinder.getDistancesToAllVertices",
              {"g": G3, "start": v, "inputDistances": None})
    c.add("pathfinder", "getDistancesToAllVertices/spider_5", "PathFinder.getDistancesToAllVertices",
          {"g": SPIDER, "start": 5, "inputDistances": None})
    c.add("pathfinder", "getDistancesToAllVertices/twocomp", "PathFinder.getDistancesToAllVertices",
          {"g": TWOCOMP, "start": 0, "inputDistances": None}, note="unreachable vertices get 0")
    c.add("pathfinder", "getDistancesToAllVertices/prefilled", "PathFinder.getDistancesToAllVertices",
          {"g": TWOCOMP, "start": 1, "inputDistances": [9, 9, 9, 9]})
    c.add("pathfinder", "getDistancesToAllVertices/holes10x10", "PathFinder.getDistancesToAllVertices",
          {"g": ph, "start": phs[0], "inputDistances": None})
    for v, lo, hi in [(4, 0, 1), (4, 1, 1), (0, 2, 2), (0, 0, 100), (4, 3, 2), (8, 1, 3)]:
        c.add("pathfinder", f"findGoalsInDistanceRange/3x3_{v}_{lo}_{hi}", "PathFinder.findGoalsInDistanceRange",
              {"g": G3, "start": v, "dMin": lo, "dMax": hi}, note="result in HashSet iteration order")
    c.add("pathfinder", "findGoalsInDistanceRange/8x8_10_3_5", "PathFinder.findGoalsInDistanceRange",
          {"g": pg, "start": 10, "dMin": 3, "dMax": 5})
    c.add("pathfinder", "findGoalsInDistanceRange/holes_2_4", "PathFinder.findGoalsInDistanceRange",
          {"g": ph, "start": phs[1], "dMin": 2, "dMax": 4})
    for seed in (0, 1, 2):
        c.add("pathfinder", f"findShortestPaths/3x3_s{seed}", "PathFinder.findShortestPaths",
              {"g": G3, "start": TS[0], "goal": TS[1]}, seed=seed)
    c.add("pathfinder", "findShortestPaths/8x8_10", "PathFinder.findShortestPaths",
          {"g": pg, "start": ps, "goal": pt}, seed=3)
    c.add("pathfinder", "findShortestPaths/holes10x10", "PathFinder.findShortestPaths",
          {"g": ph, "start": phs, "goal": pht}, seed=4)
    c.add("pathfinder", "findShortestPaths/a9", "PathFinder.findShortestPaths",
          {"g": P("a9")["graph"], "start": P("a9")["sg"][0], "goal": P("a9")["sg"][1]}, seed=5)
    c.add("pathfinder", "findShortestPaths/unreachable", "PathFinder.findShortestPaths",
          {"g": TWOCOMP, "start": [0, 0], "goal": [3, 1]}, seed=6, tags=["defect:unreachable-goal-npe"],
          note="unreachable goal -> null entry")
    c.add("pathfinder", "findShortestPaths/bHeuristic_3x3", "PathFinder.findShortestPaths",
          {"g": G3, "start": TS[0], "goal": TS[1]}, seed=7, statics={"PathFinder.bHeuristic": True},
          tags=["defect:bheuristic-leak-nonoptimal"], note="random queue insertion: paths need not be shortest")
    c.add("pathfinder", "findShortestPaths/bHeuristic_8x8", "PathFinder.findShortestPaths",
          {"g": pg, "start": ps, "goal": pt}, seed=8, statics={"PathFinder.bHeuristic": True},
          tags=["defect:bheuristic-leak-nonoptimal"])
    c.add("pathfinder", "findOneShortestPath/same", "PathFinder.findOneShortestPath",
          {"g": G3, "start": 4, "goal": 4, "alreadyVisitedVertices": None}, seed=1)
    c.add("pathfinder", "findOneShortestPath/3x3", "PathFinder.findOneShortestPath",
          {"g": G3, "start": 0, "goal": 8, "alreadyVisitedVertices": None}, seed=2)
    c.add("pathfinder", "findOneShortestPath/visited", "PathFinder.findOneShortestPath",
          {"g": G3, "start": 0, "goal": 8, "alreadyVisitedVertices": [1, 4]}, seed=3,
          note="caller-provided visited set is used (and mutated)")
    c.add("pathfinder", "findOneShortestPath/unreachable", "PathFinder.findOneShortestPath",
          {"g": TWOCOMP, "start": 0, "goal": 3, "alreadyVisitedVertices": None}, seed=4)
    paths = [[0, 1, 1, 2], [3, 3, 3, 3], [5, 4, 5, 4]]
    c.add("pathfinder", "getTotalDistance(int[][])", "PathFinder.getTotalDistance(int[][])", {"paths": paths})
    c.add("pathfinder", "getTotalDistance(Path[])", "PathFinder.getTotalDistance(Path[])",
          {"paths": [[0, 1, 2], [3], [5, 4, 5, 4]]})
    for name, (gg, s, t), seed in [("3x3", (G3, TS[0], TS[1]), 1), ("8x8", (pg, ps, pt), 2),
                                   ("same", (G3, [4, 0], [4, 0]), 3)]:
        c.add("pathfinder", f"getMakespanLowerBound/{name}", "PathFinder.getMakespanLowerBound",
              {"g": gg, "start": s, "goal": t}, seed=seed, note="returns the max VERTEX count (edges + 1)")
        c.add("pathfinder", f"getTotalTimeLowerBound/{name}", "PathFinder.getTotalTimeLowerBound",
              {"g": gg, "start": s, "goal": t}, seed=seed)
    for fn in ("getMakespanLowerBound", "getTotalTimeLowerBound"):
        c.add("pathfinder", f"{fn}/unreachable_npe", f"PathFinder.{fn}",
              {"g": TWOCOMP, "start": [0], "goal": [3]}, seed=1, tags=["crash", "defect:unreachable-goal-npe"])


def _model_cases(c: Cases, g, P, G3, G6, TS):
    T = "MultiagentGraphSolverGurobiTime"
    R = "MultiagentGraphSolverGurobiTimeR"
    Z = "MultiagentGraphSolverGurobiTimePuzzle"
    TT = "MultiagentGraphSolverGurobiTotalTime"
    DI = "MultiagentGraphSolverGurobiDistance"
    st = {**ALL_QUIET}

    def model(cid, cls, graph, s, t, T_, integer=True, setOptimal=True, timeLimit=-1.0, note=None, tags=(), **kw):
        args = {"graph": graph, "starts": s, "goals": t, "integer": integer, "timeSteps": T_}
        if cls in (TT, DI):
            args["timeLimit"] = timeLimit
        elif cls == Z:
            args["setOptimal"] = setOptimal
        else:
            args["setOptimal"] = setOptimal
            args["timeLimit"] = timeLimit
        args.update(kw)  # e.g. gap (set on the environment, as solve() does)
        c.add("models", cid, f"{cls}.prepareModel", args, statics=st, note=note, tags=tags)

    swap = ([0, 3], [3, 0])
    model("Time/star_T3", T, STAR, *swap, 3)
    model("Time/star_T3_lp", T, STAR, *swap, 3, integer=False, setOptimal=False)
    model("Time/star_T2", T, STAR, *swap, 2)
    model("Time/stay_T0", T, STAR, [0, 3], [0, 3], 0, note="T = 0: -x + x merges to an empty row")
    model("Time/star_T3_tl", T, STAR, *swap, 3, timeLimit=12.5)
    model("Time/3x3_T4", T, G3, *TS, 4)
    model("Time/3x3_4agents_T6", T, G3, [0, 2, 6, 8], [8, 6, 2, 0], 6)
    model("Time/a9_T4", T, P("a9")["graph"], *P("a9")["sg"], 4)
    model("Time/6x6_swap_T10", T, G6, [30, 35], [35, 30], 10, note="vertex ids >= 22 (edge-id int overflow)")
    model("Time/fig8_T4", T, g("grid4/4x4"), [13, 8], [0, 7], 4, note="TOR Fig. 8 instance")
    model("Time/comb_T18", T, COMB, [0], [144], 18, note="HashSet bin-overflow resize changes the column order")
    pg, ps, pt = problem_parts(P("p8x8_20_8_s4"))
    model("Time/p8x8_20_8_lb", T, pg, ps, pt, makespan_lb(pg, ps, pt))
    pg2, ps2, pt2 = problem_parts(P("p10x10_20_12_s5"))
    model("Time/p10x10_20_12_lb1", T, pg2, ps2, pt2, makespan_lb(pg2, ps2, pt2) + 1)
    model("Time/dupgoal_T3", T, STAR, [0, 2], [3, 3], 3, note="duplicate goals")
    model("TimeR/star_T6", R, STAR, *swap, 6)
    model("TimeR/3x3_T8", R, G3, [0, 2], [8, 6], 8)
    model("TimeR/6x6_swap_T10", R, G6, [30, 35], [35, 30], 10, tags=["defect:timer-overflow-single-move"],
          note="arcs leaving ids >= 22 are missing from the one-move rows")
    model("TimePuzzle/star_T4", Z, STAR, *swap, 4)
    model("TimePuzzle/3x3_T4_lp", Z, G3, *TS, 4, integer=False, setOptimal=False)
    model("TotalTime/star_T4", TT, STAR, *swap, 4)
    model("TotalTime/line3_T2", TT, LINE3, [0], [2], 2)
    model("TotalTime/3x3_T5", TT, G3, *TS, 5, gap=0.01)
    model("TotalTime/stay_T0", TT, STAR, [0], [0], 0)
    model("Distance/star_T4", DI, STAR, *swap, 4)
    model("Distance/cycle5_T4", DI, CYCLE5, [0, 2], [2, 4], 4, gap=0.05)
    model("Distance/3x3_T5_lp", DI, G3, *TS, 5, integer=False)
    for cls in (T, R):
        c.add("models", f"{cls.split('Gurobi')[1]}/keyHelpers", f"{cls}.keyHelpers",
              {"tuples": [[0, 21, 22, 3], [0, 22, 21, 3], [1, 22, 23, 5], [0, 3, 4, 0], [0, 2, 3, 0], [0, 30, 24, 0],
                          [1, 35, 34, 0], [2, 1590, 1591, 7], [0, 0, 0, 9999], [3, 214748, 5, 1]]},
              note="getEdgeId, getVertexTimeId, getAgentVertexTimeId, getEdgeTimeId (+ TimeR decoders) with int wrap")
    for cls, name in [(T, "Time"), (TT, "TotalTime"), (DI, "Distance"), ("PuzzleSolver", "PuzzleSolver")]:
        c.add("models", f"{name}/updateReachableSets_star_T3", f"{cls}.updateReachableSets",
              {"graph": STAR, "starts": [0, 3], "goals": [3, 0], "timeSteps": 3})
    c.add("models", "Time/updateReachableSets_3x3_T5", f"{T}.updateReachableSets",
          {"graph": G3, "starts": TS[0], "goals": TS[1], "timeSteps": 5})
    c.add("models", "TimeR/updateReachableSets_twocomp", f"{R}.updateReachableSets",
          {"graph": TWOCOMP, "starts": [0], "goals": [1], "timeSteps": 2})


def _solver_cases(c: Cases, g, P, G3, G6, TS):
    T = "MultiagentGraphSolverGurobiTime"
    R = "MultiagentGraphSolverGurobiTimeR"
    TT = "MultiagentGraphSolverGurobiTotalTime"
    DI = "MultiagentGraphSolverGurobiDistance"
    swap = ([0, 3], [3, 0])

    def tsolve(cid, cls, graph, s, t, relax=False, setOptimal=True, extra=0, has=False, tl=-1.0, statics=None,
               seed=1, **kw):
        c.add("solvers", cid, f"{cls}.solve",
              {"graph": graph, "starts": s, "goals": t, "solveRelaxedFirst": relax, "setOptimal": setOptimal,
               "extraSteps": extra, "hasTimeLimit": has, "timeLimit": tl},
              seed=seed, statics=QUIET if statics is None else statics, **kw)

    tsolve("Time/star", T, STAR, *swap)
    tsolve("Time/star_relaxed_debug", T, STAR, *swap, relax=True, statics={}, note="bDebugInfo/bPrintPaths on")
    tsolve("Time/star_relaxed_tl", T, STAR, *swap, relax=True, has=True, tl=BIG,
           statics={"MultiagentGraphSolverGurobiTime.bDebugInfo": True, "MultiagentGraphSolverGurobiTime.bPrintPaths": False})
    tsolve("Time/star_notopt", T, STAR, *swap, setOptimal=False)
    tsolve("Time/star_extra2", T, STAR, *swap, extra=2, relax=True, note="extraSteps > 0 skips the LP phase")
    tsolve("Time/3x3", T, G3, *TS)
    tsolve("Time/3x3_relaxed", T, G3, *TS, relax=True)
    tsolve("Time/a9", T, P("a9")["graph"], *P("a9")["sg"], note="TOR Fig. 1/3 9-puzzle (makespan 4)")
    tsolve("Time/fig8", T, g("grid4/4x4"), [13, 8], [0, 7], note="TOR Fig. 8 unsplit (makespan 4)")
    for name in ("p8x8_0_10_s1", "p6x6_10_5_s2", "p8c_6x6_10_6_s1", "p10x10_20_12_s5"):
        tsolve(f"Time/{name}", T, *problem_parts(P(name)), seed=2)
    tsolve("Time/n2p3_s1", T, *problem_parts(P("n2p3_s1")), relax=True, seed=3)
    tsolve("Time/stay", T, STAR, [0, 3], [0, 3], note="T = 0")
    tsolve("Time/hasTL_nonpositive_star", T, STAR, *swap, has=True, tl=-1.0,
           tags=["defect:hastimelimit-nonpositive"], note="returns null at the first infeasible T")
    tsolve("Time/hasTL_zero_star", T, STAR, *swap, has=True, tl=0.0, tags=["defect:hastimelimit-nonpositive"],
           nondet=["result"], note="the original tests 0 - elapsed < 0: null unless no millisecond elapsed")
    tsolve("Time/hasTL_nonpositive_relaxed_3x3", T, G3, *TS, relax=True, has=True, tl=-1.0,
           tags=["defect:hastimelimit-nonpositive"], note="relaxed loop returns null even when feasible")
    tsolve("Time/dupstarts", T, STAR, [0, 0], [2, 3], tags=["defect:vertex-cap-t0-unconstrained"],
           note="colliding paths at t=0 are returned")
    tsolve("Time/unreachable_npe", T, TWOCOMP, [0], [3], tags=["crash", "defect:unreachable-goal-npe"])
    tsolve("Time/swap_line2_hang", T, LINE2, [0, 1], [1, 0], budget=8,
           tags=["hang", "defect:minmakespan-no-tmax"], note="infeasible: T grows forever")
    tsolve("Time/dupgoals_hang", T, STAR, [0, 2], [3, 3], budget=8,
           tags=["hang", "defect:vertex-cap-t0-unconstrained"], note="duplicate goals: every T infeasible")
    rq = {"MultiagentGraphSolverGurobiTimeR.bDebugInfo": False, "MultiagentGraphSolverGurobiTimeR.bPrintPaths": False}
    tsolve("TimeR/star", R, STAR, *swap, statics=rq, note="one move per step: makespan 6")
    tsolve("TimeR/star_debug", R, STAR, *swap, relax=True, statics={})
    tsolve("TimeR/3x3_2agents", R, G3, [0, 2], [8, 6], statics=rq)
    tsolve("TimeR/6x6_swap_high_ids", R, G6, [30, 35], [35, 30], statics=rq, budget=300,
           tags=["defect:timer-overflow-single-move"], note="ids >= 22 escape the one-move rows: makespan 10")
    tsolve("TimeR/6x6_swap_low_ids", R, G6, [0, 5], [5, 0], statics=rq, budget=300,
           tags=["defect:timer-overflow-single-move"], note="mirror instance: makespan 12")

    def ttsolve(cid, graph, s, t, steps, gap=0.0, has=False, tl=-1.0, opt=(-777,), statics=None, seed=1, **kw):
        c.add("solvers", cid, f"{TT}.solve",
              {"graph": graph, "starts": s, "goals": t, "opt": list(opt), "gap": gap, "timeSteps": steps,
               "hasTimeLimit": has, "timeLimit": tl}, seed=seed,
              statics={"MultiagentGraphSolverGurobiTotalTime.bPrintPaths": False} if statics is None else statics, **kw)

    ttsolve("TotalTime/star_T4", STAR, *swap, 4)
    ttsolve("TotalTime/star_T6_debug", STAR, *swap, 6, statics={})
    ttsolve("TotalTime/3x3_T4", G3, *TS, 4)
    ttsolve("TotalTime/3x3_T6_tl", G3, *TS, 6, has=True, tl=BIG)
    ttsolve("TotalTime/line3_T3", LINE3, [0], [2], 3)
    ttsolve("TotalTime/line3_T2_nan", LINE3, [0], [2], 2, tags=["defect:tt-nan-gap-zero-objective"],
            note="optimum 0 -> gap NaN -> rejected, opt untouched")
    ttsolve("TotalTime/line3_T2_any", LINE3, [0], [2], 2,
            statics={"MultiagentGraphSolverGurobiTotalTime.bPrintPaths": False,
                     "MultiagentGraphSolverGurobiTotalTime.bReturnAnySolution": True},
            tags=["defect:tt-nan-gap-zero-objective"])
    ttsolve("TotalTime/cycle3_rotation_T1_nan", CYCLE3, [0, 1, 2], [1, 2, 0], 1, tags=["defect:tt-nan-gap-zero-objective"])
    ttsolve("TotalTime/infeasible_T1", STAR, *swap, 1, note="no solution at this horizon")
    pg, ps, pt = problem_parts(P("p8x8_0_10_s1"))
    ttsolve("TotalTime/p8x8_0_10_lb2", pg, ps, pt, makespan_lb(pg, ps, pt) + 2, gap=0.0, seed=2)
    pg, ps, pt = problem_parts(P("p6x6_10_5_s2"))
    ttsolve("TotalTime/p6x6_10_5_lb1", pg, ps, pt, makespan_lb(pg, ps, pt) + 1, gap=0.0, seed=3)

    def dsolve(cid, graph, s, t, steps, gap=0.0, tl=-1.0, statics=None, seed=1, **kw):
        c.add("solvers", cid, f"{DI}.solve",
              {"graph": graph, "starts": s, "goals": t, "gap": gap, "expansion": steps, "timeLimit": tl}, seed=seed,
              statics={} if statics is None else statics, **kw)

    dsolve("Distance/star_T4", STAR, *swap, 4)
    dsolve("Distance/star_T6_tl", STAR, *swap, 6, tl=BIG)
    dsolve("Distance/3x3_T4", G3, *TS, 4)
    dsolve("Distance/3x3_T6", G3, *TS, 6, statics={"MultiagentGraphSolverGurobiDistance.bPrintPaths": True})
    dsolve("Distance/stay_T3_nan", STAR, [0, 3], [0, 3], 3, tags=["defect:dist-nan-gap-zero-objective"])
    dsolve("Distance/stay_T0_nan", STAR, [0, 3], [0, 3], 0, tags=["defect:dist-nan-gap-zero-objective"])
    dsolve("Distance/stay_T3_any", STAR, [0, 3], [0, 3], 3,
           statics={"MultiagentGraphSolverGurobiDistance.bReturnAnySolution": True},
           tags=["defect:dist-nan-gap-zero-objective"])
    dsolve("Distance/infeasible_T1", STAR, *swap, 1)
    pg, ps, pt = problem_parts(P("p8x8_0_10_s1"))
    dsolve("Distance/p8x8_0_10_lb2", pg, ps, pt, makespan_lb(pg, ps, pt) + 2, seed=2)
    pg, ps, pt = problem_parts(P("p8c_6x6_10_6_s1"))
    dsolve("Distance/p8c_6x6_lb1", pg, ps, pt, makespan_lb(pg, ps, pt) + 1, seed=3)

    # PuzzleSolver: a two-thread race; only the makespan is deterministic.
    c.add("solvers", "PuzzleSolver.solve(double)/3x3", "PuzzleSolver.solve(double)",
          {"graph": G3, "starts": [0, 6], "goals": [2, 8], "timeLimit": -1.0}, seed=1, budget=60,
          nondet=["result"], note="whichever model finishes first wins; makespan 2")
    c.add("solvers", "PuzzleSolver.solve(double)/star_deadlock", "PuzzleSolver.solve(double)",
          {"graph": STAR, "starts": [0, 3], "goals": [3, 0], "timeLimit": 10.0}, seed=1, budget=25,
          tags=["hang", "defect:puzzle-deadlock-both-fail"],
          note="both workers fail at the first T; the wait loop never ends (time limit ignored)")
    for name in ("n2p3_s0", "n2p4_s1", "n2m1p3_s2"):
        c.add("solvers", f"PuzzleSolver.solve(Problem,double)/{name}", "PuzzleSolver.solve(Problem,double)",
              {"p": P(name), "timeLimit": -1.0}, seed=4, statics=QUIET, budget=300,
              note="<= 16 agents: PathPlanner.planPathsAdvanced(..., setOptimal=true)")


def _planner_cases(c: Cases, g, P, G3, G2, G4, G8, TS):
    pp = "PathPlanner"
    p8 = P("p8x8_0_10_s1")
    pg, ps, pt = problem_parts(p8)
    paths_ok = [[0, 1, 2, 5], [6, 3, 4, 4], [8, 8, 7, 6]]
    # --- printing / small utilities
    c.add("planner", "printPaths/3x3", f"{pp}.printPaths", {"g": G3, "paths": paths_ok})
    c.add("planner", "printPaths/star_nopoint", f"{pp}.printPaths", {"g": STAR, "paths": [[0, 0, 1, 3, 3], [3, 1, 2, 1, 0]]})
    c.add("planner", "printPaths/empty", f"{pp}.printPaths", {"g": G3, "paths": []}, tags=["defect:empty-input-crash"])
    c.add("planner", "printPaths/null_npe", f"{pp}.printPaths", {"g": G3, "paths": None},
          tags=["crash", "defect:empty-input-crash"])
    c.add("planner", "printPaths/ragged_aioobe", f"{pp}.printPaths", {"g": G3, "paths": [[0, 1, 2], [3, 4]]},
          tags=["crash", "defect:empty-input-crash"])
    many = [[i, i] for i in range(11)]
    c.add("planner", "printPathToString/11agents", f"{pp}.printPathToString", {"g": g("grid4/4x4"), "paths": many},
          note="agent numbers are 1-based and padded below 10")
    c.add("planner", "printPathToString/star", f"{pp}.printPathToString", {"g": STAR, "paths": [[0, 1], [3, 3]]})
    c.add("planner", "printStartAndGoals/3x3", f"{pp}.printStartAndGoals", {"g": G3, "start": TS[0], "goal": TS[1]})
    c.add("planner", "printStartAndGoals/star", f"{pp}.printStartAndGoals", {"g": STAR, "start": [0, 3], "goal": [3, 0]})
    c.add("planner", "numberOfGoalsReached", f"{pp}.numberOfGoalsReached", {"paths": paths_ok, "goal": [5, 4, 7]})
    c.add("planner", "parsePaths/ragged", f"{pp}.parsePaths", {"paths": [[0, 1, 2], [5], [7, 8]]})
    c.add("planner", "parsePaths/null_npe", f"{pp}.parsePaths", {"paths": [[0, 1], None]},
          tags=["crash", "defect:unreachable-goal-npe"])
    for name, paths in [("rotation", [[0, 1], [1, 2], [2, 0]]), ("none", paths_ok),
                        ("two", [[0, 1, 0], [1, 0, 1], [5, 5, 5]]), ("swap2", [[0, 1], [1, 0]]),
                        ("chain", [[0, 1], [1, 2]]), ("rows0", [[], []]), ("single", [[3]])]:
        c.add("planner", f"hasCycles/{name}", f"{pp}.hasCycles", {"paths": paths})
    c.add("planner", "hasCycles/empty_aioobe", f"{pp}.hasCycles", {"paths": []}, tags=["crash", "defect:empty-input-crash"])
    c.add("planner", "hasCycles/null_npe", f"{pp}.hasCycles", {"paths": None}, tags=["crash", "defect:empty-input-crash"])
    valid = [
        ("ok", paths_ok, [0, 6, 8], [5, 4, 6]),
        ("badstart", paths_ok, [1, 6, 8], [5, 4, 6]),
        ("badgoal", paths_ok, [0, 6, 8], [5, 4, 7]),
        ("nonadjacent", [[0, 2]], [0], [2]),
        ("meet", [[0, 1], [2, 1]], [0, 2], [1, 1]),
        ("headon", [[0, 1], [1, 0]], [0, 1], [1, 0]),
        ("follow", [[0, 1, 2], [1, 2, 5]], [0, 1], [2, 5]),
        ("rows0", [[], []], [0, 3], [3, 0]),
        ("ragged_short", [[0, 1, 2], [3, 4]], [0, 3], [2, 4]),
        ("ragged_long", [[0, 1], [3, 3, 4, 5]], [0, 3], [1, 5]),
    ]
    for name, paths, s, t in valid:
        c.add("planner", f"isPathSetValid/{name}", f"{pp}.isPathSetValid",
              {"paths": paths, "graph": G3, "starts": s, "goals": t},
              tags=["defect:empty-input-crash"] if name.startswith(("rows0", "ragged")) else ())
    c.add("planner", "isPathSetValid/empty_aioobe", f"{pp}.isPathSetValid",
          {"paths": [], "graph": G3, "starts": [], "goals": []}, tags=["crash", "defect:empty-input-crash"])
    c.add("planner", "isPathSetValid/null_npe", f"{pp}.isPathSetValid",
          {"paths": None, "graph": G3, "starts": [], "goals": []}, tags=["crash", "defect:empty-input-crash"])
    c.add("planner", "mergePaths/ok", f"{pp}.mergePaths",
          {"pathFromStart": [[0, 1], [2, 2]], "pathFromMiddle": [[1, 4, 7], [2, 5, 8]]})
    c.add("planner", "mergePaths/discontinuous", f"{pp}.mergePaths",
          {"pathFromStart": [[0, 1]], "pathFromMiddle": [[6, 7]]}, note="no continuity check")
    c.add("planner", "mergePaths/null", f"{pp}.mergePaths", {"pathFromStart": None, "pathFromMiddle": [[1]]})
    c.add("planner", "moveAgentsIndependently/free", f"{pp}.moveAgentsIndependently", {"paths": [[0, 1, 2], [6, 7, 8]]})
    c.add("planner", "moveAgentsIndependently/meet", f"{pp}.moveAgentsIndependently",
          {"paths": [[0, 1, 2, 2], [6, 3, 4, 5], [8, 7, 4, 1]]})
    for name, (s, t), seed in [("3x3", TS, 1), ("swap", ([0, 1], [1, 0]), 2)]:
        c.add("planner", f"moveAgentsIndependentlyWhenCan/{name}", f"{pp}.moveAgentsIndependentlyWhenCan",
              {"g": G3, "start": s, "goal": t, "pathArray": [[v] for v in s]}, seed=seed)
    c.add("planner", "moveAgentsIndependentlyWhenCan/8x8", f"{pp}.moveAgentsIndependentlyWhenCan",
          {"g": pg, "start": ps, "goal": pt, "pathArray": [[v] for v in ps]}, seed=3)
    # createSubProblem around a conflict (vertex 4 of the 3x3 grid, entered by agents at 1 and 3)
    sub_paths = [[1, 4, 7], [3, 4, 5], [8, 8, 8]]
    for att in (1, 2, 3):
        c.add("planner", f"createSubProblem/3x3_attempt{att}", f"{pp}.createSubProblem",
              {"g": G3, "paths": sub_paths, "svs": [1, 3], "goal": [7, 5, 8], "distance": 2, "attempt": att,
               "distanceToGoals": "fromGoals"}, seed=att,
              note="attempt odd: rotate goals among svs; even: nearest unoccupied vertex to each goal")
    c.add("planner", "createSubProblem/8x8_distance3", f"{pp}.createSubProblem",
          {"g": pg, "paths": [[ps[0], 9, 10], [ps[1], 9, 8]], "svs": [ps[0], ps[1]], "goal": pt[:2], "distance": 3,
           "attempt": 2, "distanceToGoals": "fromGoals"}, seed=5)
    c.add("planner", "createSubProblem/no_distance_table_npe", f"{pp}.createSubProblem",
          {"g": G3, "paths": sub_paths, "svs": [1, 3], "goal": [7, 5, 8], "distance": 2, "attempt": 2,
           "distanceToGoals": None}, seed=1, tags=["crash", "defect:distanceToGoals-null-npe"])
    # split helpers
    for seed, rnd in [(1, False), (2, True), (3, True)]:
        c.add("planner", f"splitPaths/3x3_{'random' if rnd else 'sorted'}_s{seed}", f"{pp}.splitPaths",
              {"g": G3, "start": TS[0], "goal": TS[1], "random": rnd}, seed=seed)
    c.add("planner", "splitPaths/fig8", f"{pp}.splitPaths", {"g": G4, "start": [13, 8], "goal": [0, 7], "random": False},
          seed=1, tags=["defect:fig8-split-child-needs-3"], note="TOR Fig. 8: intermediate goals [5, 10]")
    c.add("planner", "splitPaths/8x8", f"{pp}.splitPaths", {"g": pg, "start": ps, "goal": pt, "random": False}, seed=4)
    c.add("planner", "splitPaths/8x8_random", f"{pp}.splitPaths", {"g": pg, "start": ps, "goal": pt, "random": True},
          seed=5)
    c.add("planner", "splitPaths/square_hang", f"{pp}.splitPaths",
          {"g": SQUARE, "start": [0, 2, 3], "goal": [3, 1, 2], "random": False}, seed=1, budget=3,
          tags=["hang", "defect:splitpaths-infinite-loop"])
    c.add("planner", "splitPaths/square_random_ok", f"{pp}.splitPaths",
          {"g": SQUARE, "start": [0, 2, 3], "goal": [3, 1, 2], "random": True}, seed=0, budget=3,
          tags=["defect:splitpaths-infinite-loop"], note="a different order avoids the hang")
    c.add("planner", "splitPaths/line3_swap_hang", f"{pp}.splitPaths",
          {"g": LINE3, "start": [0, 2], "goal": [2, 0], "random": False}, seed=1, budget=3,
          tags=["hang", "defect:nocycle-unbounded-retry-hang"])
    n11 = P("n2p3_s11")
    c.add("planner", "splitPaths/n2p3_s11_hang", f"{pp}.splitPaths",
          {"g": n11["graph"], "start": n11["sg"][0], "goal": n11["sg"][1], "random": False}, seed=1, budget=3,
          tags=["hang", "defect:splitpaths-infinite-loop"])
    for size in (2, 3, 4, 5):
        c.add("planner", f"splitPathBySize/3x3_size{size}", f"{pp}.splitPathBySize",
              {"g": G3, "start": TS[0], "goal": TS[1], "random": False, "firstSize": size}, seed=size, budget=3,
              tags=["hang", "defect:splitbysize-infinite-loop"] if size == 2 else ())
    c.add("planner", "splitPathBySize/3x3_random_size3", f"{pp}.splitPathBySize",
          {"g": G3, "start": TS[0], "goal": TS[1], "random": True, "firstSize": 3}, seed=6, budget=15,
          tags=["hang", "defect:splitbysize-infinite-loop"],
          note="random order: every candidate already used and dMax never grows")
    for name, s, t in [("a", [0, 1], [4, 6]), ("b", [1, 0], [6, 4]), ("c", [0], [4]), ("d", [0, 0], [3, 5]),
                       ("e", [3], [3])]:
        c.add("planner", f"getLongestPathLength/line8_{name}", f"{pp}.getLongestPathLength",
              {"g": LINE8, "start": s, "goal": t}, seed=1, tags=["defect:longest-path-off-by-one"])

    # --- planners
    def plan(cid, fn, args, seed=1, statics=QUIET, **kw):
        c.add("planner", cid, f"{pp}.{fn}", args, seed=seed, statics=statics, **kw)

    adv = lambda gg, s, t, relax=False, extra=0, opt=True, tl=-1.0: {  # noqa: E731
        "g": gg, "start": s, "goal": t, "solveRelaxed": relax, "extraSteps": extra, "setOptimal": opt, "timeLimit": tl}
    plan("planPathsAdvanced/star", "planPathsAdvanced", adv(STAR, [0, 3], [3, 0], relax=True))
    plan("planPathsAdvanced/3x3", "planPathsAdvanced", adv(G3, *TS))
    plan("planPathsAdvanced/3x3_tl", "planPathsAdvanced", adv(G3, *TS, tl=BIG))
    plan("planPathsAdvanced/8x8", "planPathsAdvanced", adv(pg, ps, pt), seed=2)
    plan("planPathsAdvanced/n2m1p3_s0", "planPathsAdvanced", adv(*problem_parts(P("n2m1p3_s0"))), seed=3)
    plan("planPathsAdvanced/fig8", "planPathsAdvanced", adv(G4, [13, 8], [0, 7]))
    plan("planPathsAdvanced/dupstarts", "planPathsAdvanced", adv(STAR, [0, 0], [2, 3]),
         tags=["defect:vertex-cap-t0-unconstrained"], note="invalid path set rejected -> null")
    plan("planPathsAdvanced/debug", "planPathsAdvanced", adv(STAR, [0, 3], [3, 0]), statics={},
         note="bDebugInfo/bPrintPaths default (true)")
    plan("planPathsAdvancedPMG/star", "planPathsAdvancedPMG", adv(STAR, [0, 3], [3, 0]),
         statics={**QUIET, "MultiagentGraphSolverGurobiTimeR.bDebugInfo": False,
                  "MultiagentGraphSolverGurobiTimeR.bPrintPaths": False})
    plan("planPathsAdvancedPMG/3x3", "planPathsAdvancedPMG", adv(G3, [0, 2], [8, 6]),
         statics={**QUIET, "MultiagentGraphSolverGurobiTimeR.bDebugInfo": False,
                  "MultiagentGraphSolverGurobiTimeR.bPrintPaths": False})
    ttq = {**QUIET, "MultiagentGraphSolverGurobiTotalTime.bDebugInfo": False,
           "MultiagentGraphSolverGurobiTotalTime.bPrintPaths": False}
    plan("planPathsAdvancedTT/3x3_T4", "planPathsAdvancedTT",
         {"g": G3, "start": TS[0], "goal": TS[1], "opt": [0], "gap": 0.0, "timeSteps": 4, "timeLimit": -1.0}, statics=ttq)
    plan("planPathsAdvancedTT/3x3_T6_tl", "planPathsAdvancedTT",
         {"g": G3, "start": TS[0], "goal": TS[1], "opt": [0], "gap": 0.0, "timeSteps": 6, "timeLimit": BIG}, statics=ttq)
    plan("planPathsAdvancedTT/line3_T2_nan", "planPathsAdvancedTT",
         {"g": LINE3, "start": [0], "goal": [2], "opt": [-777], "gap": 0.0, "timeSteps": 2, "timeLimit": -1.0},
         statics={"MultiagentGraphSolverGurobiTotalTime.bPrintPaths": False},
         tags=["defect:tt-nan-gap-zero-objective"])
    dq = {**QUIET, "MultiagentGraphSolverGurobiDistance.bDebugInfo": False}
    plan("planPathsAdvancedTD/3x3_T4", "planPathsAdvancedTD",
         {"g": G3, "start": TS[0], "goal": TS[1], "gap": 0.0, "timeSteps": 4, "timeLimit": -1.0}, statics=dq)
    plan("planPathsAdvancedTD/stay_T0_nan", "planPathsAdvancedTD",
         {"g": STAR, "start": [0, 3], "goal": [0, 3], "gap": 0.0, "timeSteps": 0, "timeLimit": -1.0}, statics={},
         tags=["defect:dist-nan-gap-zero-objective"])
    for splits in (0, 1, 2):
        plan(f"planPathsAdvancedSplit/3x3_k{splits}", "planPathsAdvancedSplit",
             {"g": G3, "start": TS[0], "goal": TS[1], "setOptimal": True, "timeLimit": -1.0, "splits": splits})
    plan("planPathsAdvancedSplit/3x3_k1_notopt", "planPathsAdvancedSplit",
         {"g": G3, "start": TS[0], "goal": TS[1], "setOptimal": False, "timeLimit": -1.0, "splits": 1})
    plan("planPathsAdvancedSplit/fig8_k1", "planPathsAdvancedSplit",
         {"g": G4, "start": [13, 8], "goal": [0, 7], "setOptimal": True, "timeLimit": -1.0, "splits": 1},
         tags=["defect:fig8-split-child-needs-3"], note="3 + 2 = 5 steps (paper claims 2 + 2)")
    for splits in (1, 2):
        plan(f"planPathsAdvancedSplit/8x8_k{splits}", "planPathsAdvancedSplit",
             {"g": pg, "start": ps, "goal": pt, "setOptimal": True, "timeLimit": BIG, "splits": splits}, seed=2)
    plan("planPathsAdvancedSplit/square_hang", "planPathsAdvancedSplit",
         {"g": SQUARE, "start": [0, 2, 3], "goal": [3, 1, 2], "setOptimal": False, "timeLimit": -1.0, "splits": 1},
         budget=4, tags=["hang", "defect:splitpaths-infinite-loop"])
    sd = lambda gg, s, t, tl, k, gap=0.0: {"g": gg, "start": s, "goal": t, "gap": gap, "timeLimit": tl, "splits": k}  # noqa: E731
    plan("planPathsAdvancedSplitDist/3x3_k0", "planPathsAdvancedSplitDist", sd(G3, *TS, -1.0, 0), statics=dq)
    plan("planPathsAdvancedSplitDist/3x3_k1", "planPathsAdvancedSplitDist", sd(G3, *TS, BIG, 1), statics=dq)
    plan("planPathsAdvancedSplitDist/3x3_k2", "planPathsAdvancedSplitDist", sd(G3, *TS, BIG, 2), statics=dq)
    plan("planPathsAdvancedSplitDist/3x3_k1_any", "planPathsAdvancedSplitDist", sd(G3, *TS, BIG, 1),
         statics={**dq, "MultiagentGraphSolverGurobiDistance.bReturnAnySolution": True},
         note="bReturnAnySolution halves the first half's limit")
    plan("planPathsAdvancedSplitDist/3x3_k1_nolimit_null", "planPathsAdvancedSplitDist", sd(G3, *TS, -1.0, 1), statics=dq,
         tags=["defect:splitdist-null-without-limit"], note="tl <= 0 with splits > 0 always returns null")
    plan("planPathsAdvancedSplitDist/3x3_k2_zero_null", "planPathsAdvancedSplitDist", sd(G3, *TS, 0.0, 2), statics=dq,
         tags=["defect:splitdist-null-without-limit"])
    plan("planPathsAdvancedSplitDist/line2_swap_k0_npe", "planPathsAdvancedSplitDist", sd(LINE2, [0, 1], [1, 0], 1.0, 0),
         statics=dq, budget=30, tags=["crash", "defect:caller-npe-null-makespan"],
         note="the makespan leaf returns null when its time runs out; tp[0] then throws")
    plan("planPathsAdvancedSplitDist/stay_k0_nan", "planPathsAdvancedSplitDist", sd(STAR, [0, 3], [0, 3], 100.0, 0),
         statics=QUIET, tags=["defect:dist-nan-gap-zero-objective"])
    plan("planPathsAdvancedSplitDist/stay_k1_detour", "planPathsAdvancedSplitDist",
         sd(STAR, [0, 3], [0, 3], BIG, 1), statics=dq, tags=["defect:dist-nan-gap-zero-objective"],
         note="the split middle differs from start and goal, forcing a detour")
    nc = lambda gg, s, t, k, top=True, opt=True, tl=-1.0: {  # noqa: E731
        "g": gg, "start": s, "goal": t, "setOptimal": opt, "timeLimit": tl, "splits": k, "top": top}
    plan("planPathsAdvancedSplitNoCycle/3x3_k1", "planPathsAdvancedSplitNoCycle", nc(G3, *TS, 1))
    plan("planPathsAdvancedSplitNoCycle/3x3_k2", "planPathsAdvancedSplitNoCycle", nc(G3, *TS, 2))
    plan("planPathsAdvancedSplitNoCycle/3x3_k0", "planPathsAdvancedSplitNoCycle", nc(G3, *TS, 0))
    plan("planPathsAdvancedSplitNoCycle/3x3_k1_print", "planPathsAdvancedSplitNoCycle", nc(G3, *TS, 1),
         statics={"MultiagentGraphSolverGurobiTime.bDebugInfo": False}, tags=["defect:nocycle-printpaths-leak"],
         note="bPrintPaths true: printed once at top level, flag restored")
    plan("planPathsAdvancedSplitNoCycle/8x8_k1", "planPathsAdvancedSplitNoCycle", nc(pg, ps, pt, 1, tl=BIG), seed=2,
         budget=300)
    plan("planPathsAdvancedSplitNoCycle/n2m1p3_s0_hang", "planPathsAdvancedSplitNoCycle",
         nc(*problem_parts(P("n2m1p3_s0")), 1), seed=0, budget=60,
         tags=["hang", "defect:nocycle-unbounded-retry-hang"],
         note="seed 0 is reset by the case seed; the generator draws happened in phase 1")
    plan("planPathsAdvancedSplitNoCycle/n2m1p3_s0_nontop_hang", "planPathsAdvancedSplitNoCycle",
         nc(*problem_parts(P("n2m1p3_s0")), 1, top=False), seed=0, budget=60,
         tags=["hang", "defect:nocycle-unbounded-retry-hang"])
    bs = lambda gg, s, t, size, opt=True, tl=-1.0: {"g": gg, "start": s, "goal": t, "setOptimal": opt,  # noqa: E731
                                                     "timeLimit": tl, "size": size}
    plan("planPathsAdvancedSplitBySize/3x3_size2_hang", "planPathsAdvancedSplitBySize", bs(G3, *TS, 2), budget=4,
         tags=["hang", "defect:splitbysize-infinite-loop"])
    for size in (3, 4, 6):
        plan(f"planPathsAdvancedSplitBySize/3x3_size{size}", "planPathsAdvancedSplitBySize", bs(G3, *TS, size), budget=60)
    plan("planPathsAdvancedSplitBySize/line8_size5", "planPathsAdvancedSplitBySize", bs(LINE8, [0, 1], [4, 6], 5),
         tags=["defect:longest-path-off-by-one"])
    for size in (3, 4):
        plan(f"planPathsAdvancedSplitBySizeNoCycle/3x3_size{size}", "planPathsAdvancedSplitBySizeNoCycle",
             {**bs(G3, *TS, size), "top": True}, budget=60)
    mt = lambda gg, s, t, tl, k: {"g": gg, "start": s, "goal": t, "timeLimit": tl, "splits": k}  # noqa: E731
    plan("multiThreadedSplitPlanning/3x3_k0", "multiThreadedSplitPlanning", mt(G3, *TS, BIG, 0))
    for k in (1, 2):
        plan(f"multiThreadedSplitPlanning/3x3_k{k}", "multiThreadedSplitPlanning", mt(G3, *TS, BIG, k),
             nondet=["rng_next", "draws"], tags=["defect:mt-nondeterminism-scope"])
    plan("multiThreadedSplitPlanning/3x3_k1_print", "multiThreadedSplitPlanning", mt(G3, *TS, BIG, 1),
         statics={"MultiagentGraphSolverGurobiTime.bDebugInfo": False}, nondet=["rng_next", "draws"])
    plan("multiThreadedSplitPlanning/3x3_k0_nolimit", "multiThreadedSplitPlanning", mt(G3, *TS, -1.0, 0),
         tags=["defect:mt-nonpositive-timelimit"])
    plan("multiThreadedSplitPlanning/3x3_k1_nolimit_null", "multiThreadedSplitPlanning", mt(G3, *TS, -1.0, 1),
         nondet=["rng_next", "draws"], tags=["defect:mt-nonpositive-timelimit"])
    plan("multiThreadedSplitPlanning/3x3_k2_zero_null", "multiThreadedSplitPlanning", mt(G3, *TS, 0.0, 2),
         nondet=["result", "rng_next", "draws"], tags=["defect:mt-nonpositive-timelimit"],
         note="the original tests 0 - elapsed < 0: null unless no millisecond elapsed")
    p20 = P("p8x8_0_20_s7")
    plan("multiThreadedSplitPlanning/8x8_20_k2", "multiThreadedSplitPlanning", mt(*problem_parts(p20), BIG, 2), seed=7,
         budget=300, nondet=["rng_next", "draws"], tags=["defect:mt-split-race"])
    plan("multiThreadedSplitPlanning/8x8_10_k1", "multiThreadedSplitPlanning", mt(pg, ps, pt, BIG, 1), seed=8,
         budget=300, nondet=["rng_next", "draws"])
    plan("multiThreadedSplitPlanning/worker_npe_hang", "multiThreadedSplitPlanning", mt(TWOCOMP, [0], [3], BIG, 0),
         budget=5, tags=["hang", "defect:mt-grbexception-hang"],
         note="an exception in a worker leaves mdone[i] false; the wait loop never ends")
    a40, b40 = P("p8x8_0_40_s1"), P("p8x8_0_40_s100")
    c.add("planner", "Sequence/mt_stale_mpath", "Sequence",
          {"calls": [{"fn": f"{pp}.multiThreadedSplitPlanning", "args": mt(*problem_parts(a40), BIG, 1)},
                     {"fn": f"{pp}.multiThreadedSplitPlanning", "args": mt(*problem_parts(b40), -1.0, 1)}]},
          seed=1, statics=QUIET, budget=300, nondet=["rng_next", "draws"], tags=["defect:mt-stale-mpath"],
          note="one PathPlanner reused: pieces of call 2 that fail (timeLimit -1: null at the first infeasible T) "
               "keep call 1's paths")


def _heuristic_cases(c: Cases, g, P, G3, TS):
    garage = P("garage")
    gg, gs, gt = problem_parts(garage)
    p8, p3 = P("p8x8_10_10_s11"), P("p3x3_0_3_s9")
    for cls in ("PathPlanner", "HeuristicPathPlanner"):
        short = "pp" if cls == "PathPlanner" else "hpp"
        c.add("heuristic", f"{short}/planHeuristicPaths/garage", f"{cls}.planHeuristicPaths",
              {"graph": gg, "start": gs, "goal": gt}, seed=1, budget=120)
        c.add("heuristic", f"{short}/planHeuristicPaths/3x3", f"{cls}.planHeuristicPaths",
              {"graph": G3, "start": TS[0], "goal": TS[1]}, seed=2, budget=120)
        c.add("heuristic", f"{short}/planHeuristicPaths/p3x3", f"{cls}.planHeuristicPaths",
              {"graph": p3["graph"], "start": p3["sg"][0], "goal": p3["sg"][1]}, seed=3, budget=120)
        c.add("heuristic", f"{short}/planHeuristicPaths/p8x8_10", f"{cls}.planHeuristicPaths",
              {"graph": p8["graph"], "start": p8["sg"][0], "goal": p8["sg"][1]}, seed=4, budget=300,
              tags=["defect:wallclock-nondeterminism"], note="loose budgets: attempt 1 finishes well within time")
        c.add("heuristic", f"{short}/planHeuristicPaths/zero_agents", f"{cls}.planHeuristicPaths",
              {"graph": gg, "start": [], "goal": []}, seed=5, tags=["crash", "defect:zero-agents-aioobe"])
        sarg = "start" if cls == "PathPlanner" else "s"
        c.add("heuristic", f"{short}/planHeuristicPath/garage_npe", f"{cls}.planHeuristicPath",
              {"g": gg, sarg: gs, "goal": gt, "timeLimit": 5.0}, seed=6,
              tags=["crash", "defect:distanceToGoals-null-npe"], note="distanceToGoals is only set by planHeuristicPaths")
        c.add("heuristic", f"{short}/planHeuristicPath/garage_table", f"{cls}.planHeuristicPath",
              {"g": gg, sarg: gs, "goal": gt, "timeLimit": 30.0, "distanceToGoals": "fromGoals"}, seed=7, budget=120,
              note="distance table supplied (as planHeuristicPaths does); attempt field = 0")
        c.add("heuristic", f"{short}/planHeuristicPath/no_conflict", f"{cls}.planHeuristicPath",
              {"g": G3, sarg: [0, 2], "goal": [6, 8], "timeLimit": 30.0}, seed=8)
        c.add("heuristic", f"{short}/main", f"{cls}.main", {}, seed=9, budget=120)
        c.add("heuristic", f"{short}/numberOfGoalsReached", f"{cls}.numberOfGoalsReached",
              {"paths": [[0, 1], [3, 3]], "goal": [1, 0]})
        c.add("heuristic", f"{short}/printPaths", f"{cls}.printPaths", {"g": gg, "paths": [[0, 1, 2], [8, 7, 6]]})
    h = "HeuristicPathPlanner"
    coll = [[0, 1, 2], [2, 1, 0], [5, 4, 3]]
    for t in (0, 1):
        c.add("heuristic", f"hpp/existsEdgeCollision/t{t}", f"{h}.existsEdgeCollision", {"paths": coll, "t": t})
        c.add("heuristic", f"hpp/existsVertexCollision/t{t}", f"{h}.existsVertexCollision", {"paths": coll, "t": t})
    c.add("heuristic", "hpp/existsEdgeCollision/swap", f"{h}.existsEdgeCollision", {"paths": [[0, 1], [1, 0]], "t": 0})
    c.add("heuristic", "hpp/moveAgentsIndependentlyOneStep", f"{h}.moveAgentsIndependentlyOneStep",
          {"pathArray": [[0], [8]], "paths": [[0, 1, 2], [8, 7, 6]], "t": 2})
    for seed in (1, 2):
        c.add("heuristic", f"hpp/createSubProblem/3x3_s{seed}", f"{h}.createSubProblem",
              {"g": G3, "paths": [[0, 1, 4, 7], [2, 1, 4, 5], [8, 8, 8, 8]], "ct": 1, "svs": [1], "goal": [7, 5, 8],
               "distance": 2, "distanceToGoals": "fromGoals"}, seed=seed)
    c.add("heuristic", "hpp/createSubProblem/garage", f"{h}.createSubProblem",
          {"g": gg, "paths": [[0, 1, 2], [8, 7, 6]], "ct": 0, "svs": [0, 8], "goal": gt, "distance": 3,
           "distanceToGoals": "fromGoals"}, seed=3)
    c.add("heuristic", "Sequence/stale_distanceToGoals", "Sequence",
          {"calls": [{"fn": f"{h}.planHeuristicPaths", "args": {"graph": gg, "start": gs, "goal": gt}},
                     {"fn": f"{h}.planHeuristicPath", "args": {"g": gg, "s": [9, 0], "goal": [0, 9], "timeLimit": 30.0}}]},
          seed=4, budget=200, tags=["defect:distanceToGoals-null-npe"],
          note="the second call silently uses the first call's distance table")


def _main_cases(c: Cases, g, P, G3, TS):
    star_p = {"graph": STAR, "sg": [[0, 3], [3, 0]]}
    p3 = P("p3x3_0_3_s9")
    p8 = P("p8x8_0_10_s1")
    p66 = P("p6x6_10_5_s2")
    # main(argv)
    for argv, seed, budget, extra in [
        (["3"], 1, 120, {}), (["6"], 6, 300, {}), (["1"], 1, 600, {}), (["2"], 2, 300, {}),
        (["5"], 5, 900, {"tags": ["defect:case5-fallthrough"], "note": "missing break: also runs case 6"}),
        (["4", "$CASEDIR/none.dat"], 1, 60, {"tags": ["crash", "defect:readfromfile-swallow"]}),
        (["4"], 1, 60, {"tags": ["crash"]}),
        (["0"], 1, 60, {"tags": ["crash", "defect:readfromfile-swallow"]}),
        (["7"], 1, 60, {}), (["x"], 1, 60, {"tags": ["crash"]}),
    ]:
        c.add("main", f"main/{'_'.join(a.replace('$CASEDIR/', '') for a in argv)}", "Main.main", {"argv": argv},
              seed=seed, budget=budget, **extra)
    # solve* helpers (stat[1], the wall-clock time, is replaced by null)
    c.add("main", "solveProblem/star", "Main.solveProblem", {"p": star_p, "solveLP": True, "timeLimit": -1.0},
          seed=1, statics=QUIET)
    c.add("main", "solveProblem/p3x3", "Main.solveProblem", {"p": p3, "solveLP": False, "timeLimit": BIG},
          seed=2, statics=QUIET)
    c.add("main", "solveProblem/p8x8", "Main.solveProblem", {"p": p8, "solveLP": False, "timeLimit": BIG},
          seed=3, statics=QUIET)
    pmg = {**QUIET, "MultiagentGraphSolverGurobiTimeR.bDebugInfo": False,
           "MultiagentGraphSolverGurobiTimeR.bPrintPaths": False}
    c.add("main", "solveProblemPMG/star", "Main.solveProblemPMG", {"p": star_p, "solveLP": False, "timeLimit": -1.0},
          seed=1, statics=pmg)
    sub = lambda p, tl, k, nc, opt=True: {"p": p, "solveLP": False, "setOptimal": opt, "extraSteps": 0,  # noqa: E731
                                          "timeLimit": tl, "splitLevel": k, "noCycle": nc}
    c.add("main", "solveProblemSuboptimal/star_k0", "Main.solveProblemSuboptimal", sub(star_p, 100.0, 0, False),
          seed=1, statics=QUIET)
    c.add("main", "solveProblemSuboptimal/star_tl0_npe", "Main.solveProblemSuboptimal", sub(star_p, 0.0, 0, False),
          seed=1, statics=QUIET, tags=["crash", "defect:subopt-npe-on-failure", "defect:mt-nonpositive-timelimit"],
          note="worker returns null at the first infeasible T; printPaths(null) throws")
    c.add("main", "solveProblemSuboptimal/p3x3_k1_nocycle", "Main.solveProblemSuboptimal",
          sub(p3, BIG, 1, True), seed=2, statics=QUIET)
    c.add("main", "solveProblemSuboptimal/p8x8_k1", "Main.solveProblemSuboptimal", sub(p8, BIG, 1, False),
          seed=3, statics=QUIET, nondet=["rng_next", "draws"], budget=300)
    c.add("main", "solveProblemSuboptimal/p8x8_k2_nocycle", "Main.solveProblemSuboptimal", sub(p8, BIG, 2, True),
          seed=4, statics=QUIET, budget=300)
    tt = {**QUIET}  # TT/DIST flags stay at their defaults, as in Main
    c.add("main", "solveProblemTotalTime/p3x3", "Main.solveProblemTotalTime", {"p": p3, "gap": 0.0, "timeLimit": BIG},
          seed=1, statics=tt)
    c.add("main", "solveProblemTotalTime/p6x6", "Main.solveProblemTotalTime", {"p": p66, "gap": 0.01, "timeLimit": BIG},
          seed=2, statics=tt, budget=300)
    c.add("main", "solveProblemDistance/p3x3", "Main.solveProblemDistance", {"p": p3, "gap": 0.0, "timeLimit": BIG},
          seed=3, statics=tt)
    c.add("main", "solveProblemDistance/p6x6", "Main.solveProblemDistance", {"p": p66, "gap": 0.01, "timeLimit": BIG},
          seed=4, statics=tt, budget=300)
    c.add("main", "solveProblemDistanceSplit/p3x3_k1", "Main.solveProblemDistanceSplit",
          {"p": p3, "gap": 0.0, "timeLimit": BIG, "splits": 1}, seed=5, statics=tt)
    c.add("main", "solveProblemDistanceSplit/p6x6_k2", "Main.solveProblemDistanceSplit",
          {"p": p66, "gap": 0.01, "timeLimit": BIG, "splits": 2}, seed=6, statics=tt, budget=300)
    c.add("main", "solvePuzzleProblem/n2p3_s1", "Main.solvePuzzleProblem",
          {"instance": 0, "p": P("n2p3_s1"), "timeLimit": -1.0}, seed=7, statics=QUIET, budget=300,
          note="also writes basePath + '\\\\25puzzle.txt'")
    c.add("main", "solveProblemWithHeuristic/p8x8_10", "Main.solveProblemWithHeuristic", {"p": P("p8x8_10_10_s11")},
          seed=8, budget=300, tags=["defect:heuristic-stats-off-by-one", "defect:bheuristic-leak-nonoptimal"],
          note="leaves PathFinder.bHeuristic = true")
    # stats with injected stat vectors ([len, time_ms, ., makespanLB] / heuristic [len, ms, goals, steps, exp])
    seqs = [
        ("plain_median_dedup", "plain", "8x8, 10 agents, 10 runs. ", [[10, 1000, 0, 7], [10, 1000, 0, 7],
                                                                      [10, 1000, 0, 7], [10, 5000, 0, 7]], None),
        ("plain_odd", "plain", "x ", [[10, 1000, 0, 7], [12, 2000, 0, 8], [9, 2000, 0, 6], [11, 2000, 0, 9],
                                      [10, 9000, 0, 7]], None),
        ("plain_fractions", "plain", "24x18, 5% obstacles, 20 agents, 10 runs. ",
         [[30, 1109, 0, 30], None, [31, 107, 0, 30], [33, 1373, 0, 31]], None),
        ("plain_empty_aioobe", "plain", "none ", [None, None], None),
        ("heuristic", "heuristic", "32x32, 20% obstacles, 20 agents, 100 runs. ",
         [[132, 29, 25, 679, 579], [140, 31, 24, 0, 0], [99, 29, 25, 600, 512]], 25),
        ("heuristic_empty_aioobe", "heuristic", "h ", [], 5),
    ]
    for name, kind, test, stats, n in seqs:
        args = {"kind": kind, "test": test, "stats": stats}
        if n is not None:
            args["n"] = n
        c.add("main", f"statsSequence/{name}", "Main.statsSequence", args, normalize=False,
              tags=["defect:median-treeset-dedup"] if "median" in name else ())
    c.add("main", "writePuzzleStats", "Main.writePuzzleStats", {"instance": 3, "time": 12.345}, normalize=False)
    c.add("main", "writeString", "Main.writeString", {"str": "24 x 18 grid, exact\n"}, normalize=False)
    c.add("main", "writePuzzleResult", "Main.writePuzzleResult",
          {"instance": 4, "time": 21500.0, "g": G3, "paths": [[0, 1, 2], [8, 7, 6]]}, normalize=False)
    # run* drivers without data (basePath is an empty directory): they crash or end quickly.
    runs = [
        ("runN2PuzzlePerformanceMetric", []), ("runCrowdedPerformanceMetric8x8", [10, 20, 1800.0, 1, False]),
        ("runCrowdedPerformanceMetric16x16", [10, 20, 1800.0, 1, True]), ("run24x18PerformanceMetric", []),
        ("run32x32PerformanceMetric", []), ("run32x32PerformanceMetric8ConnectedAll", []),
        ("runCrowdedPerformanceMetricCrowded", []), ("run24x18PerformanceMetricTotalTime", []),
        ("run24x18PerformanceMetricTotalDistance", []), ("run24x18PerformanceMetricTotalDistanceSplit", []),
        ("run24x18PerformanceMetricTotalTimeFixedTime", []),
        ("run24x18PerformanceMetricTotalDistanceSplitFixedTime", []),
        ("run32x32PerformanceMetricSplitting", [25, 50, 25, 3600.0, 1, False]),
        ("run32x32PerformanceMetric8Connected", []), ("run32x32PerformanceMetricHeuristic", []),
        ("run32x32PerformanceMetricSplitting8Connected", [25, 50, 3600.0, 1, True]),
        ("runFullPerformanceMetric", []), ("runFullPerformanceMetricExtras", []),
    ]
    for name, params in runs:
        c.add("main", f"{name}/nodata", f"Main.{name}", {"params": params}, seed=1, statics=QUIET, budget=60,
              tags=["defect:readfromfile-swallow", "defect:legacy-missing-files"],
              note="no .dat files under basePath")
