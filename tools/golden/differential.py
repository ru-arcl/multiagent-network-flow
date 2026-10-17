#!/usr/bin/env python3
"""Differential testing of ``mppflow`` (``compat="original"``) against the live original Java code.

    python3 tools/golden/differential.py --n 30 --seed 1 --report diff.json \\
        (--classes CLASSES | --original-src ~/src/optimal-mrppg-journal/source) \\
        [--gurobi-jar $GUROBI_HOME/lib/gurobi.jar] [--jdk /path/to/jdk] \\
        [--groups generators,bfs,...] [--ids REGEX] [--work DIR] [--java-results FILE]

Every run generates FRESH random inputs (seeded by ``--seed``; the ``Math.random()`` seeds are
drawn from ``[1000, 2^31)``, so they never coincide with the small seeds of ``cases.py``), runs
each case once through the Java harness (``Harness.java``, built with ``build.sh``, Gurobi
``Threads 1``) and once through ``mppflow`` with ``compat="original"``, ``backend="gurobi"``,
``params={"Threads": 1}``, and compares the two records field by field: outcome (result,
exception class, hang), result, stdout (wall-clock numbers replaced by ``<t>``), random draws,
the next ``Math.random()`` value, mutated arguments, ``extra`` and (``main``) the static flags
left changed. Fields that the original itself does not reproduce run to run are skipped (the
draws of ``multiThreadedSplitPlanning``, see README.md "Determinism").

Groups (``--n`` cases each):

``generators``   Graph.create2DGridGraph(8Connected), createGeneric2DGridGraphWithHoles(8Connected),
                 getRandomStartGoal(Many), Problem.createGridProblem(8Connected), createN2Puzzle
``bfs``          PathFinder.getDistancesToAllVertices, findShortestPaths (draws, bHeuristic),
                 findOneShortestPath, findGoalsInDistanceRange (HashSet order)
``lowerbounds``  PathFinder.getMakespanLowerBound, getTotalTimeLowerBound
``split``        PathPlanner.splitPaths, splitPathBySize, getLongestPathLength
``models``       <Time|TimeR|TimePuzzle|TotalTime|Distance>.prepareModel (canonical dump SHA-256)
``solvers``      MultiagentGraphSolverGurobi<Time|TimeR|TotalTime|Distance>.solve
``planner``      PathPlanner.planPathsAdvanced, planPathsAdvancedSplitNoCycle, planPathsAdvancedSplitDist
``mtsplit``      PathPlanner.multiThreadedSplitPlanning with splits 1..3
``heuristic``    PathPlanner.planHeuristicPaths, HeuristicPathPlanner.planHeuristicPaths
``validity``     PathPlanner.isPathSetValid, hasCycles on valid and perturbed path sets
``main``         Main.solveProblem, solveProblemPMG, solveProblemSuboptimal, solveProblemTotalTime,
                 solveProblemDistance, solveProblemDistanceSplit, solveProblemWithHeuristic (stat vectors)

A difference with a documented cause is counted as ``skipped`` (see :func:`skip_reason`): the
original hung in a time-limited ``T++`` loop that the port short-cuts, the local-repair
heuristic's wall-clock budgets bound in the Java run, or ``multiThreadedSplitPlanning`` lost a
piece to its unsynchronized ``HashMap``. The report (``--report``) lists, per group, the number of cases compared and matched, and for
every mismatch the differing fields and a self-contained repro (``fn``, ``args``, ``seed``,
``statics``: the same format as the golden records). ``--ids REGEX`` reruns only matching cases
of the same generated list (same ``--seed`` and ``--n``). Exit status: 0 if every compared case
matches, 1 otherwise, 2 on usage / harness errors.
"""
from __future__ import annotations

import argparse
import contextlib
import json
import os
import random
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO))

import record  # noqa: E402  (Runner, assemble)

from mppflow import experiments as E  # noqa: E402
from mppflow import graph as G  # noqa: E402
from mppflow import heuristic as H  # noqa: E402
from mppflow import model as M  # noqa: E402
from mppflow import pathfinder as PF  # noqa: E402
from mppflow import planner as P  # noqa: E402
from mppflow import problem as PR  # noqa: E402
from mppflow import solvers as S  # noqa: E402
from mppflow._java import MathRandom  # noqa: E402
from mppflow.errors import JavaException, OriginalWouldHang  # noqa: E402
from mppflow.graph import Graph  # noqa: E402
from mppflow.problem import Problem  # noqa: E402

GROUPS = ["generators", "bfs", "lowerbounds", "split", "models", "solvers", "planner", "mtsplit", "heuristic",
          "validity", "main"]
COMPAT = "original"
BACKEND = "gurobi"
GRB = {"Threads": 1}
BIG = 1.0e6  # a time limit that never binds

T_ = "MultiagentGraphSolverGurobiTime"
TR_ = "MultiagentGraphSolverGurobiTimeR"
TZ_ = "MultiagentGraphSolverGurobiTimePuzzle"
TT_ = "MultiagentGraphSolverGurobiTotalTime"
DI_ = "MultiagentGraphSolverGurobiDistance"
QUIET = {f"{T_}.bDebugInfo": False, f"{T_}.bPrintPaths": False}
ALL_QUIET = {**QUIET, f"{TR_}.bDebugInfo": False, f"{TR_}.bPrintPaths": False, f"{TT_}.bDebugInfo": False,
             f"{TT_}.bPrintPaths": False, f"{DI_}.bDebugInfo": False}
DIST_QUIET = {**QUIET, f"{DI_}.bDebugInfo": False}


_TIMES = re.compile(r"(found in |max time limit |Time spent = |Total time spent: |, time: )"
                    r"(-?\d+(?:\.\d+)?(?:E-?\d+)?|-?Infinity|NaN)")


def norm_text(s: str | None) -> str | None:
    return None if s is None else _TIMES.sub(r"\1<t>", s)


def same_text(py: str, java: str) -> bool:
    """Equal after time normalization; a Java text cut at 20000 characters is a prefix."""
    py, java = norm_text(py), norm_text(java)
    m = re.search(r"\n?<truncated \d+ chars>$", java)
    if m:
        return py.startswith(java[:m.start()])
    return py == java


# ----------------------------------------------------------------------------------------------
# JSON <-> mppflow
# ----------------------------------------------------------------------------------------------

def _num(x):
    return int(x) if isinstance(x, float) and x.is_integer() else x


def graph_to_json(g: Graph, sort_vertices: bool = False) -> dict:
    """Encode a mppflow graph in the golden-record format (tools/golden/README.md)."""
    verts = sorted(g.vertices) if sort_vertices else list(g.vertices)
    out_v = []
    for v in verts:
        p = g.point(v)
        out_v.append([v] if p is None else [v, _num(p[0]), _num(p[1])])
    hs = bool(g.adj_order)
    adj = [[v, list(g.neighbors(v))] for v in sorted(g.adj)]
    return {"vertices": out_v, "adj": adj, "adj_type": "HashSet" if hs else "TreeSet", "rows": g.rows,
            "cols": g.cols}


def graph_from_json(d) -> Graph:
    g = Graph()
    for e in d["vertices"]:
        g.xy[e[0]] = (e[1], e[2]) if len(e) == 3 else None
    g.vertices = [e[0] for e in d["vertices"]]
    hs = d.get("adj_type") == "HashSet"
    for v, nb in d["adj"]:
        g.adj[v] = sorted(nb)
        if hs:
            if g.adj_order is None:
                g.adj_order = {}
            g.adj_order[v] = list(nb)
    g.rows, g.cols = d.get("rows", 0), d.get("cols", 0)
    return g


def problem_to_json(p: Problem) -> dict:
    d = {"graph": graph_to_json(p.graph), "sg": p.sg}
    if p.vid_new_id_map is not None:
        d["vidNewIdMap"] = [[k, v] for k, v in p.vid_new_id_map.items()]
    if p.new_id_vid_map is not None:
        d["newIdVidMap"] = [[k, v] for k, v in p.new_id_vid_map.items()]
    return d


def problem_from_json(d) -> Problem:
    return Problem.from_sg(graph_from_json(d["graph"]), d["sg"])


# ----------------------------------------------------------------------------------------------
# Random inputs
# ----------------------------------------------------------------------------------------------

def jseed(rr: random.Random) -> int:
    """A Math.random() seed outside the range used by cases.py."""
    return rr.randrange(1000, 2 ** 31 - 1)


def bfs(gj: dict, s: int) -> dict[int, int]:
    adj = {a[0]: a[1] for a in gj["adj"]}
    d = {s: 0}
    q = [s]
    for u in q:
        for v in adj.get(u, []):
            if v not in d:
                d[v] = d[u] + 1
                q.append(v)
    return d


def makespan_lb(gj, starts, goals) -> int:
    return max((bfs(gj, s).get(t, 0) for s, t in zip(starts, goals)), default=0)


def _cut(gj: dict, rr: random.Random) -> dict:
    """Remove every edge across a random vertical line x = c - 1/2 (two components; goals on the
    other side become unreachable)."""
    xs = sorted({v[1] for v in gj["vertices"] if len(v) == 3})
    if len(xs) < 2:
        return gj
    c = rr.choice(xs[1:])
    side = {v[0]: (v[1] >= c) for v in gj["vertices"]}
    gj = dict(gj)
    gj["adj"] = [[u, [w for w in nb if side[w] == side[u]]] for u, nb in gj["adj"]]
    return gj


def instance(rr: random.Random, min_side=3, max_side=8, max_agents=10, eight=None, obstacles=True,
             min_agents=1, density=3, cut=0.0, stay=0.0):
    """A random grid problem built with the port's generator (compat original, a fresh seed).
    With probability ``cut`` the graph is split in two components (unreachable goals), with
    probability ``stay`` every goal equals its start. Returns (graph_json, starts, goals,
    description)."""
    gj, s, t, d = _instance(rr, min_side, max_side, max_agents, eight, obstacles, min_agents, density)
    if cut and rr.random() < cut:
        gj, d = _cut(gj, rr), d + "_cut"
    if stay and rr.random() < stay:
        t, d = list(s), d + "_stay"
    return gj, s, t, d


def _instance(rr, min_side, max_side, max_agents, eight, obstacles, min_agents, density):
    col, row = rr.randint(min_side, max_side), rr.randint(min_side, max_side)
    frac = rr.choice([0.0, 0.1, 0.2, 0.3]) if obstacles else 0.0
    if eight is None:
        eight = rr.random() < 0.25
    nv = col * row - int(col * row * frac)
    n = rr.randint(min(min_agents, max(1, nv // density)), max(1, min(max_agents, nv // density)))
    s = jseed(rr)
    f = PR.create_grid_problem_8connected if eight else PR.create_grid_problem
    p = f(col, row, frac, n, MathRandom(s), COMPAT)
    desc = f"{'8c' if eight else '4c'}_{col}x{row}_{frac}_{n}_s{s}"
    return graph_to_json(p.graph), list(p.starts), list(p.goals), desc


class Cases:
    def __init__(self):
        self.items: list[dict] = []

    def add(self, group, name, fn, args, seed=0, statics=None, budget=None, nondet=None, note=None):
        c = {"id": f"diff/{group}/{len([x for x in self.items if x['group'] == group]):03d}_{name}",
             "group": group, "fn": fn, "args": args, "seed": seed}
        if statics:
            c["statics"] = dict(statics)
        if budget is not None:
            c["budget"] = budget
        if nondet:
            c["nondet"] = list(nondet)
        if note:
            c["note"] = note
        self.items.append(c)


def gen_generators(c: Cases, rr: random.Random, n: int):
    for i in range(n):
        k = i % 8
        s = jseed(rr)
        if k == 0:
            r, q, u = rr.randint(1, 9), rr.randint(1, 9), rr.random() < 0.6
            c.add("generators", f"grid4_{r}x{q}_{u}", "Graph.create2DGridGraph", {"rows": r, "cols": q, "update": u},
                  seed=s)
        elif k == 1:
            r, q, u = rr.randint(1, 8), rr.randint(1, 8), rr.random() < 0.6
            c.add("generators", f"grid8_{r}x{q}_{u}", "Graph.create2DGridGraph8Connected",
                  {"rows": r, "cols": q, "update": u}, seed=s)
        elif k in (2, 3):
            r, q = rr.randint(2, 9), rr.randint(2, 9)
            p = round(rr.uniform(0.0, 0.4), 3)
            fn = "Graph.createGeneric2DGridGraphWithHoles" + ("" if k == 2 else "8Connected")
            c.add("generators", f"holes{4 if k == 2 else 8}_{r}x{q}_{p}", fn,
                  {"rows": r, "cols": q, "percentObstacle": p}, seed=s, budget=20)
        elif k == 4:
            r, q = rr.randint(2, 8), rr.randint(2, 8)
            p = rr.choice([0.0, 0.1, 0.2, 0.3])
            eight = rr.random() < 0.3
            g = (G.create_generic_2d_grid_graph_with_holes_8connected if eight
                 else G.create_generic_2d_grid_graph_with_holes)(r, q, p, MathRandom(jseed(rr)), COMPAT)
            if rr.random() < 0.5:
                g = G.convert_graph(g, [], [], COMPAT)
            num = rr.randint(0, len(g.vertices))
            c.add("generators", f"rsg_{r}x{q}_{p}_{num}", "Graph.getRandomStartGoal",
                  {"g": graph_to_json(g, sort_vertices=True), "number": num}, seed=s, budget=20)
        elif k == 5:
            r, q = rr.randint(2, 6), rr.randint(2, 6)
            g = G.create_2d_grid_graph(r, q, True, COMPAT)
            num = rr.randint(1, r * q) if rr.random() < 0.9 else r * q + rr.randint(1, 3)  # > |V|: IOOBE
            c.add("generators", f"rsgmany_{r}x{q}_{num}", "Graph.getRandomStartGoalMany",
                  {"g": graph_to_json(g), "number": num}, seed=s)
        elif k == 6:
            col, row = rr.randint(3, 10), rr.randint(3, 10)
            f = rr.choice([0.0, 0.1, 0.2, 0.3])
            num = rr.randint(1, max(1, (col * row - int(col * row * f)) // 2))
            fn = "Problem.createGridProblem" + ("8Connected" if rr.random() < 0.3 else "")
            c.add("generators", f"gp_{col}x{row}_{f}_{num}", fn, {"col": col, "row": row, "fracObs": f, "n": num},
                  seed=s, budget=20)
        else:
            m = rr.randint(2, 5)
            c.add("generators", f"n2p{m}", "Problem.createN2Puzzle", {"n": m}, seed=s)


def gen_bfs(c: Cases, rr, n):
    for i in range(n):
        gj, s, t, d = instance(rr, max_side=10, max_agents=12, cut=0.15)
        ids = [v[0] for v in gj["vertices"]]
        k = i % 5
        if k == 0:
            c.add("bfs", f"dist_{d}", "PathFinder.getDistancesToAllVertices",
                  {"g": gj, "start": rr.choice(ids), "inputDistances": None})
        elif k == 1:
            lo = rr.randint(0, 4)
            hi = lo + rr.randint(-1, 4)
            c.add("bfs", f"range_{d}_{lo}_{hi}", "PathFinder.findGoalsInDistanceRange",
                  {"g": gj, "start": rr.choice(ids), "dMin": lo, "dMax": hi})
        elif k in (2, 3):
            st = {"PathFinder.bHeuristic": True} if rr.random() < 0.3 else None
            c.add("bfs", f"paths_{d}{'_h' if st else ''}", "PathFinder.findShortestPaths",
                  {"g": gj, "start": s, "goal": t}, seed=jseed(rr), statics=st)
        else:
            st = {"PathFinder.bHeuristic": True} if rr.random() < 0.3 else None
            c.add("bfs", f"one_{d}{'_h' if st else ''}", "PathFinder.findOneShortestPath",
                  {"g": gj, "start": rr.choice(ids), "goal": rr.choice(ids), "alreadyVisitedVertices": None},
                  seed=jseed(rr), statics=st)


def gen_lowerbounds(c: Cases, rr, n):
    for i in range(n):
        gj, s, t, d = instance(rr, max_side=10, max_agents=15, cut=0.15)
        fn = "PathFinder.getMakespanLowerBound" if i % 2 == 0 else "PathFinder.getTotalTimeLowerBound"
        st = {"PathFinder.bHeuristic": True} if rr.random() < 0.2 else None
        c.add("lowerbounds", f"{fn.split('.')[1]}_{d}", fn, {"g": gj, "start": s, "goal": t}, seed=jseed(rr),
              statics=st)


def gen_split(c: Cases, rr, n):
    for i in range(n):
        gj, s, t, d = instance(rr, max_side=8, max_agents=10, density=4)
        k = i % 4
        if k in (0, 1):
            rnd = rr.random() < 0.5
            c.add("split", f"splitPaths_{d}_{rnd}", "PathPlanner.splitPaths",
                  {"g": gj, "start": s, "goal": t, "random": rnd}, seed=jseed(rr), budget=8)
        elif k == 2:
            rnd = rr.random() < 0.5
            lpl = makespan_lb(gj, s, t) + 1
            size = rr.randint(max(1, lpl // 2), lpl + 1)
            c.add("split", f"splitPathBySize_{d}_{rnd}_{size}", "PathPlanner.splitPathBySize",
                  {"g": gj, "start": s, "goal": t, "random": rnd, "firstSize": size}, seed=jseed(rr), budget=8)
        else:
            c.add("split", f"longest_{d}", "PathPlanner.getLongestPathLength", {"g": gj, "start": s, "goal": t},
                  seed=jseed(rr))


def gen_models(c: Cases, rr, n):
    classes = [T_, TR_, TZ_, TT_, DI_]
    for i in range(n):
        cls = classes[i % len(classes)]
        gj, s, t, d = instance(rr, max_side=7, max_agents=8)
        T = max(0, makespan_lb(gj, s, t) + rr.randint(-1, 3))
        integer = rr.random() < 0.75
        args = {"graph": gj, "starts": s, "goals": t, "integer": integer, "timeSteps": T}
        if cls in (TT_, DI_):
            args["timeLimit"] = rr.choice([-1.0, 12.5, BIG])
            g = rr.choice([None, 0.0, 0.01, 0.1])
            if g is not None:
                args["gap"] = g
        else:
            args["setOptimal"] = rr.random() < 0.7
            if cls != TZ_:
                args["timeLimit"] = rr.choice([-1.0, 12.5, BIG])
        c.add("models", f"{cls[len('MultiagentGraphSolverGurobi'):]}_{d}_T{T}", f"{cls}.prepareModel", args,
              statics=ALL_QUIET)


def _flags(rr, cls, default_print=True):
    """Random printing flags for one solver class (quiet, debug only, or Java defaults)."""
    x = rr.random()
    if x < 0.5:
        return {f"{cls}.bDebugInfo": False, f"{cls}.bPrintPaths": False}
    if x < 0.75:
        return {f"{cls}.bDebugInfo": True, f"{cls}.bPrintPaths": False}
    return {} if default_print else {f"{cls}.bPrintPaths": True}


def gen_solvers(c: Cases, rr, n):
    for i in range(n):
        k = i % 5
        if k in (0, 1):
            gj, s, t, d = instance(rr, max_side=7, max_agents=10, cut=0.1)
            args = {"graph": gj, "starts": s, "goals": t, "solveRelaxedFirst": rr.random() < 0.5,
                    "setOptimal": rr.random() < 0.7, "extraSteps": rr.choice([0, 0, 0, 1, 2]),
                    "hasTimeLimit": rr.random() < 0.3, "timeLimit": BIG}
            c.add("solvers", f"Time_{d}", f"{T_}.solve", args, seed=jseed(rr), statics=_flags(rr, T_), budget=60)
        elif k == 2:
            gj, s, t, d = instance(rr, min_side=2, max_side=5, max_agents=3, obstacles=False, eight=False)
            args = {"graph": gj, "starts": s, "goals": t, "solveRelaxedFirst": rr.random() < 0.5,
                    "setOptimal": True, "extraSteps": 0, "hasTimeLimit": False, "timeLimit": -1.0}
            c.add("solvers", f"TimeR_{d}", f"{TR_}.solve", args, seed=jseed(rr), statics=_flags(rr, TR_),
                  budget=120)
        elif k == 3:
            gj, s, t, d = instance(rr, max_side=6, max_agents=8, stay=0.15)
            T = makespan_lb(gj, s, t) + rr.randint(0, 2)
            st = {f"{TT_}.bPrintPaths": False} if rr.random() < 0.6 else {}
            if rr.random() < 0.15:
                st[f"{TT_}.bReturnAnySolution"] = True
            args = {"graph": gj, "starts": s, "goals": t, "opt": [-777], "gap": rr.choice([0.0, 0.0, 0.01]),
                    "timeSteps": T, "hasTimeLimit": rr.random() < 0.3, "timeLimit": BIG}
            c.add("solvers", f"TotalTime_{d}_T{T}", f"{TT_}.solve", args, seed=jseed(rr), statics=st, budget=60)
        else:
            gj, s, t, d = instance(rr, max_side=6, max_agents=8, stay=0.15)
            T = makespan_lb(gj, s, t) + rr.randint(0, 2)
            st = {} if rr.random() < 0.6 else {f"{DI_}.bPrintPaths": True}
            if rr.random() < 0.15:
                st[f"{DI_}.bReturnAnySolution"] = True
            args = {"graph": gj, "starts": s, "goals": t, "gap": rr.choice([0.0, 0.0, 0.01]), "expansion": T,
                    "timeLimit": rr.choice([-1.0, BIG])}
            c.add("solvers", f"Distance_{d}_T{T}", f"{DI_}.solve", args, seed=jseed(rr), statics=st, budget=60)


def gen_planner(c: Cases, rr, n):
    pp = "PathPlanner"
    for i in range(n):
        k = i % 3
        gj, s, t, d = instance(rr, max_side=8, max_agents=12, density=4)
        if k == 0:
            args = {"g": gj, "start": s, "goal": t, "solveRelaxed": rr.random() < 0.5,
                    "extraSteps": rr.choice([0, 0, 1]), "setOptimal": rr.random() < 0.7,
                    "timeLimit": rr.choice([-1.0, BIG])}
            st = QUIET if rr.random() < 0.6 else {}
            c.add("planner", f"adv_{d}", f"{pp}.planPathsAdvanced", args, seed=jseed(rr), statics=st, budget=60)
        elif k == 1:
            args = {"g": gj, "start": s, "goal": t, "setOptimal": rr.random() < 0.7,
                    "timeLimit": rr.choice([-1.0, BIG, BIG]), "splits": rr.randint(1, 2), "top": rr.random() < 0.8}
            st = QUIET if rr.random() < 0.8 else {f"{T_}.bDebugInfo": False}
            c.add("planner", f"nocycle_{d}_k{args['splits']}", f"{pp}.planPathsAdvancedSplitNoCycle", args,
                  seed=jseed(rr), statics=st, budget=90)
        else:
            args = {"g": gj, "start": s, "goal": t, "gap": rr.choice([0.0, 0.0, 0.01]),
                    "timeLimit": rr.choice([-1.0, BIG, BIG, BIG]), "splits": rr.randint(0, 2)}
            c.add("planner", f"splitdist_{d}_k{args['splits']}", f"{pp}.planPathsAdvancedSplitDist", args,
                  seed=jseed(rr), statics=DIST_QUIET, budget=90)


def gen_mtsplit(c: Cases, rr, n):
    for i in range(n):
        k = 1 + i % 3
        gj, s, t, d = instance(rr, min_side=4, max_side=9, max_agents=14, density=4)
        tl = BIG if rr.random() < 0.85 else -1.0
        st = QUIET if rr.random() < 0.8 else {f"{T_}.bDebugInfo": False}
        c.add("mtsplit", f"mt_{d}_k{k}", "PathPlanner.multiThreadedSplitPlanning",
              {"g": gj, "start": s, "goal": t, "timeLimit": tl, "splits": k}, seed=jseed(rr), statics=st,
              budget=120, nondet=["rng_next", "draws"])


def gen_heuristic(c: Cases, rr, n):
    for i in range(n):
        cls = "PathPlanner" if i % 2 == 0 else "HeuristicPathPlanner"
        gj, s, t, d = instance(rr, max_side=6, max_agents=8, density=3, min_agents=2)
        c.add("heuristic", f"{'pp' if i % 2 == 0 else 'hpp'}_{d}", f"{cls}.planHeuristicPaths",
              {"graph": gj, "start": s, "goal": t}, seed=jseed(rr), budget=120)


def _valid_base(gj, s, t, rr):
    """A path set for (s, t): an optimal plan (Gurobi, fixed mode) or padded shortest paths."""
    g = graph_from_json(gj)
    if rr.random() < 0.6:
        try:
            res = S.solve_time(g, s, t, rng=MathRandom(jseed(rr)), backend=BACKEND, params=GRB, compat="fixed")
            if res.paths is not None:
                return [list(p) for p in res.paths]
        except Exception:
            pass
    paths = PF.find_shortest_paths(g, s, t, MathRandom(jseed(rr)), compat="fixed")
    L = max(len(p) for p in paths)
    return [list(p) + [p[-1]] * (L - len(p)) for p in paths]


def _perturb(paths, gj, rr):
    adj = {a[0]: a[1] for a in gj["adj"]}
    ids = [v[0] for v in gj["vertices"]]
    paths = [list(p) for p in paths]
    A, L = len(paths), len(paths[0])
    kind = rr.choice(["none", "neighbor", "jump", "meet", "swap", "truncate", "extend", "wait_insert",
                      "reverse", "dup", "empty"])
    a = rr.randrange(A)
    t = rr.randrange(L)
    if kind == "neighbor" and t > 0:
        paths[a][t] = rr.choice(adj[paths[a][t - 1]])
    elif kind == "jump":
        paths[a][t] = rr.choice(ids)
    elif kind == "meet" and A > 1:
        b = rr.choice([x for x in range(A) if x != a])
        paths[b][t] = paths[a][t]
    elif kind == "swap" and A > 1 and L > 1:
        b = rr.choice([x for x in range(A) if x != a])
        t = rr.randrange(L - 1)
        paths[a][t + 1], paths[b][t + 1] = paths[b][t], paths[a][t]
    elif kind == "truncate" and L > 1:
        paths[a] = paths[a][:-1]
    elif kind == "extend":
        paths[a] = paths[a] + [paths[a][-1]]
    elif kind == "wait_insert":
        paths = [p[:t + 1] + [p[t]] + p[t + 1:] for p in paths]
    elif kind == "reverse":
        paths[a] = paths[a][::-1]
    elif kind == "empty":
        paths[a] = []
    elif kind == "dup" and A > 1:
        b = rr.choice([x for x in range(A) if x != a])
        paths[b] = list(paths[a])
    return paths, kind


def _rotation_case(rr):
    """Robots rotating around a cycle of a grid (hasCycles counts these)."""
    r = rr.randint(2, 4)
    q = rr.randint(2, 4)
    g = G.create_2d_grid_graph(r, q, True, COMPAT)
    ring = [y * q for y in range(r)] + [(r - 1) * q + x for x in range(1, q)] + \
           [y * q + q - 1 for y in range(r - 2, -1, -1)] + [x for x in range(q - 2, 0, -1)]
    m = len(ring) if rr.random() < 0.5 else rr.randint(1, len(ring))
    occ = sorted(rr.sample(range(len(ring)), m))
    steps = rr.randint(1, 4)
    paths = []
    for j in occ:
        paths.append([ring[(j + u) % len(ring)] for u in range(steps + 1)])
    s = [p[0] for p in paths]
    t = [p[-1] for p in paths]
    return graph_to_json(g), s, t, paths


def gen_validity(c: Cases, rr, n):
    for i in range(n):
        if rr.random() < 0.3:
            gj, s, t, paths = _rotation_case(rr)
            kind, d = "rotation", f"ring{len(s)}"
        else:
            gj, s, t, d = instance(rr, max_side=5, max_agents=6, density=3)
            paths, kind = _perturb(_valid_base(gj, s, t, rr), gj, rr)
        if i % 2 == 0:
            ss, tt = list(s), list(t)
            if rr.random() < 0.1:
                ss[0] = rr.choice([v[0] for v in gj["vertices"]])
            c.add("validity", f"valid_{kind}_{d}", "PathPlanner.isPathSetValid",
                  {"paths": paths, "graph": gj, "starts": ss, "goals": tt})
        else:
            c.add("validity", f"cycles_{kind}_{d}", "PathPlanner.hasCycles", {"paths": paths})


def gen_main(c: Cases, rr, n):
    for i in range(n):
        k = i % 7
        seed = jseed(rr)
        if k == 0:
            gj, s, t, d = instance(rr, max_side=8, max_agents=12)
            c.add("main", f"solveProblem_{d}", "Main.solveProblem",
                  {"p": {"graph": gj, "sg": [s, t]}, "solveLP": rr.random() < 0.5, "timeLimit": rr.choice([-1.0, BIG])},
                  seed=seed, statics=QUIET, budget=90)
        elif k == 1:
            gj, s, t, d = instance(rr, min_side=2, max_side=4, max_agents=3, obstacles=False, eight=False)
            c.add("main", f"solveProblemPMG_{d}", "Main.solveProblemPMG",
                  {"p": {"graph": gj, "sg": [s, t]}, "solveLP": rr.random() < 0.5, "timeLimit": -1.0},
                  seed=seed, statics={**QUIET, f"{TR_}.bDebugInfo": False, f"{TR_}.bPrintPaths": False}, budget=120)
        elif k == 2:
            gj, s, t, d = instance(rr, max_side=8, max_agents=12, density=4)
            kk, nc = rr.randint(0, 2), rr.random() < 0.5
            c.add("main", f"solveProblemSuboptimal_{d}_k{kk}_{nc}", "Main.solveProblemSuboptimal",
                  {"p": {"graph": gj, "sg": [s, t]}, "solveLP": False, "setOptimal": rr.random() < 0.8,
                   "extraSteps": 0, "timeLimit": BIG, "splitLevel": kk, "noCycle": nc},
                  seed=seed, statics=QUIET, budget=120,
                  nondet=["rng_next", "draws"] if kk > 0 and not nc else None)
        elif k == 3:
            gj, s, t, d = instance(rr, max_side=6, max_agents=8)
            c.add("main", f"solveProblemTotalTime_{d}", "Main.solveProblemTotalTime",
                  {"p": {"graph": gj, "sg": [s, t]}, "gap": rr.choice([0.0, 0.01]), "timeLimit": BIG},
                  seed=seed, statics=QUIET, budget=120)
        elif k == 4:
            gj, s, t, d = instance(rr, max_side=6, max_agents=8)
            c.add("main", f"solveProblemDistance_{d}", "Main.solveProblemDistance",
                  {"p": {"graph": gj, "sg": [s, t]}, "gap": rr.choice([0.0, 0.01]), "timeLimit": BIG},
                  seed=seed, statics=QUIET, budget=120)
        elif k == 5:
            gj, s, t, d = instance(rr, max_side=7, max_agents=10, density=4)
            kk = rr.randint(1, 2)
            c.add("main", f"solveProblemDistanceSplit_{d}_k{kk}", "Main.solveProblemDistanceSplit",
                  {"p": {"graph": gj, "sg": [s, t]}, "gap": rr.choice([0.0, 0.01]), "timeLimit": BIG, "splits": kk},
                  seed=seed, statics=QUIET, budget=120)
        else:
            gj, s, t, d = instance(rr, max_side=6, max_agents=8, min_agents=2)
            c.add("main", f"solveProblemWithHeuristic_{d}", "Main.solveProblemWithHeuristic",
                  {"p": {"graph": gj, "sg": [s, t]}}, seed=seed, statics=QUIET, budget=120)


GENERATORS = {"generators": gen_generators, "bfs": gen_bfs, "lowerbounds": gen_lowerbounds, "split": gen_split,
              "models": gen_models, "solvers": gen_solvers, "planner": gen_planner, "mtsplit": gen_mtsplit,
              "heuristic": gen_heuristic, "validity": gen_validity, "main": gen_main}


def generate(groups, n: int, seed: int) -> list[dict]:
    """The case list: deterministic in (groups, n, seed); each group has its own stream."""
    c = Cases()
    for grp in groups:
        rr = random.Random(f"mppflow-differential/{seed}/{grp}")
        GENERATORS[grp](c, rr, n)
    return c.items


# ----------------------------------------------------------------------------------------------
# The port's side
# ----------------------------------------------------------------------------------------------

class _PyTimeout(Exception):
    pass


# Count the port's "time-limited endless loop" shortcuts (mppflow.solvers docstring, "Infinite
# loops": an instance proved infeasible under a time limit returns null at once, where the
# original keeps increasing T until its limit expires).
_SHORTCUTS = [0]
_solve_makespan = S._solve_makespan


def _counting_solve_makespan(*a, **k):
    r = _solve_makespan(*a, **k)
    if r.extra.get("shortcut"):
        _SHORTCUTS[0] += 1
    return r


S._solve_makespan = _counting_solve_makespan


@contextlib.contextmanager
def _alarm(seconds: float):
    if seconds is None or not hasattr(signal, "setitimer"):
        yield
        return

    def handler(signum, frame):
        raise _PyTimeout(f"no result within {seconds} s")

    old = signal.signal(signal.SIGALRM, handler)
    signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old)


def _planner(case, rng):
    flags = P.SolverFlags.java_defaults().apply_java(case.get("statics") or {})
    return P.PathPlanner(backend=BACKEND, params=GRB, rng=rng, compat=COMPAT, flags=flags)


def _cls_flags(st: dict, cls: str, debug_default=True, print_default=True):
    return st.get(f"{cls}.bDebugInfo", debug_default), st.get(f"{cls}.bPrintPaths", print_default)


def _py_call(case: dict, rng: MathRandom, out: dict):
    """Run ``case`` on mppflow; fills out['stdout'/'mutated'/'extra'/'statics_after'] and
    returns the result in the golden JSON encoding."""
    fn, a, st = case["fn"], case["args"], case.get("statics") or {}
    # ------------------------------------------------------------- generators
    if fn == "Graph.create2DGridGraph":
        return graph_to_json(G.create_2d_grid_graph(a["rows"], a["cols"], a["update"], COMPAT))
    if fn == "Graph.create2DGridGraph8Connected":
        return graph_to_json(G.create_2d_grid_graph_8connected(a["rows"], a["cols"], a["update"], COMPAT))
    if fn == "Graph.createGeneric2DGridGraphWithHoles":
        return graph_to_json(G.create_generic_2d_grid_graph_with_holes(a["rows"], a["cols"], a["percentObstacle"],
                                                                       rng, COMPAT), sort_vertices=True)
    if fn == "Graph.createGeneric2DGridGraphWithHoles8Connected":
        return graph_to_json(G.create_generic_2d_grid_graph_with_holes_8connected(
            a["rows"], a["cols"], a["percentObstacle"], rng, COMPAT), sort_vertices=True)
    if fn == "Graph.getRandomStartGoal":
        return G.get_random_start_goal(graph_from_json(a["g"]), a["number"], rng, COMPAT)
    if fn == "Graph.getRandomStartGoalMany":
        return G.get_random_start_goal_many(graph_from_json(a["g"]), a["number"], rng, COMPAT)
    if fn == "Problem.createGridProblem":
        return problem_to_json(PR.create_grid_problem(a["col"], a["row"], a["fracObs"], a["n"], rng, COMPAT))
    if fn == "Problem.createGridProblem8Connected":
        return problem_to_json(PR.create_grid_problem_8connected(a["col"], a["row"], a["fracObs"], a["n"], rng,
                                                                 COMPAT))
    if fn == "Problem.createN2Puzzle":
        return problem_to_json(PR.create_n2_puzzle(a["n"], rng, COMPAT))
    # ------------------------------------------------------------- pathfinder
    if fn == "PathFinder.getDistancesToAllVertices":
        return PF.get_distances_to_all_vertices(graph_from_json(a["g"]), a["start"], a["inputDistances"], COMPAT)
    if fn == "PathFinder.findGoalsInDistanceRange":
        return PF.find_goals_in_distance_range(graph_from_json(a["g"]), a["start"], a["dMin"], a["dMax"], COMPAT)
    if fn == "PathFinder.findShortestPaths":
        return PF.find_shortest_paths(graph_from_json(a["g"]), a["start"], a["goal"], rng, compat=COMPAT)
    if fn == "PathFinder.findOneShortestPath":
        av = None if a["alreadyVisitedVertices"] is None else set(a["alreadyVisitedVertices"])
        return PF.find_one_shortest_path(graph_from_json(a["g"]), a["start"], a["goal"], av, rng, compat=COMPAT)
    if fn == "PathFinder.getMakespanLowerBound":
        return PF.get_makespan_lower_bound(graph_from_json(a["g"]), a["start"], a["goal"], rng, compat=COMPAT)
    if fn == "PathFinder.getTotalTimeLowerBound":
        return PF.get_total_time_lower_bound(graph_from_json(a["g"]), a["start"], a["goal"], rng, compat=COMPAT)
    # ------------------------------------------------------------- models
    if fn.endswith(".prepareModel"):
        cls = fn.split(".")[0]
        g = graph_from_json(a["graph"])
        s, t, T = a["starts"], a["goals"], a["timeSteps"]
        if cls == T_:
            m = M.build_time_model(g, s, t, T, a["integer"], a["setOptimal"], time_limit=a["timeLimit"], compat=COMPAT)
        elif cls == TR_:
            m = M.build_time_r_model(g, s, t, T, a["integer"], a["setOptimal"], time_limit=a["timeLimit"],
                                     compat=COMPAT)
        elif cls == TZ_:
            m = M.build_time_puzzle_model(g, s, t, T, a["integer"], a["setOptimal"], compat=COMPAT)
        elif cls == TT_:
            m = M.build_total_time_model(g, s, t, T, a["integer"], time_limit=a["timeLimit"], gap=a.get("gap"),
                                         compat=COMPAT)
        else:
            m = M.build_distance_model(g, s, t, T, a["integer"], time_limit=a["timeLimit"], gap=a.get("gap"),
                                       compat=COMPAT)
        return m.summary(with_dump=False)
    # ------------------------------------------------------------- solvers
    if fn in (f"{T_}.solve", f"{TR_}.solve"):
        cls = fn.split(".")[0]
        f = S.solve_time_r if cls == TR_ else S.solve_time
        dbg, pr = _cls_flags(st, cls)
        res = f(graph_from_json(a["graph"]), a["starts"], a["goals"], a["solveRelaxedFirst"], a["setOptimal"],
                a["extraSteps"], a["hasTimeLimit"], a["timeLimit"], rng=rng, backend=BACKEND, params=GRB,
                compat=COMPAT, debug=dbg, print_paths=pr)
        out["stdout"] = res.log
        return res.paths
    if fn == f"{TT_}.solve":
        opt = list(a["opt"])
        dbg, pr = _cls_flags(st, TT_)
        res = S.solve_total_time(graph_from_json(a["graph"]), a["starts"], a["goals"], opt, a["gap"], a["timeSteps"],
                                 a["hasTimeLimit"], a["timeLimit"],
                                 return_any_solution=st.get(f"{TT_}.bReturnAnySolution", False), horizon="single",
                                 rng=rng, backend=BACKEND, params=GRB, compat=COMPAT, debug=dbg, print_paths=pr)
        out["stdout"] = res.log
        out["mutated"] = {"opt": opt} if opt != a["opt"] else {}
        return res.paths
    if fn == f"{DI_}.solve":
        dbg, pr = _cls_flags(st, DI_, True, False)
        res = S.solve_distance(graph_from_json(a["graph"]), a["starts"], a["goals"], a["gap"], a["expansion"],
                               a["timeLimit"], return_any_solution=st.get(f"{DI_}.bReturnAnySolution", False),
                               horizon="single", backend=BACKEND, params=GRB, compat=COMPAT, debug=dbg,
                               print_paths=pr)
        out["stdout"] = res.log
        return res.paths
    # ------------------------------------------------------------- planner
    if fn.startswith("PathPlanner.") and fn != "PathPlanner.planHeuristicPaths" and fn not in (
            "PathPlanner.isPathSetValid", "PathPlanner.hasCycles"):
        pl = _planner(case, rng)
        out["_planner"] = pl
        name = fn.split(".", 1)[1]
        g = graph_from_json(a["g"])
        if name == "splitPaths":
            return pl.split_paths(g, a["start"], a["goal"], a["random"])
        if name == "splitPathBySize":
            return pl.split_path_by_size(g, a["start"], a["goal"], a["random"], a["firstSize"])
        if name == "getLongestPathLength":
            return pl.get_longest_path_length(g, a["start"], a["goal"])
        if name == "planPathsAdvanced":
            r = pl.plan_paths_advanced(g, a["start"], a["goal"], a["solveRelaxed"], a["extraSteps"], a["setOptimal"],
                                       a["timeLimit"])
        elif name == "planPathsAdvancedSplitNoCycle":
            r = pl.plan_paths_advanced_split_no_cycle(g, a["start"], a["goal"], a["setOptimal"], a["timeLimit"],
                                                      a["splits"], a["top"])
        elif name == "planPathsAdvancedSplitDist":
            r = pl.plan_paths_advanced_split_dist(g, a["start"], a["goal"], a["gap"], a["timeLimit"], a["splits"])
        elif name == "multiThreadedSplitPlanning":
            r = pl.multi_threaded_split_planning(g, a["start"], a["goal"], a["timeLimit"], a["splits"])
            out["extra"] = {"msg": pl.msg, "mpath_keys": sorted(pl.mpath)}
        else:
            raise AssertionError(fn)
        out["stdout"] = pl.log
        return r
    if fn == "PathPlanner.isPathSetValid":
        msg = P.path_set_error(a["paths"], graph_from_json(a["graph"]), a["starts"], a["goals"], COMPAT)
        out["stdout"] = "" if msg is None else msg + "\n"
        return msg is None
    if fn == "PathPlanner.hasCycles":
        out["stdout"] = ""
        return P.has_cycles(a["paths"], COMPAT)
    # ------------------------------------------------------------- heuristic
    if fn.endswith(".planHeuristicPaths"):
        cls = H.HeuristicPathPlanner if fn.startswith("HeuristicPathPlanner") else H.LocalRepairPlanner
        pl = cls(compat=COMPAT, backend=BACKEND, params=GRB, rng=rng)
        res = pl.plan_heuristic_paths(graph_from_json(a["graph"]), a["start"], a["goal"])
        out["stdout"] = res.log
        return res.paths
    # ------------------------------------------------------------- main
    if fn.startswith("Main.solve"):
        base = out["_tmpdir"]
        ex = E.Experiments(base + "/", compat=COMPAT, backend=BACKEND, params=GRB, rng=rng,
                           flags=P.SolverFlags.java_defaults().apply_java(st))
        out["_ex"] = ex
        before = _static_values(ex)
        p = problem_from_json(a["p"])
        name = fn.split(".", 1)[1]
        try:
            if name == "solveProblem":
                r = ex.solve_problem(p, a["solveLP"], a["timeLimit"])
            elif name == "solveProblemPMG":
                r = ex.solve_problem_pmg(p, a["solveLP"], a["timeLimit"])
            elif name == "solveProblemSuboptimal":
                r = ex.solve_problem_suboptimal(p, a["solveLP"], a["setOptimal"], a["extraSteps"], a["timeLimit"],
                                                a["splitLevel"], a["noCycle"])
            elif name == "solveProblemTotalTime":
                r = ex.solve_problem_total_time(p, a["gap"], a["timeLimit"])
            elif name == "solveProblemDistance":
                r = ex.solve_problem_distance(p, a["gap"], a["timeLimit"])
            elif name == "solveProblemDistanceSplit":
                r = ex.solve_problem_distance_split(p, a["gap"], a["timeLimit"], a["splits"])
            elif name == "solveProblemWithHeuristic":
                r = ex.solve_problem_with_heuristic(p)
            else:
                raise AssertionError(fn)
        finally:
            out["stdout"] = ex.log
            after = _static_values(ex)
            out["statics_after"] = {k: v for k, v in after.items() if before[k] != v}
        return None if r is None else [r[0], None, *r[2:]]
    raise AssertionError(f"no mppflow mapping for {fn}")


def _static_values(ex) -> dict:
    d = {k: getattr(ex.flags, v) for k, v in P.SolverFlags._JAVA.items()}
    d["PathFinder.bHeuristic"] = bool(ex.b_heuristic)
    return d


def run_python(case: dict, timeout: float | None = None) -> dict:
    """The port's record for ``case``: {"result" | "exception" | "hang" | "py_error", "stdout", "draws",
    "rng_next", "mutated", "extra", "statics_after", "seconds"}."""
    rng = MathRandom(case["seed"])
    out: dict = {}
    st = case.get("statics") or {}
    old_flag = PF.B_HEURISTIC.value
    PF.B_HEURISTIC.set(bool(st.get("PathFinder.bHeuristic", False)))
    tmp = tempfile.mkdtemp(prefix="mppflow-diff-")
    out["_tmpdir"] = tmp
    t0 = time.time()
    rec: dict = {}
    _SHORTCUTS[0] = 0
    try:
        with _alarm(timeout):
            rec["result"] = _py_call(case, rng, out)
    except OriginalWouldHang as e:
        rec["hang"] = True
        rec["hang_reason"] = str(e)[:300]
    except JavaException as e:
        rec["exception"] = {"class": getattr(e, "java_class", type(e).__name__), "message": str(e)}
        if getattr(e, "output", None) is not None:
            out.setdefault("stdout", e.output)
        elif "_planner" in out:
            out["stdout"] = out["_planner"].log
    except _PyTimeout as e:
        rec["py_timeout"] = str(e)
    except Exception as e:  # a crash of the port that is not a Java emulation
        rec["py_error"] = f"{type(e).__name__}: {e}"
        rec["py_traceback"] = traceback.format_exc()[-3000:]
    finally:
        PF.B_HEURISTIC.set(old_flag)
    rec["seconds"] = round(time.time() - t0, 3)
    rec["shortcuts"] = _SHORTCUTS[0]
    for k in ("stdout", "mutated", "extra", "statics_after"):
        if k in out:
            rec[k] = out[k]
    if "hang" not in rec:
        rec["draws"] = rng.draws
        rec["rng_next"] = rng.next_double()
    if case["fn"].startswith("Main.solve"):
        files = {}
        for f in sorted(os.listdir(tmp)):
            pth = os.path.join(tmp, f)
            if os.path.isfile(pth):
                with open(pth, encoding="utf-8", errors="replace", newline="") as fh:
                    files[f] = fh.read()
        rec["files"] = files
    shutil.rmtree(tmp, ignore_errors=True)
    return rec


# ----------------------------------------------------------------------------------------------
# Comparison
# ----------------------------------------------------------------------------------------------

MODEL_KEYS = ("ncols", "nrows", "nnz", "sense", "edgeVarMap_size", "sha256")
MODEL_PARAMS = ("TimeLimit", "MIPGap", "OutputFlag")


def _jnum_eq(a, b) -> bool:
    """JSON equality with ints == integral floats and Java's "NaN"/"Infinity" strings."""
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_jnum_eq(x, y) for x, y in zip(a, b))
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_jnum_eq(a[k], b[k]) for k in a)
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if isinstance(a, float) and isinstance(b, float) and a != a and b != b:
            return True
        return a == b
    if isinstance(a, str) and isinstance(b, (int, float)) or isinstance(b, str) and isinstance(a, (int, float)):
        x, y = (a, b) if isinstance(a, str) else (b, a)
        return {"NaN": y != y, "Infinity": y == float("inf"), "-Infinity": y == float("-inf")}.get(x, False)
    return a == b


def _model_eq(j: dict, p: dict) -> list[str]:
    bad = [k for k in MODEL_KEYS if j.get(k) != p.get(k)]
    bad += [f"params.{k}" for k in MODEL_PARAMS if not _jnum_eq((j.get("params") or {}).get(k),
                                                                 (p.get("params") or {}).get(k))]
    return bad


def outcome(rec: dict) -> str:
    for k in ("jvm_exit", "py_error", "py_timeout", "hang", "exception"):
        if k in rec:
            return k
    return "result" if "result" in rec else "missing"


def compare(case: dict, j: dict, p: dict) -> list[dict]:
    """Differences between the Java record ``j`` and the port's record ``p`` (empty = match)."""
    nondet = set(case.get("nondet") or ())
    diffs = []

    def diff(field, jv, pv):
        diffs.append({"field": field, "java": _clip(jv), "python": _clip(pv)})

    oj, op = outcome(j), outcome(p)
    if oj != op:
        diff("outcome", {oj: j.get(oj) if oj != "result" else _clip(j.get("result"))},
             {op: p.get(op) if op != "result" else _clip(p.get("result"))})
        return diffs
    if oj == "exception":
        if j["exception"]["class"] != p["exception"]["class"]:
            diff("exception.class", j["exception"]["class"], p["exception"]["class"])
    elif oj == "result" and "result" not in nondet:
        jr, pr = j["result"], p["result"]
        if case["fn"].endswith(".prepareModel"):
            for k in _model_eq(jr, pr):
                diff(f"result.{k}", jr.get(k) if "." not in k else jr.get("params", {}).get(k.split(".")[1]),
                     pr.get(k) if "." not in k else pr.get("params", {}).get(k.split(".")[1]))
        elif not _jnum_eq(jr, pr):
            diff("result", jr, pr)
    if oj in ("hang",):
        return diffs
    if "stdout" not in nondet and "stdout" in p and p["stdout"] is not None:
        if not same_text(p["stdout"], j.get("stdout", "")):
            diff("stdout", j.get("stdout", ""), norm_text(p["stdout"]))
    for k in ("draws", "rng_next"):
        if k not in nondet and k in p and k in j and not _jnum_eq(j[k], p[k]):
            diff(k, j[k], p[k])
    if "mutated" in p and p["mutated"] is not None:
        if not _jnum_eq(j.get("mutated") or {}, p["mutated"]):
            diff("mutated", j.get("mutated") or {}, p["mutated"])
    if "extra" in p and "extra" in j:
        je, pe = j["extra"], p["extra"]
        if not _jnum_eq(je.get("msg"), pe.get("msg")):
            diff("extra.msg", je.get("msg"), pe.get("msg"))
        if "result" not in nondet and not _jnum_eq(je.get("mpath_keys"), pe.get("mpath_keys")):
            diff("extra.mpath_keys", je.get("mpath_keys"), pe.get("mpath_keys"))
    if "statics_after" in p:
        if (j.get("statics_after") or {}) != p["statics_after"]:
            diff("statics_after", j.get("statics_after") or {}, p["statics_after"])
    if "files" in p and j.get("files"):
        for name, f in j["files"].items():
            if not isinstance(f, dict) or "text" not in f:
                continue
            got = p["files"].get(name)
            if got is None or not same_text(got, f["text"]):
                diff(f"files[{name}]", f["text"], got)
    return diffs


_WALLCLOCK_HIT = re.compile(r"time limit reached| time out\.")
_HEURISTIC_FNS = ("PathPlanner.planHeuristicPaths", "HeuristicPathPlanner.planHeuristicPaths",
                  "Main.solveProblemWithHeuristic")


def skip_reason(case: dict, j: dict, p: dict) -> str | None:
    """A documented reason why a difference is not a port bug, or None.

    * ``time-limited-endless-loop``: the original hung (budget exceeded) in a T++ loop on an
      instance the port proved infeasible under a time limit; the port returns the original's
      eventual ``null`` at once (mppflow.solvers docstring, "Infinite loops") and its callers
      continue (here a JavaNullPointerException is the original's eventual outcome, too).
    * ``wallclock``: the local-repair heuristic's attempt budgets bound in the Java run (its
      output says "time limit reached" / "time out", or it hung); the port runs the heuristic
      on a frozen clock (an infinitely fast machine), see ``docs/notes/original-bugs.md`` B26.
    * ``mt-hashmap-race``: ``multiThreadedSplitPlanning`` returned null in Java, and Java's
      ``mpath`` lacks some keys of the port's with nothing printed, while every piece succeeded
      in the port (or the port's result is null too, with the same messages: a piece failed in
      both runs and Java also lost another piece): worker threads finishing at the same moment
      lose a ``put`` into the unsynchronized ``HashMap mpath`` (original-bugs B14, item 4).
      Rerunning the Java case usually gives the port's result. The same race reached through
      ``Main.solveProblemSuboptimal`` (split level > 0, ``noCycle`` false) shows as a
      ``NullPointerException`` in ``PathPlanner.printPaths`` called from
      ``Main.solveProblemSuboptimal`` (B21) while the port returns a plan.
    """
    if case["fn"] in _HEURISTIC_FNS and (j.get("hang") or _WALLCLOCK_HIT.search(j.get("stdout") or "")):
        return "wallclock"
    if j.get("hang") and not p.get("hang") and p.get("shortcuts"):
        return "time-limited-endless-loop"
    if case["fn"] == "PathPlanner.multiThreadedSplitPlanning" and j.get("result", 0) is None and "extra" in j \
            and "extra" in p and not j.get("stderr") and not j.get("stdout"):
        pieces = 2 ** case["args"]["splits"]
        jk, pk = set(j["extra"].get("mpath_keys") or ()), set(p["extra"].get("mpath_keys") or ())
        if jk < pk and (pk == set(range(pieces))
                        # a piece failed in both runs (result null in both, same messages) and
                        # Java also lost the put of a piece that succeeded in the port
                        or (p.get("result", 0) is None
                            and _jnum_eq(j["extra"].get("msg"), p["extra"].get("msg")))):
            return "mt-hashmap-race"
    if case["fn"] == "Main.solveProblemSuboptimal" and (case["args"].get("splitLevel") or 0) > 0 \
            and not case["args"].get("noCycle") and p.get("result") is not None:
        exc = j.get("exception") or {}
        at = exc.get("at") or ()
        if exc.get("class") == "java.lang.NullPointerException" and at \
                and at[0].startswith("PathPlanner.printPaths:") \
                and any(f.startswith("Main.solveProblemSuboptimal:") for f in at):
            return "mt-hashmap-race"
    return None


def _clip(v, n=1500):
    s = json.dumps(v, default=str)
    return v if len(s) <= n else s[:n] + f"...<{len(s) - n} more chars>"


# ----------------------------------------------------------------------------------------------
# Java side
# ----------------------------------------------------------------------------------------------

def java_setup(args) -> tuple[str, str, str]:
    """Returns (java executable, classpath, gurobi lib dir); builds the harness if needed."""
    jar = os.path.abspath(args.gurobi_jar) if args.gurobi_jar else None
    if not jar or not os.path.isfile(jar):
        raise SystemExit(f"gurobi.jar not found: {jar} (use --gurobi-jar or GUROBI_HOME)")
    java = os.path.join(args.jdk, "bin", "java") if args.jdk else "java"
    javac = os.path.join(args.jdk, "bin", "javac") if args.jdk else "javac"
    classes = args.classes
    if not classes:
        if not args.original_src:
            raise SystemExit("need --classes (a build.sh output) or --original-src")
        out = os.path.join(args.work, "classes")
        env = dict(os.environ, ORIGINAL_SRC=os.path.abspath(args.original_src), GUROBI_JAR=jar, JAVAC=javac, OUT=out)
        r = subprocess.run(["bash", str(HERE / "build.sh")], env=env, capture_output=True, text=True)
        if r.returncode != 0:
            sys.stderr.write(r.stderr)
            raise SystemExit("build.sh failed")
        classes = r.stdout.strip().splitlines()[-1]
    if not os.path.isfile(os.path.join(classes, "projects", "multipath", "ILP", "Harness.class")):
        raise SystemExit(f"{classes} has no projects/multipath/ILP/Harness.class")
    return java, os.path.abspath(classes) + os.pathsep + jar, os.path.dirname(jar)


def run_java(cases: list[dict], args) -> dict[str, dict]:
    java, cp, libdir = java_setup(args)
    rundir = Path(args.work) / "run"
    if rundir.exists():
        shutil.rmtree(rundir)
    rundir.mkdir(parents=True)
    (rundir / "gurobi.env").write_text("Threads 1\n")
    with open(Path(args.work) / "harness.log", "w") as log:
        raw = record.Runner(java, cp, libdir, rundir, log).run(cases)
    return {c["id"]: record.assemble(c, raw[c["id"]]) for c in cases if c["id"] in raw}


# ----------------------------------------------------------------------------------------------
# Driver
# ----------------------------------------------------------------------------------------------

def run(cases: list[dict], java: dict[str, dict], log=sys.stderr, py_timeout_factor: float = 3.0) -> dict:
    groups: dict[str, dict] = {}
    mismatches, skipped = [], []
    for i, case in enumerate(cases):
        grp = groups.setdefault(case["group"], {"cases": 0, "compared": 0, "match": 0, "mismatch": 0, "skipped": 0,
                                                "java_hang": 0, "java_exception": 0, "harness_error": 0,
                                                "python_seconds": 0.0})
        grp["cases"] += 1
        j = java.get(case["id"])
        if j is None or "jvm_exit" in j:
            grp["harness_error"] += 1
            mismatches.append({"id": case["id"], "group": case["group"], "fn": case["fn"],
                               "diffs": [{"field": "harness", "java": j and j.get("jvm_exit"), "python": None}],
                               "repro": _repro(case)})
            continue
        budget = case.get("budget", 120)
        p = run_python(case, timeout=max(60.0, py_timeout_factor * budget))
        grp["python_seconds"] = round(grp["python_seconds"] + p["seconds"], 3)
        grp["compared"] += 1
        grp["java_hang"] += bool(j.get("hang"))
        grp["java_exception"] += "exception" in j
        d = compare(case, j, p)
        why = skip_reason(case, j, p) if d else None
        if why:
            grp["skipped"] += 1
            skipped.append({"id": case["id"], "group": case["group"], "fn": case["fn"], "reason": why,
                            "diffs": [x["field"] for x in d]})
            print(f"  skipped {case['id']}: {why} {[x['field'] for x in d]}", file=log)
        elif d:
            grp["mismatch"] += 1
            m = {"id": case["id"], "group": case["group"], "fn": case["fn"], "diffs": d, "repro": _repro(case)}
            if "py_traceback" in p:
                m["python_traceback"] = p["py_traceback"]
            mismatches.append(m)
            print(f"  MISMATCH {case['id']}: {[x['field'] for x in d]}", file=log)
        else:
            grp["match"] += 1
        if (i + 1) % 25 == 0:
            print(f"  python: {i + 1}/{len(cases)} cases", file=log)
    return {"groups": groups, "mismatches": mismatches, "skipped": skipped}


def _repro(case):
    return {k: case[k] for k in ("fn", "args", "seed", "statics", "nondet") if k in case}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=30, help="cases per group (default 30)")
    ap.add_argument("--seed", type=int, default=1, help="seed of the input generator (default 1)")
    ap.add_argument("--groups", default=",".join(GROUPS), help=f"comma-separated subset of {GROUPS}")
    ap.add_argument("--ids", help="regular expression: run only the matching case ids")
    ap.add_argument("--report", help="write the JSON report here")
    ap.add_argument("--classes", default=os.environ.get("MPPFLOW_JAVA_HARNESS"),
                    help="compiled harness (build.sh output; default $MPPFLOW_JAVA_HARNESS)")
    ap.add_argument("--original-src", help="original 'source' directory (builds the harness when --classes is unset)")
    ap.add_argument("--gurobi-jar", default=os.environ.get("GUROBI_JAR") or (
        os.path.join(os.environ["GUROBI_HOME"], "lib", "gurobi.jar") if os.environ.get("GUROBI_HOME") else None))
    ap.add_argument("--jdk", default=os.environ.get("MPPFLOW_JDK"), help="JDK home (default: java on PATH)")
    ap.add_argument("--work", help="work directory (default: a temporary directory)")
    ap.add_argument("--java-results", help="cache of the Java records (JSON): read if it exists, else written")
    ap.add_argument("--list", action="store_true", help="only print the generated case ids")
    args = ap.parse_args(argv)

    groups = [g for g in args.groups.split(",") if g]
    bad = [g for g in groups if g not in GROUPS]
    if bad:
        ap.error(f"unknown groups {bad}; known: {GROUPS}")
    t0 = time.time()
    cases = generate(groups, args.n, args.seed)
    if args.ids:
        cases = [c for c in cases if re.search(args.ids, c["id"])]
    if args.list:
        for c in cases:
            print(c["id"])
        return 0
    print(f"{len(cases)} case(s) in {len(groups)} group(s), generated in {time.time() - t0:.1f}s", file=sys.stderr)

    tmp_work = None
    if not args.work:
        tmp_work = args.work = tempfile.mkdtemp(prefix="mppflow-differential-")
    # the harness runs java with cwd = <work>/run: relative paths would resolve there
    args.work = os.path.abspath(args.work)
    if args.jdk:
        args.jdk = os.path.abspath(args.jdk)
    os.makedirs(args.work, exist_ok=True)
    java = None
    if args.java_results and os.path.exists(args.java_results):
        with open(args.java_results, encoding="utf-8") as f:
            cached = json.load(f)
        java = {c["id"]: cached[c["id"]] for c in cases if c["id"] in cached}
        missing = [c for c in cases if c["id"] not in java]
        if missing:
            java.update(run_java(missing, args))
    else:
        t1 = time.time()
        java = run_java(cases, args)
        print(f"java: {len(java)} record(s) in {time.time() - t1:.1f}s", file=sys.stderr)
    if args.java_results:
        old = {}
        if os.path.exists(args.java_results):
            with open(args.java_results, encoding="utf-8") as f:
                old = json.load(f)
        old.update(java)
        with open(args.java_results, "w", encoding="utf-8") as f:
            json.dump(old, f)

    t2 = time.time()
    rep = run(cases, java)
    rep["meta"] = {"n": args.n, "seed": args.seed, "groups": groups, "ids": args.ids, "cases": len(cases),
                   "compat": COMPAT, "backend": BACKEND, "params": GRB, "python_seconds": round(time.time() - t2, 1),
                   "total_seconds": round(time.time() - t0, 1)}
    tot = {k: sum(g[k] for g in rep["groups"].values()) for k in ("cases", "compared", "match", "mismatch",
                                                                  "skipped", "harness_error")}
    rep["totals"] = tot
    for name, g in rep["groups"].items():
        print(f"  {name:12s} compared {g['compared']:4d}  match {g['match']:4d}  mismatch {g['mismatch']:3d}  "
              f"skipped {g['skipped']:2d}  "
              f"(java hangs {g['java_hang']}, exceptions {g['java_exception']}, harness errors "
              f"{g['harness_error']})", file=sys.stderr)
    print(f"  total        compared {tot['compared']:4d}  match {tot['match']:4d}  mismatch {tot['mismatch']:3d}  "
          f"skipped {tot['skipped']:2d}",
          file=sys.stderr)
    if args.report:
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump(rep, f, indent=1)
    if tmp_work:
        shutil.rmtree(tmp_work, ignore_errors=True)
    return 0 if tot["mismatch"] == 0 and tot["harness_error"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
