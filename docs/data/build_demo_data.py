#!/usr/bin/env python3
"""Build ``docs/data/demo-data.js``: the maps and precomputed plans of the browser demo.

Usage (from the repository root)::

    python3 docs/data/build_demo_data.py                 # every map (about 15-30 minutes)
    python3 docs/data/build_demo_data.py --only tor16-fig8-split   # rebuild one map, keep the rest
    python3 docs/data/build_demo_data.py --only tor16-fig9-24x18-seed1 --plan total_distance/k4
                                                         # rebuild one plan of a map, keep the rest
    python3 docs/data/build_demo_data.py --list          # the map ids
    python3 docs/data/build_demo_data.py --refresh-text  # refresh the map texts only, keep every plan

The output sets ``window.MPP_DEMO`` (see ``docs/data/README.md`` for the format). Every
plan is computed with the public ``mppflow`` API (``mppflow.plan``), validated with
``mppflow.validate`` / ``mppflow.is_path_set_valid``, and measured with
``mppflow.plan_metrics`` and ``mppflow.has_cycles``. The ILP backend is Gurobi when
gurobipy is installed, else HiGHS (scipy); each plan records the backend it used.

A plan is marked ``live`` when the browser can re-solve it: the same ``mppflow.plan`` call
with ``backend="builtin"`` (the pure-Python solver that ``docs/js/lp.js`` ports line by line,
pivot for pivot) reaches the same objective value within ``--live-work`` simplex iterations
per model (default 100000; a deterministic budget, so the flag does not depend on the
machine). Only models with at most ``--live-columns`` columns are tried.

The instances are either transcribed from the T-RO 2016 paper (``tests/data/paper``) or
regenerated from seeds with ``mppflow``'s ports of the original generators (the original
benchmark files are lost). Timings (``seconds``) depend on the machine; everything else is
deterministic for a given backend version.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

import mppflow  # noqa: E402
from mppflow.model import (  # noqa: E402
    build_distance_model,
    build_max_distance_model,
    build_time_model,
    build_total_time_model,
)

OUT = os.path.join(HERE, "demo-data.js")
PAPER = os.path.join(REPO, "tests", "data", "paper")
FORMAT_VERSION = 1
GENERATOR = "docs/data/build_demo_data.py"
MAX_BYTES = 1_500_000

OBJECTIVES = ("makespan", "total_time", "total_distance", "max_distance")
_BUILDERS = {
    "makespan": build_time_model,
    "total_time": build_total_time_model,
    "total_distance": build_distance_model,
    "max_distance": build_max_distance_model,
}


def _paper(name: str) -> dict:
    with open(os.path.join(PAPER, name + ".json"), encoding="utf-8") as f:
        return json.load(f)


def _configs_to_paths(cfgs):
    """``cfg[t][i]`` (robot ``i``'s vertex at time ``t``) -> ``paths[i][t]``."""
    return [[c[i] for c in cfgs] for i in range(len(cfgs[0]))]


# ---------------------------------------------------------------------------- maps
# Each map: id, title, caption, source, problem, rows/cols (None for a non-grid graph),
# optional generator (the call that regenerates the instance), optional paper plan, and
# jobs = [(objective, k, options)].

ALL4 = [(o, 1, {}) for o in OBJECTIVES]
SINGLE = {"horizon": "single",
          "note": "Solved at one horizon, T = the optimal makespan: optimal among plans of that length; "
                  "the search over longer horizons (Alg. 1) is too slow here. (The original also solves at "
                  "one horizon, but at the makespan of a 4-way split.)"}


def map_9puzzle():
    d = _paper("tor16_fig1_fig3_9puzzle")
    java = _paper("tor16_java_original_outputs")["fig1_fig3_9puzzle"]
    g = mppflow.create_2d_grid_graph(3, 3)
    return {
        "id": "tor16-fig1-9puzzle",
        "title": "9-puzzle",
        "figure": "Figs. 1, 3",
        "caption": ("Nine robots fill a 3 × 3 grid (Fig. 1(a)); robot i must reach the i-th vertex in "
                    "reading order (Fig. 1(b), the goal configuration below the view). Every move "
                    "is a rotation along a fully occupied cycle. The optimal makespan is 4 (robot 9 is "
                    "four steps from its goal), and exactly four optimal plans exist; the paper's "
                    "Fig. 3 plan is one of them."),
        "source": "T-RO 2016 Fig. 1 (instance) and Fig. 3 (4-step plan), p. 1165; transcribed in "
                  "tests/data/paper/tor16_fig1_fig3_9puzzle.json",
        "problem": mppflow.Problem(g, d["starts"], d["goals"]),
        "rows": 3, "cols": 3,
        "paper": {"paths": _configs_to_paths(d["fig3_configs_robot_to_vertex"]),
                  "note": "Fig. 3 of T-RO 2016, read arrow by arrow (4 rotations). The original Java "
                          "code with Gurobi 13.0.2 returns this same plan."},
        "java_plan": java["paths"],
        "jobs": ALL4 + [("makespan", 2, {}), ("makespan", 4, {})],
    }


def _pretty_name(nm: str) -> str:
    """'s1-' -> 's₁⁻' (the paper's notation)."""
    m = re.fullmatch(r"([a-z])(\d)([+-])", nm)
    if not m:
        return nm
    sub = "₀₁₂₃₄₅₆₇₈₉"[int(m.group(2))]
    return m.group(1) + sub + ("⁺" if m.group(3) == "+" else "⁻")


def map_tree():
    d = _paper("tor16_fig4_fig7_network")
    names = [_pretty_name(nm) for nm in d["graph"]["vertex_names"]]
    n = d["graph"]["num_vertices"]
    adj = [[] for _ in range(n)]
    for u, v in d["graph"]["edges"]:
        adj[u].append(v)
        adj[v].append(u)
    # Fig. 4(a) is drawn as a vertical trunk s1- / A / B / s2+ with two branches to the left.
    xy = [(1, 0), (0, 1), (1, 1), (0, 2), (1, 2), (1, 3)]
    g = mppflow.from_adjacency(adj, xy)
    return {
        "id": "tor16-fig4a-tree",
        "title": "Two robots on a tree",
        "figure": "Fig. 4(a)",
        "caption": ("Robot r₁ goes from s₁⁺ to s₁⁻ and r₂ from s₂⁺ to s₂⁻ on the tree of Fig. 4(a) (vertices "
                    + ", ".join(f"{i} = {nm}" for i, nm in enumerate(names))
                    + "). Both shortest paths have length 3 but share the trunk, so the optimal "
                    "makespan is 4; the T = 2 network that Fig. 5 draws is infeasible."),
        "source": "T-RO 2016 Fig. 4(a) (graph), Figs. 5-7 (time-expanded networks), pp. 1166-1169; "
                  "tests/data/paper/tor16_fig4_fig7_network.json",
        "problem": mppflow.Problem(g, d["starts"], d["goals"]),
        "rows": None, "cols": None,
        "vertex_names": names,
        "jobs": ALL4 + [("makespan", 2, {})],
    }


def map_fig8():
    d = _paper("tor16_fig8_split")
    g = mppflow.create_2d_grid_graph(4, 4)
    return {
        "id": "tor16-fig8-split",
        "title": "Split example",
        "figure": "Fig. 8",
        "caption": ("Two robots on a 4 × 4 grid (3 × 3 cells, Fig. 8). The 2-way split puts the "
                    "intermediate goals at the middles of the shortest paths (vertices 5 and 10). "
                    "Unsplit, the optimal makespan is 4; split, the two halves need 3 + 2 = 5 steps "
                    "(the paper says 2 + 2, see docs/notes/paper-errata.md)."),
        "source": "T-RO 2016 Fig. 8 and Sec. V-B, p. 1170; tests/data/paper/tor16_fig8_split.json",
        "problem": mppflow.Problem(g, d["starts"], d["goals"]),
        "rows": 4, "cols": 4,
        "jobs": ALL4 + [("makespan", 2, {})],
    }


def map_star():
    from mppflow.graph import create_main_case3_graph

    g0 = create_main_case3_graph()
    xy = [(0, 1), (1, 1), (1, 0), (2, 1)]
    g = mppflow.from_adjacency([[u for u in g0.adj[v] if u != v] for v in range(g0.n)], xy)
    return {
        "id": "main-case3-star",
        "title": "Star swap",
        "caption": ("Two robots swap the ends 0 and 3 of a path through the hub 1; the spur 2 is the "
                    "only place to step aside. Optimal makespan 4."),
        "source": "the original code's Main case 3 (star graph 0–1, 1–2, 1–3, robots 0 → 3 and 3 → 0); "
                  "mppflow.graph.create_main_case3_graph",
        "problem": mppflow.Problem(g, [0, 3], [3, 0]),
        "rows": None, "cols": None,
        "jobs": ALL4 + [("makespan", 2, {})],
    }


def map_16puzzle(seed=1):
    p = mppflow.create_n2_puzzle(4, mppflow.MathRandom(seed))
    return {
        "id": f"random-16puzzle-seed{seed}",
        "title": "Random 16-puzzle",
        "caption": ("Sixteen robots fill a 4 × 4 grid: they start in order and their goals are a random "
                    "permutation (the goal configuration below the view). T-RO Sec. VI-B reports that such "
                    "instances generally need 6 steps; this one needs 7 (the ILP proves 7 optimal, while the "
                    "shortest-path bound is 5)."),
        "source": f"regenerated, seed {seed}: mppflow.create_n2_puzzle(4, MathRandom({seed})) "
                  "(port of the original Problem.createN2Puzzle)",
        "generator": {"fn": "createN2Puzzle", "args": [4], "seed": seed},
        "problem": p,
        "rows": 4, "cols": 4,
        "jobs": [("makespan", 1, {}), ("makespan", 2, {}), ("makespan", 4, {}), ("max_distance", 1, {}),
                 ("total_time", 1, SINGLE), ("total_distance", 1, SINGLE)],
    }


def map_25puzzle():
    d = _paper("tor16_fig14_fig15_25puzzle")
    java = _paper("tor16_java_original_outputs")["fig14_fig15_25puzzle"]
    g = mppflow.create_2d_grid_graph(5, 5)
    return {
        "id": "tor16-fig14-25puzzle",
        "title": "25-puzzle",
        "figure": "Figs. 14, 15",
        "caption": ("Twenty-five robots fill a 5 × 5 grid (Fig. 14). Robot 10 is seven steps from its "
                    "goal, so 7 is a lower bound, and the paper's 7-step plan (Fig. 15) attains it. The k-way "
                    "split does not pay off here: with every vertex occupied, each piece again has to "
                    "solve a full permutation, so the four pieces of k = 4 need 4 + 4 + 7 + 7 = 22 steps "
                    "against the optimal 7. The split is meant for large instances, where the pieces are "
                    "much smaller models than the whole (Sec. VI)."),
        "source": "T-RO 2016 Fig. 14 (instance) and Fig. 15 (7-step plan), p. 1173; transcribed in "
                  "tests/data/paper/tor16_fig14_fig15_25puzzle.json",
        "problem": mppflow.Problem(g, d["starts"], d["goals"]),
        "rows": 5, "cols": 5,
        "paper": {"paths": _configs_to_paths(d["fig15_configs_robot_to_vertex"]),
                  "note": (f"Fig. 15 of T-RO 2016, transcribed from its {sum(map(len, d['fig15_moves_rowcol']))} arrows "
                           f"({len(d['fig15_moves_rowcol'])} panels). Valid and "
                          "optimal; the original Java code returns a different 7-step plan.")},
        "java_plan": java["paths"],
        "jobs": [("makespan", 1, {}), ("makespan", 2, {}), ("makespan", 4, {})],
    }


def map_crowded(seed=1):
    p = mppflow.create_grid_problem(8, 8, 0.0, 40, mppflow.MathRandom(seed))
    return {
        "id": f"crowded-8x8-40-seed{seed}",
        "title": "Crowded 8 × 8 grid",
        "caption": ("Forty robots on an 8 × 8 grid without obstacles (62.5% of the vertices occupied). "
                    "Splitting the horizon into k pieces trades optimality for much smaller models."),
        "source": f"regenerated, seed {seed}: mppflow.create_grid_problem(8, 8, 0.0, 40, MathRandom({seed})) "
                  "(port of the original Problem.createGridProblem)",
        "generator": {"fn": "createGridProblem", "args": [8, 8, 0.0, 40], "seed": seed},
        "problem": p,
        "rows": 8, "cols": 8,
        "jobs": [("makespan", 1, {}), ("makespan", 2, {}), ("makespan", 4, {}), ("max_distance", 1, {})],
    }


def map_fig9(seed=1):
    p = mppflow.create_grid_problem(24, 18, 0.25, 50, mppflow.MathRandom(seed))
    if seed == 1:  # the instance the original Java generator produced with Math.random seeded 1
        rec = _paper("tor16_java_original_outputs")["fig9_generator"]
        assert [list(p.graph.xy[v]) for v in range(p.graph.n)] == rec["seed1_coords_xy"]
        assert list(p.starts) == rec["seed1_starts"] and list(p.goals) == rec["seed1_goals"]
    return {
        "id": f"tor16-fig9-24x18-seed{seed}",
        "title": "24 × 18 grid, 50 robots",
        "figure": "as in Fig. 9",
        "caption": ("A grid with 25% of its vertices removed (connectivity kept) and 50 robots, like the "
                    "T-RO experiments (Fig. 9, Sec. VI-A); starts may overlap goals. As in the original "
                    "generator, the grid has 24 rows and 18 columns (Fig. 9 draws it transposed). "
                    "Makespan plans for k = 1, 2, 4, 8, 16 show the cost of the k-way split; total distance is "
                    "planned with a 4-way split and total time at a single horizon."),
        "source": (f"regenerated, seed {seed}: mppflow.create_grid_problem(24, 18, 0.25, 50, MathRandom({seed})), "
                   "the port of the original Problem.createGridProblem"
                   + ("; identical to the original Java generator's instance with Math.random seeded 1 "
                      "(tests/data/paper/tor16_java_original_outputs.json)" if seed == 1 else "")
                   + ". The paper's own benchmark instances are lost."),
        "generator": {"fn": "createGridProblem", "args": [24, 18, 0.25, 50], "seed": seed},
        "problem": p,
        "rows": 24, "cols": 18,
        "jobs": [("makespan", k, {}) for k in (1, 2, 4, 8, 16)]
        + [("total_distance", 4, {"note": "4-way split with minimum-total-distance pieces (T-RO Sec. V-B "
                                          "remark; the original's planPathsAdvancedSplitDist)."}),
           ("total_time", 1, {"horizon": "single", "time_limit": 2400.0, "optional": True,
                              # plan() with gap 0 returns a single-horizon plan only when the solver proved
                              # it optimal at that T (tt_return_any_solution is off): a run stopped by the
                              # limit gives no plan, and the job is skipped
                              "note": "Solved at one horizon, T = the optimal makespan: optimal among plans of "
                                      "that length (proved within the job's 2400 s limit); the search over "
                                      "longer horizons (Alg. 1) is too slow here."})],
    }


MAP_FUNCS = [map_9puzzle, map_tree, map_fig8, map_star, map_16puzzle, map_25puzzle, map_crowded, map_fig9]


# --------------------------------------------------------------------------- plans
def _config(paths, t):
    return [p[t] for p in paths]


def _pieces(paths, starts, goals, inter):
    """The pieces of a k-way split plan: the configurations ``[starts] + inter + [goals]``
    and the time at which the merged plan reaches each one. Each piece is optimal for its
    own endpoints, so the first time a configuration appears after the previous boundary
    is its boundary."""
    cfgs = [list(starts)] + [list(c) for c in inter] + [list(goals)]
    T = len(paths[0]) - 1
    times = [0]
    for c in cfgs[1:]:
        t = times[-1]
        while t <= T and _config(paths, t) != c:
            t += 1
        if t > T:
            raise AssertionError("split plan does not pass through its intermediate configurations")
        times.append(t)
    if times[-1] != T:
        raise AssertionError("split plan is longer than its pieces")
    return cfgs, times


def _model_size(objective, g, s, goals, T):
    m = _BUILDERS[objective](g, s, goals, T).model
    return {"columns": m.num_columns, "rows": m.num_rows}


def _check_plan(problem, paths, what):
    err = mppflow.validate(problem, paths)
    ok = mppflow.is_path_set_valid(paths, problem.graph, list(problem.starts), list(problem.goals))
    if err is not None or not ok:
        raise AssertionError(f"{what}: invalid plan: {err}")


def _metrics(paths, goals):
    m = mppflow.plan_metrics(paths, goals)
    # plan_metrics takes the makespan as len - 1; T-RO Sec. II-B defines it as the latest arrival
    # time. They agree when the plan does not end with steps in which every robot only waits.
    last = max((_arrival(p, g) for p, g in zip(paths, goals)), default=0)
    if m["makespan"] != last:
        raise AssertionError(f"plan ends with {m['makespan'] - last} all-wait steps (makespan {m['makespan']}, "
                             f"latest arrival {last}); trim it")
    return {"makespan": m["makespan"], "totalTime": m["total_time"],
            "totalDistance": m["total_distance"], "maxDistance": m["max_distance"]}


def _arrival(path, goal):
    """The first time after which the robot stays at its goal."""
    t = len(path) - 1
    if path[t] != goal:
        return t
    while t > 0 and path[t - 1] == goal:
        t -= 1
    return t


def solve_plan(spec, objective, k, opts, backend, args, log):
    p = spec["problem"]
    g, s, goals = p.graph, list(p.starts), list(p.goals)
    key = f"{objective}/k{k}"
    kw = {"horizon": opts.get("horizon"), "gap": opts.get("gap", 0.0)}
    t0 = time.time()
    r = mppflow.plan(p, objective, k, backend=backend, time_limit=opts.get("time_limit", args.time_limit),
                     rng=mppflow.MathRandom(args.seed), **kw)
    secs = time.time() - t0
    if r.paths is None:
        raise RuntimeError(f"{spec['id']} {key}: no plan ({r.status}); raise --time-limit")
    _check_plan(p, r.paths, f"{spec['id']} {key}")
    T = len(r.paths[0]) - 1
    # plan() reports a single horizon above the lower bound as "feasible" (optimal at that T only)
    status = r.status
    out = {
        "objective": objective,
        "k": k,
        "status": status,
        "value": r.value,
        "paths": r.paths,
        "metrics": _metrics(r.paths, goals),
        "rotations": mppflow.has_cycles(r.paths),
        "backend": backend,
        "seconds": round(secs, 3),
        "solves": r.stats["solves"],
        "horizons": r.stats["horizons"],
        "phases": r.stats["phases"],
        "rng_seed": args.seed,
    }
    if kw["horizon"] is not None:
        out["horizon"] = kw["horizon"]
    if kw["gap"]:
        out["gap"] = kw["gap"]
    if opts.get("note"):
        out["note"] = opts["note"]
    if k == 1:
        out["model"] = dict(_model_size(objective, g, s, goals, T), T=T)
    elif not r.intermediate_goals:
        out["model"] = None
    else:
        # the pieces of a split (a total-distance split: each piece's distance model, at the
        # makespan its piece's makespan solve found)
        cfgs, times = _pieces(r.paths, s, goals, r.intermediate_goals)
        pieces = []
        for i in range(k):
            Ti = times[i + 1] - times[i]
            pieces.append(dict(_model_size(objective, g, cfgs[i], cfgs[i + 1], Ti), T=Ti, t0=times[i]))
        out["intermediate_goals"] = r.intermediate_goals
        out["pieces"] = pieces
        big = max(pieces, key=lambda q: q["columns"])
        out["model"] = {"columns": big["columns"], "rows": big["rows"], "T": big["T"], "largest_piece": True}
    if spec.get("java_plan") is not None and key == "makespan/k1":
        out["matches_original_java"] = r.paths == spec["java_plan"]
    # live: can the pure-Python builtin solver (ported to docs/js/lp.js) re-solve it?
    live = False
    check = None
    if args.live_work > 0 and out["model"] is not None and out["model"]["columns"] <= args.live_columns:
        t1 = time.time()
        rb = mppflow.plan(p, objective, k, backend="builtin", params={"work_limit": args.live_work},
                          rng=mppflow.MathRandom(args.seed), **kw)
        bsecs = time.time() - t1
        live = rb.paths is not None and rb.value == r.value
        if rb.paths is not None:
            _check_plan(p, rb.paths, f"{spec['id']} {key} (builtin)")
        check = {"backend": "builtin", "work_limit": args.live_work, "status": rb.status, "value": rb.value,
                 "python_seconds": round(bsecs, 3)}
    out["live"] = live
    if check is not None:
        out["live_check"] = check
    log(f"  {key:22s} {status:9s} value={r.value} T={T} cols={(out['model'] or {}).get('columns')} "
        f"{secs:.2f}s live={live}" + (f" (builtin {check['status']} {check['python_seconds']}s)" if check else ""))
    return key, out


def build_map(spec, backend, args, log, old=None):
    p = spec["problem"]
    g, s, goals = p.graph, list(p.starts), list(p.goals)
    log(f"{spec['id']}: n={g.n}, robots={len(s)}")
    xy = []
    for v in range(g.n):
        pt = g.xy.get(v)
        xy.append(None if pt is None else [pt[0], pt[1]])
    lb = _lower_bounds(g, s, goals)
    plans = {}
    skipped = []
    for objective, k, opts in spec["jobs"]:
        key0 = f"{objective}/k{k}"
        if args.plan and key0 not in args.plan:
            if old is not None and key0 in old["plans"]:
                plans[key0] = old["plans"][key0]
                log(f"  {key0:22s} kept from {args.out}")
            continue
        try:
            key, out = solve_plan(spec, objective, k, opts, backend, args, log)
        except RuntimeError as e:
            if not opts.get("optional"):
                raise
            log(f"  {objective}/k{k}: skipped ({e})")
            skipped.append({"key": f"{objective}/k{k}", "reason": str(e)})
            continue
        plans[key] = out
    if spec.get("paper"):
        pp = spec["paper"]["paths"]
        _check_plan(p, pp, f"{spec['id']} paper plan")
        plans["paper"] = {"paths": pp, "note": spec["paper"]["note"], "metrics": _metrics(pp, goals),
                          "rotations": mppflow.has_cycles(pp), "live": False}
    m = {
        "id": spec["id"],
        "title": spec["title"],
        "caption": spec["caption"],
        "source": spec["source"],
        "graph": {"n": g.n, "adj": g.adjacency_lists(), "xy": xy, "rows": spec["rows"], "cols": spec["cols"]},
        "starts": s,
        "goals": goals,
        "lower_bounds": lb,
        "live": bool(plans.get("makespan/k1", {}).get("live")),
        "live_plans": [k for k, v in plans.items() if v.get("live")],
        "plans": plans,
    }
    if skipped:
        m["skipped"] = skipped
    if spec.get("generator"):
        m["generator"] = spec["generator"]
    if spec.get("vertex_names"):
        m["vertex_names"] = spec["vertex_names"]
    if spec.get("figure"):
        m["figure"] = spec["figure"]
    return m


def _lower_bounds(g, s, goals):
    """BFS lower bounds (the same values as ``PlanResult.lower_bounds``): the longest and
    the sum of the shortest-path lengths. Uses ``get_total_time_lower_bound`` /
    ``get_makespan_lower_bound`` (the original's functions) with a private stream, so no
    global random state is touched."""
    rng = mppflow.MathRandom(0)
    mk = mppflow.get_makespan_lower_bound(g, s, goals, rng) - 1
    tt = mppflow.get_total_time_lower_bound(g, s, goals, rng)
    return {"makespan": mk, "total_time": tt, "total_distance": tt, "max_distance": mk}


# ---------------------------------------------------------------------------- I/O
_PREFIX = "/* Generated by docs/data/build_demo_data.py -- do not edit by hand. */\n"


def _render(data: dict) -> str:
    """One map per line keeps the file diffable and small."""
    head = {k: v for k, v in data.items() if k != "maps"}
    body = ",\n".join(json.dumps(m, separators=(",", ":")) for m in data["maps"])
    head_json = json.dumps(head, separators=(",", ":"))[:-1]
    return (_PREFIX
            + "(function (root) {\n"
            + "  root.MPP_DEMO = " + head_json + ',"maps":[\n' + body + "\n]};\n"
            + "  if (typeof module !== 'undefined' && module.exports) { module.exports = root.MPP_DEMO; }\n"
            + "})(typeof window !== 'undefined' ? window : globalThis);\n")


def _read_existing(path: str) -> dict | None:
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        text = f.read()
    mt = re.search(r"root\.MPP_DEMO = (.*?);\n  if \(typeof module", text, re.S)
    if not mt:
        return None
    return json.loads(mt.group(1))


def _refresh_text(specs, out, log) -> int:
    """Rewrite the texts of the maps in ``out`` from ``specs`` and keep the plans as they are."""
    old = _read_existing(out)
    if old is None:
        raise SystemExit(f"{out} not found or not readable; build it first")
    by_id = {sp["id"]: sp for sp in specs}
    for m in old["maps"]:
        sp = by_id.get(m["id"])
        if sp is None:
            log(f"{m['id']}: no longer built by this script, kept as is")
            continue
        g = sp["problem"]
        if list(g.starts) != m["starts"] or list(g.goals) != m["goals"]:
            raise SystemExit(f"{m['id']}: the instance changed; rebuild it with --only {m['id']}")
        for key in ("title", "caption", "source"):
            m[key] = sp[key]
        if sp.get("figure"):
            m["figure"] = sp["figure"]
        else:
            m.pop("figure", None)
        if sp.get("vertex_names"):
            m["vertex_names"] = sp["vertex_names"]
        if sp.get("paper") and "paper" in m["plans"]:
            m["plans"]["paper"]["note"] = sp["paper"]["note"]
        for objective, k, opts in sp["jobs"]:
            pl = m["plans"].get(f"{objective}/k{k}")
            if pl is not None:
                if opts.get("note"):
                    pl["note"] = opts["note"]
                else:
                    pl.pop("note", None)
        for pl in m["plans"].values():
            if pl.get("paths"):
                _metrics(pl["paths"], m["goals"])      # the trimmed-plan check
        log(f"{m['id']}: texts refreshed")
    text = _render(old)
    if len(text.encode("utf-8")) > MAX_BYTES:
        raise SystemExit(f"demo-data.js would be {len(text)} bytes (limit {MAX_BYTES})")
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    log(f"wrote {out} ({len(text.encode('utf-8'))} bytes, {len(old['maps'])} maps)")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--only", action="append", default=[], help="rebuild only this map id (repeatable); "
                    "the other maps are kept from the existing file")
    ap.add_argument("--list", action="store_true", help="list the map ids and exit")
    ap.add_argument("--plan", action="append", default=[], help="with --only: rebuild only this plan key "
                    "(e.g. total_distance/k4; repeatable) and keep the map's other plans from the existing file")
    ap.add_argument("--backend", choices=("gurobi", "highs"), default=None,
                    help="ILP backend (default: gurobi if installed, else highs)")
    ap.add_argument("--time-limit", type=float, default=1800.0, help="seconds per plan (default 1800)")
    ap.add_argument("--live-work", type=int, default=100000,
                    help="simplex iterations per model allowed to the builtin solver in the live check "
                         "(deterministic; 0 disables live plans)")
    ap.add_argument("--live-columns", type=int, default=600,
                    help="largest model (columns) tried with the builtin solver")
    ap.add_argument("--seed", type=int, default=1,
                    help="MathRandom seed passed to every mppflow.plan call (split goals, tie-breaks)")
    ap.add_argument("--refresh-text", action="store_true",
                    help="keep every plan of the existing file and only refresh the map texts (title, "
                         "caption, source, vertex names, paper-plan and plan notes) from this script; no solving")
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args(argv)

    specs = [f() for f in MAP_FUNCS]
    if args.list:
        for sp in specs:
            print(sp["id"])
        return 0
    ids = {sp["id"] for sp in specs}
    for i in args.only:
        if i not in ids:
            ap.error(f"unknown map id {i!r} (see --list)")
    if args.plan and not args.only:
        ap.error("--plan needs --only")
    if args.refresh_text:
        return _refresh_text(specs, args.out, log=lambda m: print(m, flush=True))
    backend = args.backend or ("gurobi" if "gurobi" in mppflow.available_backends() else "highs")
    if backend not in mppflow.available_backends():
        ap.error(f"backend {backend!r} is not available here")
    backend_version = None
    if backend == "gurobi":
        import gurobipy

        backend_version = "Gurobi %d.%d.%d" % gurobipy.gurobi.version()
    else:
        import scipy

        backend_version = f"HiGHS (scipy {scipy.__version__})"

    def log(msg):
        print(msg, flush=True)

    old = _read_existing(args.out) if args.only else None
    old_maps = {m["id"]: m for m in (old or {}).get("maps", [])}
    maps = []
    for sp in specs:
        if args.only and sp["id"] not in args.only:
            if sp["id"] in old_maps:
                maps.append(old_maps[sp["id"]])
            else:
                log(f"{sp['id']}: not in {args.out}, left out (run without --only to build it)")
            continue
        maps.append(build_map(sp, backend, args, log, old_maps.get(sp["id"])))
    backends = sorted({pl["backend"] for m in maps for pl in m["plans"].values() if "backend" in pl})
    data = {
        "version": FORMAT_VERSION,
        "generator": GENERATOR,
        "mppflow_version": mppflow.__version__,
        "backend": backend if len(backends) <= 1 else "mixed",
        "backend_version": backend_version if not old else (old.get("backend_version") if old.get(
            "backend_version") == backend_version else "mixed"),
        "maps": maps,
    }
    text = _render(data)
    if len(text.encode("utf-8")) > MAX_BYTES:
        raise SystemExit(f"demo-data.js would be {len(text)} bytes (limit {MAX_BYTES})")
    with open(args.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    log(f"wrote {args.out} ({len(text.encode('utf-8'))} bytes, {len(maps)} maps)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
