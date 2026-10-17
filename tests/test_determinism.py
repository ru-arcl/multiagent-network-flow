"""Results do not depend on ``PYTHONHASHSEED`` or on allocation history.

Python randomizes the hash of ``str``/``bytes`` per process (and in 3.11 the hash of
``None`` and of plain objects follows their address), so iterating a ``set`` or
``frozenset`` of such values, or ordering by ``id()``, gives a process-dependent order.
These tests run the same probe (planner incl. the no-cycle retry analysis, local-repair and
conflict heuristics, puzzle solver, solvers, model dumps, generators) and a subset of the
test suite in fresh interpreters with different hash seeds and different amounts of
allocation noise, and require identical outputs.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import pytest

from helpers import GUROBI_SWITCH, child_env

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SEEDS = ("0", "1", "2", "123")

PROBE = r'''
import hashlib, json, os, re, sys, time
# shift the addresses of everything allocated later (id()-based orders would change)
_noise = [object() for _ in range(int(os.environ.get("MPPFLOW_NOISE", "0")))]

from mppflow import heuristic as H, model as M, planner as P, puzzle as Z, solvers as S
from mppflow._java import MathRandom
from mppflow.errors import OriginalWouldHang
from mppflow.graph import (Graph, create_2d_grid_graph, create_generic_2d_grid_graph_with_holes,
                           get_random_start_goal)
from mppflow.problem import create_grid_problem, create_n2_puzzle, problem_to_dat_bytes

out = {}


def norm(s):
    # wall-clock numbers in the logs
    return re.sub(r"-?\d+\.\d+(E-?\d+)?", "<f>", s or "")


def rec(name, f):
    t0 = time.perf_counter()
    try:
        out[name] = f()
    except Exception as e:  # the exception itself is part of the output
        out[name] = ["raised", type(e).__name__, str(e)]
    if os.environ.get("MPPFLOW_PROBE_TIMES"):
        print(f"{time.perf_counter() - t0:8.2f}s {name}", file=sys.stderr, flush=True)


def ring(n):
    return Graph.from_adjacency([[(i - 1) % n, (i + 1) % n] for i in range(n)])


def sha(b):
    return hashlib.sha256(b if isinstance(b, bytes) else b.encode()).hexdigest()


def frozen():
    return 0.0


# -- planner: the no-cycle retry analysis (original: hang detection in installments)
def installments(kind, k, budget, compat="original"):
    g = create_2d_grid_graph(2, 3, True)
    s, t = [4, 0, 5, 2, 3], [2, 1, 4, 3, 0]
    calls = []

    class Counting(P.PathPlanner):
        def _retry_check(self, watch):
            calls.append(watch.solves)
            if len(calls) > 40:
                raise RuntimeError("capped")
            super()._retry_check(watch)

    kw = {} if budget is None else {"analysis_budget": budget}
    pl = Counting(rng=MathRandom(1), compat=compat, backend="builtin", retry_analysis_after=1, **kw)
    try:
        if kind == "nc":
            r = pl.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, k, True)
        else:
            r = pl.plan_paths_advanced_split_by_size_no_cycle(g, s, t, True, -1, k, True)
        res = ["ok", r]
    except (OriginalWouldHang, RuntimeError) as e:
        res = [type(e).__name__, str(e)]
    return [res, calls, len(pl.solves), pl.rng.draws, list(pl.failures)]


for kind, k in (("nc", 1), ("nc", 2), ("bs", 2)):
    for budget in (1, None):
        rec(f"installments/{kind}{k}/{budget}", lambda: installments(kind, k, budget))
rec("installments/nc1/None/fixed", lambda: installments("nc", 1, None, "fixed"))


def ring_rotation(k):
    g = ring(6)
    s = list(range(6))
    t = [(i + 1) % 6 for i in s]
    pf = P.PathPlanner(rng=MathRandom(1), backend="builtin")
    r = pf.plan_paths_advanced_split_no_cycle(g, s, t, True, -1, k, True)
    return [r, len(pf.solves), list(pf.failures)]


rec("ring/nc2", lambda: ring_rotation(2))

# -- planner entry points
p55 = create_grid_problem(5, 5, 0.2, 8, MathRandom(3))
for obj, split in (("makespan", 1), ("makespan", 2), ("makespan", 4), ("total_time", 1),
                   ("total_distance", 2)):
    for compat in ("fixed", "original"):
        def _plan(obj=obj, split=split, compat=compat):
            r = P.plan(p55, obj, split, compat=compat, rng=MathRandom(7))
            return [r.paths, r.status, r.value, r.intermediate_goals, r.stats["horizons"], norm(r.log),
                    {k: v for k, v in r.extra.items()}]
        rec(f"plan/{obj}/{split}/{compat}", _plan)


def split_goals(compat):
    # one planner (one random stream) for the three calls, each recorded on its own, so that
    # an exception in one does not replace the others' results
    pl = P.PathPlanner(rng=MathRandom(11), compat=compat)
    g = p55.graph
    rec(f"split_goals/{compat}/split_paths", lambda: pl.split_paths(g, p55.starts, p55.goals, False))
    rec(f"split_goals/{compat}/split_paths_random", lambda: pl.split_paths(g, p55.starts, p55.goals, True))
    rec(f"split_goals/{compat}/by_size", lambda: pl.split_path_by_size(g, p55.starts, p55.goals, True, 3))
    rec(f"split_goals/{compat}/draws", lambda: pl.rng.draws)


for compat in ("fixed", "original"):
    split_goals(compat)

# -- heuristics
p66 = create_grid_problem(6, 6, 0.15, 12, MathRandom(5))
for compat in ("fixed", "original"):
    def _lr(compat=compat):
        r = H.plan_heuristic_paths(p66.graph, p66.starts, p66.goals, compat=compat, rng=MathRandom(2),
                                   clock=frozen)
        return [r.paths, r.status, norm(r.log), sorted(map(str, r.extra))]
    rec(f"heuristic/local_repair/{compat}", _lr)

    def _hp(compat=compat):
        pl = H.HeuristicPathPlanner(compat=compat, rng=MathRandom(2), clock=frozen)
        r = pl.plan_heuristic_paths(p66.graph, p66.starts, p66.goals)
        return [r.paths, r.status, norm(r.log)]
    rec(f"heuristic/conflict/{compat}", _hp)
    rec(f"heuristic/main/{compat}", lambda compat=compat: norm(H.path_planner_main(compat=compat, rng=MathRandom(1))))

# -- puzzle solver. In original mode, race="fast" hangs (OriginalWouldHang) at the first
# horizon without a plan; race="slow" goes on to T + 1 (here 2, 3, 4 fail; plan at T = 5)
pz = create_n2_puzzle(3, MathRandom(5))
pz11 = create_n2_puzzle(3, MathRandom(11))
for compat, race, q in (("fixed", "measured", pz), ("original", "slow", pz11), ("original", "fast", pz11)):
    def _pz(compat=compat, race=race, q=q):
        r = Z.PuzzleSolver(q.graph, q.starts, q.goals, compat=compat, rng=MathRandom(1), clock=frozen,
                           race=race).solve()
        return [r.paths, r.status, r.T, norm(r.log), r.extra]
    rec(f"puzzle/{compat}/{race}", _pz)

# -- solvers
p44 = create_grid_problem(4, 4, 0.1, 5, MathRandom(9))
g44, s44, t44 = p44.graph, p44.starts, p44.goals
rec("solve_time", lambda: (lambda r: [r.paths, r.status, r.T])(S.solve_time(g44, s44, t44, True, rng=MathRandom(1))))
rec("solve_time_r", lambda: (lambda r: [r.paths, r.status, r.T])(
    S.solve_time_r(g44, s44, t44, True, rng=MathRandom(1))))
rec("solve_total_time", lambda: (lambda r: [r.paths, r.status, r.objective])(
    S.solve_total_time(g44, s44, t44, rng=MathRandom(1))))
rec("solve_distance", lambda: (lambda r: [r.paths, r.status, r.objective])(S.solve_distance(g44, s44, t44)))
rec("solve_max_distance", lambda: (lambda r: [r.paths, r.status, r.objective])(
    S.solve_max_distance(g44, s44, t44, backend="builtin")))
rec("joint_feasibility", lambda: S.joint_feasibility(ring(5), [0, 1], [1, 0], 10**5))

# -- model dumps (canonical text, both modes)
gh = create_generic_2d_grid_graph_with_holes(5, 6, 0.2, MathRandom(4))
sh, th = get_random_start_goal(gh, 6, MathRandom(8))
gh = gh.convert_graph(sh, th) if hasattr(gh, "convert_graph") else gh
for compat in ("fixed", "original"):
    for name in ("time", "time_r", "total_time", "distance", "max_distance"):
        if compat == "original" and name == "max_distance":
            continue
        def _m(name=name, compat=compat):
            m = getattr(M, f"build_{name}_model")(p55.graph, p55.starts, p55.goals, 8, compat=compat)
            return sha(m.canonical_text())
        rec(f"model/{name}/{compat}", _m)

for obj in ("makespan", "total_time", "distance", "max_distance"):
    def _full(obj=obj):
        fm = M.build_full_network_model(p44.graph, p44.starts, p44.goals, 5, obj)
        return sha(M.canonical_text(fm.model, fm.column_names()))
    rec(f"model/full/{obj}", _full)

# -- generators and serialization
rec("gen/holes", lambda: [sorted(gh.adj.items()) if isinstance(gh.adj, dict) else gh.adj, sh, th])
rec("gen/dat", lambda: sha(problem_to_dat_bytes(p55)))
rec("gen/puzzle", lambda: [pz.starts, pz.goals])

json.dump(out, sys.stdout, sort_keys=True, default=repr)
'''


def _env(seed: str, noise: int) -> dict:
    env = child_env(PYTHONHASHSEED=seed, MPPFLOW_NOISE=str(noise))
    env.setdefault("OMP_NUM_THREADS", "1")
    return env


def _run_probe(seed: str, noise: int) -> dict:
    p = subprocess.run([sys.executable, "-c", PROBE], cwd=ROOT, env=_env(seed, noise), capture_output=True,
                       text=True, timeout=600)
    assert p.returncode == 0, f"probe failed under PYTHONHASHSEED={seed}:\n{p.stderr[-4000:]}"
    return json.loads(p.stdout)


@pytest.mark.slow
def test_probe_outputs_identical_across_hash_seeds():
    runs = [(seed, 1000 * i + 7) for i, seed in enumerate(SEEDS)]
    with ThreadPoolExecutor(len(runs)) as ex:
        outs = list(ex.map(lambda r: _run_probe(*r), runs))
    ref = outs[0]
    # the probe really ran: the retry analysis decided, the planners planned
    assert ref["installments/nc1/1"][0][0] == "OriginalWouldHang"
    assert ref["installments/nc1/1"][1:3] == [[0, 1, 2, 3, 4, 5], 12]
    assert not [k for k, v in ref.items() if isinstance(v, list) and v[:1] == ["raised"] and v[1] == "TypeError"]
    assert ref["plan/makespan/1/fixed"][1] == "optimal"
    raised = sorted(k for k, v in ref.items() if isinstance(v, list) and v[:1] == ["raised"])
    assert raised == ["puzzle/original/fast", "split_goals/original/by_size"], raised  # both OriginalWouldHang
    assert ref["puzzle/original/slow"][1:3] == ["optimal", 5] and ref["puzzle/fixed/measured"][1] == "optimal"
    assert all(ref[f"split_goals/{c}/{k}"] for c in ("fixed", "original")
               for k in ("split_paths", "split_paths_random", "draws"))
    for (seed, _), o in zip(runs[1:], outs[1:]):
        assert o.keys() == ref.keys()
        diff = sorted(k for k in ref if o[k] != ref[k])
        assert not diff, f"PYTHONHASHSEED={seed} differs from {runs[0][0]} in {diff}"


# A subset of the test suite that exercises hash-order-sensitive code (the retry
# analysis, split goals, heuristics, the puzzle race, solvers, model dumps), run under
# each seed; each must pass in every interpreter. SUBSET_SLOW are tests marked slow that
# belong to it (run in a second pytest call without the slow filter; each must run).
SUBSET_SLOW = [
    "tests/test_planner.py::test_no_cycle_original_analysis_in_installments",  # about 20-40 s
]
SUBSET = [
    "tests/test_planner.py::test_no_cycle_fixed_stops_rotating_loops_quickly",
    "tests/test_planner.py::test_no_cycle_unbounded_retry_hang_detected",
    "tests/test_heuristic.py",
    "tests/test_puzzle.py",
    "tests/test_model.py",
    "tests/test_network.py",
    "tests/test_solvers.py",
]
SUBSET_MARKS = "not gurobi and not slow"


def _run_subset(seed: str) -> tuple[str, str | None]:
    """Run SUBSET and SUBSET_SLOW under ``seed``: ``(seed, None)`` if they pass, else the
    pytest output. Both runs leave out the Gurobi tests (``-m "not gurobi"``), so no child
    needs the license; the switch makes sure that none probes it (an unreachable token
    server makes each probe wait about a minute)."""
    env = _env(seed, 0)
    env[GUROBI_SWITCH] = "0"

    def pytest_run(marks, tests):
        return subprocess.run([sys.executable, "-m", "pytest", "-q", "-x", "-p", "no:cacheprovider", "-m", marks,
                               *tests], cwd=ROOT, env=env, capture_output=True, text=True, timeout=900)

    p = pytest_run(SUBSET_MARKS, SUBSET)
    if p.returncode not in (0, 5):
        return seed, f"exit {p.returncode}:\n{p.stdout[-3000:]}{p.stderr[-2000:]}"
    q = pytest_run("not gurobi", SUBSET_SLOW)
    if q.returncode != 0 or f"{len(SUBSET_SLOW)} passed" not in q.stdout:  # failed, deselected or skipped
        return seed, f"SUBSET_SLOW, exit {q.returncode}:\n{q.stdout[-3000:]}{q.stderr[-2000:]}"
    return seed, None


@pytest.mark.slow
def test_test_subset_passes_under_every_hash_seed():
    with ThreadPoolExecutor(len(SEEDS)) as ex:
        res = list(ex.map(_run_subset, SEEDS))
    bad = [(s, txt) for s, txt in res if txt is not None]
    assert not bad, "\n\n".join(f"PYTHONHASHSEED={s}, {txt}" for s, txt in bad)
