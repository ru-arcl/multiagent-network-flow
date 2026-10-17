"""Tests of mppflow.experiments (port of ILP/Main.java): the golden Main records (solve*
stat vectors, printed text, random draws, the statistics writers, the run* drivers without
data, main's cases and crashes), end-to-end reduced experiments on regenerated data checked
against the original's outputs, the drivers' control flow, the fixed-mode corrections, the
figure mapping and the problem I/O helpers."""
from __future__ import annotations

import json
import os
import re

import pytest

from mppflow import experiments as E
from mppflow import planner as P
from mppflow._java import MathRandom
from mppflow.errors import (
    JavaArrayIndexOutOfBoundsException,
    JavaNullPointerException,
)
from mppflow.graph import Graph
from mppflow.pathfinder import B_HEURISTIC, find_shortest_paths, get_total_distance
from mppflow.planner import SolverFlags, path_set_error
from mppflow.problem import Problem, create_grid_problem, read_dat, write_dat, write_txt
from helpers import GRB, graph_from_json, load_golden, needs_gurobi  # noqa: F401


HERE = os.path.dirname(__file__)
RECS = load_golden("main")
BY_ID = {r["id"]: r for r in RECS}
JAVA_TO_KEY = {d["java"]: k for k, d in E.DRIVERS.items()}

_TIMES = re.compile(r"(found in |max time limit |Time spent = |Total time spent: |, time: )"
                    r"(-?\d+\.\d+(E-?\d+)?|-?Infinity|NaN)")


def norm(s: str) -> str:
    return _TIMES.sub(r"\1<t>", s)


def same_text(mine: str, golden: str) -> bool:
    """Equal after time normalization; a golden text cut at 20000 characters is a prefix."""
    mine, golden = norm(mine), norm(golden)
    m = re.search(r"\n?<truncated \d+ chars>$", golden)
    if m:  # the recorder cut the text at 20000 characters and appended "\n<truncated N chars>"
        return mine.startswith(golden[:m.start()])
    return mine == golden


def exception_lines(s: str) -> list[str]:
    return [line for line in s.splitlines() if not line.startswith("\t")]


def problem_from_rec(p) -> Problem:
    return Problem.from_sg(graph_from_json(p["graph"]), p["sg"])


def make_ex(rec, compat="original", base=None, backend="gurobi", params=GRB, **kw) -> E.Experiments:
    flags = SolverFlags.java_defaults().apply_java(rec.get("statics") or {})
    return E.Experiments(base, compat=compat, backend=backend, params=params, rng=MathRandom(rec["seed"]),
                         flags=flags, **kw)


def call_solve(ex: E.Experiments, fn: str, a: dict):
    name = fn.split(".", 1)[1]
    p = problem_from_rec(a["p"])
    if name == "solveProblem":
        return ex.solve_problem(p, a["solveLP"], a["timeLimit"])
    if name == "solveProblemPMG":
        return ex.solve_problem_pmg(p, a["solveLP"], a["timeLimit"])
    if name == "solveProblemSuboptimal":
        return ex.solve_problem_suboptimal(p, a["solveLP"], a["setOptimal"], a["extraSteps"], a["timeLimit"],
                                           a["splitLevel"], a["noCycle"])
    if name == "solveProblemTotalTime":
        return ex.solve_problem_total_time(p, a["gap"], a["timeLimit"])
    if name == "solveProblemDistance":
        return ex.solve_problem_distance(p, a["gap"], a["timeLimit"])
    if name == "solveProblemDistanceSplit":
        return ex.solve_problem_distance_split(p, a["gap"], a["timeLimit"], a["splits"])
    if name == "solvePuzzleProblem":
        return ex.solve_puzzle_problem(a["instance"], p, a["timeLimit"])
    if name == "solveProblemWithHeuristic":
        return ex.solve_problem_with_heuristic(p)
    raise AssertionError(name)


def check_statics(ex: E.Experiments, after: dict) -> None:
    for k, v in (after or {}).items():
        if k == "PathFinder.bHeuristic":
            assert ex.b_heuristic is v, k
        else:
            assert getattr(ex.flags, SolverFlags._JAVA[k]) is v, k


def read_files(d: str) -> dict:
    out = {}
    for f in os.listdir(d):
        path = os.path.join(d, f)
        if os.path.isfile(path) and f.endswith(".txt"):
            with open(path, encoding="utf-8", newline="") as fh:
                out[f] = fh.read()
    return out


@pytest.fixture(autouse=True)
def _restore_global_flag():
    old = B_HEURISTIC.value
    yield
    B_HEURISTIC.set(old)


# ------------------------------------------------------------------ golden: solve*
SOLVE_RECS = [r for r in RECS if r["fn"].startswith("Main.solve")]


@needs_gurobi
@pytest.mark.parametrize("rec", SOLVE_RECS, ids=[r["id"] for r in SOLVE_RECS])
def test_golden_solve_helpers(rec, tmp_path):
    """Stat vectors (time entry ignored), printed text, draws and crashes of the solve*
    helpers, original mode under Gurobi Threads=1."""
    ex = make_ex(rec, base=str(tmp_path) + "/")
    if "exception" in rec:
        with pytest.raises(JavaNullPointerException):
            call_solve(ex, rec["fn"], rec["args"])
    else:
        res = call_solve(ex, rec["fn"], rec["args"])
        assert res is not None
        assert [res[0], None, *res[2:]] == rec["result"]
    assert same_text(ex.log, rec["stdout"])
    if "draws" not in (rec.get("nondet") or []):
        assert ex.rng.draws == rec["draws"]
        assert ex.rng.random() == rec["rng_next"]
    check_statics(ex, rec.get("statics_after"))
    for name, f in (rec.get("files") or {}).items():
        got = read_files(str(tmp_path))[name]
        assert norm(got) == norm(f["text"])


@needs_gurobi
@pytest.mark.parametrize("rid", ["main/solveProblem/star", "main/solveProblem/p3x3", "main/solveProblem/p8x8",
                                 "main/solveProblemSuboptimal/star_k0", "main/solveProblemSuboptimal/p8x8_k1",
                                 "main/solvePuzzleProblem/n2p3_s1"])
def test_fixed_equals_original_on_golden(rid, tmp_path):
    """Where the original is correct, fixed gives the same stat vector, text and draws.
    (The total time / distance helpers take their horizon from a 4-way split, where fixed's
    split rule may give another horizon: see mppflow.planner.)"""
    rec = BY_ID[rid]
    ex = make_ex(rec, compat="fixed", results_dir=str(tmp_path))
    res = call_solve(ex, rec["fn"], rec["args"])
    assert [res[0], None, *res[2:]] == rec["result"]
    assert same_text(ex.log, rec["stdout"])
    if "draws" not in (rec.get("nondet") or []):
        assert ex.rng.draws == rec["draws"]


def test_solve_problem_star_highs():
    """``solveProblem`` on main case 3's star (HiGHS): makespan 4, distance 6, LB 2."""
    rec = BY_ID["main/solveProblem/star"]
    for compat in ("original", "fixed"):
        ex = make_ex(rec, compat=compat, backend="highs", params=None)
        res = call_solve(ex, rec["fn"], rec["args"])
        assert [res[0], res[2], res[3]] == [4, 6, 2]
        assert ex.rng.draws == rec["draws"]
        assert "Agent 1:" in ex.log


def test_one_shot_functions():
    p = problem_from_rec(BY_ID["main/solveProblem/star"]["args"]["p"])
    res = E.solve_problem(p, True, -1, rng=MathRandom(1))
    assert [res[0], res[2], res[3]] == [4, 6, 2]
    res = E.solve_problem_suboptimal(p, False, True, 0, 100.0, 1, False, rng=1)
    assert res[0] >= 4 and res[3] == 2
    assert "Experiments.solve_problem" in E.solve_problem.__doc__


# ------------------------------------------------------- golden: stats and writers
STAT_RECS = [r for r in RECS if r["fn"] in ("Main.statsSequence", "Main.writePuzzleStats", "Main.writeString",
                                            "Main.writePuzzleResult")]


@pytest.mark.parametrize("rec", STAT_RECS, ids=[r["id"] for r in STAT_RECS])
def test_golden_stats_and_writers(rec, tmp_path):
    base = str(tmp_path) + "/"
    ex = make_ex(rec, base=base, backend="highs", params=None)
    a = rec["args"]
    fn = rec["fn"].split(".", 1)[1]
    if fn == "statsSequence":
        if a["kind"] == "plain":
            ex.reset_stats(a["test"])
            for s in a["stats"]:
                ex.process_stats(s)
            ex.write_stats()
        else:
            ex.reset_heuristic_stats(a["test"])
            for s in a["stats"]:
                ex.process_heuristic_stats(s, a["n"])
            ex.write_heuristic_stats()
        res = rec["result"]
        mine = {"currentTest": ex.current_test, "subTotal": ex.sub_total, "pathLengths": ex.path_lengths,
                "totalTime": ex.total_time, "successCount": ex.success_count, "goalsReached": ex.goals_reached,
                "pathLength": ex.path_length, "expPathLength": ex.exp_path_length,
                "finishingTimeSet": ex.finishing_time_set, "makeSpanTotal": ex.make_span_total}
        assert mine == res
    elif fn == "writePuzzleStats":
        ex.write_puzzle_stats(a["instance"], a["time"])
    elif fn == "writeString":
        ex.write_string(a["str"])
    else:
        ex.write_puzzle_result(a["instance"], a["time"], graph_from_json(a["g"]), a["paths"])
    assert ex.log == rec["stdout"]
    assert ex.stderr == rec.get("stderr", "")
    files = read_files(str(tmp_path))
    assert files == {k: v["text"] for k, v in (rec.get("files") or {}).items()}
    # the in-memory copy
    for k, v in (rec.get("files") or {}).items():
        name = k.lstrip("\\")
        assert (ex.results if name == E.RESULT_FILE else ex.puzzle_results) == v["text"]


def test_fixed_write_stats_median_over_all_times(tmp_path):
    """original-bugs B23: fixed takes the median over the multiset."""
    ex = E.Experiments(str(tmp_path), rng=0)
    ex.reset_stats("8x8, 10 agents, 10 runs. ")
    for s in ([10, 1000, 0, 7], [10, 1000, 0, 7], [10, 1000, 0, 7], [10, 5000, 0, 7]):
        ex.process_stats(s)
    line = ex.write_stats()
    assert line == ("8x8, 10 agents, 10 runs. Total success: 4, ave time: 2.0, median:1.0, min: 1.0, max: 5.0, "
                    "ave solution: 10.0, min possible: 7.0\n")
    assert ex.finishing_time_set == [1.0, 1.0, 1.0, 5.0]
    assert read_files(str(tmp_path))[E.RESULT_FILE] == line


def test_fixed_write_stats_without_success_writes_nan_row():
    ex = E.Experiments(rng=0)  # no folder: results kept in memory
    ex.reset_stats("none ")
    ex.process_stats(None)
    assert ex.write_stats() == ("none Total success: 0, ave time: NaN, median:NaN, min: NaN, max: NaN, "
                                "ave solution: NaN, min possible: NaN\n")
    ex.reset_heuristic_stats("h ")
    assert "ratio: NaN" in ex.write_heuristic_stats()
    assert ex.stderr == ""
    assert ex.results.count("NaN") == 11


def test_original_counters_wrap_at_32_bits():
    ex = E.Experiments(compat="original", base_path="/nonexistent-dir/", rng=0)
    ex.reset_stats("x ")
    ex.process_stats([2 ** 31 - 1, 0, 0, 2 ** 31 - 1])
    ex.process_stats([1, 0, 0, 1])
    assert ex.path_lengths == -2 ** 31 and ex.make_span_total == -2 ** 31
    fx = E.Experiments(rng=0)
    fx.reset_stats("x ")
    fx.process_stats([2 ** 31 - 1, 0, 0, 2 ** 31 - 1])
    fx.process_stats([1, 0, 0, 1])
    assert fx.path_lengths == 2 ** 31


def test_original_unwritable_results_folder_prints_and_continues():
    ex = E.Experiments(compat="original", rng=0)  # D:/temp/data/dmrpp/ does not exist here
    ex.write_string("x\n")
    assert ex.stderr.startswith("java.io.FileNotFoundException: D:/temp/data/dmrpp/\\n2puzzle-result.txt (")
    assert ex.results == ""


# ------------------------------------------------- golden: drivers without data, main
RUN_RECS = [r for r in RECS if r["fn"].startswith("Main.run")]
MAIN_FAST = ["main/main/3", "main/main/6", "main/main/2", "main/main/4_none.dat", "main/main/4", "main/main/0",
             "main/main/7", "main/main/x"]
MAIN_SLOW = ["main/main/1", "main/main/5"]


def _check_run_record(rec, ex, exc, d):
    if "exception" in rec:
        assert exc is not None and getattr(exc, "java_class", None) == rec["exception"]["class"], exc
    else:
        assert exc is None, exc
    cdir = d.rstrip("/")
    assert same_text(ex.log.replace(d, "$CASEDIR/"), rec.get("stdout", ""))
    assert exception_lines(ex.stderr.replace(cdir, "$CASEDIR")) == exception_lines(rec.get("stderr", ""))
    files = {k: norm(v) for k, v in read_files(cdir).items()}
    assert files == {k: norm(v["text"]) for k, v in (rec.get("files") or {}).items()}
    if rec.get("draws") is not None:
        assert ex.rng.draws == rec["draws"]
    check_statics(ex, rec.get("statics_after"))


@pytest.mark.parametrize("rec", RUN_RECS, ids=[r["id"] for r in RUN_RECS])
def test_golden_drivers_without_data(rec, tmp_path):
    """original-bugs B24: each driver's reaction to an
    empty data folder (printed exceptions, the NPE, the headers written, the flags left)."""
    d = str(tmp_path) + "/"
    ex = make_ex(rec, base=d, backend="highs", params=None)
    exc = None
    try:
        ex.run_driver(JAVA_TO_KEY[rec["fn"].split(".", 1)[1]], *rec["args"]["params"])
    except JavaNullPointerException as e:
        exc = e
    _check_run_record(rec, ex, exc, d)


def _run_main(rec, d):
    ex = make_ex(rec, base=d)
    exc = None
    try:
        ex.main([x.replace("$CASEDIR/", d) for x in rec["args"]["argv"]])
    except (JavaNullPointerException, JavaArrayIndexOutOfBoundsException, E.JavaNumberFormatException) as e:
        exc = e
    return ex, exc


@needs_gurobi
@pytest.mark.parametrize("rid", MAIN_FAST)
def test_golden_main(rid, tmp_path):
    rec = BY_ID[rid]
    d = str(tmp_path) + "/"
    ex, exc = _run_main(rec, d)
    _check_run_record(rec, ex, exc, d)
    if rec.get("rng_next") is not None:
        assert ex.rng.random() == rec["rng_next"]


@needs_gurobi
@pytest.mark.slow
@pytest.mark.parametrize("rid", MAIN_SLOW)
def test_golden_main_slow(rid, tmp_path):
    """Case 1 (16-puzzle, about 1 min) and case 5 (16x16 with 60 robots, then case 6:
    original-bugs B20)."""
    rec = BY_ID[rid]
    d = str(tmp_path) + "/"
    ex, exc = _run_main(rec, d)
    _check_run_record(rec, ex, exc, d)
    assert ex.rng.random() == rec["rng_next"]


class _StubMain(E.Experiments):
    """Records solve_problem calls instead of solving."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.calls = []

    def solve_problem(self, p, solve_lp=True, time_limit=-1.0):
        self.calls.append((p.graph.n, len(p.starts), solve_lp, time_limit))
        return None


def test_main_case5_fallthrough_only_in_original():
    orig = _StubMain(compat="original", rng=5)
    orig.main(["5"])
    assert orig.calls == [(256, 60, True, 7200), (17, 8, True, -1)]
    assert "Adjacency list:" in orig.log
    fixed = _StubMain(rng=5)
    fixed.main(["5"])
    assert fixed.calls == [(256, 60, True, 7200)]
    assert "Adjacency list:" not in fixed.log
    # the same instance: identical draws up to the end of case 5
    assert fixed.log == orig.log[:len(fixed.log)]


def test_main_case_parsing():
    ex = _StubMain(compat="original", rng=1)
    ex.main([])  # no argument: case 1
    assert ex.calls == [(16, 16, True, -1)]
    ex = _StubMain(compat="original", rng=1)
    ex.main(["+3"])  # Integer.parseInt accepts a sign
    assert ex.calls == [(4, 2, True, -1)]
    for bad in ("x", " 3", "3.0", "99999999999"):
        with pytest.raises(E.JavaNumberFormatException):
            _StubMain(compat="original", rng=1).main([bad])
    with pytest.raises(ValueError):
        _StubMain(rng=1).main(["x"])
    with pytest.raises(ValueError):
        _StubMain(rng=1).main(["4"])
    for bad in ("7", "42", "-1"):
        assert _StubMain(compat="original", rng=1).main([bad]) == ""  # Java's switch has no default
        with pytest.raises(ValueError, match=r"case must be 0\.\.6"):
            _StubMain(rng=1).main([bad])


def test_main_case4_fixed_reads_any_format(tmp_path):
    p = create_grid_problem(4, 4, 0.0, 3, MathRandom(3))
    for ext in (".dat", ".txt", ".json"):
        fn = str(tmp_path / ("p" + ext))
        if ext == ".dat":
            write_dat(p, fn)
        elif ext == ".txt":
            write_txt(p, fn)
        else:
            with open(fn, "w") as f:
                json.dump(E.problem_to_json(p), f)
        ex = _StubMain(rng=1)
        ex.main(["4", fn])
        assert ex.calls == [(16, 3, True, 1000)]
        assert ex.log.startswith(f"Solving problem from file: {fn} with 1000 seconds limit.\n\nStart and goal")
    with pytest.raises(FileNotFoundError):
        _StubMain(rng=1).main(["4", str(tmp_path / "none.dat")])


def test_module_main_returns_instance():
    ex = E.main(7, compat="original", rng=1)
    assert ex.log == "" and ex.rng.draws == 0
    ex = E.main(["3"], backend="highs", rng=1)
    assert "Agent 0: 0:0" in ex.log


# --------------------------------------------- end-to-end on regenerated data sets
def _gen_8x8(tmp_path, compat):
    d = str(tmp_path) + ("/" if compat == "original" else "")
    files = E.generate(42, d, ["crowded"], compat, write_text=False)
    assert len(files) == 310
    return d


def _rows(results: str) -> list[str]:
    return [re.sub(r"ave time: [^,]*, median:[^,]*, min: [^,]*, max: [^,]*, ", "", line)
            for line in results.splitlines()]


# Output of the live Java drivers (8x8, k = 1 and the 2-way no-cycle split):
JAVA_8X8_K1 = ["8x8, 10 agents, 10 runs. Total success: 10, ave solution: 9.9, min possible: 9.6",
               "8x8, 20 agents, 10 runs. Total success: 10, ave solution: 10.4, min possible: 10.0",
               "8x8, 30 agents, 10 runs. Total success: 10, ave solution: 11.6, min possible: 10.8", ""]
JAVA_8X8_K2_NC = ["8x8, 10 agents, no cycles,  10 runs. Total success: 10, ave solution: 11.5, min possible: 9.6",
                  "8x8, 20 agents, no cycles,  10 runs. Total success: 10, ave solution: 13.9, min possible: 10.0",
                  ""]


@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_crowded_8x8_on_regenerated_data_matches_java(compat, tmp_path):
    """Live Java rows on data regenerated with seed 42 (``Problem``'s 8x8 family),
    ``runCrowdedPerformanceMetric8x8(10, 30, 60, 1, false)`` with seed 1. The makespans of
    the 2-way split are optimal per piece, so every backend gives the Java rows."""
    d = _gen_8x8(tmp_path, compat)
    ex = E.Experiments(d, compat=compat, backend="highs", rng=1)
    ex.run_crowded_performance_metric_8x8(10, 30, 60, 1, False)
    assert _rows(ex.results) == JAVA_8X8_K1
    assert ex.log.startswith("Working on 8x8 grid performance testing, problem: 10-agts-1001.dat\nAgent 0: ")
    assert ex.log.count("Time spent = ") == 30
    on_disk = read_files(d.rstrip("/"))[("\\" if compat == "original" else "") + E.RESULT_FILE]
    assert on_disk == ex.results


@needs_gurobi
def test_crowded_8x8_no_cycle_on_regenerated_data_matches_java(tmp_path):
    """``runCrowdedPerformanceMetric8x8(10, 20, 60, 2, true)``: the no-cycle 4-way split
    (its retries depend on the plans; Gurobi Threads=1 gives the Java rows)."""
    d = _gen_8x8(tmp_path, "original")
    ex = E.Experiments(d, compat="original", backend="gurobi", params=GRB, rng=1)
    ex.run_crowded_performance_metric_8x8(10, 20, 60, 2, True)
    assert _rows(ex.results) == JAVA_8X8_K2_NC


# SHA-256 of the console output of the live Java driver (JDK 17, Gurobi 13.0.2, Threads 1)
# on the same regenerated data, with the "Time spent" numbers masked and Gurobi's own lines
# removed.
JAVA_8X8_LOG_SHA = {
    (10, 30, 60, 1, False): "fa7921e12050462424bcd9b30e97df105aa7c8705599ca6d6701bd51e6f52a1b",
    (10, 20, 60, 2, True): "403fc18f8eb8eefb1080d4f48ffafe0f778dc63a52275ac512cd334602b16ee0",
}


@needs_gurobi
@pytest.mark.parametrize("args", list(JAVA_8X8_LOG_SHA))
def test_crowded_8x8_console_output_equals_live_java(args, tmp_path):
    """Every printed plan, retry and line of ``runCrowdedPerformanceMetric8x8`` equals the
    live original's."""
    import hashlib

    d = _gen_8x8(tmp_path, "original")
    ex = E.Experiments(d, compat="original", backend="gurobi", params=GRB, rng=1)
    ex.run_crowded_performance_metric_8x8(*args)
    log = re.sub(r"(Time spent = |Total time spent: )[0-9.E-]+", r"\1<t>", "\n".join(ex.log.splitlines()))
    assert hashlib.sha256(log.encode()).hexdigest() == JAVA_8X8_LOG_SHA[args]


@needs_gurobi
@pytest.mark.slow
def test_grid_variants_sequence_matches_java():
    """Live Java run: seed 7,
    ``createGridProblem(24, 18, 0.10, 20)``, then every solve* helper in a row on the same
    stream (live Java)."""
    ex = E.Experiments(compat="original", backend="gurobi", params=GRB, rng=7, clock=lambda: 0.0)
    p = create_grid_problem(24, 18, 0.10, 20, ex.rng, "original")
    assert p.graph.n == 389
    got = [ex.solve_problem_suboptimal(p, False, False, 0, 600.0, k, False) for k in range(3)]
    got.append(ex.solve_problem_suboptimal(p, False, True, 0, 600.0, 1, True))
    got.append(ex.solve_problem(p, False, 600.0))
    got.append(ex.solve_problem_total_time(p, 20 * 0.002 + 10 * 0.002, 600.0))
    got.append(ex.solve_problem_distance(p, 20 * 0.002 + 10 * 0.002, 600.0))
    got.append(ex.solve_problem_distance_split(p, 20 * 0.001 + 10 * 0.0025, 600.0, 2))
    java = [[30, 0, 30], [30, 0, 30], [30, 0, 30], [30, 0, 30], [30, 525, 30], [343, 343, 341], [345, 345, 341],
            [351, 351, 341]]
    assert [[g[0], g[2], g[3]] for g in got] == java


def _write_24x18(tmp_path, ns=(10, 20)):
    for n in ns:
        p = create_grid_problem(24, 18, 0.0, n, MathRandom(n))
        os.makedirs(tmp_path / "24x18-grid", exist_ok=True)
        write_dat(p, str(tmp_path / "24x18-grid" / f"{n}-agt-0-pct-obs-1001.dat"))


def test_reduced_24x18_rows_with_a_stub_solver(tmp_path):
    """``run24x18PerformanceMetric``'s sweep, headers and labels, with the per-instance
    solve replaced by a stub (the exact solves take several seconds each; see the slow test
    below): the calls (robots, split level, time limit), the rows, and a failure that writes
    the partial row and ends the obstacle percentage."""
    _write_24x18(tmp_path)
    sc = E.Scale(instances=1, max_agents=20, obstacles=(0,), sections=("exact", "2-way"), time_limit=60)
    ex = E.Experiments(str(tmp_path), backend="highs", rng=1, scale=sc)
    calls = []

    def stub(p, solve_lp, set_optimal, extra_steps, time_limit, split_level, no_cycle):
        calls.append((len(p.starts), split_level, time_limit, set_optimal, no_cycle))
        if fail and len(p.starts) == 10:
            return None
        lb = P._lower_bounds(p.graph, p.starts, p.goals)["makespan"]
        return [lb + split_level, 1500, 0, lb]

    ex.solve_problem_suboptimal = stub
    fail = False
    ex.run_24x18_performance_metric()
    assert calls == [(10, 0, 60, False, False), (20, 0, 60, False, False), (20, 1, 60, False, False)]
    lines = ex.results.splitlines()
    assert lines[0] == "24 x 18 grid, exact"
    assert lines[1].startswith("24x18, 0% obstacles, 10 agents, 1 runs. Total success: 1, ave time: 1.5")
    assert lines[2].startswith("24x18, 0% obstacles, 20 agents, 1 runs. Total success: 1,")
    assert lines[3:5] == ["", "24 x 18 grid, 2-way split"]
    assert lines[5].startswith("24x18, 0% obstacles, 20 agents, 1 runs. Total success: 1,")
    assert len(lines) == 6
    exact = [int(float(x)) for x in re.findall(r"min possible: ([\d.]+)", ex.results)]
    sol = [float(x) for x in re.findall(r"ave solution: ([\d.]+)", ex.results)]
    assert sol == [exact[0], exact[1], exact[2] + 1]
    calls.clear()
    fail = True
    ex2 = E.Experiments(str(tmp_path), backend="highs", rng=1, scale=sc)
    ex2.solve_problem_suboptimal = stub
    ex2.run_24x18_performance_metric()
    assert [c[:2] for c in calls] == [(10, 0), (20, 1)]  # the failure ends the exact sweep
    lines = ex2.results.splitlines()
    assert lines[1].startswith("24x18, 0% obstacles, 10 agents, 1 runs. Total success: 0, ave time: NaN")
    assert lines[2:4] == ["", "24 x 18 grid, 2-way split"] and len(lines) == 5


@pytest.mark.slow
def test_reduced_24x18_exact_and_split_rows(tmp_path):
    """A reduced ``run24x18PerformanceMetric``: one instance per row, 0 % obstacles, at most
    20 robots, sections exact and 2-way; checks the rows, the headers and the labels. Slow:
    each exact HiGHS solve takes 5-10 s on an idle machine, and HiGHS's limit is wall-clock
    time (several times longer under load), hence the generous limit."""
    _write_24x18(tmp_path)
    sc = E.Scale(instances=1, max_agents=20, obstacles=(0,), sections=("exact", "2-way"), time_limit=600)
    ex = E.Experiments(str(tmp_path), backend="highs", rng=1, scale=sc)
    ex.run_24x18_performance_metric()
    lines = ex.results.splitlines()
    assert lines[0] == "24 x 18 grid, exact"
    assert lines[1].startswith("24x18, 0% obstacles, 10 agents, 1 runs. Total success: 1, ave time: ")
    assert lines[2].startswith("24x18, 0% obstacles, 20 agents, 1 runs. Total success: 1,")
    assert lines[3] == ""
    assert lines[4] == "24 x 18 grid, 2-way split"
    assert lines[5].startswith("24x18, 0% obstacles, 20 agents, 1 runs. Total success: 1,")
    assert len(lines) == 6
    exact = [int(float(x)) for x in re.findall(r"min possible: ([\d.]+)", ex.results)]
    sol = [float(x) for x in re.findall(r"ave solution: ([\d.]+)", ex.results)]
    assert sol[0] == exact[0] and sol[1] == exact[1]  # obstacle-free: exact makespan = lower bound here
    assert sol[2] >= exact[2]


@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_reduced_n2puzzle_driver_on_regenerated_data(compat, tmp_path):
    """``runN2PuzzlePerformanceMetric`` on regenerated puzzles, 9-puzzles only, 2 instances:
    a ``writePuzzleStats`` line per instance, one row, and ``25puzzle.txt``."""
    d = str(tmp_path) + ("/" if compat == "original" else "")
    E.generate(3, d, ["n2puzzle"], compat, write_text=False)
    ex = E.Experiments(d, compat=compat, rng=1, scale=E.Scale(instances=2, max_agents=9))
    ex.run_n2_puzzle_performance_metric()
    lines = ex.results.splitlines()
    assert [line.split(", time: ")[0] for line in lines[:2]] == ["Instance: 1", "Instance: 2"]
    assert lines[2].startswith("9-puzzle, 2 runs. Total success: 2, ave time: ")
    assert len(lines) == 3
    mk = [int(x) for x in re.findall(r"Agent 0: .* (\d+):", ex.log)]
    assert re.search(r"ave solution: ([\d.]+)", lines[2]).group(1) == E._jd(sum(mk) / 2)
    pz = ex.puzzle_results
    assert pz.startswith("Instance: 1, time: ") and "Agent  9: 0:(" in pz and pz.count("Instance: ") == 2
    assert ex.log.startswith("Working on 9-puzzle-1001.dat\n")


# ------------------------------------------------------------ driver control flow
class _Scripted(E.Experiments):
    """Drivers with scripted solve results: ``script(method, sub, name, args)`` returns a
    stat vector or None. The result files go under ``BASE`` (a per-test temporary folder, see
    ``_scripted_base``), never into the working directory."""

    BASE = "DATA"

    def __init__(self, script, **kw):
        super().__init__(self.BASE + "/" if kw.get("compat") == "original" else self.BASE, **kw)
        self.script = script
        self.calls = []
        self._cur = None

    def read_problem(self, sub, name):
        self._cur = (sub, name)
        self.calls.append(("read", sub, name))
        return Problem(Graph(), [], [])

    def _stub(self, method, *args):
        self.calls.append((method, *self._cur, *args))
        return self.script(method, *self._cur, args)

    def solve_problem_suboptimal(self, p, *args):
        return self._stub("subopt", *args)

    def solve_problem(self, p, *args):
        return self._stub("exact", *args)

    def solve_problem_total_time(self, p, *args):
        return self._stub("tt", *args)

    def solve_problem_distance(self, p, *args):
        return self._stub("td", *args)

    def solve_problem_distance_split(self, p, *args):
        return self._stub("tds", *args)

    def solve_puzzle_problem(self, i, p, *args):
        return self._stub("puzzle", i, *args)

    def solve_problem_with_heuristic(self, p):
        return self._stub("heuristic")


@pytest.fixture(autouse=True)
def _scripted_base(tmp_path, monkeypatch):
    # The folder must exist: compat="original" only appends to it (as Java's FileWriter does).
    (tmp_path / "DATA").mkdir()
    monkeypatch.setattr(_Scripted, "BASE", str(tmp_path / "DATA"))


def _ok(*_):
    return [10, 1000, 0, 9]


def test_crowded_first_failure_ends_the_sweep():
    def script(m, sub, name, args):
        return None if name == "20-agts-1003.dat" else [10, 1000, 0, 9]

    ex = _Scripted(script, rng=0)
    ex.run_crowded_performance_metric_8x8(10, 40, 60, 1, False)
    reads = [c[2] for c in ex.calls if c[0] == "read"]
    assert reads[-1] == "20-agts-1003.dat" and len(reads) == 13
    assert ex.results == ("8x8, 10 agents, 10 runs. Total success: 10, ave time: 1.0, median:1.0, min: 1.0, "
                          "max: 1.0, ave solution: 10.0, min possible: 9.0\n\n")
    assert ("subopt", "8x8-grid", "10-agts-1001.dat", False, True, 0, 60, 1, False) == ex.calls[1]


def test_24x18_partial_row_then_next_obstacle():
    def script(m, sub, name, args):
        return None if name == "20-agt-0-pct-obs-1002.dat" else [30, 500, 0, 30]

    ex = _Scripted(script, rng=0, scale=E.Scale(obstacles=(0, 5), max_agents=30, sections=("exact",)))
    ex.run_24x18_performance_metric()
    lines = ex.results.splitlines()
    assert lines[0] == "24 x 18 grid, exact"
    assert lines[1].startswith("24x18, 0% obstacles, 10 agents, 10 runs. Total success: 10,")
    assert lines[2].startswith("24x18, 0% obstacles, 20 agents, 10 runs. Total success: 1,")  # partial row
    assert lines[3] == ""
    assert lines[4].startswith("24x18, 5% obstacles, 10 agents")
    assert lines[6].startswith("24x18, 5% obstacles, 30 agents")
    assert lines[7] == "" and len(lines) == 8
    assert "Working on 24x18 grid performance testing, problem: 10-agt-0-pct-obs-1001.dat\n" in ex.log
    sub = [c for c in ex.calls if c[0] == "subopt"][0]
    assert sub[3:] == (False, False, 0, 600.0, 0, False)


def test_24x18_distance_drivers_skip_failing_row_and_switch_split():
    def script(m, sub, name, args):
        return None if name.startswith("20-agt-") else [300, 500, 300, 290]

    ex = _Scripted(script, rng=0, scale=E.Scale(instances=2))
    ex.run_24x18_performance_metric_total_distance_split()
    lines = ex.results.splitlines()
    assert lines[0] == "24 x 18 grid, total distance solution, split computation"
    assert lines[1].startswith("24x18, 0% obstacles, 10 agents, 2 runs. Total success: 2,")
    assert lines[2] == ""  # the failing 20-robot row is not written
    assert len(lines) == 1 + 7 * 2
    tds = [c for c in ex.calls if c[0] == "tds"]
    by_obs = {}
    for c in tds:
        obs = int(re.match(r"\d+-agt-(\d+)-", c[2]).group(1))
        by_obs[obs] = c[-1]
        n = int(c[2].split("-")[0])
        assert c[3] == pytest.approx(n * 0.001 + obs * 0.0025 + (n * 0.001 if n > 70 else 0))
        assert c[4] == 600.0
    assert by_obs == {0: 2, 5: 2, 10: 2, 15: 2, 20: 2, 25: 3, 30: 3}


def test_24x18_total_time_gap_formula_and_fixed_time_flags():
    seen = []

    def script(m, sub, name, args):
        seen.append((m, name, args))
        return [300, 500, 300, 290]

    ex = _Scripted(script, rng=0, scale=E.Scale(instances=1, max_agents=120, obstacles=(10,)))
    ex.run_24x18_performance_metric_total_time()
    gaps = {int(name.split("-")[0]): a[0] for m, name, a in seen}
    for n, gap in gaps.items():
        assert gap == n * 0.002 + 10 * 0.002 + (n * 0.001 if n > 50 else 0) + (n * 0.0005 if n > 100 else 0)
    assert {a[1] for _, _, a in seen} == {600.0}
    seen.clear()
    flags_during = []
    ex = _Scripted(lambda *a: flags_during.append(ex.flags.tt_return_any_solution) or [1, 1, 1, 1], rng=0,
                   scale=E.Scale(instances=1, obstacles=(0,)))
    ex.run_24x18_performance_metric_total_time_fixed_time()
    assert flags_during and all(flags_during) and ex.flags.tt_return_any_solution is False
    assert ex.results.startswith("24 x 18 grid, total time solution, fixed time (100s)\n24x18, 0% obstacles, "
                                 "70 agents, 1 runs.")
    assert [int(line.split(", ")[2].split()[0]) for line in ex.results.splitlines()[1:-1]] == [70, 80, 90, 100]


def test_32x32_sections_labels_and_no_cycle_splits():
    ex = _Scripted(lambda *a: [40, 1000, 0, 38], rng=0, scale=E.Scale(instances=1, max_agents=40))
    ex.run_32x32_performance_metric()
    lines = ex.results.splitlines()
    assert lines[0] == "32 x 32 grid, 20% obstacles, exact"
    assert lines[1].startswith("32x32, 20% obstacles, 10 agents, 1 runs. Total success: 1")
    i2 = lines.index("32 x 32 grid, 20% obstacles, 2-way split")
    assert lines[i2 + 1].startswith("32x32, 20% obstacles, 20 agents, no cycles,  1 runs. ")
    assert "32 x 32 grid, 20% obstacles, 16-way split" in lines
    subs = [c for c in ex.calls if c[0] == "subopt"]
    assert {c[-2] for c in subs} == {1, 2, 3, 4} and all(c[-1] is True for c in subs)
    assert all(c[3:7] == (False, True, 0, 1800.0) for c in subs)
    exact = [c for c in ex.calls if c[0] == "exact"]
    assert exact[0][3:] == (False, 1800.0)


def test_crowded_sections_and_n2puzzle_lines():
    ex = _Scripted(lambda *a: [10, 1000, 0, 9], rng=0,
                   scale=E.Scale(instances=1, max_agents=10, sections=("16x16/4-way-nc",)))
    ex.run_crowded_performance_metric_crowded()
    assert ex.results.splitlines()[0] == "16 x 16 grid, 4-way split, no cycle"
    assert [c for c in ex.calls if c[0] == "subopt"][0][3:] == (False, True, 0, 1800.0, 2, True)
    assert "Working on 16x16 grid performance testing,  no cycles, problem: 10-agts-1001.dat" in ex.log
    ex = _Scripted(lambda *a: [4, 1500, 30, 3], rng=0, scale=E.Scale(instances=2))
    ex.run_n2_puzzle_performance_metric()
    lines = ex.results.splitlines()
    assert lines[:2] == ["Instance: 1, time: 1.5", "Instance: 2, time: 1.5"]
    assert lines[2].startswith("9-puzzle, 2 runs. Total success: 2, ave time: 1.5")
    assert lines[5].startswith("16-puzzle, 2 runs.") and lines[8].startswith("25-puzzle, 2 runs.")
    assert ex.calls[1] == ("puzzle", "n2puzzle", "9-puzzle-1001.dat", 0, -1.0)


def test_legacy_drivers_names_and_labels():
    for compat, name, label in (("original", "20-pct-obs-20-1001.dat", "100 runs"),
                                ("fixed", "20-pct-obs-20-agts-1001.dat", "10 runs")):
        ex = _Scripted(lambda *a: [100, 30, 20, 500, 400], compat=compat, rng=0, scale=E.Scale(max_agents=20))
        ex.run_32x32_performance_metric_heuristic()
        assert ex.calls[0] == ("read", "32x32-grid", name)
        assert ex.results.startswith(f"32x32, 20% obstacles, 20 agents, {label}. Total success: 10, goals reached: "
                                     "200, ")
        assert ex.results.endswith("steps: 5000, exp: 4000, ratio: 1.25\n")
    for compat, ext in (("original", ".txt"), ("fixed", ".dat")):
        ex = _Scripted(_ok, compat=compat, rng=0, scale=E.Scale(instances=1))
        ex.run_32x32_performance_metric_splitting_8connected(25, 50, 10.0, 1, True)
        assert [c[2] for c in ex.calls if c[0] == "read"] == [f"20-pct-obs-25-agts-1001{ext}",
                                                               f"20-pct-obs-50-agts-1001{ext}"]
        assert "Working on 32x32 grid performance testing, no cycles,  problem: 20-pct-obs-25-agts-1001.txt" in ex.log


def test_full_performance_metric_plan():
    ex = _Scripted(_ok, rng=0, scale=E.Scale(instances=1, max_agents=25))
    ex.run_full_performance_metric()
    headers = [line for line in ex.results.splitlines() if "splitting" in line or line == "No splitting"]
    assert headers == ["No splitting", "2 - splitting", "2 - splitting, no cycle", "4 - splitting",
                       "4 - splitting, no cycle", "8 - splitting", "8 - splitting, no cycle", "16 - splitting",
                       "16 - splitting, no cycle"]
    reads32 = [c[2] for c in ex.calls if c[0] == "read" and c[1] == "32x32-grid"]
    assert "20-pct-obs-25-agts-1001.dat" in reads32  # the legacy n = 25 rows (never generated)


def test_run_driver_names():
    for key, d in E.DRIVERS.items():
        assert callable(getattr(E.Experiments, d["method"]))
        assert E._driver_key(d["java"]) == key and E._driver_key(d["method"]) == key
    with pytest.raises(KeyError):
        E._driver_key("nope")


# ------------------------------------------------------------- fixed corrections
def _swap_problem() -> Problem:
    g = Graph.from_adjacency([[1], [0]], [(0, 0), (1, 0)])
    return Problem(g, [0, 1], [1, 0])  # two robots swapping on one edge: infeasible


def test_fixed_infeasible_instance_returns_none_and_driver_stops(tmp_path):
    p = _swap_problem()
    ex = E.Experiments(rng=1)
    assert ex.solve_problem_suboptimal(p, False, True, 0, 10.0, 0, False) is None
    assert ex.solve_problem(p, True, 10.0) is None
    assert ex.solve_problem_total_time(p, 0.0, 10.0) is None
    assert ex.solve_problem_distance_split(p, 0.0, 10.0, 1) is None
    assert len(ex.failures) == 4
    os.makedirs(tmp_path / "8x8-grid")
    write_dat(p, str(tmp_path / "8x8-grid" / "10-agts-1001.dat"))
    ex = E.Experiments(str(tmp_path), rng=1)
    ex.run_crowded_performance_metric_8x8(10, 20, 10.0, 0, False)
    assert ex.results == "\n"  # the intended break: no row for the failing robot count


def test_fixed_total_time_nonpositive_limit_means_no_limit():
    p = problem_from_rec(BY_ID["main/solveProblemTotalTime/p3x3"]["args"]["p"])
    assert E.Experiments(compat="original", rng=1).solve_problem_total_time(p, 0.0, -1.0) is None
    res = E.Experiments(rng=1).solve_problem_total_time(p, 0.0, -1.0)
    assert res is not None and res[0] == 6 and res[3] == 4


def test_original_subopt_failure_raises_npe():
    """original-bugs B21: a non-positive limit makes the one-piece split fail at
    its first (infeasible) horizon; the original then prints ``null``."""
    p = problem_from_rec(BY_ID["main/solveProblem/star"]["args"]["p"])
    ex = E.Experiments(compat="original", rng=1)
    with pytest.raises(JavaNullPointerException):
        ex.solve_problem_suboptimal(p, False, True, 0, -1.0, 0, False)
    res = E.Experiments(rng=1).solve_problem_suboptimal(p, False, True, 0, -1.0, 0, False)
    assert res is not None and res[0] == 4


def test_fixed_missing_data_file_raises(tmp_path):
    ex = E.Experiments(str(tmp_path), rng=1)
    with pytest.raises(FileNotFoundError, match="generate"):
        ex.run_crowded_performance_metric_8x8(10, 10, 60, 1, False)
    with pytest.raises(FileNotFoundError):
        ex.run_24x18_performance_metric_total_time_fixed_time()
    assert ex.flags.tt_return_any_solution is False  # restored in fixed
    orig = E.Experiments(str(tmp_path) + "/", compat="original", rng=1)
    with pytest.raises(JavaNullPointerException):
        orig.run_24x18_performance_metric_total_time_fixed_time()
    assert orig.flags.tt_return_any_solution is True  # the original leaves it set
    with pytest.raises(ValueError):
        E.Experiments(rng=1).read_problem("8x8-grid", "x.dat")  # no base_path


def test_fixed_corrupt_data_file_raises(tmp_path):
    os.makedirs(tmp_path / "8x8-grid")
    (tmp_path / "8x8-grid" / "10-agts-1001.dat").write_bytes(b"0:0:0 1:1:0")
    with pytest.raises(ValueError):
        E.Experiments(str(tmp_path), rng=1).run_crowded_performance_metric_8x8(10, 10, 60, 1, False)


@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_heuristic_stat_vector_and_flag(compat):
    """original-bugs B19."""
    p = create_grid_problem(8, 8, 0.1, 6, MathRandom(11))
    ex = E.Experiments(compat=compat, rng=8, heuristic_options={"max_subproblems": 200})
    stat = ex.solve_problem_with_heuristic(p)
    paths = ex.last_paths
    assert stat is not None and paths is not None
    assert path_set_error(paths, p.graph, p.starts, p.goals) is None
    lb = sum(len(sp) - 1 for sp in find_shortest_paths(p.graph, p.starts, p.goals, MathRandom(0), heuristic=False))
    moves = get_total_distance(paths)
    assert stat[2] == 6
    if compat == "original":
        assert [stat[0], stat[3], stat[4]] == [len(paths[0]), moves - 1, lb - 1]
        assert ex.b_heuristic is True
    else:
        assert [stat[0], stat[3], stat[4]] == [len(paths[0]) - 1, moves, lb]
        assert ex.b_heuristic is False
    assert B_HEURISTIC.value is False  # the process-wide flag is never left set


def test_original_b_heuristic_leaks_into_later_solves():
    p = problem_from_rec(BY_ID["main/solveProblem/p3x3"]["args"]["p"])
    a = E.Experiments(compat="original", rng=2)
    a.solve_problem(p, False, -1)
    b = E.Experiments(compat="original", rng=2)
    b.b_heuristic = True  # as after solveProblemWithHeuristic
    b.solve_problem(p, False, -1)
    assert b.rng.draws > a.rng.draws  # random-insertion searches draw more
    assert B_HEURISTIC.value is False


# ------------------------------------------------------------------ figures, I/O
def test_figures_table():
    for key, f in E.FIGURES.items():
        assert f.key == key
        if f.driver is not None:
            assert f.driver in E.DRIVERS
            for s in f.sections or ():
                assert s in E.DRIVERS[f.driver]["sections"], (key, s)
    assert E.FIGURES["tor-fig19"].driver == "24x18-td-split"
    assert E.FIGURES["tor-fig16-top"].sections[0] == "8x8/exact"
    assert E.FIGURES["tor-fig13"].driver is None
    text = E.figures_text()
    assert "tor-fig17" in text and "run32x32PerformanceMetric" in text


@pytest.mark.parametrize("fam", ["crowded", "n2puzzle"])
def test_generate_fixed_equals_original_contents(fam, tmp_path):
    a = E.generate(5, str(tmp_path / "f"), [fam], "fixed")
    b = E.generate(5, str(tmp_path / "o") + "/", [fam], "original")
    assert len(a) == len(b)
    for fa, fb in zip(a, b):
        assert os.path.basename(fb).endswith(os.path.basename(fa))
        with open(fa, "rb") as x, open(fb, "rb") as y:
            assert x.read() == y.read()


def test_problem_json_round_trip_and_formats(tmp_path):
    p = create_grid_problem(5, 4, 0.2, 4, MathRandom(9))
    d = E.problem_to_json(p)
    q = E.problem_from_json(json.loads(json.dumps(d)))
    assert q.graph.adj == p.graph.adj and q.graph.xy == p.graph.xy and q.graph.vertices == p.graph.vertices
    assert (q.starts, q.goals) == (p.starts, p.goals)
    # golden-record encoding with "sg"
    rec = BY_ID["main/solveProblem/p3x3"]["args"]["p"]
    r = E.problem_from_json(rec)
    assert r.graph.adj == graph_from_json(rec["graph"]).adj and r.starts == rec["sg"][0]
    # plain neighbour lists
    s = E.problem_from_json({"graph": {"adj": [[1], [0, 2], [1]]}, "starts": [0], "goals": [2]})
    assert s.graph.adj == {0: [0, 1], 1: [0, 1, 2], 2: [1, 2]}
    with pytest.raises(ValueError):
        E.problem_from_json({"starts": [0]})
    with pytest.raises(ValueError):
        E.problem_from_json({"graph": {"adj": [[1], [0]]}})
    # neighbour lists at the top level: the error shows the shape (B3-JSON-PROBLEM-SHAPE)
    with pytest.raises(ValueError, match=r'needs a "graph", e\.g\. \{"graph": \{"adj"'):
        E.problem_from_json({"adj": [[1], [0, 2], [1]], "starts": [0], "goals": [2]})
    with pytest.raises(ValueError, match="must be an object"):
        E.problem_from_json([[1], [0]])
    # the output of solve --json --include-problem: the problem is its "problem" member
    t = E.problem_from_json({"status": "optimal", "problem": d, "paths": []})
    assert t.graph.adj == p.graph.adj and (t.starts, t.goals) == (p.starts, p.goals)
    for ext in (".dat", ".txt", ".json"):
        fn = str(tmp_path / ("p" + ext))
        if ext == ".dat":
            write_dat(p, fn)
        elif ext == ".txt":
            write_txt(p, fn)
        else:
            with open(fn, "w") as f:
                json.dump(d, f)
        q = E.load_problem(fn)
        assert q.graph.adj == p.graph.adj and q.starts == p.starts and q.goals == p.goals
    assert read_dat(str(tmp_path / "p.dat")).goals == p.goals


def test_scale_helpers():
    sc = E.Scale.smoke()
    assert (sc.instances, sc.max_agents, sc.time_limit) == (1, 20, 60.0)
    assert list(E.Experiments._agents(sc, 10, 60, 10)) == [10, 20]
    assert E.Experiments._obs(E.Scale(obstacles=(0, 25)), 0, 20, 5) == [0]
    with pytest.raises(ValueError):
        E.Experiments(compat="original", horizon="search")


def _ticking_clock(step):
    """A deterministic clock that advances ``step`` seconds per call."""
    import itertools

    c = itertools.count()
    return lambda: 1000.0 + step * next(c)


@pytest.mark.parametrize("compat", ["original", "fixed"])
def test_heuristic_runs_on_the_instance_clock(compat):
    """Regression: solve_problem_with_heuristic must forward Experiments.clock to the
    heuristic (as to the planners and PuzzleSolver). Before the fix it ran on the
    heuristic's frozen default clock, so the 3 s attempt budget and the sub-solvers'
    remaining time never shrank (original-mode Time ILP loop never stopped on 32x32)."""
    p = create_grid_problem(10, 10, 0.2, 20, MathRandom(99))
    # The clock advances: the sub-solvers' time limits are what is left of the 3 s.
    ex = E.Experiments(compat=compat, rng=8, clock=_ticking_clock(0.01))
    stat = ex.solve_problem_with_heuristic(p)
    lims = [float(x) for x in re.findall(r"max time limit (\S+)", ex.log)]
    assert stat is not None and stat[2] == 20
    assert lims and max(lims) < 3.0
    # A clock that jumps 10 s per reading: every 3 s attempt runs out at once.
    ex = E.Experiments(compat=compat, rng=8, clock=_ticking_clock(10.0))
    stat = ex.solve_problem_with_heuristic(p)
    assert stat is not None and stat[2] < 20
    assert ex.log.count("Attempting with 3.0 seconds...") == 10
    # heuristic_options["clock"] still overrides the instance's clock.
    ex = E.Experiments(compat=compat, rng=8, clock=_ticking_clock(10.0), heuristic_options={"clock": lambda: 0.0})
    stat = ex.solve_problem_with_heuristic(p)
    assert stat is not None and stat[2] == 20
    assert ex.log.count("Attempting with") == 1
    assert set(re.findall(r"max time limit (\S+)", ex.log)) == {"3.0"}


def test_heuristic_clock_forwarding(monkeypatch):
    import mppflow.heuristic as H

    seen = []
    real = H.plan_heuristic_paths

    def spy(*a, **kw):
        seen.append(kw.get("clock"))
        return real(*a, **kw)

    monkeypatch.setattr(H, "plan_heuristic_paths", spy)
    p = create_grid_problem(8, 8, 0.1, 6, MathRandom(11))
    clk = _ticking_clock(0.0)
    E.Experiments(rng=8, clock=clk).solve_problem_with_heuristic(p)
    other = _ticking_clock(0.0)
    E.Experiments(rng=8, clock=clk, heuristic_options={"clock": other}).solve_problem_with_heuristic(p)
    assert seen[0] is clk and seen[1] is other


# ------------------------------------------- TT/TD horizon default
def _corridor_problem(L=24, bs=9, rg=10) -> Problem:
    """Single-horizon TT/DIST (paper-vs-code), Driver3 24 9 10: corridor 0..L, pendant Q = L+1 on
    vertex 1, garage R = L+2 on vertex rg; robots 0 -> L and bs -> Q. The 4-way-split
    horizon is 29; the optima (TT 41, TD 33) need T = 32 <= n * T_opt = 48."""
    adj: dict[int, set[int]] = {}

    def add(u, v):
        adj.setdefault(u, set()).add(v)
        adj.setdefault(v, set()).add(u)

    for i in range(L):
        add(i, i + 1)
    add(1, L + 1)
    add(rg, L + 2)
    g = Graph.from_adjacency([sorted(adj[v]) for v in range(L + 3)])
    return Problem(g, [0, bs], [L, L + 1])


@pytest.mark.parametrize("backend,params", [pytest.param("highs", None, marks=pytest.mark.slow),
                                            pytest.param("gurobi", GRB, marks=needs_gurobi)])
def test_fixed_tt_td_default_searches_the_horizon(backend, params):
    """Regression: fixed solve_problem_total_time / solve_problem_distance (and the
    one-shot functions) must follow Algorithm 1 by default (DESIGN section 4). Before the
    fix they solved only at the 4-way-split makespan and returned 45 / 37 with gap 0."""
    p = _corridor_problem()
    kw = dict(backend=backend, params=params)
    ex = E.Experiments(rng=1, **kw)
    assert ex.solve_problem_total_time(p, 0.0, -1)[0] == 41
    assert ex.solve_problem_distance(p, 0.0, -1)[0] == 33
    assert E.solve_problem_total_time(p, 0.0, -1, rng=1, **kw)[0] == 41
    assert E.solve_problem_distance(p, 0.0, -1, rng=1, **kw)[0] == 33
    # The original's single horizon: explicitly in fixed, always in original.
    ex = E.Experiments(rng=1, horizon="single", **kw)
    assert ex.solve_problem_total_time(p, 0.0, -1)[0] == 45
    assert E.Experiments(rng=1, **kw).solve_problem_distance(p, 0.0, -1, horizon="single")[0] == 37
    orig = E.Experiments(compat="original", rng=1, **kw)
    assert orig.solve_problem_total_time(p, 0.0, 600.0)[0] == 45
    assert orig.solve_problem_distance(p, 0.0, 600.0)[0] == 37
    with pytest.raises(ValueError):
        orig.solve_problem_total_time(p, 0.0, 600.0, horizon="search")


class _HorizonSpy(_Scripted):
    def _record(self, m):
        self.hz.append((m, self._resolve_horizon(None)))
        return [10, 1000, 10, 9]

    def solve_problem_total_time(self, p, *args):
        return self._record("tt")

    def solve_problem_distance(self, p, *args):
        return self._record("td")


@pytest.mark.parametrize("horizon,want_driver", [(None, "single"), ("single", "single"), ("search", "search")])
def test_tt_td_drivers_keep_the_split_horizon(horizon, want_driver):
    """The paper's TT/TD figures (drivers) keep the 4-way-split horizon unless the
    constructor chooses one explicitly; direct calls default to the search afterwards."""
    sc = E.Scale(instances=1, max_agents=10, obstacles=(0,))
    for drv, n in (("run_24x18_performance_metric_total_time", 10), ("run_24x18_performance_metric_total_distance", 10),
                   ("run_24x18_performance_metric_total_time_fixed_time", 70)):
        ex = _HorizonSpy(lambda *a: None, rng=0, horizon=horizon)
        ex.hz = []
        getattr(ex, drv)(scale=E.Scale(instances=1, max_agents=n, obstacles=(0,)))
        assert ex.hz and {h for _, h in ex.hz} == {want_driver}, drv
        assert ex._resolve_horizon(None) == (horizon or "search")
    ex = _HorizonSpy(lambda *a: None, rng=0, compat="original")
    ex.hz = []
    ex.run_24x18_performance_metric_total_time(scale=sc)
    assert {h for _, h in ex.hz} == {"single"} and ex._resolve_horizon(None) == "single"
    with pytest.raises(ValueError):
        E.Experiments(horizon="sometimes")


def test_problem_json_verbatim_encodings_keep_the_adjacency():
    """Regression (C-JSON-GRAPH-NO-SELF-LOOPS): the golden-record and to_dict encodings are
    used as written (they can describe graphs without wait loops), so a hand-written file
    that lists only the real edges gives a graph on which nobody can wait: a UserWarning
    now says so. Plain neighbour lists get the loops."""
    rec = {"graph": {"vertices": [[0, 0, 0], [1, 1, 0], [2, 2, 0]], "adj": [[0, [1]], [1, [0, 2]], [2, [1]]]},
           "starts": [0, 2], "goals": [1, 2]}
    with pytest.warns(UserWarning, match="wait self-loop"):
        p = E.problem_from_json(rec)
    assert p.graph.adj == {0: [1], 1: [0, 2], 2: [1]}
    q = E.problem_from_json({"graph": {"adj": [[1], [0, 2], [1]]}, "starts": [0, 2], "goals": [1, 2]})
    assert q.graph.adj == {0: [0, 1], 1: [0, 1, 2], 2: [1, 2]}
    rec["graph"]["adj"][0][1].append(0)  # one wait loop: a deliberate no-wait graph, no warning
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        E.problem_from_json(rec)


def test_figures_table_agrees_with_paper_vs_code_notes():
    """``python -m mppflow figures`` (``E.FIGURES``) and section 4 of
    docs/notes/paper-vs-code.md, which says it prints the same table, give the same ranges
    ``a..b`` for each paper item."""
    with open(os.path.join(HERE, "..", "docs", "notes", "paper-vs-code.md"), encoding="utf-8") as f:
        text = f.read()
    section = text.split("## 4. Figure", 1)[1].split("\n## ", 1)[0]
    venue = {"[TRO16]": "T-RO 2016", "[ICRA13]": "ICRA 2013"}
    rows = {}
    for line in section.splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) >= 2 and cells[0].split(" ", 1)[0] in venue:
            tag, item = cells[0].split(" ", 1)
            rows[(venue[tag], item)] = cells[1]
    matched = 0
    for fig in E.FIGURES.values():
        row = rows.get((fig.paper, fig.item))
        if row is None:
            continue
        matched += 1
        for rng_text in re.findall(r"\d+\.\.\d+", fig.what):
            assert rng_text in row, (fig.key, rng_text, row)
    assert matched >= 15, matched
