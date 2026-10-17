"""Smoke tests of the command line interface (mppflow.cli, ``python -m mppflow``) on small
instances."""
from __future__ import annotations

import json
import os
import subprocess
import sys

import pytest

from mppflow import cli
from mppflow._java import MathRandom
from mppflow.planner import path_set_error
from mppflow.problem import create_grid_problem, read_dat, write_dat
from mppflow.experiments import load_problem

from helpers import child_env, needs_gurobi  # noqa: F401  (gurobipy and a full-size license)
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def run(capsys, *argv):
    rc = cli.main(list(argv))
    out, err = capsys.readouterr()
    return rc, out, err


def solve_json(capsys, *argv):
    rc, out, err = run(capsys, "solve", *argv, "--json")
    return rc, json.loads(out)


def test_solve_grid_json(capsys, tmp_path):
    rc, r = solve_json(capsys, "--grid", "6x6", "--obstacles", "0.1", "--robots", "5", "--seed", "2",
                       "--save-problem", str(tmp_path / "p.dat"))
    assert rc == 0 and r["status"] == "optimal" and r["objective"] == "makespan"
    p = read_dat(str(tmp_path / "p.dat"))
    assert r["problem"]["starts"] == p.starts and r["problem"]["robots"] == 5
    assert path_set_error(r["paths"], p.graph, p.starts, p.goals) is None
    assert r["value"] == r["metrics"]["makespan"] == len(r["paths"][0]) - 1
    assert r["metrics"]["makespan"] >= r["lower_bounds"]["makespan"]
    # the same instance from the generator: the seed fixes it
    p2 = create_grid_problem(6, 6, 0.1, 5, MathRandom(2))
    assert (p2.starts, p2.goals) == (p.starts, p.goals)


def test_solve_json_no_paths(capsys):
    """Regression (CLI-JSON-NOPATHS): --no-paths left the paths in the --json output."""
    rc, r = solve_json(capsys, "--puzzle", "3", "--seed", "1", "--no-paths")
    assert rc == 0 and r["status"] == "optimal" and "paths" not in r
    rc, r = solve_json(capsys, "--puzzle", "3", "--seed", "1")
    assert rc == 0 and len(r["paths"]) == r["problem"]["robots"] == 9


@pytest.mark.parametrize("ext", [".dat", ".txt", ".json"])
def test_solve_from_file(capsys, tmp_path, ext):
    fn = str(tmp_path / ("p" + ext))
    rc, _, _ = run(capsys, "solve", "--grid", "5x5", "--robots", "4", "--seed", "3", "--save-problem", fn,
                   "--no-paths")
    assert rc == 0
    rc, r = solve_json(capsys, fn, "--include-problem")
    assert rc == 0 and r["status"] == "optimal"
    p = load_problem(fn)
    assert r["problem"]["starts"] == p.starts
    assert path_set_error(r["paths"], p.graph, p.starts, p.goals) is None


def test_solve_json_include_problem_round_trip(capsys, tmp_path):
    """Regression (B3-JSON-PROBLEM-SHAPE): the --include-problem output could not be solved
    again ('JSON problem needs a "graph"'); the documented top-level shape is accepted."""
    rc, out, _ = run(capsys, "solve", "--grid", "3x3", "--robots", "3", "--seed", "5", "--json",
                     "--include-problem", "--no-paths")
    assert rc == 0
    fn = tmp_path / "r.json"
    fn.write_text(out, encoding="utf-8")
    rc, r = solve_json(capsys, str(fn), "--include-problem")
    assert rc == 0 and r["problem"] == json.loads(out)["problem"] and r["value"] == json.loads(out)["value"]
    fn = tmp_path / "a.json"
    fn.write_text('{"graph": {"adj": [[1], [0, 2], [1]]}, "starts": [0], "goals": [2]}', encoding="utf-8")
    rc, r = solve_json(capsys, str(fn))
    assert rc == 0 and r["paths"] == [[0, 1, 2]]
    fn.write_text('{"adj": [[1], [0, 2], [1]], "starts": [0], "goals": [2]}', encoding="utf-8")
    rc, _, err = run(capsys, "solve", str(fn))
    assert rc == 1 and 'needs a "graph", e.g. {"graph": {"adj"' in err


def test_solve_text_output_and_objectives(capsys):
    rc, out, _ = run(capsys, "solve", "--puzzle", "3", "--seed", "4")
    assert rc == 0
    assert out.startswith("instance: 9 vertices, 9 robots\nstatus: optimal  objective: makespan = ")
    assert "Agent 8:" in out
    import oracles as O
    nbrs = O.grid_graph(4, 4)  # the same row-major ids as create_2d_grid_graph(4, 4)
    best = {"total_time": O.min_total_time, "total_distance": O.min_total_distance,
            "max_distance": O.min_max_distance}
    for obj in ("total_time", "total_distance", "max_distance"):
        rc, r = solve_json(capsys, "--grid", "4x4", "--robots", "3", "--seed", "5", "--objective", obj)
        assert rc == 0 and r["objective"] == obj and r["status"] == "optimal"
        s, g = r["problem"]["starts"], r["problem"]["goals"]
        assert O.validate_plan(nbrs, s, g, r["paths"]) is None
        # checked against an independent metric and the joint-state optimum
        assert r["value"] == O.plan_metrics(r["paths"], g)[obj] == best[obj](nbrs, s, g)[0]
        assert r["value"] >= r["lower_bounds"][obj]


def test_solve_split_and_log(capsys):
    rc, r = solve_json(capsys, "--grid", "8x8", "--robots", "10", "--seed", "1", "--split", "4", "--log")
    assert rc == 0 and r["split"] == 4 and len(r["intermediate_goals"]) == 3
    assert r["metrics"]["makespan"] >= r["lower_bounds"]["makespan"]
    assert "Trying T = " in r["log"]


def test_solve_rejects_a_huge_split(capsys):
    """Regression (TS1A-HUGE-SPLIT-OOM): --split 1099511627776 (2**40) ran out of memory; above
    1024 it is one error line."""
    for k in ("2048", str(2 ** 40)):
        rc, out, err = run(capsys, "solve", "--grid", "4x4", "--robots", "4", "--split", k)
        assert rc == 1 and out == "" and err.startswith("error: ValueError: split must be at most 1024"), err


def test_solve_errors(capsys):
    rc, _, err = run(capsys, "solve", "--grid", "4x4", "--robots", "2", "--objective", "max_distance",
                     "--compat", "original")
    assert rc == 1 and "NotInOriginal" in err
    with pytest.raises(SystemExit):
        cli.main(["solve"])  # no instance
    with pytest.raises(SystemExit):
        cli.main(["solve", "--grid", "4x4"])  # no --robots
    with pytest.raises(SystemExit):
        cli.main(["solve", "--grid", "4by4", "--robots", "2"])
    rc, _, err = run(capsys, "solve", "/nonexistent/x.json")
    assert rc == 1 and "FileNotFoundError" in err


def test_solve_infeasible_exit_status(capsys, tmp_path):
    fn = tmp_path / "swap.json"
    fn.write_text(json.dumps({"graph": {"adj": [[1], [0]]}, "starts": [0, 1], "goals": [1, 0]}))
    rc, out, err = run(capsys, "solve", str(fn), "--json")
    assert rc == 1 and out == "" and "error: Infeasible" in err


def test_generate_and_experiment(capsys, tmp_path):
    data = str(tmp_path / "data")
    rc, out, _ = run(capsys, "generate", "--out", data, "--seed", "42", "--families", "crowded", "--no-text")
    assert rc == 0 and out == f"wrote 310 files to {data}\n"
    assert os.path.exists(os.path.join(data, "8x8-grid", "10-agts-1001.dat"))
    rc, out, _ = run(capsys, "experiment", "crowded-8x8", "10", "20", "60", "1", "false", "--data", data,
                     "--seed", "1")
    assert rc == 0
    assert "Working on 8x8 grid performance testing, problem: 10-agts-1001.dat" in out
    rows = [line for line in out.splitlines() if "Total success" in line]
    assert rows[0].startswith("8x8, 10 agents, 10 runs. Total success: 10,")
    assert rows[0].endswith("ave solution: 9.9, min possible: 9.6")  # the live Java rows on the same data
    assert rows[1].endswith("ave solution: 10.4, min possible: 10.0")
    with open(os.path.join(data, "n2puzzle-result.txt")) as f:
        assert f.read().count("Total success") == 2
    # a figure key, reduced, results in another folder, quiet
    res = str(tmp_path / "res")
    rc, out, _ = run(capsys, "experiment", "crowded", "--data", data, "--instances", "1", "--max-agents", "10",
                     "--sections", "8x8/2-way", "--results", res, "--quiet")
    assert rc == 0 and [line for line in out.splitlines() if "grid" in line] == ["8 x 8 grid, 2-way split"]
    rc, out, _ = run(capsys, "experiment", "tor-fig16-top", "--data", data, "--instances", "1", "--max-agents", "10",
                     "--results", res, "--quiet")
    assert rc == 0
    lines = out.splitlines()
    assert lines[0] == "8 x 8 grid, exact" and lines[1].startswith("8x8, 10 agents, 1 runs. Total success: 1,")
    assert [line for line in lines if "grid" in line] == ["8 x 8 grid, exact", "8 x 8 grid, 2-way split",
                                                          "8 x 8 grid, 4-way split", "8 x 8 grid, 8-way split"]
    assert os.path.exists(os.path.join(res, "n2puzzle-result.txt"))


def test_generate_original_names(capsys, tmp_path):
    out = str(tmp_path)
    rc, _, _ = run(capsys, "generate", "--out", out, "--seed", "1", "--families", "crowded", "--compat", "original",
                   "--no-text")
    assert rc == 0
    names = os.listdir(out)
    assert "\\8x8-grid\\10-agts-1001.dat" in names and len(names) == 310


def test_generate_original_no_text_new_folder(capsys, tmp_path):
    """Regression (D2-GEN-ORIG-NOTEXT-FALSE-SUCCESS): with --no-text, original mode wrote no
    file into a folder that did not exist yet (the original's text writer creates it), yet
    reported every file as written and exited 0."""
    out = str(tmp_path / "new")
    rc, text, err = run(capsys, "generate", "--out", out, "--families", "n2puzzle", "--compat", "original",
                        "--no-text")
    assert rc == 0 and text == f"wrote 300 files to {out}\n" and err == ""
    assert len(os.listdir(out)) == 300 and "\\n2puzzle\\9-puzzle-1001.dat" in os.listdir(out)


@pytest.mark.skipif(not hasattr(os, "geteuid") or os.geteuid() == 0, reason="needs a non-root POSIX user")
@pytest.mark.parametrize("no_text", [True, False])
def test_generate_original_unwritable_folder_fails(capsys, tmp_path, no_text):
    """Files that original mode cannot open (the Java prints the exception and goes on) are
    not counted as written, and the command exits 1."""
    ro = tmp_path / "ro"
    ro.mkdir()
    ro.chmod(0o555)
    try:
        rc, text, err = run(capsys, "generate", "--out", str(ro / "x"), "--families", "n2puzzle", "--compat",
                            "original", *(["--no-text"] if no_text else []))
    finally:
        ro.chmod(0o755)
    assert rc == 1 and text == f"wrote 0 files to {ro / 'x'}\n"
    assert f"error: could not write {300 if no_text else 600} files" in err


def test_experiment_errors(capsys, tmp_path):
    with pytest.raises(SystemExit):
        cli.main(["experiment", "nope", "--data", str(tmp_path)])
    with pytest.raises(SystemExit):
        cli.main(["experiment", "tor-fig13", "--data", str(tmp_path)])  # ID: not in the Java code
    with pytest.raises(SystemExit):
        cli.main(["experiment", "24x18", "1", "--data", str(tmp_path)])  # takes no parameters
    rc, _, err = run(capsys, "experiment", "crowded-8x8", "--data", str(tmp_path), "--quiet")
    assert rc == 1 and "FileNotFoundError" in err
    rc, out, err = run(capsys, "experiment", "crowded-8x8", "--data", str(tmp_path), "--compat", "original",
                       "--quiet")
    assert rc == 1 and "java.lang.NullPointerException" in err


def test_case_command(capsys):
    rc, out, _ = run(capsys, "case", "3")
    assert rc == 0
    assert out.startswith("Create a simple graph manually and then solve a problem on this graph.\n\n"
                          "Adjacency list:\n0: 1, \n1: 0, 2, 3, \n")
    assert "Agent 1: 0:3 " in out
    rc, out, err = run(capsys, "case", "7")
    assert rc == 1 and out == "" and "ValueError: case must be 0..6, not 7" in err
    rc, out, _ = run(capsys, "case", "7", "--compat", "original")
    assert rc == 0 and out == ""
    rc, _, err = run(capsys, "case", "x", "--compat", "original")
    assert rc == 1 and 'java.lang.NumberFormatException: For input string: "x"' in err
    rc, _, err = run(capsys, "case", "4", "--compat", "original")
    assert rc == 1 and "ArrayIndexOutOfBoundsException" in err


@needs_gurobi
def test_case_3_original_gurobi_matches_java(capsys):
    rc, out, _ = run(capsys, "case", "3", "--compat", "original", "--backend", "gurobi", "--threads", "1")
    assert rc == 0
    with open(os.path.join(REPO, "tests", "golden", "main.jsonl"), encoding="utf-8") as f:
        rec = next(r for r in map(json.loads, f) if r["id"] == "main/main/3")
    assert out == rec["stdout"]


def test_case_4_with_file(capsys, tmp_path):
    p = create_grid_problem(4, 4, 0.0, 2, MathRandom(6))
    fn = str(tmp_path / "p.dat")
    write_dat(p, fn)
    rc, out, _ = run(capsys, "case", "4", fn)
    assert rc == 0 and out.startswith(f"Solving problem from file: {fn} with 1000 seconds limit.\n")
    assert "Agent 1:" in out


def test_figures(capsys):
    rc, out, _ = run(capsys, "figures")
    assert rc == 0 and "tor-fig18" in out and "run24x18PerformanceMetricTotalTime" in out
    rc, out, _ = run(capsys, "figures", "--json")
    figs = json.loads(out)
    assert {f["key"] for f in figs} >= {"tor-fig10", "tor-fig17", "icra-table3"}


def test_no_command_prints_help(capsys):
    rc, out, _ = run(capsys)
    assert rc == 2 and "usage: mppflow" in out


def test_threads_ignored_by_highs(capsys):
    rc, out, err = run(capsys, "solve", "--grid", "3x3", "--robots", "2", "--threads", "1", "--no-paths")
    assert rc == 0 and "--threads is ignored" in err


def test_python_dash_m(tmp_path):
    env = child_env()  # this checkout, also under PYTHONSAFEPATH=1
    r = subprocess.run([sys.executable, "-m", "mppflow", "solve", "--grid", "4x4", "--robots", "3", "--json"],
                       cwd=REPO, capture_output=True, text=True, timeout=300, env=env)
    assert r.returncode == 0, r.stderr
    assert json.loads(r.stdout)["status"] == "optimal"
    r = subprocess.run([sys.executable, "-m", "mppflow", "figures"], cwd=REPO, capture_output=True, text=True,
                       timeout=300, env=env)
    assert r.returncode == 0 and "tor-fig10" in r.stdout


def test_solve_horizon_option(capsys, monkeypatch):
    """--horizon reaches plan(horizon=...): one solve at the optimal makespan is not proved
    optimal over all horizons (status "feasible" here, total time 32 vs the lower bound 18)."""
    rc, single = solve_json(capsys, "--puzzle", "3", "--objective", "total_time", "--horizon", "single")
    assert rc == 0 and single["status"] == "feasible" and single["extra"]["horizon"] == single["metrics"]["makespan"]
    rc, search = solve_json(capsys, "--puzzle", "3", "--objective", "total_time")
    assert rc == 0 and search["status"] == "optimal" and search["value"] <= single["value"]
    # argparse wraps the help at the terminal width (COLUMNS): words compared with the
    # whitespace normalised, so no width splits "each piece" (regression TD2-HELP-TEXT-TERMINAL-WIDTH)
    for cols in ("40", "66", "85", "200"):
        monkeypatch.setenv("COLUMNS", cols)
        help_text = " ".join(cli.build_parser()._subparsers._group_actions[0].choices["solve"].format_help().split())
        assert "--horizon" in help_text and "each piece" in help_text, cols


def test_solve_8_connected(capsys):
    """--8-connected builds create_grid_problem_8connected: the diagonal moves shorten this
    instance from makespan 5 to 3, and the plan is valid on the 8-connected grid only."""
    from mppflow.problem import create_grid_problem_8connected

    rc, r8 = solve_json(capsys, "--grid", "4x4", "--robots", "6", "--seed", "3", "--8-connected")
    rc4, r4 = solve_json(capsys, "--grid", "4x4", "--robots", "6", "--seed", "3")
    assert rc == rc4 == 0 and r8["status"] == r4["status"] == "optimal"
    assert (r8["value"], r8["lower_bounds"]["makespan"], r4["value"], r4["lower_bounds"]["makespan"]) == (3, 3, 5, 5)
    p8 = create_grid_problem_8connected(4, 4, 0.0, 6, MathRandom(3))
    p4 = create_grid_problem(4, 4, 0.0, 6, MathRandom(3))
    assert (r8["problem"]["starts"], r8["problem"]["goals"]) == (p8.starts, p8.goals)
    assert path_set_error(r8["paths"], p8.graph, p8.starts, p8.goals) is None
    assert path_set_error(r8["paths"], p4.graph, p8.starts, p8.goals) is not None


def test_solve_time_limit_and_gap_reach_plan(capsys, monkeypatch):
    """--time-limit and --gap are passed to plan(); an expired limit is status "time_limit"
    with exit status 1."""
    rc, r = solve_json(capsys, "--grid", "4x4", "--robots", "6", "--seed", "3", "--objective", "total_distance",
                       "--time-limit", "1e-9")
    assert rc == 1 and r["status"] == "time_limit" and r["paths"] is None
    from mppflow import planner

    seen = {}
    real = planner.plan

    def spy(*a, **k):
        seen.update(k)
        return real(*a, **k)

    monkeypatch.setattr(planner, "plan", spy)
    rc, r = solve_json(capsys, "--grid", "4x4", "--robots", "3", "--seed", "5", "--objective", "total_time",
                       "--time-limit", "30", "--gap", "0.25")
    assert rc == 0 and (seen["time_limit"], seen["gap"]) == (30.0, 0.25)


@pytest.mark.parametrize("bad", ["0", "-1", "nan"])
def test_solve_rejects_a_time_limit_that_is_not_positive(capsys, bad):
    """fixed: --time-limit must be > 0 (omit it for no limit); 0, -1 and nan used to mean no
    limit and solved without one."""
    rc, out, err = run(capsys, "solve", "--grid", "4x4", "--robots", "3", "--no-paths", "--time-limit", bad)
    assert rc == 1 and "time_limit must be None (no limit) or a finite number of seconds > 0" in err


def test_solve_prints_the_failures_of_a_split(capsys, tmp_path):
    """A k-way split that fails on a solvable instance (a 4-cycle: the robots cannot change
    their cyclic order, so the intermediate configuration is unreachable): exit status 1,
    status "failed", and one "failure:" line per piece (``extra["failures"]`` in --json)."""
    fn = tmp_path / "cycle4.json"
    fn.write_text(json.dumps({"graph": {"vertices": [0, 1, 2, 3], "adj": {"0": [0, 1, 2], "1": [0, 1, 3],
                                                                          "2": [0, 2, 3], "3": [1, 2, 3]}},
                              "starts": [0, 1, 3], "goals": [3, 2, 1]}))
    rc, out, _ = run(capsys, "solve", str(fn), "--split", "2")
    assert rc == 1
    assert "status: failed  objective: makespan = None" in out
    fails = [line for line in out.splitlines() if line.startswith("failure: ")]
    assert fails == [f"failure: piece {i}: the instance has no solution for any horizon (joint-state search)"
                     for i in (0, 1)]
    rc, r = solve_json(capsys, str(fn), "--split", "2")
    assert rc == 1 and r["status"] == "failed" and r["paths"] is None
    assert [f"failure: {f}" for f in r["extra"]["failures"]] == fails
    rc, r = solve_json(capsys, str(fn))  # the whole instance is solvable
    assert rc == 0 and r["status"] == "optimal"


def test_version(capsys):
    import mppflow

    with pytest.raises(SystemExit) as e:
        cli.main(["--version"])
    out, _ = capsys.readouterr()
    assert e.value.code == 0 and out.strip() == f"mppflow {mppflow.__version__}"


def test_missing_gurobipy_is_one_error_line(capsys, monkeypatch):
    """A missing optional dependency prints one 'error:' line (exit status 1), as 'view' does
    without matplotlib, instead of a traceback."""
    monkeypatch.setitem(sys.modules, "gurobipy", None)  # import gurobipy -> ImportError
    rc, out, err = run(capsys, "solve", "--grid", "3x3", "--robots", "2", "--backend", "gurobi")
    assert rc == 1 and err.startswith("error: The 'gurobi' backend needs gurobipy") and "Traceback" not in err


def test_view_without_matplotlib_is_one_error_line(capsys, monkeypatch):
    """'view' without matplotlib: one 'error:' line with the install hint (exit status 1),
    before any solve. Here, not in test_viewer.py, which is skipped without matplotlib."""
    from mppflow import planner

    monkeypatch.setitem(sys.modules, "matplotlib", None)  # import matplotlib -> ImportError
    monkeypatch.setattr(planner, "plan", lambda *a, **k: pytest.fail("solved before the check"))
    rc, out, err = run(capsys, "view", "--puzzle", "3")
    assert rc == 1 and out == ""
    assert err.splitlines() == ['error: mppflow.viewer needs matplotlib: pip install matplotlib, or '
                                'pip install ".[viewer]" from a checkout of the repository']


def test_viewer_import_is_lazy():
    """Importing mppflow and mppflow.viewer loads no matplotlib (so both import without it)."""
    code = "import sys, mppflow.viewer, mppflow; assert 'matplotlib' not in sys.modules, 'eager import'"
    subprocess.run([sys.executable, "-c", code], cwd=REPO, check=True, env=child_env())
    code = ("import sys; sys.modules['matplotlib'] = None; import mppflow.viewer as V\n"
            "try:\n    V._mpl()\nexcept ImportError as e:\n    print(e)")
    p = subprocess.run([sys.executable, "-c", code], cwd=REPO, check=True, capture_output=True, text=True,
                       env=child_env())
    assert 'pip install ".[viewer]"' in p.stdout


def test_gurobi_license_error_is_one_error_line(capsys, monkeypatch):
    """A Gurobi license failure (pip's size-limited license on a large model, an expired
    license) prints one 'error:' line (exit status 1) instead of a traceback; other
    exceptions still propagate."""
    from mppflow.lp import gurobi as G

    GurobiError = type("GurobiError", (Exception,), {"__module__": "gurobipy._exception"})

    def fail(*a, **k):
        raise GurobiError("Model too large for size-limited license")

    monkeypatch.setattr(G, "solve", fail)
    rc, out, err = run(capsys, "solve", "--grid", "3x3", "--robots", "2", "--backend", "gurobi")
    assert rc == 1 and "Traceback" not in err
    assert err.splitlines() == ["error: GurobiError: Model too large for size-limited license "
                                "(a Gurobi license problem? try --backend highs)"]

    def boom(*a, **k):
        raise RuntimeError("not a license problem")

    monkeypatch.setattr(G, "solve", boom)
    with pytest.raises(RuntimeError):
        cli.main(["solve", "--grid", "3x3", "--robots", "2", "--backend", "gurobi"])
