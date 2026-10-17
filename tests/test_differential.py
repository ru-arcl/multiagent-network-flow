"""Differential tests against the live original Java code (tools/golden/differential.py).

Skipped unless ``MPPFLOW_JAVA_HARNESS`` names a compiled harness (the classes directory printed
by ``tools/golden/build.sh``). Also needed: Gurobi's ``gurobi.jar`` (``GUROBI_JAR``, or
``GUROBI_HOME``), its native libraries on ``LD_LIBRARY_PATH``, a license, and a JDK
(``MPPFLOW_JDK``, else ``java`` on ``PATH``). ``MPPFLOW_DIFF_N`` (cases per group, default 4)
and ``MPPFLOW_DIFF_SEED`` (default: a fixed seed) choose the fresh random inputs.

    MPPFLOW_JAVA_HARNESS=/path/to/classes GUROBI_JAR=$GUROBI_HOME/lib/gurobi.jar \\
        python3 -m pytest -q -m java tests/test_differential.py
"""
from __future__ import annotations

import json
import os
import sys

import pytest

HERE = os.path.dirname(__file__)
TOOLS = os.path.join(HERE, "..", "tools", "golden")
sys.path.insert(0, TOOLS)

import differential as D  # noqa: E402

HARNESS = os.environ.get("MPPFLOW_JAVA_HARNESS")
N = int(os.environ.get("MPPFLOW_DIFF_N", "4"))
SEED = int(os.environ.get("MPPFLOW_DIFF_SEED", "20161005"))

needs_java = pytest.mark.skipif(not HARNESS, reason="MPPFLOW_JAVA_HARNESS is not set")


def test_generation_is_deterministic_and_fresh():
    """Same (groups, n, seed) -> same cases; Math.random() seeds avoid those of cases.py."""
    a = D.generate(["generators", "bfs", "split"], 3, 5)
    b = D.generate(["generators", "bfs", "split"], 3, 5)
    assert json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True)
    assert len({c["id"] for c in a}) == len(a) == 9
    assert all(c["seed"] >= 1000 for c in a if c["fn"] in ("PathFinder.findShortestPaths",
                                                            "Graph.createGeneric2DGridGraphWithHoles"))
    c = D.generate(["generators"], 3, 6)
    assert json.dumps(a[:3], sort_keys=True) != json.dumps(c, sort_keys=True)


def test_graph_json_round_trip():
    from mppflow.graph import create_2d_grid_graph

    g = create_2d_grid_graph(3, 4, True)
    assert D.graph_to_json(D.graph_from_json(D.graph_to_json(g))) == D.graph_to_json(g)


def test_skip_reason_hashmap_race_through_main_suboptimal():
    """The mpath HashMap race (original-bugs B14, item 4) reached through
    Main.solveProblemSuboptimal shows as an NPE in printPaths (B21): skipped, not a mismatch."""
    case = {"fn": "Main.solveProblemSuboptimal", "args": {"splitLevel": 2, "noCycle": False}}
    j = {"exception": {"class": "java.lang.NullPointerException", "message": "...",
                       "at": ["PathPlanner.printPaths:41", "Main.solveProblemSuboptimal:148"]}}
    p = {"result": [4, None, 0, 4]}
    assert D.skip_reason(case, j, p) == "mt-hashmap-race"
    assert D.skip_reason({**case, "args": {"splitLevel": 0, "noCycle": False}}, j, p) is None
    assert D.skip_reason({**case, "args": {"splitLevel": 2, "noCycle": True}}, j, p) is None
    assert D.skip_reason(case, j, {"exception": {"class": "java.lang.NullPointerException"}}) is None



def test_skip_reason_hashmap_race_on_top_of_a_failed_piece():
    """C2D-DIFF-RACE-NONNULL-RESULT-UNCLASSIFIED: piece 1 failed in both runs (both results
    null) and Java also lost piece 3's put; only extra.mpath_keys differed (--n 150 --seed
    5550123, diff/mtsplit/035_mt_8c_...). Skipped as the race, not a mismatch."""
    case = {"fn": "PathPlanner.multiThreadedSplitPlanning", "args": {"splits": 3}}
    j = {"result": None, "extra": {"msg": None, "mpath_keys": [0, 2, 4, 5, 6, 7]}}
    p = {"result": None, "extra": {"msg": None, "mpath_keys": [0, 2, 3, 4, 5, 6, 7]}}
    assert D.skip_reason(case, j, p) == "mt-hashmap-race"
    full = {"result": [[1, 2]], "extra": {"msg": None, "mpath_keys": list(range(8))}}
    assert D.skip_reason(case, j, full) == "mt-hashmap-race"
    # not a lost put: the same keys, a superset in Java, different messages, or output printed
    assert D.skip_reason(case, {**j, "extra": dict(p["extra"])}, p) is None
    assert D.skip_reason(case, p, j) is None
    assert D.skip_reason(case, j, {**p, "extra": {**p["extra"], "msg": "x"}}) is None
    assert D.skip_reason(case, {**j, "stdout": "Exception"}, p) is None

def test_relative_work_dir_is_resolved(tmp_path, monkeypatch):
    """--work and --jdk are made absolute: the harness runs java with cwd = <work>/run."""
    seen = {}

    def fake_run_java(cases, args):
        seen["work"], seen["jdk"] = args.work, args.jdk
        raise SystemExit(0)

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(D, "run_java", fake_run_java)
    with pytest.raises(SystemExit):
        D.main(["--n", "1", "--groups", "bfs", "--work", "w", "--jdk", "jdk", "--classes", "c",
                "--gurobi-jar", "g.jar"])
    assert seen["work"] == str(tmp_path / "w") and seen["jdk"] == str(tmp_path / "jdk")


@pytest.mark.java
@pytest.mark.gurobi
@pytest.mark.slow
@needs_java
@pytest.mark.parametrize("group", D.GROUPS)
def test_differential_group(group, tmp_path):
    """Fresh random inputs of one group: the port (compat="original", Gurobi Threads=1) equals
    the live original field by field."""
    report = tmp_path / "report.json"
    argv = ["--n", str(N), "--seed", str(SEED), "--groups", group, "--report", str(report),
            "--classes", HARNESS, "--work", str(tmp_path / "work")]
    rc = D.main(argv)
    rep = json.loads(report.read_text())
    g = rep["groups"][group]
    assert g["compared"] == N, g
    if rc != 0:
        lines = [f"{m['id']}: " + ", ".join(f"{d['field']}: java={json.dumps(d['java'])[:200]} "
                                            f"python={json.dumps(d['python'])[:200]}" for d in m["diffs"])
                 for m in rep["mismatches"]]
        pytest.fail(f"{len(rep['mismatches'])} mismatch(es) in group {group}:\n" + "\n".join(lines))
