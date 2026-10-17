"""The public API (mppflow/__init__.py, docs/API.md) and the consolidated exceptions."""

from __future__ import annotations

import os
import re
import subprocess
import sys

import mppflow
from mppflow import errors

from helpers import child_env

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_import_is_cheap():
    code = ("import sys, mppflow; heavy = [m for m in ('gurobipy', 'scipy', 'numpy') if m in sys.modules];"
            "print(heavy)")
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, cwd=ROOT,
                       env=child_env(), timeout=60)
    assert r.returncode == 0, r.stderr
    assert r.stdout.strip() == "[]"


def test_all_names_resolve():
    assert len(set(mppflow.__all__)) == len(mppflow.__all__)
    for name in mppflow.__all__:
        assert hasattr(mppflow, name), name
    for name in errors.__all__:
        assert hasattr(errors, name), name
    assert mppflow.from_adjacency([[1], [0]]).adj == {0: [0, 1], 1: [0, 1]}


def test_api_doc_signatures_use_the_real_parameter_names():
    """Every ``name(a, b=...)`` signature in the name column of docs/API.md's tables names
    real parameters of that mppflow callable, so a keyword call written as documented works."""
    import importlib
    import inspect

    with open(os.path.join(ROOT, "docs", "API.md"), encoding="utf-8") as f:
        text = f.read()
    modules = [mppflow]
    for extra in ("mppflow.lp", "mppflow.viewer"):
        try:
            modules.append(importlib.import_module(extra))
        except ImportError:  # the viewer needs matplotlib
            pass
    cells = [line.split("|")[1] for line in text.splitlines() if line.startswith("| `")]
    checked, bad = 0, []
    for m in re.finditer(r"`(?:mppflow\.)?([A-Za-z_][\w.]*)\(([^`()]*)\)`", "\n".join(cells)):
        name, args = m.group(1), m.group(2)
        obj = None
        for mod in modules:
            obj = mod
            for part in name.split("."):
                obj = getattr(obj, part, None)
                if obj is None:
                    break
            if obj is not None:
                break
        if obj is None or not callable(obj):
            continue  # builtins, numpy, Java names
        params = inspect.signature(obj).parameters
        if any(p.kind is p.VAR_KEYWORD for p in params.values()):
            continue
        for arg in args.split(","):
            arg = arg.split("=")[0].strip()
            if not arg or arg.startswith("*") or arg == "...":
                continue
            checked += 1
            if arg not in params:
                bad.append(f"{name}: {arg!r} not in {list(params)}")
    assert checked >= 40, checked
    assert not bad, bad


def test_moved_exceptions_keep_their_old_import_paths():
    from mppflow import experiments, model, planner, problem, solvers

    assert planner.JavaStackOverflowError is errors.JavaStackOverflowError
    assert solvers.JavaGRBException is errors.JavaGRBException
    assert experiments.JavaNumberFormatException is errors.JavaNumberFormatException
    assert model.InexactSolutionWarning is errors.InexactSolutionWarning
    assert problem.JavaClassCastException is errors.JavaClassCastException
    assert problem.JavaStreamCorruptedException is errors.JavaStreamCorruptedException
    assert problem.JavaArrayStoreException is errors.JavaArrayStoreException
    assert problem._JavaEOFException is errors.JavaEOFException
    assert problem._JavaInvalidClassException is errors.JavaInvalidClassException
    assert problem._JavaClassNotFoundException is errors.JavaClassNotFoundException
    # no Java* exception class is defined outside mppflow.errors
    import inspect

    from mppflow import cli, graph, heuristic, network, pathfinder, puzzle
    for mod in (cli, experiments, graph, heuristic, model, network, pathfinder, planner, problem, puzzle, solvers):
        for name, obj in vars(mod).items():
            if inspect.isclass(obj) and issubclass(obj, BaseException) and obj.__name__.startswith("Java"):
                assert obj.__module__ == "mppflow.errors", (mod.__name__, name)
    for name in errors.__all__:
        obj = getattr(errors, name)
        if name.startswith("Java") and name != "JavaException":
            assert issubclass(obj, errors.JavaException) and obj.java_class.rsplit(".", 1)[-1] == name[4:], name


def test_api_doc_example_runs(tmp_path):
    """The usage example of docs/API.md prints exactly the output shown there."""
    with open(os.path.join(ROOT, "docs", "API.md"), encoding="utf-8") as f:
        doc = f.read()
    code = re.search(r"```python\n(.*?)```", doc, re.S).group(1)
    expected = re.search(r"```text\n(.*?)```", doc, re.S).group(1)
    assert len(code.strip().splitlines()) == 15
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, cwd=tmp_path,
                       env=child_env(), timeout=300)
    assert r.returncode == 0, r.stderr
    assert r.stdout == expected


def _readme():
    with open(os.path.join(ROOT, "README.md"), encoding="utf-8") as f:
        return f.read()


def test_readme_python_example_runs(tmp_path):
    """The Quick start's Python example prints what the README says it prints."""
    doc = _readme()
    code = re.search(r"## Quick start\n.*?```python\n(.*?)```", doc, re.S).group(1)
    expected = re.search(r"```\n\nprints `([^`]*)`", doc).group(1)
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, cwd=tmp_path,
                       env=child_env(), timeout=300)
    assert r.returncode == 0, r.stderr
    assert r.stdout == expected + "\n"


def test_readme_console_example_runs(tmp_path):
    """The Quick start's command line example: the instance, status, objective and lower
    bounds exactly; the plan statistics that the makespan objective does not fix (they
    differ between HiGHS versions, as the README says) only against their lower bounds;
    the time line only in form."""
    doc = _readme()
    block = re.search(r"## Quick start\n.*?```console\n\$ python -m mppflow (.*?)\n(.*?)```", doc, re.S)
    argv, shown = block.group(1).split(), block.group(2).splitlines()
    r = subprocess.run([sys.executable, "-m", "mppflow", *argv], capture_output=True, text=True,
                       cwd=tmp_path, env=child_env(), timeout=600)
    assert r.returncode == 0, r.stderr
    got = r.stdout.splitlines()
    assert len(got) == len(shown) == 4
    assert got[:2] == shown[:2]
    stats = re.compile(r"makespan (\d+) \(lb (\d+)\), total time (\d+) \(lb (\d+)\), "
                       r"total distance (\d+) \(lb (\d+)\), max distance (\d+)$")
    timing = re.compile(r"time \d+\.\d{3} s, \d+ models solved$")
    for lines in (got, shown):
        assert stats.match(lines[2]) and timing.match(lines[3]), lines
    g, s = (list(map(int, stats.match(x[2]).groups())) for x in (got, shown))
    assert g[0] == s[0] and g[1] == s[1] and g[3] == s[3] and g[5] == s[5]  # makespan, lower bounds
    assert g[2] >= g[3] and g[4] >= g[5] and g[1] <= g[6] <= g[0]  # total time, distance, max distance


def test_time_limits_are_statuses_not_exceptions():
    """``TimeLimitReached`` is exported but reserved (docs/API.md): an expired budget is a
    status of the result, and no module of the package raises the exception."""
    assert issubclass(mppflow.TimeLimitReached, mppflow.MPPError)
    p = mppflow.create_grid_problem(4, 4, 0.0, 4, mppflow.MathRandom(2))
    r = mppflow.plan(p, "total_time", time_limit=1e-9)
    assert (r.status, r.paths) == ("time_limit", None)
    pkg = os.path.dirname(os.path.abspath(mppflow.__file__))  # the imported package (also out of tree)
    scanned = set()
    for d, _, files in os.walk(pkg):
        for f in files:
            if f.endswith(".py"):
                with open(os.path.join(d, f), encoding="utf-8") as fh:
                    assert not re.search(r"raise\s+(errors\.)?TimeLimitReached", fh.read()), f
                scanned.add(f)
    assert {"__init__.py", "planner.py", "solvers.py"} <= scanned, (pkg, sorted(scanned))
    with open(os.path.join(ROOT, "docs", "API.md"), encoding="utf-8") as f:
        assert re.search(r"`TimeLimitReached` \| Reserved; mppflow does not raise it", f.read())


def test_validate_problem_and_tuple_forms():
    """``mppflow.validate`` (README, docs/API.md) on a Problem and on ``(graph, starts,
    goals)``: ``None`` for a valid plan, else the reason of :func:`path_set_error`."""
    g = mppflow.create_2d_grid_graph(3, 3)  # 0 1 2 / 3 4 5 / 6 7 8
    p = mppflow.Problem(g, [0, 2], [2, 0])
    assert mppflow.validate(p, [[0, 1, 2, 2, 2], [2, 5, 4, 3, 0]]) is None
    assert mppflow.validate(p, [[0, 1, 2], [2, 1, 0]]) == "Path 0 and 1 meet on vertex 1 at time step 1"
    assert mppflow.validate(p, [[0, 1, 2, 2], [2, 5, 4, 3]]) == \
        "Goal location for agent 1 is incorrect in the path set."
    assert mppflow.validate(p, [[0, 2], [2, 0]]) == "Path 0 is not valid at t = 0"
    # tuple form, in the order (graph, starts, goals)
    q = (g, [0, 1], [1, 2])
    assert mppflow.validate(q, [[0, 1], [1, 2]]) is None
    assert mppflow.validate(q, [[1, 2], [0, 1]]) == "Start location for agent 0 is incorrect in the path set."
    assert mppflow.validate(q, [[0, 1], [1, 1]]) == "Goal location for agent 1 is incorrect in the path set."
    assert mppflow.validate((g, [3, 4], [4, 3]), [[3, 4], [4, 3]]) == \
        "Path 0 and 1 cross the same edge at time step 1"
    assert mppflow.validate((g, [0, 1], [2, 2]), [[0, 1, 2], [1, 2, 2]]) == \
        "Path 0 and 1 meet on vertex 2 at time step 2"
