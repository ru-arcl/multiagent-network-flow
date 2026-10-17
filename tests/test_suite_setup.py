"""The suite's own setup (tests/conftest.py, tests/helpers.py): the Gurobi license probe.

Regression (TS1A-GUROBI-PROBE-BLOCKS-COLLECTION): the probe ran when tests/helpers.py was
imported, so every pytest process paid it before marker selection; with a token-server
license whose server cannot be reached that is about a minute, also for ``-m "not gurobi"``
and for a single file without Gurobi tests.

Regression (TSW-NOT-GUROBI-STILL-PROBES-AND-SOLVES): tests without the ``gurobi`` marker called
``helpers.have_gurobi()`` in their body and then solved with Gurobi, so ``-m "not gurobi"``
still probed the license and ran Gurobi solves (and a licensed machine never ran their HiGHS
branch). They are now backend-parametrized or split into a ``needs_gurobi`` part, and the
whole suite is run here under ``-m "not gurobi"`` with a gurobipy that records every use.
"""
from __future__ import annotations

import os
import subprocess
import sys

import pytest

from helpers import GUROBI_SWITCH, ROOT, child_env

# gurobipy that fails loudly when imported: no probe may run where no selected test needs it
FAKE_GUROBIPY = 'raise RuntimeError("gurobipy imported")\n'
# gurobipy without a license
UNLICENSED_GUROBIPY = """\
class GurobiError(Exception):
    pass


class Env:
    def __init__(self, *a, **k):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def setParam(self, *a):
        pass

    def start(self):
        raise GurobiError("fake: no license")
"""


# gurobipy that imports (so the backend-parametrized tests get their gurobi cases, which -m
# "not gurobi" leaves out) and records each license start or model, with the test that made it
SPY_GUROBIPY = """\
import os


class GurobiError(Exception):
    pass


class GRB:
    class Status:
        pass


def _used(what):
    with open(os.environ["MPPFLOW_SPY_LOG"], "a", encoding="utf-8") as f:
        f.write(what + " " + os.environ.get("PYTEST_CURRENT_TEST", "(outside a test)") + "\\n")
    raise GurobiError("spy: Gurobi used")


class Env:
    def __init__(self, *a, **k):
        _used("Env")


class Model:
    def __init__(self, *a, **k):
        _used("Model")
"""


def _pytest(tmp_path, *args, gurobipy=FAKE_GUROBIPY, switch=None, timeout=600, **env):
    fake = tmp_path / "fake"
    fake.mkdir(exist_ok=True)
    (fake / "gurobipy.py").write_text(gurobipy, encoding="utf-8")
    e = child_env(**env)
    e["PYTHONPATH"] = str(fake) + os.pathsep + e["PYTHONPATH"]
    e.pop(GUROBI_SWITCH, None)
    e.pop("PYTEST_CURRENT_TEST", None)  # this test's: the child sets its own
    if switch is not None:
        e[GUROBI_SWITCH] = switch
    return subprocess.run([sys.executable, "-m", "pytest", "-q", "-rs", "-p", "no:cacheprovider", *args],
                          cwd=ROOT, env=e, capture_output=True, text=True, timeout=timeout)


def test_importing_helpers_does_not_probe_gurobi():
    code = ("import sys; sys.path[:0] = ['tests', '.']; import helpers, conftest; "
            "assert 'gurobipy' not in sys.modules and not helpers.GUROBI_PROBE")
    subprocess.run([sys.executable, "-c", code], cwd=ROOT, check=True, env=child_env())


def test_no_probe_without_a_selected_gurobi_test(tmp_path):
    for args in (["tests/test_graph.py"], ["-m", "not gurobi", "-k", "gurobi or isolated", "tests/test_lp.py"]):
        r = _pytest(tmp_path, *args)
        assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-2000:]
        assert "gurobipy imported" not in r.stdout + r.stderr and "gurobi license probe" not in r.stdout


def test_switch_skips_the_gurobi_tests_without_a_probe(tmp_path):
    r = _pytest(tmp_path, "-k", "test_gurobi_lps_and_statuses", "tests/test_lp.py", switch="0")
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-2000:]
    assert "1 passed" not in r.stdout and "1 skipped" in r.stdout
    assert f"Gurobi tests disabled ({GUROBI_SWITCH}=0)" in r.stdout
    assert "gurobipy imported" not in r.stdout + r.stderr


def test_probe_runs_once_for_the_selected_gurobi_tests(tmp_path):
    """Without a license, the selected Gurobi tests skip with the probe's reason; the probe
    runs once, after collection, announced before and reported in the summary."""
    r = _pytest(tmp_path, "-k", "test_gurobi_lps_and_statuses or test_gurobi_random_milps_agree",
                "tests/test_lp.py", gurobipy=UNLICENSED_GUROBIPY)
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-2000:]
    assert "2 skipped" in r.stdout and r.stdout.count("fake: no license") == 2, r.stdout  # -rs line, summary
    assert r.stdout.count("gurobi: probing the license for 2 selected test(s)") == 1
    assert "gurobi license probe: Gurobi license missing or size-limited (fake: no license)" in r.stdout


@pytest.mark.slow
def test_not_gurobi_never_uses_gurobi(tmp_path):
    """The whole fast tier under ``-m "not gurobi"``: no license probe, no Gurobi environment or
    model (this file is left out: it runs pytest itself)."""
    log = tmp_path / "gurobi-uses.txt"
    r = _pytest(tmp_path, "-m", "not gurobi and not slow", "--ignore", os.path.join("tests", "test_suite_setup.py"),
                "tests", gurobipy=SPY_GUROBIPY, timeout=3600, MPPFLOW_SPY_LOG=str(log))
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-2000:]
    assert " passed" in r.stdout and "gurobi license probe" not in r.stdout, r.stdout[-3000:]
    assert not log.exists(), "Gurobi used by tests without the gurobi marker:\n" + log.read_text(encoding="utf-8")


def test_spy_gurobipy_records_a_use(tmp_path):
    """The spy of :func:`test_not_gurobi_never_uses_gurobi` sees the license probe that a
    selected Gurobi test causes (after collection, outside the tests)."""
    log = tmp_path / "gurobi-uses.txt"
    r = _pytest(tmp_path, "-k", "test_gurobi_lps_and_statuses", "tests/test_lp.py", gurobipy=SPY_GUROBIPY,
                MPPFLOW_SPY_LOG=str(log))
    assert "1 skipped" in r.stdout and "gurobi license probe: Gurobi license missing" in r.stdout, r.stdout[-3000:]
    assert log.read_text(encoding="utf-8") == "Env (outside a test)\n"
