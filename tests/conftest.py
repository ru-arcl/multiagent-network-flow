"""Suite-wide pytest setup.

* The ``gurobi`` marker (pyproject.toml) is added to every test that needs Gurobi: the tests
  marked ``@needs_gurobi`` and the ``backend="gurobi"`` / ``ilp="gurobi"`` cases of the
  backend-parametrized tests (which also get ``needs_gurobi``). So ``-m "not gurobi"``
  deselects them all and ``-m gurobi`` selects them all. A test that uses Gurobi only when a
  license is there would escape both (and probe the license under ``-m "not gurobi"``):
  parametrize it by backend or split off its Gurobi part instead (checked on the whole fast
  tier by tests/test_suite_setup.py).
* The Gurobi license is probed once, after collection and deselection, and only if a selected
  test needs it (``helpers.gurobi_skip_reason``); the ``needs_gurobi`` tests skip with its
  reason. ``MPPFLOW_TEST_GUROBI=0`` skips them without a probe. The summary reports the probe.
* The HiGHS backend's two environment switches, ``MPPFLOW_HIGHS_ISOLATE`` and
  ``MPPFLOW_HIGHS_RAW_STDOUT`` (``mppflow.lp.highs``), are read when the module is imported.
  Every test runs with their defaults (in process, stray native line filtered), whatever the
  environment says; tests of the switches set them explicitly.
"""
from __future__ import annotations

import pytest

import helpers
from helpers import GUROBI_PROBE, GUROBI_SWITCH, gurobi_skip_reason

_GUROBI_PARAMS = ("backend", "ilp")


def _gurobi_param(item) -> bool:
    params = getattr(getattr(item, "callspec", None), "params", {})
    return any(params.get(k) == "gurobi" for k in _GUROBI_PARAMS)


@pytest.hookimpl(tryfirst=True)  # before -m deselects
def pytest_collection_modifyitems(config, items):
    for item in items:
        if _gurobi_param(item) and not item.get_closest_marker("needs_gurobi"):
            item.add_marker(helpers.needs_gurobi)
        if item.get_closest_marker("needs_gurobi") and not item.get_closest_marker("gurobi"):
            item.add_marker(pytest.mark.gurobi)


def pytest_report_header(config):
    return (f"gurobi: license probed only if a selected test needs it "
            f"({GUROBI_SWITCH}=0 skips those tests without a probe)")


def pytest_collection_finish(session):
    if session.config.option.collectonly or "reason" in GUROBI_PROBE:
        return
    n = sum(1 for item in session.items if item.get_closest_marker("needs_gurobi"))
    if not n:
        return
    tr = session.config.pluginmanager.get_plugin("terminalreporter")
    if tr is not None and not helpers.gurobi_probe_disabled():
        tr.write_line(f"gurobi: probing the license for {n} selected test(s) "
                      f"({GUROBI_SWITCH}=0 skips them) ...")
    gurobi_skip_reason()


def pytest_runtest_setup(item):
    if item.get_closest_marker("needs_gurobi"):
        reason = gurobi_skip_reason()
        if reason:
            pytest.skip(reason)


def pytest_terminal_summary(terminalreporter):
    if "reason" in GUROBI_PROBE:
        r = GUROBI_PROBE["reason"]
        terminalreporter.write_line(f"gurobi license probe: {'usable' if r is None else r} "
                                    f"({GUROBI_PROBE['seconds']:.1f} s)")


@pytest.fixture(autouse=True)
def _highs_defaults(monkeypatch):
    try:
        from mppflow.lp import highs
    except ImportError:  # numpy missing: nothing to reset
        return
    monkeypatch.setattr(highs, "ISOLATE", False)
    monkeypatch.setattr(highs, "FILTER_NATIVE_STDOUT", True)
