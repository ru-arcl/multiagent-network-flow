"""The demo's precomputed plans (docs/data/demo-data.js, built by docs/data/build_demo_data.py
with Gurobi) against the package: every plan is valid and its stored metrics are mppflow's,
and on the small maps the package (HiGHS) still proves the stored exact (k = 1) values optimal.
So a change of the package that changes a value or a status the page shows fails here, not
only in tools/check_demo.mjs (Chrome, run by hand). docs/js/test/demo-data.test.js checks the
same data against the JavaScript engine.
"""
from __future__ import annotations

import json
import os
import re

import pytest

import mppflow

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_JS = os.path.join(ROOT, "docs", "data", "demo-data.js")
METRIC = {"makespan": "makespan", "total_time": "totalTime", "total_distance": "totalDistance",
          "max_distance": "maxDistance"}


def _load() -> dict:
    with open(DATA_JS, encoding="utf-8") as f:
        text = f.read()
    m = re.search(r"root\.MPP_DEMO = (.*?);\n  if \(typeof module", text, re.S)
    assert m, "docs/data/demo-data.js: no root.MPP_DEMO = {...};"
    return json.loads(m.group(1))


DATA = _load()
MAPS = {m["id"]: m for m in DATA["maps"]}
SMALL = [mid for mid, m in MAPS.items() if len(m["starts"]) <= 9]  # 9-puzzle, Figs. 4(a) and 8, star
EXACT = [(mid, key) for mid in SMALL for key, p in MAPS[mid]["plans"].items() if key != "paper" and p["k"] == 1]


def _problem(m):
    """The map as ``(graph, starts, goals)`` (``graph.adj`` lists the wait move, which
    :func:`mppflow.from_adjacency` adds itself)."""
    adj = [[u for u in nb if u != v] for v, nb in enumerate(m["graph"]["adj"])]
    return mppflow.from_adjacency(adj, m["graph"]["xy"]), list(m["starts"]), list(m["goals"])


def test_small_maps_present():
    assert len(SMALL) >= 4 and len(EXACT) >= 16, (SMALL, EXACT)


@pytest.mark.parametrize("mid", sorted(MAPS))
def test_plans_are_valid_with_their_metrics(mid):
    m = MAPS[mid]
    g, s, t = _problem(m)
    # the original's makespan bound counts the vertices of the longest shortest path, one more
    # than its length (the build script's _lower_bounds); a private stream for the BFS draws
    rng = mppflow.MathRandom(0)
    ms = mppflow.get_makespan_lower_bound(g, s, t, rng) - 1
    tt = mppflow.get_total_time_lower_bound(g, s, t, rng)
    assert m["lower_bounds"] == {"makespan": ms, "total_time": tt, "total_distance": tt, "max_distance": ms}
    for key, p in m["plans"].items():
        assert mppflow.path_set_error(p["paths"], g, s, t) is None, key
        met = mppflow.plan_metrics(p["paths"], t)
        assert p["metrics"] == {METRIC[k]: met[k] for k in METRIC}, key
        assert p["rotations"] == mppflow.has_cycles(p["paths"]), key
        if key != "paper":
            assert p["value"] == met[p["objective"]] >= m["lower_bounds"][p["objective"]], key
            assert p["status"] == "feasible" or (p["status"] == "optimal" and p["k"] == 1), key


@pytest.mark.parametrize("mid,key", EXACT, ids=[f"{a}-{b}" for a, b in EXACT])
def test_exact_values_are_still_optimal(mid, key):
    """The stored k = 1 status and value are what mppflow.plan gives today (HiGHS; the plan
    itself may be another optimal one)."""
    m = MAPS[mid]
    p = m["plans"][key]
    r = mppflow.plan(_problem(m), p["objective"], 1, backend="highs", horizon=p.get("horizon"))
    assert (r.status, r.value) == (p["status"], p["value"])
