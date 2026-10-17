"""The browser engine's parity fixtures (``tests/parity/*.json``) are current, and the
JavaScript engine (``docs/js/engine.js``, ``docs/js/lp.js``) reproduces them exactly.

* Each fast fixture is regenerated in memory by :mod:`mppflow.parity` and compared with the
  committed file, text for text (so a change of the package that changes an engine function's
  output fails here until ``python -m mppflow.parity`` is run again).
* The slow tier (builtin solves of the 9- and 16-puzzle models, a few minutes of pure Python) is
  regenerated under the ``slow`` marker.
* If Node.js >= 18 is installed, the ``npm test`` script (``node --test docs/js/test/``, run
  with that ``node`` directly, so npm is not needed) runs ``docs/js/test/parity.test.js``
  against the fixtures (exact equality, bit-identical floats), and the engine's other tests
  (``docs/js/test/*.test.js``, the demo's precomputed plans included).
"""

from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess

import pytest

from mppflow import parity

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIXTURES = os.path.join(ROOT, "tests", "parity")
NODE = shutil.which("node")


def _committed(name: str) -> str:
    path = os.path.join(FIXTURES, name + ".json")
    assert os.path.exists(path), f"{path} is missing: run `python -m mppflow.parity`"
    with open(path, encoding="utf-8") as f:
        return f.read()


def _check_current(name: str) -> None:
    fresh = parity.dumps(parity.generate(name))
    old = _committed(name)
    if fresh != old:
        a, b = json.loads(old), json.loads(fresh)
        diff = [k for k in sorted(set(a["cases"]) | set(b["cases"])) if a["cases"].get(k) != b["cases"].get(k)]
        if a.get("instances") != b.get("instances"):
            diff.append("instances")
        pytest.fail(f"tests/parity/{name}.json is stale (groups {diff}): run `python -m mppflow.parity {name}`")


@pytest.mark.parametrize("name", sorted(parity.FAST))
def test_fast_fixture_is_current(name):
    _check_current(name)


@pytest.mark.slow
@pytest.mark.parametrize("name", sorted(parity.SLOW))
def test_slow_fixture_is_current(name):
    _check_current(name)


def test_fixture_sizes():
    total = 0
    for name in parity.FILES:
        size = len(_committed(name).encode("utf-8"))
        assert size < 2 * 1024 * 1024, f"{name}.json is {size} bytes"
        total += size
    assert total < 1.5 * 1024 * 1024, f"the parity fixtures total {total} bytes (limit 1.5 MB)"


def test_fixtures_cover_every_engine_function():
    """Every function of the engine contract has at least one fixture case."""
    groups = {}
    for name in parity.FILES:
        for k, v in json.loads(_committed(name))["cases"].items():
            groups[k] = groups.get(k, 0) + len(v)
    for g in ("rng", "hashset", "doubles", "grid", "holes", "grid_problem", "n2_puzzle", "from_adjacency", "bfs",
              "goals_in_range", "shortest_paths", "lower_bounds", "reachability", "compact_arcs", "network",
              "models", "lp", "split_goals", "solvers", "plan_split", "checks"):
        assert groups.get(g, 0) > 0, g
    models = json.loads(_committed("models"))["cases"]["models"]
    assert {m["kind"] for m in models} >= {"makespan", "total_time", "total_distance", "max_distance"}
    plans = [c for n in ("planners", "planners_slow") for c in json.loads(_committed(n))["cases"]["plan_split"]]
    assert {c["k"] for c in plans} >= {1, 2, 4}


def _node_major() -> int | None:
    """The major version of ``node`` (``None`` if it is missing or does not answer)."""
    if NODE is None:
        return None
    try:
        v = subprocess.run([NODE, "--version"], capture_output=True, text=True, timeout=60).stdout.strip()
        return int(v.lstrip("v").split(".")[0])
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


NODE_MIN = 18  # node --test (package.json "engines")


def _npm_test_argv() -> list[str]:
    """package.json's ``test`` script with ``node`` replaced by :data:`NODE`: the script is a
    single ``node`` command, so it runs without npm (Debian and Ubuntu package npm separately,
    and an ``npm`` on the PATH may belong to another, older Node)."""
    with open(os.path.join(ROOT, "package.json"), encoding="utf-8") as f:
        argv = shlex.split(json.load(f)["scripts"]["test"])
    assert argv[0] == "node" and not {"&&", "||", ";", "|"} & set(argv), argv
    return [NODE, *argv[1:]]


@pytest.mark.skipif(NODE is None, reason="Node.js not installed")
def test_npm_test_passes():
    major = _node_major()
    if major is None or major < NODE_MIN:
        pytest.skip(f"Node.js >= {NODE_MIN} required (found {major})")
    r = subprocess.run(_npm_test_argv(), cwd=ROOT, capture_output=True, text=True, timeout=1800)
    assert r.returncode == 0, (r.stdout[-6000:] + "\n" + r.stderr[-3000:])


def test_default_output_is_tests_parity_in_a_checkout():
    """From a checkout the default output directory is tests/parity."""
    if os.path.dirname(os.path.dirname(os.path.abspath(parity.__file__))) != ROOT:
        pytest.skip("mppflow is imported from an installed package, not from this checkout")
    assert parity.default_out() == FIXTURES


def test_default_output_only_in_a_checkout(monkeypatch, tmp_path):
    """An installed package (no tests/ and docs/js/ next to it) needs --out instead of writing
    into site-packages/tests/parity."""
    monkeypatch.setattr(parity, "HERE", str(tmp_path / "site-packages" / "mppflow"))
    assert parity.default_out() is None
    with pytest.raises(ValueError, match="checkout"):
        parity.write(names=["java"], verbose=False)
    with pytest.raises(SystemExit):
        parity.main(["java"])
    parity.write(str(tmp_path / "out"), ["java"], verbose=False)
    assert (tmp_path / "out" / "java.json").exists()
