"""Consistency checks of the golden fixtures themselves (no Java needed).

The fixtures are produced by tools/golden/record.py; see tools/golden/README.md.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
GROUPS = ["graph", "problem", "pathfinder", "models", "solvers", "planner", "heuristic", "main"]
OUTCOMES = ("result", "exception", "hang", "jvm_exit")


def load(group):
    return [json.loads(line) for line in (HERE / f"{group}.jsonl").read_text(encoding="utf-8").splitlines()
            if line.strip()]


def test_meta():
    meta = json.loads((HERE / "META.json").read_text())
    assert meta["gurobi_threads"] == 1
    assert set(meta["groups"]) == set(GROUPS)
    for g in GROUPS:
        assert meta["groups"][g]["cases"] == len(load(g))


@pytest.mark.parametrize("group", GROUPS)
def test_records(group):
    ids = set()
    for r in load(group):
        assert r["id"].startswith(group + "/"), r["id"]
        assert r["id"] not in ids
        ids.add(r["id"])
        assert isinstance(r["fn"], str) and isinstance(r["args"], dict) and isinstance(r["seed"], int)
        assert sum(k in r for k in OUTCOMES) == 1, r["id"]
        assert "jvm_exit" not in r, r["id"]
        if r.get("hang"):
            assert "hang" in r.get("tags", []), r["id"]
            assert "rng_next" not in r
        else:
            assert isinstance(r["rng_next"], float) and 0.0 <= r["rng_next"] < 1.0
            assert isinstance(r["draws"], int)
        if "exception" in r:
            assert r["exception"]["class"].startswith("java."), r["id"]
        assert set(r.get("nondet", [])) <= set(r)


def test_model_dumps():
    n = 0
    for r in load("models"):
        if not r["fn"].endswith(".prepareModel"):
            continue
        res = r["result"]
        assert res["params"]["Threads"] == 1
        if "dump" in res:
            text = res["dump"]
            assert hashlib.sha256(text.encode("utf-8")).hexdigest() == res["sha256"]
            lines = text.splitlines()
            assert lines[0] in ("SENSE max", "SENSE min")
            assert lines[1] == f"NCOLS {res['ncols']}" and lines[2] == f"NROWS {res['nrows']}"
            assert len(lines) == 3 + res["ncols"] + res["nrows"]
            n += 1
    assert n > 0
