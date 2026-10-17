"""Shared test helpers: decoders for the golden records, data loaders, small graphs.

Import with ``from helpers import ...`` (``tests/`` is on ``pythonpath`` in pyproject.toml).
The golden record format is described in ``tools/golden/README.md``.
"""

from __future__ import annotations

import gzip
import json
import os
import threading
import time

import pytest

from mppflow.graph import Graph

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
"""The checkout (the directory above ``tests/``)."""


def child_env(**extra: str) -> dict:
    """``os.environ`` for a child interpreter that imports this checkout's ``mppflow`` first:
    :data:`ROOT` is prepended to ``PYTHONPATH`` and the caller's entries are kept (numpy and
    scipy may be reachable only through them: ``pip install --target``, Nix, HPC modules).
    ``extra`` sets more variables."""
    env = dict(os.environ, **extra)
    old = os.environ.get("PYTHONPATH")
    env["PYTHONPATH"] = ROOT + os.pathsep + old if old else ROOT
    return env

GOLDEN = os.path.join(HERE, "golden")
"""Records of the live original (``tests/golden/<group>.jsonl``)."""
CORE_DATA = os.path.join(HERE, "data", "core")
"""Generator / pathfinder / problem cases (``tests/data/core/<name>.json.gz``)."""
PAPER_DATA = os.path.join(HERE, "data", "paper")
"""Worked examples of the papers (``tests/data/paper/<name>.json``)."""

GUROBI_SWITCH = "MPPFLOW_TEST_GUROBI"
"""Environment switch: ``MPPFLOW_TEST_GUROBI=0`` skips the Gurobi tests without probing the
license (a token-server license whose server cannot be reached makes the probe wait about a
minute before it fails)."""


def _probe_gurobi() -> str | None:
    """``None`` if gurobipy imports and its license solves a model above the size limit of
    the license bundled with pip's gurobipy (2000 variables and constraints), else why not."""
    try:
        import gurobipy as gp
    except ImportError:
        return "gurobipy not installed"
    try:
        with gp.Env(empty=True) as env:
            env.setParam("OutputFlag", 0)
            env.start()
            with gp.Model(env=env) as m:
                x = m.addVars(2100, ub=1.0)
                m.addConstr(x.sum() >= 1)
                m.optimize()
    except gp.GurobiError as e:  # no license, or the size-limited one
        return f"Gurobi license missing or size-limited ({e})"
    return None


_GUROBI_PROBE_LOCK = threading.Lock()
GUROBI_PROBE: dict = {}
"""The probe's result once it has run in this process: ``reason`` (``None`` = usable) and
``seconds`` (how long it took)."""


def gurobi_probe_disabled() -> bool:
    """Whether :data:`GUROBI_SWITCH` is ``0`` (the Gurobi tests skip without a probe)."""
    return os.environ.get(GUROBI_SWITCH, "").strip() == "0"


def gurobi_skip_reason() -> str | None:
    """``None`` if gurobipy imports and has a license for full-size models (pip's bundled
    license is limited to 2000 variables and constraints, too small for most tests), else why
    not. The license is probed on the first call only (not at import: tests/conftest.py calls
    this once collection is done, and only if a selected test needs Gurobi)."""
    with _GUROBI_PROBE_LOCK:  # one probe, also when tests call this from threads
        if "reason" not in GUROBI_PROBE:
            t = time.monotonic()
            reason = (f"Gurobi tests disabled ({GUROBI_SWITCH}=0)" if gurobi_probe_disabled()
                      else _probe_gurobi())
            GUROBI_PROBE.update(reason=reason, seconds=time.monotonic() - t)
    return GUROBI_PROBE["reason"]


needs_gurobi = pytest.mark.needs_gurobi
"""Marker for tests that need gurobipy and a (full-size) Gurobi license: tests/conftest.py
skips them when :func:`gurobi_skip_reason` gives a reason, and gives them, and the
``backend="gurobi"`` / ``ilp="gurobi"`` cases of the backend-parametrized tests, the
``gurobi`` marker."""
GRB = {"Threads": 1}
"""Gurobi parameters for exact comparisons with the golden records (they use one thread)."""


# ------------------------------------------------------------------ golden data
def graph_from_json(d: dict) -> Graph:
    """Decode a golden-record graph: ``vertices`` = ``[[id, x, y] | [id], ...]`` in array
    order, ``adj`` = ``[[v, [neighbours...]], ...]``, optional ``rows``/``cols`` and
    ``adj_type`` (``"HashSet"``: the neighbour lists are in the original's HashSet
    iteration order, kept in ``Graph.adj_order``)."""
    g = Graph()
    for e in d["vertices"]:
        g.xy[e[0]] = (e[1], e[2]) if len(e) == 3 else None
    g.vertices = [e[0] for e in d["vertices"]]
    hs = d.get("adj_type") == "HashSet"
    for v, nb in d["adj"]:
        g.adj[v] = sorted(nb)
        if hs:
            if g.adj_order is None:
                g.adj_order = {}
            g.adj_order[v] = list(nb)
    g.rows, g.cols = d.get("rows", 0), d.get("cols", 0)
    return g


def load_golden(group: str) -> list[dict]:
    """All records of ``tests/golden/<group>.jsonl`` (``group`` with or without ``.jsonl``)."""
    name = group if group.endswith(".jsonl") else group + ".jsonl"
    with open(os.path.join(GOLDEN, name), encoding="utf-8") as f:
        return [json.loads(line) for line in f]


def golden_records(group: str, fn_part: str) -> list[dict]:
    """The records of ``group`` whose ``fn`` contains ``fn_part``."""
    return [r for r in load_golden(group) if fn_part in r["fn"]]


def load_core(name: str):
    """``tests/data/core/<name>.json.gz``."""
    with gzip.open(os.path.join(CORE_DATA, name + ".json.gz"), "rt", encoding="utf-8") as f:
        return json.load(f)


def load_paper(name: str):
    """``tests/data/paper/<name>.json``."""
    with open(os.path.join(PAPER_DATA, name + ".json"), encoding="utf-8") as f:
        return json.load(f)


def case_id(c: dict) -> str:
    """pytest id of a ``tests/data/core`` case: ``kind-args-sSEED``."""
    return f"{c['kind']}-{json.dumps(c['args'], sort_keys=True)}-s{c['seed']}"


def java_class(e: BaseException) -> str:
    """The Java class an exception stands for (``java_class``), else its Python name."""
    return getattr(e, "java_class", type(e).__name__)


# ------------------------------------------------------------------ small graphs
def from_edges(n: int, edges) -> Graph:
    """Undirected graph on ids ``0..n-1`` from an edge list (every vertex gets its wait
    self-loop)."""
    adj = [[] for _ in range(n)]
    for u, v in edges:
        adj[u].append(v)
        adj[v].append(u)
    return Graph.from_adjacency(adj)


def ring(n: int) -> Graph:
    """The cycle ``0 - 1 - ... - (n-1) - 0``."""
    return from_edges(n, [(i, (i + 1) % n) for i in range(n)])
