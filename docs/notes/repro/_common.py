"""Shared helpers for the reproduction scripts in docs/notes/repro/.

Run any script from anywhere, e.g. ``python3 docs/notes/repro/eq15_off_by_one.py``. This
module puts the repository root on ``sys.path``, so ``mppflow`` does not need to be
installed. Scripts that solve ILPs use HiGHS (scipy >= 1.15.3) unless they say otherwise.
"""
from __future__ import annotations

import atexit
import os
import shutil
import sys
import tempfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def _silence_native_stdout() -> None:
    """Send what native code writes to file descriptor 1 to /dev/null, keeping Python's
    output. scipy 1.17.1's HiGHS prints a stray debug line ("HighsMipSolverData::
    transformNewIntegerFeasibleSolution tmpSolver.run();") on some MIPs; see
    docs/notes/solver-notes.md."""
    try:
        sys.stdout.flush()
        real = os.dup(1)
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, 1)
        os.close(devnull)
        sys.stdout = os.fdopen(real, "w", buffering=1)
    except OSError:  # pragma: no cover - no POSIX file descriptors
        pass


_silence_native_stdout()

from mppflow.graph import Graph, create_2d_grid_graph  # noqa: E402


def edges_graph(n: int, edges) -> Graph:
    """Undirected graph on ids 0..n-1 (every vertex gets its wait self-loop)."""
    adj = [[] for _ in range(n)]
    for u, v in edges:
        adj[u].append(v)
        adj[v].append(u)
    return Graph.from_adjacency(adj)


def grid(rows: int, cols: int) -> Graph:
    """4-connected rows x cols grid, ids row * cols + col."""
    return create_2d_grid_graph(rows, cols, True)


def show(label: str, fn, *args, **kwargs):
    """Print ``label: result`` or ``label: ExceptionClass: message``; return the result."""
    try:
        r = fn(*args, **kwargs)
    except Exception as e:  # noqa: BLE001 - we want to show every outcome
        print(f"{label}: {type(e).__name__}: {e}")
        return None
    print(f"{label}: {r}")
    return r


def scratch_dir(chdir: bool = False) -> str:
    """A new empty temporary directory (the working directory from now on if ``chdir``),
    removed when the script exits, after changing back to the starting directory."""
    d = tempfile.mkdtemp(prefix="mppflow-repro-")
    home = os.getcwd()

    def cleanup():
        os.chdir(home)
        shutil.rmtree(d, ignore_errors=True)

    atexit.register(cleanup)
    if chdir:
        os.chdir(d)
    return d


def have_gurobi() -> bool:
    """True when gurobipy imports and its license solves a full-size model. The license
    bundled with pip's gurobipy is limited to 2000 variables and constraints, too small for
    these scripts, so it counts as no license (the scripts then use HiGHS), as in
    tests/helpers.py. ``MPPFLOW_TEST_GUROBI=0`` answers False without probing (a token-server
    license whose server cannot be reached makes the probe wait about a minute)."""
    if os.environ.get("MPPFLOW_TEST_GUROBI", "").strip() == "0":
        return False
    try:
        import gurobipy as gp
    except ImportError:
        return False
    try:
        with gp.Env(empty=True) as env:
            env.setParam("OutputFlag", 0)
            env.start()
            with gp.Model(env=env) as m:
                x = m.addVars(2100, ub=1.0)
                m.addConstr(x.sum() >= 1)
                m.optimize()
    except gp.GurobiError:  # no license, or the size-limited one
        return False
    return True


def moves(paths) -> int:
    """Total number of moves (waits are free)."""
    return sum(p[t] != p[t + 1] for p in paths for t in range(len(p) - 1))


def arrival_times(paths, goals) -> list[int]:
    """Final arrival time of each robot: the first t after which it stays at its goal."""
    out = []
    for p, g in zip(paths, goals):
        t = len(p) - 1
        while t > 0 and p[t - 1] == g:
            t -= 1
        out.append(t)
    return out
