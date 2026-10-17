"""HiGHS backend through ``scipy.optimize.milp`` (scipy >= 1.15.3): the open-source default.

Older scipy releases are not supported (:data:`MIN_SCIPY`; a solve warns once): before 1.11
``milp`` ignores ``mip_rel_gap`` (so ``gap=`` has no effect), and the HiGHS of scipy
1.11-1.14 can report "optimal" for a solution that violates the flow constraints (plan
extraction then fails), maps an iteration limit to the time-limit status and gets some
infeasible models wrong. scipy 1.15.0-1.15.2 copy the column duals out of HiGHS one element
at a time, refetching the whole list each time: quadratic in the number of columns, so a
model with 40000 columns (a 2-vertex instance at ``T = 10000``) had not returned after 150 s
there; 1.15.3 solves it in about 7 s.

The model is passed as one sparse matrix in row order (repeated terms of a row are summed).
A maximized objective is negated for ``milp`` and the objective and bound are negated back.

Status mapping. The HiGHS model status is read from the ``"HiGHS Status <code>"`` part of
the scipy message (scipy's own ``status`` folds several HiGHS codes into 4); it is kept in
``extra["highs_model_status"]``:

* 7 ``kOptimal`` -> ``optimal``;
* 8 ``kInfeasible`` and 9 ``kUnboundedOrInfeasible`` -> ``infeasible`` (as Gurobi's
  ``INF_OR_UNBD`` is mapped by the Gurobi backend);
* 10 ``kUnbounded`` -> ``unbounded``;
* 13 ``kTimeLimit`` and 14 ``kIterationLimit`` -> ``time_limit``;
* 16 ``kSolutionLimit`` -> ``node_limit``. HiGHS reports a reached ``mip_max_nodes`` (scipy's
  ``node_limit`` option) as "Solution limit reached"; scipy does not expose HiGHS's other
  solution-limit options, so through scipy this code means the node limit;
* any other code -> ``feasible`` if HiGHS returned a solution; without one a
  :class:`RuntimeError` is raised (a solver error is not a proof of infeasibility).

Integer bounds. For an integer or binary column (not under ``relax``) the bounds are first
rounded inward, ``ceil(lb - INT_EPS)`` and ``floor(ub + INT_EPS)``, exactly as the builtin
backend does (binary bounds are also intersected with ``[0, 1]``). The presolve of scipy
1.17.1's HiGHS gives wrong answers on integer columns with fractional bounds ("infeasible" on
feasible models, "optimal" on suboptimal points; scipy 1.15.3's gives the second); with the
rounded bounds it agrees with builtin and Gurobi on those models (but see "Crashes" below for
one with integral bounds). The rounding
does not change the feasible set, and bounds that are already integral (all MPP models) are
passed unchanged.

Presolve fallback. The presolve of scipy 1.17.1's HiGHS (HiGHS 1.12.0) sometimes ends a
small (infeasible) MIP with status 4 ``Solve error`` and no solution. When a solve ends with
an error or unknown code and no solution, and presolve was not already switched off, the
model is solved once more with ``presolve=False`` (with the time that is left, if there is
a time limit). The second result is used if its code is a definite one (any code mapped
above, or a returned solution); otherwise :class:`RuntimeError` is raised.
``extra["highs_retry"]`` then holds the first attempt's code and message, and ``runtime``
covers both attempts.

At a limit, ``x`` holds the incumbent when HiGHS returns one (otherwise ``None``).
``mip_gap`` is always :func:`~mppflow.lp.linear.relative_gap` of the reported ``objective``
and ``bound`` (both include ``obj_constant``, which HiGHS never sees), i.e. Gurobi's
``MIPGap`` definition, not HiGHS's own gap.

Native stdout. scipy 1.17.1's HiGHS prints a stray debug line,
``HighsMipSolverData::transformNewIntegerFeasibleSolution tmpSolver.run();``, from C code
on some MIPs, even with ``disp=False``. It goes into the C ``stdout`` buffer and usually
surfaces much later (at process exit, or after pytest has stopped capturing). Each solve
therefore runs inside :func:`native_stdout_filtered`: file descriptor 1 is pointed at a
temporary file, the C buffers are flushed with ``fflush(NULL)`` at the end, and everything
captured except that debug line is written back to the real file descriptor 1. Nothing
else is lost, only delayed until the solve ends (also output of other threads during the
solve). The redirect is skipped, and the solve runs unchanged, when:

* :data:`FILTER_NATIVE_STDOUT` is ``False`` (or the environment variable
  ``MPPFLOW_HIGHS_RAW_STDOUT`` is set to a non-empty value other than ``0``);
* the caller asked for HiGHS output (``params={"disp": True}``);
* the platform has no C library ``fflush`` reachable through ``ctypes`` (e.g. Windows);
* file descriptor 1 is not open (e.g. a daemon), or cannot be duplicated.

``sys.stdout`` itself is never replaced, so this works under pytest's capture (both the
``fd`` and the ``sys`` methods) and in Jupyter, where ``sys.stdout`` has no file
descriptor; there fd 1 is the kernel's own stdout. Concurrent solves in several threads
share one redirect (reference counted under a lock).

Crashes. scipy 1.17.1's presolve can also crash the process (segmentation fault) on some
small general-integer models (integer columns with non-binary bounds), even with integral
bounds, and on the same models it occasionally reports "infeasible" for a feasible model
(``docs/notes/repro/highs_presolve_bug.py``, model m320: about 95% crashes, 5% wrong
"infeasible"; the optimum is 1.0). Neither the bound rounding nor the presolve fallback can
help in-process. With ``params={"isolate": True}`` (or the environment variable
``MPPFLOW_HIGHS_ISOLATE=1``) every ``milp`` call runs in a worker process: a fresh
interpreter (``python -c``; not ``fork``, since a forked child inherits a HiGHS thread pool
without its threads and deadlocks, and not ``multiprocessing``'s ``spawn``, which re-imports
the caller's main script). It is reused between solves, and its native output is
discarded. A crashed worker counts as an error without a solution, so the presolve fallback
runs (also isolated, in a new worker), and :class:`RuntimeError` is raised if that crashes
too. On models with general-integer columns an "infeasible" answer is also confirmed with
presolve off. Isolation costs the model's transfer per solve (and about a second to start a
worker), so it is off by default. No MPP model has triggered the
crash so far: their integer columns are binary arcs and counters with bounds ``[0, T]``.

This backend is new (the original always uses Gurobi); it has no ``compat`` switch.
"""

from __future__ import annotations

import contextlib
import math
import os
import re
import sys
import tempfile
import threading
import time
import warnings

import numpy as np

from .linear import INF, LinearModel, Solution, relative_gap

INT_EPS = 1e-6
"""Integrality tolerance used to round integer bounds (the same as ``builtin.INT_EPS``)."""

MIN_SCIPY = (1, 15, 3)
"""The oldest scipy release whose HiGHS the test suite passes on (see the module docstring)."""

_scipy_checked = False


def _check_scipy() -> None:
    """Warn once (``RuntimeWarning``) when scipy is older than :data:`MIN_SCIPY`."""
    global _scipy_checked
    if _scipy_checked:
        return
    _scipy_checked = True
    import scipy

    try:
        v = tuple(int(x) for x in re.findall(r"\d+", scipy.__version__)[:3])
    except ValueError:
        return
    if v < MIN_SCIPY:
        need = ".".join(map(str, MIN_SCIPY))
        warnings.warn(f"mppflow needs scipy >= {need} for the HiGHS backend, found {scipy.__version__}: "
                      "older HiGHS releases can return wrong results (gap= is ignored before 1.11), and "
                      "before 1.15.3 large models take quadratic time", RuntimeWarning, stacklevel=3)


_DEFINITE = (7, 8, 9, 10, 13, 14, 16)
"""HiGHS model status codes that ``solve`` maps to a status without needing a solution."""

STRAY_LINE = b"HighsMipSolverData::transformNewIntegerFeasibleSolution tmpSolver.run();"
"""The debug line scipy 1.17.1's HiGHS prints to the C ``stdout`` (dropped by
:func:`native_stdout_filtered`)."""

FILTER_NATIVE_STDOUT = os.environ.get("MPPFLOW_HIGHS_RAW_STDOUT", "") in ("", "0")
"""Run solves inside :func:`native_stdout_filtered` (default ``True``)."""


def _load_fflush():
    if os.name != "posix":
        return None
    try:
        import ctypes

        f = ctypes.CDLL(None).fflush
    except (OSError, AttributeError, ImportError):  # pragma: no cover - unusual libc
        return None
    f.argtypes = [ctypes.c_void_p]
    f.restype = ctypes.c_int
    return f


_fflush = _load_fflush()
_lock = threading.Lock()
_state = {"depth": 0, "saved": None, "tmp": None}


def _flush_all() -> None:
    try:
        sys.stdout.flush()
    except Exception:  # noqa: BLE001 - a closed or exotic sys.stdout must not break a solve
        pass
    _fflush(None)


def _enter() -> bool:
    """Start (or join) the redirect of fd 1; ``False`` if it cannot be done here."""
    with _lock:
        if _state["depth"] > 0:
            _state["depth"] += 1
            return True
        try:
            os.fstat(1)
        except OSError:
            return False
        _flush_all()
        tmp = None
        try:
            tmp = tempfile.TemporaryFile()
            saved = os.dup(1)
        except OSError:
            if tmp is not None:
                tmp.close()
            return False
        try:
            os.dup2(tmp.fileno(), 1)
        except OSError:
            os.close(saved)
            tmp.close()
            return False
        _state.update(depth=1, saved=saved, tmp=tmp)
        return True


def _exit() -> None:
    with _lock:
        _state["depth"] -= 1
        if _state["depth"] > 0:
            return
        saved, tmp = _state["saved"], _state["tmp"]
        _state.update(saved=None, tmp=None)
        try:
            _flush_all()
        finally:
            os.dup2(saved, 1)
            os.close(saved)
        try:
            tmp.seek(0)
            data = tmp.read()
        finally:
            tmp.close()
    if data:
        _replay(data)


def _replay(data: bytes) -> None:
    """Write ``data`` minus the stray HiGHS lines back to fd 1."""
    lines = data.splitlines(keepends=True)
    keep = b"".join(ln for ln in lines if ln.rstrip(b"\r\n") != STRAY_LINE)
    try:
        sys.stdout.flush()
    except Exception:  # noqa: BLE001
        pass
    view = memoryview(keep)
    while view:
        try:
            n = os.write(1, view)
        except OSError:  # pragma: no cover - fd 1 went away
            return
        view = view[n:]


@contextlib.contextmanager
def native_stdout_filtered(enabled: bool = True):
    """Drop :data:`STRAY_LINE` from what native code writes to fd 1 inside the block; all
    other output (native or Python) is kept and written out when the block ends. A no-op
    when ``enabled`` is false or fd 1 cannot be redirected safely (see the module
    docstring)."""
    active = bool(enabled) and _fflush is not None and _enter()
    try:
        yield active
    finally:
        if active:
            _exit()


def solve(
    model: LinearModel,
    time_limit: float | None = None,
    mip_gap: float | None = None,
    params: dict | None = None,
    relax: bool = False,
) -> Solution:
    """Solve ``model`` with HiGHS.

    Args:
        model: the model.
        time_limit: seconds (``None``: no limit), counted from the call: the conversion to
            scipy's arrays is subtracted before HiGHS starts.
        mip_gap: relative MIP gap (HiGHS ``mip_rel_gap``; ``None``: HiGHS default).
        params: extra ``milp`` options (e.g. ``{"presolve": False, "node_limit": 100}``);
            ``time_limit`` and ``mip_gap`` override the same keys in ``params``.
        relax: solve the LP relaxation (all integrality dropped; binary bounds kept,
            intersected with ``[0, 1]``).
    """
    from scipy.optimize import Bounds, LinearConstraint, milp
    from scipy.sparse import csr_matrix

    _check_scipy()
    t0 = time.perf_counter()
    n = model.num_columns
    m = model.num_rows
    sign = -1.0 if model.sense == "max" else 1.0
    c = sign * np.asarray(model.obj, dtype=float)
    lb = np.asarray(model.lb, dtype=float)
    ub = np.asarray(model.ub, dtype=float)
    vt = model.vtypes
    integrality = np.zeros(n, dtype=np.uint8)
    for j, t in enumerate(vt):
        if t == "B":
            lb[j] = max(lb[j], 0.0)
            ub[j] = min(ub[j], 1.0)
        if t != "C" and not relax:
            integrality[j] = 1
            # round integer bounds inward, as builtin does (HiGHS presolve mishandles
            # fractional integer bounds); ceil/floor keep +-inf as they are
            lb[j] = math.ceil(lb[j] - INT_EPS) if math.isfinite(lb[j]) else lb[j]
            ub[j] = math.floor(ub[j] + INT_EPS) if math.isfinite(ub[j]) else ub[j]
    constraints = []
    if m > 0:
        indptr, indices, data = model.csr()
        A = csr_matrix((np.asarray(data, dtype=float), np.asarray(indices, dtype=np.int64),
                        np.asarray(indptr, dtype=np.int64)), shape=(m, n))
        A.sum_duplicates()
        rhs = np.empty(m)
        lo = np.empty(m)
        hi = np.empty(m)
        for i in range(m):
            r = model.row_rhs(i)
            s = model.row_sense(i)
            rhs[i] = r
            lo[i] = -INF if s == "<=" else r
            hi[i] = INF if s == ">=" else r
        constraints.append(LinearConstraint(A, lo, hi))
    options = {"disp": False}
    if params:
        options.update(params)
    # the arguments win over params, as in the Gurobi and builtin backends
    if time_limit is not None:
        options["time_limit"] = float(time_limit)
    if mip_gap is not None:
        options["mip_rel_gap"] = float(mip_gap)
    if n == 0:
        status = "infeasible"
        for i in range(m):
            r, s = model.row_rhs(i), model.row_sense(i)
            if (s == "<=" and r < 0) or (s == ">=" and r > 0) or (s == "==" and r != 0):
                break
        else:
            status = "optimal"
        if status == "optimal":
            v = model.obj_constant
            return Solution(status="optimal", objective=v, x=[], mip_gap=0.0, bound=v,
                            sol_count=1, backend="highs", raw_status="empty model",
                            runtime=time.perf_counter() - t0)
        return Solution(status="infeasible", backend="highs", raw_status="empty model",
                        runtime=time.perf_counter() - t0)
    isolate = options.pop("isolate", None)
    if isolate is None:
        isolate = ISOLATE
    isolate = bool(isolate)
    quiet = FILTER_NATIVE_STDOUT and not options.get("disp")

    def call(opts):
        if isolate:
            cons = None if m == 0 else (A, lo, hi)
            return _milp_isolated(c, integrality, lb, ub, cons, opts)
        with native_stdout_filtered(quiet):
            return milp(c, integrality=integrality, bounds=Bounds(lb, ub), constraints=constraints,
                        options=opts)

    if time_limit is not None:
        # the conversion above (seconds for a model with millions of columns) counts
        # against the limit
        options["time_limit"] = max(0.0, float(time_limit) - (time.perf_counter() - t0))
    res = call(options)
    extra: dict = {}
    code = _code_of(res)
    general_int = not relax and any(integrality[j] and not (lb[j] >= 0.0 and ub[j] <= 1.0) for j in range(n))
    retry = code not in _DEFINITE and res.x is None
    if isolate and code in (8, 9) and general_int:
        retry = True  # confirm "infeasible" (scipy 1.17.1 presolve can report it wrongly)
    if retry and options.get("presolve", True) is not False:
        # HiGHS presolve error (or, isolated, a crash) without a solution: try once more
        # without presolve
        extra["highs_retry"] = {"presolve": False, "first_status": code,
                                "first_message": str(getattr(res, "message", ""))}
        options = dict(options, presolve=False)
        if time_limit is not None:
            options["time_limit"] = max(0.0, float(time_limit) - (time.perf_counter() - t0))
        res = call(options)
        code = _code_of(res)
    runtime = time.perf_counter() - t0
    msg = str(getattr(res, "message", ""))
    x = None if res.x is None else [float(v) for v in res.x]
    objective = None
    if x is not None:
        objective = model.obj_constant + sign * float(res.fun)
    dual = getattr(res, "mip_dual_bound", None)
    if dual is not None and np.isfinite(dual):
        bound = model.obj_constant + sign * float(dual)
    elif dual is not None:
        bound = sign * float(dual)
    else:
        bound = objective
    if code == 7:
        status = "optimal"
        if bound is None:
            bound = objective
    elif code in (8, 9):
        status = "infeasible"
    elif code == 10:
        status = "unbounded"
    elif code in (13, 14):
        status = "time_limit"
    elif code == 16:
        status = "node_limit"
    elif x is not None:
        status = "feasible"
    else:
        retry = " (also with presolve off)" if "highs_retry" in extra else ""
        raise RuntimeError(f"HiGHS failed without a solution{retry}: {msg}")
    gap = None
    if objective is not None and bound is not None:
        gap = relative_gap(objective, bound)
    node_count = getattr(res, "mip_node_count", None)
    return Solution(status=status, objective=objective, x=x,
                    mip_gap=gap, bound=bound, sol_count=0 if x is None else 1,
                    backend="highs", raw_status=msg, runtime=runtime,
                    node_count=None if node_count is None else int(node_count),
                    extra={"highs_model_status": code, **extra})


ISOLATE = os.environ.get("MPPFLOW_HIGHS_ISOLATE", "") not in ("", "0")
"""Run every ``milp`` call in a worker process (default ``False``; also
``params={"isolate": True}``), see the module docstring."""


class _Res:
    """The fields of a ``milp`` result that :func:`solve` reads."""

    def __init__(self, status, message, x, fun, mip_dual_bound=None, mip_node_count=None):
        self.status = status
        self.message = message
        self.x = x
        self.fun = fun
        self.mip_dual_bound = mip_dual_bound
        self.mip_node_count = mip_node_count


_worker: dict = {"proc": None, "atexit": False}
_worker_lock = threading.Lock()


def _send(f, obj) -> None:
    import pickle

    data = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
    f.write(len(data).to_bytes(8, "little") + data)
    f.flush()


def _recv(f):
    import pickle

    head = f.read(8)
    if len(head) < 8:
        raise EOFError
    n = int.from_bytes(head, "little")
    data = f.read(n)
    if len(data) < n:
        raise EOFError
    return pickle.loads(data)


def _serve() -> None:  # pragma: no cover - runs in the worker process
    """The isolated worker: read requests from stdin, answer on the original stdout, and
    send native output (fd 1) to /dev/null."""
    from scipy.optimize import Bounds, LinearConstraint, milp

    out = os.fdopen(os.dup(1), "wb")
    devnull = os.open(os.devnull, os.O_WRONLY)
    os.dup2(devnull, 1)
    os.close(devnull)
    inp = sys.stdin.buffer
    while True:
        try:
            c, integrality, lb, ub, cons, options = _recv(inp)
        except EOFError:
            return
        if options.pop("_mppflow_test_crash", False):
            import signal

            os.kill(os.getpid(), signal.SIGKILL)
        try:
            constraints = [] if cons is None else [LinearConstraint(*cons)]
            r = milp(c, integrality=integrality, bounds=Bounds(lb, ub), constraints=constraints,
                     options=options)
            d = {"status": int(r.status), "message": str(getattr(r, "message", "")),
                 "x": None if r.x is None else [float(v) for v in r.x],
                 "fun": None if r.fun is None else float(r.fun),
                 "mip_dual_bound": getattr(r, "mip_dual_bound", None),
                 "mip_node_count": getattr(r, "mip_node_count", None)}
        except Exception as e:  # noqa: BLE001 - reported to the parent
            d = {"error": f"{type(e).__name__}: {e}"}
        _send(out, d)


def _close_pipes(p) -> None:
    """Close a worker's pipes (no ``ResourceWarning`` for an unclosed file later)."""
    for f in (p.stdin, p.stdout):
        try:
            f.close()
        except Exception:  # noqa: BLE001 - a broken pipe may fail to flush
            pass


def _stop_worker() -> None:
    with _worker_lock:
        p = _worker["proc"]
        _worker["proc"] = None
    if p is not None:
        try:
            p.stdin.close()
            p.wait(timeout=5)
        except Exception:  # noqa: BLE001
            p.kill()
            p.wait()
        finally:
            _close_pipes(p)


def _discard_worker(p) -> None:
    """Kill worker ``p``, close its pipes and forget it (the caller holds the lock)."""
    p.kill()
    p.wait()
    _close_pipes(p)
    _worker["proc"] = None


def _milp_isolated(c, integrality, lb, ub, cons, options):
    """``milp`` in a separate worker process (a fresh interpreter, ``python -c``, reused
    between solves). A worker that dies (the scipy 1.17.1 presolve segfault) comes back as
    a result with code -2 and no solution; the next call starts a new worker. A call that is
    interrupted (``KeyboardInterrupt``, or any exception raised while it waits) kills its
    worker before the exception propagates, so no later call reads the stale answer."""
    import subprocess

    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    with _worker_lock:
        p = _worker["proc"]
        if p is None or p.poll() is not None:
            if p is not None:
                _close_pipes(p)
            code = f"import sys; sys.path.insert(0, {root!r}); from mppflow.lp.highs import _serve; _serve()"
            p = subprocess.Popen([sys.executable, "-c", code], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                 stderr=subprocess.DEVNULL)
            _worker["proc"] = p
            if not _worker["atexit"]:
                import atexit

                atexit.register(_stop_worker)
                _worker["atexit"] = True
        try:
            _send(p.stdin, (c, integrality, lb, ub, cons, dict(options)))
            d = _recv(p.stdout)
        except (EOFError, BrokenPipeError, OSError):
            _discard_worker(p)
            how = f"signal {-p.returncode}" if p.returncode and p.returncode < 0 else f"exit code {p.returncode}"
            return _Res(-2, f"HiGHS worker process died ({how})", None, None)
        except BaseException:
            # interrupted while the request is out (KeyboardInterrupt, an exception from a
            # signal handler such as SIGALRM): the worker would still answer it, and the next
            # call would read that answer as its own. A new worker serves the next call.
            _discard_worker(p)
            raise
    if "error" in d:
        raise RuntimeError(f"HiGHS worker: {d['error']}")
    x = None if d["x"] is None else np.asarray(d["x"], dtype=float)
    return _Res(d["status"], d["message"], x, d["fun"], d["mip_dual_bound"], d["mip_node_count"])


def _code_of(res) -> int:
    """The HiGHS model status of a ``milp`` result (from scipy's status if the message
    carries no HiGHS code; -1 if unknown)."""
    code = _highs_model_status(str(getattr(res, "message", "")))
    if code is None:
        code = {0: 7, 1: 13, 2: 8, 3: 10, -2: -2}.get(int(res.status), -1)
    return code


_CODE_RE = re.compile(r"HiGHS Status (\d+)")


def _highs_model_status(message: str) -> int | None:
    """The HiGHS model status code embedded in a scipy ``milp`` message, if any."""
    mt = _CODE_RE.search(message)
    return None if mt is None else int(mt.group(1))
