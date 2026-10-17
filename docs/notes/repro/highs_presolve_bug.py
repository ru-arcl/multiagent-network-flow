"""HiGHS MIP presolve defects seen with scipy 1.17.1 (bundled HiGHS), on small general-integer
models from a random stress generator (not MPP models).

Each solve runs in a subprocess because one of them crashes the interpreter: first raw
scipy.optimize.milp with presolve on and off, then mppflow's HiGHS backend, which rounds the
bounds of integer columns inward and retries once with presolve off after an error status,
and the same with params={"isolate": True} (each milp call in a worker process, so a crash
becomes an error status and is retried with presolve off).
Known answers (builtin solver and Gurobi): m293 optimal 0.1, m390 optimal -0.75,
m968 infeasible, m320 optimal 1.0.
"""
import subprocess
import sys

MODELS = {
    # name: (sense, [(lb, ub, vtype, obj)], [(terms, sense, rhs)])
    "m293": ("min", [(0, 1, "B", 0.0), (0, 0, "I", 1.0), (0, 1, "B", 3.0), (0.5, 1.2, "I", 3.0),
                     (0.5, 0.5, "C", 1.0), (1.0, 1.7, "C", -2.0)],
             [([(2, -1.0), (1, -1.0), (3, 1.0), (4, 3.0)], ">=", 2.5)]),
    "m390": ("max", [(0, 3, "I", 0.25), (0, 1, "B", -2.0), (0, 0, "C", 3.0), (1, 2, "C", -2.0),
                     (-1.5, -0.8, "I", -1.0)],
             [([(0, 2.0), (2, 0.5), (3, 3.0), (4, 1.0), (4, 2.0), (4, 1.0)], "<=", 1.0),
              ([(0, -3.0), (0, -1.0), (1, 3.0), (0, 0.5)], "<=", 0.0),
              ([(3, -1.0), (1, -2.0), (2, 2.0), (1, 2.0), (4, 2.0)], "<=", 7.0),
              ([(2, -2.0), (3, -3.0), (0, -2.0)], "<=", -2.0),
              ([(4, 2.0), (1, -2.0)], "<=", -2.0)]),
    "m968": ("min", [(-2, -1, "I", 0.0), (0, 1, "B", 0.25), (0, 1, "B", 3.0)],
             [([(0, 0.5), (1, -3), (2, 2), (0, -2)], "==", 2.5), ([(0, 3)], "<=", 2.5)]),
    "m320": ("min", [(0, float("inf"), "C", -4.0), (0, 1, "B", 0.0), (-3, -1, "I", 2.0),
                     (0, float("inf"), "C", 2.0), (0, float("inf"), "C", 1.0), (0, 1, "B", 5.0)],
             [([(0, 0.0), (1, -1.0), (4, 2.0), (5, 0.0), (2, -2.0), (3, 1.0)], ">=", 3.0),
              ([(4, -1.0), (1, 3.0), (2, -1.0)], "==", 2.0),
              ([(3, 2.0), (1, 1.0)], ">=", -2.5),
              ([(4, 1.0), (1, 1.0), (2, -1.0), (0, -1.0)], ">=", 1.0),
              ([(3, 2.0), (2, 1.0), (5, 2.0), (4, 1.0), (1, -2.0)], ">=", -2.5),
              ([(0, 1.0), (3, 0.0), (1, -2.0), (5, 2.0), (4, 3.0), (2, 2.0)], "<=", 0.5),
              ([(0, 1.0), (4, 1.0), (3, 1.0), (1, -1.0)], "<=", 1.0),
              ([(0, 3.0), (3, -2.0)], "<=", 0.0),
              ([(2, 2.0), (3, 1.0), (1, 1.0), (5, 3.0), (0, 2.0)], "==", -1.0),
              ([(2, -2.0), (1, 1.0)], "<=", 5.0),
              ([(1, 2.0), (0, 1.0), (2, 0.0), (3, 1.0)], ">=", -1.0),
              ([(1, 1.0), (4, 3.0)], "==", 0.0),
              ([(5, 3.0), (4, 3.0), (1, -1.0), (2, -2.0), (0, 3.0)], "<=", 7.0),
              ([(0, 1.0), (4, 0.0), (1, 3.0), (5, -1.0), (3, 0.0), (2, 2.0)], "<=", -4.0),
              ([(3, -1.0), (2, 1.0), (4, -2.0), (1, 0.0), (5, -1.0)], "<=", -0.5),
              ([(2, 1.0), (0, 1.0), (1, -1.0), (5, 1.0), (4, -2.0), (3, -1.0)], ">=", -2.0)]),
}


def raw(name, presolve):
    import numpy as np
    from scipy.optimize import Bounds, LinearConstraint, milp
    from scipy.sparse import csr_matrix

    sense, cols, rows = MODELS[name]
    A = np.zeros((len(rows), len(cols)))
    for i, (terms, _, _) in enumerate(rows):
        for j, a in terms:
            A[i, j] += a
    lo = [-np.inf if s == "<=" else r for _, s, r in rows]
    hi = [np.inf if s == ">=" else r for _, s, r in rows]
    c = np.array([o for *_, o in cols]) * (-1 if sense == "max" else 1)
    res = milp(c, integrality=[0 if v == "C" else 1 for _, _, v, _ in cols],
               bounds=Bounds([l for l, *_ in cols], [u for _, u, *_ in cols]),
               constraints=[LinearConstraint(csr_matrix(A), lo, hi)],
               options={"disp": False, "presolve": presolve, "mip_rel_gap": 0.0})
    fun = None if res.fun is None else (-res.fun if sense == "max" else res.fun)
    return f"objective {fun}, {res.message}"


def with_mppflow(name, isolate=False):
    import _common  # noqa: F401
    from mppflow.lp import LinearModel, solve

    sense, cols, rows = MODELS[name]
    m = LinearModel(sense)
    for lb, ub, vt, obj in cols:
        m.add_column(lb, ub, vt, obj=obj)
    for terms, s, r in rows:
        m.add_row(terms, s, r)
    sol = solve(m, backend="highs", mip_gap=0.0, params={"isolate": True} if isolate else None)
    retry = sol.extra.get("highs_retry")
    return f"{sol.status} {sol.objective}" + (
        f" (first attempt: HiGHS status {retry['first_status']}; retried with presolve off)" if retry else "")


if len(sys.argv) == 3:
    name, how = sys.argv[1:]
    if how in ("mppflow", "isolate"):
        print("RESULT " + with_mppflow(name, how == "isolate"))
    else:
        print("RESULT " + raw(name, how == "on"))
    sys.exit(0)

import scipy  # noqa: E402

print("scipy", scipy.__version__)
for name in MODELS:
    for how, label in (("on", "raw milp, presolve on "), ("off", "raw milp, presolve off"), ("mppflow", "mppflow highs         "),
                       ("isolate", "mppflow highs isolate ")):
        p = subprocess.run([sys.executable, __file__, name, how], capture_output=True, text=True)
        msg = " ".join(ln[7:] for ln in p.stdout.splitlines() if ln.startswith("RESULT "))
        print(f"{name} {label}: " + (msg if p.returncode == 0 else f"process died (exit code {p.returncode})"))
