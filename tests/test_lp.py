"""Tests for mppflow.lp: LinearModel, and the builtin / HiGHS / Gurobi backends."""

from __future__ import annotations

import contextlib
import ctypes
import itertools
import os
import warnings
import random
import re
import signal
import sys

import pytest

from mppflow import lp
from mppflow.lp import INF, LinearModel, Solution, builtin, highs, relative_gap

from helpers import needs_gurobi  # noqa: E402,F401  (gurobipy and a full-size license)

GRB_PARAMS = {"Threads": 1}


# --------------------------------------------------------------------------- helpers
def random_model(rng: random.Random, integer: bool, allow_inf: bool = False) -> LinearModel:
    n = rng.randint(1, 7)
    m = rng.randint(0, 6)
    model = LinearModel(rng.choice(["max", "min"]))
    for _ in range(n):
        lo = rng.choice([0.0, 0.0, -2.0, 1.0, 0.5])
        if allow_inf and rng.random() < 0.3:
            hi = INF
        else:
            hi = lo + rng.choice([0, 1, 2, 3, 5, 2.5])
        vt = "C"
        if integer and rng.random() < 0.7:
            vt = rng.choice(["I", "B"]) if lo == 0.0 and hi <= 1.0 else rng.choice(["I", "I", "I", "B"])
            if vt == "B" and rng.random() < 0.5:
                lo, hi = 0.0, 1.0  # else bounds outside [0, 1]: every backend intersects them with [0, 1]
        model.add_column(lo, hi, vt)
    for _ in range(m):
        k = rng.randint(1, n)
        cols = rng.sample(range(n), k)
        terms = [(c, float(rng.randint(-5, 5)) or 1.0) for c in cols]
        if rng.random() < 0.2:
            terms = [(c, v + rng.choice([0.25, -0.5])) for c, v in terms]
        model.add_row(terms, rng.choice(["<=", ">=", "=="]), float(rng.randint(-6, 8)))
    model.set_objective([(j, float(rng.randint(-4, 4))) for j in range(n)],
                        constant=rng.choice([0.0, 1.5]))
    return model


def check_feasible(model: LinearModel, x, tol=1e-6):
    for j in range(model.num_columns):
        lo, hi, vt, _, _ = model.column(j)
        assert x[j] >= lo - tol and x[j] <= hi + tol
        if vt != "C":
            assert abs(x[j] - round(x[j])) <= tol
    for terms, sense, rhs in model.rows():
        v = 0.0
        for c, a in terms:
            v += a * x[c]
        if sense == "<=":
            assert v <= rhs + tol
        elif sense == ">=":
            assert v >= rhs - tol
        else:
            assert abs(v - rhs) <= tol


def assert_agree(model: LinearModel, a: Solution, b: Solution, tol=1e-6):
    assert a.status == b.status, (a.status, b.status, model.to_canonical_text())
    if a.status == "optimal":
        assert abs(a.objective - b.objective) <= tol * (1 + abs(b.objective)), \
            (a.objective, b.objective, model.to_canonical_text())
        check_feasible(model, a.x)
        check_feasible(model, b.x)
        assert abs(model.objective_value(a.x) - a.objective) <= 1e-9 * (1 + abs(a.objective))


# ----------------------------------------------------------------------- LinearModel
def test_model_order_and_accessors():
    m = LinearModel("min", name="t")
    assert m.add_column(0, 1, "B", name="a") == 0
    assert m.add_column(-1, INF, "C") == 1
    assert m.add_var(0, 4, "I", 2.0) == 2
    assert m.num_columns == 3
    assert m.column(0) == (0.0, 1.0, "B", 0.0, "a")
    assert m.column(1)[1] == INF and m.column_name(1) is None
    assert m.add_row([(2, 1.0), (0, -1.0), (2, 3.0)], "<", 4) == 0
    assert m.add_row([], "=", 0) == 1
    m.add_constr([(1, 2.5)], "G", -1.0, name="r2")
    assert m.row(0) == [(2, 1.0), (0, -1.0), (2, 3.0)]  # insertion order, duplicates kept
    assert [m.row_sense(i) for i in range(3)] == ["<=", "==", ">="]
    assert m.row_rhs(2) == -1.0 and m.row_name(2) == "r2" and m.num_terms == 4
    assert m.csr() == ([0, 3, 3, 4], [2, 0, 2, 1], [1.0, -1.0, 3.0, 2.5])
    assert m.is_mip
    with pytest.raises(ValueError):
        m.add_row([(0, 1.0)], "!=", 0)
    with pytest.raises(IndexError):
        m.add_row([(5, 1.0)], "<=", 0)
    with pytest.raises(ValueError):
        m.add_column(0, 1, "X")
    with pytest.raises(ValueError):
        LinearModel("up")


def test_set_objective_accumulates_and_resets():
    m = LinearModel()
    for _ in range(3):
        m.add_column(0, 1, "C", obj=5.0)
    m.set_objective([(2, 1.0), (0, 2.0), (2, 0.5)], sense="min", constant=1.0)
    assert m.obj == [2.0, 0.0, 1.5]
    assert m.sense == "min" and m.obj_constant == 1.0
    assert m.objective_terms == [(2, 1.0), (0, 2.0), (2, 0.5)]
    assert m.objective_value([1.0, 1.0, 2.0]) == 1.0 + 2.0 + 3.0


def test_canonical_text_and_hash_stable():
    def build(names=True):
        m = LinearModel("max")
        m.add_column(0, 1, "B", name="x" if names else None)
        m.add_column(0, INF, "C")
        m.add_row([(1, 1.0), (0, -1.0)], "<=", 0.0, name="r" if names else None)
        m.set_objective([(0, 1.0)])
        return m

    a, b = build(), build()
    # the golden canonical dump (tools/golden/README.md)
    text = a.to_canonical_text()
    assert text == b.to_canonical_text() and a.sha256() == b.sha256()
    assert text.splitlines() == [
        "SENSE max", "NCOLS 2", "NROWS 1",
        "C0 0 1 B 1 x", "C1 0 inf C 0 C1",
        "R0 <= 0 : (0,-1) (1,1)",
    ]
    assert a.to_canonical_text(False).splitlines()[3:5] == ["C0 0 1 B 1", "C1 0 inf C 0"]
    assert a.to_canonical_text(["p", "q"]).splitlines()[3:5] == ["C0 0 1 B 1 p", "C1 0 inf C 0 q"]
    assert a.sha256(names=False) == build(names=False).sha256(names=False)
    assert a.sha256() != build(names=False).sha256()
    import hashlib

    assert a.sha256() == hashlib.sha256(text.encode()).hexdigest()
    # the raw construction-order listing
    raw = a.to_raw_text()
    assert raw == b.to_raw_text()
    assert raw.splitlines() == [
        "LINEARMODEL 1", "SENSE max", "OBJCONST 0.0", "COLUMNS 2",
        "V 0 0.0 1.0 B 1.0 x", "V 1 0.0 Infinity C 0.0",
        "ROWS 1", "R 0 <= 0.0 : 1.0*1 -1.0*0 ; r",
    ]
    assert a.to_raw_text(names=False) == build(names=False).to_raw_text()


def test_canonical_text_objective_constant_and_merged_rows():
    m = LinearModel("min")
    m.add_column(0, 3, "I")
    m.add_column(-2.5, 2.5, "C")
    m.add_row([(1, 1.0), (0, 2.0), (1, -1.0), (0, 0.5)], "==", 1.5)
    m.set_objective([(0, 1.0)], constant=-4.0)
    assert m.merged_row(0) == [(0, 2.5)]
    assert m.to_canonical_text().splitlines() == [
        "SENSE min", "OBJCONST -4", "NCOLS 2", "NROWS 1", "C0 0 3 I 1 C0", "C1 -2.5 2.5 C 0 C1",
        "R0 = 1.5 : (0,2.5)",
    ]
    assert "1.0*1 2.0*0 -1.0*1 0.5*0" in m.to_raw_text()


def test_relax_and_copy_are_independent():
    m = LinearModel()
    m.add_column(-1, 3, "B")
    m.add_column(0, 7, "I")
    m.add_column(0, 2, "C")
    m.add_row([(0, 1.0)], "<=", 1)
    r = m.relax()
    assert r.vtypes == ["C", "C", "C"] and not r.is_mip
    assert r.lb == [0.0, 0.0, 0.0] and r.ub == [1.0, 7.0, 2.0]
    assert m.vtypes == ["B", "I", "C"] and m.lb[0] == -1.0
    c = m.copy()
    c.add_row([(1, 1.0)], ">=", 0)
    c.set_bounds(2, ub=5)
    assert m.num_rows == 1 and m.ub[2] == 2.0
    assert c.to_canonical_text() != m.to_canonical_text()


def test_add_column_obj_after_set_objective_extends_terms():
    """Regression: the Gurobi backend reads objective_terms, which add_column(obj=...) after
    set_objective did not update."""
    m = LinearModel("max")
    a = m.add_column(0, 1)
    m.set_objective([(a, 1.0)])
    b = m.add_column(0, 1, obj=5.0)
    m.add_column(0, 1)
    assert m.objective_terms == [(a, 1.0), (b, 5.0)]
    assert m.obj == [1.0, 5.0, 0.0]
    for backend in ("builtin", "highs"):
        s = lp.solve(m, backend=backend)
        assert s.objective == 6.0 and s.x[:2] == [1.0, 1.0]
    from mppflow.lp import gurobi as gmod

    assert gmod._objective_terms(m) == [(a, 1.0), (b, 5.0)]
    m.objective_terms[0] = (a, 2.0)  # out of sync with obj: column order is used
    assert gmod._objective_terms(m) == [(0, 1.0), (1, 5.0)]
    m2 = LinearModel()
    m2.add_column(0, 1, obj=2.0)
    m2.add_column(0, 1, obj=0.0)
    assert m2.objective_terms is None and gmod._objective_terms(m2) == [(0, 2.0)]
    m3 = LinearModel()
    c = m3.add_column(0, 1)
    m3.set_objective([(c, 1.0), (c, 0.5), (c, -1.5)])  # repeated terms keep their order
    assert gmod._objective_terms(m3) == [(c, 1.0), (c, 0.5), (c, -1.5)]


def test_relative_gap():
    assert relative_gap(10.0, 11.0) == pytest.approx(0.1)
    assert relative_gap(0.0, 0.0) == 0.0
    assert relative_gap(0.0, 1.0) == INF


# --------------------------------------------------------------------------- builtin
def test_builtin_random_lps_vs_highs():
    rng = random.Random(12345)
    counts = {}
    for _ in range(400):
        model = random_model(rng, integer=False, allow_inf=True)
        a = builtin.solve(model)
        b = highs.solve(model)
        assert_agree(model, a, b)
        counts[a.status] = counts.get(a.status, 0) + 1
    # the generator exercises all three outcomes
    assert counts.get("optimal", 0) > 50
    assert counts.get("infeasible", 0) > 20
    assert counts.get("unbounded", 0) > 5


def test_builtin_random_milps_vs_highs():
    rng = random.Random(777)
    n_opt = 0
    for _ in range(300):
        model = random_model(rng, integer=True)
        a = builtin.solve(model)
        b = highs.solve(model, mip_gap=0.0)
        assert_agree(model, a, b)
        if a.status == "optimal":
            n_opt += 1
            assert a.bound == a.objective and a.mip_gap == 0.0
            for j, t in enumerate(model.vtypes):
                if t != "C":
                    assert a.x[j] == float(int(a.x[j]))  # snapped to exact integers
    assert n_opt > 50


def test_builtin_bounded_integer_brute_force():
    """Pure-integer models checked against exhaustive enumeration."""
    rng = random.Random(4242)
    for _ in range(60):
        n = rng.randint(1, 4)
        model = LinearModel(rng.choice(["max", "min"]))
        for _ in range(n):
            model.add_column(rng.choice([0, -1]), rng.choice([1, 2, 3]), "I")
        for _ in range(rng.randint(1, 4)):
            model.add_row([(j, float(rng.randint(-3, 3))) for j in range(n)],
                          rng.choice(["<=", ">="]), float(rng.randint(-3, 4)))
        model.set_objective([(j, float(rng.randint(-3, 3))) for j in range(n)])
        best = None
        ranges = [range(int(model.lb[j]), int(model.ub[j]) + 1) for j in range(n)]
        for pt in itertools.product(*ranges):
            x = [float(v) for v in pt]
            ok = True
            for terms, sense, rhs in model.rows():
                v = sum(a * x[c] for c, a in terms)
                if (sense == "<=" and v > rhs) or (sense == ">=" and v < rhs):
                    ok = False
                    break
            if ok:
                o = model.objective_value(x)
                if best is None or (o > best if model.sense == "max" else o < best):
                    best = o
        s = builtin.solve(model)
        if best is None:
            assert s.status == "infeasible"
        else:
            assert s.status == "optimal" and s.objective == best


def test_builtin_iteration_cap_is_a_limit(monkeypatch):
    """C2D-BUILTIN-ITERATION-CAP-SMALL-MODELS: a simplex phase that reaches the iteration cap
    (Bland's rule stalling on degenerate vertices; a 638-column max-distance node LP needed more
    than 144797 pivots, all bases distinct) used to raise RuntimeError; the solve now ends as at a
    work limit, and the solvers report the horizon as undecided."""
    from mppflow import solvers as S
    from mppflow.graph import create_2d_grid_graph

    real = builtin._simplex

    def capped(*a):
        a = list(a)
        a[9] = 1
        return real(*a)
    monkeypatch.setattr(builtin, "_simplex", capped)
    m = LinearModel("min")
    x = [m.add_column(0, INF) for _ in range(4)]
    m.add_row([(x[0], 0.25), (x[1], -8.0), (x[2], -1.0), (x[3], 9.0)], "<=", 0.0)
    m.add_row([(x[0], 0.5), (x[1], -12.0), (x[2], -0.5), (x[3], 3.0)], "<=", 0.0)
    m.add_row([(x[2], 1.0)], "<=", 1.0)
    m.set_objective([(x[0], -0.75), (x[1], 20.0), (x[2], -0.5), (x[3], 6.0)])
    s = builtin.solve(m)
    assert s.status == "time_limit" and s.raw_status == "iteration_cap" and s.x is None
    r = S.min_makespan(create_2d_grid_graph(3, 3), [0, 1], [8, 7], backend="builtin")
    assert r.paths is None and r.status == "time_limit"
    assert r.attempts[-1]["raw_status"] == "iteration_cap"

def test_builtin_beale_cycling_example():
    """Beale's classic example cycles under the textbook rule; Bland's rule terminates."""
    m = LinearModel("min")
    x = [m.add_column(0, INF) for _ in range(4)]
    m.add_row([(x[0], 0.25), (x[1], -8.0), (x[2], -1.0), (x[3], 9.0)], "<=", 0.0)
    m.add_row([(x[0], 0.5), (x[1], -12.0), (x[2], -0.5), (x[3], 3.0)], "<=", 0.0)
    m.add_row([(x[2], 1.0)], "<=", 1.0)
    m.set_objective([(x[0], -0.75), (x[1], 20.0), (x[2], -0.5), (x[3], 6.0)])
    s = builtin.solve(m)
    assert s.status == "optimal"
    assert s.objective == pytest.approx(-1.25)
    assert s.objective == pytest.approx(highs.solve(m).objective)


def test_builtin_redundant_equalities_and_shifted_bounds():
    m = LinearModel("min")
    a = m.add_column(2, 5)
    b = m.add_column(-3, 4)
    c = m.add_column(1, 1)  # fixed
    m.add_row([(a, 1.0), (b, 1.0)], "==", 3)
    m.add_row([(a, 2.0), (b, 2.0)], "==", 6)  # redundant copy
    m.add_row([(a, 1.0), (b, 1.0), (c, 0.0)], "==", 3)  # redundant again
    m.add_row([(a, -1.0), (c, 1.0)], ">=", -4)
    m.set_objective([(a, 1.0), (b, 2.0), (c, 1.0)])
    s = builtin.solve(m)
    assert s.status == "optimal"
    assert s.x == pytest.approx([5.0, -2.0, 1.0])
    assert s.objective == pytest.approx(2.0)


def test_builtin_empty_and_trivial_models():
    m = LinearModel("max")
    s = builtin.solve(m)
    assert s.status == "optimal" and s.x == [] and s.objective == 0.0
    m.add_row([], "<=", -1.0)
    assert builtin.solve(m).status == "infeasible"
    m2 = LinearModel("max")
    m2.add_column(0, INF)
    m2.set_objective([(0, 1.0)])
    assert builtin.solve(m2).status == "unbounded"
    assert highs.solve(m2).status == "unbounded"
    m3 = LinearModel("min")
    m3.add_column(3, 1)  # lb > ub
    assert builtin.solve(m3).status == "infeasible"
    m4 = LinearModel()
    m4.add_column(-INF, 1)
    with pytest.raises(ValueError):
        builtin.solve(m4)


def test_builtin_branching_order_and_limits():
    # max x + y, 2x + 2y <= 3, x, y binary. Bland's rule gives the LP vertex x = 1, y = 0.5,
    # so y is branched on; y >= 1 first (x = 0.5), then x >= 1 (infeasible), x <= 0.
    m = LinearModel("max")
    m.add_column(0, 1, "B")
    m.add_column(0, 1, "B")
    m.add_row([(0, 2.0), (1, 2.0)], "<=", 3.0)
    m.set_objective([(0, 1.0), (1, 1.0)])
    s = builtin.solve(m)
    assert s.status == "optimal" and s.x == [0.0, 1.0] and s.objective == 1.0
    assert s.node_count == 5
    s1 = builtin.solve(m, node_limit=1)
    assert s1.status == "node_limit" and s1.x is None and s1.bound == pytest.approx(1.5)
    s2 = builtin.solve(m, params={"node_limit": 4})
    assert s2.status == "node_limit" and s2.x == [0.0, 1.0] and s2.sol_count == 1
    assert s2.mip_gap == pytest.approx(0.5)
    s3 = builtin.solve(m, work_limit=0)
    assert s3.status == "time_limit" and s3.raw_status == "work_limit" and s3.x is None


def test_builtin_mip_gap_prunes():
    rng = random.Random(99)
    for _ in range(40):
        model = random_model(rng, integer=True)
        exact = builtin.solve(model)
        loose = builtin.solve(model, mip_gap=0.5)
        assert loose.status == exact.status
        if exact.status == "optimal":
            assert abs(loose.objective - exact.objective) <= 0.5 * abs(loose.objective) + 1e-9
            assert loose.node_count <= exact.node_count


def test_builtin_mip_gap_bound_is_valid():
    """Regression: with mip_gap > 0 the reported bound used to be the incumbent objective
    (with mip_gap 0.0) although better solutions had been pruned."""
    lowered = 0
    for seed in range(20):
        r = random.Random(seed)
        n = 12
        w = [r.randint(5, 40) for _ in range(n)]
        v = [r.randint(5, 40) for _ in range(n)]
        for sense in ("max", "min"):
            k = LinearModel(sense)
            for _ in range(n):
                k.add_column(0, 1, "B")
            if sense == "max":
                k.add_row([(i, float(w[i])) for i in range(n)], "<=", float(sum(w) // 2))
                k.set_objective([(i, float(v[i])) for i in range(n)], constant=3.0)
            else:
                k.add_row([(i, float(w[i])) for i in range(n)], ">=", float(sum(w) // 2))
                k.set_objective([(i, float(v[i])) for i in range(n)], constant=3.0)
            exact = builtin.solve(k)
            assert exact.status == "optimal"
            assert exact.bound == exact.objective and exact.mip_gap == 0.0
            loose = builtin.solve(k, mip_gap=0.05)
            assert loose.status == "optimal"
            if sense == "max":
                assert loose.bound >= exact.objective - 1e-9
            else:
                assert loose.bound <= exact.objective + 1e-9
            assert loose.mip_gap == relative_gap(loose.objective, loose.bound)
            assert loose.mip_gap <= 0.05 + 1e-12
            if loose.objective != exact.objective:
                lowered += 1
                assert loose.mip_gap > 0.0
    assert lowered >= 5
    # the documented case: seed 0, max
    r = random.Random(0)
    w = [r.randint(5, 40) for _ in range(12)]
    v = [r.randint(5, 40) for _ in range(12)]
    k = LinearModel("max")
    for _ in range(12):
        k.add_column(0, 1, "B")
    k.add_row([(i, float(w[i])) for i in range(12)], "<=", float(sum(w) // 2))
    k.set_objective([(i, float(v[i])) for i in range(12)])
    s = builtin.solve(k, mip_gap=0.05)
    assert s.objective == 157.0 and builtin.solve(k).objective == 160.0
    assert s.bound >= 160.0


def test_builtin_time_limit_stops_inside_one_lp():
    """Regression: time_limit used to be checked only between B&B nodes, so one large LP
    could run far past it."""
    r = random.Random(1)
    k = LinearModel("max")
    for _ in range(400):
        k.add_column(0, 10)
    for _ in range(400):
        k.add_row([(j, float(r.randint(1, 9))) for j in range(400)], "<=",
                  float(r.randint(100, 1000)))
    k.set_objective([(j, float(r.randint(1, 9))) for j in range(400)])
    # the full root LP takes about 2 s and 683 iterations here
    s = builtin.solve(k, time_limit=0.05)
    assert s.status == "time_limit" and s.raw_status == "time_limit"
    assert s.x is None and s.node_count == 1 and s.iterations < 683
    assert s.runtime < 1.0
    s = builtin.solve(k, work_limit=10)
    assert s.status == "time_limit" and s.raw_status == "work_limit" and s.iterations == 10


def test_builtin_deterministic():
    rng = random.Random(5)
    for _ in range(30):
        model = random_model(rng, integer=True)
        a, b = builtin.solve(model), builtin.solve(model)
        assert (a.status, a.x, a.objective, a.iterations, a.node_count) == \
            (b.status, b.x, b.objective, b.iterations, b.node_count)


# ------------------------------------------------------------------ flow-like models
def assignment_model(rng, n, integer=True):
    m = LinearModel("max")
    var = {}
    w = {}
    for i in range(n):
        for j in range(n):
            if rng.random() < 0.8 or i == j:
                var[i, j] = m.add_column(0, 1, "B" if integer else "C")
                w[i, j] = float(rng.randint(0, 9))
    for i in range(n):
        m.add_row([(var[i, j], 1.0) for j in range(n) if (i, j) in var], "<=", 1.0)
    for j in range(n):
        m.add_row([(var[i, j], 1.0) for i in range(n) if (i, j) in var], "<=", 1.0)
    m.set_objective([(var[k], w[k]) for k in var])
    return m, var, w


def test_assignment_vs_brute_force_and_highs():
    rng = random.Random(1)
    for n in (2, 3, 4, 5):
        for _ in range(4):
            m, var, w = assignment_model(rng, n)
            best = 0.0
            for perm in itertools.permutations(range(n)):
                for mask in range(1 << n):
                    if all(((i, perm[i]) in var) or not (mask >> i) & 1 for i in range(n)):
                        v = sum(w[i, perm[i]] for i in range(n) if (mask >> i) & 1)
                        best = max(best, v)
            s = builtin.solve(m)
            assert s.status == "optimal" and s.objective == best
            assert highs.solve(m).objective == pytest.approx(best)
            # bipartite matching LPs are integral
            r = builtin.solve(m.relax())
            assert r.objective == pytest.approx(best)


def shortest_path_model(rng, n):
    m = LinearModel("min")
    arcs = {}
    for u in range(n):
        for v in range(n):
            if u != v and rng.random() < 0.4:
                arcs[u, v] = (m.add_column(0, 1, "B"), float(rng.randint(1, 9)))
    for v in range(n):
        out = [(arcs[a][0], 1.0) for a in arcs if a[0] == v]
        inn = [(arcs[a][0], -1.0) for a in arcs if a[1] == v]
        rhs = 1.0 if v == 0 else (-1.0 if v == n - 1 else 0.0)
        m.add_row(inn + out, "==", rhs)
    m.set_objective([(c, w) for c, w in arcs.values()])
    return m, arcs


def dijkstra(n, arcs, s, t):
    dist = [INF] * n
    dist[s] = 0.0
    done = [False] * n
    for _ in range(n):
        u = min((d, i) for i, d in enumerate(dist) if not done[i])[1]
        done[u] = True
        for (a, b), (_, w) in arcs.items():
            if a == u and dist[u] + w < dist[b]:
                dist[b] = dist[u] + w
    return dist[t]


def test_shortest_path_flow():
    rng = random.Random(2)
    for _ in range(25):
        n = rng.randint(3, 8)
        m, arcs = shortest_path_model(rng, n)
        d = dijkstra(n, arcs, 0, n - 1)
        s = builtin.solve(m)
        h = highs.solve(m)
        if d == INF:
            assert s.status == "infeasible" and h.status == "infeasible"
        else:
            assert s.status == "optimal" and s.objective == d
            assert h.objective == pytest.approx(d)


def grid_graph(rows, cols):
    adj = {}
    for r in range(rows):
        for c in range(cols):
            v = r * cols + c
            nb = [v]
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < rows and 0 <= cc < cols:
                    nb.append(rr * cols + cc)
            adj[v] = sorted(nb)
    return adj


def mpp_model(adj, starts, goals, T, objective="makespan", integer=True):
    """A small time-expanded MPP flow model (paper Sec. IV), built independently here."""
    vt = "B" if integer else "C"
    m = LinearModel("max")
    x = {}
    for a in range(len(starts)):
        for t in range(T):
            for u in adj:
                for v in adj[u]:
                    x[a, u, v, t] = m.add_column(0, 1, vt)
        x[a, "loop"] = m.add_column(0, 1, vt)
    n = len(adj)
    for t in range(T):
        for v in range(n):
            terms = [(x[a, u, v, t], 1.0) for a in range(len(starts)) for u in adj if v in adj[u]]
            m.add_row(terms, "<=", 1.0)
    for a in range(len(starts)):
        for t in range(T + 1):
            for v in range(n):
                inn = [(x[a, u, v, t - 1], -1.0) for u in adj if t > 0 and v in adj[u]]
                out = [(x[a, v, w, t], 1.0) for w in adj[v] if t < T]
                if t == 0 and v == starts[a]:
                    inn.append((x[a, "loop"], -1.0))
                if t == T and v == goals[a]:
                    out.append((x[a, "loop"], 1.0))
                if t == 0 and v != starts[a]:
                    m.add_row(out, "==", 0.0)  # robots start at their starts
                    continue
                m.add_row(inn + out, "==", 0.0)
    for t in range(T):
        for u in adj:
            for v in adj[u]:
                if u < v:
                    m.add_row([(x[a, u, v, t], 1.0) for a in range(len(starts))]
                              + [(x[a, v, u, t], 1.0) for a in range(len(starts))], "<=", 1.0)
    if objective == "makespan":
        m.set_objective([(x[a, "loop"], 1.0) for a in range(len(starts))])
    else:  # total distance with all robots delivered
        for a in range(len(starts)):
            m.set_bounds(x[a, "loop"], lb=1.0)
        m.set_objective([(x[k], 1.0) for k in x if k[1] != "loop" and k[1] != k[2]], "min")
    return m


@pytest.mark.parametrize("T", [1, 2, 3, 4])
def test_mpp_flow_makespan_models(T):
    adj = grid_graph(2, 3)
    starts, goals = [0, 2, 3], [2, 0, 5]
    m = mpp_model(adj, starts, goals, T)
    s = builtin.solve(m)
    h = highs.solve(m, mip_gap=0.0)
    assert s.status == h.status == "optimal"
    assert s.objective == h.objective
    lpb = builtin.solve(m.relax())
    assert lpb.objective == pytest.approx(highs.solve(m, relax=True).objective)
    assert lpb.objective >= s.objective - 1e-9


def test_mpp_flow_distance_model():
    adj = grid_graph(2, 3)
    m = mpp_model(adj, [0, 5], [5, 0], 4, objective="dist")
    s = builtin.solve(m)
    h = highs.solve(m, mip_gap=0.0)
    assert s.status == h.status == "optimal"
    assert s.objective == h.objective == 6.0  # two robots, Manhattan distance 3 each


# ----------------------------------------------------------------------------- HiGHS
def test_highs_statuses_and_relax():
    m = LinearModel("min")
    m.add_column(0, 1, "B")
    m.add_row([(0, 1.0)], ">=", 0.5)
    m.set_objective([(0, 1.0)], constant=2.0)
    s = highs.solve(m)
    assert s.status == "optimal" and s.objective == 3.0 and s.x == [1.0]
    r = highs.solve(m, relax=True)
    assert r.objective == pytest.approx(2.5) and r.bound == pytest.approx(2.5)
    m.add_row([(0, 1.0)], "<=", 0.2)
    assert highs.solve(m).status == "infeasible"
    e = LinearModel()
    assert highs.solve(e).status == "optimal"
    e.add_row([], ">=", 1.0)
    assert highs.solve(e).status == "infeasible"


def test_highs_time_and_node_limit_mapping():
    rng = random.Random(8)
    m, _, _ = assignment_model(rng, 6)
    s = highs.solve(m, time_limit=60.0, params={"presolve": True})
    assert s.status == "optimal"
    # a knapsack-like model that needs branching, stopped at one node
    k = LinearModel("max")
    w = [rng.randint(5, 40) for _ in range(30)]
    for i in range(30):
        k.add_column(0, 1, "B")
    k.add_row([(i, float(w[i])) for i in range(30)], "<=", float(sum(w) // 2) + 0.5)
    k.set_objective([(i, float(w[i]) + rng.random()) for i in range(30)])
    s = highs.solve(k, params={"node_limit": 1, "presolve": False})
    assert s.status in ("node_limit", "optimal")
    assert (s.status == "node_limit") == (s.extra["highs_model_status"] == 16)


@contextlib.contextmanager
def quiet_c_stdout():
    """Discard what native code prints to stdout (scipy 1.17.1's HiGHS prints a debug line,
    "HighsMipSolverData::transformNewIntegerFeasibleSolution tmpSolver.run();", on some
    MIPs; it sits in the C stdio buffer and would surface after pytest's capture ends)."""
    try:
        libc = ctypes.CDLL(None)
    except OSError:  # pragma: no cover - not a POSIX libc
        yield
        return
    sys.stdout.flush()
    libc.fflush(None)
    saved = os.dup(1)
    devnull = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull, 1)
        yield
    finally:
        libc.fflush(None)
        os.dup2(saved, 1)
        os.close(devnull)
        os.close(saved)


def equality_knapsack(seed: int, n: int, constant: float = 0.0) -> LinearModel:
    """max sum v x, sum w x == w(S) for a random subset S (so it is feasible), x binary."""
    r = random.Random(seed)
    w = [r.randint(1000, 9999) for _ in range(n)]
    sub = [r.random() < 0.5 for _ in range(n)]
    rhs = sum(wi for wi, si in zip(w, sub) if si)
    m = LinearModel("max")
    for _ in range(n):
        m.add_column(0, 1, "B")
    m.add_row([(i, float(w[i])) for i in range(n)], "==", float(rhs))
    m.set_objective([(i, float(r.randint(1, 50))) for i in range(n)], constant=constant)
    return m


def test_highs_node_limit_without_incumbent_is_not_infeasible():
    """Regression: HiGHS reports a node limit as model status 16 ("Solution limit reached",
    scipy status 4); it used to be mapped to 'infeasible' when there was no incumbent."""
    m = equality_knapsack(11, 40)
    with quiet_c_stdout():
        assert highs.solve(m).status == "optimal"
    for p in ({"node_limit": 1}, {"node_limit": 1, "presolve": False}, {"node_limit": 2}):
        with quiet_c_stdout():
            s = highs.solve(m, params=p)
        assert s.status == "node_limit", (p, s.raw_status)
        assert s.x is None and s.objective is None and s.mip_gap is None
        assert s.extra["highs_model_status"] == 16


def test_highs_mip_gap_is_relative_gap_with_constant():
    """Regression: mip_gap is relative_gap(objective, bound) of the reported values (which
    include obj_constant), not HiGHS's own gap of the constant-free objective."""
    with_incumbent = 0
    for seed in range(11, 20):
        m = equality_knapsack(seed, 60, constant=-1000.0)
        with quiet_c_stdout():
            s = highs.solve(m, params={"node_limit": 3})
        assert s.status in ("node_limit", "optimal")
        if s.x is not None:
            with_incumbent += 1
            assert s.objective == pytest.approx(m.objective_value(s.x))
            assert s.mip_gap == relative_gap(s.objective, s.bound)
            if s.status == "node_limit":
                assert s.bound >= s.objective
    assert with_incumbent >= 1
    with quiet_c_stdout():
        s = highs.solve(equality_knapsack(11, 40, constant=5.0))
    assert s.status == "optimal" and s.mip_gap == relative_gap(s.objective, s.bound)


def test_highs_status_code_parsing():
    assert highs._highs_model_status(
        "The HiGHS status code was not recognized. (HiGHS Status 16: model_status is "
        "Solution limit reached; primal_status is None)") == 16
    assert highs._highs_model_status("no code here") is None


def model_293() -> LinearModel:
    """Stress model 293 (seed 11): feasible, optimum 0.1; integer column 3 has bounds
    [0.5, 1.2]. Unrounded, the presolve of scipy 1.17.1's HiGHS called it infeasible."""
    m = LinearModel("min")
    for lb, ub, vt, c in ((0, 1, "B", 0.0), (0, 0, "I", 1.0), (0, 1, "B", 3.0),
                          (0.5, 1.2, "I", 3.0), (0.5, 0.5, "C", 1.0), (1.0, 1.7, "C", -2.0)):
        m.add_column(lb, ub, vt, obj=c)
    m.add_row([(2, -1.0), (1, -1.0), (3, 1.0), (4, 3.0)], ">=", 2.5)
    return m


def model_390() -> LinearModel:
    """Stress model 390 (seed 11): max, optimum -0.75; integer column 4 has bounds
    [-1.5, -0.8]. Unrounded, the HiGHS of scipy 1.17.1 and 1.15.3 called the suboptimal -2.75
    optimal."""
    m = LinearModel("max")
    for lb, ub, vt, c in ((0, 3, "I", 0.25), (0, 1, "B", -2.0), (0, 0, "C", 3.0),
                          (1, 2, "C", -2.0), (-1.5, -0.8, "I", -1.0)):
        m.add_column(lb, ub, vt, obj=c)
    m.add_row([(0, 2.0), (2, 0.5), (3, 3.0), (4, 1.0), (4, 2.0), (4, 1.0)], "<=", 1.0)
    m.add_row([(0, -3.0), (0, -1.0), (1, 3.0), (0, 0.5)], "<=", 0.0)
    m.add_row([(3, -1.0), (1, -2.0), (2, 2.0), (1, 2.0), (4, 2.0)], "<=", 7.0)
    m.add_row([(2, -2.0), (3, -3.0), (0, -2.0)], "<=", -2.0)
    m.add_row([(4, 2.0), (1, -2.0)], "<=", -2.0)
    return m


def test_highs_rounds_fractional_integer_bounds():
    """Regression: integer columns' fractional bounds are rounded inward before HiGHS (as
    builtin does); HiGHS presolve gave wrong answers on them."""
    for m, want in ((model_293(), 0.1), (model_390(), -0.75)):
        b = builtin.solve(m)
        with quiet_c_stdout():
            h = highs.solve(m, mip_gap=0.0)
            h_np = highs.solve(m, mip_gap=0.0, params={"presolve": False})
        assert b.status == h.status == h_np.status == "optimal"
        assert b.objective == pytest.approx(want)
        assert h.objective == pytest.approx(want) and h_np.objective == pytest.approx(want)
        check_feasible(m, h.x)
    # an integer column whose rounded bounds are empty
    e = LinearModel("min")
    e.add_column(0.2, 0.8, "I", obj=1.0)
    assert highs.solve(e).status == builtin.solve(e).status == "infeasible"
    # relax keeps the fractional bounds (true LP relaxation)
    r = highs.solve(e, relax=True)
    assert r.status == "optimal" and r.objective == pytest.approx(0.2)
    # integral and near-integral bounds, and infinite bounds
    g = LinearModel("max")
    g.add_column(-INF, 2.0000000001, "I", obj=1.0)
    g.add_column(0.9999999999, INF, "I", obj=-1.0)
    g.add_row([(0, 1.0), (1, 1.0)], ">=", -5.0)
    s = highs.solve(g)
    assert s.status == "optimal" and s.x == [2.0, 1.0] and s.objective == 1.0


def stress_model(rng: random.Random) -> LinearModel:
    """A stress generator: small mixed
    models, with fractional bounds on integer columns."""
    n = rng.randint(1, 7)
    m_rows = rng.randint(0, 6)
    m = LinearModel(rng.choice(["max", "min"]))
    for _ in range(n):
        vt = rng.choice("CBI")
        lb = rng.choice([0, -2, -1.5, 1, 0.5])
        ub = lb + rng.choice([0, 0.7, 1, 3, 5.5])
        if vt == "B":
            lb, ub = 0, 1
        m.add_column(lb, ub, vt)
    for _ in range(m_rows):
        terms = [(rng.randrange(n), rng.choice([-3, -2, -1, 1, 2, 3, 0.5]))
                 for _ in range(rng.randint(1, n + 1))]
        m.add_row(terms, rng.choice(["<=", ">=", "=="]), rng.choice([-2, 0, 1, 2.5, 4, 7]))
    m.set_objective([(j, rng.choice([-2, -1, 0, 1, 3, 0.25])) for j in range(n)],
                    constant=rng.choice([0, 5, -3]))
    return m


def test_highs_agrees_with_builtin_on_stress_models():
    """Regression for both HiGHS fixes: the first 1000 stress models of seed 11 (which
    include models 293, 296, 390 and 619, wrong before the bound rounding, and 968, a
    presolve 'Solve error' before the fallback) agree between HiGHS and builtin."""
    rng = random.Random(11)
    models = [stress_model(rng) for _ in range(1000)]
    for t, m in enumerate(models):
        b = builtin.solve(m, node_limit=20000)
        with quiet_c_stdout():
            h = highs.solve(m, mip_gap=0.0)
        assert h.status == b.status, (t, h.raw_status, b.raw_status)
        if b.objective is not None:
            assert h.objective == pytest.approx(b.objective, rel=1e-6, abs=1e-6), t


def model_968() -> LinearModel:
    """Stress model 968 (seed 11): infeasible, integral bounds; scipy 1.17.1's HiGHS presolve
    ends it with status 4 'Solve error' and no solution."""
    m = LinearModel("min")
    x0 = m.add_column(-2, -1, "I")
    x1 = m.add_column(0, 1, "B")
    x2 = m.add_column(0, 1, "B")
    m.add_row([(x0, 0.5), (x1, -3), (x2, 2), (x0, -2)], "==", 2.5)
    m.add_row([(x0, 3)], "<=", 2.5)
    m.set_objective([(x1, 0.25), (x2, 3)], constant=5)
    return m


def test_highs_presolve_error_falls_back_to_no_presolve():
    """Regression: an error status without a solution is retried once with presolve off
    before RuntimeError is raised (an infeasible MIP used to raise)."""
    m = model_968()
    assert builtin.solve(m).status == "infeasible"
    for g in (None, 0.0):
        with quiet_c_stdout():
            s = highs.solve(m, mip_gap=g)
        assert s.status == "infeasible" and s.extra["highs_model_status"] == 8
        if "highs_retry" in s.extra:  # scipy 1.17.1's HiGHS: presolve fails with status 4
            assert s.extra["highs_retry"]["first_status"] not in highs._DEFINITE
    with quiet_c_stdout():
        s = highs.solve(m, params={"presolve": False})
    assert s.status == "infeasible" and "highs_retry" not in s.extra


def test_highs_retry_logic(monkeypatch):
    """The fallback, independently of the HiGHS version: a faked status-4 first result."""
    import scipy.optimize
    from scipy.optimize import OptimizeResult

    real = scipy.optimize.milp
    calls = []

    def error_once(*a, options=None, **k):
        calls.append(dict(options))
        if len(calls) == 1:
            return OptimizeResult(status=4, x=None, fun=None, mip_dual_bound=None,
                                  message="(HiGHS Status 4: Solve error)")
        return real(*a, options=options, **k)

    monkeypatch.setattr(scipy.optimize, "milp", error_once)
    m = LinearModel("max")
    m.add_column(0, 3, "I", obj=1.0)
    m.add_row([(0, 2.0)], "<=", 5.0)
    s = highs.solve(m, time_limit=30.0)
    assert s.status == "optimal" and s.objective == 2.0
    assert s.extra["highs_retry"] == {"presolve": False, "first_status": 4,
                                      "first_message": "(HiGHS Status 4: Solve error)"}
    assert len(calls) == 2 and "presolve" not in calls[0] and calls[1]["presolve"] is False
    assert 0.0 < calls[1]["time_limit"] <= 30.0

    def always_error(*a, options=None, **k):
        calls.append(dict(options))
        return OptimizeResult(status=4, x=None, fun=None, mip_dual_bound=None,
                              message="(HiGHS Status 4: Solve error)")

    monkeypatch.setattr(scipy.optimize, "milp", always_error)
    calls.clear()
    with pytest.raises(RuntimeError, match="also with presolve off"):
        highs.solve(m)
    assert len(calls) == 2
    calls.clear()
    with pytest.raises(RuntimeError):  # presolve already off: no second attempt
        highs.solve(m, params={"presolve": False})
    assert len(calls) == 1
    # a definite status is never retried
    calls.clear()
    monkeypatch.setattr(scipy.optimize, "milp", lambda *a, options=None, **k: (
        calls.append(1), OptimizeResult(status=2, x=None, fun=None, mip_dual_bound=None,
                                        message="(HiGHS Status 8: Infeasible)"))[1])
    assert highs.solve(m).status == "infeasible" and len(calls) == 1
    # another status with a solution (e.g. HiGHS's solution limit) is "feasible", not retried
    calls.clear()
    monkeypatch.setattr(scipy.optimize, "milp", lambda *a, options=None, **k: (
        calls.append(1), OptimizeResult(status=1, x=[1.0], fun=-1.0, mip_dual_bound=-2.0,
                                        message="(HiGHS Status 15: Solution limit reached)"))[1])
    s = highs.solve(m)
    assert (s.status, s.objective, s.bound, s.x, len(calls)) == ("feasible", 1.0, 2.0, [1.0], 1)


# ------------------------------------------------------------------- solve() dispatch
def test_dispatch_and_available_backends():
    av = lp.available_backends()
    assert "builtin" in av and "highs" in av
    m = LinearModel("max")
    m.add_column(0, 2, "I")
    m.add_row([(0, 2.0)], "<=", 3.0)
    m.set_objective([(0, 1.0)])
    for b in ("highs", "builtin"):
        assert lp.solve(m, backend=b).objective == 1.0
        assert lp.solve(m, backend=b, relax=True).objective == pytest.approx(1.5)
    with pytest.raises(ValueError):
        lp.solve(m, backend="cplex")


# (lb, ub, sense) of one binary column x maximised or minimised -> (status, x), also relaxed
BINARY_BOUNDS = [
    ((-5, 10, "max"), ("optimal", 1.0), ("optimal", 1.0)),
    ((-5, 10, "min"), ("optimal", 0.0), ("optimal", 0.0)),
    ((0.5, 3, "min"), ("optimal", 1.0), ("optimal", 0.5)),
    ((-3, 0.5, "max"), ("optimal", 0.0), ("optimal", 0.5)),
    ((0.25, 0.75, "max"), ("infeasible", None), ("optimal", 0.75)),
    ((-2, -1, "max"), ("infeasible", None), ("infeasible", None)),
    ((2, 3, "min"), ("infeasible", None), ("infeasible", None)),
]


@pytest.mark.parametrize("backend", ["highs", "builtin", "gurobi"])
def test_binary_bounds_are_intersected_with_0_1(backend):
    """Every backend intersects a binary column's bounds with [0, 1] (as Gurobi does, and as
    LinearModel.relax keeps them), also under relax; HiGHS used to need the clamp spelled
    out: without it, x = 10 for bounds [-5, 10]."""
    for (lo, hi, sense), want, want_relaxed in BINARY_BOUNDS:
        for relax, w in ((False, want), (True, want_relaxed)):
            m = LinearModel(sense)
            j = m.add_column(lo, hi, "B")
            m.set_objective([(j, 1.0)])
            sol = lp.solve(m, backend=backend, relax=relax, params=GRB_PARAMS if backend == "gurobi" else None)
            got = (sol.status, None if sol.x is None else sol.x[0] + 0.0)
            assert got == w, (lo, hi, sense, relax, got)


# ---------------------------------------------------------------------------- Gurobi
@pytest.mark.gurobi
@needs_gurobi
def test_gurobi_random_milps_agree():
    rng = random.Random(31)
    for _ in range(80):
        model = random_model(rng, integer=True)
        g = lp.solve(model, backend="gurobi", mip_gap=0.0, params=GRB_PARAMS)
        b = builtin.solve(model)
        assert_agree(model, g, b)
        if g.status == "optimal":
            assert g.sol_count >= 1 and g.raw_status == 2


@pytest.mark.gurobi
@needs_gurobi
def test_gurobi_lps_and_statuses():
    rng = random.Random(32)
    for _ in range(80):
        model = random_model(rng, integer=False, allow_inf=False)
        g = lp.solve(model, backend="gurobi", params=GRB_PARAMS)
        h = highs.solve(model)
        assert_agree(model, g, h)
        if g.status == "optimal":
            assert g.bound == g.objective and g.mip_gap == 0.0


@pytest.mark.gurobi
@needs_gurobi
def test_gurobi_mpp_model_exact_binaries_and_repeatable():
    adj = grid_graph(2, 3)
    m = mpp_model(adj, [0, 2, 3], [2, 0, 5], 4)
    a = lp.solve(m, backend="gurobi", params=GRB_PARAMS)
    b = lp.solve(m, backend="gurobi", params=GRB_PARAMS)
    assert a.status == "optimal" and a.objective == 3.0
    assert a.x == b.x  # deterministic with Threads=1
    assert all(v in (0.0, 1.0) for v in a.x)
    r = lp.solve(m, backend="gurobi", relax=True, params=GRB_PARAMS)
    assert r.objective == pytest.approx(highs.solve(m, relax=True).objective)


@pytest.mark.gurobi
@needs_gurobi
def test_gurobi_builds_model_in_order(tmp_path):
    """The Gurobi model has the columns, rows and objective in LinearModel order."""
    import gurobipy as gp

    from mppflow.lp import gurobi as gmod

    m = LinearModel("min")
    m.add_column(0, 1, "B", name="a")
    m.add_column(0, 5, "I")
    m.add_column(-1, 2, "C", name="c")
    m.add_row([(2, 1.0), (0, 2.0)], ">=", 1.0, name="r0")
    m.add_row([(1, 1.0), (2, -1.0)], "==", 0.0)
    m.set_objective([(1, 3.0), (0, 1.0)], constant=0.5)
    captured = {}
    orig = gmod._build_and_solve

    def spy(gp_, GRB, gm, model, t0, *rest):
        sol = orig(gp_, GRB, gm, model, t0, *rest)
        gm.update()
        vs = gm.getVars()
        captured["vars"] = [(v.LB, v.UB, v.VType, v.Obj) for v in vs]
        captured["names"] = [v.VarName for v in vs]
        captured["rows"] = []
        for c in gm.getConstrs():
            row = gm.getRow(c)
            captured["rows"].append(([(row.getVar(k).index, row.getCoeff(k))
                                      for k in range(row.size())], c.Sense, c.RHS))
        captured["sense"] = gm.ModelSense
        captured["objcon"] = gm.ObjCon
        return sol

    gmod._build_and_solve = spy
    try:
        s = gmod.solve(m, params=GRB_PARAMS)
    finally:
        gmod._build_and_solve = orig
    assert captured["vars"] == [(0.0, 1.0, "B", 1.0), (0.0, 5.0, "I", 3.0), (-1.0, 2.0, "C", 0.0)]
    assert captured["names"][0] == "a" and captured["names"][2] == "c"
    assert sorted(captured["rows"][0][0]) == [(0, 2.0), (2, 1.0)]
    assert captured["rows"][0][1:] == (">", 1.0)
    assert sorted(captured["rows"][1][0]) == [(1, 1.0), (2, -1.0)]
    assert captured["sense"] == 1 and captured["objcon"] == 0.5
    assert s.status == "optimal" and s.objective == pytest.approx(1.5)  # a = 1, b = c = 0
    assert isinstance(s.x, list) and s.extra["gurobi_version"][0] >= 11


@pytest.mark.gurobi
@needs_gurobi
def test_gurobi_infeasible_and_limits():
    m = LinearModel("max")
    m.add_column(0, 1, "B")
    m.add_row([(0, 1.0)], ">=", 2.0)
    s = lp.solve(m, backend="gurobi", params=GRB_PARAMS)
    assert s.status == "infeasible" and s.x is None and s.sol_count == 0
    rng = random.Random(8)
    k = LinearModel("max")
    w = [rng.randint(5, 40) for _ in range(40)]
    for i in range(40):
        k.add_column(0, 1, "B")
    k.add_row([(i, float(w[i])) for i in range(40)], "<=", float(sum(w) // 2) + 0.5)
    k.set_objective([(i, float(w[i]) + rng.random()) for i in range(40)])
    s = lp.solve(k, backend="gurobi",
                 params={"Threads": 1, "NodeLimit": 0, "Presolve": 0, "Cuts": 0,
                         "Heuristics": 0})
    assert s.status in ("node_limit", "optimal")
    assert s.raw_status in (8, 2)


@pytest.mark.gurobi
@needs_gurobi
def test_gurobi_objective_after_add_column_matches_other_backends():
    m = LinearModel("max")
    a = m.add_column(0, 1)
    m.set_objective([(a, 1.0)])
    m.add_column(0, 1, obj=5.0)
    s = lp.solve(m, backend="gurobi", params=GRB_PARAMS)
    assert s.status == "optimal" and s.objective == 6.0 and s.x == [1.0, 1.0]


@pytest.mark.gurobi
@needs_gurobi
def test_gurobi_unbounded_has_no_solution():
    """Regression: Gurobi's unbounded result carried x = [1e30, ...] and SolCount 1, and with
    DualReductions=0 an unbounded MIP came back as 'optimal' with objective -inf."""
    for vt in ("C", "I"):
        for extra in ({}, {"DualReductions": 0}):
            m = LinearModel("min")
            x = m.add_column(0, INF, vt)
            y = m.add_column(0, 1, "B")
            m.set_objective([(x, -1.0), (y, 1.0)])
            s = lp.solve(m, backend="gurobi", params={**GRB_PARAMS, **extra})
            assert s.status == "unbounded", (vt, extra, s.raw_status)
            assert s.x is None and s.objective is None and not s.has_solution
            assert s.sol_count == 0 and s.bound is None and s.mip_gap is None
            assert builtin.solve(m).status == "unbounded"


def test_gurobi_missing_gives_clear_import_error(monkeypatch):
    import sys

    from mppflow.lp import gurobi as gmod

    monkeypatch.setitem(sys.modules, "gurobipy", None)  # makes "import gurobipy" fail
    assert not gmod.available()
    assert "gurobi" not in lp.available_backends()
    m = LinearModel()
    m.add_column(0, 1)
    with pytest.raises(ImportError, match="gurobipy"):
        lp.solve(m, backend="gurobi")


# ------------------------------------------- HiGHS crash isolation (regression notes-2)
def _m320():
    """docs/notes/repro/highs_presolve_bug.py model m320 (integral bounds): scipy 1.17.1's
    presolve usually segfaults on it and sometimes reports "infeasible"; optimum 1.0."""
    path = os.path.join(os.path.dirname(__file__), "..", "docs", "notes", "repro", "highs_presolve_bug.py")
    with open(path, encoding="utf-8") as f:
        src = f.read()
    ns: dict = {}
    exec(compile(src.split("\ndef raw(")[0], path, "exec"), ns)  # only the MODELS table
    sense, cols, rows = ns["MODELS"]["m320"]
    m = LinearModel(sense)
    for lb, ub, vt, obj in cols:
        m.add_column(lb, ub, vt, obj=obj)
    for terms, s, r in rows:
        m.add_row(terms, s, r)
    return m


def test_highs_isolated_solve_survives_presolve_crash():
    """Regression (notes-2): in-process, m320 kills the interpreter (or, rarely, is reported
    infeasible); isolated, a crash is retried with presolve off and the optimum is found."""
    import numpy as np

    m = _m320()
    for _ in range(3):
        s = highs.solve(m, mip_gap=0.0, params={"isolate": True})
        assert s.status == "optimal" and s.objective == pytest.approx(1.0)
    assert builtin.solve(m).objective == pytest.approx(1.0)
    # a worker that dies comes back as an error status without a solution
    r = highs._milp_isolated(np.zeros(1), np.zeros(1, dtype=np.uint8), np.zeros(1), np.ones(1), None,
                             {"_mppflow_test_crash": True})
    assert r.x is None and highs._code_of(r) == -2 and "died (signal 9)" in r.message
    one = LinearModel("min")
    one.add_column(0, 1, "B", obj=1.0)
    one.add_row([(0, 1.0)], ">=", 1.0)
    assert highs.solve(one, params={"isolate": True}).objective == 1.0  # a new worker


def test_highs_isolated_worker_pipes_are_closed():
    """Regression (TB2): a worker that died, and the worker stopped at exit, kept their pipes
    open until garbage collection (``ResourceWarning: unclosed file`` under ``-W error``)."""
    import numpy as np

    a = (np.zeros(1), np.zeros(1, dtype=np.uint8), np.zeros(1), np.ones(1), None)
    highs._milp_isolated(*a, {})
    p = highs._worker["proc"]
    highs._milp_isolated(*a, {"_mppflow_test_crash": True})
    assert p.stdin.closed and p.stdout.closed and highs._worker["proc"] is None
    highs._milp_isolated(*a, {})
    p = highs._worker["proc"]
    highs._stop_worker()
    assert p.returncode is not None and p.stdin.closed and p.stdout.closed and highs._worker["proc"] is None


def _distinct_models():
    """Three small MILPs with different sizes and optima (a stale answer shows in both)."""
    out = []
    for n in (3, 5, 8):
        m = LinearModel("min")
        for j in range(n):
            m.add_column(0, 4, "I", obj=1.0 + j)
        m.add_row([(j, 1.0) for j in range(n)], ">=", n + 0.5)
        out.append((m, highs.solve(m, mip_gap=0.0).objective))  # in process: 4, 8, 15
    assert len({o for _, o in out}) == 3
    return out


def _assert_isolated_answers_match(models):
    for _ in range(2):
        for m, opt in models:
            s = highs.solve(m, mip_gap=0.0, params={"isolate": True})
            assert s.status == "optimal" and s.objective == pytest.approx(opt)
            assert len(s.x) == m.num_columns


def test_highs_isolated_interrupt_does_not_desync_the_worker(monkeypatch):
    """Regression (TD2-ISOLATE-INTERRUPT-DESYNC): an exception raised while the parent waited
    for the worker (KeyboardInterrupt, a SIGALRM handler) left the worker alive with the
    request in progress, so every later isolated solve read the answer to the previous
    request (wrong objective, an x of another model's length). Now the worker is killed."""
    import numpy as np

    models = _distinct_models()
    highs._milp_isolated(np.zeros(1), np.zeros(1, dtype=np.uint8), np.zeros(1), np.ones(1), None, {})
    p = highs._worker["proc"]

    def interrupted(f):
        raise KeyboardInterrupt  # the request is out, its answer not read

    monkeypatch.setattr(highs, "_recv", interrupted)
    with pytest.raises(KeyboardInterrupt):
        highs.solve(models[0][0], params={"isolate": True})
    monkeypatch.undo()
    assert highs._worker["proc"] is None and p.returncode is not None and p.stdin.closed and p.stdout.closed
    _assert_isolated_answers_match(models)
    highs._stop_worker()


@pytest.mark.skipif(not hasattr(signal, "setitimer"), reason="needs signal.setitimer (POSIX)")
def test_highs_isolated_solve_interrupted_by_a_signal():
    """The same through a real signal: SIGALRM 0.5 s into an isolated solve of the 16-puzzle
    model at T = 7 (about 50 s in the worker) raises from the handler; the next isolated
    solves get their own answers, at once (the old worker is not waited for)."""
    import time

    from mppflow._java import MathRandom
    from mppflow.model import build_time_model
    from mppflow.problem import create_n2_puzzle

    big = create_n2_puzzle(4, MathRandom(1))
    mb = build_time_model(big.graph, big.starts, big.goals, 7).model

    class Stop(Exception):
        pass

    def handler(*a):
        raise Stop

    old = signal.signal(signal.SIGALRM, handler)
    try:
        signal.setitimer(signal.ITIMER_REAL, 0.5)
        with pytest.raises(Stop):
            highs.solve(mb, params={"isolate": True})
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old)
    t = time.monotonic()
    _assert_isolated_answers_match(_distinct_models())
    assert time.monotonic() - t < 30
    highs._stop_worker()


def test_plan_isolated_matches_in_process():
    """plan(..., params={"isolate": True}) gives the in-process plan's makespan and a valid plan."""
    from mppflow._java import MathRandom
    from mppflow.planner import path_set_error, plan
    from mppflow.problem import create_grid_problem

    p = create_grid_problem(3, 3, 0.0, 2, MathRandom(3))
    ref = plan(p, rng=MathRandom(1))
    r = plan(p, rng=MathRandom(1), params={"isolate": True})
    highs._stop_worker()
    assert r.status == ref.status == "optimal" and r.makespan == ref.makespan
    assert path_set_error(r.paths, p.graph, p.starts, p.goals) is None


# ----------------------------- builtin numerical robustness (regression builtin-lp-*)
@pytest.mark.parametrize("integer", [False, True])
def test_builtin_refactorization_every_iteration_agrees(monkeypatch, integer):
    """The refactorization (builtin._reinvert) recomputes the tableau for the current basis,
    complemented columns included; with REFRESH = 1 it runs before every pivot and once more
    before optimality is accepted, and the results still agree with HiGHS."""
    monkeypatch.setattr(builtin, "REFRESH", 1)
    rng = random.Random(777 + integer)
    for _ in range(150):
        # (as the other MILP tests, no infinite bounds with integers: HiGHS reports an
        # unbounded MILP as "infeasible", status 9)
        model = random_model(rng, integer=integer, allow_inf=not integer)
        assert_agree(model, builtin.solve(model), highs.solve(model, mip_gap=0.0))


def test_builtin_primal_check_rejects_garbage():
    """A numerically broken tableau used to return "optimal" for x ~ 1e14 that violated its
    own bounds and rows (the B&B even accepted such a point as integral). The final primal
    check turns that into an error."""
    A = [[1.0, 1.0]]
    builtin._check_primal(2, A, ["<="], [1.0], [0.0, 0.0], [1.0, 1.0], [0.5, 0.5])
    with pytest.raises(RuntimeError, match="numerical failure.*column 0"):
        builtin._check_primal(2, A, ["<="], [1.0], [0.0, 0.0], [1.0, 1.0], [3.1e14, 0.0])
    with pytest.raises(RuntimeError, match="numerical failure.*row 0"):
        builtin._check_primal(2, A, ["=="], [1.0], [0.0, 0.0], [1.0, 1.0], [1.0, 1.0])
    with pytest.raises(RuntimeError, match="numerical failure"):
        builtin._check_primal(1, [[1.0]], [">="], [0.0], [0.0], [1.0], [float("nan")])


def test_old_scipy_warns_once(monkeypatch):
    """scipy older than highs.MIN_SCIPY (pyproject's floor): the first HiGHS solve warns."""
    import scipy

    m = LinearModel("max")
    m.add_column(0, 3, "I", obj=1.0)
    monkeypatch.setattr(highs, "_scipy_checked", False)
    monkeypatch.setattr(scipy, "__version__", "1.14.1")
    with pytest.warns(RuntimeWarning, match=r"scipy >= 1\.15\.3 .* found 1\.14\.1"):
        assert highs.solve(m).objective == 3.0
    with warnings.catch_warnings():
        warnings.simplefilter("error")
        highs.solve(m)  # once per process
    # 1.15.0-1.15.2 fetch the column duals in quadratic time (test_model's T = 10000 model
    # does not finish there); the patch level counts
    for old in ("1.15.0", "1.15.2", "1.15"):
        monkeypatch.setattr(highs, "_scipy_checked", False)
        monkeypatch.setattr(scipy, "__version__", old)
        with pytest.warns(RuntimeWarning, match=rf"scipy >= 1\.15\.3 .* found {re.escape(old)}:.*quadratic"):
            highs.solve(m)
    for ok in ("1.15.3", "1.16.0rc1", "1.17.1", "2.0"):
        monkeypatch.setattr(highs, "_scipy_checked", False)
        monkeypatch.setattr(scipy, "__version__", ok)
        with warnings.catch_warnings():
            warnings.simplefilter("error")
            highs.solve(m)
    with open(os.path.join(os.path.dirname(__file__), "..", "pyproject.toml"), encoding="utf-8") as f:
        assert '"scipy>=' + ".".join(map(str, highs.MIN_SCIPY)) + '"' in f.read()
