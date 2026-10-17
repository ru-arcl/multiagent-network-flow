"""Pure-Python exact MILP solver for small models.

This backend needs nothing beyond the standard library. It is written to be ported line
by line to JavaScript (``docs/js/lp.js``) and to give **bit-identical** floating-point results
there: it uses plain lists of floats, explicit loops in a fixed order, only ``+ - * /``,
comparisons and ``abs`` (``floor`` is computed from truncation, which is exact in both
languages), and never sums in an order that depends on hashing.

Algorithm
---------
LP: dense two-phase primal simplex on a full tableau, with bounded variables handled
natively (no explicit bound rows):

* Each column is shifted to ``x = lb + x'`` with ``0 <= x' <= u = ub - lb`` (``u`` may be
  ``inf``). Lower bounds must be finite.
* Every row gets a slack (``+s`` for ``<=``, ``-s`` for ``>=``, none for ``==``), and is
  multiplied by -1 if its shifted right-hand side is negative. A row whose slack then has
  coefficient +1 starts with the slack basic; every other row gets an artificial column.
  Column order: structural, then slacks (in row order), then artificials (in row order).
* A nonbasic variable at its upper bound is *complemented* (``y = u - x'``), so every
  nonbasic variable of the tableau is at 0. The ratio test considers a basic variable
  falling to 0, a basic variable rising to its upper bound, and the entering variable
  reaching its own upper bound (a bound flip, which changes no basis).
* Bland's rule: the entering variable is the lowest-index column with reduced cost
  ``< -EPS``; among tied blocking variables, the lowest index leaves. Ratios within
  ``TIE_TOL * (1 + t)`` of the best ratio ``t`` count as tied, and a basic variable within
  ``ZERO_TOL`` of its bound counts as at the bound, so degenerate steps tie exactly at 0.
  (Floating-point noise used to break these ties. That broke Bland's rule, so the method
  could stall or cycle on the highly degenerate MPP models.)
* Refactorization: every ``REFRESH`` iterations, and once more before optimality is
  accepted on an LP that needed at least that many, the tableau, the basic values and the
  reduced costs are recomputed from the initial rows for the current basis (Gauss-Jordan
  with partial pivoting, :func:`_reinvert`). Without this, rounding errors in the dense
  tableau grew without bound on models of a few hundred rows. The tableau entries reached
  1e21, noise was accepted as pivots, and the solver either hit the iteration cap or
  reported "optimal" for a point around 1e14 that violated its own bounds.
* Every LP solution is checked against the original bounds and rows (relative tolerance
  ``FEAS_TOL``). A violation raises ``RuntimeError`` ("numerical failure") instead of
  reaching the branch and bound as a wrong optimum.
* Phase 1 minimizes the sum of artificials; the model is infeasible if that sum exceeds
  ``EPS * (1 + max_i |b_i|)``. Artificials still basic (at zero) are then pivoted out on the
  lowest-index column with ``|entry| > EPS``; rows where none exists are redundant and are
  dropped. Phase 2 removes the artificial columns and minimizes the objective (a maximized
  objective is negated).

MILP: depth-first branch and bound, solving each node's LP from scratch (no warm start).
The branching variable is the lowest-index integer column whose LP value is fractional
(distance to the nearest integer above ``INT_EPS``). The child ``x >= ceil(v)`` is explored
before ``x <= floor(v)``. A node is pruned when its bound (its parent's LP value when popped,
its own LP value after solving) cannot improve the incumbent by more than
``max(EPS * (1 + |incumbent|), mip_gap * |incumbent objective|)``. The reported incumbent has its
integer columns snapped to the nearest integer, and its objective is recomputed from those
values. ``bound`` is the incumbent objective, except when ``mip_gap`` pruned a node whose
bound is better than the incumbent by more than ``EPS``: then ``bound`` is the best such
pruned bound (a valid bound, as Gurobi's ``ObjBound``) and ``mip_gap`` is
:func:`~mppflow.lp.linear.relative_gap` of objective and bound; the status stays
``'optimal'``, as Gurobi's does within ``MIPGap``. With ``mip_gap`` 0 (the default) the
bound always equals the objective.

Tolerances: ``EPS = 1e-9`` (pivots, reduced costs, phase-1 infeasibility),
``INT_EPS = 1e-6`` (integrality), ``ZERO_TOL = TIE_TOL = 1e-11`` (ratio test),
``FEAS_TOL = 1e-6`` (the final primal check).

Limits: ``node_limit`` (LP solves) and ``work_limit`` (total simplex iterations, bound flips
included) are deterministic. ``time_limit`` (wall clock) is not; it is checked between nodes
and before every simplex pivot (also inside a node's LP), so a solve overruns it by at most
one pivot. A dense pivot costs ``O(rows * columns)`` Python operations, which is why this
backend is for small models: a real MPP model with a few thousand columns needs minutes per
LP here and well under a second in HiGHS (the 532-column LP relaxation of a total-time
model on a 3x3 grid takes about 60 s and 32635 iterations, the same count as exact rational
arithmetic), and with a time limit it stops at the limit
(status ``'time_limit'``, ``x`` the incumbent or ``None``).
A reached work limit is reported as status ``'time_limit'`` with ``raw_status
'work_limit'``. Each simplex phase of an LP also stops after
``max(1000000, 1000 * (rows + columns))`` iterations: Bland's rule did not cycle in our runs
(ties are exact up to the tolerances) but can stall for very long on degenerate vertices (a
638-column, 404-row node LP of a max-distance model needed 268876 phase-1 iterations), and the
solve then ends as at a work limit (status ``'time_limit'``, ``raw_status 'iteration_cap'``,
``x`` the incumbent or ``None``). The cap was ``max(100000, 100 * (rows + columns))``, and a
reached cap raised ``RuntimeError``.

Integer columns should have finite upper bounds (all MPP models do): with an unbounded
integer column, the depth-first, ceil-first search can dive forever (on an
integer-infeasible model, and also on some feasible ones, where every ceil child stays
LP-feasible and fractional), and only ``node_limit`` / ``work_limit`` / ``time_limit`` stop it. An unbounded LP
relaxation is reported as ``'unbounded'``, even for a MIP.

This backend is new (the original always uses Gurobi); it has no ``compat`` switch.
"""

from __future__ import annotations

import time

from .linear import INF, LinearModel, Solution, relative_gap

EPS = 1e-9
INT_EPS = 1e-6
REFRESH = 100
"""Simplex iterations between two refactorizations of the tableau (:func:`_reinvert`)."""
ZERO_TOL = 1e-11
"""A basic variable within ``ZERO_TOL`` of its bound counts as at the bound in the ratio test
(so degenerate steps tie exactly at 0)."""
TIE_TOL = 1e-11
"""Ratios within ``TIE_TOL * (1 + t)`` of the best one are ties (Bland: lowest index leaves)."""
FEAS_TOL = 1e-6
"""Relative tolerance of the final primal check of every LP solution (:func:`_check_primal`)."""

_OPTIMAL = 0
_INFEASIBLE = 1
_UNBOUNDED = 2
_LIMIT = 3


def _floor(x: float) -> float:
    """``floor(x)`` for finite ``x`` via truncation (``Math.trunc`` in JavaScript)."""
    f = float(int(x))
    if f > x:
        f = f - 1.0
    return f


def _ceil(x: float) -> float:
    return -_floor(-x)


class _Work:
    """Simplex iteration counter and limits shared by all LPs of one solve.

    ``limit`` caps the total iterations; ``deadline`` is a ``time.perf_counter()`` value
    (``None``: none). ``reason`` records which of the two stopped an LP.
    """

    def __init__(self, limit, deadline=None):
        self.iterations = 0
        self.limit = limit
        self.deadline = deadline
        self.reason = None

    def exhausted(self):
        """True (and ``reason`` set) if the next simplex pivot must not start."""
        if self.limit is not None and self.iterations >= self.limit:
            self.reason = "work_limit"
            return True
        if self.deadline is not None and time.perf_counter() >= self.deadline:
            self.reason = "time_limit"
            return True
        return False


def _pivot(T, beta, d, r, e):
    """Pivot the tableau on row ``r``, column ``e`` (also updates the cost row ``d``)."""
    row = T[r]
    p = row[e]
    n = len(row)
    nz = []
    for k in range(n):
        if row[k] != 0.0:
            row[k] = row[k] / p
            nz.append(k)
    beta[r] = beta[r] / p
    row[e] = 1.0
    br = beta[r]
    m = len(T)
    for i in range(m):
        if i == r:
            continue
        Ti = T[i]
        f = Ti[e]
        if f != 0.0:
            for k in nz:
                Ti[k] = Ti[k] - f * row[k]
            beta[i] = beta[i] - f * br
            Ti[e] = 0.0
    f = d[e]
    if f != 0.0:
        for k in nz:
            d[k] = d[k] - f * row[k]
        d[e] = 0.0


def _flip(T, beta, d, e, u):
    """Complement nonbasic column ``e`` (``y_e -> u - y_e``): it moves to its other bound."""
    m = len(T)
    for i in range(m):
        a = T[i][e]
        if a != 0.0:
            beta[i] = beta[i] - a * u
            T[i][e] = -a
    d[e] = -d[e]


def _reduced_costs(T, basis, c):
    """``d_j = c_j - sum_i c[basis[i]] * T[i][j]`` (rows in order)."""
    n = len(c)
    d = []
    for j in range(n):
        d.append(c[j])
    m = len(T)
    for i in range(m):
        cb = c[basis[i]]
        if cb != 0.0:
            row = T[i]
            for k in range(n):
                if row[k] != 0.0:
                    d[k] = d[k] - cb * row[k]
    return d


def _reinvert(T0, beta0, basis, flipped, U, ncols, cost, T, beta, d):
    """Recompute the tableau of the current basis from the initial rows (a refactorization).

    The initial tableau ``T0 z = beta0`` is rewritten for the current complementation
    (column ``j`` negated and ``beta0`` reduced by ``T0[:, j] * U[j]`` when ``flipped[j]``),
    then Gauss-Jordan elimination pivots every basic column in, choosing the pivot row by the
    largest magnitude (lowest row on ties). Columns are processed in increasing order of
    their nonzero count (lowest position on ties), which keeps unit columns (slacks)
    free of fill. Rows that receive no pivot are dropped (they are the redundant rows dropped
    after phase 1). ``T``, ``beta`` and ``d`` (reduced costs of ``cost`` in the current
    complementation) are replaced in place, row ``i`` holding ``basis[i]``.

    Returns False, leaving everything unchanged, if the basis is numerically singular.
    """
    m0 = len(T0)
    M = []
    b = []
    for r in range(m0):
        src = T0[r]
        row = []
        br = beta0[r]
        for j in range(ncols):
            a = src[j]
            if a != 0.0 and flipped[j]:
                br = br - a * U[j]
                a = -a
            row.append(a)
        M.append(row)
        b.append(br)
    mrows = len(basis)
    nnz = []
    for i in range(mrows):
        k = basis[i]
        c = 0
        for r in range(m0):
            if M[r][k] != 0.0:
                c += 1
        nnz.append(c)
    order = sorted(range(mrows), key=lambda i: (nnz[i], i))
    used = [False] * m0
    prow = [-1] * mrows
    for i in order:
        k = basis[i]
        best = -1
        bv = 0.0
        for r in range(m0):
            if not used[r]:
                a = abs(M[r][k])
                if a > bv:
                    bv = a
                    best = r
        if best < 0 or bv <= EPS:
            return False
        used[best] = True
        prow[i] = best
        row = M[best]
        p = row[k]
        nz = []
        for j in range(ncols):
            if row[j] != 0.0:
                row[j] = row[j] / p
                nz.append(j)
        b[best] = b[best] / p
        row[k] = 1.0
        bb = b[best]
        for r in range(m0):
            if r == best:
                continue
            Mr = M[r]
            f = Mr[k]
            if f != 0.0:
                for j in nz:
                    Mr[j] = Mr[j] - f * row[j]
                b[r] = b[r] - f * bb
                Mr[k] = 0.0
    newT = []
    newb = []
    for i in range(mrows):
        newT.append(M[prow[i]])
        newb.append(b[prow[i]])
    c = []
    for j in range(ncols):
        cj = cost[j]
        if flipped[j]:
            cj = -cj
        c.append(cj)
    T[:] = newT
    beta[:] = newb
    d[:] = _reduced_costs(T, basis, c)
    return True


def _simplex(T, beta, d, basis, pos, U, flipped, ncols, work, cap, refresh=None):
    """Primal simplex with Bland's rule on columns ``0..ncols-1``.

    ``refresh()`` (optional) recomputes ``T``, ``beta`` and ``d`` from the original rows for
    the current basis (see :func:`_reinvert`); it is called every ``REFRESH`` iterations and,
    when the LP needed at least that many, once more before optimality is accepted.

    Returns ``_OPTIMAL``, ``_UNBOUNDED`` or ``_LIMIT``.
    """
    m = len(T)
    count = 0
    since = 0  # iterations since the last refresh
    while True:
        if refresh is not None and since >= REFRESH:
            refresh()
            since = 0
        e = -1
        for j in range(ncols):
            if pos[j] < 0 and U[j] > 0.0 and d[j] < -EPS:
                e = j
                break
        if e < 0:
            if refresh is not None and count >= REFRESH and since > 0:
                refresh()  # confirm optimality on a freshly computed tableau
                since = 0
                continue
            return _OPTIMAL
        if work.exhausted():
            return _LIMIT
        count += 1
        if count > cap:
            # Bland's rule can stall for a very long time on the degenerate MPP models (one
            # node LP of a max-distance model: 268876 phase-1 pivots, almost all degenerate,
            # every basis distinct, so no cycle): the solve stops as at a work limit
            work.reason = "iteration_cap"
            return _LIMIT
        since += 1
        work.iterations += 1
        best_t = U[e]
        best_var = e
        best_row = -1
        best_up = False
        for i in range(m):
            a = T[i][e]
            if a > EPS:
                num = beta[i]
                if num < ZERO_TOL:
                    num = 0.0
                t = num / a
                up = False
            elif a < -EPS:
                ubv = U[basis[i]]
                if ubv == INF:
                    continue
                num = ubv - beta[i]
                if num < ZERO_TOL:
                    num = 0.0
                t = num / (-a)
                up = True
            else:
                continue
            k = basis[i]
            if best_t == INF:
                better = True
            else:
                tt = TIE_TOL * (1.0 + best_t)
                better = t < best_t - tt or (t <= best_t + tt and k < best_var)
            if better:
                best_t = t
                best_var = k
                best_row = i
                best_up = up
        if best_row < 0:
            if best_t == INF:
                return _UNBOUNDED
            _flip(T, beta, d, e, U[e])
            flipped[e] = not flipped[e]
            continue
        r = best_row
        if best_up:
            lv = basis[r]
            T[r][lv] = -1.0
            beta[r] = beta[r] - U[lv]
            flipped[lv] = not flipped[lv]
        _pivot(T, beta, d, r, e)
        pos[basis[r]] = -1
        basis[r] = e
        pos[e] = r


def _solve_lp(n, A, senses, rhs, lb, ub, cmin, work):
    """Solve ``min cmin.x`` s.t. rows ``A x (senses) rhs``, ``lb <= x <= ub``.

    ``A`` is a list of dense rows. Returns ``(status, x)``; ``x`` is ``None`` unless optimal.
    """
    m = len(A)
    u = []
    for j in range(n):
        uj = ub[j] - lb[j]
        if uj < 0.0:
            return _INFEASIBLE, None
        u.append(uj)
    # shifted right-hand sides and row signs
    b = []
    bmax = 0.0
    for i in range(m):
        bi = rhs[i]
        Ai = A[i]
        for j in range(n):
            if Ai[j] != 0.0 and lb[j] != 0.0:
                bi = bi - Ai[j] * lb[j]
        b.append(bi)
        if abs(bi) > bmax:
            bmax = abs(bi)
    nslack = 0
    for i in range(m):
        if senses[i] != "==":
            nslack += 1
    first_art = n + nslack
    # decide the initial basis
    slack_col = []
    slack_coef = []
    sign = []
    needs_art = []
    k = n
    nart = 0
    for i in range(m):
        s = 1.0
        if b[i] < 0.0:
            s = -1.0
        sign.append(s)
        if senses[i] == "==":
            slack_col.append(-1)
            slack_coef.append(0.0)
            needs_art.append(True)
            nart += 1
        else:
            c = 1.0
            if senses[i] == ">=":
                c = -1.0
            c = c * s
            slack_col.append(k)
            slack_coef.append(c)
            k += 1
            if c > 0.0:
                needs_art.append(False)
            else:
                needs_art.append(True)
                nart += 1
    N = first_art + nart
    U = []
    for j in range(n):
        U.append(u[j])
    for j in range(n, N):
        U.append(INF)
    T = []
    beta = []
    basis = []
    pos = [-1] * N
    a_col = first_art
    for i in range(m):
        row = [0.0] * N
        Ai = A[i]
        s = sign[i]
        for j in range(n):
            if Ai[j] != 0.0:
                row[j] = s * Ai[j]
        if slack_col[i] >= 0:
            row[slack_col[i]] = slack_coef[i]
        beta.append(s * b[i])
        if needs_art[i]:
            row[a_col] = 1.0
            basis.append(a_col)
            pos[a_col] = i
            a_col += 1
        else:
            basis.append(slack_col[i])
            pos[slack_col[i]] = i
        T.append(row)
    flipped = [False] * N
    cap = 1000 * (m + N)  # a guard only: Bland's rule can need far more than 100 * (m + N)
    if cap < 1000000:
        cap = 1000000
    # the initial rows, for refactorizations (see _reinvert)
    T0 = []
    for i in range(m):
        T0.append(list(T[i]))
    beta0 = list(beta)

    # ---- phase 1
    if nart > 0:
        c1 = []
        for j in range(N):
            if j >= first_art:
                c1.append(1.0)
            else:
                c1.append(0.0)
        d = _reduced_costs(T, basis, c1)

        def refresh1():
            _reinvert(T0, beta0, basis, flipped, U, N, c1, T, beta, d)

        st = _simplex(T, beta, d, basis, pos, U, flipped, N, work, cap, refresh1)
        if st == _LIMIT:
            return _LIMIT, None
        infeas = 0.0
        for i in range(len(T)):
            if basis[i] >= first_art:
                infeas = infeas + beta[i]
        if infeas > EPS * (1.0 + bmax):
            return _INFEASIBLE, None
        # drive the remaining (zero-level) artificials out of the basis
        keep = []
        for i in range(len(T)):
            if basis[i] >= first_art:
                e = -1
                Ti = T[i]
                for j in range(first_art):
                    if pos[j] < 0 and abs(Ti[j]) > EPS:
                        e = j
                        break
                if e >= 0:
                    if work.deadline is not None and time.perf_counter() >= work.deadline:
                        work.reason = "time_limit"
                        return _LIMIT, None
                    beta[i] = 0.0
                    _pivot(T, beta, d, i, e)
                    pos[basis[i]] = -1
                    basis[i] = e
                    pos[e] = i
                    keep.append(True)
                else:
                    keep.append(False)
            else:
                keep.append(True)
        T2 = []
        beta2 = []
        basis2 = []
        for i in range(len(T)):
            if keep[i]:
                row = T[i]
                del row[first_art:]
                T2.append(row)
                beta2.append(beta[i])
                basis2.append(basis[i])
        T = T2
        beta = beta2
        basis = basis2
        pos = [-1] * first_art
        for i in range(len(basis)):
            pos[basis[i]] = i
        del U[first_art:]
        del flipped[first_art:]

    # ---- phase 2
    c2 = []
    for j in range(first_art):
        if j < n:
            cj = cmin[j]
            if flipped[j]:
                cj = -cj
            c2.append(cj)
        else:
            c2.append(0.0)
    d = _reduced_costs(T, basis, c2)
    cbase = []
    for j in range(first_art):
        if j < n:
            cbase.append(cmin[j])
        else:
            cbase.append(0.0)

    def refresh2():
        _reinvert(T0, beta0, basis, flipped, U, first_art, cbase, T, beta, d)

    st = _simplex(T, beta, d, basis, pos, U, flipped, first_art, work, cap, refresh2)
    if st != _OPTIMAL:
        return st, None
    x = []
    for j in range(n):
        y = 0.0
        if pos[j] >= 0:
            y = beta[pos[j]]
        if flipped[j]:
            y = u[j] - y
        x.append(lb[j] + y)
    _check_primal(n, A, senses, rhs, lb, ub, x)
    return _OPTIMAL, x


def _check_primal(n, A, senses, rhs, lb, ub, x):
    """Guard against a numerically broken tableau: raise ``RuntimeError`` rather than report
    a point that violates its bounds or rows by more than ``FEAS_TOL`` (relative to the size
    of the row's terms) as optimal."""
    for j in range(n):
        t = FEAS_TOL * (1.0 + abs(x[j]))
        if x[j] < lb[j] - t or x[j] > ub[j] + t or x[j] != x[j]:
            raise RuntimeError(f"builtin simplex: numerical failure (column {j} = {x[j]!r} is outside "
                               f"[{lb[j]!r}, {ub[j]!r}])")
    for i in range(len(A)):
        Ai = A[i]
        v = 0.0
        size = abs(rhs[i])
        for j in range(n):
            if Ai[j] != 0.0:
                t = Ai[j] * x[j]
                v = v + t
                if abs(t) > size:
                    size = abs(t)
        t = FEAS_TOL * (1.0 + size)
        sn = senses[i]
        if (sn != ">=" and v > rhs[i] + t) or (sn != "<=" and v < rhs[i] - t) or v != v:
            raise RuntimeError(f"builtin simplex: numerical failure (row {i} = {v!r}, {sn} {rhs[i]!r})")


def _objective(obj, x, const):
    z = 0.0
    for j in range(len(obj)):
        if obj[j] != 0.0:
            z = z + obj[j] * x[j]
    return const + z, z


def solve(
    model: LinearModel,
    time_limit: float | None = None,
    mip_gap: float | None = None,
    node_limit: int | None = None,
    work_limit: int | None = None,
    params: dict | None = None,
) -> Solution:
    """Solve ``model`` exactly (up to the tolerances) with the builtin simplex / B&B.

    Args:
        model: the model; all lower bounds must be finite.
        time_limit: wall-clock seconds, checked between nodes (nondeterministic).
        mip_gap: relative gap at which nodes are pruned (default 0: exact).
        node_limit: maximum number of LPs solved (deterministic).
        work_limit: maximum total simplex iterations (deterministic).
        params: optional dict with any of the keys ``time_limit``, ``mip_gap``,
            ``node_limit``, ``work_limit`` (Gurobi-style ``TimeLimit``, ``MIPGap``,
            ``NodeLimit`` are accepted too); explicit arguments take precedence.

    Returns:
        A :class:`~mppflow.lp.linear.Solution` (``backend='builtin'``). ``node_count`` is the
        number of LPs solved, ``iterations`` the simplex iterations.
    """
    p = dict(params or {})
    if time_limit is None:
        time_limit = p.get("time_limit", p.get("TimeLimit"))
    if mip_gap is None:
        mip_gap = p.get("mip_gap", p.get("MIPGap"))
    if node_limit is None:
        node_limit = p.get("node_limit", p.get("NodeLimit"))
    if work_limit is None:
        work_limit = p.get("work_limit")
    t0 = time.perf_counter()

    n = model.num_columns
    m = model.num_rows
    obj = model.obj
    const = model.obj_constant
    maximize = model.sense == "max"
    cmin = []
    for j in range(n):
        if maximize:
            cmin.append(-obj[j])
        else:
            cmin.append(obj[j])
    lb0 = []
    ub0 = []
    is_int = []
    vt = model.vtypes
    mlb = model.lb
    mub = model.ub
    for j in range(n):
        lo = mlb[j]
        hi = mub[j]
        if lo == -INF or lo != lo:
            raise ValueError(f"builtin backend: column {j} needs a finite lower bound")
        if vt[j] == "B":
            if lo < 0.0:
                lo = 0.0
            if hi > 1.0:
                hi = 1.0
        if vt[j] != "C":
            lo = _ceil(lo - INT_EPS)
            if hi != INF:
                hi = _floor(hi + INT_EPS)
            is_int.append(True)
        else:
            is_int.append(False)
        lb0.append(lo)
        ub0.append(hi)
    indptr, indices, data = model.csr()
    A = []
    senses = []
    rhs = []
    for i in range(m):
        row = [0.0] * n
        for k in range(indptr[i], indptr[i + 1]):
            row[indices[k]] = row[indices[k]] + data[k]
        A.append(row)
        senses.append(model.row_sense(i))
        rhs.append(model.row_rhs(i))

    work = _Work(work_limit, None if time_limit is None else t0 + time_limit)
    gap = 0.0 if mip_gap is None else float(mip_gap)
    inc_x = None
    inc_obj = None
    inc_z = INF  # incumbent in minimization space (no constant)
    sol_count = 0
    nodes = 0
    stack = [(lb0, ub0, -INF)]
    stop = None
    open_bounds = []
    # best (minimization-space) bound of the nodes pruned by the mip_gap part of the
    # tolerance: their subtrees may hold better solutions, so they cap the reported bound
    pruned = [INF]

    def tol():
        t = EPS * (1.0 + abs(inc_z))
        g = gap * abs(inc_obj)
        if g > t:
            t = g
        return t

    def prune(v):
        """True if a node with bound ``v`` cannot improve the incumbent enough."""
        if v >= inc_z - tol():
            if v < inc_z - EPS * (1.0 + abs(inc_z)) and v < pruned[0]:
                pruned[0] = v
            return True
        return False

    while len(stack) > 0:
        if node_limit is not None and nodes >= node_limit:
            stop = "node_limit"
            break
        if time_limit is not None and time.perf_counter() - t0 >= time_limit:
            stop = "time_limit"
            break
        lbn, ubn, pb = stack.pop()
        if inc_x is not None and prune(pb):
            continue
        nodes += 1
        st, x = _solve_lp(n, A, senses, rhs, lbn, ubn, cmin, work)
        if st == _LIMIT:
            stop = work.reason
            open_bounds.append(pb)
            break
        if st == _INFEASIBLE:
            continue
        if st == _UNBOUNDED:
            return Solution(status="unbounded", backend="builtin", raw_status="unbounded",
                            runtime=time.perf_counter() - t0, node_count=nodes,
                            iterations=work.iterations)
        objv, z = _objective(obj, x, const)
        zmin = z
        if maximize:
            zmin = -z
        if inc_x is not None and prune(zmin):
            continue
        br = -1
        for j in range(n):
            if is_int[j]:
                v = x[j]
                f = v - _floor(v)
                if f > INT_EPS and f < 1.0 - INT_EPS:
                    br = j
                    break
        if br < 0:
            for j in range(n):
                if is_int[j]:
                    x[j] = _floor(x[j] + 0.5)
            objv, z = _objective(obj, x, const)
            zmin = z
            if maximize:
                zmin = -z
            inc_x = x
            inc_obj = objv
            inc_z = zmin
            sol_count += 1
            continue
        fl = _floor(x[br])
        if fl >= lbn[br]:
            ub_f = list(ubn)
            ub_f[br] = fl
            stack.append((lbn, ub_f, zmin))
        if fl + 1.0 <= ubn[br]:
            lb_c = list(lbn)
            lb_c[br] = fl + 1.0
            stack.append((lb_c, ubn, zmin))

    runtime = time.perf_counter() - t0
    if stop is None:
        if inc_x is None:
            return Solution(status="infeasible", backend="builtin", raw_status="infeasible",
                            runtime=runtime, node_count=nodes, iterations=work.iterations)
        bound = inc_obj
        g = 0.0
        if pruned[0] < inc_z:
            bound = const + (-pruned[0] if maximize else pruned[0])
            g = relative_gap(inc_obj, bound)
        return Solution(status="optimal", objective=inc_obj, x=inc_x,
                        mip_gap=g, bound=bound, sol_count=sol_count, backend="builtin",
                        raw_status="optimal", runtime=runtime, node_count=nodes,
                        iterations=work.iterations)
    # stopped at a limit: bound = best of the open nodes, the gap-pruned nodes and the
    # incumbent
    bmin = inc_z
    if pruned[0] < bmin:
        bmin = pruned[0]
    for item in stack:
        if item[2] < bmin:
            bmin = item[2]
    for v in open_bounds:
        if v < bmin:
            bmin = v
    bound = const + (-bmin if maximize else bmin)
    status = "node_limit" if stop == "node_limit" else "time_limit"
    g = INF if inc_x is None else relative_gap(inc_obj, bound)
    return Solution(status=status, objective=inc_obj, x=inc_x, mip_gap=g, bound=bound,
                    sol_count=sol_count, backend="builtin", raw_status=stop, runtime=runtime,
                    node_count=nodes, iterations=work.iterations)
