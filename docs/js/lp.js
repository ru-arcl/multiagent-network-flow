/*
 * lp.js -- exact MILP solver for small models: a line-by-line port of mppflow/lp/builtin.py.
 *
 * Dense two-phase bounded-variable primal simplex (Bland's rule, periodic refactorization)
 * and depth-first branch and bound. Every floating-point operation is performed in the same
 * order as in the Python code, with the same IEEE doubles, so both give IDENTICAL results:
 * the same pivots, iteration and node counts, and bit-identical solution vectors (checked by
 * tests/parity/lp.json and `npm test`).
 *
 * MPP.lp.solve(model, opts) -> {status, objective, x, mipGap, bound, solCount, backend,
 *   rawStatus, runtime, nodeCount, iterations}
 *   model: an MPP.LinearModel (or an MPP.MPPModel: its .model is used)
 *   opts: {mipGap, nodeLimit, workLimit, timeLimit (seconds), deadlineMs (absolute, Date.now()),
 *          maxEntries}
 *   maxEntries (JavaScript only, for the browser demo): refuse a model whose dense matrices would hold
 *   more than this many numbers (MPP.lp.denseEntries), throwing an error named 'ModelTooLarge' before
 *   allocating anything. The dense constraint matrix and simplex tableaux of a large model need
 *   gigabytes, and a browser tab that runs out of memory crashes instead of reaching its time limit.
 *   Without it (the default) nothing changes, so the parity with mppflow/lp/builtin.py is untouched.
 *
 * Classic script: attaches MPP.lp to window.MPP (browser) or exports it (Node).
 */
(function mppLpFactory(root) {
  'use strict';

  var EPS = 1e-9;
  var INT_EPS = 1e-6;
  var REFRESH = 100;
  var ZERO_TOL = 1e-11;
  var TIE_TOL = 1e-11;
  var FEAS_TOL = 1e-6;
  var INF = Infinity;

  var OPTIMAL = 0, INFEASIBLE = 1, UNBOUNDED = 2, LIMIT = 3;

  function nowMs() {
    return (typeof performance !== 'undefined' && performance.now && performance.timeOrigin)
      ? performance.timeOrigin + performance.now() : Date.now();
  }

  function numericalFailure(msg) {
    var e = new Error('builtin simplex: ' + msg);
    e.name = 'RuntimeError';
    return e;
  }

  function floor(x) {
    var f = Math.trunc(x);
    if (f > x) f = f - 1.0;
    return f;
  }

  function ceil(x) { return -floor(-x); }

  function relativeGap(objective, bound) {
    var d = Math.abs(bound - objective);
    if (objective === 0.0) return d === 0.0 ? 0.0 : INF;
    return d / Math.abs(objective);
  }

  function Work(limit, deadline) {
    this.iterations = 0;
    this.limit = limit;
    this.deadline = deadline;
    this.reason = null;
  }
  Work.prototype.exhausted = function () {
    if (this.limit !== null && this.iterations >= this.limit) { this.reason = 'work_limit'; return true; }
    if (this.deadline !== null && nowMs() >= this.deadline) { this.reason = 'time_limit'; return true; }
    return false;
  };

  function pivot(T, beta, d, r, e) {
    var row = T[r];
    var p = row[e];
    var n = row.length;
    var nz = [];
    var k;
    for (k = 0; k < n; k++) {
      if (row[k] !== 0.0) {
        row[k] = row[k] / p;
        nz.push(k);
      }
    }
    beta[r] = beta[r] / p;
    row[e] = 1.0;
    var br = beta[r];
    var m = T.length;
    var nzl = nz.length;
    for (var i = 0; i < m; i++) {
      if (i === r) continue;
      var Ti = T[i];
      var f = Ti[e];
      if (f !== 0.0) {
        for (var q = 0; q < nzl; q++) {
          k = nz[q];
          Ti[k] = Ti[k] - f * row[k];
        }
        beta[i] = beta[i] - f * br;
        Ti[e] = 0.0;
      }
    }
    var fd = d[e];
    if (fd !== 0.0) {
      for (var q2 = 0; q2 < nzl; q2++) {
        k = nz[q2];
        d[k] = d[k] - fd * row[k];
      }
      d[e] = 0.0;
    }
  }

  function flip(T, beta, d, e, u) {
    var m = T.length;
    for (var i = 0; i < m; i++) {
      var a = T[i][e];
      if (a !== 0.0) {
        beta[i] = beta[i] - a * u;
        T[i][e] = -a;
      }
    }
    d[e] = -d[e];
  }

  function reducedCosts(T, basis, c) {
    var n = c.length;
    var d = [];
    for (var j = 0; j < n; j++) d.push(c[j]);
    var m = T.length;
    for (var i = 0; i < m; i++) {
      var cb = c[basis[i]];
      if (cb !== 0.0) {
        var row = T[i];
        for (var k = 0; k < n; k++) {
          if (row[k] !== 0.0) d[k] = d[k] - cb * row[k];
        }
      }
    }
    return d;
  }

  function replaceArray(dst, src) {
    dst.length = 0;
    for (var i = 0; i < src.length; i++) dst.push(src[i]);
  }

  function reinvert(T0, beta0, basis, flipped, U, ncols, cost, T, beta, d) {
    var m0 = T0.length;
    var M = [];
    var b = [];
    var r, j;
    for (r = 0; r < m0; r++) {
      var src = T0[r];
      var row = [];
      var br = beta0[r];
      for (j = 0; j < ncols; j++) {
        var a = src[j];
        if (a !== 0.0 && flipped[j]) {
          br = br - a * U[j];
          a = -a;
        }
        row.push(a);
      }
      M.push(row);
      b.push(br);
    }
    var mrows = basis.length;
    var nnz = [];
    var i;
    for (i = 0; i < mrows; i++) {
      var k = basis[i];
      var c = 0;
      for (r = 0; r < m0; r++) if (M[r][k] !== 0.0) c += 1;
      nnz.push(c);
    }
    var order = [];
    for (i = 0; i < mrows; i++) order.push(i);
    order.sort(function (p, q) { return (nnz[p] - nnz[q]) || (p - q); });
    var used = new Array(m0).fill(false);
    var prow = new Array(mrows).fill(-1);
    for (var oi = 0; oi < order.length; oi++) {
      i = order[oi];
      var kk = basis[i];
      var best = -1;
      var bv = 0.0;
      for (r = 0; r < m0; r++) {
        if (!used[r]) {
          var av = Math.abs(M[r][kk]);
          if (av > bv) {
            bv = av;
            best = r;
          }
        }
      }
      if (best < 0 || bv <= EPS) return false;
      used[best] = true;
      prow[i] = best;
      var prw = M[best];
      var p = prw[kk];
      var nz = [];
      for (j = 0; j < ncols; j++) {
        if (prw[j] !== 0.0) {
          prw[j] = prw[j] / p;
          nz.push(j);
        }
      }
      b[best] = b[best] / p;
      prw[kk] = 1.0;
      var bb = b[best];
      var nzl = nz.length;
      for (r = 0; r < m0; r++) {
        if (r === best) continue;
        var Mr = M[r];
        var f = Mr[kk];
        if (f !== 0.0) {
          for (var q = 0; q < nzl; q++) {
            j = nz[q];
            Mr[j] = Mr[j] - f * prw[j];
          }
          b[r] = b[r] - f * bb;
          Mr[kk] = 0.0;
        }
      }
    }
    var newT = [];
    var newb = [];
    for (i = 0; i < mrows; i++) {
      newT.push(M[prow[i]]);
      newb.push(b[prow[i]]);
    }
    var cc = [];
    for (j = 0; j < ncols; j++) {
      var cj = cost[j];
      if (flipped[j]) cj = -cj;
      cc.push(cj);
    }
    replaceArray(T, newT);
    replaceArray(beta, newb);
    replaceArray(d, reducedCosts(T, basis, cc));
    return true;
  }

  function simplex(T, beta, d, basis, pos, U, flipped, ncols, work, cap, refresh) {
    var m = T.length;
    var count = 0;
    var since = 0;
    for (;;) {
      if (refresh && since >= REFRESH) {
        refresh();
        since = 0;
      }
      var e = -1;
      var j;
      for (j = 0; j < ncols; j++) {
        if (pos[j] < 0 && U[j] > 0.0 && d[j] < -EPS) {
          e = j;
          break;
        }
      }
      if (e < 0) {
        if (refresh && count >= REFRESH && since > 0) {
          refresh();
          since = 0;
          continue;
        }
        return OPTIMAL;
      }
      if (work.exhausted()) return LIMIT;
      count += 1;
      if (count > cap) {
        // Bland's rule stalled (degenerate pivots, no cycle): stop as at a work limit (builtin.py)
        work.reason = 'iteration_cap';
        return LIMIT;
      }
      since += 1;
      work.iterations += 1;
      var bestT = U[e];
      var bestVar = e;
      var bestRow = -1;
      var bestUp = false;
      for (var i = 0; i < m; i++) {
        var a = T[i][e];
        var num, t, up;
        if (a > EPS) {
          num = beta[i];
          if (num < ZERO_TOL) num = 0.0;
          t = num / a;
          up = false;
        } else if (a < -EPS) {
          var ubv = U[basis[i]];
          if (ubv === INF) continue;
          num = ubv - beta[i];
          if (num < ZERO_TOL) num = 0.0;
          t = num / (-a);
          up = true;
        } else continue;
        var k = basis[i];
        var better;
        if (bestT === INF) better = true;
        else {
          var tt = TIE_TOL * (1.0 + bestT);
          better = t < bestT - tt || (t <= bestT + tt && k < bestVar);
        }
        if (better) {
          bestT = t;
          bestVar = k;
          bestRow = i;
          bestUp = up;
        }
      }
      if (bestRow < 0) {
        if (bestT === INF) return UNBOUNDED;
        flip(T, beta, d, e, U[e]);
        flipped[e] = !flipped[e];
        continue;
      }
      var r = bestRow;
      if (bestUp) {
        var lv = basis[r];
        T[r][lv] = -1.0;
        beta[r] = beta[r] - U[lv];
        flipped[lv] = !flipped[lv];
      }
      pivot(T, beta, d, r, e);
      pos[basis[r]] = -1;
      basis[r] = e;
      pos[e] = r;
    }
  }

  function checkPrimal(n, A, senses, rhs, lb, ub, x) {
    var j, t;
    for (j = 0; j < n; j++) {
      t = FEAS_TOL * (1.0 + Math.abs(x[j]));
      if (x[j] < lb[j] - t || x[j] > ub[j] + t || x[j] !== x[j]) {
        throw numericalFailure('numerical failure (column ' + j + ' = ' + x[j] + ' is outside [' + lb[j] + ', ' + ub[j] + '])');
      }
    }
    for (var i = 0; i < A.length; i++) {
      var Ai = A[i];
      var v = 0.0;
      var size = Math.abs(rhs[i]);
      for (j = 0; j < n; j++) {
        if (Ai[j] !== 0.0) {
          t = Ai[j] * x[j];
          v = v + t;
          if (Math.abs(t) > size) size = Math.abs(t);
        }
      }
      t = FEAS_TOL * (1.0 + size);
      var sn = senses[i];
      if ((sn !== '>=' && v > rhs[i] + t) || (sn !== '<=' && v < rhs[i] - t) || v !== v) {
        throw numericalFailure('numerical failure (row ' + i + ' = ' + v + ', ' + sn + ' ' + rhs[i] + ')');
      }
    }
  }

  function solveLp(n, A, senses, rhs, lb, ub, cmin, work) {
    var m = A.length;
    var u = [];
    var j, i;
    for (j = 0; j < n; j++) {
      var uj = ub[j] - lb[j];
      if (uj < 0.0) return [INFEASIBLE, null];
      u.push(uj);
    }
    var b = [];
    var bmax = 0.0;
    for (i = 0; i < m; i++) {
      var bi = rhs[i];
      var Ai = A[i];
      for (j = 0; j < n; j++) {
        if (Ai[j] !== 0.0 && lb[j] !== 0.0) bi = bi - Ai[j] * lb[j];
      }
      b.push(bi);
      if (Math.abs(bi) > bmax) bmax = Math.abs(bi);
    }
    var nslack = 0;
    for (i = 0; i < m; i++) if (senses[i] !== '==') nslack += 1;
    var firstArt = n + nslack;
    var slackCol = [];
    var slackCoef = [];
    var sign = [];
    var needsArt = [];
    var k = n;
    var nart = 0;
    for (i = 0; i < m; i++) {
      var s = 1.0;
      if (b[i] < 0.0) s = -1.0;
      sign.push(s);
      if (senses[i] === '==') {
        slackCol.push(-1);
        slackCoef.push(0.0);
        needsArt.push(true);
        nart += 1;
      } else {
        var c = 1.0;
        if (senses[i] === '>=') c = -1.0;
        c = c * s;
        slackCol.push(k);
        slackCoef.push(c);
        k += 1;
        if (c > 0.0) needsArt.push(false);
        else {
          needsArt.push(true);
          nart += 1;
        }
      }
    }
    var N = firstArt + nart;
    var U = [];
    for (j = 0; j < n; j++) U.push(u[j]);
    for (j = n; j < N; j++) U.push(INF);
    var T = [];
    var beta = [];
    var basis = [];
    var pos = new Array(N).fill(-1);
    var aCol = firstArt;
    for (i = 0; i < m; i++) {
      var row = new Array(N).fill(0.0);
      var Ar = A[i];
      var sg = sign[i];
      for (j = 0; j < n; j++) {
        if (Ar[j] !== 0.0) row[j] = sg * Ar[j];
      }
      if (slackCol[i] >= 0) row[slackCol[i]] = slackCoef[i];
      beta.push(sg * b[i]);
      if (needsArt[i]) {
        row[aCol] = 1.0;
        basis.push(aCol);
        pos[aCol] = i;
        aCol += 1;
      } else {
        basis.push(slackCol[i]);
        pos[slackCol[i]] = i;
      }
      T.push(row);
    }
    var flipped = new Array(N).fill(false);
    var cap = 1000 * (m + N);  // a guard only: Bland's rule can need far more than 100 * (m + N)
    if (cap < 1000000) cap = 1000000;
    var T0 = [];
    for (i = 0; i < m; i++) T0.push(T[i].slice());
    var beta0 = beta.slice();
    var d, st;

    // ---- phase 1
    if (nart > 0) {
      var c1 = [];
      for (j = 0; j < N; j++) {
        if (j >= firstArt) c1.push(1.0);
        else c1.push(0.0);
      }
      d = reducedCosts(T, basis, c1);
      var refresh1 = function () {
        reinvert(T0, beta0, basis, flipped, U, N, c1, T, beta, d);
      };
      st = simplex(T, beta, d, basis, pos, U, flipped, N, work, cap, refresh1);
      if (st === LIMIT) return [LIMIT, null];
      var infeas = 0.0;
      for (i = 0; i < T.length; i++) {
        if (basis[i] >= firstArt) infeas = infeas + beta[i];
      }
      if (infeas > EPS * (1.0 + bmax)) return [INFEASIBLE, null];
      var keep = [];
      for (i = 0; i < T.length; i++) {
        if (basis[i] >= firstArt) {
          var e = -1;
          var Ti = T[i];
          for (j = 0; j < firstArt; j++) {
            if (pos[j] < 0 && Math.abs(Ti[j]) > EPS) {
              e = j;
              break;
            }
          }
          if (e >= 0) {
            if (work.deadline !== null && nowMs() >= work.deadline) {
              work.reason = 'time_limit';
              return [LIMIT, null];
            }
            beta[i] = 0.0;
            pivot(T, beta, d, i, e);
            pos[basis[i]] = -1;
            basis[i] = e;
            pos[e] = i;
            keep.push(true);
          } else keep.push(false);
        } else keep.push(true);
      }
      var T2 = [];
      var beta2 = [];
      var basis2 = [];
      for (i = 0; i < T.length; i++) {
        if (keep[i]) {
          var rw = T[i];
          rw.length = firstArt;
          T2.push(rw);
          beta2.push(beta[i]);
          basis2.push(basis[i]);
        }
      }
      T = T2;
      beta = beta2;
      basis = basis2;
      pos = new Array(firstArt).fill(-1);
      for (i = 0; i < basis.length; i++) pos[basis[i]] = i;
      U.length = firstArt;
      flipped.length = firstArt;
    }

    // ---- phase 2
    var c2 = [];
    for (j = 0; j < firstArt; j++) {
      if (j < n) {
        var cj = cmin[j];
        if (flipped[j]) cj = -cj;
        c2.push(cj);
      } else c2.push(0.0);
    }
    d = reducedCosts(T, basis, c2);
    var cbase = [];
    for (j = 0; j < firstArt; j++) {
      if (j < n) cbase.push(cmin[j]);
      else cbase.push(0.0);
    }
    var refresh2 = function () {
      reinvert(T0, beta0, basis, flipped, U, firstArt, cbase, T, beta, d);
    };
    st = simplex(T, beta, d, basis, pos, U, flipped, firstArt, work, cap, refresh2);
    if (st !== OPTIMAL) return [st, null];
    var x = [];
    for (j = 0; j < n; j++) {
      var y = 0.0;
      if (pos[j] >= 0) y = beta[pos[j]];
      if (flipped[j]) y = u[j] - y;
      x.push(lb[j] + y);
    }
    checkPrimal(n, A, senses, rhs, lb, ub, x);
    return [OPTIMAL, x];
  }

  function objectiveOf(obj, x, cnst) {
    var z = 0.0;
    for (var j = 0; j < obj.length; j++) {
      if (obj[j] !== 0.0) z = z + obj[j] * x[j];
    }
    return [cnst + z, z];
  }

  function opt(o, k) { return (o && o[k] !== undefined && o[k] !== null) ? o[k] : null; }

  /** An upper bound on the numbers that solve() keeps in dense arrays for a model with n columns and m rows:
   *  the constraint matrix (m x n) and, per LP, the tableau and its copy (m x N each, N <= n + 2m columns
   *  with the slack and artificial columns). */
  function denseEntries(n, m) { return m * n + 2 * m * (n + 2 * m); }

  function modelTooLarge(n, m, entries, limit) {
    var e = new Error('the model has ' + n + ' columns and ' + m + ' rows, so the dense simplex would keep about ' +
                      Math.round(entries / 1e6) + ' million numbers (' + (entries * 8 / 1e9).toFixed(1) +
                      ' GB) in memory; the limit here is ' + Math.round(limit / 1e6) + ' million');
    e.name = 'ModelTooLarge';
    e.columns = n; e.rows = m;
    return e;
  }

  /** builtin.solve: see the header comment. */
  function solve(model, opts) {
    if (model && model.model && model.model.lb) model = model.model;
    var timeLimit = opt(opts, 'timeLimit');
    var mipGap = opt(opts, 'mipGap');
    var nodeLimit = opt(opts, 'nodeLimit');
    var workLimit = opt(opts, 'workLimit');
    var deadlineMs = opt(opts, 'deadlineMs');
    var t0 = nowMs();
    var deadline = null;
    if (timeLimit !== null) deadline = t0 + timeLimit * 1000;
    if (deadlineMs !== null && (deadline === null || deadlineMs < deadline)) deadline = deadlineMs;
    function runtime() { return (nowMs() - t0) / 1000; }
    function timeUp() { return deadline !== null && nowMs() >= deadline; }

    var n = model.lb.length;
    var m = model.rhs.length;
    var maxEntries = opt(opts, 'maxEntries');
    if (maxEntries !== null && denseEntries(n, m) > maxEntries) throw modelTooLarge(n, m, denseEntries(n, m), maxEntries);
    var obj = model.obj;
    var cnst = model.objConstant;
    var maximize = model.sense === 'max';
    var cmin = [];
    var j, i;
    for (j = 0; j < n; j++) {
      if (maximize) cmin.push(-obj[j]);
      else cmin.push(obj[j]);
    }
    var lb0 = [];
    var ub0 = [];
    var isInt = [];
    var vt = model.vtype;
    for (j = 0; j < n; j++) {
      var lo = model.lb[j];
      var hi = model.ub[j];
      if (lo === -INF || lo !== lo) throw new Error('builtin backend: column ' + j + ' needs a finite lower bound');
      if (vt[j] === 'B') {
        if (lo < 0.0) lo = 0.0;
        if (hi > 1.0) hi = 1.0;
      }
      if (vt[j] !== 'C') {
        lo = ceil(lo - INT_EPS);
        if (hi !== INF) hi = floor(hi + INT_EPS);
        isInt.push(true);
      } else isInt.push(false);
      lb0.push(lo);
      ub0.push(hi);
    }
    var A = [];
    var senses = [];
    var rhs = [];
    for (i = 0; i < m; i++) {
      var row = new Array(n).fill(0.0);
      for (var k = model.rowStart[i]; k < model.rowStart[i + 1]; k++) {
        var c = model.termCol[k];
        row[c] = row[c] + model.termCoef[k];
      }
      A.push(row);
      senses.push(model.rowSense[i]);
      rhs.push(model.rhs[i]);
    }

    var work = new Work(workLimit, deadline);
    var gap = mipGap === null ? 0.0 : +mipGap;
    var incX = null;
    var incObj = null;
    var incZ = INF;
    var solCount = 0;
    var nodes = 0;
    var stack = [[lb0, ub0, -INF]];
    var stop = null;
    var openBounds = [];
    var pruned = INF;

    function tol() {
      var t = EPS * (1.0 + Math.abs(incZ));
      var g = gap * Math.abs(incObj);
      if (g > t) t = g;
      return t;
    }

    function prune(v) {
      if (v >= incZ - tol()) {
        if (v < incZ - EPS * (1.0 + Math.abs(incZ)) && v < pruned) pruned = v;
        return true;
      }
      return false;
    }

    while (stack.length > 0) {
      if (nodeLimit !== null && nodes >= nodeLimit) {
        stop = 'node_limit';
        break;
      }
      if (timeUp()) {
        stop = 'time_limit';
        break;
      }
      var item = stack.pop();
      var lbn = item[0], ubn = item[1], pb = item[2];
      if (incX !== null && prune(pb)) continue;
      nodes += 1;
      var res = solveLp(n, A, senses, rhs, lbn, ubn, cmin, work);
      var st = res[0], x = res[1];
      if (st === LIMIT) {
        stop = work.reason;
        openBounds.push(pb);
        break;
      }
      if (st === INFEASIBLE) continue;
      if (st === UNBOUNDED) {
        return { status: 'unbounded', objective: null, x: null, mipGap: null, bound: null, solCount: 0,
                 backend: 'builtin', rawStatus: 'unbounded', runtime: runtime(), nodeCount: nodes,
                 iterations: work.iterations };
      }
      var oz = objectiveOf(obj, x, cnst);
      var objv = oz[0], z = oz[1];
      var zmin = z;
      if (maximize) zmin = -z;
      if (incX !== null && prune(zmin)) continue;
      var br = -1;
      for (j = 0; j < n; j++) {
        if (isInt[j]) {
          var v = x[j];
          var f = v - floor(v);
          if (f > INT_EPS && f < 1.0 - INT_EPS) {
            br = j;
            break;
          }
        }
      }
      if (br < 0) {
        for (j = 0; j < n; j++) {
          if (isInt[j]) x[j] = floor(x[j] + 0.5);
        }
        oz = objectiveOf(obj, x, cnst);
        objv = oz[0]; z = oz[1];
        zmin = z;
        if (maximize) zmin = -z;
        incX = x;
        incObj = objv;
        incZ = zmin;
        solCount += 1;
        continue;
      }
      var fl = floor(x[br]);
      if (fl >= lbn[br]) {
        var ubF = ubn.slice();
        ubF[br] = fl;
        stack.push([lbn, ubF, zmin]);
      }
      if (fl + 1.0 <= ubn[br]) {
        var lbC = lbn.slice();
        lbC[br] = fl + 1.0;
        stack.push([lbC, ubn, zmin]);
      }
    }

    var rt = runtime();
    if (stop === null) {
      if (incX === null) {
        return { status: 'infeasible', objective: null, x: null, mipGap: null, bound: null, solCount: 0,
                 backend: 'builtin', rawStatus: 'infeasible', runtime: rt, nodeCount: nodes,
                 iterations: work.iterations };
      }
      var bound = incObj;
      var g0 = 0.0;
      if (pruned < incZ) {
        bound = cnst + (maximize ? -pruned : pruned);
        g0 = relativeGap(incObj, bound);
      }
      return { status: 'optimal', objective: incObj, x: incX, mipGap: g0, bound: bound, solCount: solCount,
               backend: 'builtin', rawStatus: 'optimal', runtime: rt, nodeCount: nodes,
               iterations: work.iterations };
    }
    var bmin = incZ;
    if (pruned < bmin) bmin = pruned;
    for (i = 0; i < stack.length; i++) if (stack[i][2] < bmin) bmin = stack[i][2];
    for (i = 0; i < openBounds.length; i++) if (openBounds[i] < bmin) bmin = openBounds[i];
    var bnd = cnst + (maximize ? -bmin : bmin);
    var status = stop === 'node_limit' ? 'node_limit' : 'time_limit';
    var gg = incX === null ? INF : relativeGap(incObj, bnd);
    return { status: status, objective: incObj, x: incX, mipGap: gg, bound: bnd, solCount: solCount,
             backend: 'builtin', rawStatus: stop, runtime: rt, nodeCount: nodes, iterations: work.iterations };
  }

  var lp = { solve: solve, EPS: EPS, INT_EPS: INT_EPS, REFRESH: REFRESH, ZERO_TOL: ZERO_TOL,
             TIE_TOL: TIE_TOL, FEAS_TOL: FEAS_TOL, relativeGap: relativeGap, denseEntries: denseEntries,
             /** This file's program text, for building a Web Worker from a Blob (MPP.workerSource). */
             source: '(' + mppLpFactory.toString() + ')(self);\n' };

  if (root) {
    root.MPP = root.MPP || {};
    root.MPP.lp = lp;
  }
  if (typeof module !== 'undefined' && module.exports) module.exports = lp;
})(typeof window !== 'undefined' ? window : (typeof self !== 'undefined' ? self : (typeof globalThis !== 'undefined' ? globalThis : this)));
