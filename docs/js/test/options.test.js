/*
 * Engine options that only the browser demo uses (no Python counterpart, so no parity fixture):
 * planSplit's timeLimitPerPiece: false, the demo's "time limit covers the whole split".
 * Also the time-limit statuses of the solvers, which the demo's time limit reaches and the parity
 * fixtures (no time limits: wall-clock results are not reproducible) do not: driven here by a
 * stubbed MPP.lp.solve and a shifted performance.now, as tests/test_solvers.py does in Python.
 *
 * Run: npm test   (node --test docs/js/test/; Node >= 18, no dependencies)
 */
'use strict';

const test = require('node:test');
const assert = require('node:assert');
const path = require('node:path');

const MPP = require(path.join(__dirname, '..', 'engine.js'));

// Fig. 8 of T-RO 2016: two robots on a 4 x 4 grid (tests/data/paper/tor16_fig8_split.json)
const g = MPP.grid(4, 4), starts = [13, 8], goals = [0, 7];

test('planSplit timeLimitPerPiece: false gives the same plan when the limit does not bind', () => {
  const a = MPP.planSplit(g, starts, goals, 2, { rng: new MPP.MathRandom(1) });
  const b = MPP.planSplit(g, starts, goals, 2, { rng: new MPP.MathRandom(1), timeLimit: 600, timeLimitPerPiece: false });
  assert.deepStrictEqual(b.paths, a.paths);
  assert.deepStrictEqual(b.stats.horizons, a.stats.horizons);
  assert.strictEqual(b.status, 'feasible');
  assert.strictEqual(b.stats.skipped_pieces, 0);
  assert.strictEqual(a.stats.skipped_pieces, undefined, 'the mppflow-compatible default reports no skipped pieces');
});

test('planSplit timeLimitPerPiece: false stops once the whole limit is used up', () => {
  // 1 ns: gone before the first piece (splitting the goals alone takes longer)
  const r = MPP.planSplit(g, starts, goals, 4, { rng: new MPP.MathRandom(1), timeLimit: 1e-9, timeLimitPerPiece: false });
  assert.strictEqual(r.paths, null);
  assert.strictEqual(r.status, 'time_limit');
  assert.ok(r.stats.skipped_pieces >= 3, 'pieces skipped: ' + r.stats.skipped_pieces);
  assert.ok(r.stats.seconds < 5, 'took ' + r.stats.seconds + ' s');
});

test('planSplit timeLimitPerPiece: false stops at the first piece without a plan, with time left', () => {
  // the 4-cycle 0-1-3-2-0 of tests/test_cli.py: pieces 1 and 3 of the 4-way split have no plan
  const c4 = MPP.fromAdjacency([[1, 2], [0, 3], [0, 3], [1, 2]]), s = [0, 1, 3], t = [3, 2, 1], k = 4;
  const all = MPP.planSplit(c4, s, t, k, { rng: new MPP.MathRandom(1), timeLimit: 600 });
  assert.deepStrictEqual(all.failures.map((f) => f.split(':')[0]), ['piece 1', 'piece 3']);
  assert.strictEqual(all.stats.skipped_pieces, undefined);
  const r = MPP.planSplit(c4, s, t, k, { rng: new MPP.MathRandom(1), timeLimit: 600, timeLimitPerPiece: false });
  assert.strictEqual(r.status, 'failed');
  assert.strictEqual(r.paths, null);
  assert.deepStrictEqual(r.failures, all.failures.slice(0, 1));
  assert.strictEqual(r.stats.skipped_pieces, k - 1 - 1, 'the pieces after piece 1 are not attempted');
  assert.strictEqual(r.stats.solves, all.stats.solves - 1);
});

// ------------------------------------------------------------------ time-limit statuses
const realSolve = MPP.lp.solve;
const TIMED_OUT = { status: 'time_limit', objective: null, x: null, bound: null, mipGap: null, solCount: 0,
                    backend: 'builtin', runtime: 0 };

// Run f with MPP.lp.solve replaced: solve call k (1-based) is answered by stub(k, model, opts), which
// returns a result or undefined (then the real solver runs); restored afterwards.
function withSolve(stub, f) {
  let k = 0;
  MPP.lp.solve = function (m, o) {
    k += 1;
    const r = stub(k, m, o);
    return r === undefined ? realSolve(m, o) : r;
  };
  try { return f(); } finally { MPP.lp.solve = realSolve; }
}

// Shift the engine's clock (performance.now) by `seconds` from now on; returns the undo.
// defineProperty, not assignment: Node 18's Performance.prototype.now is read-only
// (non-writable), so `performance.now = ...` throws there in strict mode.
function shiftClock() {
  const own = Object.getOwnPropertyDescriptor(performance, 'now');
  const real = performance.now.bind(performance);
  let offset = 0;
  Object.defineProperty(performance, 'now',
    { value: () => real() + offset * 1000, configurable: true, writable: true });
  return {
    by(seconds) { offset += seconds; },
    undo() { if (own) Object.defineProperty(performance, 'now', own); else delete performance.now; },
  };
}

// T given (= the optimal makespan): solve 1 is the model at T0, solve 2 the one at T_hi. Both
// instances have more than JOINT_SEARCH_MAX_CONFIGS configurations, so no joint-state search.
const G44 = MPP.grid(4, 4), S44 = [11, 1, 0, 12, 4, 10], T44 = [12, 0, 13, 15, 2, 14];   // T_opt 4
const G26 = MPP.grid(2, 6), S26 = [8, 4, 2, 3, 11, 0, 10], T26 = [11, 10, 9, 3, 2, 4, 6];  // T_opt 5
const SEARCH = [
  ['minTotalTime', G44, S44, T44, 4, 18, 16, 6],         // [fn, g, s, t, T, value at T0, lower bound, T_hi]
  ['minTotalDistance', G44, S44, T44, 4, 18, 16, 18],
  ['minMaxDistance', G26, S26, T26, 5, 5, 4, 35],
];

for (const [fn, gg, s, t, T, val, lb, Thi] of SEARCH) {
  test(fn + ': a time limit hit by the solve at T_hi gives "feasible", not "optimal"', () => {
    const r = withSolve((k) => (k >= 2 ? TIMED_OUT : undefined), () => MPP[fn](gg, s, t, { T: T, timeLimit: 1000 }));
    assert.strictEqual(r.status, 'feasible');
    assert.strictEqual(r.objective, val);
    assert.strictEqual(r.lowerBound, lb);
    assert.strictEqual(r.extra.time_limit_at_T_hi, true);
    assert.deepStrictEqual(r.extra.horizon_search, { T0: T, T_hi: Thi, T_opt: null });
    assert.deepStrictEqual(r.attempts.map((a) => [a.T, a.status]), [[T, 'optimal'], [Thi, 'time_limit']]);
  });

  test(fn + ': a time limit spent before the solve at T_hi gives "feasible", without that solve', () => {
    const clock = shiftClock();
    try {
      const r = withSolve((k, m, o) => {
        if (k === 1) { const sol = realSolve(m, o); clock.by(2000); return sol; }
        assert.fail('no solve after the time limit');
      }, () => MPP[fn](gg, s, t, { T: T, timeLimit: 1000 }));
      assert.strictEqual(r.status, 'feasible');
      assert.strictEqual(r.objective, val);
      assert.strictEqual(r.extra.time_limit_at_T_hi, true);
      assert.strictEqual(r.attempts.length, 1);
    } finally { clock.undo(); }
  });

  test(fn + ': a timed-out single horizon is "time_limit"', () => {
    const r = withSolve(() => TIMED_OUT, () => MPP[fn](gg, s, t, { T: T, horizon: 'single', timeLimit: 5 }));
    assert.strictEqual(r.status, 'time_limit');
    assert.strictEqual(r.paths, null);
  });
}

test('minMakespan: a timed-out integer or relaxed model is "time_limit"', () => {
  let r = withSolve(() => TIMED_OUT, () => MPP.minMakespan(G44, S44, T44, { timeLimit: 5, rng: new MPP.MathRandom(1) }));
  assert.strictEqual(r.status, 'time_limit');
  assert.deepStrictEqual(r.attempts.map((a) => [a.T, a.phase]), [[4, 'ilp']]);
  r = withSolve(() => TIMED_OUT, () => MPP.minMakespan(G44, S44, T44,
    { timeLimit: 5, solveRelaxedFirst: true, rng: new MPP.MathRandom(1) }));
  assert.strictEqual(r.status, 'time_limit');
  assert.deepStrictEqual(r.attempts.map((a) => [a.T, a.phase]), [[4, 'lp']]);
  // the limit passes during an infeasible horizon: no solve at the next one
  const clock = shiftClock();
  try {
    r = withSolve((k) => { clock.by(10); return Object.assign({}, TIMED_OUT, { status: 'infeasible' }); },
      () => MPP.minMakespan(G44, S44, T44, { timeLimit: 5, rng: new MPP.MathRandom(1) }));
  } finally { clock.undo(); }
  assert.strictEqual(r.status, 'time_limit');
  assert.strictEqual(r.attempts.length, 1);
  // no limit: the same instance is solved
  r = MPP.minMakespan(G44, S44, T44, { rng: new MPP.MathRandom(1) });
  assert.strictEqual(r.status, 'optimal');
  assert.strictEqual(r.objective, 4);
});

// ------------------------------------------------------------------ lp.js maxEntries (the demo's memory guard)
test('lp.solve maxEntries: a model over the limit throws ModelTooLarge before allocating; under it nothing changes', () => {
  const m = MPP.buildModel('makespan', g, starts, goals, 4).model;
  const need = MPP.lp.denseEntries(m.numColumns, m.numRows);
  assert.strictEqual(need, m.numRows * m.numColumns + 2 * m.numRows * (m.numColumns + 2 * m.numRows));
  assert.throws(() => MPP.lp.solve(m, { maxEntries: need - 1 }), (e) => e.name === 'ModelTooLarge' && /columns/.test(e.message));
  assert.deepStrictEqual(MPP.lp.solve(m, { maxEntries: need }).x, MPP.lp.solve(m, {}).x);
  // through the engine and planSplit (as the demo's worker runs it): the error reaches the caller
  assert.throws(() => MPP.minMakespan(g, starts, goals, { rng: new MPP.MathRandom(1), lp: { maxEntries: 10 } }),
                (e) => e.name === 'ModelTooLarge');
  assert.throws(() => MPP.planSplit(g, starts, goals, 2, { rng: new MPP.MathRandom(1), lp: { maxEntries: 10 } }),
                (e) => e.name === 'ModelTooLarge');
});

// ------------------------------------------------------------------ input checks (as mppflow)
test('invalid horizon, gap and feasibilityBudget throw ValueError, as in mppflow', () => {
  const path3 = MPP.fromAdjacency([[1], [0, 2], [1]]);
  const isValueError = (re) => (e) => e.name === 'ValueError' && re.test(e.message);
  for (const horizon of ['foo', 'Single', '', 0]) {
    assert.throws(() => MPP.minTotalTime(path3, [0], [2], { horizon }), isValueError(/horizon/), String(horizon));
  }
  for (const gap of [-0.1, NaN, Infinity]) {
    for (const f of ['minTotalTime', 'minTotalDistance', 'minMaxDistance']) {
      assert.throws(() => MPP[f](path3, [0], [2], { gap }), isValueError(/gap/), f + ' gap ' + gap);
    }
  }
  // a budget <= 0 left every infeasibility proof inconclusive: the path swap looped forever
  for (const feasibilityBudget of [0, -5, NaN]) {
    assert.throws(() => MPP.minMakespan(path3, [0, 2], [2, 0], { feasibilityBudget }), isValueError(/feasibility_budget/));
    assert.throws(() => MPP.planSplit(path3, [0, 2], [2, 0], 2, { feasibilityBudget }), isValueError(/feasibility_budget/));
  }
  assert.strictEqual(MPP.minTotalTime(path3, [0], [2], { horizon: null, gap: 0 }).objective, 2);
});

test('T and tMax must be integers (TypeError), as in mppflow', () => {
  const path3 = MPP.fromAdjacency([[1], [0, 2], [1]]);
  const isTypeError = (re) => (e) => e.name === 'TypeError' && re.test(e.message);
  for (const f of ['minTotalTime', 'minTotalDistance', 'minMaxDistance']) {
    // T = 2.5 gave a false Infeasible (distance) or a RangeError (total time)
    assert.throws(() => MPP[f](path3, [0], [2], { T: 2.5, horizon: 'single' }), isTypeError(/^T must be an integer/), f);
    for (const tMax of [2.5, NaN, true, '3']) {
      assert.throws(() => MPP[f](path3, [0], [2], { tMax }), isTypeError(/^t_max must be an integer/), f + ' ' + tMax);
    }
  }
  assert.throws(() => MPP.minMakespan(path3, [0], [2], { tMax: 2.5 }), isTypeError(/^t_max/));
  assert.strictEqual(MPP.minTotalDistance(path3, [0], [2], { T: 2, horizon: 'single' }).objective, 2);
});

test('vertex ids, neighbours and k must be integers (TypeError), as in mppflow', () => {
  // a string id was solved and came back in an 'optimal' plan, a fractional or boolean one gave
  // a false Infeasible, and k = 1.5 or '2' ran a split
  const g3 = MPP.grid(3, 3);
  const isTypeError = (re) => (e) => e.name === 'TypeError' && re.test(e.message);
  for (const f of ['minMakespan', 'minTotalTime', 'minTotalDistance', 'minMaxDistance']) {
    for (const bad of ['0', 0.5, true, null, NaN]) {
      assert.throws(() => MPP[f](g3, [bad, 1], [8, 7]), isTypeError(/^each of the starts must be an integer/), f + ' ' + bad);
      assert.throws(() => MPP[f](g3, [0, 1], [8, bad]), isTypeError(/^each of the goals must be an integer/), f + ' ' + bad);
    }
  }
  for (const k of [1.5, '2', true, NaN]) {
    assert.throws(() => MPP.planSplit(g3, [0, 1], [8, 7], k, {}), isTypeError(/^split must be an integer/), String(k));
  }
  for (const k of [1, 2]) {
    assert.throws(() => MPP.planSplit(g3, ['0', 1], [8, 7], k, {}), isTypeError(/^each of the starts/), String(k));
  }
  for (const bad of ['1', 1.5, false]) {
    assert.throws(() => MPP.fromAdjacency([[bad], [0, 2], [1]]), isTypeError(/^each of the neighbours must be an integer/));
  }
  // typed arrays are sequences of integers
  assert.strictEqual(MPP.minMakespan(g3, new Int32Array([0, 1]), Uint8Array.from([8, 7])).objective, 4);
});

// ------------------------------------------------------------------ solver limits (as mppflow)
test('a node limit leaves a horizon undecided: no false Infeasible, no endless makespan loop', () => {
  const p = MPP.createGridProblem(5, 5, 0.1, 10, new MPP.MathRandom(1));  // optimal makespan 6
  for (const f of ['minTotalTime', 'minTotalDistance', 'minMaxDistance']) {
    const r = MPP[f](p.graph, p.starts, p.goals, { T: 8, horizon: 'single', lp: { nodeLimit: 0 } });
    assert.strictEqual(r.status, 'time_limit', f);
    assert.strictEqual(r.paths, null, f);
  }
  const t0 = Date.now();
  const r = MPP.minMakespan(p.graph, p.starts, p.goals, { lp: { nodeLimit: 0 } });
  assert.ok(Date.now() - t0 < 20000);
  assert.strictEqual(r.status, 'time_limit');
  assert.strictEqual(r.extra.solver_status, 'node_limit');
  assert.deepStrictEqual(r.attempts.map((a) => a.T), [6]);
});

test('the makespan loop takes the joint-state plan after a failed horizon', () => {
  // a 2-way split piece fails at T = 2 and is feasible only at T = 6: the branch and bound took 30-50 s
  const g = MPP.fromAdjacency([[1, 7], [0, 2, 6], [1, 3, 5], [2, 4], [3], [2], [1], [0]]);
  const t0 = Date.now();
  const r = MPP.planSplit(g, [4, 6, 1, 3], [2, 6, 0, 7], 2, { rng: new MPP.MathRandom(1) });
  assert.ok(Date.now() - t0 < 10000);
  assert.strictEqual(r.status, 'feasible');
  assert.strictEqual(r.metrics.makespan, 8);
  assert.ok(MPP.validate(r.paths, g, [4, 6, 1, 3], [2, 6, 0, 7]));
});

test('jointOptimum returns null above 30 robots (32-bit robot sets), as mppflow', () => {
  for (const [A, obj] of [[31, 'total_time'], [32, 'total_time'], [33, 'distance']]) {
    const adj = [[1]];
    for (let i = 1; i < A; i++) adj.push([i - 1, i + 1]);
    adj.push([A - 1]);
    const s = [], t = [];
    for (let i = 0; i < A; i++) { s.push(i); t.push(i + 1); }
    assert.strictEqual(MPP.jointOptimum(MPP.fromAdjacency(adj), s, t, obj, 2e6), null, A + ' ' + obj);
  }
});

test('jointOptimum: the budget also bounds the 2^k closure states (k robots at their goals), as mppflow', () => {
  // formerly every closure of the start was on the heap before the first budget check
  // (22 robots: 1.5 GB at budget 1000)
  const g56 = MPP.grid(5, 6);
  for (const A of [24, 30]) {
    const s = [], t = [];
    for (let i = 0; i < A; i++) { s.push(i); t.push(i); }
    s[0] = 29;
    const t0 = Date.now();
    assert.strictEqual(MPP.jointOptimum(g56, s, t, 'total_time', 1000), null, String(A));
    assert.ok(Date.now() - t0 < 2000, String(A));
  }
});

// ------------------------------------------------------------------ time-limit validation
test('the solvers reject a time limit that is not a finite number > 0 (null: no limit), as mppflow.plan', () => {
  for (const bad of [0, -1, NaN, Infinity, '5', true]) {
    assert.throws(() => MPP.minMakespan(g, starts, goals, { timeLimit: bad }), /time_limit must be None/);
    assert.throws(() => MPP.minTotalTime(g, starts, goals, { timeLimit: bad }), /time_limit must be None/);
    assert.throws(() => MPP.minTotalDistance(g, starts, goals, { timeLimit: bad }), /time_limit must be None/);
    assert.throws(() => MPP.planSplit(g, starts, goals, 2, { timeLimit: bad }), /time_limit must be None/);
  }
  assert.strictEqual(MPP.minMakespan(g, starts, goals, { timeLimit: null }).status, 'optimal');
  assert.strictEqual(MPP.minMakespan(g, starts, goals, { timeLimit: 600 }).status, 'optimal');
});

// ------------------------------------------------- infeasibility certificates and horizons (as mppflow)
test('polynomial certificates prove large infeasible instances (as mppflow joint_feasibility)', () => {
  const g5 = MPP.grid(5, 5);
  const big = MPP.fromAdjacency(g5.adj.concat([[26], [25]]));  // a separate edge 25 - 26
  const s = [0, 4, 20, 24, 12, 7, 25, 26], t = [24, 20, 4, 0, 2, 22, 26, 25];
  assert.strictEqual(MPP.jointFeasibility(big, s, t), false);
  assert.throws(() => MPP.minMakespan(big, s, t), (e) => e.name === 'Infeasible' && /joint-state search/.test(e.message));
  const corridor = MPP.fromAdjacency([...Array(16).keys()].map((u) => [u - 1, u + 1].filter((v) => v >= 0 && v < 16)));
  assert.strictEqual(MPP.jointFeasibility(corridor, [0, 1, 2, 3, 4], [15, 14, 13, 12, 11]), false);
  const a = [...Array(18)].map(() => []);
  for (const off of [0, 9]) {
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) {
        const v = off + 3 * r + c;
        if (c < 2) { a[v].push(v + 1); a[v + 1].push(v); }
        if (r < 2) { a[v].push(v + 3); a[v + 3].push(v); }
      }
    }
  }
  a[8].push(9); a[9].push(8);
  const bg = [...Array(18).keys()]; bg[8] = 9; bg[9] = 8;
  assert.strictEqual(MPP.jointFeasibility(MPP.fromAdjacency(a), [...Array(18).keys()], bg), false);
  const ring6 = MPP.fromAdjacency([...Array(6).keys()].map((u) => [(u + 5) % 6, (u + 1) % 6]));
  assert.strictEqual(MPP.jointFeasibility(ring6, [0, 2, 4], [2, 4, 0]), true);
  assert.strictEqual(MPP.jointFeasibility(ring6, [0, 2, 4], [2, 0, 4]), false);
  assert.strictEqual(MPP.jointFeasibility(MPP.grid(3, 3), [0, 1, 2, 5, 8, 7, 6, 3, 4], [1, 2, 5, 8, 7, 6, 3, 0, 4]), true);
});

test('horizon "single" without goal wait loops: "makespan exactly T"; negative T / tMax: ValueError', () => {
  const nw = { n: 2, adj: [[1], [0]], xy: [null, null], rows: 0, cols: 0 };
  const path3 = MPP.fromAdjacency([[1], [0, 2], [1]]);
  const isValueError = (re) => (e) => e.name === 'ValueError' && re.test(e.message);
  for (const f of ['minTotalTime', 'minTotalDistance', 'minMaxDistance']) {
    assert.throws(() => MPP[f](nw, [0], [1], { T: 2, horizon: 'single' }), /makespan exactly T = 2/, f);
    assert.throws(() => MPP[f](nw, [0], [1], { T: 3, tMax: 2, horizon: 'single' }), /makespan exactly t_max = 2/, f);
    assert.deepStrictEqual(MPP[f](nw, [0], [1], { tMax: 2 }).paths, [[0, 1]], f);
    for (const st of [[[0], [2]], [[], []]]) {
      assert.throws(() => MPP[f](path3, st[0], st[1], { tMax: -1 }), isValueError(/^t_max must be >= 0/), f);
      assert.throws(() => MPP[f](path3, st[0], st[1], { T: -1, horizon: 'single' }), isValueError(/^T must be >= 0/), f);
    }
  }
  assert.throws(() => MPP.minMakespan(path3, [], [], { tMax: -1 }), isValueError(/^t_max must be >= 0/));
});
