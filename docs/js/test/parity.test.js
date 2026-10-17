/*
 * Parity of docs/js/engine.js + docs/js/lp.js with the Python package mppflow.
 *
 * Every fixture in tests/parity/*.json (written by `python -m mppflow.parity`) holds inputs and
 * the outputs of mppflow; this test recomputes them in JavaScript and requires EXACT equality
 * (numbers compared with Object.is: bit-identical floats, -0 distinguished from 0).
 *
 * Run: npm test   (node --test docs/js/test/; Node >= 18, no dependencies)
 */
'use strict';

const test = require('node:test');
const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const crypto = require('node:crypto');

const ROOT = path.resolve(__dirname, '..', '..', '..');
const JS = path.join(ROOT, 'docs', 'js');
const FIX = path.join(ROOT, 'tests', 'parity');
const MPP = require(path.join(JS, 'engine.js'));

function load(name) {
  const file = path.join(FIX, name + '.json');
  const text = fs.readFileSync(file, 'utf8');
  return JSON.parse(text, (k, v) => {
    if (v === '__Infinity__') return Infinity;
    if (v === '__-Infinity__') return -Infinity;
    if (v === '__NaN__') return NaN;
    return v;
  });
}

function has(name) { return fs.existsSync(path.join(FIX, name + '.json')); }

function sha256(text) { return crypto.createHash('sha256').update(text, 'utf8').digest('hex'); }

/** Exact structural equality; throws an AssertionError naming the first differing path. */
function exact(actual, expected, where) {
  where = where || '$';
  if (typeof expected === 'number') {
    if (typeof actual !== 'number' || !Object.is(actual, expected)) {
      assert.fail(`${where}: expected ${Object.is(expected, -0) ? '-0' : expected}, got ${Object.is(actual, -0) ? '-0' : actual}`);
    }
    return;
  }
  if (expected === null || typeof expected !== 'object') {
    if (actual !== expected) assert.fail(`${where}: expected ${JSON.stringify(expected)}, got ${JSON.stringify(actual)}`);
    return;
  }
  if (Array.isArray(expected)) {
    if (!Array.isArray(actual)) assert.fail(`${where}: expected an array, got ${JSON.stringify(actual)}`);
    if (actual.length !== expected.length) assert.fail(`${where}: expected length ${expected.length}, got ${actual.length}`);
    for (let i = 0; i < expected.length; i++) exact(actual[i], expected[i], `${where}[${i}]`);
    return;
  }
  if (actual === null || typeof actual !== 'object' || Array.isArray(actual)) {
    assert.fail(`${where}: expected an object, got ${JSON.stringify(actual)}`);
  }
  const ek = Object.keys(expected).sort(), ak = Object.keys(actual).sort();
  if (ek.join(',') !== ak.join(',')) assert.fail(`${where}: keys ${JSON.stringify(ak)} != expected ${JSON.stringify(ek)}`);
  for (const k of ek) exact(actual[k], expected[k], `${where}.${k}`);
}

function graphOf(d) { return MPP.makeGraph(d.adj, d.xy, 0, 0); }
function graphJson(g) { return { n: g.n, adj: g.adj, xy: g.xy }; }
function instance(fx, name) {
  const d = fx.instances[name];
  return { g: graphOf(d.graph), s: d.starts, t: d.goals };
}
function caseGraph(fx, c) {
  if (c.instance !== undefined) return instance(fx, c.instance);
  return { g: graphOf(c.graph), s: c.starts, t: c.goals };
}

// ---------------------------------------------------------------- loading as classic scripts
test('engine.js and lp.js load as classic browser scripts (window.MPP)', () => {
  const sandbox = { window: {}, console };
  vm.createContext(sandbox);
  for (const f of ['lp.js', 'engine.js']) vm.runInContext(fs.readFileSync(path.join(JS, f), 'utf8'), sandbox, { filename: f });
  const W = sandbox.window.MPP;
  assert.ok(W && typeof W.planSplit === 'function' && W.lp && typeof W.lp.solve === 'function');
  const sb2 = { window: {}, console };
  vm.createContext(sb2);
  for (const f of ['engine.js', 'lp.js']) vm.runInContext(fs.readFileSync(path.join(JS, f), 'utf8'), sb2, { filename: f });
  const W2 = sb2.window.MPP;
  const g = W2.grid(4, 4);
  const r = W2.minMakespan(g, [13, 8], [0, 7], { rng: new W2.MathRandom(1) });
  assert.strictEqual(r.objective, 4);
});

test('MPP.workerSource() is a complete worker program (lp.js + engine.js + handler)', () => {
  const src = MPP.workerSource();
  const msgs = [];
  const self = { postMessage: (m) => msgs.push(m) };
  self.self = self;
  const sandbox = { self, console, performance };
  vm.createContext(sandbox);
  vm.runInContext('var self = this.self;\n' + src, sandbox, { filename: 'worker' });
  const g = MPP.grid(4, 4);
  self.onmessage({ data: { id: 1, fn: 'planSplit', args: [g, [13, 8], [0, 7], 2, {}], seed: 1 } });
  self.onmessage({ data: { id: 2, fn: 'minMakespan', args: [MPP.fromAdjacency([[1], [0, 2], [1]]), [0, 2], [2, 0]] } });
  self.onmessage({ data: { id: 3, fn: 'noSuchFunction', args: [] } });
  const direct = MPP.planSplit(g, [13, 8], [0, 7], 2, { rng: new MPP.MathRandom(1) });
  assert.strictEqual(msgs.length, 3);
  assert.ok(msgs[0].ok);
  exact(msgs[0].result.paths, direct.paths, 'worker planSplit');
  exact(msgs[1].ok, false, 'worker infeasible');
  exact(msgs[1].error.name, 'Infeasible', 'worker error name');
  exact(msgs[2].ok, false, 'worker unknown fn');
});

// ---------------------------------------------------------------- java.json
test('java: MathRandom / JavaRandom streams', () => {
  const fx = load('java');
  for (const c of fx.cases.rng) {
    const r = new MPP.MathRandom(c.seed);
    const vals = [];
    for (let i = 0; i < c.random.length; i++) vals.push(r.random());
    exact(vals, c.random, `rng(${c.seed}).random`);
    exact(r.draws, c.draws, 'draws');
    exact(r.state(), c.state, 'state');
    const jr = new MPP.JavaRandom(c.seed);
    const bounds = [1, 2, 3, 7, 10, 16, 100, 1000, 2 ** 30, 2 ** 31 - 1, 5, 64];
    const ints = bounds.map((b) => jr.nextInt(b));
    ints.push(jr.nextInt());
    exact(ints, c.next_int, `rng(${c.seed}).nextInt`);
    exact(jr.state(), c.state_after_ints, 'state after ints');
  }
});

test('java: JavaIntHashSet iteration order (incl. treeified bins)', () => {
  const fx = load('java');
  for (const c of fx.cases.hashset) {
    const hs = new MPP.JavaIntHashSet();
    let si = 0;
    c.ops.forEach((op, step) => {
      const ret = op[0] === 0 ? hs.add(op[1]) : hs.remove(op[1]);
      if ((ret ? 1 : 0) !== op[2]) assert.fail(`${c.name} step ${step}: return ${ret}, expected ${op[2]}`);
      if (si < c.snapshots.length && c.snapshots[si][0] === step) {
        exact(hs.toList(), c.snapshots[si][1], `${c.name} snapshot ${step}`);
        si++;
      }
    });
    exact(hs.toList(), c.final, `${c.name} final`);
    exact(hs.size, c.size, `${c.name} size`);
  }
});

test('java: Double.toString, golden numbers, round()', () => {
  const fx = load('java');
  for (const [v, js, golden] of fx.cases.doubles) {
    exact(MPP.javaDoubleStr(v), js, `javaDoubleStr(${v})`);
    if (golden !== null) exact(MPP.goldenNumber(v), golden, `goldenNumber(${v})`);
  }
  for (const [v, r] of fx.cases.round) exact(MPP.pyRound(v), r, `pyRound(${v})`);
});

// ---------------------------------------------------------------- graphs.json
test('graphs: generators reproduce mppflow exactly (same draws)', () => {
  const fx = load('graphs');
  for (const c of fx.cases.grid) exact(graphJson(MPP.grid(c.rows, c.cols)), c.expected, `grid(${c.rows},${c.cols})`);
  for (const c of fx.cases.holes) {
    const rng = new MPP.MathRandom(c.seed);
    exact(graphJson(MPP.gridWithHoles(c.rows, c.cols, c.percent, rng)), c.expected, `holes ${JSON.stringify(c)}`);
    exact(rng.draws, c.draws, 'holes draws');
  }
  for (const c of fx.cases.grid_problem) {
    const rng = new MPP.MathRandom(c.seed);
    const p = MPP.createGridProblem(c.col, c.row, c.frac, c.n, rng);
    const where = `createGridProblem(${c.col},${c.row},${c.frac},${c.n},seed ${c.seed})`;
    exact({ graph: graphJson(p.graph), starts: p.starts, goals: p.goals }, c.expected, where);
    exact(p.graph.rows, c.col, where + ' rows');
    exact(p.graph.cols, c.row, where + ' cols');
    exact(rng.draws, c.draws, where + ' draws');
  }
  for (const c of fx.cases.n2_puzzle) {
    const rng = new MPP.MathRandom(c.seed);
    const p = MPP.createN2Puzzle(c.n, rng);
    exact({ graph: graphJson(p.graph), starts: p.starts, goals: p.goals }, c.expected, `createN2Puzzle(${c.n})`);
    exact(rng.draws, c.draws, 'n2 draws');
  }
  for (const c of fx.cases.from_adjacency) exact(graphJson(MPP.fromAdjacency(c.adj, c.xy)), c.expected, 'fromAdjacency');
});

// ---------------------------------------------------------------- pathfinder.json
test('pathfinder: BFS, distance rings, randomized shortest paths, lower bounds', () => {
  const fx = load('pathfinder');
  for (const c of fx.cases.bfs) {
    const { g } = caseGraph(fx, c);
    exact(MPP.bfsDistances(g, c.start), c.expected, `bfs ${c.instance} ${c.start}`);
    exact(MPP.bfsDistances(g, c.start, -1), c.expected_unreachable, `bfs(-1) ${c.instance} ${c.start}`);
  }
  for (const c of fx.cases.goals_in_range) {
    const { g } = caseGraph(fx, c);
    exact(MPP.findGoalsInDistanceRange(g, c.start, c.dmin, c.dmax), c.expected,
          `range ${c.instance} ${c.start} [${c.dmin},${c.dmax}]`);
  }
  for (const c of fx.cases.shortest_paths) {
    const { g, s, t } = caseGraph(fx, c);
    const rng = new MPP.MathRandom(c.seed);
    const where = `shortest paths ${c.instance} seed ${c.seed}`;
    exact(MPP.findShortestPaths(g, s, t, rng), c.expected, where);
    exact(rng.draws, c.draws, where + ' draws');
    exact(MPP.getMakespanLowerBound(g, s, t, rng), c.makespan_lb, where + ' makespan lb');
    exact(rng.draws, c.draws_after_mlb, where + ' draws');
    exact(MPP.getTotalTimeLowerBound(g, s, t, rng), c.total_time_lb, where + ' tt lb');
    exact(rng.draws, c.draws_after_tlb, where + ' draws');
  }
  for (const c of fx.cases.lower_bounds) {
    const { g, s, t } = caseGraph(fx, c);
    exact(MPP.lowerBounds(g, s, t), c.expected, `lowerBounds ${c.instance}`);
  }
});

// ---------------------------------------------------------------- network.json
function pyNetwork(net) {
  const out = Object.assign({}, net);
  out.arcs = net.arcs.map((a) => {
    const b = Object.assign({}, a);
    b.kind = b.pyKind;
    delete b.pyKind;
    delete b.part;
    return b;
  });
  return out;
}

test('network: reachability, compact arcs, full and compact networks', () => {
  const fx = load('network');
  for (const c of fx.cases.reachability) {
    const { g, s, t } = caseGraph(fx, c);
    const r = MPP.reachability(g, s, t, c.T);
    exact(r.distStart, c.dist_start, `reach ${c.instance} T=${c.T} start`);
    exact(r.distGoal, c.dist_goal, `reach ${c.instance} T=${c.T} goal`);
  }
  for (const c of fx.cases.compact_arcs) {
    const { g, s, t } = caseGraph(fx, c);
    const arcs = MPP.compactArcs(g, s, MPP.reachability(g, s, t, c.T));
    exact(arcs.length, c.count, `arcs ${c.instance} T=${c.T} count`);
    if (c.arcs) exact(arcs, c.arcs, `arcs ${c.instance} T=${c.T}`);
    exact(sha256(JSON.stringify(arcs)), c.sha256, `arcs ${c.instance} T=${c.T} sha256`);
  }
  for (const c of fx.cases.network) {
    const { g, s, t } = caseGraph(fx, c);
    if (c.error !== undefined) {
      assert.throws(() => MPP.network(g, s, t, c.T, { variant: c.variant, prune: c.prune }),
                    (e) => e.name === 'ValueError' && e.message === c.error, `network ${c.variant} ${c.instance}`);
      continue;
    }
    const net = MPP.network(g, s, t, c.T, { variant: c.variant, prune: c.prune });
    exact(pyNetwork(net), c.expected, `network ${c.variant} ${c.instance} T=${c.T}`);
    for (const a of net.arcs) assert.ok(['green', 'blue', 'gadget', 'loopback'].includes(a.kind));
  }
  for (const c of fx.cases.network_counts) {
    const { g, s, t } = caseGraph(fx, c);
    exact(MPP.network(g, s, t, c.T, { variant: 'full' }).counts, c.full, `full counts ${c.instance}`);
    exact(MPP.network(g, s, t, c.T, { variant: 'compact', prune: true }).counts, c.compact, `compact counts ${c.instance}`);
  }
});

// ---------------------------------------------------------------- models.json
test('models: canonical text identical to mppflow.model.canonical_text', () => {
  const fx = load('models');
  for (const c of fx.cases.models) {
    const { g, s, t } = caseGraph(fx, c);
    const m = MPP.buildModel(c.kind, g, s, t, c.T, { integer: c.integer, setOptimal: c.set_optimal });
    const where = `model ${c.kind} ${c.instance} T=${c.T} integer=${c.integer} setOptimal=${c.set_optimal}`;
    exact(m.model.numColumns, c.ncols, where + ' ncols');
    exact(m.model.numRows, c.nrows, where + ' nrows');
    const text = m.canonicalText();
    if (c.text !== undefined) exact(text, c.text, where + ' text');
    exact(sha256(text), c.sha256, where + ' sha256');
    exact(MPP.canonicalText(m), text, where + ' canonicalText()');
  }
});

// ---------------------------------------------------------------- lp*.json
function solJson(sol) {
  return { status: sol.status, objective: sol.objective, x: sol.x, bound: sol.bound, mip_gap: sol.mipGap,
           sol_count: sol.solCount, node_count: sol.nodeCount, iterations: sol.iterations, raw_status: sol.rawStatus };
}

function lpCases(name) {
  const fx = load(name);
  for (const c of fx.cases.lp) {
    const { g, s, t } = caseGraph(fx, c);
    const m = MPP.buildModel(c.kind, g, s, t, c.T, { integer: c.integer, setOptimal: c.set_optimal });
    exact(m.model.numColumns, c.ncols, 'ncols');
    const opts = {};
    if (c.opts.node_limit !== undefined) opts.nodeLimit = c.opts.node_limit;
    if (c.opts.work_limit !== undefined) opts.workLimit = c.opts.work_limit;
    const sol = MPP.lp.solve(m, opts);
    exact(solJson(sol), c.expected, `lp ${c.kind} ${c.instance} T=${c.T} integer=${c.integer} ${JSON.stringify(c.opts)}`);
  }
}

test('lp: builtin simplex / branch and bound, bit-identical', () => lpCases('lp'));
test('lp (slow tier): 9-puzzle and 16-puzzle models', { skip: !has('lp_slow') }, () => lpCases('lp_slow'));

// ---------------------------------------------------------------- planners*.json
function solveJson(r, rng) {
  return { paths: r.paths, status: r.status, objective: r.objective, T: r.T, lower_bound: r.lowerBound,
           attempts: r.attempts.map((a) => ({ T: a.T, phase: a.phase, status: a.status, objective: a.objective, bound: a.bound })),
           extra: r.extra, draws: rng.draws };
}

function planJson(r, rng) {
  return { paths: r.paths, status: r.status, intermediate_goals: r.intermediateGoals,
           metrics: r.metrics === null ? { makespan: null, totalTime: null, totalDistance: null, maxDistance: null } : r.metrics,
           lower_bounds: r.lowerBounds, horizons: r.stats.horizons, phases: r.stats.phases,
           solves: r.stats.solves, failures: r.failures, draws: rng.draws };
}

function planCases(fx) {
  for (const c of fx.cases.plan_split) {
    const { g, s, t } = caseGraph(fx, c);
    const rng = new MPP.MathRandom(c.seed);
    const where = `planSplit ${c.instance || JSON.stringify(s)} k=${c.k}`;
    if (c.expected.error !== undefined) {
      assert.throws(() => MPP.planSplit(g, s, t, c.k, { rng }), (e) => e.name === c.expected.error, where);
      exact(rng.draws, c.expected.draws, where + ' draws');
      continue;
    }
    const r = MPP.planSplit(g, s, t, c.k, { rng });
    exact(planJson(r, rng), c.expected, where);
    if (r.paths !== null) assert.ok(MPP.validate(r.paths, g, s, t));
  }
}

const SOLVERS = { min_makespan: 'minMakespan', min_total_time: 'minTotalTime', min_total_distance: 'minTotalDistance',
                  min_max_distance: 'minMaxDistance' };

test('planners: split goals, optimal planners, k-way split, validation', () => {
  const fx = load('planners');
  for (const c of fx.cases.split_goals) {
    const { g, s, t } = caseGraph(fx, c);
    const rng = new MPP.MathRandom(c.seed);
    if (c.splits !== undefined) {
      exact(MPP.splitConfigurations(g, s, t, c.splits, { rng }), c.levels, `split levels ${c.instance} splits=${c.splits}`);
    } else {
      exact(MPP.splitGoals(g, s, t, { random: c.random, rng }), c.expected, `splitGoals ${c.instance || JSON.stringify(s)} random=${c.random}`);
    }
    exact(rng.draws, c.draws, 'split draws');
  }
  for (const c of fx.cases.solvers) {
    const { g, s, t } = caseGraph(fx, c);
    const rng = new MPP.MathRandom(c.seed);
    const kw = c.kwargs, opts = {};
    if (kw.solve_relaxed_first) opts.solveRelaxedFirst = true;
    if (kw.horizon) opts.horizon = kw.horizon;
    if (kw.T !== undefined) opts.T = kw.T;
    if (kw.t_max !== undefined) opts.tMax = kw.t_max;
    if (kw.gap !== undefined) opts.gap = kw.gap;
    if (kw.node_limit !== undefined) opts.lp = { nodeLimit: kw.node_limit };  // mppflow params={"node_limit": ...}
    if (c.fn === 'min_makespan' || c.fn === 'min_total_time') opts.rng = rng;
    const where = `${c.fn} ${c.instance || JSON.stringify(s)} ${JSON.stringify(kw)}`;
    let got;
    // joint_search: false disables the joint-state stage of the horizon search (as mppflow's
    // solvers.JOINT_SEARCH_MAX_CONFIGS = 0)
    const oldMax = kw.joint_search === false ? MPP._testing.setJointSearchMaxConfigs(0) : null;
    try {
      got = solveJson(MPP[SOLVERS[c.fn]](g, s, t, opts), rng);
    } catch (e) {
      if (!(e instanceof MPP.MPPError) && e.name !== 'ValueError') throw e;
      got = { error: e.name, message: e.message, draws: rng.draws };
    } finally {
      if (oldMax !== null) MPP._testing.setJointSearchMaxConfigs(oldMax);
    }
    exact(got, c.expected, where);
    if (got.paths) assert.ok(MPP.validate(got.paths, g, s, t), where + ' valid');
  }
  for (const c of fx.cases.joint_feasibility) {
    const { g, s, t } = caseGraph(fx, c);
    exact(MPP.jointFeasibility(g, s, t, c.budget, false, c.t_first), c.expected,
          `jointFeasibility ${JSON.stringify(c.graph.adj)} ${JSON.stringify(s)} tFirst=${c.t_first}`);
  }
  for (const c of fx.cases.joint_optimum) {
    const { g, s, t } = caseGraph(fx, c);
    const where = `jointOptimum ${c.objective} upper=${c.upper} ${JSON.stringify(s)}`;
    const got = MPP.jointOptimum(g, s, t, c.objective, c.budget, c.upper === null ? undefined : c.upper);
    exact(got, c.expected, where);
    if (got && got[1]) assert.ok(MPP.validate(got[1], g, s, t), where + ' valid');
  }
  planCases(fx);
  for (const c of fx.cases.checks) {
    const g = graphOf(c.graph);
    exact(MPP.pathSetError(c.paths, g, c.starts, c.goals), c.error, `pathSetError ${JSON.stringify(c.paths)}`);
    exact(MPP.validate(c.paths, g, c.starts, c.goals), c.error === null, 'validate');
    if (c.cycles !== null) exact(MPP.hasCycles(c.paths), c.cycles, `hasCycles ${JSON.stringify(c.paths)}`);
    if (c.metrics.error !== undefined) {
      assert.throws(() => MPP.metrics(c.paths, c.goals), (e) => e.name === c.metrics.error, `metrics ${JSON.stringify(c.paths)}`);
      continue;
    }
    const m = MPP.metrics(c.paths, c.goals);
    exact({ makespan: m.makespan, total_time: m.totalTime, total_distance: m.totalDistance, max_distance: m.maxDistance },
          c.metrics, `metrics ${JSON.stringify(c.paths)}`);
  }
});

test('planners (slow tier): the 9-puzzle with k = 1, 2, 4', { skip: !has('planners_slow') }, () => planCases(load('planners_slow')));
