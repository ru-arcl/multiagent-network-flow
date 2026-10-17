/*
 * bench.js -- how long the builtin solver (docs/js/lp.js) takes on the demo maps' models.
 *
 *   node docs/js/bench.js [--limit SECONDS] [--quick]
 *
 * Prints one line per model: columns x rows, simplex iterations, B&B nodes and wall time
 * (Node times only; the Python times in docs/js/README.md are demo-data.js's python_seconds).
 * Used to choose the live-solve size cap documented in docs/js/README.md. Node only.
 * --limit: the time limit per solve (default 60 s). --quick leaves out the 16-puzzle ILP, the
 * 16-puzzle planSplit k = 4 and most 24x18 pieces; it still runs the 16-puzzle LP and the
 * 8x8 20-robot LP and ILP, which stop at the limit (about 4 minutes in all with --limit 60). The last
 * section prints model sizes only, for the crowded 8x8 map of the demo (40 robots).
 */
'use strict';
const path = require('path');
const MPP = require(path.join(__dirname, 'engine.js'));

const args = process.argv.slice(2);
const LIMIT = args.includes('--limit') ? +args[args.indexOf('--limit') + 1] : 60;
const QUICK = args.includes('--quick');

function now() { return Number(process.hrtime.bigint()) / 1e9; }

const rows = [];
function solveModel(label, g, s, t, kind, T, opts) {
  const m = MPP.buildModel(kind, g, s, t, T, opts || {});
  const t0 = now();
  const sol = MPP.lp.solve(m, { timeLimit: LIMIT });
  const sec = now() - t0;
  const row = { label, kind: kind + ((opts && opts.integer === false) ? ' (LP)' : ''), T, robots: s.length,
                cols: m.model.numColumns, rows: m.model.numRows, status: sol.status, obj: sol.objective,
                iters: sol.iterations, nodes: sol.nodeCount, sec };
  rows.push(row);
  console.log(`${label.padEnd(28)} ${row.kind.padEnd(20)} T=${String(T).padStart(2)} ` +
              `${String(row.cols).padStart(5)}x${String(row.rows).padEnd(5)} ${sol.status.padEnd(10)} ` +
              `obj=${String(sol.objective).padEnd(5)} it=${String(sol.iterations).padStart(7)} ` +
              `nodes=${String(sol.nodeCount).padStart(4)} ${sec.toFixed(3)} s`);
  return sol;
}

function timed(label, f) {
  const t0 = now();
  const r = f();
  const sec = now() - t0;
  console.log(`${label.padEnd(49)} ${(r.status || '').padEnd(10)} ${r.metrics ? JSON.stringify(r.metrics) : ''} ${sec.toFixed(3)} s`);
  return r;
}

function lb(g, s, t) { return MPP.lowerBounds(g, s, t).makespan; }

// ---- instances
const fig4adj = [[2], [2], [0, 1, 4], [4], [2, 3, 5], [4]];
const fig4 = { g: MPP.fromAdjacency(fig4adj), s: [3, 5], t: [0, 1] };
const fig8 = { g: MPP.grid(4, 4), s: [13, 8], t: [0, 7] };
const star = { g: MPP.fromAdjacency([[1], [0, 2, 3], [1], [1]]), s: [0, 3], t: [3, 0] };
const nine = { g: MPP.grid(3, 3), s: [2, 4, 5, 1, 8, 6, 7, 3, 0], t: [0, 1, 2, 3, 4, 5, 6, 7, 8] };
const p16 = MPP.createN2Puzzle(4, new MPP.MathRandom(2016));
const sixteen = { g: p16.graph, s: p16.starts, t: p16.goals };

console.log('--- single models');
for (const [name, I] of [['Fig. 4(a) tree', fig4], ['Fig. 8 4x4 split', fig8], ['star swap', star]]) {
  const T = Math.max(lb(I.g, I.s, I.t), 1) + 1;
  for (const kind of ['makespan', 'total_time', 'total_distance', 'max_distance']) solveModel(name, I.g, I.s, I.t, kind, T);
}
solveModel('9-puzzle (Fig. 1)', nine.g, nine.s, nine.t, 'makespan', 4, { integer: false, setOptimal: false });
solveModel('9-puzzle (Fig. 1)', nine.g, nine.s, nine.t, 'makespan', 4);
// the total-time ILP at T = 4: the second of minTotalTime's two ILPs (the first finds the makespan). Its build with
// integer: false is no LP relaxation: the loopback and per-robot columns stay binary, so lp.solve still branches
solveModel('9-puzzle (Fig. 1)', nine.g, nine.s, nine.t, 'total_time', 4);
const l16 = lb(sixteen.g, sixteen.s, sixteen.t);
solveModel('16-puzzle seed 2016', sixteen.g, sixteen.s, sixteen.t, 'makespan', l16, { integer: false, setOptimal: false });
if (!QUICK) solveModel('16-puzzle seed 2016', sixteen.g, sixteen.s, sixteen.t, 'makespan', l16);

for (const [col, row, frac, n, seed] of [[5, 5, 0.2, 4, 3], [6, 6, 0.1, 8, 7], [8, 8, 0.2, 10, 1], [8, 8, 0.0, 20, 2]]) {
  const p = MPP.createGridProblem(col, row, frac, n, new MPP.MathRandom(seed));
  const T = lb(p.graph, p.starts, p.goals);
  solveModel(`${col}x${row} ${frac * 100}% ${n} robots s${seed}`, p.graph, p.starts, p.goals, 'makespan', T, { integer: false, setOptimal: false });
  solveModel(`${col}x${row} ${frac * 100}% ${n} robots s${seed}`, p.graph, p.starts, p.goals, 'makespan', T);
}

console.log('--- planners (whole runs)');
timed('Fig. 8 planSplit k=1', () => MPP.planSplit(fig8.g, fig8.s, fig8.t, 1, { rng: new MPP.MathRandom(1) }));
timed('Fig. 8 planSplit k=2', () => MPP.planSplit(fig8.g, fig8.s, fig8.t, 2, { rng: new MPP.MathRandom(1) }));
timed('Fig. 8 minTotalTime', () => MPP.minTotalTime(fig8.g, fig8.s, fig8.t));
timed('star minTotalDistance', () => MPP.minTotalDistance(star.g, star.s, star.t));
timed('9-puzzle planSplit k=1 (LP first)', () => MPP.planSplit(nine.g, nine.s, nine.t, 1, { rng: new MPP.MathRandom(1) }));
timed('9-puzzle planSplit k=2', () => MPP.planSplit(nine.g, nine.s, nine.t, 2, { rng: new MPP.MathRandom(1) }));
timed('9-puzzle planSplit k=4', () => MPP.planSplit(nine.g, nine.s, nine.t, 4, { rng: new MPP.MathRandom(1) }));
timed('9-puzzle minTotalTime', () => MPP.minTotalTime(nine.g, nine.s, nine.t, { rng: new MPP.MathRandom(1) }));
if (!QUICK) timed('16-puzzle planSplit k=4', () => MPP.planSplit(sixteen.g, sixteen.s, sixteen.t, 4, { rng: new MPP.MathRandom(1), timeLimit: LIMIT }));

console.log('--- pieces of the k-way split of a 24x18 grid, 25% obstacles, 50 robots (seed 1)');
const big = MPP.createGridProblem(24, 18, 0.25, 50, new MPP.MathRandom(1));
for (const splits of (QUICK ? [4] : [4, 3, 2])) {
  const msg = MPP.splitConfigurations(big.graph, big.starts, big.goals, splits, { rng: new MPP.MathRandom(1) });
  const k = msg.length - 1;
  let total = 0, worst = 0;
  for (let i = 0; i < k && (!QUICK || i < 3); i++) {
    const T = lb(big.graph, msg[i], msg[i + 1]);
    solveModel(`24x18 k=${k} piece ${i}`, big.graph, msg[i], msg[i + 1], 'makespan', T, { integer: false, setOptimal: false });
    const r = rows[rows.length - 1];
    total += r.sec; worst = Math.max(worst, r.sec);
  }
  console.log(`k=${k}: LP relaxations of the pieces: total ${total.toFixed(2)} s, worst ${worst.toFixed(2)} s`);
}

// model sizes only (nothing is solved): the crowded 8x8 map of the demo, no obstacles, 40 robots (seed 1). The split
// configurations are those of the demo's makespan/k2 and makespan/k4 plans (same seed), and each piece is shown at
// its lower-bound horizon and the next one
console.log('--- model sizes: 8x8 grid, no obstacles, 40 robots (seed 1)');
const crowd = MPP.createGridProblem(8, 8, 0.0, 40, new MPP.MathRandom(1));
function size(s, t, T) {
  const m = MPP.buildModel('makespan', crowd.graph, s, t, T);
  return `${m.model.numColumns}x${m.model.numRows}`;
}
const lc = lb(crowd.graph, crowd.starts, crowd.goals);
console.log(`unsplit: T=${lc} ${size(crowd.starts, crowd.goals, lc)}`);
for (const splits of [1, 2]) {
  const msg = MPP.splitConfigurations(crowd.graph, crowd.starts, crowd.goals, splits, { rng: new MPP.MathRandom(1) });
  for (let i = 0; i + 1 < msg.length; i++) {
    const T = lb(crowd.graph, msg[i], msg[i + 1]);
    console.log(`k=${msg.length - 1} piece ${i}: T=${T} ${size(msg[i], msg[i + 1], T)}, T=${T + 1} ${size(msg[i], msg[i + 1], T + 1)}`);
  }
}
