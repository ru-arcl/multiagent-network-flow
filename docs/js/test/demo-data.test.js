/*
 * The precomputed plans of the demo (docs/data/demo-data.js, written by
 * docs/data/build_demo_data.py) against the engine, without a browser: every plan is valid on
 * its map, its stored metrics, value and rotations are what docs/js/engine.js computes, its
 * status is consistent with its kind (only a k = 1 plan can be "optimal"), its value respects
 * the map's lower bound, and no plan of a map beats a plan stored there as optimal. The format
 * is described in docs/data/README.md. tools/check_demo.mjs plays the same plans in Chrome.
 *
 * Run: npm test   (node --test docs/js/test/; Node >= 18, no dependencies)
 */
'use strict';

const test = require('node:test');
const assert = require('node:assert');
const path = require('node:path');

const ROOT = path.resolve(__dirname, '..', '..', '..');
const MPP = require(path.join(ROOT, 'docs', 'js', 'engine.js'));
const DATA = require(path.join(ROOT, 'docs', 'data', 'demo-data.js'));

const OBJECTIVES = ['makespan', 'total_time', 'total_distance', 'max_distance'];
const METRIC = { makespan: 'makespan', total_time: 'totalTime', total_distance: 'totalDistance', max_distance: 'maxDistance' };

function config(paths, t) { return paths.map((p) => p[t]); }

test('demo data: the maps', () => {
  assert.ok(Array.isArray(DATA.maps) && DATA.maps.length >= 8, 'MPP_DEMO.maps');
  const ids = DATA.maps.map((m) => m.id);
  assert.strictEqual(new Set(ids).size, ids.length, 'map ids are unique');
  const plans = DATA.maps.reduce((n, m) => n + Object.keys(m.plans).length, 0);
  assert.ok(plans >= 43, plans + ' plans');
});

for (const map of DATA.maps) {
  test('demo data: ' + map.id, () => {
    const g = MPP.fromAdjacency(map.graph.adj, map.graph.xy);
    const { starts, goals } = map;
    assert.strictEqual(map.graph.n, map.graph.adj.length);
    assert.strictEqual(starts.length, goals.length);
    map.graph.adj.forEach((nb, v) => assert.ok(nb.includes(v), 'vertex ' + v + ' has its wait move'));
    assert.deepStrictEqual(map.lower_bounds, MPP.lowerBounds(g, starts, goals), 'lower_bounds');
    for (const key of map.live_plans) assert.ok(map.plans[key] && map.plans[key].live, 'live plan ' + key);

    const best = {};  // objective -> the value of the plan stored as optimal
    for (const [key, p] of Object.entries(map.plans)) {
      const where = map.id + ' ' + key;
      assert.strictEqual(MPP.pathSetError(p.paths, g, starts, goals), null, where + ': invalid plan');
      const m = MPP.metrics(p.paths, goals);
      assert.deepStrictEqual(p.metrics, m, where + ': metrics');
      assert.strictEqual(p.rotations, MPP.hasCycles(p.paths), where + ': rotations');
      if (key === 'paper') { assert.strictEqual(p.live, false, where); continue; }

      assert.strictEqual(key, p.objective + '/k' + p.k, where + ': key');
      assert.ok(OBJECTIVES.includes(p.objective), where + ': objective');
      assert.strictEqual(p.value, m[METRIC[p.objective]], where + ': value = the plan\'s ' + p.objective);
      assert.ok(p.value >= map.lower_bounds[p.objective], where + ': value below the lower bound');
      assert.ok(['optimal', 'feasible'].includes(p.status), where + ': status ' + p.status);
      if (p.status === 'optimal') {
        assert.strictEqual(p.k, 1, where + ': only an exact (k = 1) plan is proved optimal');
        assert.notStrictEqual(p.horizon, 'single', where + ': a single-horizon plan is not proved optimal');
        best[p.objective] = p.value;
      }
      if (p.horizon === 'single') assert.strictEqual(p.status, 'feasible', where);
      assert.strictEqual(p.horizons.length, p.solves, where + ': one horizon per solve');
      assert.strictEqual(p.phases.length, p.solves, where + ': one phase per solve');

      if (p.k > 1 && p.intermediate_goals) {
        // the split's pieces: the plan passes the k - 1 configurations at the pieces' start times
        const cfgs = [starts, ...p.intermediate_goals, goals];
        assert.strictEqual(p.intermediate_goals.length, p.k - 1, where + ': intermediate_goals');
        assert.strictEqual(p.pieces.length, p.k, where + ': pieces');
        let t = 0;
        p.pieces.forEach((pc, i) => {
          assert.strictEqual(pc.t0, t, where + ': piece ' + i + ' start');
          assert.deepStrictEqual(config(p.paths, t), cfgs[i], where + ': configuration at piece ' + i);
          t += pc.T;
        });
        assert.strictEqual(t, m.makespan, where + ': the pieces add up to the makespan');
        assert.deepStrictEqual(config(p.paths, t), goals, where);
        const largest = Math.max(...p.pieces.map((pc) => pc.columns));
        assert.strictEqual(p.model.columns, largest, where + ': model = the largest piece');
      } else if (p.model) {
        assert.strictEqual(p.model.T, m.makespan, where + ': model horizon');
      }
    }
    // no stored plan (a split, another objective's plan, the paper's) beats a proved optimum
    for (const [obj, v] of Object.entries(best)) {
      for (const [key, p] of Object.entries(map.plans)) {
        assert.ok(p.metrics[METRIC[obj]] >= v, map.id + ': ' + key + ' has ' + obj + ' ' + p.metrics[METRIC[obj]] +
          ' below the optimum ' + v);
      }
    }
  });
}
