/*
 * engine.js -- the mppflow engine for the browser demo of our T-RO 2016 paper (Yu and LaValle).
 *
 * A line-by-line port of the Python package `mppflow` in its default `compat="fixed"` mode:
 * mppflow/_java.py (JavaRandom, MathRandom, JavaIntHashSet), graph.py / problem.py (grid
 * generators, holes, start/goal draws, conversion), pathfinder.py (BFS, randomized shortest
 * paths, lower bounds), network.py (reachability, compact arcs, the full and compact
 * time-expanded networks), model.py (the ILP builders, same column and row order, the
 * golden canonical text), solvers.py (MinMakespan, MinTotalTime, MinTotalDistance,
 * MinMaxDistance with the horizon search of Algorithm 1) and planner.py (splitPaths, the
 * k-way split, validation, rotations, metrics). The LP/MILP solver is docs/js/lp.js (a port of
 * mppflow/lp/builtin.py); load it before or after this file.
 *
 * Parity with Python is enforced by tests/parity/*.json (written by `python -m mppflow.parity`)
 * and `npm test` (docs/js/test/parity.test.js). See docs/js/README.md.
 *
 * Classic script: defines window.MPP in a browser and module.exports in Node. No dependencies.
 */
(function mppEngineFactory(root) {
  'use strict';

  var MPP = (root && root.MPP) || {};

  // ===================================================================== errors
  class MPPError extends Error {
    constructor(msg) { super(msg); this.name = 'MPPError'; }
  }
  /** The instance has no solution (or none within the given horizon bound). */
  class Infeasible extends MPPError {
    constructor(msg) { super(msg); this.name = 'Infeasible'; }
  }
  MPP.MPPError = MPPError;
  MPP.Infeasible = Infeasible;

  function valueError(msg) { var e = new Error(msg); e.name = 'ValueError'; return e; }
  function runtimeError(msg) { var e = new Error(msg); e.name = 'RuntimeError'; return e; }

  // ===================================================================== Java emulation
  var TWO24 = 16777216;
  var TWO53 = 9007199254740992;
  // 0x5DEECE66D = M_HI * 2^24 + M_LO
  var M_HI = 0x5DE;
  var M_LO = 0xECE66D;
  var POW2 = [];
  (function () { var p = 1; for (var i = 0; i <= 53; i++) { POW2.push(p); p *= 2; } })();

  /** Java `int` wrap-around of an exactly representable integer. */
  function i32(x) { return x | 0; }

  /** java.util.Random: same seed, same stream (48-bit LCG kept as two 24-bit halves). */
  class JavaRandom {
    constructor(seed) { this.setSeed(seed === undefined ? 0 : seed); }
    setSeed(seed) {
      var s = (BigInt(seed) ^ BigInt('0x5DEECE66D')) & BigInt('0xFFFFFFFFFFFF');
      this._hi = Number(s >> BigInt(24));
      this._lo = Number(s & BigInt(0xFFFFFF));
    }
    nextBits(bits) {
      var p = this._lo * M_LO + 0xB;
      var lo = p % TWO24;
      var carry = Math.floor(p / TWO24);
      var hi = (this._hi * M_LO + this._lo * M_HI + carry) % TWO24;
      this._hi = hi;
      this._lo = lo;
      return Math.floor((hi * TWO24 + lo) / POW2[48 - bits]) | 0;
    }
    nextDouble() {
      return (this.nextBits(26) * POW2[27] + this.nextBits(27)) / TWO53;
    }
    nextInt(bound) {
      if (bound === undefined) return this.nextBits(32);
      if (!(bound > 0)) throw valueError('bound must be positive');
      var r = this.nextBits(31);
      var m = bound - 1;
      if ((bound & m) === 0) return Math.floor((bound * r) / POW2[31]);
      var u = r;
      for (;;) {
        r = u % bound;
        if (u - r + m < POW2[31]) return r;
        u = this.nextBits(31);
      }
    }
    /** The internal 48-bit state (for tests). */
    state() { return this._hi * TWO24 + this._lo; }
  }

  /** Stand-in for the JVM-wide Math.random() generator (mppflow._java.MathRandom). */
  class MathRandom extends JavaRandom {
    constructor(seed) { super(seed); this.draws = 0; }
    random() { this.draws += 1; return this.nextDouble(); }
  }

  var _defaultRng = new MathRandom(0);
  function defaultRng() { return _defaultRng; }
  function seedDefaultRng(seed) { _defaultRng = new MathRandom(seed); }
  function resolveRng(rng) { return rng === undefined || rng === null ? _defaultRng : rng; }

  // ---- java.util.HashSet<Integer> iteration order (JDK 8+ HashMap, treeified bins included)
  function TreeNode(key, h) {
    this.key = key; this.hash = h;
    this.left = null; this.right = null; this.parent = null; this.red = false;
  }

  function tnDir(h, k, p) {
    if (p.hash > h) return -1;
    if (p.hash < h) return 1;
    return k < p.key ? -1 : 1;
  }

  function rotateLeft(root, p) {
    var r, rl, pp;
    if (p !== null && (r = p.right) !== null) {
      rl = p.right = r.left;
      if (rl !== null) rl.parent = p;
      pp = r.parent = p.parent;
      if (pp === null) { root = r; r.red = false; }
      else if (pp.left === p) pp.left = r;
      else pp.right = r;
      r.left = p;
      p.parent = r;
    }
    return root;
  }

  function rotateRight(root, p) {
    var l, lr, pp;
    if (p !== null && (l = p.left) !== null) {
      lr = p.left = l.right;
      if (lr !== null) lr.parent = p;
      pp = l.parent = p.parent;
      if (pp === null) { root = l; l.red = false; }
      else if (pp.right === p) pp.right = l;
      else pp.left = l;
      l.right = p;
      p.parent = l;
    }
    return root;
  }

  function balanceInsertion(root, x) {
    x.red = true;
    for (;;) {
      var xp = x.parent, xpp, xppl, xppr;
      if (xp === null) { x.red = false; return x; }
      if (!xp.red || xp.parent === null) return root;
      xpp = xp.parent;
      xppl = xpp.left;
      if (xp === xppl) {
        xppr = xpp.right;
        if (xppr !== null && xppr.red) {
          xppr.red = false; xp.red = false; xpp.red = true; x = xpp;
        } else {
          if (x === xp.right) {
            x = xp;
            root = rotateLeft(root, x);
            xp = x.parent;
            xpp = xp === null ? null : xp.parent;
          }
          if (xp !== null) {
            xp.red = false;
            if (xpp !== null) { xpp.red = true; root = rotateRight(root, xpp); }
          }
        }
      } else {
        if (xppl !== null && xppl.red) {
          xppl.red = false; xp.red = false; xpp.red = true; x = xpp;
        } else {
          if (x === xp.left) {
            x = xp;
            root = rotateRight(root, x);
            xp = x.parent;
            xpp = xp === null ? null : xp.parent;
          }
          if (xp !== null) {
            xp.red = false;
            if (xpp !== null) { xpp.red = true; root = rotateLeft(root, xpp); }
          }
        }
      }
    }
  }

  function balanceDeletion(root, x) {
    for (;;) {
      if (x === null || x === root) return root;
      var xp = x.parent, xpl, xpr, sl, sr;
      if (xp === null) { x.red = false; return x; }
      if (x.red) { x.red = false; return root; }
      xpl = xp.left;
      if (xpl === x) {
        xpr = xp.right;
        if (xpr !== null && xpr.red) {
          xpr.red = false; xp.red = true;
          root = rotateLeft(root, xp);
          xp = x.parent;
          xpr = xp === null ? null : xp.right;
        }
        if (xpr === null) x = xp;
        else {
          sl = xpr.left; sr = xpr.right;
          if ((sr === null || !sr.red) && (sl === null || !sl.red)) {
            xpr.red = true; x = xp;
          } else {
            if (sr === null || !sr.red) {
              if (sl !== null) sl.red = false;
              xpr.red = true;
              root = rotateRight(root, xpr);
              xp = x.parent;
              xpr = xp === null ? null : xp.right;
            }
            if (xpr !== null) {
              xpr.red = xp === null ? false : xp.red;
              sr = xpr.right;
              if (sr !== null) sr.red = false;
            }
            if (xp !== null) { xp.red = false; root = rotateLeft(root, xp); }
            x = root;
          }
        }
      } else {
        if (xpl !== null && xpl.red) {
          xpl.red = false; xp.red = true;
          root = rotateRight(root, xp);
          xp = x.parent;
          xpl = xp === null ? null : xp.left;
        }
        if (xpl === null) x = xp;
        else {
          sl = xpl.left; sr = xpl.right;
          if ((sl === null || !sl.red) && (sr === null || !sr.red)) {
            xpl.red = true; x = xp;
          } else {
            if (sl === null || !sl.red) {
              if (sr !== null) sr.red = false;
              xpl.red = true;
              root = rotateLeft(root, xpl);
              xp = x.parent;
              xpl = xp === null ? null : xp.left;
            }
            if (xpl !== null) {
              xpl.red = xp === null ? false : xp.red;
              sl = xpl.left;
              if (sl !== null) sl.red = false;
            }
            if (xp !== null) { xp.red = false; root = rotateRight(root, xp); }
            x = root;
          }
        }
      }
    }
  }

  function removeFrom(arr, k) {
    var i = arr.indexOf(k);
    if (i >= 0) arr.splice(i, 1);
  }

  /** A treeified bin: `order` is the bin's `next` chain (the same array as the table bin). */
  class TreeBin {
    constructor(order, hfun) {
      this.order = order;
      this.nodes = new Map();
      var root = null;
      for (var idx = 0; idx < order.length; idx++) {
        var k = order[idx];
        var x = new TreeNode(k, hfun(k) | 0);
        this.nodes.set(k, x);
        if (root === null) { root = x; continue; }
        var p = root;
        for (;;) {
          var d = tnDir(x.hash, k, p);
          var xp = p;
          p = d <= 0 ? p.left : p.right;
          if (p === null) {
            x.parent = xp;
            if (d <= 0) xp.left = x; else xp.right = x;
            root = balanceInsertion(root, x);
            break;
          }
        }
      }
      this._moveRootToFront(root);
    }
    _root() {
      var r = this.nodes.get(this.order[0]);
      while (r.parent !== null) r = r.parent;
      return r;
    }
    _moveRootToFront(root) {
      if (root !== null && this.order[0] !== root.key) {
        removeFrom(this.order, root.key);
        this.order.unshift(root.key);
      }
    }
    put(k, h) {
      h = h | 0;
      var root = this._root();
      var p = root;
      for (;;) {
        var d = tnDir(h, k, p);
        var xp = p;
        p = d <= 0 ? p.left : p.right;
        if (p === null) {
          var x = new TreeNode(k, h);
          this.nodes.set(k, x);
          if (d <= 0) xp.left = x; else xp.right = x;
          x.parent = xp;
          this.order.splice(this.order.indexOf(xp.key) + 1, 0, k);
          this._moveRootToFront(balanceInsertion(root, x));
          return;
        }
      }
    }
    /** removeTreeNode; returns true when the bin must be untreeified. */
    remove(k) {
      var root = this._root();
      var p = this.nodes.get(k);
      this.nodes.delete(k);
      removeFrom(this.order, k);
      if (this.order.length === 0) return true;
      if (root.right === null || root.left === null || root.left.left === null) return true;
      var pl = p.left, pr = p.right, replacement, pp;
      if (pl !== null && pr !== null) {
        var s = pr;
        while (s.left !== null) s = s.left;
        var c = s.red; s.red = p.red; p.red = c;
        var sr = s.right;
        pp = p.parent;
        if (s === pr) {
          p.parent = s;
          s.right = p;
        } else {
          var sp = s.parent;
          p.parent = sp;
          if (sp !== null) {
            if (s === sp.left) sp.left = p; else sp.right = p;
          }
          s.right = pr;
          if (pr !== null) pr.parent = s;
        }
        p.left = null;
        p.right = sr;
        if (sr !== null) sr.parent = p;
        s.left = pl;
        if (pl !== null) pl.parent = s;
        s.parent = pp;
        if (pp === null) root = s;
        else if (p === pp.left) pp.left = s;
        else pp.right = s;
        replacement = sr !== null ? sr : p;
      } else if (pl !== null) replacement = pl;
      else if (pr !== null) replacement = pr;
      else replacement = p;
      if (replacement !== p) {
        pp = replacement.parent = p.parent;
        if (pp === null) { root = replacement; replacement.red = false; }
        else if (p === pp.left) pp.left = replacement;
        else pp.right = replacement;
        p.left = p.right = p.parent = null;
      }
      var r = p.red ? root : balanceDeletion(root, replacement);
      if (replacement === p) {
        pp = p.parent;
        p.parent = null;
        if (pp !== null) {
          if (p === pp.left) pp.left = null;
          else if (p === pp.right) pp.right = null;
        }
      }
      this._moveRootToFront(r);
      return false;
    }
  }

  function intHash(k) {
    var h = k >>> 0;
    return (h ^ (h >>> 16)) >>> 0;
  }

  /** Iteration-order emulation of java.util.HashSet<Integer> (mppflow._java.JavaIntHashSet). */
  class JavaIntHashSet {
    constructor(items) {
      this._cap = 0;
      this._tab = null;
      this._size = 0;
      this._thr = 0;
      this._trees = new Map();
      if (items) for (var k of items) this.add(k);
    }
    _resize() {
      var ncap = this._cap === 0 ? 16 : this._cap * 2;
      var ntab = new Array(ncap);
      for (var i = 0; i < ncap; i++) ntab[i] = [];
      var ntrees = new Map();
      if (this._tab !== null) {
        var mask = ncap - 1, bit = this._cap;
        for (var j = 0; j < this._tab.length; j++) {
          var b = this._tab[j];
          var tree = this._trees.get(j);
          if (tree === undefined) {
            for (var q = 0; q < b.length; q++) ntab[intHash(b[q]) & mask].push(b[q]);
            continue;
          }
          var lo = [], hi = [];
          for (var q2 = 0; q2 < b.length; q2++) {
            if (intHash(b[q2]) & bit) hi.push(b[q2]); else lo.push(b[q2]);
          }
          var parts = [[j, lo, hi], [j + bit, hi, lo]];
          for (var pi = 0; pi < 2; pi++) {
            var idx = parts[pi][0], half = parts[pi][1], other = parts[pi][2];
            if (half.length === 0) continue;
            if (half.length <= 6) ntab[idx] = half;
            else if (other.length === 0) { ntab[idx] = tree.order; ntrees.set(idx, tree); }
            else { ntrees.set(idx, new TreeBin(half, intHash)); ntab[idx] = half; }
          }
        }
      }
      this._tab = ntab; this._cap = ncap; this._thr = Math.floor(ncap * 0.75); this._trees = ntrees;
    }
    add(k) {
      if (this._tab === null) this._resize();
      var idx = intHash(k) & (this._cap - 1);
      var b = this._tab[idx];
      if (b.indexOf(k) >= 0) return false;
      var tree = this._trees.get(idx);
      if (tree !== undefined) tree.put(k, intHash(k));
      else {
        b.push(k);
        if (b.length >= 9) {
          if (this._cap < 64) this._resize();
          else this._trees.set(idx, new TreeBin(b, intHash));
        }
      }
      this._size += 1;
      if (this._size > this._thr) this._resize();
      return true;
    }
    remove(k) {
      if (this._tab === null) return false;
      var idx = intHash(k) & (this._cap - 1);
      var b = this._tab[idx];
      if (b.indexOf(k) < 0) return false;
      var tree = this._trees.get(idx);
      if (tree === undefined) removeFrom(b, k);
      else if (tree.remove(k)) this._trees.delete(idx);
      this._size -= 1;
      return true;
    }
    has(k) {
      if (this._tab === null) return false;
      return this._tab[intHash(k) & (this._cap - 1)].indexOf(k) >= 0;
    }
    get size() { return this._size; }
    toList() {
      var out = [];
      if (this._tab === null) return out;
      for (var j = 0; j < this._tab.length; j++) {
        var b = this._tab[j];
        for (var q = 0; q < b.length; q++) out.push(b[q]);
      }
      return out;
    }
    [Symbol.iterator]() { return this.toList()[Symbol.iterator](); }
  }

  /** Double.toString (JDK 17) for 1e-4 <= |d| < 1e16, plus 0, NaN and infinities. */
  function javaDoubleStr(d) {
    if (d !== d) return 'NaN';
    if (d === Infinity) return 'Infinity';
    if (d === -Infinity) return '-Infinity';
    if (d === 0) return (1 / d < 0) ? '-0.0' : '0.0';
    var sign = d < 0 ? '-' : '';
    var a = Math.abs(d);
    var e = a.toExponential();            // shortest round-trip digits, d.ddde[+-]x
    var parts = e.split('e');
    var digits = parts[0].replace('.', '').replace(/0+$/, '') || '0';
    var pt = parseInt(parts[1], 10) + 1;  // position of the decimal point
    var s;
    if (a >= 1e-3 && a < 1e7) {
      if (pt <= 0) s = '0.' + '0'.repeat(-pt) + digits;
      else if (pt >= digits.length) s = digits + '0'.repeat(pt - digits.length) + '.0';
      else s = digits.slice(0, pt) + '.' + digits.slice(pt);
      return sign + s;
    }
    var frac = digits.slice(1) || '0';
    return sign + digits[0] + '.' + frac + 'E' + (pt - 1);
  }

  /** Python's round(): half to even. */
  function pyRound(x) {
    var f = Math.floor(x);
    var diff = x - f;
    if (diff > 0.5) return f + 1;
    if (diff < 0.5) return f;
    return (f % 2 === 0) ? f : f + 1;
  }

  MPP.i32 = i32;
  MPP.JavaRandom = JavaRandom;
  MPP.MathRandom = MathRandom;
  MPP.defaultRng = defaultRng;
  MPP.seedDefaultRng = seedDefaultRng;
  MPP.JavaIntHashSet = JavaIntHashSet;
  MPP.javaDoubleStr = javaDoubleStr;
  MPP.pyRound = pyRound;

  // ===================================================================== graphs (graph.py)
  /*
   * A graph is {n, adj, xy, rows, cols}: vertices 0..n-1, adj[v] the ascending neighbour list
   * including v itself (the wait self-loop), xy[v] = [x, y] (x = column, y = row) or null.
   * rows/cols are the grid dimensions for drawing (0 for a graph that is not a grid); mppflow's
   * convert_graph resets them to 0, but no engine function reads them. An optional adjOrder[v]
   * overrides the neighbour iteration order (mppflow Graph.adj_order).
   */
  function makeGraph(adj, xy, rows, cols) {
    return { n: adj.length, adj: adj, xy: xy, rows: rows || 0, cols: cols || 0 };
  }

  function neighbors(g, v) {
    if (g.adjOrder) {
      var o = g.adjOrder[v];
      if (o) return o;
    }
    var a = g.adj[v];
    return a === undefined ? null : a;
  }

  // as_int / as_ids (mppflow.graph): a vertex id (or k) is an integer number; strings, booleans,
  // fractional and non-finite numbers raise TypeError (a string id used to be solved and returned in
  // the plan, a fractional or boolean one gave a false Infeasible)
  function asInt(x, what) {
    if (typeof x !== 'number' || !Number.isInteger(x)) {
      var shown = x === null || x === undefined ? String(x) :
        (Array.isArray(x) ? 'array' : typeof x) + ' ' + (typeof x === 'string' ? JSON.stringify(x) : String(x));
      throw new TypeError(what + ' must be an integer, got ' + shown);
    }
    return x;
  }

  function asIds(seq, what) {
    var a = Array.from(seq);
    for (var i = 0; i < a.length; i++) asInt(a[i], 'each of the ' + what);
    return a;
  }

  function sortedUnique(list) {
    var a = list.slice().sort(function (p, q) { return p - q; });
    var out = [];
    for (var i = 0; i < a.length; i++) if (i === 0 || a[i] !== a[i - 1]) out.push(a[i]);
    return out;
  }

  /** Graph.from_adjacency: ids 0..n-1, a self-loop added to every vertex, no reverse edges. */
  function fromAdjacency(adj, xy) {
    var n = adj.length, A = new Array(n), P = new Array(n);
    for (var v = 0; v < n; v++) {
      A[v] = sortedUnique([v].concat(asIds(adj[v], 'neighbours')));
      var p = xy ? xy[v] : null;
      P[v] = (p === null || p === undefined) ? null : [p[0], p[1]];
    }
    return makeGraph(A, P, 0, 0);
  }

  /** create_2d_grid_graph(rows, cols) (converted): vertex y*cols+x at (x, y), 4-connected. */
  function grid(rows, cols) {
    var n = rows * cols, adj = new Array(n), xy = new Array(n);
    for (var y = 0; y < rows; y++) {
      for (var x = 0; x < cols; x++) {
        var v = y * cols + x, nb = [];
        if (y > 0) nb.push(v - cols);
        if (x > 0) nb.push(v - 1);
        nb.push(v);
        if (x < cols - 1) nb.push(v + 1);
        if (y < rows - 1) nb.push(v + cols);
        adj[v] = nb;
        xy[v] = [x, y];
      }
    }
    return makeGraph(adj, xy, rows, cols);
  }

  // A raw (unconverted) grid as the Java code builds it: 1-based ids y*cols+x+1.
  function rawGrid(rows, cols) {
    var N = rows * cols + 1;
    var adj = new Array(N), present = new Uint8Array(N);
    for (var i = 0; i < rows; i++) {
      for (var j = 0; j < cols; j++) {
        var vid = i * cols + j + 1, nb = [];
        if (i > 0) nb.push(vid - cols);
        if (j > 0) nb.push(vid - 1);
        nb.push(vid);
        if (j < cols - 1) nb.push(vid + 1);
        if (i < rows - 1) nb.push(vid + cols);
        adj[vid] = nb;
        present[vid] = 1;
      }
    }
    return { rows: rows, cols: cols, adj: adj, present: present, count: rows * cols };
  }

  // Graph.are_vertices_connected_without_v3 on a raw grid
  function rawConnectedWithout(G, v1, v2, v3) {
    var visited = new Set([v3]);
    var queue = [v1], head = 0;
    while (head < queue.length) {
      var v = queue[head++];
      var nb = G.adj[v];
      if (nb === undefined) continue;
      for (var k = 0; k < nb.length; k++) {
        var u = nb[k];
        if (!visited.has(u)) {
          if (u === v2) return true;
          queue.push(u);
          visited.add(u);
        }
      }
    }
    return false;
  }

  function rawRemovable(G, vid) {
    var nb = G.adj[vid].filter(function (u) { return u !== vid; });
    if (nb.length === 1) return true;
    for (var i = 0; i < nb.length - 1; i++) {
      if (!rawConnectedWithout(G, nb[i], nb[i + 1], vid)) return false;
    }
    return true;
  }

  // _holes (fixed): remove (int)(rows*cols*percent) random vertices, keeping the grid connected
  function rawHoles(G, rows, cols, percent, rng) {
    rng = resolveRng(rng);
    var toRemove = Math.trunc((rows * cols) * percent);
    if (toRemove > G.count) {
      throw valueError('cannot remove ' + toRemove + ' vertices from a graph with ' + G.count +
                       ' vertices (percent_obstacle too large)');
    }
    while (toRemove > 0) {
      var y = Math.trunc(rng.random() * rows);
      var x = Math.trunc(rng.random() * cols);
      var vid = y * cols + x + 1;
      if (G.present[vid] && rawRemovable(G, vid)) {
        G.present[vid] = 0;
        G.count -= 1;
        var nb = G.adj[vid];
        for (var k = 0; k < nb.length; k++) {
          var u = nb[k];
          if (u !== vid) removeFrom(G.adj[u], vid);
        }
        G.adj[vid] = undefined;
        toRemove -= 1;
      }
    }
    return G;
  }

  // get_random_start_goal on a raw grid (cols != 0)
  function rawRandomStartGoal(G, number, rng) {
    rng = resolveRng(rng);
    if (number < 0) throw valueError('number must be >= 0');
    if (number > G.count) {
      throw valueError('cannot draw ' + number + ' distinct starts/goals from ' + G.count + ' vertices');
    }
    var startSet = new Set(), goalSet = new Set(), starts = [], goals = [];
    for (var a = 0; a < number; a++) {
      for (;;) {
        var x = Math.trunc(rng.random() * G.cols);
        var y = Math.trunc(rng.random() * G.rows);
        var vid = y * G.cols + x + 1;
        if (startSet.has(vid) || !G.present[vid]) continue;
        var x2 = Math.trunc(rng.random() * G.cols);
        var y2 = Math.trunc(rng.random() * G.rows);
        var vid2 = y2 * G.cols + x2 + 1;
        if (goalSet.has(vid2) || !G.present[vid2]) continue;
        startSet.add(vid); goalSet.add(vid2); starts.push(vid); goals.push(vid2);
        break;
      }
    }
    return [starts, goals];
  }

  // convert_graph: renumber 0..n-1 by ascending old id (starts/goals renumbered in place)
  function rawConvert(G, starts, goals) {
    var index = new Map(), ids = [];
    for (var vid = 1; vid < G.present.length; vid++) {
      if (G.present[vid]) { index.set(vid, ids.length); ids.push(vid); }
    }
    var adj = new Array(ids.length), xy = new Array(ids.length);
    for (var i = 0; i < ids.length; i++) {
      var old = ids[i];
      xy[i] = [(old - 1) % G.cols, Math.floor((old - 1) / G.cols)];
      adj[i] = sortedUnique(G.adj[old].map(function (u) { return index.get(u); }));
    }
    for (var k = 0; k < (starts ? starts.length : 0); k++) {
      starts[k] = index.get(starts[k]);
      goals[k] = index.get(goals[k]);
    }
    return makeGraph(adj, xy, G.rows, G.cols);
  }

  /** create_generic_2d_grid_graph_with_holes(rows, cols, percent, rng), converted to 0..n-1. */
  function gridWithHoles(rows, cols, percent, rng) {
    return rawConvert(rawHoles(rawGrid(rows, cols), rows, cols, percent, rng), null, null);
  }

  /**
   * create_grid_problem(col, row, fracObs, n, rng): as in the original, the grid has `col`
   * rows and `row` columns. Returns {graph, starts, goals}.
   */
  function createGridProblem(col, row, fracObs, n, rng, opts) {
    var asNamed = opts && opts.asNamed;
    var rows = asNamed ? row : col, cols = asNamed ? col : row;
    rng = resolveRng(rng);
    var G = rawHoles(rawGrid(rows, cols), rows, cols, fracObs, rng);
    var sg = rawRandomStartGoal(G, n, rng);
    var g = rawConvert(G, sg[0], sg[1]);
    return { graph: g, starts: sg[0], goals: sg[1] };
  }

  /** get_random_start_goal_many(g, number, rng): starts 0..number-1, goals drawn without replacement. */
  function getRandomStartGoalMany(g, number, rng) {
    rng = resolveRng(rng);
    if (number < 0) throw valueError('number must be >= 0');
    if (number > g.n) throw valueError('cannot assign ' + number + ' agents on ' + g.n + ' vertices');
    var pool = [], starts = [], goals = [];
    for (var v = 0; v < g.n; v++) pool.push(v);
    for (var a = 0; a < number; a++) {
      var idx = Math.trunc(rng.random() * pool.length);
      goals.push(pool.splice(idx, 1)[0]);
      starts.push(a);
    }
    return [starts, goals];
  }

  /** create_n2_puzzle(n, rng): the fully occupied n x n puzzle (n^2 draws). */
  function createN2Puzzle(n, rng) {
    var g = grid(n, n);
    var sg = getRandomStartGoalMany(g, n * n, rng);
    return { graph: g, starts: sg[0], goals: sg[1] };
  }

  MPP.makeGraph = makeGraph;
  MPP.neighbors = neighbors;
  MPP.fromAdjacency = fromAdjacency;
  MPP.grid = grid;
  MPP.gridWithHoles = gridWithHoles;
  MPP.createGridProblem = createGridProblem;
  MPP.getRandomStartGoalMany = getRandomStartGoalMany;
  MPP.createN2Puzzle = createN2Puzzle;

  // ===================================================================== pathfinder.py
  /**
   * get_distances_to_all_vertices(g, start) (fixed): BFS hop distances; unreachable vertices
   * keep 0 unless `unreachable` is given.
   */
  function bfsDistances(g, start, unreachable) {
    var n = g.n, dist = new Array(n).fill(0);
    if (!(start >= 0 && start < n)) throw valueError('vertex ' + start + ' out of range 0..' + (n - 1));
    dist[start] = 0;
    var visited = new Uint8Array(n);
    var queue = [start], head = 0;
    while (head < queue.length) {
      var v = queue[head++];
      var nb = neighbors(g, v);
      if (nb === null) continue;
      var dv = dist[v] + 1;
      for (var k = 0; k < nb.length; k++) {
        var u = nb[k];
        if (!(u >= 0 && u < n)) throw valueError('vertex ' + u + ' out of range 0..' + (n - 1));
        if (!visited[u]) {
          visited[u] = 1;
          queue.push(u);
          if (u !== start) dist[u] = dv;
        } else if (dist[u] > dv) {
          dist[u] = dv;
        }
      }
    }
    if (unreachable !== undefined && unreachable !== null) {
      for (var i = 0; i < n; i++) if (i !== start && !visited[i]) dist[i] = unreachable;
    }
    return dist;
  }

  /** find_goals_in_distance_range: BFS layers max(dMin,1)..dMax, in HashSet<Integer> order. */
  function findGoalsInDistanceRange(g, start, dMin, dMax) {
    var vset = new JavaIntHashSet();
    var visited = new JavaIntHashSet([start]);
    var current = new JavaIntHashSet([start]);
    var tobe = new JavaIntHashSet();
    var distance = 1, n = g.n;
    while (distance <= dMax && visited.size < n) {
      var cur = current.toList();
      for (var i = 0; i < cur.length; i++) {
        var v = cur[i];
        var nb = neighbors(g, v);
        if (nb === null) continue;
        for (var k = 0; k < nb.length; k++) {
          var u = nb[k];
          if (u !== v && !visited.has(u)) {
            tobe.add(u);
            if (distance >= dMin) vset.add(u);
          }
        }
      }
      distance += 1;
      current = tobe;
      tobe = new JavaIntHashSet();
      var cl = current.toList();
      for (var j = 0; j < cl.length; j++) visited.add(cl[j]);
    }
    return vset.toList();
  }

  /** find_one_shortest_path(g, start, goal, rng): random tie-breaking BFS, the original's draws. */
  function findOneShortestPath(g, start, goal, rng) {
    if (start === goal) return [start];
    rng = resolveRng(rng);
    var visited = new Set();
    var parent = new Map();
    var queue = [start], head = 0;
    var found = false;
    while (head < queue.length) {
      var v = queue[head++];
      var nb = neighbors(g, v);
      if (nb === null) continue;
      var nvec = [];
      for (var k = 0; k < nb.length; k++) if (nb[k] !== v) nvec.push(nb[k]);
      while (nvec.length > 0) {
        var idx = Math.trunc(nvec.length * rng.random());
        var u = nvec.splice(idx, 1)[0];
        if (!visited.has(u)) {
          parent.set(u, v);
          if (u === goal) { found = true; head = queue.length; break; }
          queue.push(u);
          visited.add(u);
        }
      }
    }
    if (!found) return null;
    var path = [goal], last = goal;
    for (;;) {
      var par = parent.get(last);
      path.push(par);
      last = par;
      if (last === start) break;
    }
    path.reverse();
    return path;
  }

  /** find_shortest_paths: one random shortest path per robot, in robot order (null if unreachable). */
  function findShortestPaths(g, starts, goals, rng) {
    rng = resolveRng(rng);
    var out = [];
    for (var i = 0; i < starts.length; i++) out.push(findOneShortestPath(g, starts[i], goals[i], rng));
    return out;
  }

  /** get_makespan_lower_bound: the largest NUMBER OF VERTICES on a shortest path (draws). */
  function getMakespanLowerBound(g, starts, goals, rng) {
    var paths = findShortestPaths(g, starts, goals, rng), lb = 0;
    for (var i = 0; i < paths.length; i++) {
      if (paths[i] === null) throw new Infeasible('goal of robot ' + i + ' is unreachable from its start');
      if (paths[i].length > lb) lb = paths[i].length;
    }
    return lb;
  }

  /** get_total_time_lower_bound: the sum of the shortest-path lengths (draws). */
  function getTotalTimeLowerBound(g, starts, goals, rng) {
    var paths = findShortestPaths(g, starts, goals, rng), tt = 0;
    for (var i = 0; i < paths.length; i++) {
      if (paths[i] === null) throw new Infeasible('goal of robot ' + i + ' is unreachable from its start');
      tt += paths[i].length - 1;
    }
    return tt;
  }

  /**
   * The lower bounds of mppflow.planner.PlanResult (BFS, no draws):
   * {makespan, total_time, total_distance, max_distance} (all null if a goal is unreachable).
   */
  function lowerBounds(g, starts, goals) {
    var d = [];
    for (var i = 0; i < starts.length; i++) {
      var dist = bfsDistances(g, starts[i], -1), t = goals[i];
      d.push(t >= 0 && t < dist.length && dist[t] >= 0 ? dist[t] : null);
    }
    if (d.some(function (x) { return x === null; })) {
      return { makespan: null, total_time: null, total_distance: null, max_distance: null };
    }
    var mx = 0, sm = 0;
    for (var j = 0; j < d.length; j++) { if (d[j] > mx) mx = d[j]; sm += d[j]; }
    return { makespan: mx, total_time: sm, total_distance: sm, max_distance: mx };
  }

  MPP.bfsDistances = bfsDistances;
  MPP.findGoalsInDistanceRange = findGoalsInDistanceRange;
  MPP.findOneShortestPath = findOneShortestPath;
  MPP.findShortestPaths = findShortestPaths;
  MPP.getMakespanLowerBound = getMakespanLowerBound;
  MPP.getTotalTimeLowerBound = getTotalTimeLowerBound;
  MPP.lowerBounds = lowerBounds;

  // ===================================================================== network.py
  // _reversed_adjacency: predecessor lists, or null when the adjacency is symmetric
  function reversedAdjacency(g) {
    var n = g.n, arcs = new Set(), u, k, w, nb;
    for (u = 0; u < n; u++) {
      nb = g.adj[u];
      if (!nb) continue;
      for (k = 0; k < nb.length; k++) {
        w = nb[k];
        if (!(w >= 0 && w < n)) return null;
        arcs.add(u * n + w);
      }
    }
    var symmetric = true;
    for (var key of arcs) {
      u = Math.floor(key / n); w = key - u * n;
      if (!arcs.has(w * n + u)) { symmetric = false; break; }
    }
    if (symmetric) return null;
    var rev = new Array(n);
    for (u = 0; u < n; u++) rev[u] = [];
    for (u = 0; u < n; u++) {
      nb = g.adj[u];
      if (!nb) continue;
      var srt = sortedUnique(nb);
      for (k = 0; k < srt.length; k++) rev[srt[k]].push(u);
    }
    return rev;
  }

  // _distances_to: dist(v, goal) over predecessor lists (0 where goal is unreachable)
  function distancesTo(rev, goal, n) {
    if (!(goal >= 0 && goal < n)) throw valueError('vertex ' + goal + ' out of range 0..' + (n - 1));
    var dist = new Array(n).fill(0), seen = new Uint8Array(n);
    seen[goal] = 1;
    var queue = [goal], head = 0;
    while (head < queue.length) {
      var v = queue[head++], r = rev[v] || [];
      for (var k = 0; k < r.length; k++) {
        var u = r[k];
        if (!seen[u]) { seen[u] = 1; dist[u] = dist[v] + 1; queue.push(u); }
      }
    }
    return dist;
  }

  /**
   * update_reachable_sets (fixed): reach.reachable(a, v, t) iff distStart[a][v] <= t and
   * distGoal[a][v] <= T - t.
   */
  function reachability(g, starts, goals, T) {
    if (T < 0) throw valueError('horizon T must be >= 0, got ' + T);
    var rev = reversedAdjacency(g), ds = [], dg = [];
    for (var a = 0; a < starts.length; a++) {
      ds.push(bfsDistances(g, starts[a]));
      dg.push(rev === null ? bfsDistances(g, goals[a]) : distancesTo(rev, goals[a], g.n));
    }
    return {
      T: T, distStart: ds, distGoal: dg,
      reachable: function (a, v, t) { return ds[a][v] <= t && dg[a][v] <= T - t; },
    };
  }

  /**
   * compact_arcs: the arcs [a, u, v, t] of the pruned compact network in the original's column
   * order (agent-major; frontier HashSet<Integer> order; neighbour order).
   */
  function compactArcs(g, starts, reach, emit) {
    var T = reach.T, out = emit ? null : [];
    for (var a = 0; a < starts.length; a++) {
      var ds = reach.distStart[a], dg = reach.distGoal[a], n = ds.length;
      var rvs = new JavaIntHashSet([starts[a]]);
      for (var t = 1; t <= T; t++) {
        var lim = T - t, snap = rvs.toList();
        for (var i = 0; i < snap.length; i++) {
          var cur = snap[i], nb = neighbors(g, cur);
          if (nb === null) throw valueError('vertex ' + cur + ' has no adjacency entry');
          for (var k = 0; k < nb.length; k++) {
            var nxt = nb[k];
            if (!(nxt >= 0 && nxt < n)) throw valueError('vertex ' + nxt + ' out of range 0..' + (n - 1));
            if (ds[nxt] <= t && dg[nxt] <= lim) {
              rvs.add(nxt);
              if (emit) emit(a, cur, nxt, t - 1); else out.push([a, cur, nxt, t - 1]);
            }
          }
          if (!(cur >= 0 && cur < n)) throw valueError('vertex ' + cur + ' out of range 0..' + (n - 1));
          if (!(ds[cur] <= t && dg[cur] <= lim)) rvs.remove(cur);
        }
      }
    }
    return out;
  }

  /** undirected_edges: the edges [u, w] (u < w, no self-loops), ascending. */
  function undirectedEdges(g) {
    var keys = new Set(), n = g.n, out = [];
    for (var u = 0; u < n; u++) {
      var nb = g.adj[u] || [];
      for (var k = 0; k < nb.length; k++) {
        var w = nb[k];
        if (w !== u) keys.add(u < w ? u * n + w : w * n + u);
      }
    }
    var arr = Array.from(keys).sort(function (p, q) { return p - q; });
    for (var i = 0; i < arr.length; i++) out.push([Math.floor(arr[i] / n), arr[i] % n]);
    return out;
  }

  var FULL_KIND = { loop: 'loopback', green: 'green', blue: 'blue', g_in: 'gadget', g_mid: 'gadget', g_out: 'gadget' };
  var FULL_PART = { g_in: 'in', g_mid: 'mid', g_out: 'out' };
  var COMPACT_KIND = { loop: 'loopback', stay: 'green', move: 'gadget' };

  // FullNetwork.build(...).to_json()
  function fullNetwork(g, starts, goals, T) {
    if (T < 0) throw valueError('horizon T must be >= 0, got ' + T);
    var V = [], v, t, j;
    for (v = 0; v < g.n; v++) V.push(v);
    for (v = 0; v < g.n; v++) {  // the network is defined for undirected graphs with wait loops
      var nbv = g.adj[v] || [];
      if (nbv.indexOf(v) < 0) {
        throw valueError('the full network needs a wait loop at every vertex; vertex ' + v + ' has none (use the compact models)');
      }
      for (j = 0; j < nbv.length; j++) {
        if ((g.adj[nbv[j]] || []).indexOf(v) < 0) {
          throw valueError('the full network needs an undirected graph; the arc ' + v + ' -> ' + nbv[j] +
                           ' is one-way (use the compact models)');
        }
      }
    }
    var E = undirectedEdges(g);
    var nodes = [], index = new Map(), arcs = [];
    function node(keyStr, desc) {
      var i = index.get(keyStr);
      if (i === undefined) { i = nodes.length; index.set(keyStr, i); desc.id = i; nodes.push(desc); }
      return i;
    }
    function cp(vv, tt, prime) {
      if (tt === 0) return node('v' + vv + ',0,0', { id: 0, kind: 'copy', v: vv, t: 0, col: 0, row: vv });
      return node('v' + vv + ',' + tt + ',' + prime,
                  { id: 0, kind: prime ? "copy'" : 'copy', v: vv, t: tt, col: 2 * tt + (prime ? 0.5 : 0), row: vv });
    }
    function gnode(kind, u, w, tt) {
      return node(kind + u + ',' + w + ',' + tt, {
        id: 0, kind: kind === 'm' ? 'merge' : 'split', edge: [u, w], t: tt,
        col: 2 * tt + (kind === 'm' ? 1 : 1.5), row: (u + w) / 2 });
    }
    for (v = 0; v < V.length; v++) cp(V[v], 0, 0);
    for (t = 1; t <= T; t++) for (v = 0; v < V.length; v++) { cp(V[v], t, 0); cp(V[v], t, 1); }
    for (t = 0; t < T; t++) for (j = 0; j < E.length; j++) { gnode('m', E[j][0], E[j][1], t); gnode('s', E[j][0], E[j][1], t); }
    function arc(a, b, kind, pc, c, e) {
      var d = { id: arcs.length, tail: a, head: b, kind: FULL_KIND[kind], pyKind: kind, cost: c, paper_cost: pc };
      if (FULL_PART[kind]) d.part = FULL_PART[kind];
      if (e) d.edge = [e[0], e[1]];
      if (kind === 'loop') d.robot = arcs.length;
      arcs.push(d);
    }
    for (var r = 0; r < starts.length; r++) arc(cp(goals[r], T, 1), cp(starts[r], 0, 0), 'loop', 0, 0, null);
    for (t = 0; t < T; t++) {
      for (v = 0; v < V.length; v++) {
        var nb = g.adj[V[v]] || [];
        if (nb.indexOf(V[v]) >= 0) arc(cp(V[v], t, 1), cp(V[v], t + 1, 0), 'green', 1, 0, null);
      }
      for (j = 0; j < E.length; j++) {
        var u = E[j][0], w = E[j][1];
        var m = index.get('m' + u + ',' + w + ',' + t), s = index.get('s' + u + ',' + w + ',' + t);
        arc(cp(u, t, 1), m, 'g_in', 0, 0, E[j]);
        arc(cp(w, t, 1), m, 'g_in', 0, 0, E[j]);
        arc(m, s, 'g_mid', 1, 1, E[j]);
        arc(s, cp(u, t + 1, 0), 'g_out', 0, 0, E[j]);
        arc(s, cp(w, t + 1, 0), 'g_out', 0, 0, E[j]);
      }
    }
    for (t = 1; t <= T; t++) for (v = 0; v < V.length; v++) arc(cp(V[v], t, 0), cp(V[v], t, 1), 'blue', 0, 0, null);
    var byKind = { loop: 0, green: 0, blue: 0, g_in: 0, g_mid: 0, g_out: 0 };
    for (j = 0; j < arcs.length; j++) byKind[arcs[j].pyKind] += 1;
    return {
      type: 'full', T: T, starts: starts.slice(), goals: goals.slice(), vertices: V, edges: E,
      nodes: nodes, arcs: arcs,
      counts: { vertices_total: nodes.length, arcs_total: arcs.length, by_kind: byKind },
    };
  }

  // CompactNetwork.build(..., prune).to_json()
  function compactNetwork(g, starts, goals, T, prune) {
    if (T < 0) throw valueError('horizon T must be >= 0, got ' + T);
    var n = g.n, nodes = [], arcs = [], t, u, k;
    for (t = 0; t <= T; t++) for (u = 0; u < n; u++) nodes.push({ id: nodes.length, v: u, t: t, col: t, row: u });
    function idx(v, tt) { return tt * n + v; }
    var stay = 0, move = 0;
    for (t = 0; t < T; t++) {
      for (u = 0; u < n; u++) {
        var nb = neighbors(g, u) || [];
        for (k = 0; k < nb.length; k++) {
          var w = nb[k], kind = u === w ? 'stay' : 'move';
          if (u === w) stay++; else move++;
          arcs.push({ id: arcs.length, tail: idx(u, t), head: idx(w, t + 1), kind: COMPACT_KIND[kind], pyKind: kind,
                      cost: u === w ? 0 : 1 });
        }
      }
    }
    for (var r = 0; r < starts.length; r++) {
      arcs.push({ id: arcs.length, tail: idx(goals[r], T), head: idx(starts[r], 0), kind: 'loopback', pyKind: 'loop', robot: r });
    }
    var counts = {
      vertices: nodes.length, stay_arcs: stay, move_arcs: move, arcs_total: stay + move,
      loopback_arcs: starts.length, arcs_total_with_loopbacks: stay + move + starts.length,
      crossing_pairs_per_step: undirectedEdges(g).length,
    };
    var out = { type: 'compact', T: T, starts: starts.slice(), goals: goals.slice(), vertices: [],
                edges: undirectedEdges(g), nodes: nodes, arcs: arcs, counts: counts };
    for (u = 0; u < n; u++) out.vertices.push(u);
    if (prune) {
      var reach = reachability(g, starts, goals, T), per = [];
      for (var a = 0; a < starts.length; a++) per.push([]);
      compactArcs(g, starts, reach, function (a2, cu, nx, t0) { per[a2].push([idx(cu, t0), idx(nx, t0 + 1)]); });
      counts.pruned_arcs_per_agent = per.map(function (x) { return x.length; });
      out.agent_arcs = per;
    }
    return out;
  }

  /**
   * network(g, starts, goals, T, {variant: 'full' | 'compact', prune}): the time-expanded
   * network for drawing (mppflow.network FullNetwork / CompactNetwork .to_json()). Arc `kind`
   * is 'green' (wait), 'blue', 'gadget' (with `part` in/mid/out in the full network; the two
   * move arcs per edge in the compact one) or 'loopback'; `pyKind` is mppflow's kind.
   */
  function network(g, starts, goals, T, opts) {
    var variant = (opts && opts.variant) || 'full';
    if (variant === 'full') return fullNetwork(g, starts, goals, T);
    if (variant === 'compact') return compactNetwork(g, starts, goals, T, !!(opts && opts.prune));
    throw valueError("variant must be 'full' or 'compact'");
  }

  MPP.reachability = reachability;
  MPP.compactArcs = compactArcs;
  MPP.undirectedEdges = undirectedEdges;
  MPP.network = network;

  // ===================================================================== lp/linear.py
  var INF = Infinity;
  var SENSES = { '<=': '<=', '<': '<=', 'L': '<=', '==': '==', '=': '==', 'E': '==', '>=': '>=', '>': '>=', 'G': '>=' };

  /** mppflow.lp.linear.LinearModel: columns and rows in construction order. */
  class LinearModel {
    constructor(sense) {
      this.sense = sense === 'min' ? 'min' : 'max';
      this.objConstant = 0;
      this.lb = []; this.ub = []; this.obj = []; this.vtype = [];
      this.rowStart = [0]; this.termCol = []; this.termCoef = [];
      this.rowSense = []; this.rhs = [];
      this.objectiveTerms = null;
    }
    get numColumns() { return this.lb.length; }
    get numRows() { return this.rhs.length; }
    addColumn(lb, ub, vtype, obj) {
      var j = this.lb.length;
      obj = obj || 0;
      this.lb.push(+lb); this.ub.push(+ub); this.obj.push(+obj); this.vtype.push(vtype || 'C');
      if (this.objectiveTerms !== null && obj !== 0) this.objectiveTerms.push([j, obj]);
      return j;
    }
    addRow(terms, sense, rhs) {
      var s = SENSES[sense];
      if (!s) throw valueError('unknown row sense ' + sense);
      var n = this.lb.length;
      for (var k = 0; k < terms.length; k++) {
        var c = terms[k][0];
        if (!(c >= 0 && c < n)) throw valueError('row term refers to column ' + c);
        this.termCol.push(c); this.termCoef.push(+terms[k][1]);
      }
      this.rowStart.push(this.termCol.length);
      this.rowSense.push(s);
      this.rhs.push(+rhs);
      return this.rhs.length - 1;
    }
    setObjective(terms, sense, constant) {
      if (sense) this.sense = sense === 'min' ? 'min' : 'max';
      var obj = new Array(this.lb.length).fill(0);
      var tl = [];
      for (var k = 0; k < terms.length; k++) {
        obj[terms[k][0]] += +terms[k][1];
        tl.push([terms[k][0], +terms[k][1]]);
      }
      this.obj = obj;
      this.objConstant = +(constant || 0);
      this.objectiveTerms = tl;
    }
    row(i) {
      var out = [];
      for (var k = this.rowStart[i]; k < this.rowStart[i + 1]; k++) out.push([this.termCol[k], this.termCoef[k]]);
      return out;
    }
    mergedRow(i) {
      var acc = new Map();
      for (var k = this.rowStart[i]; k < this.rowStart[i + 1]; k++) {
        var c = this.termCol[k];
        acc.set(c, (acc.has(c) ? acc.get(c) : 0) + this.termCoef[k]);
      }
      var cols = Array.from(acc.keys()).sort(function (p, q) { return p - q; }), out = [];
      for (var q = 0; q < cols.length; q++) if (acc.get(cols[q]) !== 0) out.push([cols[q], acc.get(cols[q])]);
      return out;
    }
    get isMip() { return this.vtype.some(function (t) { return t !== 'C'; }); }
    copy() {
      var m = new LinearModel(this.sense);
      m.objConstant = this.objConstant;
      m.lb = this.lb.slice(); m.ub = this.ub.slice(); m.obj = this.obj.slice(); m.vtype = this.vtype.slice();
      m.rowStart = this.rowStart.slice(); m.termCol = this.termCol.slice(); m.termCoef = this.termCoef.slice();
      m.rowSense = this.rowSense.slice(); m.rhs = this.rhs.slice();
      m.objectiveTerms = this.objectiveTerms === null ? null : this.objectiveTerms.slice();
      return m;
    }
    /** The LP relaxation (GRBModel.relax()). */
    relax() {
      var m = this.copy();
      for (var j = 0; j < m.vtype.length; j++) {
        if (m.vtype[j] === 'B') { if (m.lb[j] < 0) m.lb[j] = 0; if (m.ub[j] > 1) m.ub[j] = 1; }
        m.vtype[j] = 'C';
      }
      return m;
    }
    objectiveValue(x) {
      var v = this.objConstant;
      for (var j = 0; j < this.obj.length; j++) if (this.obj[j] !== 0) v += this.obj[j] * x[j];
      return v;
    }
    /** LinearModel.to_canonical_text(names): the golden canonical model dump. */
    canonicalText(names) {
      var out = ['SENSE ' + this.sense];
      if (this.objConstant !== 0) out.push('OBJCONST ' + goldenNumber(this.objConstant));
      var n = this.lb.length;
      out.push('NCOLS ' + n, 'NROWS ' + this.rhs.length);
      for (var j = 0; j < n; j++) {
        var line = 'C' + j + ' ' + goldenNumber(this.lb[j]) + ' ' + goldenNumber(this.ub[j]) + ' ' + this.vtype[j] +
                   ' ' + goldenNumber(this.obj[j]);
        if (names === undefined || names === true) line += ' C' + j;
        else if (names !== false && names !== null) line += ' ' + names[j];
        out.push(line);
      }
      for (var i = 0; i < this.rhs.length; i++) {
        var s = this.rowSense[i] === '==' ? '=' : this.rowSense[i];
        var l = 'R' + i + ' ' + s + ' ' + goldenNumber(this.rhs[i]) + ' :';
        var terms = this.mergedRow(i);
        for (var k = 0; k < terms.length; k++) l += ' (' + terms[k][0] + ',' + goldenNumber(terms[k][1]) + ')';
        out.push(l);
      }
      return out.join('\n') + '\n';
    }
  }

  /** golden_number: integral values below 1e15 as integers, +-1e100 as (-)inf, else Double.toString. */
  function goldenNumber(x) {
    if (x >= 1e100) return 'inf';
    if (x <= -1e100) return '-inf';
    if (x === Math.trunc(x) && Math.abs(x) < 1e15) return String(x === 0 ? 0 : x);
    return javaDoubleStr(x);
  }

  /** relative_gap: |bound - objective| / |objective| (0 when both are 0, inf when only objective is 0). */
  function relativeGap(objective, bound) {
    var d = Math.abs(bound - objective);
    if (objective === 0) return d === 0 ? 0 : INF;
    return d / Math.abs(objective);
  }

  MPP.LinearModel = LinearModel;
  MPP.goldenNumber = goldenNumber;
  MPP.relativeGap = relativeGap;

  // ===================================================================== model.py
  function edgeTimeId(v1, v2, t) {
    if (v1 > v2) { var x = v1; v1 = v2; v2 = x; }
    return (v1 * 1000000000 + v2 * 10000 + t) | 0;
  }

  var KIND_ALIASES = { makespan: 'time', time: 'time', time_r: 'time_r', total_time: 'total_time',
                       total_distance: 'distance', distance: 'distance', max_distance: 'max_distance' };

  /** An ILP and the meaning of its columns (mppflow.model.MPPModel, fixed mode). */
  class MPPModel {
    constructor(f) { Object.assign(this, f); }
    get nAgents() { return this.starts.length; }
    edgeVar(a, u, v, t) {
      var j = this._ev.get(this._key(a, u, v, t));
      return j === undefined ? null : j;
    }
    columnNames() {
      var names = [], j, a;
      for (j = 0; j < this.arcs.length; j++) {
        var c = this.arcs[j];
        names.push('x[' + c[0] + ',' + c[1] + ',' + c[2] + ',' + c[3] + ']');
      }
      while (names.length < this.model.numColumns) names.push('');
      for (a = 0; a < this.loopbacks.length; a++) {
        names[this.loopbacks[a]] = 'x[' + a + ',' + this.goals[a] + ',' + this.starts[a] + ',' + this.T + ']';
      }
      if (this.yTotal !== null) {
        for (a = 0; a < this.yTotal.length; a++) names[this.yTotal[a]] = 'yT[' + a + ']';
        for (a = 0; a < this.y.length; a++) {
          for (var t = 0; t < this.y[a].length; t++) names[this.y[a][t]] = 'y[' + a + ',' + t + ']';
        }
      }
      if (this.xMax !== null) names[this.xMax] = 'x_max';
      return names;
    }
    /** mppflow.model.canonical_text of this model (with columnNames()). */
    canonicalText() { return this.model.canonicalText(this.columnNames()); }
    /** retrieve_paths (fixed): follow each robot along the first arc with x > 0.5. */
    retrievePaths(x, g) {
      var paths = [];
      for (var a = 0; a < this.starts.length; a++) {
        var p = [this.starts[a]];
        for (var t = 1; t <= this.T; t++) {
          var prev = p[p.length - 1], nb = neighbors(g, prev) || [], nxt = null;
          for (var k = 0; k < nb.length; k++) {
            var j = this.edgeVar(a, prev, nb[k], t - 1);
            if (j !== null && x[j] > 0.5) { nxt = nb[k]; break; }
          }
          if (nxt === null) {
            throw runtimeError('no arc of agent ' + a + ' leaves vertex ' + prev + ' at time ' + (t - 1) + ' in the solution');
          }
          p.push(nxt);
        }
        paths.push(p);
      }
      return paths;
    }
  }

  /**
   * buildModel(kind, g, starts, goals, T, {integer, setOptimal, reach}): the compact ILP of
   * mppflow.model (fixed mode), column for column and row for row. kind: 'makespan' (= 'time',
   * build_time_model), 'time_r', 'total_time', 'total_distance' (= 'distance'), 'max_distance'.
   */
  function buildModel(kindIn, g, starts, goals, T, opts) {
    opts = opts || {};
    var kind = KIND_ALIASES[kindIn];
    if (!kind) throw valueError('unknown model kind ' + kindIn);
    var integer = opts.integer === undefined ? true : !!opts.integer;
    var setOptimal = opts.setOptimal === undefined ? true : !!opts.setOptimal;
    if (kind === 'total_time' || kind === 'distance' || kind === 'max_distance') setOptimal = true;
    starts = Array.from(starts); goals = Array.from(goals);
    if (starts.length !== goals.length) throw valueError('starts and goals must have the same length');
    var reach = opts.reach;
    if (!reach) reach = reachability(g, starts, goals, T);
    else if (reach.T !== T) throw valueError('reachability was computed for T=' + reach.T + ', not T=' + T);
    var A = starts.length, n = g.n, T1 = T + 1, T2 = T + 2;
    var lm = new LinearModel('max');
    var vt = integer ? 'B' : 'C';
    function kE(a, u, v, t) { return ((a * n + u) * n + v) * T2 + t; }  // (a, u, v, t), tuple order
    function kVT(v, t) { return t * n + v; }                               // sorted as (t, v)
    function kAVT(a, v, t) { return (a * T1 + t) * n + v; }               // sorted as (a, t, v)
    var ev = new Map(), vin = new Map(), ain = new Map(), aout = new Map(), et = new Map();
    var arcs = [], moveTerms = [], movesOf = [];
    for (var a0 = 0; a0 < A; a0++) movesOf.push([]);
    function push(map, key, j) {
      var l = map.get(key);
      if (l === undefined) { l = []; map.set(key, l); }
      l.push(j);
    }

    // (1) arc columns, in the original's order
    compactArcs(g, starts, reach, function (a, cur, nxt, t0) {
      var t1 = t0 + 1;
      var j = lm.addColumn(0, 1, vt);
      arcs.push([a, cur, nxt, t0]);
      ev.set(kE(a, cur, nxt, t0), j);
      push(vin, kVT(nxt, t1), j);
      push(aout, kAVT(a, cur, t0), j);
      push(ain, kAVT(a, nxt, t1), j);
      if (cur !== nxt) {
        var lo = cur < nxt ? cur : nxt, hi = cur < nxt ? nxt : cur;
        var ek = (lo * n + hi) * T1 + t0;
        var e = et.get(ek);
        if (e === undefined) { e = { id: edgeTimeId(lo, hi, t0), u: lo, v: hi, t: t0, cols: [] }; et.set(ek, e); }
        e.cols.push(j);
        moveTerms.push([j, 1]);
        movesOf[a].push(j);
      }
    });

    // (TimeR) one move per time step, before the loopback columns
    if (kind === 'time_r') {
      var buckets = [];
      for (var tb = 0; tb <= T; tb++) buckets.push([]);
      var evKeys = Array.from(ev.keys()).sort(function (p, q) { return p - q; });
      for (var q = 0; q < evKeys.length; q++) {
        var key = evKeys[q], tt = key % T2, rest = (key - tt) / T2, vv = rest % n, uu = ((rest - vv) / n) % n;
        if (uu !== vv) buckets[tt].push(ev.get(key));
      }
      for (var tr = 0; tr <= T; tr++) lm.addRow(buckets[tr].map(function (j) { return [j, 1]; }), '<=', 1);
    }

    // (DIST) the objective is set before the loopbacks exist
    if (kind === 'distance') lm.setObjective(moveTerms, 'min');

    // (2) loopback columns
    var loopCols = [], a;
    if (kind === 'total_time') for (a = 0; a < A; a++) loopCols.push(lm.addColumn(1, 1, 'B'));
    for (a = 0; a < A; a++) {
      var j;
      if (kind === 'total_time') j = loopCols[a];
      else {
        var lb;
        if (kind === 'distance') lb = integer ? 1 : 0;
        else if (kind === 'max_distance') lb = 1;
        else lb = setOptimal ? 1 : 0;
        j = lm.addColumn(lb, 1, vt);
        loopCols.push(j);
      }
      var s = starts[a], gg = goals[a];
      push(vin, kVT(s, 0), j);
      push(aout, kAVT(a, gg, T), j);
      push(ain, kAVT(a, s, 0), j);
      ev.set(kE(a, gg, s, T), j);
    }
    if (kind === 'time' || kind === 'time_r') lm.setObjective(loopCols.map(function (c) { return [c, 1]; }), 'max');

    // (3) TotalTime's y variables and chain (eq. (14))
    var yTotal = null, y = null;
    if (kind === 'total_time') {
      yTotal = []; y = [];
      for (a = 0; a < A; a++) {
        yTotal.push(lm.addColumn(0, T, 'I'));
        var ya = new Array(T).fill(0);
        for (var t2 = T - 1; t2 >= 0; t2--) ya[t2] = lm.addColumn(0, 1, 'B');
        y.push(ya);
      }
      for (a = 0; a < A; a++) {
        var gl = goals[a], ytTerms = [[yTotal[a], 1]];
        for (var t3 = T - 1; t3 >= 0; t3--) {
          var xj = ev.get(kE(a, gl, gl, t3));
          if (xj !== undefined) {
            if (t3 === T - 1) lm.addRow([[y[a][t3], 1], [xj, -1]], '==', 0);
            else {
              lm.addRow([[y[a][t3], 1], [y[a][t3 + 1], -1], [xj, -1]], '>=', -1);
              lm.addRow([[y[a][t3], 1], [y[a][t3 + 1], -1]], '<=', 0);
              lm.addRow([[y[a][t3], 1], [xj, -1]], '<=', 0);
            }
            ytTerms.push([y[a][t3], -1]);
          }
        }
        lm.addRow(ytTerms, '==', 0);
      }
      lm.setObjective(yTotal.map(function (c) { return [c, 1]; }), 'max');
    }

    var xMax = null;
    if (kind === 'max_distance') {
      xMax = lm.addColumn(0, T, 'I');
      lm.setObjective([[xMax, 1]], 'min');
    }

    function sortedKeys(map) { return Array.from(map.keys()).sort(function (p, q) { return p - q; }); }
    function ones(list) { return list.map(function (c) { return [c, 1]; }); }

    // (a) vertex capacity (eq. (18)), (t, v) ascending, t >= 1
    var vk = sortedKeys(vin);
    for (var i = 0; i < vk.length; i++) {
      if (vk[i] < n) continue;  // t == 0
      var lst = vin.get(vk[i]);
      if (lst.length > 1) lm.addRow(ones(lst), '<=', 1);
    }

    // (b) flow conservation (eq. (9)), (a, t, v) ascending
    var avt = new Set();
    ain.forEach(function (_, k) { avt.add(k); });
    aout.forEach(function (_, k) { avt.add(k); });
    var ak = Array.from(avt).sort(function (p, q) { return p - q; });
    for (i = 0; i < ak.length; i++) {
      var ins = ain.get(ak[i]) || [], outs = aout.get(ak[i]) || [], terms = [];
      for (var x1 = 0; x1 < ins.length; x1++) terms.push([ins[x1], -1]);
      for (var x2 = 0; x2 < outs.length; x2++) terms.push([outs[x2], 1]);
      lm.addRow(terms, '==', 0);
    }

    // (c) head-on collisions (eq. (17)), in TreeMap order of the int32-wrapped keys
    var ets = Array.from(et.values()).sort(function (p, q) {
      return (p.id - q.id) || (p.u - q.u) || (p.v - q.v) || (p.t - q.t);
    });
    for (i = 0; i < ets.length; i++) if (ets[i].cols.length > 1) lm.addRow(ones(ets[i].cols), '<=', 1);

    // max distance (eqs. (12)-(13)): one row per agent
    if (kind === 'max_distance') {
      for (a = 0; a < A; a++) lm.addRow(ones(movesOf[a]).concat([[xMax, -1]]), '<=', 0);
    }

    return new MPPModel({
      model: lm, kind: kind, T: T, starts: starts, goals: goals, arcs: arcs, loopbacks: loopCols,
      yTotal: yTotal, y: y, xMax: xMax, reach: reach, _ev: ev, _key: kE,
    });
  }

  /** mppflow.model.canonical_text(model) of an MPPModel (or of a LinearModel with its C<j> names). */
  function canonicalText(m) {
    if (m instanceof MPPModel) return m.canonicalText();
    return m.canonicalText(true);
  }

  MPP.MPPModel = MPPModel;
  MPP.buildModel = buildModel;
  MPP.canonicalText = canonicalText;
  MPP.edgeTimeId = edgeTimeId;

  // ===================================================================== solvers.py (fixed mode)
  var DEFAULT_FEASIBILITY_BUDGET = 50000;
  var JOINT_SEARCH_BUDGET = 2000000;
  var JOINT_SEARCH_MAX_CONFIGS = 1000000;

  class BudgetExhausted extends Error {}

  function nowSec() {
    return (typeof performance !== 'undefined' && performance.now) ? performance.now() / 1000 : Date.now() / 1000;
  }

  function Clock() { this.t0 = nowSec(); }
  Clock.prototype.elapsed = function () { return nowSec() - this.t0; };

  function popcount(x) { var c = 0; while (x) { x &= x - 1; c++; } return c; }

  function cfgKey(cfg) { return cfg.join(','); }

  function attempt(T, phase, sol) {
    var rec = { T: T, phase: phase, status: sol.status, objective: sol.objective, bound: sol.bound, runtime: sol.runtime };
    if (sol.rawStatus === 'iteration_cap') rec.raw_status = sol.rawStatus;  // a stalled simplex (_attempt)
    return rec;
  }

  // _run: one model, through MPP.lp.solve (the builtin backend)
  function runModel(m, timeLimit, mipGap, lpOpts) {
    var o = Object.assign({}, lpOpts || {});
    if (timeLimit !== null && timeLimit !== undefined && timeLimit > 0) o.timeLimit = timeLimit;
    if (mipGap !== null && mipGap !== undefined) o.mipGap = mipGap;
    if (!MPP.lp) throw runtimeError('docs/js/lp.js is not loaded');
    return MPP.lp.solve(m.model, o);
  }

  function checkInstance(g, starts, goals) {
    if (starts.length !== goals.length) throw valueError('starts and goals must have the same length');
    var n = g.n, all = starts.concat(goals);
    for (var i = 0; i < all.length; i++) {
      if (!(all[i] >= 0 && all[i] < n)) throw valueError('vertex ' + all[i] + ' out of range 0..' + (n - 1));
    }
    if (new Set(starts).size < starts.length) throw valueError('two robots share a start vertex');
    if (new Set(goals).size < goals.length) throw new Infeasible('two robots share a goal vertex');
  }

  // _distances: BFS start -> goal along the (possibly one-way) adjacency, null if unreachable
  function robotDistances(g, starts, goals) {
    var out = [];
    for (var i = 0; i < starts.length; i++) {
      var d = bfsDistances(g, starts[i], -1);
      out.push(d[goals[i]] < 0 ? null : d[goals[i]]);
    }
    return out;
  }

  function nbCache(g) {
    var cache = new Map();
    return function (v) {
      var r = cache.get(v);
      if (r === undefined) { r = (neighbors(g, v) || []).slice(); cache.set(v, r); }
      return r;
    };
  }

  /**
   * joint_feasibility: is the compact makespan model feasible for SOME horizon T >= tFirst?
   * (true/false/null). Without a wait loop at some goal the feasible horizons need not be
   * upward-closed; the search then runs over (configuration, min(t, tFirst)).
   */
  function jointFeasibility(g, starts, goals, budget, oneMove, tFirst) {
    return jointBfs(g, starts, goals, budget, oneMove, tFirst)[0];
  }

  // _joint_bfs: [verdict, T, paths]: when the search reaches the goal configuration, T is the smallest
  // feasible horizon >= tFirst and paths a plan of makespan T (else both null)
  function jointBfs(g, starts, goals, budget, oneMove, tFirst) {
    if (budget === undefined) budget = DEFAULT_FEASIBILITY_BUDGET;
    tFirst = tFirst || 0;
    var A = starts.length, start = starts.slice(), goal = goals.slice();
    var goalKey = cfgKey(goal), same = cfgKey(start) === goalKey;
    var layered = false;
    if (tFirst > 0 && goal.some(function (v) { return (neighbors(g, v) || []).indexOf(v) < 0; })) {
      // every plan has T >= max(dist): a tFirst up to that bound restricts nothing
      var dm = 0;
      robotDistances(g, starts, goals).forEach(function (d) { if (d !== null && d > dm) dm = d; });
      layered = same || tFirst > dm;
    }
    if (same && !layered) return [true, null, null];
    if (new Set(goal).size < A) return [false, null, null];
    var dist = robotDistances(g, starts, goals);
    if (dist.some(function (d) { return d === null; })) return [false, null, null];
    if (new Set(start).size === A) {
      // polynomial certificates and the per-component search, before the joint search
      var und = undirectedOf(g), cc = componentsOf(und);
      if (structuralRefutation(und, cc, starts, goals) !== null ||
          componentRefutes(g, cc, starts, goals, budget, oneMove, tFirst)) return [false, null, null];
    }
    var nb = nbCache(g);
    var work = [0];
    var K = layered ? tFirst : 0;
    var key0 = cfgKey(start) + '|0';
    var seen = new Map([[key0, [start, null]]]);  // state -> [configuration, parent state]
    var queue = [[start, 0, 0, key0]], head = 0;
    function witness(stKey, last, tGoal) {
      var cfgs = [last];
      while (stKey !== null) { var e = seen.get(stKey); cfgs.push(e[0]); stKey = e[1]; }
      cfgs.reverse();
      while (cfgs.length < tGoal + 1) cfgs.push(last);  // T = tFirst > depth: wait at the goals
      var out = [];
      for (var a = 0; a < A; a++) out.push(cfgs.map(function (c) { return c[a]; }));
      return out;
    }
    function* successors(cfg) {
      var occ = new Map();
      for (var i = 0; i < cfg.length; i++) {
        var l = occ.get(cfg[i]);
        if (l === undefined) { l = []; occ.set(cfg[i], l); }
        l.push(i);
      }
      var target = new Array(A).fill(0);
      var used = new Set();
      function* rec(i, moved) {
        work[0] += 1;
        if (work[0] > budget) throw new BudgetExhausted();
        if (i === A) { yield target.slice(); return; }
        var u = cfg[i], nbs = nb(u);
        for (var k = 0; k < nbs.length; k++) {
          var w = nbs[k];
          if (used.has(w)) continue;
          if (w !== u) {
            if (oneMove && moved) continue;
            var swap = false, js = occ.get(w) || [];
            for (var q = 0; q < js.length; q++) {
              var jj = js[q];
              if (jj < i && target[jj] === u) { swap = true; break; }
            }
            if (swap) continue;
          }
          target[i] = w;
          used.add(w);
          yield* rec(i + 1, moved || w !== u);
          used.delete(w);
        }
      }
      yield* rec(0, false);
    }
    try {
      while (head < queue.length) {
        var item = queue[head++], cfg = item[0], k2 = Math.min(item[1] + 1, K), t1 = item[2] + 1;
        for (var nxt of successors(cfg)) {
          var key = cfgKey(nxt);
          if (key === goalKey && k2 >= K) {
            var Tw = Math.max(t1, tFirst);
            return [true, Tw, witness(item[3], nxt, Tw)];
          }
          var st = key + '|' + k2;
          if (!seen.has(st)) { seen.set(st, [nxt, item[3]]); work[0] += 1; queue.push([nxt, k2, t1, st]); }
        }
      }
    } catch (e) {
      if (e instanceof BudgetExhausted) return [null, null, null];
      throw e;
    }
    return [false, null, null];
  }

  // _undirected: the underlying simple undirected graph (self-loops dropped), an array of Sets
  function undirectedOf(g) {
    var und = [];
    for (var v = 0; v < g.n; v++) und.push(new Set());
    for (v = 0; v < g.n; v++) {
      var nb = neighbors(g, v) || [];
      for (var k = 0; k < nb.length; k++) {
        if (nb[k] !== v) { und[v].add(nb[k]); und[nb[k]].add(v); }
      }
    }
    return und;
  }

  // _components: {comp: comp[v], comps: the vertices of each component (ascending)}
  function componentsOf(und) {
    var n = und.length, comp = new Array(n).fill(-1), comps = [];
    for (var v = 0; v < n; v++) {
      if (comp[v] >= 0) continue;
      var c = comps.length, members = [v], stack = [v];
      comp[v] = c;
      while (stack.length) {
        var u = stack.pop();
        und[u].forEach(function (w) { if (comp[w] < 0) { comp[w] = c; members.push(w); stack.push(w); } });
      }
      comps.push(members.sort(function (p, q) { return p - q; }));
    }
    return { comp: comp, comps: comps };
  }

  function sortedSet(set) { return Array.from(set).sort(function (p, q) { return p - q; }); }

  // _line_order: the position of every vertex of a path (cyc false) or cycle component
  function lineOrder(und, C, cyc) {
    var cur = C[0];
    if (!cyc) for (var i = 0; i < C.length; i++) if (und[C[i]].size === 1) { cur = C[i]; break; }
    var pos = new Map([[cur, 0]]);
    for (;;) {
      var nxt = sortedSet(und[cur]).filter(function (w) { return !pos.has(w); });
      if (!nxt.length) return pos;
      cur = nxt[0];
      pos.set(cur, pos.size);
    }
  }

  // _two_edge_labels: the 2-edge-connected component of every vertex of component C (iterative Tarjan)
  function twoEdgeLabels(und, C) {
    var disc = new Map(), low = new Map(), bridges = new Set();
    function ek(a, b) { return Math.min(a, b) + ',' + Math.max(a, b); }
    var root = C[0];
    disc.set(root, 0); low.set(root, 0);
    var stack = [[root, null, sortedSet(und[root]), 0]];
    while (stack.length) {
      var top = stack[stack.length - 1], u = top[0], pushed = false;
      while (top[3] < top[2].length) {
        var w = top[2][top[3]++];
        if (w === top[1]) continue;
        if (disc.has(w)) low.set(u, Math.min(low.get(u), disc.get(w)));
        else {
          disc.set(w, disc.size); low.set(w, disc.get(w));
          stack.push([w, u, sortedSet(und[w]), 0]);
          pushed = true;
          break;
        }
      }
      if (pushed) continue;
      stack.pop();
      if (stack.length) {
        var p = stack[stack.length - 1][0];
        low.set(p, Math.min(low.get(p), low.get(u)));
        if (low.get(u) > disc.get(p)) bridges.add(ek(p, u));
      }
    }
    var lab = new Map();
    for (var i = 0; i < C.length; i++) {
      var v = C[i];
      if (lab.has(v)) continue;
      lab.set(v, v);
      var st = [v];
      while (st.length) {
        var x = st.pop();
        und[x].forEach(function (y) { if (!lab.has(y) && !bridges.has(ek(x, y))) { lab.set(y, v); st.push(y); } });
      }
    }
    return lab;
  }

  // _structural_refutation: sound polynomial certificates that no horizon has a plan (distinct starts
  // and goals, every goal reachable); the reason, or null
  function structuralRefutation(und, cc, starts, goals) {
    var robots = new Map();
    for (var i = 0; i < starts.length; i++) {
      if (cc.comp[starts[i]] !== cc.comp[goals[i]]) return 'robot ' + i + ' must leave its connected component';
      var l = robots.get(cc.comp[starts[i]]);
      if (l === undefined) { l = []; robots.set(cc.comp[starts[i]], l); }
      l.push(i);
    }
    var keys = Array.from(robots.keys()).sort(function (p, q) { return p - q; });
    for (var kk = 0; kk < keys.length; kk++) {
      var rs = robots.get(keys[kk]);
      if (rs.every(function (r) { return starts[r] === goals[r]; })) continue;
      var C = cc.comps[keys[kk]], k = C.length, E = 0, dmax = 0;
      for (var j = 0; j < k; j++) { E += und[C[j]].size; dmax = Math.max(dmax, und[C[j]].size); }
      E = E / 2;
      if (k >= 2 && dmax <= 2 && (E === k - 1 || E === k)) {
        var cyc = E === k, pos = lineOrder(und, C, cyc);
        var order = rs.slice().sort(function (p, q) { return pos.get(starts[p]) - pos.get(starts[q]); });
        var gp = order.map(function (r) { return pos.get(goals[r]); }), m = gp.length;
        if (!cyc) {
          for (j = 0; j + 1 < m; j++) {
            if (gp[j] > gp[j + 1]) {
              return 'robots ' + order[j] + ' and ' + order[j + 1] + ' must pass each other on a component that is a path';
            }
          }
        } else if (m >= 3) {
          var desc = 0;
          for (j = 0; j < m; j++) if (gp[j] > gp[(j + 1) % m]) desc++;
          if (desc > 1) return 'the robots on a component that is a cycle must change their cyclic order';
        }
      }
      if (rs.length === k) {
        var lab = twoEdgeLabels(und, C);
        for (j = 0; j < rs.length; j++) {
          if (lab.get(starts[rs[j]]) !== lab.get(goals[rs[j]])) {
            return 'robot ' + rs[j] + ' must cross a bridge of a fully occupied component (only rotations along ' +
                   'cycles are possible there)';
          }
        }
      }
    }
    return null;
  }

  // _component_refutes: with robots on two or more components, does one component's own search fail?
  function componentRefutes(g, cc, starts, goals, budget, oneMove, tFirst) {
    var robots = new Map();
    for (var i = 0; i < starts.length; i++) {
      var l = robots.get(cc.comp[starts[i]]);
      if (l === undefined) { l = []; robots.set(cc.comp[starts[i]], l); }
      l.push(i);
    }
    if (robots.size < 2) return false;
    var keys = Array.from(robots.keys()).sort(function (p, q) { return p - q; });
    for (var kk = 0; kk < keys.length; kk++) {
      var rs = robots.get(keys[kk]), C = cc.comps[keys[kk]], idx = new Map();
      C.forEach(function (v, j) { idx.set(v, j); });
      var adj = C.map(function (v) {
        return (neighbors(g, v) || []).map(function (w) { return idx.get(w); }).sort(function (p, q) { return p - q; });
      });
      var sub = makeGraph(adj, C.map(function () { return null; }), 0, 0);
      var v = jointBfs(sub, rs.map(function (r) { return idx.get(starts[r]); }),
                       rs.map(function (r) { return idx.get(goals[r]); }), budget, oneMove, tFirst)[0];
      if (v === false) return true;
    }
    return false;
  }

  // _joint_moves: valid joint moves from cfg, yielding [nextCfg, movedMask]
  function* jointMoves(nb, cfg, frozen, work, budget) {
    var A = cfg.length, occ = new Map();
    for (var i = 0; i < A; i++) {
      var l = occ.get(cfg[i]);
      if (l === undefined) { l = []; occ.set(cfg[i], l); }
      l.push(i);
    }
    var target = new Array(A).fill(0), used = new Set();
    function* rec(i, moved) {
      work[0] += 1;
      if (work[0] > budget) throw new BudgetExhausted();
      if (i === A) { yield [target.slice(), moved]; return; }
      var u = cfg[i], nbs = nb(u);
      for (var k = 0; k < nbs.length; k++) {
        var w = nbs[k];
        if (used.has(w)) continue;
        if (w !== u) {
          if ((frozen >> i) & 1) continue;
          var swap = false, js = occ.get(w) || [];
          for (var q = 0; q < js.length; q++) {
            if (js[q] < i && target[js[q]] === u) { swap = true; break; }
          }
          if (swap) continue;
        }
        target[i] = w;
        used.add(w);
        yield* rec(i + 1, moved | ((w !== u ? 1 : 0) << i));
        used.delete(w);
      }
    }
    yield* rec(0, 0);
  }

  // _trim: drop trailing steps in which no robot moves
  function trimPaths(paths) {
    if (!paths || paths.length === 0) return paths;
    var L = paths[0].length;
    while (L > 1 && paths.every(function (p) { return p[L - 1] === p[L - 2]; })) L -= 1;
    return paths.map(function (p) { return p.slice(0, L); });
  }

  // a binary min-heap of [f, t, tie, state] (tie is unique: the pop order is a total order)
  function Heap() { this.a = []; }
  Heap.prototype.less = function (x, y) {
    if (x[0] !== y[0]) return x[0] < y[0];
    if (x[1] !== y[1]) return x[1] < y[1];
    return x[2] < y[2];
  };
  Heap.prototype.push = function (x) {
    var a = this.a, i = a.length;
    a.push(x);
    while (i > 0) {
      var p = (i - 1) >> 1;
      if (!this.less(a[i], a[p])) break;
      var tmp = a[i]; a[i] = a[p]; a[p] = tmp; i = p;
    }
  };
  Heap.prototype.pop = function () {
    var a = this.a, top = a[0], last = a.pop();
    if (a.length > 0) {
      a[0] = last;
      var i = 0, n = a.length;
      for (;;) {
        var l = 2 * i + 1, r = l + 1, m = i;
        if (l < n && this.less(a[l], a[m])) m = l;
        if (r < n && this.less(a[r], a[m])) m = r;
        if (m === i) break;
        var tmp = a[i]; a[i] = a[m]; a[m] = tmp; i = m;
      }
    }
    return top;
  };

  /**
   * joint_optimum: exact optimum over all horizons by search over joint configurations.
   * Returns [value, paths], false (no plan) or null (budget exhausted, or more than 30 robots); goals without a
   * self-loop are allowed.
   */
  var JOINT_OPTIMUM_MAX_ROBOTS = 30;  // joint_optimum: null above this many robots

  function jointOptimum(g, starts, goals, objective, budget, upper) {
    if (budget === undefined) budget = JOINT_SEARCH_BUDGET;
    var A = starts.length, start = starts.slice(), goal = goals.slice(), i;
    if (cfgKey(start) === cfgKey(goal)) return [0, starts.map(function (v) { return [v]; })];
    if (new Set(goal).size < A) return false;
    if (A > JOINT_OPTIMUM_MAX_ROBOTS) return null;  // robot sets are 32-bit masks (F, moved)
    var rev = reversedAdjacency(g), dist = [];
    for (i = 0; i < goals.length; i++) dist.push(rev === null ? bfsDistances(g, goals[i]) : distancesTo(rev, goals[i], g.n));
    var nb = nbCache(g);
    var work = [0];

    function pathsOf(parent, stKey, cfgOf) {
      var cfgs = [];
      while (stKey !== null) { cfgs.push(cfgOf(stKey)); stKey = parent.get(stKey); }
      cfgs.reverse();
      var out = [];
      for (var r = 0; r < A; r++) out.push(cfgs.map(function (c) { return c[r]; }));
      return trimPaths(out);
    }

    try {
      if (objective === 'total_time' || objective === 'distance') {
        var tt = objective === 'total_time', full = (1 << A) - 1;
        var h = function (cfg, F) {
          var s = 0;
          for (var r = 0; r < A; r++) if (!((F >> r) & 1)) s += dist[r][cfg[r]];
          return s;
        };
        var closures = function (cfg, F) {
          if (!tt) return [F];
          var at = [];
          for (var r = 0; r < A; r++) if (cfg[r] === goal[r] && !((F >> r) & 1)) at.push(r);
          var out = [];
          for (var mask = 0; mask < (1 << at.length); mask++) {
            // one enumeration of the 2^k closure states may not exceed what is left of the
            // budget (see joint_optimum)
            if (mask && work[0] + mask > budget) throw new BudgetExhausted();
            var m = F;
            for (var k = 0; k < at.length; k++) if ((mask >> k) & 1) m |= 1 << at[k];
            out.push(m);
          }
          return out;
        };
        var best = new Map(), parent = new Map(), states = new Map(), pq = new Heap(), tie = 0;
        var mk = function (cfg, F) { return cfgKey(cfg) + '|' + F; };
        var cl0 = closures(start, 0);
        for (i = 0; i < cl0.length; i++) {
          var k0 = mk(start, cl0[i]);
          best.set(k0, [0, 0]); parent.set(k0, null); states.set(k0, [start, cl0[i]]);
          pq.push([h(start, cl0[i]), 0, tie, k0]);
          tie += 1;
        }
        var done = new Set();
        var goalKey = cfgKey(goal);
        while (pq.a.length > 0) {
          var item = pq.pop(), t = item[1], stKey = item[3];
          if (done.has(stKey)) continue;
          done.add(stKey);
          var st = states.get(stKey), cfg = st[0], F = st[1];
          var c = best.get(stKey)[0];
          if ((tt && F === full) || (!tt && cfgKey(cfg) === goalKey)) {
            return [c, pathsOf(parent, stKey, function (k) { return states.get(k)[0]; })];
          }
          var cost = A - popcount(F);
          for (var mv of jointMoves(nb, cfg, tt ? F : 0, work, budget)) {
            var nxt = mv[0], moved = mv[1], nc;
            if (tt) nc = c + cost;
            else {
              if (moved === 0) continue;
              nc = c + popcount(moved);
            }
            var cls = closures(nxt, F);
            for (var q = 0; q < cls.length; q++) {
              var F2 = cls[q], k2 = mk(nxt, F2), old = best.get(k2);
              if (old === undefined || nc < old[0] || (nc === old[0] && t + 1 < old[1])) {
                best.set(k2, [nc, t + 1]);
                parent.set(k2, stKey);
                if (!states.has(k2)) states.set(k2, [nxt, F2]);
                work[0] += 1;
                pq.push([nc + h(nxt, F2), t + 1, tie, k2]);
                tie += 1;
              }
            }
          }
        }
        return false;
      }
      if (objective === 'max_distance') {
        if (upper === undefined || upper === null) throw valueError("objective='max_distance' needs upper");
        var lbm = 0;
        for (i = 0; i < A; i++) if (dist[i][starts[i]] > lbm) lbm = dist[i][starts[i]];
        var found = null, D = Math.trunc(upper) - 1;
        var goalKey2 = cfgKey(goal);
        while (D >= lbm) {
          var zeros = new Array(A).fill(0);
          var key0 = cfgKey(start) + '|' + cfgKey(zeros);
          var par = new Map([[key0, null]]), stOf = new Map([[key0, [start, zeros]]]);
          // front: per configuration, the Pareto-minimal move vectors; dominated states are not expanded
          var front = new Map([[cfgKey(start), [zeros]]]), dead = new Set();
          var queue = [key0], head = 0, hit = null;
          while (head < queue.length && hit === null) {
            var sk = queue[head++];
            if (dead.has(sk)) continue;
            var s0 = stOf.get(sk), cfg2 = s0[0], mvv = s0[1];
            for (var mv2 of jointMoves(nb, cfg2, 0, work, budget)) {
              var nx = mv2[0], md = mv2[1];
              if (md === 0) continue;
              var m2 = [];
              for (var r = 0; r < A; r++) m2.push(mvv[r] + ((md >> r) & 1));
              var bad = false;
              for (r = 0; r < A; r++) if (m2[r] + dist[r][nx[r]] > D) { bad = true; break; }
              if (bad) continue;
              var nk = cfgKey(nx), k3 = nk + '|' + cfgKey(m2);
              if (par.has(k3)) continue;
              var fr = front.get(nk) || [], dom = false, f, q2;
              for (q2 = 0; q2 < fr.length && !dom; q2++) {
                f = fr[q2]; dom = true;
                for (r = 0; r < A; r++) if (f[r] > m2[r]) { dom = false; break; }
              }
              if (dom) continue;
              var keep = [];
              for (q2 = 0; q2 < fr.length; q2++) {
                f = fr[q2];
                var worse = true;
                for (r = 0; r < A; r++) if (m2[r] > f[r]) { worse = false; break; }
                if (worse) dead.add(nk + '|' + cfgKey(f)); else keep.push(f);
              }
              keep.push(m2);
              front.set(nk, keep);
              work[0] += fr.length;
              par.set(k3, sk);
              stOf.set(k3, [nx, m2]);
              work[0] += 1;
              if (cfgKey(nx) === goalKey2) { hit = k3; break; }
              queue.push(k3);
            }
          }
          if (hit === null) break;
          var mx = 0, hm = stOf.get(hit)[1];
          for (r = 0; r < A; r++) if (hm[r] > mx) mx = hm[r];
          found = [mx, pathsOf(par, hit, function (k) { return stOf.get(k)[0]; })];
          D = found[0] - 1;
        }
        if (found === null) return [Math.trunc(upper), null];
        return found;
      }
      throw valueError('unknown objective ' + objective);
    } catch (e) {
      if (e instanceof BudgetExhausted) return null;
      throw e;
    }
  }

  function result(paths, status, T, lowerBound, attempts, extra) {
    var obj = paths === null ? null : (paths.length ? paths[0].length - 1 : 0);
    return { paths: paths, status: status, objective: obj, T: T, lowerBound: lowerBound,
             attempts: attempts, extra: extra || {} };
  }

  // _check_budget: with a budget <= 0 every joint-state proof is inconclusive and the makespan loop never ends
  function checkBudget(budget) {
    if (typeof budget !== 'number' || !(budget >= 1)) {
      throw valueError('feasibility_budget must be a number >= 1, got ' + String(budget));
    }
    return budget;
  }

  // _check_horizon_arg: a horizon argument (T, tMax) is null/undefined or an integer (TypeError otherwise)
  function checkHorizonArg(x, what) {
    if (x === undefined || x === null) return null;
    if (typeof x !== 'number' || !Number.isInteger(x)) throw new TypeError(what + ' must be an integer, got ' + String(x));
    if (x < 0) throw valueError(what + ' must be >= 0, got ' + x);
    return x;
  }

  // _check_gap: the MIP gap must be a finite number >= 0
  function checkGap(gap) {
    if (typeof gap !== 'number' || !(gap >= 0 && gap < Infinity)) {
      throw valueError('gap must be a finite number >= 0, got ' + String(gap));
    }
    return gap;
  }

  // _check_time_limit (min_* and plan, fixed): null / undefined is the only "no limit"; anything else
  // must be a finite number of seconds > 0 (0, a negative limit or NaN used to mean no limit)
  function checkTimeLimit(tl) {
    if (tl === undefined || tl === null) return null;
    if (typeof tl !== 'number' || !(tl > 0 && tl < Infinity)) {
      throw valueError('time_limit must be None (no limit) or a finite number of seconds > 0, got ' + String(tl));
    }
    return tl;
  }

  // _refutes: does a makespan solve without a plan of value n prove that its horizon has no plan?
  // (with n: an 'optimal' whose bound still admits the value n, e.g. under a gap tolerance, decides nothing)
  function refutes(sol, n) {
    if (sol.status === 'optimal' && n !== undefined && sol.bound !== null && sol.bound !== undefined &&
        sol.bound > n - 0.5) return false;
    return sol.status === 'infeasible' || sol.status === 'optimal';
  }

  // _single_fail_status (fixed): a horizon without an accepted plan is infeasible only when proved so;
  // a node limit or an incumbent outside the gap leaves it undecided (reported as 'time_limit')
  function singleFailStatus(sol) { return sol.status === 'infeasible' ? 'infeasible' : 'time_limit'; }

  // _solve_makespan (kind 'time', fixed mode)
  function solveMakespan(g, starts, goals, o) {
    var budget = checkBudget(o.feasibilityBudget === undefined || o.feasibilityBudget === null ?
                             DEFAULT_FEASIBILITY_BUDGET : o.feasibilityBudget);
    var tMax = checkHorizonArg(o.tMax, 't_max');
    starts = asIds(starts, 'starts'); goals = asIds(goals, 'goals');
    var A = starts.length;
    checkInstance(g, starts, goals);
    var clk = new Clock();
    var T;
    if (o.tStart === undefined || o.tStart === null) {
      var sp = findShortestPaths(g, starts, goals, resolveRng(o.rng));
      T = 0;
      for (var i = 0; i < sp.length; i++) {
        if (sp[i] === null) throw new Infeasible('goal of robot ' + i + ' is unreachable from its start');
        T = Math.max(T, sp[i].length - 1);
      }
    } else T = o.tStart;
    var lowerBound = T;
    var extraSteps = o.extraSteps || 0;
    T += extraSteps;
    var Tfirst = T;  // the proofs cover the horizons >= Tfirst
    if (A > 0 && tMax !== null && T > tMax) {
      throw new Infeasible('the first horizon T = ' + T + ' (T_min = ' + lowerBound + ' plus ' + extraSteps +
                           ' extra steps) exceeds t_max = ' + tMax);
    }
    var tl = o.timeLimit === undefined || o.timeLimit === null ? -1 : o.timeLimit;
    var has = tl > 0;
    var attempts = [];
    var feas = {};
    var setOptimal = o.setOptimal === undefined ? true : o.setOptimal;
    function res(paths, status, T_, extra) { return result(paths, status, T_, lowerBound, attempts, extra); }
    function afterFailure(T_) {
      if (tMax !== null && T_ + 1 > tMax) throw new Infeasible('no solution with makespan <= t_max = ' + tMax);
      var nf = feas.n = (feas.n || 0) + 1;
      var due = (nf & (nf - 1)) === 0;
      var b = budget * nf;
      function bfs(b_) {
        var t0 = nowSec();
        try { return jointBfs(g, starts, goals, b_, false, Tfirst); } finally {
          feas.bfsT = (feas.bfsT || 0) + nowSec() - t0;
          if (b_ > (feas.bMax || 0)) feas.bMax = b_;
        }
      }
      function tmaxCheck(minT) {
        if (minT !== null && tMax !== null && minT > tMax) {
          throw new Infeasible('no solution with makespan <= t_max = ' + tMax +
                               ' (joint-state search: the smallest feasible makespan is ' + minT + ')');
        }
      }
      if (feas.v === null && due && b <= (feas.bNone || 0)) {
        // a larger budget was inconclusive already (a check paced by time, below)
      } else if ((feas.v === null && due) || !('v' in feas)) {
        if (A > 0 && new Set(goals).size < A && cfgKey(starts) !== cfgKey(goals)) feas.v = false;
        else {
          var jb = bfs(b);
          feas.v = jb[0]; feas.minT = jb[1]; feas.plan = jb[2];
          tmaxCheck(feas.minT);
        }
      } else if (feas.v === null && !feas.known && nowSec() - tLoop - (feas.bfsT || 0) > (feas.bfsT || 0)) {
        // the horizons solved so far took longer than all checks: check with twice the largest budget;
        // only a proof is used (a plan found here is dropped, so jointPlan stays deterministic)
        var b2 = 2 * (feas.bMax || budget), j2 = bfs(b2);
        if (j2[0] === false) feas.v = false;
        else if (j2[0] === true) { feas.known = true; tmaxCheck(j2[1]); }
        else feas.bNone = b2;
      }
      if (feas.v !== false) return null;
      if (Tfirst > lowerBound && goals.some(function (v) { return (neighbors(g, v) || []).indexOf(v) < 0; })) {
        // without wait loops at the goals a plan does not extend to longer horizons
        throw new Infeasible('no plan with makespan T >= ' + Tfirst + ' (T_min = ' + lowerBound + ' plus ' +
                             (Tfirst - lowerBound) + ' extra steps; joint-state search)');
      }
      throw new Infeasible('the instance has no solution for any horizon (joint-state search)');
    }
    if (A === 0) return res([], 'optimal', 0);
    // joint_plan: after a failed horizon, the joint-state search's plan at the smallest feasible horizon
    // (the model is a feasibility problem; proving each horizon below it by branch and bound is exponential)
    function jointPlan() {
      if (!feas.plan) return null;
      // _joint_attempt: recorded at its horizon (phase 'joint', no model), so the horizons account for the plan
      attempts.push({ T: feas.minT, phase: 'joint', status: 'optimal', objective: A, bound: null, runtime: null });
      return res(feas.plan, extraSteps > 0 && feas.minT !== lowerBound ? 'feasible' : 'optimal', feas.minT,
                 { proof: 'joint-state search' });
    }
    function remaining() { return tl - clk.elapsed(); }
    var tLoop = nowSec();  // the work since then (models and checks) paces the checks
    var reach = reachability(g, starts, goals, T);
    var sol, m, lim;
    if (o.solveRelaxedFirst && extraSteps === 0) {
      for (;;) {
        lim = has ? remaining() : -1;
        if (has && lim <= 0) return res(null, 'time_limit', T);
        m = buildModel('time', g, starts, goals, T, { integer: false, setOptimal: false, reach: reach });
        sol = runModel(m, has ? lim : null, null, o.lp);
        attempts.push(attempt(T, 'lp', sol));
        if (sol.x === null) {
          if (sol.status === 'time_limit') return res(null, 'time_limit', T);
          if (sol.status === 'node_limit') return res(null, 'time_limit', T, { solver_status: sol.status });
          throw runtimeError('LP relaxation at T=' + T + ' returned ' + sol.status);
        }
        // a limit stopped the LP at a point below n: the relaxation is not refuted
        if (sol.status !== 'optimal' && !(sol.objective + 0.01 > A)) {
          return res(null, 'time_limit', T, { solver_status: sol.status });
        }
        if (sol.objective + 0.01 > A) break;
        else if (has && remaining() < 0) return res(null, 'time_limit', T);
        afterFailure(T);
        if (feas.plan) return jointPlan();
        T += 1;
        reach = reachability(g, starts, goals, T);
      }
    }
    var val = 0, ps = null, Tfound = null;
    while (val !== A) {
      lim = has ? remaining() : -1;
      if (has && lim <= 0) return res(null, 'time_limit', T);
      m = buildModel('time', g, starts, goals, T, { integer: true, setOptimal: setOptimal, reach: reach });
      sol = runModel(m, has ? lim : null, null, o.lp);
      attempts.push(attempt(T, 'ilp', sol));
      val = 0;
      if (sol.x !== null) val = pyRound(sol.objective);
      if (val === A) {
        ps = m.retrievePaths(sol.x, g);
        Tfound = T;
      } else {
        if ((has && remaining() < 0) || sol.status === 'time_limit') return res(null, 'time_limit', T);
        // _refutes: only 'infeasible' or 'optimal' below n proves T infeasible; a node limit decides nothing
        if (!refutes(sol, A)) return res(null, 'time_limit', T, { solver_status: sol.status });
        afterFailure(T);
        if (feas.plan) return jointPlan();
        T += 1;
        reach = reachability(g, starts, goals, T);
      }
    }
    // started at T_min + extraSteps: the first feasible horizon is not proved optimal
    return res(ps, extraSteps > 0 && Tfound !== lowerBound ? 'feasible' : 'optimal', Tfound);
  }

  /**
   * minMakespan(g, starts, goals, {timeLimit, tMax, solveRelaxedFirst, rng, feasibilityBudget, lp}):
   * mppflow.solvers.min_makespan (fixed, backend 'builtin'). One findShortestPaths draw set.
   * Returns {paths, status, objective, T, lowerBound, attempts, extra}; throws MPP.Infeasible.
   */
  function minMakespan(g, starts, goals, opts) {
    opts = opts || {};
    var tl = checkTimeLimit(opts.timeLimit);
    return solveMakespan(g, starts, goals, {
      solveRelaxedFirst: !!opts.solveRelaxedFirst, setOptimal: true, extraSteps: 0,
      timeLimit: tl === null ? -1 : tl, rng: opts.rng, tMax: opts.tMax,
      feasibilityBudget: opts.feasibilityBudget, lp: opts.lp });
  }

  // _proven: is the integer value of a minimization proved optimal at this horizon? The solver says
  // 'optimal' as soon as it is within its tolerances (gap, or one in the lp options), so the bound must
  // close; only without a bound is 'optimal' with gap 0 taken as the proof.
  function proven(sol, gap, value, lower) {
    if (lower !== null && lower !== undefined) return Math.ceil(lower - 1e-6) >= value;
    return sol.status === 'optimal' && gap === 0;
  }

  // _tt_model_gap: the backend gap on the model objective n*T - TT that keeps the gap on TT <= gap
  function ttModelGap(gap, A, T, lb) {
    if (gap === 0) return 0;
    var room = A * T - lb;
    return room > 0 && lb > 0 ? gap * lb / room : 0;
  }

  // _tt_single (fixed)
  function ttSingle(g, starts, goals, gap, T, has, timeLimit, ctx, draw, lowerBound) {
    var A = starts.length, lb;
    if (draw) {
      var sp = findShortestPaths(g, starts, goals, resolveRng(ctx.rng));
      lb = 0;
      for (var i = 0; i < sp.length; i++) {
        if (sp[i] === null) throw new Infeasible('goal of robot ' + i + ' is unreachable from its start');
        lb += sp[i].length - 1;
      }
    } else lb = lowerBound;
    var reach = reachability(g, starts, goals, T);
    var lim = has ? timeLimit : -1;
    var m = buildModel('total_time', g, starts, goals, T, { integer: true, reach: reach });
    var sol = runModel(m, lim, ttModelGap(gap, A, T, lb), ctx.lp);
    ctx.attempts.push(attempt(T, 'ilp', sol));
    var ps = null, status = singleFailStatus(sol), total = null;
    if (sol.x !== null) {
      var val = pyRound(sol.objective);
      var noBound = sol.bound === null || sol.bound === undefined;
      var g_ = noBound ? INF : relativeGap(A * T - val, A * T - sol.bound);
      var accept = sol.status === 'optimal' || g_ <= gap + 1e-9;
      if (accept) {
        total = A * T - val;
        ps = m.retrievePaths(sol.x, g);
        status = proven(sol, gap, total, noBound ? null : A * T - sol.bound) ? 'optimal' : 'feasible';
      }
    }
    return { paths: ps, status: status, objective: total, T: T, lowerBound: lb, attempts: ctx.attempts, extra: {} };
  }

  // _dist_single (fixed), kind 'distance' or 'max_distance'
  function distSingle(g, starts, goals, gap, T, timeLimit, ctx, kind) {
    var reach = reachability(g, starts, goals, T);
    var m = buildModel(kind, g, starts, goals, T, { integer: true, reach: reach });
    var sol = runModel(m, timeLimit, gap, ctx.lp);
    ctx.attempts.push(attempt(T, 'ilp', sol));
    var ps = null, status = singleFailStatus(sol), obj = null;
    if (sol.x !== null) {
      var objval = sol.objective;
      var rg = (sol.bound !== null && sol.bound !== undefined) ? relativeGap(objval, sol.bound) : INF;
      var accept = sol.status === 'optimal' || rg <= gap + 1e-9;
      if (accept) {
        obj = pyRound(objval);
        ps = m.retrievePaths(sol.x, g);
        status = proven(sol, gap, obj, sol.bound) ? 'optimal' : 'feasible';
      }
    }
    return { paths: ps, status: status, objective: obj, T: T, lowerBound: null, attempts: ctx.attempts, extra: {} };
  }

  function fewConfigs(nv, A) {
    var c = 1;
    for (var k = 0; k < A; k++) {
      c *= nv - k;
      if (c > JOINT_SEARCH_MAX_CONFIGS) return false;
    }
    return true;
  }

  // _horizon_bound: T_hi of horizonSearch for a plan of objective `value`
  function horizonBound(kind, value, lb, dists, A) {
    if (kind === 'total_time') return Math.trunc(value) - lb + Math.max.apply(null, dists);
    if (kind === 'distance') return Math.trunc(value);
    return A * Math.trunc(value);
  }

  // _joint_settle: the joint-state stage of the horizon search (null: skipped, out of budget, or a better
  // plan longer than tMax)
  function jointSettle(kind, g, starts, goals, r0, info, tMax, lt) {
    var A = starts.length;
    if (!((lt === null || lt > 0) && fewConfigs(g.n, A))) return null;
    var jo = jointOptimum(g, starts, goals, kind, JOINT_SEARCH_BUDGET,
                          kind === 'max_distance' ? Math.trunc(r0.objective) : undefined);
    if (!jo) return null;
    var val = jo[0], jpaths = jo[1];
    if (val >= r0.objective) {
      info.proof = 'joint-state search';
      r0.status = 'optimal';
      r0.extra.horizon_search = info;
      return r0;
    }
    if (jpaths !== null && (tMax === null || jpaths[0].length - 1 <= tMax)) {
      info.proof = 'joint-state search';
      var ex = Object.assign({}, r0.extra);
      ex.horizon_search = info;
      return { paths: jpaths, status: 'optimal', objective: val, T: jpaths.length ? jpaths[0].length - 1 : 0,
               lowerBound: r0.lowerBound, attempts: r0.attempts, extra: ex };
    }
    return null;
  }

  // _search_every: some goal has no wait self-loop, so the optimum need not be monotone in T: solve at
  // T_opt, try the joint-state search, else solve every horizon up to T_hi (shrinking with the best value)
  function searchEvery(kind, g, starts, goals, single, lb, dists, tMax, timeLimit, clk, ctx, budget) {
    var A = starts.length;
    var info = { T0: null, T_hi: null, T_opt: null, every_horizon: true };
    function left() { return timeLimit === null ? null : timeLimit - clk.elapsed(); }
    function stopped() { var lt = left(); return lt !== null && lt <= 0; }
    function timedOut(T_) {
      return { paths: null, status: 'time_limit', objective: null, T: T_, lowerBound: lb, attempts: ctx.attempts,
               extra: { horizon_search: info } };
    }
    if (stopped()) return timedOut(null);
    var lt0 = left();
    var rm = solveMakespan(g, starts, goals, {
      solveRelaxedFirst: false, setOptimal: true, extraSteps: 0, timeLimit: lt0 === null ? -1 : lt0,
      tStart: dists.length ? Math.max.apply(null, dists) : 0, tMax: tMax, feasibilityBudget: budget, lp: ctx.lp });
    for (var i = 0; i < rm.attempts.length; i++) ctx.attempts.push(rm.attempts[i]);
    if (rm.paths === null || stopped()) return timedOut(rm.paths === null ? null : rm.objective);
    var Topt = rm.objective;
    info.T_opt = info.T0 = Topt;
    var best = single(Topt, true, left());
    best.extra.horizon_search = info;
    if (best.paths === null) return best;  // time limit
    best.paths = trimPaths(best.paths);
    best.T = best.paths.length ? best.paths[0].length - 1 : 0;
    if (best.objective <= lb) { best.status = 'optimal'; return best; }
    var Thi = Math.min(horizonBound(kind, best.objective, lb, dists, A), A * Topt);
    if (tMax !== null) Thi = Math.min(Thi, tMax);
    info.T_hi = Thi;
    if (Thi <= Topt) return best;
    var rj = jointSettle(kind, g, starts, goals, best, info, tMax, left());
    if (rj !== null) return rj;
    var provenAll = best.status === 'optimal', cut = false, T = Topt;
    while (T < Thi) {
      T += 1;
      if (stopped()) { cut = true; break; }
      var r = single(T, false, left());
      if (r.paths !== null) {
        if (r.objective < best.objective) {
          r.paths = trimPaths(r.paths);
          r.T = r.paths.length ? r.paths[0].length - 1 : 0;
          r.extra.horizon_search = info;
          best = r;
          if (best.objective <= lb) break;
          Thi = Math.min(Thi, horizonBound(kind, best.objective, lb, dists, A));
          info.T_hi = Thi;
        }
        if (r.status !== 'optimal') provenAll = false;  // solved within the gap only
      } else if (r.status === 'time_limit') {
        cut = true;
        break;
      }
    }
    if (best.objective <= lb) best.status = 'optimal';
    else if (cut) { best.status = 'feasible'; best.extra.time_limit_in_scan = true; }
    else best.status = provenAll ? 'optimal' : 'feasible';
    return best;
  }

  // _search: Algorithm 1 for TT / DIST / max distance (fixed mode)
  function horizonSearch(kind, g, starts, goals, Tgiven, single, lb, dists, tMax, timeLimit, clk, ctx, budget) {
    if (goals.some(function (v) { return (neighbors(g, v) || []).indexOf(v) < 0; })) {
      return searchEvery(kind, g, starts, goals, single, lb, dists, tMax, timeLimit, clk, ctx, budget);
    }
    var A = starts.length;
    var info = { T0: null, T_hi: null, T_opt: null };
    if (tMax !== null && Tgiven !== null && Tgiven !== undefined && Tgiven > tMax) Tgiven = tMax;
    function left() { return timeLimit === null ? null : timeLimit - clk.elapsed(); }
    function topt() {
      var lt = left();
      if (lt !== null && lt <= 0) return null;
      var t0 = dists.length ? Math.max.apply(null, dists) : 0;
      var r = solveMakespan(g, starts, goals, {
        solveRelaxedFirst: false, setOptimal: true, extraSteps: 0, timeLimit: lt === null ? -1 : lt,
        tStart: t0, tMax: tMax, feasibilityBudget: budget, lp: ctx.lp });
      for (var i = 0; i < r.attempts.length; i++) ctx.attempts.push(r.attempts[i]);
      if (r.paths === null) return null;
      info.T_opt = r.objective;
      return r.objective;
    }
    function outOfTime(T_) {  // the budget is spent (e.g. by the makespan stage): no solve without a limit
      var lt = left();
      if (lt !== null && lt <= 0) return { paths: null, status: 'time_limit', objective: null, T: T_, lowerBound: lb,
                                           attempts: ctx.attempts, extra: { horizon_search: info } };
      return null;
    }
    var firstDraw = true;
    var T0 = Tgiven;
    if (T0 === null || T0 === undefined) {
      T0 = topt();
      if (T0 === null) return { paths: null, status: 'time_limit', objective: null, T: null, lowerBound: lb,
                                attempts: ctx.attempts, extra: { horizon_search: info } };
    }
    var rt = outOfTime(T0);
    if (rt !== null) return rt;
    var r0 = single(T0, firstDraw, left());
    firstDraw = false;
    if (r0.paths === null && r0.status === 'infeasible') {
      if (Tgiven === null || Tgiven === undefined) return r0;
      var to = topt();
      if (to === null) { r0.status = 'time_limit'; r0.extra.horizon_search = info; return r0; }
      T0 = Math.max(T0, to);
      rt = outOfTime(T0);
      if (rt !== null) return rt;
      r0 = single(T0, firstDraw, left());
    }
    info.T0 = T0;
    if (r0.paths === null) { r0.extra.horizon_search = info; return r0; }
    if (r0.objective <= lb) { r0.status = 'optimal'; r0.extra.horizon_search = info; return r0; }
    var Thi = horizonBound(kind, r0.objective, lb, dists, A);
    if (info.T_opt !== null) Thi = Math.min(Thi, A * info.T_opt);
    if (tMax !== null) Thi = Math.min(Thi, tMax);
    info.T_hi = Thi;
    var best = r0;
    if (Thi > T0) {
      var rj = jointSettle(kind, g, starts, goals, r0, info, tMax, left());
      if (rj !== null) return rj;
      var lt = left();
      if (lt === null || lt > 0) {
        var r1 = single(Thi, false, lt);
        if (r1.paths !== null && r1.objective < r0.objective) {
          r1.paths = trimPaths(r1.paths);
          r1.T = r1.paths.length ? r1.paths[0].length - 1 : 0;
          best = r1;
        } else if (r1.paths !== null) {
          if (r1.status !== 'optimal') best.status = 'feasible';  // T_hi solved within the gap only
        } else {
          // no plan at T_hi: a limit stopped the solve; optimal at T0, not proved for every T
          if (r1.status === 'time_limit') best.extra.time_limit_at_T_hi = true;
          best.status = 'feasible';
        }
      } else {
        best.extra.time_limit_at_T_hi = true;
        best.status = 'feasible';
      }
    }
    best.extra.horizon_search = info;
    return best;
  }

  function exactHorizonMsg(what) {
    return 'no plan with makespan exactly ' + what + " (horizon='single'; a goal has no wait self-loop, so a " +
           "shorter plan does not extend to this horizon: horizon='search' tries every horizon)";
  }

  function objectiveSolver(kind, g, starts, goals, opts) {
    opts = opts || {};
    // the order of mppflow's checks: time limit, horizon, feasibility budget, T and t_max, gap, ids
    var tl = checkTimeLimit(opts.timeLimit);
    var horizon = opts.horizon === undefined || opts.horizon === null ? 'search' : opts.horizon;
    if (horizon !== 'search' && horizon !== 'single') throw valueError("horizon must be 'single' or 'search'");
    var budget = checkBudget(opts.feasibilityBudget === undefined || opts.feasibilityBudget === null ?
                             DEFAULT_FEASIBILITY_BUDGET : opts.feasibilityBudget);
    var T = checkHorizonArg(opts.T, 'T'), tMax = checkHorizonArg(opts.tMax, 't_max');
    var gap = checkGap(opts.gap === undefined || opts.gap === null ? 0 : opts.gap);
    starts = asIds(starts, 'starts'); goals = asIds(goals, 'goals');
    var clk = new Clock();
    var ctx = { attempts: [], rng: opts.rng, lp: opts.lp };
    checkInstance(g, starts, goals);
    var dists = robotDistances(g, starts, goals);
    for (var i = 0; i < dists.length; i++) {
      if (dists[i] === null) throw new Infeasible('goal of robot ' + i + ' is unreachable from its start');
    }
    var lb = 0;
    if (kind === 'max_distance') { for (i = 0; i < dists.length; i++) if (dists[i] > lb) lb = dists[i]; }
    else for (i = 0; i < dists.length; i++) lb += dists[i];
    var single;
    if (kind === 'total_time') {
      single = function (T, draw, lft) {
        var has = lft !== null;
        return ttSingle(g, starts, goals, gap, T, has, has ? lft : -1, ctx, draw, lb);
      };
    } else {
      single = function (T, draw, lft) {
        return distSingle(g, starts, goals, gap, T, lft !== null ? lft : -1, ctx, kind);
      };
    }
    var r;
    if (horizon === 'single') {
      if (T === null) throw valueError("horizon='single' needs T");
      var T1 = T;
      if (tMax !== null) {  // solve at min(T, tMax)
        for (i = 0; i < dists.length; i++) {
          if (dists[i] > tMax) {
            throw new Infeasible('the shortest path of robot ' + i + ' has length ' + dists[i] + ' > t_max = ' + tMax);
          }
        }
        T1 = Math.min(T, tMax);
      }
      r = single(T1, true, tl);
      // without a wait loop at some goal only this one horizon is proved infeasible
      var waits = goals.every(function (v) { return (neighbors(g, v) || []).indexOf(v) >= 0; });
      if (T1 < T) {
        r.extra.T_requested = T;
        if (r.paths === null && r.status === 'infeasible') {
          throw new Infeasible(waits ? 'no plan with makespan <= t_max = ' + tMax : exactHorizonMsg('t_max = ' + tMax));
        }
      }
      if (r.paths === null && r.status === 'infeasible') {
        throw new Infeasible(waits ? 'no plan with makespan <= T = ' + T : exactHorizonMsg('T = ' + T));
      }
      // optimal at this T only: optimal for the objective only when it meets the lower bound
      if (r.paths !== null && r.status === 'optimal' && r.objective > lb) { r.status = 'feasible'; r.extra.optimal_at_T = true; }
    } else {
      r = horizonSearch(kind, g, starts, goals, T, single, lb, dists, tMax, tl, clk, ctx, budget);
    }
    r.lowerBound = lb;
    r.attempts = ctx.attempts;
    return r;
  }

  /** minTotalTime(g, starts, goals, {T, gap, timeLimit, horizon, tMax, rng}) = mppflow min_total_time. */
  function minTotalTime(g, starts, goals, opts) { return objectiveSolver('total_time', g, starts, goals, opts); }
  /** minTotalDistance(g, starts, goals, {T, gap, timeLimit, horizon, tMax}) = mppflow min_total_distance. */
  function minTotalDistance(g, starts, goals, opts) { return objectiveSolver('distance', g, starts, goals, opts); }
  /** minMaxDistance(g, starts, goals, {T, gap, timeLimit, horizon, tMax}) = mppflow min_max_distance. */
  function minMaxDistance(g, starts, goals, opts) { return objectiveSolver('max_distance', g, starts, goals, opts); }

  MPP.jointFeasibility = jointFeasibility;
  MPP.jointOptimum = jointOptimum;
  // test hook (parity fixtures): the configuration limit of the horizon search's joint-state stage
  MPP._testing = {
    setJointSearchMaxConfigs: function (n) { var old = JOINT_SEARCH_MAX_CONFIGS; JOINT_SEARCH_MAX_CONFIGS = n; return old; }
  };
  MPP.minMakespan = minMakespan;
  MPP.minTotalTime = minTotalTime;
  MPP.minTotalDistance = minTotalDistance;
  MPP.minMaxDistance = minMaxDistance;

  // ===================================================================== planner.py (fixed mode)
  // _ecc: the largest BFS layer reachable from c (self-loops ignored)
  function eccentricity(g, c) {
    var dist = new Map([[c, 0]]), q = [c], head = 0, m = 0;
    while (head < q.length) {
      var v = q[head++], nb = neighbors(g, v) || [];
      for (var k = 0; k < nb.length; k++) {
        var u = nb[k];
        if (u !== v && !dist.has(u)) {
          var d = dist.get(v) + 1;
          dist.set(u, d);
          if (d > m) m = d;
          q.push(u);
        }
      }
    }
    return m;
  }

  // _ring (fixed): BFS distances in [lo, hi] from c, the centre included at distance 0
  function ring(g, c, lo, hi) {
    if (hi < 0 || lo > hi) return [];
    var out = lo <= 0 ? [c] : [];
    if (hi >= 1) out = out.concat(findGoalsInDistanceRange(g, c, lo, hi));
    return out;
  }

  // _hops: BFS hop distances from c over nbrs(v) (self-loops ignored), as a Map
  function hops(nbrs, c) {
    var dist = new Map([[c, 0]]), q = [c], head = 0;
    while (head < q.length) {
      var v = q[head++], nb = nbrs(v) || [];
      for (var k = 0; k < nb.length; k++) if (!dist.has(nb[k])) { dist.set(nb[k], dist.get(v) + 1); q.push(nb[k]); }
    }
    return dist;
  }

  // _hops_to: dist(v, c) for every v that reaches c (reversed arcs on a one-way graph)
  function hopsTo(g, c) {
    var rev = reversedAdjacency(g);
    if (rev === null) return hops(function (v) { return neighbors(g, v); }, c);
    return hops(function (v) { return rev[v]; }, c);
  }

  /**
   * _SplitWindows with endpoints (fixed): the candidate lists splitPaths examines for one robot.
   * The goal side is dist(v, goal) (reversed arcs on a one-way graph); for an odd distance
   * D = L - 1 the first list holds the middle vertices of the shortest paths (T-RO Sec. V-B).
   */
  class SplitWindows {
    constructor(g, s, goal, L) {
      this.g = g; this.s = s; this.goal = goal;
      this.dMin = this.dMax = Math.floor(L / 2);
      this.flipped = true;
      this.lists = [];
      this.done = false;
      this.toGoal = hopsTo(g, goal);
      var eg = 0;
      this.toGoal.forEach(function (d) { if (d > eg) eg = d; });
      this.ecc = Math.max(eccentricity(g, s), eg);
      this._last = null;
      var D = L - 1;
      if (D % 2 === 1) {
        var ds = hops(function (v) { return neighbors(g, v); }, s), to = this.toGoal;
        var mid = ring(g, s, Math.floor(D / 2), Math.floor(D / 2) + 1).filter(function (v) {
          return to.has(v) && (ds.has(v) ? ds.get(v) : -1) + to.get(v) === D;
        });
        this.lists.push(new JavaIntHashSet(mid).toList());
      }
    }
    _next() {
      var lo = this.dMin <= 0 ? 0 : Math.max(this.dMin, 1);
      var key = lo + ',' + this.dMax;
      if (key !== this._last) {
        var S = ring(this.g, this.s, this.dMin, this.dMax);
        var G = new Set(), dLo = this.dMin, dHi = this.dMax;
        this.toGoal.forEach(function (d, v) { if (dLo <= d && d <= dHi) G.add(v); });
        this.lists.push(new JavaIntHashSet(S.filter(function (v) { return G.has(v); })).toList());
        this._last = key;
      }
      if (lo === 0 && this.dMax >= this.ecc) this.done = true;
      if (this.flipped) this.dMin = Math.max(this.dMin - 1, 0);
      else this.dMax += 1;
      this.flipped = !this.flipped;
    }
    choose(used) {
      var j = 0;
      for (;;) {
        while (j >= this.lists.length) {
          if (this.done) return null;
          this._next();
        }
        var l = this.lists[j];
        for (var k = 0; k < l.length; k++) if (!used.has(l[k])) return l[k];
        j += 1;
      }
    }
  }

  function reaches(g, src, dst) {
    var seen = new Set([src]), q = [src], head = 0;
    while (head < q.length) {
      var v = q[head++];
      if (v === dst) return true;
      var nb = neighbors(g, v) || [];
      for (var k = 0; k < nb.length; k++) if (!seen.has(nb[k])) { seen.add(nb[k]); q.push(nb[k]); }
    }
    return false;
  }

  function splitFallback(g, s, goal, used, agent) {
    if (!used.has(s)) return s;
    if (!used.has(goal)) return goal;
    var seen = new Set([s]), q = [s], head = 0;
    while (head < q.length) {
      var v = q[head++];
      if (!used.has(v) && reaches(g, v, goal)) return v;
      var nb = neighbors(g, v) || [];
      for (var k = 0; k < nb.length; k++) if (!seen.has(nb[k])) { seen.add(nb[k]); q.push(nb[k]); }
    }
    throw new SplitStuck('no free vertex can serve as the intermediate goal of robot ' + agent);
  }

  /** _SplitStuck: the split heuristic found no intermediate goal (not a proof of infeasibility). */
  class SplitStuck extends Infeasible {}

  /**
   * splitGoals(g, starts, goals, {random, rng}): PathPlanner.split_paths (fixed): one intermediate
   * goal per robot (T-RO Sec. V-B, Fig. 8). One findShortestPaths draw set, plus n draws with random.
   */
  function splitGoals(g, starts, goals, opts) {
    opts = opts || {};
    var rng = resolveRng(opts.rng);
    starts = Array.from(starts); goals = Array.from(goals);
    var paths = findShortestPaths(g, starts, goals, rng), L = [], i;
    for (i = 0; i < paths.length; i++) {
      if (paths[i] === null) throw new Infeasible('goal of robot ' + i + ' is unreachable from its start');
      L.push(paths[i].length);
    }
    var n = L.length, order = [];
    for (i = 0; i < n; i++) order.push(i);
    order.sort(function (p, q) { return (L[q] - L[p]) || (p - q); });
    if (opts.random) {
      var vec = [];
      for (i = 0; i < n; i++) vec.push(i);
      order = [];
      for (i = 0; i < n; i++) order.push(vec.splice(Math.trunc(vec.length * rng.random()), 1)[0]);
    }
    var middle = new Array(n).fill(0), used = new Set();
    for (var k = 0; k < order.length; k++) {
      var ag = order[k];
      var v = new SplitWindows(g, starts[ag], goals[ag], L[ag]).choose(used);
      if (v === null) v = splitFallback(g, starts[ag], goals[ag], used, ag);
      middle[ag] = v;
      used.add(v);
    }
    return middle;
  }

  /** merge_paths (fixed): A followed by B without its first column; null if either is null. */
  function mergePaths(A, B) {
    if (A === null || A === undefined || B === null || B === undefined) return null;
    if (A.length !== B.length) throw valueError('the pieces have different numbers of robots');
    var out = [];
    for (var i = 0; i < A.length; i++) {
      var ra = A[i], rb = B[i];
      if (!ra.length || !rb.length) throw valueError('empty path in a piece');
      if (ra[ra.length - 1] !== rb[0]) throw valueError('the pieces do not join for robot ' + i);
      out.push(ra.concat(rb.slice(1)));
    }
    return out;
  }

  function adjacent(g, u, v) {
    var nb = g.adj[u];
    if (nb === undefined) return null;
    var lo = 0, hi = nb.length;
    while (lo < hi) { var mid = (lo + hi) >> 1; if (nb[mid] < v) lo = mid + 1; else hi = mid; }
    return lo < nb.length && nb[lo] === v;
  }

  function lexLess(c, b) {
    for (var i = 0; i < 4; i++) if (c[i] !== b[i]) return c[i] < b[i];
    return false;
  }

  function firstCollision(paths) {
    var n = paths.length, L = n ? paths[0].length : 0, best = null, a, c;
    for (var t = 0; t < L; t++) {
      var at = new Map();
      for (a = 0; a < n; a++) {
        var l = at.get(paths[a][t]);
        if (l === undefined) { l = []; at.set(paths[a][t], l); }
        l.push(a);
      }
      at.forEach(function (ags, v) {
        if (ags.length > 1) {
          var cc = [ags[0], ags[1], t, 0, v];
          if (best === null || lexLess(cc, best)) best = cc;
        }
      });
      if (t > 0) {
        var moves = new Map();
        for (a = 0; a < n; a++) {
          var u = paths[a][t - 1], w = paths[a][t];
          if (u !== w) {
            var key = u + ',' + w, ml = moves.get(key);
            if (ml === undefined) { ml = []; moves.set(key, ml); }
            ml.push(a);
          }
        }
        moves.forEach(function (ags, key) {
          var parts = key.split(','), rev = moves.get(parts[1] + ',' + parts[0]);
          if (rev) {
            for (var x = 0; x < ags.length; x++) {
              for (var y = 0; y < rev.length; y++) {
                c = [Math.min(ags[x], rev[y]), Math.max(ags[x], rev[y]), t, 1, null];
                if (best === null || lexLess(c, best)) best = c;
              }
            }
          }
        });
      }
    }
    if (best === null) return null;
    if (best[3] === 0) return 'Path ' + best[0] + ' and ' + best[1] + ' meet on vertex ' + best[4] + ' at time step ' + best[2];
    return 'Path ' + best[0] + ' and ' + best[1] + ' cross the same edge at time step ' + best[2];
  }

  /** path_set_error (fixed): the reason a plan is invalid, or null. */
  function pathSetError(paths, g, starts, goals) {
    if (paths === null || paths === undefined) throw valueError('paths is None');
    var n = paths.length, a;
    if (n !== starts.length || n !== goals.length) return 'The path set has ' + n + ' paths for ' + starts.length + ' robots.';
    if (n === 0) return null;
    for (a = 0; a < n; a++) if (!paths[a] || paths[a].length === 0) return 'The path set contains an empty path.';
    var L = paths[0].length;
    for (a = 0; a < n; a++) if (paths[a].length !== L) return 'The paths of the path set have different lengths.';
    for (a = 0; a < n; a++) {
      if (paths[a][0] !== starts[a]) return 'Start location for agent ' + a + ' is incorrect in the path set.';
      if (paths[a][L - 1] !== goals[a]) return 'Goal location for agent ' + a + ' is incorrect in the path set.';
    }
    for (var t = 1; t < L; t++) {
      for (a = 0; a < n; a++) if (!adjacent(g, paths[a][t - 1], paths[a][t])) return 'Path ' + a + ' is not valid at t = ' + (t - 1);
    }
    return firstCollision(paths);
  }

  /** validate(paths, g, starts, goals): is_path_set_valid (true iff pathSetError finds nothing). */
  function validate(paths, g, starts, goals) { return pathSetError(paths, g, starts, goals) === null; }

  /** has_cycles (fixed): the number of rotations, summed over time steps. */
  function hasCycles(paths) {
    if (paths === null || paths === undefined) throw valueError('paths is None');
    if (paths.length === 0) return 0;
    var L = paths[0].length, n = paths.length, num = 0;
    for (var a = 0; a < n; a++) if (!paths[a] || paths[a].length !== L) throw valueError('ragged path set');
    for (var i = 0; i < L - 1; i++) {
      var nxt = new Map();
      for (var v = 0; v < n; v++) nxt.set(paths[v][i], paths[v][i + 1]);
      for (var s = 0; s < n; s++) {
        var vid = paths[s][i];
        if (nxt.has(vid)) {
          if (nxt.get(vid) === vid) { nxt.delete(vid); continue; }
        } else continue;
        while (nxt.has(vid)) { var w = nxt.get(vid); nxt.delete(vid); vid = w; }
        if (vid === paths[s][i]) num += 1;
      }
    }
    return num;
  }

  function arrival(p, g) {
    if (!p.length || p[p.length - 1] !== g) return null;
    var t = p.length - 1;
    while (t > 0 && p[t - 1] === g) t -= 1;
    return t;
  }

  /** plan_metrics: {makespan, totalTime, totalDistance, maxDistance} (totalTime null if a robot misses its goal). */
  function metrics(paths, goals) {
    if (paths === null || paths === undefined) throw valueError('plan_metrics needs a plan, got None');
    if (paths.length !== goals.length) throw valueError(paths.length + ' paths for ' + goals.length + ' goals');
    if (paths.length === 0) return { makespan: 0, totalTime: 0, totalDistance: 0, maxDistance: 0 };
    if (paths[0].length === 0 || paths.some(function (p) { return p.length !== paths[0].length; })) {
      throw valueError('the paths must be non-empty and of equal length');
    }
    var tt = 0, ttNull = false, td = 0, md = 0;
    for (var a = 0; a < paths.length; a++) {
      var p = paths[a], ar = arrival(p, goals[a]), d = 0;
      if (ar === null) ttNull = true; else tt += ar;
      for (var i = 0; i < p.length - 1; i++) if (p[i] !== p[i + 1]) d++;
      td += d;
      if (d > md) md = d;
    }
    return { makespan: paths[0].length - 1, totalTime: ttNull ? null : tt, totalDistance: td, maxDistance: md };
  }

  /**
   * splitConfigurations(g, starts, goals, splits, {rng}): the 2^splits + 1 configurations of the
   * k-way split (multiThreadedSplitPlanning's split loop, level by level): [starts, ..., goals].
   */
  function splitConfigurations(g, starts, goals, splits, opts) {
    var rng = resolveRng(opts && opts.rng);
    var mp = Math.pow(2, splits), n = starts.length, msg = [], i, s;
    for (i = 0; i <= mp; i++) msg.push(new Array(n).fill(0));
    for (i = 0; i < n; i++) { msg[0][i] = starts[i]; msg[mp][i] = goals[i]; }
    for (i = 0; i < splits; i++) {
      var num = Math.pow(2, i), step = Math.pow(2, splits - i);
      for (s = 0; s < num; s++) {
        var a = msg[s * step], b = msg[(s + 1) * step];
        try {
          msg[s * step + step / 2] = splitGoals(g, a, b, { rng: rng });
        } catch (e) {
          // PathPlanner._middle: a failure of the heuristic (no free vertex; an unreachable goal of an
          // inner piece) gives null, with the reason in opts.failures; a top-level unreachable goal throws
          if (!(e instanceof Infeasible) || (i === 0 && !(e instanceof SplitStuck))) throw e;
          if (opts && opts.failures) opts.failures.push('split [' + a.join(', ') + '] -> [' + b.join(', ') + ']: ' + e.message);
          return null;
        }
      }
    }
    return msg;
  }

  /**
   * splitPrecheck(g, starts, goals, budget): mppflow.plan's instance checks before a split (fixed):
   * duplicate starts (ValueError), duplicate goals, an unreachable goal, or no plan for any horizon
   * (jointFeasibility within budget) throw, so that status 'failed' is a failure of the heuristic only.
   */
  function splitPrecheck(g, starts, goals, budget) {
    checkInstance(g, starts, goals);
    var d = robotDistances(g, starts, goals);
    for (var i = 0; i < d.length; i++) {
      if (d[i] === null) throw new Infeasible('goal of robot ' + i + ' is unreachable from its start');
    }
    budget = checkBudget(budget === undefined || budget === null ? DEFAULT_FEASIBILITY_BUDGET : budget);
    if (jointFeasibility(g, starts, goals, budget, false) === false) {
      throw new Infeasible('the instance has no solution for any horizon (joint-state search)');
    }
  }

  /**
   * PlanResult.stats from the attempts: solves / lp_solves / ilp_solves count the models (not the
   * plans taken from the joint-state search, phase 'joint'); horizons and phases list every attempt.
   */
  function attemptStats(attempts) {
    var models = attempts.filter(function (a) { return a.phase !== 'joint'; });
    return {
      solves: models.length,
      lp_solves: models.filter(function (a) { return a.phase === 'lp'; }).length,
      ilp_solves: models.filter(function (a) { return a.phase !== 'lp'; }).length,
      horizons: attempts.map(function (a) { return a.T; }),
      phases: attempts.map(function (a) { return a.phase === 'lp' || a.phase === 'joint' ? a.phase : 'ilp'; }),
    };
  }

  /**
   * planSplit(g, starts, goals, k, {timeLimit, timeLimitPerPiece, rng, solveRelaxedFirst, lp}):
   * mppflow.plan(..., 'makespan', split=k) in fixed mode with the builtin backend. k = 1 is the
   * optimal solve (LP relaxation first, as Main.solveProblem); k = 2, 4, 8, 16 is the k-way split
   * multiThreadedSplitPlanning (split levels 1..4), its pieces solved in index order.
   * timeLimitPerPiece (default true, as mppflow): every piece gets the full timeLimit, so a k-way
   * split can run for k times the limit. false (the demo): the limit covers the whole split, each
   * piece gets the time left, and the split stops at the first piece without a plan (the merged
   * plan is lost then anyway); stats.skipped_pieces then counts the pieces not attempted.
   * Returns {paths, status, k, intermediateGoals, metrics, lowerBounds, failures, attempts, stats}.
   */
  /** mppflow.planner.MAX_SPLIT: a larger k is a mistake (2**40 passes the 32-bit power-of-two test). */
  var MAX_SPLIT = 1024;

  function planSplit(g, starts, goals, k, opts) {
    opts = opts || {};
    asInt(k, 'split');  // k = 1.5 passed the power-of-two test (1.5 & 0.5 is 0)
    if (!(k >= 1) || (k & (k - 1)) !== 0) throw valueError('split must be a power of two (1, 2, 4, 8, 16, ...)');
    if (k > MAX_SPLIT) throw valueError('split must be at most ' + MAX_SPLIT + ' (pieces beyond the makespan are only waits)');
    var splits = Math.round(Math.log2(k));
    var tl = checkTimeLimit(opts.timeLimit);
    starts = asIds(starts, 'starts'); goals = asIds(goals, 'goals');
    tl = tl === null ? -1 : tl;
    var rng = resolveRng(opts.rng);
    var perPiece = opts.timeLimitPerPiece === undefined || opts.timeLimitPerPiece === null ? true : !!opts.timeLimitPerPiece;
    var solves = [], failures = [], status = null, inter = null, paths, skipped = 0, outOfTime = false;
    var t0 = nowSec();
    function pieceLimit() { return tl <= 0 || perPiece ? tl : tl - (nowSec() - t0); }
    function makespanPiece(s, gl, lim) {
      var r = solveMakespan(g, s, gl, { solveRelaxedFirst: false, setOptimal: true, extraSteps: 0,
                                        timeLimit: lim, rng: rng, lp: opts.lp });
      solves.push(r);
      if (r.paths !== null && pathSetError(r.paths, g, s, gl) !== null) return null;
      return r.paths;
    }
    if (k === 1) {
      var srf = opts.solveRelaxedFirst === undefined ? true : !!opts.solveRelaxedFirst;
      var r = solveMakespan(g, starts, goals, { solveRelaxedFirst: srf, setOptimal: true, extraSteps: 0,
                                                timeLimit: tl, rng: rng, lp: opts.lp });
      solves.push(r);
      paths = r.paths;
      if (paths !== null && pathSetError(paths, g, starts, goals) !== null) paths = null;
      if (paths !== null) status = 'optimal';
    } else {
      var mp = k, i;
      splitPrecheck(g, starts, goals, opts.feasibilityBudget);
      var msg = splitConfigurations(g, starts, goals, splits, { rng: rng, failures: failures });
      var pieces = new Map();
      for (i = 0; msg !== null && i < mp; i++) {
        var pp, lim = pieceLimit();
        if (tl > 0 && lim <= 0) { outOfTime = true; skipped = mp - i; break; }
        try {
          pp = makespanPiece(msg[i], msg[i + 1], lim);
        } catch (e) {
          if (!(e instanceof Infeasible)) throw e;
          failures.push('piece ' + i + ': ' + e.message);
          pp = null;
        }
        if (pp !== null) pieces.set(i, pp);
        else if (!perPiece) { skipped = mp - i - 1; break; }    // the merged plan is lost: stop here
      }
      paths = pieces.has(0) ? pieces.get(0) : null;
      for (i = 1; i < mp; i++) paths = mergePaths(paths, pieces.has(i) ? pieces.get(i) : null);
      inter = msg === null ? null : msg.slice(1, mp).map(function (c) { return c.slice(); });
    }
    if (paths === null) {
      status = outOfTime || solves.some(function (x) { return x.status === 'time_limit'; }) ? 'time_limit' : 'failed';
    } else if (status === null) status = 'feasible';
    var attempts = [];
    solves.forEach(function (x) { x.attempts.forEach(function (at) { attempts.push(at); }); });
    var out = {
      paths: paths, status: status, k: k, intermediateGoals: inter,
      metrics: paths !== null ? metrics(paths, goals) : null,
      lowerBounds: lowerBounds(g, starts, goals), failures: failures, attempts: attempts,
      stats: attemptStats(attempts),
    };
    out.stats.seconds = nowSec() - t0;
    if (!perPiece) out.stats.skipped_pieces = skipped;
    return out;
  }

  MPP.splitGoals = splitGoals;
  MPP.splitPrecheck = splitPrecheck;
  MPP.mergePaths = mergePaths;
  MPP.attemptStats = attemptStats;
  MPP.pathSetError = pathSetError;
  MPP.validate = validate;
  MPP.hasCycles = hasCycles;
  MPP.metrics = metrics;
  MPP.planSplit = planSplit;
  MPP.splitConfigurations = splitConfigurations;
  MPP.trimPaths = trimPaths;
  MPP.version = '0.1.0';

  /**
   * Live-solve size cap (docs/js/README.md, measured with docs/js/bench.js): the browser solves an
   * instance live only if its makespan model at the lower-bound horizon has at most this many
   * columns; larger maps play precomputed plans.
   */
  var LIVE_MAX_COLUMNS = 800;

  /** liveSolveCheck(g, starts, goals): {T, columns, rows, live} for the model at T = lower bound. */
  function liveSolveCheck(g, starts, goals, maxColumns) {
    var lim = maxColumns === undefined ? LIVE_MAX_COLUMNS : maxColumns;
    var T = lowerBounds(g, starts, goals).makespan;
    if (T === null) return { T: null, columns: 0, rows: 0, live: false };
    var m = buildModel('time', g, starts, goals, T);
    return { T: T, columns: m.model.numColumns, rows: m.model.numRows, live: m.model.numColumns <= lim };
  }

  MPP.LIVE_MAX_COLUMNS = LIVE_MAX_COLUMNS;
  MPP.liveSolveCheck = liveSolveCheck;

  // ===================================================================== Web Worker support
  /*
   * The demo solves live in a Web Worker created from a Blob (this works from file://, where a
   * worker cannot load a script file). MPP.workerSource() is the worker's whole program: lp.js,
   * engine.js and a message handler. Protocol: post {id, fn, args, seed}; `fn` names an MPP
   * function (e.g. 'planSplit', 'minTotalTime'), `args` its arguments (plain JSON), and `seed`,
   * if given, adds rng: new MathRandom(seed) to the options object (the last argument). The
   * worker answers {id, ok: true, result} or {id, ok: false, error: {name, message}}.
   */
  var ENGINE_SOURCE = '(' + mppEngineFactory.toString() + ')(self);\n';

  function workerMain() {
    self.onmessage = function (ev) {
      var m = ev.data || {};
      try {
        var f = self.MPP[m.fn];
        if (typeof f !== 'function') throw new Error('unknown function ' + m.fn);
        var args = (m.args || []).slice();
        if (m.seed !== undefined && m.seed !== null) {
          var last = args.length && args[args.length - 1] && typeof args[args.length - 1] === 'object' &&
                     !Array.isArray(args[args.length - 1]) ? args[args.length - 1] : null;
          if (last === null) { last = {}; args.push(last); }
          last.rng = new self.MPP.MathRandom(m.seed);
        }
        var result = f.apply(null, args);
        self.postMessage({ id: m.id, ok: true, result: JSON.parse(JSON.stringify(result, function (k, v) {
          return v instanceof Map || typeof v === 'function' ? undefined : v;
        })) });
      } catch (e) {
        self.postMessage({ id: m.id, ok: false, error: { name: e.name, message: e.message } });
      }
    };
  }

  /** The source text of a worker program running lp.js + engine.js + the message handler. */
  function workerSource() {
    if (!MPP.lp || !MPP.lp.source) throw runtimeError('docs/js/lp.js is not loaded');
    return MPP.lp.source + ENGINE_SOURCE + '(' + workerMain.toString() + ')();\n';
  }

  /** new Worker from a Blob of workerSource() (browsers only). */
  function createWorker() {
    var url = URL.createObjectURL(new Blob([workerSource()], { type: 'text/javascript' }));
    var w = new Worker(url);
    URL.revokeObjectURL(url);
    return w;
  }

  MPP.engineSource = ENGINE_SOURCE;
  MPP.workerSource = workerSource;
  MPP.createWorker = createWorker;

  if (!MPP.lp && typeof module !== 'undefined' && module.exports && typeof require === 'function') {
    MPP.lp = require('./lp.js');
  }
  if (typeof module !== 'undefined' && module.exports) module.exports = MPP;
  if (root) root.MPP = MPP;
})(typeof window !== 'undefined' ? window : (typeof self !== 'undefined' ? self : (typeof globalThis !== 'undefined' ? globalThis : this)));
