/*
 * demo.js -- the browser demo of mppflow, our port of the code of our T-RO 2016 paper (J. Yu and
 * S. M. LaValle, "Optimal Multirobot Path Planning on Graphs: Complete Algorithms and Effective
 * Heuristics").
 *
 * Classic script, no dependencies, works from file://. Needs, loaded before it:
 *   data/demo-data.js  -> window.MPP_DEMO (maps and precomputed plans, docs/data/build_demo_data.py)
 *   js/lp.js, js/engine.js -> window.MPP (the JavaScript port of mppflow, parity-tested)
 *
 * Live solves run in a Web Worker built from a Blob (MPP.createWorker), which works from file://.
 * Without workers they run on the main thread; a k-way split then runs piece by piece, yielding
 * to the page between pieces (same calls, same random draws as MPP.planSplit).
 */
(function () {
  'use strict';

  var MPP = window.MPP;
  var DATA = window.MPP_DEMO;
  var SVGNS = 'http://www.w3.org/2000/svg';
  var THEME_KEY = 'mppflow-demo-theme';
  var PREF_KEY = 'mppflow-demo-prefs';

  var OBJ = [
    { id: 'makespan', label: 'Makespan', metric: 'makespan', lb: 'makespan' },
    { id: 'total_time', label: 'Total arrival time', metric: 'totalTime', lb: 'total_time' },
    { id: 'total_distance', label: 'Total distance', metric: 'totalDistance', lb: 'total_distance' },
    { id: 'max_distance', label: 'Max distance', metric: 'maxDistance', lb: 'max_distance' },
  ];
  var OBJ_BY_ID = {};
  OBJ.forEach(function (o) { OBJ_BY_ID[o.id] = o; });
  var KS = [1, 2, 4, 8, 16];
  var SOLVER_FN = { makespan: 'planSplit', total_time: 'minTotalTime', total_distance: 'minTotalDistance',
                    max_distance: 'minMaxDistance' };
  var EDIT_MAX_SIDE = 8;           // edit mode: grids up to 8 x 8
  var CHECK_MAX_WORK = 150000;     // build the live-check model only if n * robots * (T + 1) is at most this
  var NET_MAX_VERTICES = { compact: 30, full: 16 };
  var NET_MAX_T = { compact: 16, full: 8 };
  var AUTO_ZOOM_PX = 28;   // phones: the smallest grid step (px) a map opens at, zooming in if "fit" is smaller
  var NET_MAX_NODES = { compact: 300, full: 280 };   // vertex copies drawn (n (T + 1), or n (2T + 1) in full)
  // Live solves: the builtin simplex (lp.js) keeps the constraint matrix and its tableaux as dense arrays, so a
  // model over this many numbers (MPP.lp.denseEntries, about 1.6 GB as doubles) is refused instead of crashing
  // the tab with "out of memory" before the time limit (in Chrome, 14346 x 6008 crashed, 10811 x 4798 did not).
  var LIVE_MAX_ENTRIES = 2e8;
  var REPO_URL = 'https://github.com/ru-arcl/multiagent-network-flow/';

  // ------------------------------------------------------------------ small helpers
  function $(id) { return document.getElementById(id); }
  function mq(q) {
    try { return window.matchMedia(q); } catch (e) { return { matches: false, addEventListener: null }; }
  }
  function onMq(m, fn) {
    if (!m) return;
    if (m.addEventListener) m.addEventListener('change', fn);
    else if (m.addListener) m.addListener(fn);
  }
  var reduceMotion = mq('(prefers-reduced-motion: reduce)');
  var darkMq = mq('(prefers-color-scheme: dark)');

  var store = {
    get: function (k) { try { return window.localStorage.getItem(k); } catch (e) { return null; } },
    set: function (k, v) { try { window.localStorage.setItem(k, v); } catch (e) { /* blocked storage */ } },
  };

  function svg(tag, attrs, parent) {
    var e = document.createElementNS(SVGNS, tag);
    if (attrs) for (var k in attrs) if (attrs[k] !== null && attrs[k] !== undefined) e.setAttribute(k, attrs[k]);
    if (parent) parent.appendChild(e);
    return e;
  }
  function html(tag, attrs, text, parent) {
    var e = document.createElement(tag);
    if (attrs) for (var k in attrs) if (attrs[k] !== null && attrs[k] !== undefined) e.setAttribute(k, attrs[k]);
    if (text !== undefined && text !== null) e.textContent = text;
    if (parent) parent.appendChild(e);
    return e;
  }
  /** Text in the graph view. The view works in grid units (one step = 1), and Chrome lays out text with a
   *  font size below about 0.25 user units wrongly, so labels are drawn 100 times larger and scaled down. */
  var TS = 100;
  function vtext(parent, x, y, fs, str, attrs) {
    var a = { transform: 'translate(' + r3(x) + ' ' + r3(y) + ') scale(' + (1 / TS) + ')', 'font-size': r3(fs * TS) };
    if (attrs) for (var k in attrs) a[k] = attrs[k];
    if (a['stroke-width']) a['stroke-width'] = r3(a['stroke-width'] * TS);
    var e = svg('text', a, parent);
    e.textContent = str;
    return e;
  }
  function clear(node, keepTitle) {
    var c = node.firstChild;
    while (c) {
      var nx = c.nextSibling;
      if (!(keepTitle && c.nodeName === 'title')) node.removeChild(c);
      c = nx;
    }
  }
  /** Set a text whose repository paths (docs/..., tests/..., mppflow/..., tools/...) become links to the
   *  repository on GitHub, so a reader of the published page can open them. */
  function setLinkedText(el, text) {
    clear(el);
    var re = /\b((?:docs|tests|mppflow|tools)\/[A-Za-z0-9_.\/-]*[A-Za-z0-9_\/])/g, last = 0, m;
    text = text || '';
    while ((m = re.exec(text)) !== null) {
      if (m.index > last) el.appendChild(document.createTextNode(text.slice(last, m.index)));
      var path = m[1].replace(/\/$/, ''), file = /\.[A-Za-z0-9]+$/.test(path);
      html('a', { href: REPO_URL + (file ? 'blob' : 'tree') + '/main/' + path }, m[1], el);
      last = m.index + m[0].length;
    }
    if (last < text.length) el.appendChild(document.createTextNode(text.slice(last)));
  }
  /** The map's name in the map menu: its title plus the paper figure it comes from. */
  function mapOptionText(m) { return m.title + (m.figure ? ' (' + m.figure + ')' : ''); }
  function clamp(x, lo, hi) { return Math.max(lo, Math.min(hi, x)); }
  function r3(x) { return Math.round(x * 1000) / 1000; }
  function fmtNum(x) { return x === null || x === undefined ? '–' : String(x); }
  function fmtSec(s) {
    if (s === null || s === undefined) return '–';
    if (s < 0.01) return '< 0.01 s';
    // choose the format from the rounded value: 9.996 s is "10.0 s", as 10 s is, not "10.00 s"
    if (Math.round(s * 100) / 100 < 10) return s.toFixed(2) + ' s';
    return s.toFixed(1) + ' s';
  }
  function plural(n, one, many) { return n + ' ' + (n === 1 ? one : (many || one + 's')); }
  function nowMs() { return (window.performance && performance.now) ? performance.now() : Date.now(); }
  function isDark() {
    var a = document.documentElement.getAttribute('data-theme');
    if (a === 'dark') return true;
    if (a === 'light') return false;
    return !!darkMq.matches;
  }
  function robotHue(i) { return (i * 137.508 + 190) % 360; }
  // lightness 72% (dark) / 74% (light) keeps the robot label at >= 4.5:1 on every hue
  function robotFill(i) { return 'hsl(' + robotHue(i).toFixed(1) + ',68%,' + (isDark() ? '72%' : '74%') + ')'; }
  // The plan-flow lines of the network (and the robots' rims): lightness 40% (light) / 63% (dark), moved per hue
  // just far enough that a line drawn at FLOW_OPACITY (.n-flow in demo.css) keeps >= 3.1:1 contrast against the
  // view background --surface-2 (WCAG 1.4.11 asks 3:1). HSL lightness ignores luminance: at a fixed 40%, the
  // yellow and green hues fell to about 2:1 on the light background.
  var FLOW_OPACITY = 0.8, FLOW_CONTRAST = 3.1, flowCache = {};
  function hslRgb(h, s, l) {
    var a = s * Math.min(l, 1 - l);
    function f(n) { var k = (n + h / 30) % 12; return l - a * Math.max(-1, Math.min(k - 3, 9 - k, 1)); }
    return [f(0), f(8), f(4)];
  }
  function relLum(c) {
    var lin = c.map(function (x) { return x <= 0.03928 ? x / 12.92 : Math.pow((x + 0.055) / 1.055, 2.4); });
    return 0.2126 * lin[0] + 0.7152 * lin[1] + 0.0722 * lin[2];
  }
  function contrast(a, b) { var x = relLum(a), y = relLum(b); return (Math.max(x, y) + 0.05) / (Math.min(x, y) + 0.05); }
  /** The view background (the --surface-2 token, a hex colour) as [r, g, b] in 0..1. */
  function viewBg(dark) {
    var hex = '';
    try { hex = getComputedStyle(document.documentElement).getPropertyValue('--surface-2').trim(); } catch (e) { /* none */ }
    if (/^#[0-9a-f]{3}$/i.test(hex)) hex = '#' + hex[1] + hex[1] + hex[2] + hex[2] + hex[3] + hex[3];
    if (!/^#[0-9a-f]{6}$/i.test(hex)) hex = dark ? '#1f262e' : '#eef1f5';
    return [1, 3, 5].map(function (j) { return parseInt(hex.slice(j, j + 2), 16) / 255; });
  }
  function flowLightness(h, dark) {
    var bg = viewBg(dark), key = h.toFixed(1) + (dark ? 'd' : 'l') + bg.join(',');
    if (flowCache[key] !== undefined) return flowCache[key];
    var l = dark ? 63 : 40, step = dark ? 0.5 : -0.5;
    for (;;) {
      var c = hslRgb(h, 0.7, l / 100).map(function (x, j) { return FLOW_OPACITY * x + (1 - FLOW_OPACITY) * bg[j]; });
      if (contrast(c, bg) >= FLOW_CONTRAST || l <= 5 || l >= 95) break;
      l += step;
    }
    flowCache[key] = l;
    return l;
  }
  function flowColor(i) { var h = robotHue(i), dark = isDark(); return 'hsl(' + h.toFixed(1) + ',70%,' + flowLightness(h, dark) + '%)'; }
  function padPaths(paths) {
    if (!paths) return null;
    var L = 1;
    paths.forEach(function (p) { if (p.length > L) L = p.length; });
    return paths.map(function (p) {
      var q = p.slice();
      while (q.length < L) q.push(q[q.length - 1]);
      return q;
    });
  }
  function trimmed(paths) {
    var P = padPaths(paths);
    if (!P || !P.length) return P;
    var L = P[0].length;
    while (L > 1 && P.every(function (p) { return p[L - 1] === p[L - 2]; })) L -= 1;
    return P.map(function (p) { return p.slice(0, L); });
  }
  function samePaths(a, b) {
    if (!a || !b) return false;
    return JSON.stringify(trimmed(a)) === JSON.stringify(trimmed(b));
  }
  function plainGraph(g) {
    return { n: g.n, adj: g.adj.map(function (a) { return a.slice(); }),
             xy: g.xy.map(function (p) { return p ? [p[0], p[1]] : null; }), rows: g.rows || 0, cols: g.cols || 0 };
  }
  function jsonClone(x) {
    return JSON.parse(JSON.stringify(x, function (k, v) { return v instanceof Map || typeof v === 'function' ? undefined : v; }));
  }

  // ------------------------------------------------------------------ state
  var S = {
    map: null,              // the shown map: an entry of MPP_DEMO.maps or the custom (edited) one
    g: null,                // its engine graph
    lastDataMap: null,
    objective: 'makespan', k: 1, source: 'computed', play: 'pre',
    liveByMap: {},          // map id -> {planKey: live result}
    checks: {},             // map id -> MPP.liveSolveCheck result (or a reason it was skipped)
    info: null,             // {kind: 'pre' | 'paper' | 'live' | 'none', plan}
    paths: null, T: 0, t: 0, playing: false, speed: 1,
    bounds: [],             // split piece boundaries (time steps) of the played plan
    inter: null,            // intermediate goal configurations of the played plan
    interIdx: -2,
    hl: -1, pinned: false,
    edit: null, arrange: false, zoom: 1,
    net: { variant: 'compact', T: 4, overlay: true },
    view: { goals: true, inter: true, colors: false, ids: false, transpose: false },
    solving: null,
    lastDone: null,         // the last live solve that finished: {mapRef, key, objective, k, entry}
    timeLimit: 30,
  };
  var V = {};               // the current drawing: geometry and element handles

  // ------------------------------------------------------------------ preferences and theme
  /** The seconds the Time limit menu offers. */
  function timeLimitChoices() {
    var sel = document.getElementById('live-tl');
    return sel ? Array.prototype.map.call(sel.options, function (o) { return +o.value; }) : [10, 30, 60, 120];
  }
  function loadPrefs() {
    var raw = store.get(PREF_KEY);
    if (!raw) return;
    try {
      var p = JSON.parse(raw);
      if (p && typeof p === 'object') {
        if (typeof p.speed === 'number') S.speed = p.speed;
        ['goals', 'inter', 'colors', 'ids'].forEach(function (k) { if (typeof p[k] === 'boolean') S.view[k] = p[k]; });
        // only a choice of the Time limit menu (a value saved by another build, or edited by hand, would leave the
        // menu blank while the solves use it), as bind() does for the playback speed
        if (typeof p.timeLimit === 'number' && timeLimitChoices().indexOf(p.timeLimit) >= 0) S.timeLimit = p.timeLimit;
        if (p.netVariant === 'full' || p.netVariant === 'compact') S.net.variant = p.netVariant;
      }
    } catch (e) { /* ignore a corrupt entry */ }
  }
  function savePrefs() {
    store.set(PREF_KEY, JSON.stringify({ speed: S.speed, goals: S.view.goals, inter: S.view.inter, colors: S.view.colors,
                                         ids: S.view.ids, timeLimit: S.timeLimit, netVariant: S.net.variant }));
  }
  function syncThemeButton() {
    // an action button: its label names the theme it switches to (no aria-pressed, which would contradict it)
    var b = $('theme-toggle'), dark = isDark();
    b.setAttribute('aria-label', dark ? 'Use light theme' : 'Use dark theme');
    b.querySelector('.theme-label').textContent = dark ? 'Light' : 'Dark';
  }
  function initTheme() {
    syncThemeButton();
    $('theme-toggle').addEventListener('click', function () {
      var next = isDark() ? 'light' : 'dark';
      document.documentElement.setAttribute('data-theme', next);
      store.set(THEME_KEY, next);
      syncThemeButton();
      themeChanged();
    });
    onMq(darkMq, function () { syncThemeButton(); themeChanged(); });
  }
  function themeChanged() {
    if (!S.map) return;
    if (S.view.colors) applyRobotColors();
    renderNet();
  }

  // ------------------------------------------------------------------ plans and selection
  function keyOf(obj, k) { return obj + '/k' + k; }
  function planKey() { return S.source === 'paper' ? 'paper' : keyOf(S.objective, S.k); }
  function prePlan(key) { return (S.map.plans && S.map.plans[key]) || null; }
  function liveStore() {
    var id = S.map.id;
    if (!S.liveByMap[id]) S.liveByMap[id] = {};
    return S.liveByMap[id];
  }
  function liveFor(key) { return liveStore()[key] || null; }
  /** A live solve of this instance that proved it has no plan at all (MPP.Infeasible: no solution for any horizon,
   *  which covers every objective and k), or null. The store is cleared when the instance changes. */
  function provedInfeasible() {
    var st = liveStore();
    for (var key in st) if (Object.prototype.hasOwnProperty.call(st, key) && st[key] && st[key].infeasible) return st[key];
    return null;
  }
  function check() { return S.checks[S.map.id] || null; }
  /** Numbers the browser's dense simplex would keep for the makespan model of the check (null: not known). */
  function checkEntries(c) { return c && c.columns ? MPP.lp.denseEntries(c.columns, c.rows) : null; }
  function tooBigForBrowser(c) { var e = checkEntries(c); return e !== null && e > LIVE_MAX_ENTRIES; }
  function memText(entries) {
    return 'about ' + Math.round(entries / 1e6) + ' million numbers (' + (entries * 8 / 1e9).toFixed(1) + ' GB)';
  }
  /** "the max distance plan (k = 1)", "the makespan plan with the 2-way split", "the paper’s plan". */
  function planWords(objective, k) {
    if (objective === 'paper') return 'the paper’s plan';
    return 'the ' + OBJ_BY_ID[objective].label.toLowerCase() + ' plan ' + (k > 1 ? 'with the ' + k + '-way split' : '(k = 1)');
  }
  /** How to show a plan: "Max distance (k = 1)", "Makespan with k = 2". */
  function planPick(objective, k) {
    return OBJ_BY_ID[objective].label + (k > 1 ? ' with k = ' + k : ' (k = 1)');
  }
  /** The k-way splits the page offers on this map (k > 1), other than `except`, as "k = 2, 4 or 8". */
  function splitChoices(except, skip) {
    var ks = KS.filter(function (k) { return k > 1 && k !== except && keyAvailable('makespan', k) && !(skip && skip(k)); });
    if (!ks.length) return '';
    return 'k = ' + (ks.length === 1 ? ks[0] : ks.slice(0, -1).join(', ') + ' or ' + ks[ks.length - 1]);
  }
  function reachable() { return S.map.lower_bounds && S.map.lower_bounds.makespan !== null; }
  /** May the browser try this map live at all? */
  function canTryLive() {
    if (!reachable()) return false;
    if (S.map.custom) return true;
    var c = check();
    return !!(c && c.live);
  }
  function inLivePlans(key) { return !!(S.map.live_plans && S.map.live_plans.indexOf(key) >= 0); }
  function keyAvailable(obj, k) {
    var key = keyOf(obj, k);
    if (prePlan(key) || liveFor(key)) return true;
    if (!canTryLive()) return false;
    if (obj !== 'makespan') return k === 1;
    return k === 1 || k <= splitMaxK();
  }
  /** The largest live split: every piece should get at least one step of the makespan lower bound
   *  (more pieces than steps give 0-step pieces and a worse plan), on data maps and edited ones alike. */
  function splitMaxK() {
    var lb = S.map.lower_bounds ? S.map.lower_bounds.makespan : null;
    return lb === null || lb === undefined ? 1 : Math.max(1, lb);
  }
  function objAvailable(obj) { return KS.some(function (k) { return keyAvailable(obj, k); }); }
  function ensureSelection() {
    if (S.source === 'paper' && !prePlan('paper')) S.source = 'computed';
    if (!objAvailable(S.objective)) {
      var o = OBJ.filter(function (x) { return objAvailable(x.id); })[0];
      S.objective = o ? o.id : 'makespan';
    }
    if (!keyAvailable(S.objective, S.k)) {
      var k = KS.filter(function (x) { return keyAvailable(S.objective, x); })[0];
      S.k = k || 1;
    }
  }
  function currentInfo() {
    if (S.source === 'paper') return { kind: 'paper', plan: prePlan('paper') };
    var key = planKey(), p = prePlan(key), l = liveFor(key);
    var lOk = l && l.paths && l.valid;
    if (S.play === 'live' && lOk) return { kind: 'live', plan: l };
    if (p) return { kind: 'pre', plan: p };
    if (lOk) return { kind: 'live', plan: l };
    return { kind: 'none', plan: null };
  }

  /** Piece boundaries of a split plan: from the stored pieces, or by locating the intermediate goals. */
  function splitBoundaries(plan, paths) {
    if (!plan || !paths) return { bounds: [], inter: null };
    var inter = plan.intermediate_goals || null;
    if (plan.pieces && plan.pieces.length > 1) {
      return { bounds: plan.pieces.slice(1).map(function (p) { return p.t0; }), inter: inter };
    }
    if (!inter || !inter.length) return { bounds: [], inter: null };
    var b = [], t = 0, L = paths[0].length;
    for (var j = 0; j < inter.length; j++) {
      while (t < L && !paths.every(function (p, r) { return p[t] === inter[j][r]; })) t++;
      if (t >= L) return { bounds: [], inter: null };
      b.push(t);
    }
    return { bounds: b, inter: inter };
  }

  function loadPlan() {
    stopPlay();
    S.info = currentInfo();
    if (S.map.custom && S.info.kind !== 'live') S.arrange = true;
    var plan = S.info.plan;
    S.paths = plan && plan.paths ? padPaths(plan.paths) : null;
    S.T = S.paths && S.paths.length ? S.paths[0].length - 1 : 0;
    S.t = 0;
    var sb = splitBoundaries(plan, S.paths);
    S.bounds = sb.bounds;
    S.inter = sb.inter;
    S.interIdx = -2;
    if (S.paths) S.net.T = clamp(Math.max(S.T, 1), 1, NET_MAX_T[S.net.variant]);
    else if (S.map.lower_bounds.makespan !== null) S.net.T = clamp(S.map.lower_bounds.makespan, 1, NET_MAX_T[S.net.variant]);
  }

  // ------------------------------------------------------------------ map selection
  function graphOf(map) {
    if (map.custom) return map._g;
    return MPP.fromAdjacency(map.graph.adj, map.graph.xy);
  }
  function computeCheck(map, g) {
    var lb = map.lower_bounds ? map.lower_bounds.makespan : null;
    if (lb === null || lb === undefined) return { live: false, reason: 'unreachable' };
    var work = map.graph.n * map.starts.length * (lb + 1);
    if (work > CHECK_MAX_WORK) return { live: false, reason: 'large', T: lb, columns: null };
    try {
      var c = MPP.liveSolveCheck(g, map.starts, map.goals);
      c.reason = c.live ? null : 'cap';
      return c;
    } catch (e) {
      return { live: false, reason: 'error', message: e.message };
    }
  }
  function selectMap(map, keepPlan) {
    S.map = map;
    S.g = graphOf(map);
    if (!map.custom) S.lastDataMap = map;
    if (!S.checks[map.id] || map.custom) S.checks[map.id] = computeCheck(map, S.g);
    if (!keepPlan) { S.source = 'computed'; S.play = 'pre'; }
    S.hl = -1; S.pinned = false;
    if (!map.custom) S.view.transpose = false;
    resetZoom();
    ensureSelection();
    renderAll();
    autoZoom();
  }
  /** Phones: a large map at "fit" draws discs of about 10 px with unreadable labels, so start it zoomed in
   *  (the smallest zoom step that gives at least AUTO_ZOOM_PX per grid step; the view then scrolls). */
  function autoZoom() {
    var view = $('view');
    if (S.map.custom || !V.vb || !V.unit || (view.clientWidth || 1000) >= 520) return;
    var fit = Math.min(view.clientWidth / V.vb[2], view.clientHeight / V.vb[3]) * V.unit;
    if (!(fit > 0) || fit >= AUTO_ZOOM_PX) return;
    var steps = [1.5, 2, 3], z = steps[steps.length - 1];
    for (var i = 0; i < steps.length; i++) if (fit * steps[i] >= AUTO_ZOOM_PX) { z = steps[i]; break; }
    S.zoom = z;
    $('view-zoom').value = String(z);
    applyZoom();
    updateViewSub();
    view.scrollLeft = (view.scrollWidth - view.clientWidth) / 2;
    view.scrollTop = (view.scrollHeight - view.clientHeight) / 2;
  }

  // ------------------------------------------------------------------ rendering: everything
  function renderAll() {
    ensureSelection();
    loadPlan();
    renderMapCard();
    renderPlanPickers();
    buildView();
    renderMetrics();
    renderLive();
    renderNet();
    frame();
    saveHash();
  }
  function planChanged() {
    ensureSelection();
    loadPlan();
    renderPlanPickers();
    buildView();
    renderMetrics();
    renderLive();
    renderNet();
    frame();
    saveHash();
  }

  // ------------------------------------------------------------------ instance card
  function mapStats(map) {
    var gr = map.graph, bits = [];
    if (gr.rows && gr.cols) bits.push(gr.rows + ' × ' + gr.cols + ' grid');
    bits.push(plural(gr.n, 'vertex', 'vertices'));
    bits.push(plural(map.starts.length, 'robot'));
    var c = S.checks[map.id];
    if (c && c.columns) bits.push('makespan ILP at T = ' + c.T + ': ' + c.columns + ' columns × ' + c.rows + ' rows');
    return bits.join(' · ');
  }
  function renderMapCard() {
    var map = S.map, editing = !!map.custom;
    $('map-pick').hidden = editing;
    $('edit-panel').hidden = !editing;
    // caption and source always describe the shown instance (in edit mode: how it was made)
    setLinkedText($('map-caption'), map.caption);
    setLinkedText($('map-source'), map.source);
    if (!editing) {
      $('map-select').value = map.id;
      $('map-stats').textContent = mapStats(map);
      var small = map.graph.rows && map.graph.cols && map.graph.rows <= EDIT_MAX_SIDE && map.graph.cols <= EDIT_MAX_SIDE;
      $('edit-btn').textContent = small ? 'Edit this instance' : 'Edit your own instance';
    } else {
      renderEditPanel();
    }
    $('view-title').textContent = map.title;
    var gr = map.graph, isGrid = !!(gr.rows && gr.cols);
    var tp = $('opt-transpose');
    // not while editing: the Rows and Columns menus and the cells' labels name the grid as stored
    tp.parentNode.hidden = editing || !(isGrid && gr.rows !== gr.cols);
    tp.checked = S.view.transpose;
  }

  // ------------------------------------------------------------------ plan card
  function renderPlanPickers() {
    var paper = !!prePlan('paper');
    $('src-set').hidden = !paper;
    $('src-computed').checked = S.source !== 'paper';
    $('src-paper').checked = S.source === 'paper';
    var lockObj = S.source === 'paper';
    var notes = [];
    var offObj = [];
    OBJ.forEach(function (o) {
      var inp = $('obj-' + o.id), ok = objAvailable(o.id);
      inp.disabled = lockObj || !ok;
      inp.checked = !lockObj && o.id === S.objective;     // the paper's plan has no objective or split to pick
      var lab = inp.nextElementSibling;
      lab.title = ok ? '' : 'No plan for this objective on this map';
      if (!ok && !lockObj) offObj.push(o.label);
    });
    function andList(a) { return a.length === 1 ? String(a[0]) : a.slice(0, -1).join(', ') + ' and ' + a[a.length - 1]; }
    if (offObj.length) {
      notes.push(andList(offObj.map(function (x) { return x.toLowerCase(); })) +
                 (reachable() ? ' (not precomputed, and too large to solve live)' : ' (some goal cannot be reached from its start)'));
    }
    var offK = [];
    KS.forEach(function (k) {
      var inp = $('k-' + k), key = keyOf(S.objective, k);
      var ok = keyAvailable(S.objective, k);
      inp.disabled = lockObj || !ok;
      inp.checked = !lockObj && k === S.k;
      var lab = inp.nextElementSibling, tip;
      if (!ok) { tip = offKReason(k); offK.push(k); tip = tip.charAt(0).toUpperCase() + tip.slice(1); }
      else if (prePlan(key)) tip = 'Precomputed' + (inLivePlans(key) ? '; can be re-solved live' : '');
      else tip = 'No precomputed plan: solve it live';
      if (k === 1) tip = 'k = 1: exact, no split. ' + tip;
      lab.title = tip;
    });
    if (offK.length && !lockObj) {
      var kTxt = 'k = ' + andList(offK);
      notes.push(kTxt + ' (' + offKReason(offK[0]) + ')');
    }
    if (!lockObj && !reachable() && offObj.length === OBJ.length) notes = ['every objective and k (some goal cannot be reached from its start)'];
    if (lockObj) notes = ['The paper’s figure is a makespan plan without a split; pick “ILP (ours)” to choose an objective and k.'];
    var pn = $('pick-note');
    pn.textContent = lockObj ? notes[0] : notes.length ? 'Dimmed: ' + notes.join('; ') + '.' : '';
    pn.hidden = !notes.length;
    $('obj-set').classList.toggle('locked', lockObj);
  }

  /** Why the k button is dimmed (all the dimmed ones of a map and objective share the reason). */
  function offKReason(k) {
    if (!reachable()) return 'some goal cannot be reached from its start';
    if (!canTryLive()) return 'not precomputed, and too large to solve live';
    if (S.objective !== 'makespan') return 'not precomputed; the page solves the k-way split live for the makespan only';
    var lb = S.map.lower_bounds.makespan;
    if (lb === 0) return 'every robot starts at its goal, so there is nothing to split';
    return 'each piece would be shorter than one step: the makespan is at least ' + lb + ', so at most ' +
           (lb === 1 ? '1 piece' : lb + ' pieces');
  }

  function statusLine() {
    var info = S.info, p = info.plan, box = $('plan-status');
    clear(box);
    function badge(cls, text) { html('span', { class: 'badge ' + cls }, text, box); }
    if (info.kind === 'none') {
      var inf = S.source !== 'paper' ? provedInfeasible() : null, l = S.source !== 'paper' ? liveFor(planKey()) : null;
      if (inf) {
        badge('bad', 'infeasible');
        box.appendChild(document.createTextNode('No plan exists: ' +
          (inf === l ? 'the live solve' : 'the live solve of ' + planWords(inf.objective, inf.k)) +
          ' proved that the instance has no solution for any horizon, so no objective or split has a plan.'));
        return;
      }
      if (l && !l.paths && !l.cancelled) {
        badge('feasible', l.status === 'time_limit' ? 'time limit' : 'no plan');
        box.appendChild(document.createTextNode(l.status === 'time_limit'
          ? 'The live solve stopped at the time limit after ' + fmtSec(l.seconds) + ' without a plan (see below).'
          : 'The live solve ended without a plan (see below).'));
        return;
      }
      badge('feasible', 'not computed');
      box.appendChild(document.createTextNode(
        canTryLive() ? 'No precomputed plan for this choice; solve it live below.' : 'No plan for this choice.'));
      return;
    }
    if (info.kind === 'paper') {
      badge('paper', 'paper');
      box.appendChild(document.createTextNode(p.note || 'Plan transcribed from the paper.'));
      return;
    }
    var o = OBJ_BY_ID[S.objective];
    if (info.kind === 'live') {
      badge('live', 'live');
      var st = p.status === 'optimal' ? 'optimal ' + o.label.toLowerCase() : (S.k > 1 ? S.k + '-way split, feasible' : p.status);
      if (p.cutAtT0 !== undefined) st = 'feasible: the time limit stopped the horizon search, optimal at T = ' + p.cutAtT0 + ' only';
      var src = '';
      if (p.phases && p.phases.indexOf('joint') >= 0) {
        var nj = p.phases.filter(function (x) { return x === 'joint'; }).length;
        src = S.k > 1 ? ' ' + (nj === 1 ? 'One piece comes' : nj + ' pieces come') + ' from the joint-state search, not from an ILP (see the facts).' :
              S.objective === 'makespan' ? ' The plan comes from the joint-state search, not from an ILP (see the facts).' :
              ' The optimal makespan comes from the joint-state search (see the facts).';
      }
      box.appendChild(document.createTextNode('Solved in this browser (' + st + ', ' + fmtSec(p.seconds) + ').' + src));
      return;
    }
    badge(p.status === 'optimal' ? 'optimal' : 'feasible', p.status);
    var txt;
    if (p.status === 'optimal') txt = 'Proved optimal ' + o.label.toLowerCase() + ' (exact ILP, no split).';
    else if (S.k > 1) txt = S.k + '-way split: a valid plan, not necessarily optimal.';
    else txt = 'A valid plan, not proved optimal.';
    box.appendChild(document.createTextNode(txt + (p.note ? ' ' + p.note : '')));
  }

  function renderMetrics() {
    statusLine();
    var table = $('metrics'), thead = table.tHead, tbody = table.tBodies[0];
    clear(thead); clear(tbody);
    // "Precomp." with a soft hyphen: on a phone the header may break as "Pre-/comp.", so the table fits the card
    var cols = [], key = planKey(), lb = S.map.lower_bounds || {}, PRE = 'Pre\u00adcomp.';
    if (S.source === 'paper') {
      cols.push({ label: 'Paper', plan: prePlan('paper') });
      var ours = prePlan('makespan/k1');
      if (ours) cols.push({ label: PRE, plan: ours, title: 'our ILP plan (makespan, k = 1), precomputed with ' + (ours.backend || 'mppflow') });
    } else {
      var p = prePlan(key), l = liveFor(key);
      if (p) cols.push({ label: PRE, plan: p, title: 'precomputed with ' + (p.backend || 'mppflow') });
      if (l && l.paths && l.valid) cols.push({ label: 'Live', plan: l, title: 'solved in this browser' });
      if (prePlan('paper') && S.objective === 'makespan' && S.k === 1) cols.push({ label: 'Paper', plan: prePlan('paper') });
    }
    var tr = html('tr', null, null, thead);
    html('th', { scope: 'col' }, 'Objective', tr);
    cols.forEach(function (c) { html('th', { scope: 'col', title: c.title || null }, c.label, tr); });
    html('th', { scope: 'col', title: 'lower bound from shortest paths that ignore the other robots' }, 'Bound', tr);
    html('th', { scope: 'col', title: 'objective / lower bound for the played plan' }, 'Ratio', tr);
    var played = S.info.plan && S.info.plan.metrics ? S.info.plan.metrics : null;
    OBJ.forEach(function (o) {
      var row = html('tr', { class: S.source !== 'paper' && o.id === S.objective ? 'sel' : null }, null, tbody);
      html('th', { scope: 'row' }, o.label, row);
      cols.forEach(function (c) {
        var v = c.plan && c.plan.metrics ? c.plan.metrics[o.metric] : null;
        html('td', { class: v === null || v === undefined ? 'na' : null }, fmtNum(v), row);
      });
      var b = lb[o.lb];
      html('td', null, fmtNum(b), row);
      var v = played ? played[o.metric] : null, ratio = '–', cls = 'na';
      if (v !== null && v !== undefined && b) {
        ratio = (v / b).toFixed(3);
        cls = v === b ? 'ratio-1' : null;
      } else if (v === 0 && b === 0) { ratio = '1.000'; cls = 'ratio-1'; }
      html('td', { class: cls }, ratio, row);
    });
    if (!cols.length) {
      var r = html('tr', null, null, tbody), lk = S.source !== 'paper' ? liveFor(key) : null;
      html('td', { colspan: 3, class: 'na' },
           S.source !== 'paper' && provedInfeasible() ? 'No plan exists for this instance, so there are no metrics.'
           : lk && !lk.paths && !lk.cancelled && canTryLive() ? 'No plan yet, so there are no metrics.'
           : 'Solve live to fill in the metrics.', r);
    }
    updateTableHint();
    renderFacts();
  }
  /** The metrics table is wider than its card (a narrow phone): fade the cut edges and say it scrolls sideways
   *  (phones hide scroll bars). */
  function updateTableHint() {
    var sc = $('metrics-scroll'), wrap = $('metrics-wrap'), hint = $('metrics-hint');
    if (!sc || !wrap || !hint) return;
    var over = sc.scrollWidth > sc.clientWidth + 1;
    var left = over && sc.scrollLeft > 1, right = over && sc.scrollLeft + sc.clientWidth < sc.scrollWidth - 1;
    wrap.classList.toggle('more-left', left);
    wrap.classList.toggle('more-right', right);
    hint.hidden = !over;
    var text = !over ? '' : right ? 'Scroll the table sideways for the other columns →'
                                  : '← Scroll back to the objectives.';
    if (hint.textContent !== text) hint.textContent = text;
  }

  /** "3 (T = 4 (LP), 4, 5)": the horizon of every model solved, with the LP relaxations marked, so that a
   *  horizon listed twice reads as two different models (an LP relaxation and an ILP, or the makespan ILP
   *  and then the objective's ILP at the same T). stoppedLast: the last model (the horizon search's T_hi) was
   *  stopped by the time limit before it found a plan, so it is named apart, not counted as solved. */
  // The 'Models solved' fact. phases[i] is 'lp', 'ilp' or 'joint': a makespan plan (or a split's piece) that the
  // builtin engine took from its joint-state search after a failed horizon, at a horizon where no model was solved
  // (listed in order, not counted).
  function modelsSolved(count, horizons, phases, objective, k, stoppedLast) {
    if (!horizons || !horizons.length) return String(count);
    function isJoint(i) { return !!(phases && phases[i] === 'joint'); }
    var nModels = 0;
    for (var j = 0; j < horizons.length; j++) if (!isJoint(j)) nModels++;
    if (stoppedLast && nModels > 1 && count === nModels && !isJoint(horizons.length - 1)) {
      var Tst = horizons[horizons.length - 1];
      return modelsSolved(count - 1, horizons.slice(0, -1), phases ? phases.slice(0, -1) : phases, objective, k) +
             '; the time limit stopped the next model, at T = ' + Tst + ', before it found a plan';
    }
    var lp = 0, ilp = 0, joint = 0;
    var hs = horizons.map(function (T, i) {
      var ph = phases ? phases[i] : 'ilp';
      if (ph === 'lp') { lp++; return T + ' (LP)'; }
      if (ph === 'joint') { joint++; return T + ' (joint-state search)'; }
      ilp++;
      return String(T);
    });
    var txt = count + ' (T = ' + hs.join(', ') + ')';
    if (lp) txt += '; (LP): an LP relaxation, solved ' + (ilp ? 'before the ILPs' : 'first') + ' to skip horizons that are too short';
    if (joint) {
      txt += '; (joint-state search): no model was solved at this horizon; after a failed horizon, the search over ' +
             'joint configurations found it as the smallest feasible horizon, with its plan';
    }
    if (k > 1) txt += '; ' + (k === 2 ? 'both' : 'all ' + k) + ' pieces, in order';
    if (objective && objective !== 'makespan' && k === 1 && horizons.length > 1) {
      // the makespan stage has increasing horizons; the objective's models start again at the optimal makespan
      var cut = -1;
      for (var i = 1; i < horizons.length && cut < 0; i++) {
        if (horizons[i] <= horizons[i - 1] && !(phases && phases[i - 1] === 'lp')) cut = i;
      }
      var what = OBJ_BY_ID[objective].label.toLowerCase();
      if (cut > 0 && nModels === count) {
        var mBefore = 0, jBefore = 0;
        for (i = 0; i < cut; i++) { if (isJoint(i)) jBefore++; else mBefore++; }
        var nObj = count - mBefore;
        var first = mBefore === 0 ? 'the joint-state search finds' :
          (mBefore === 1 ? 'the first model' : 'the first ' + mBefore + ' models') +
          (jBefore ? ' and the joint-state search find' : (mBefore === 1 ? ' finds' : ' find'));
        txt += '; ' + first + ' the optimal makespan, ' +
               (nObj === 1 ? 'the last model optimizes' : 'the last ' + nObj + ' optimize') + ' the ' + what;
      } else {
        txt += '; the makespan stage comes first, then the models that optimize the ' + what;
      }
    }
    return txt;
  }
  function renderFacts() {
    var dl = $('facts'), info = S.info, p = info.plan;
    clear(dl);
    function fact(k, v) { if (v === null || v === undefined || v === '') return; html('dt', null, k, dl); html('dd', null, v, dl); }
    if (!p) return;
    if (info.kind === 'paper') {
      fact('Rotations', p.rotations !== undefined ? String(p.rotations) : null);
      return;
    }
    if (info.kind === 'pre') {
      var bv = DATA.backend_version && p.backend === DATA.backend ? String(DATA.backend_version) : '';
      var bname = bv && bv.toLowerCase().indexOf(String(p.backend).toLowerCase()) >= 0 ? bv : (p.backend || '?') + (bv ? ' ' + bv : '');
      fact('Solver', bname + ', ' + fmtSec(p.seconds));
      if (p.model) {
        var m = p.model.columns + ' columns × ' + p.model.rows + ' rows at T = ' + p.model.T;
        if (p.model.largest_piece) m += ' (largest piece)';
        fact('ILP model', m);
      }
      if (p.pieces && p.pieces.length > 1) {
        fact('Pieces', p.pieces.map(function (x) { return x.T; }).join(' + ') + ' steps');
      }
      if (p.solves !== undefined) fact('Models solved', modelsSolved(p.solves, p.horizons, p.phases, p.objective, p.k));
      fact('Rotations', p.rotations !== undefined ? String(p.rotations) : null);
      if (p.matches_original_java) fact('Original code', 'the Java code returns this same plan');
      if (p.rng_seed !== undefined) fact('Seed', 'MathRandom(' + p.rng_seed + ')');
      return;
    }
    if (info.kind === 'live') {
      fact('Solver', 'lp.js (builtin), ' + (p.mode === 'worker' ? 'Web Worker' : 'main thread') + ', ' + fmtSec(p.seconds));
      if (p.horizons && p.horizons.length) {
        var nm = p.phases ? p.phases.filter(function (x) { return x !== 'joint'; }).length : p.horizons.length;
        fact('Models solved', modelsSolved(nm, p.horizons, p.phases, p.objective, p.k, p.stoppedLast));
      }
      if (p.proof) fact('Optimality', p.proof);
      fact('Validation', p.valid ? 'MPP.validate: valid' : 'invalid: ' + p.error);
      fact('Rotations', String(p.rotations));
      fact('Seed', 'MathRandom(' + p.seed + ')');
    }
  }

  // ------------------------------------------------------------------ live card
  function renderLive() {
    var key = planKey(), info = $('live-info'), btn = $('live-btn'), map = S.map;
    var c = check(), p = prePlan(key), l = liveFor(key), solving = S.solving;
    var tl = $('live-tl');
    tl.value = String(S.timeLimit);
    var text, enabled = false, label = 'Solve live in your browser';
    if (S.source === 'paper') {
      text = "The paper's plan is a transcription of a figure. Switch to “ILP (ours)” to solve the instance live.";
    } else if (S.objective !== 'makespan' && S.k > 1) {
      text = 'The browser runs the k-way split for the makespan only; this ' + S.k + '-way ' +
             OBJ_BY_ID[S.objective].label.toLowerCase() + ' plan was precomputed in Python.';
    } else if (!reachable()) {
      text = 'Some goal cannot be reached from its start, so there is nothing to solve.';
    } else if (provedInfeasible() && !(l && l.infeasible)) {
      var pi = provedInfeasible();
      text = 'The live solve of ' + planWords(pi.objective, pi.k) + ' proved that this instance has no solution for any ' +
             'horizon, so no other objective or split has a plan either; there is nothing to solve. Edit the instance to try another.';
    } else if (l && l.infeasible) {
      text = 'The live solve proved that this instance has no solution for any horizon (see below), so no objective or ' +
             'split has a plan; there is nothing to solve. Edit the instance to try another.';
    } else if (map.custom) {
      enabled = true;
      var splitting = S.objective === 'makespan' && S.k > 1, big = tooBigForBrowser(c);
      if (c && c.columns) {
        text = 'The makespan model at the lower-bound horizon T = ' + c.T + ' has ' + c.columns + ' columns × ' + c.rows +
               ' rows (live cap ' + MPP.LIVE_MAX_COLUMNS + ' columns).';
        if (c.live) {
          // the cap is a size heuristic at one horizon: the optimal makespan can lie well above the lower bound
          text += ' Under the cap a solve is usually quick, but not always: the optimal makespan can lie well above the ' +
                  'lower bound, and a model there can still take long.';
        } else {
          label = 'Try solving live';
          if (splitting) {
            // the pieces are not known before the split runs, and on crowded instances they are not smaller
            text += ' The ' + S.k + '-way split solves ' + S.k + ' pieces one after the other. They are often smaller ' +
                    'models, but not always: on a crowded instance a piece can need as many steps as the whole plan, ' +
                    'and its model can be larger than the exact one.' +
                    (big ? ' The exact model is too large for the browser’s memory; if a piece is too, the page stops ' +
                           'the solve and says so.' : '');
          } else if (big) {
            enabled = false;
            var sc = S.objective === 'makespan' ? splitChoices(0) : '';
            text += ' That is too large to solve in the browser: its dense simplex solver would keep ' + memText(checkEntries(c)) +
                    ' in memory, and the tab could run out of memory before the time limit. ' +
                    (sc ? 'A k-way split (' + sc + ' above) may have pieces small enough; the page checks each piece and ' +
                          'stops with a message if one is too large too.'
                        : S.objective !== 'makespan' ? 'The makespan objective with a k-way split may have pieces small enough.'
                        : 'Try fewer robots or a smaller grid.');
          } else {
            text += ' It is over the cap: expect the time limit to stop the solve (on a model this large perhaps a second or ' +
                    'two late, since the solver looks at the clock between simplex steps)' +
                    (S.objective === 'makespan' && splitChoices(0) ? '; a k-way split (k above) may be faster, though its ' +
                     'pieces are not always smaller models.' : '.');
          }
        }
      } else {
        text = 'This instance is large: expect the time limit to stop the solve, or the page to stop it with a message ' +
               'when a model is too large for the browser’s memory.';
        label = 'Try solving live';
      }
      if (S.objective !== 'makespan' && enabled) text += ' The ' + OBJ_BY_ID[S.objective].label.toLowerCase() + ' search solves several larger models.';
    } else if (inLivePlans(key)) {
      enabled = true;
      text = 'The build checked that the builtin solver, run with the same seed, reaches this plan’s objective value. ' +
             'Solve it again here with the JavaScript port and compare.';
      if (p && p.live_check && p.live_check.python_seconds >= 1) {
        text += ' (When the page data was built, the same solve with the Python builtin solver took ' +
                fmtSec(p.live_check.python_seconds) + ' on the build machine.)';
      }
    } else if (canTryLive()) {
      enabled = true;
      label = 'Try solving live';
      text = p ? 'This plan was not verified for live solving: the builtin solver may need longer than the time limit.'
               : 'No precomputed plan for this choice; the browser can try it.';
    } else if (p && S.k > 1 && p.model && p.model.columns) {
      text = 'Not offered live here: the largest piece of this ' + S.k + '-way plan has ' + p.model.columns + ' columns at T = ' +
             p.model.T + (p.model.columns > MPP.LIVE_MAX_COLUMNS ? ' (live cap ' + MPP.LIVE_MAX_COLUMNS + ')' : '') +
             ', and the build did not verify that the builtin solver finishes in time. The plans shown were precomputed. ' +
             'Edit a small grid to solve live.';
    } else if (c && c.reason === 'cap') {
      text = 'Too large to solve live: the makespan model at T = ' + c.T + ' has ' + c.columns + ' columns (live cap ' +
             MPP.LIVE_MAX_COLUMNS + '). The plans shown were precomputed. Edit a small grid to solve live.';
    } else {
      text = 'Too large to solve live in the browser; the plans shown were precomputed. Edit a small grid to solve live.';
    }
    if (enabled && S.objective === 'makespan' && S.k > 1) {
      text += ' The time limit covers the whole split, ' + (S.k === 2 ? 'both' : 'all ' + S.k) + ' pieces together.';
    }
    // a solve of another map, or of another objective or k of this map: name it, so its progress is not read as
    // the shown choice's
    var other = solving && (solving.mapRef !== map || solving.key !== key);
    var cooling = !solving && coolingDown();
    if (enabled && other) {
      text += ' The page runs one live solve at a time: wait for the solve of ' + solveName(solving) + ' or cancel it.';
    }
    info.textContent = text;
    btn.textContent = label;
    btn.disabled = !enabled || !!solving || cooling;
    btn.title = other ? 'Busy with the live solve of ' + solveName(solving)
              : cooling ? 'Waiting for the cancelled solve to release its memory' : '';
    tl.disabled = !!solving;
    $('live-cancel').hidden = !solving;
    $('live-cancel').textContent = other ? 'Cancel that solve' : 'Cancel';
    $('live-cancel').disabled = !!(solving && solving.mode === 'main');
    $('live-progress').hidden = !solving;
    renderLiveResult(l, p);
    var both = p && l && l.paths && l.valid && S.source !== 'paper';
    $('play-set').hidden = !both;
    $('play-pre').checked = S.info.kind !== 'live';
    $('play-live').checked = S.info.kind === 'live';
    if (map.custom) syncPlayTool();
  }

  /** "“9-puzzle”" for a solve of another map; "the max distance plan (k = 1) of this map" for another choice here. */
  function solveName(sv) {
    if (sv.mapRef !== S.map) return '“' + sv.mapRef.title + '”';
    return planWords(sv.objective, sv.k) + ' of this map';
  }
  /** What to try after a live solve stopped at the time limit (why = 'time') or was refused as too large for the
   *  browser's memory (why = 'memory'): every split the page offers on this map, and the exact solve after a split,
   *  since a split is not always faster (its pieces are not always smaller models, and a piece whose intermediate
   *  configuration is hard to reach can take long). */
  function liveAdvice(l, why) {
    var c = check(), mk = l.objective === 'makespan', parts = [], triedText = [];
    // a makespan choice already solved live here without a plan: too large, or stopped at a time limit at least as
    // long as the current one. It is named as tried, not suggested again
    function failed(k) {
      var e = liveFor(keyOf('makespan', k));
      if (!e || e === l || e.paths) return '';
      if (e.tooLarge) return 'too large';
      return e.status === 'time_limit' && !(e.timeLimit < S.timeLimit) ? 'stopped at the time limit' : '';
    }
    function tried(k) { return !!failed(k); }
    KS.forEach(function (k) {
      if (failed(k) && keyAvailable('makespan', k) && !(mk && k === l.k)) triedText.push('k = ' + k + ' (' + failed(k) + ')');
    });
    var sc = splitChoices(mk ? l.k : 0, tried);
    var longer = why === 'time' && S.timeLimit < maxTimeLimit();
    var longerText = 'A longer time limit may help, though not always: on some small instances branch and bound stalls ' +
                     'at a horizon above the lower bound.';
    if (!mk) {
      var mkOpts = (tried(1) ? [] : ['k = 1']).concat(sc ? ['a k-way split: ' + sc] : []);
      if (mkOpts.length) parts.push('Try the makespan objective (' + mkOpts.join(', or ') + ').');
    } else {
      if (l.k > 1 && c && c.columns && !tooBigForBrowser(c) && !tried(1)) {
        parts.push('Try the exact solve (k = 1)' + (c.live ? ', whose model is under the live cap' : '') + ': a split is not ' +
                   'always faster, since its pieces are not always smaller models, and a piece can be slow when its ' +
                   'intermediate configuration is hard to reach.');
      }
      if (sc) {
        parts.push((parts.length ? 'Or try ' : 'Try ') + (l.k > 1 ? 'another split' : 'a k-way split') + ' (' + sc +
                   ' above): its pieces are often smaller models, though not on crowded instances.');
      }
    }
    if (!parts.length) parts.push('Try fewer robots or a smaller grid.');
    if (longer) parts.push(longerText);
    if (triedText.length) {
      parts.push('Already tried here without a plan, for the makespan: ' + triedText.join(', ') + '.');
    }
    return parts.join(' ');
  }
  /** The largest choice of the time-limit menu, in seconds. */
  function maxTimeLimit() { return Math.max.apply(null, [0].concat(timeLimitChoices())); }

  function renderLiveResult(l, p) {
    var box = $('live-result');
    clear(box);
    if (S.solving) {
      var sv = S.solving;
      var where = sv.mode === 'worker' ? 'in a Web Worker'
        : sv.mode === 'main' ? 'on the main thread (no Web Worker available: the page pauses until the solve ends)'
        : sv.mode === 'chunked' ? 'on the main thread, piece by piece (no Web Worker available)'
        : '(starting)';
      var what = sv.fn + (sv.fn === 'planSplit' ? ' with k = ' + sv.k : '');
      if (sv.mapRef !== S.map) {
        // the solve belongs to another map: name it, so its progress is not read as this map's
        html('p', null, 'Solving “' + sv.mapRef.title + '” (' + OBJ_BY_ID[sv.objective].label.toLowerCase() +
             (sv.k > 1 ? ', ' + sv.k + '-way split' : '') + '): MPP.' + what + ' ' + where + (sv.progress ? '; ' + sv.progress : '') +
             '. Its result is kept on that map; switch back to it to see the plan.', box);
        return;
      }
      if (sv.key !== planKey()) {
        // the same map, another objective or k: the shown choice is not the one being solved
        html('p', null, 'Solving ' + planWords(sv.objective, sv.k) + ' of this map: MPP.' + what + ' ' + where +
             (sv.progress ? '; ' + sv.progress : '') + '. Its result is kept under that choice; pick ' +
             planPick(sv.objective, sv.k) + ' to see it.', box);
        return;
      }
      html('p', null, 'Running MPP.' + what + ' ' + where + (sv.progress ? '; ' + sv.progress : '') + '.', box);
      return;
    }
    var key = planKey(), ld = S.lastDone;
    if (ld && ld.mapRef === S.map && ld.key === key) S.lastDone = ld = null;     // it is shown now
    // the last solve finished under another choice of this map (or on another map, when this choice has no live
    // result to show): say where its result is, until it is looked at
    if (ld && S.liveByMap[ld.mapRef.id] && S.liveByMap[ld.mapRef.id][ld.key] === ld.entry &&
        (ld.mapRef === S.map || (!ld.mapRef.custom && !l))) {
      var e = ld.entry, eo = OBJ_BY_ID[e.objective];
      var got = e.paths && e.valid ? 'a valid plan, ' + eo.label.toLowerCase() + ' ' + e.metrics[eo.metric]
              : e.cancelled ? 'cancelled' : e.tooLarge ? 'too large for the browser' : 'no plan (' + e.status + ')';
      html('p', { class: 'other-solve' }, 'The live solve of ' + (ld.mapRef === S.map ? planWords(e.objective, e.k) + ' of this map'
           : '“' + ld.mapRef.title + '” (' + planWords(e.objective, e.k) + ')') + ' finished: ' + got + '. ' +
           (ld.mapRef === S.map ? 'Pick ' + planPick(e.objective, e.k) : 'Switch back to that map') + ' to see it.', box);
    }
    // Solve waits for a stopped worker to release its memory: say why, unless this choice's own cancelled result
    // below says it
    if (coolingDown() && !(l && l.cancelled && !l.paths && S.source !== 'paper')) {
      html('p', { class: 'cool-note' }, (W.coolWhy || 'A live solve was stopped.') +
           ' Solve is available again in a moment, once the stopped solver has released its memory.', box);
    }
    if (!l || S.source === 'paper') return;
    var o = OBJ_BY_ID[l.objective];
    var p1 = html('p', null, null, box);
    if (l.error && !l.paths) {
      html('span', { class: l.cancelled || l.tooLarge ? 'warn' : 'bad' },
           l.cancelled ? 'Cancelled. ' : l.tooLarge ? 'Too large for the browser. ' : 'No plan. ', p1);
      p1.appendChild(document.createTextNode(l.error));
      if (l.cancelled && coolingDown()) {
        html('p', null, 'The stopped solver is still releasing its memory; you can solve again in a moment.', box);
      }
      if (l.tooLarge) html('p', null, liveAdvice(l, 'memory'), box);
      return;
    }
    if (!l.paths) {
      html('span', { class: 'warn' }, l.status === 'time_limit' ? 'Stopped at the time limit. ' : 'No plan found. ', p1);
      p1.appendChild(document.createTextNode('After ' + fmtSec(l.seconds) + ' the solver returned status “' + l.status + '”' +
        (p ? '; the precomputed plan is still shown.' : '.')));
      if (l.status === 'time_limit') html('p', null, liveAdvice(l, 'time'), box);
      return;
    }
    if (!l.valid) {
      html('span', { class: 'bad' }, 'Invalid plan: ', p1);
      p1.appendChild(document.createTextNode(l.error));
      return;
    }
    html('span', { class: 'ok' }, 'Valid plan', p1);
    p1.appendChild(document.createTextNode(' (checked with MPP.validate) in ' + fmtSec(l.seconds) + ': ' +
      o.label.toLowerCase() + ' ' + l.metrics[o.metric] + ', status “' + l.status + '”.'));
    if (l.cutAtT0 !== undefined) {
      html('p', null, 'The time limit stopped the horizon search (Algorithm 1) after the first horizon: this plan is ' +
        'optimal at T = ' + l.cutAtT0 + ' only, and a longer horizon might still give a better ' + o.label.toLowerCase() + '. ' +
        'Where the limit cuts depends on the speed of this machine, so the Python builtin solver may stop elsewhere.', box);
    }
    var lc = p && p.live_check;
    if (lc && lc.python_seconds >= 1 && l.seconds > 0 && l.k === p.k && l.objective === p.objective) {
      // measured once, on another machine: give both times, not a speed-up that the machines' load would skew
      if (lc.value !== null && lc.value !== undefined && (lc.status === 'optimal' || lc.status === 'feasible')) {
        html('p', null, 'The same solve with the Python builtin solver took ' + fmtSec(lc.python_seconds) +
          ' when the page data was built (on the build machine, so compare the two times loosely).', box);
      } else {
        // the build's run stopped at its per-model iteration budget without a plan: it is not a finished solve
        html('p', null, 'When the page data was built, the Python builtin solver did not finish this solve: it stopped at ' +
          'its budget of ' + (lc.work_limit ? lc.work_limit + ' simplex iterations per model' : 'simplex iterations') +
          ' after ' + fmtSec(lc.python_seconds) + ', with status “' + lc.status + '” and no plan. The browser run has no ' +
          'iteration budget, only the time limit.', box);
      }
    }
    if (p) {
      var pv = p.metrics[o.metric], lv = l.metrics[o.metric], cmp;
      if (samePaths(p.paths, l.paths)) cmp = 'Identical to the precomputed plan, path for path.';
      else if (pv === lv) cmp = 'Same ' + o.label.toLowerCase() + ' as the precomputed plan (' + pv + ', ' + (p.backend || 'mppflow') +
        '), with different paths: when several plans are optimal' + (S.k > 1 ? ' (here, for some split piece)' : '') +
        ', ' + (p.backend || 'that solver') + ' and the builtin solver may pick different ones.';
      else cmp = 'The precomputed plan has ' + o.label.toLowerCase() + ' ' + pv + ' (' + p.backend + ').';
      html('p', null, cmp, box);
    }
  }

  // ------------------------------------------------------------------ the graph view
  /** Draw the grid transposed? Never for an edited instance: its Rows and Columns menus and the cells' labels name
   *  the grid as stored (renderMapCard hides the box there). */
  function transposed() { return !!S.view.transpose && !S.map.custom; }
  function buildGeometry() {
    var map = S.map, xy = map.graph.xy, tr = transposed(), gr = map.graph;
    var pts = xy.map(function (p) { return p ? (tr ? [p[1], p[0]] : [p[0], p[1]]) : null; });
    var unit = Infinity;
    for (var v = 0; v < S.g.n; v++) {
      if (!pts[v]) continue;
      var nb = S.g.adj[v];
      for (var j = 0; j < nb.length; j++) {
        var w = nb[j];
        if (w <= v || !pts[w]) continue;
        var d = Math.hypot(pts[w][0] - pts[v][0], pts[w][1] - pts[v][1]);
        if (d > 1e-9 && d < unit) unit = d;
      }
    }
    if (!isFinite(unit)) unit = 1;
    var minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    pts.forEach(function (p) {
      if (!p) return;
      minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
      minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
    });
    var isGrid = !!(gr.rows && gr.cols);
    if (isGrid) {
      var W = (tr ? gr.rows : gr.cols) - 1, H = (tr ? gr.cols : gr.rows) - 1;
      minX = Math.min(minX, 0); minY = Math.min(minY, 0); maxX = Math.max(maxX, W); maxY = Math.max(maxY, H);
    }
    if (!isFinite(minX)) { minX = minY = 0; maxX = maxY = 1; }
    var pad = 0.62 * unit, padR = pad, padL = pad;
    if (map.vertex_names) { padR += 0.75 * unit; padL += 0.75 * unit; }
    V.midX = (minX + maxX) / 2;
    V.pts = pts; V.unit = unit; V.isGrid = isGrid;
    V.r = 0.34 * unit;
    var vb = [minX - padL, minY - pad, (maxX - minX) + padL + padR, (maxY - minY) + 2 * pad];
    var minW = 5 * unit, minH = 4 * unit;          // tiny graphs: keep discs at a sensible size
    if (vb[2] < minW) { vb[0] -= (minW - vb[2]) / 2; vb[2] = minW; }
    if (vb[3] < minH) { vb[1] -= (minH - vb[3]) / 2; vb[3] = minH; }
    V.vb = vb;
  }
  function cellPoint(c) {
    var cols = S.map.graph.cols, x = c % cols, y = Math.floor(c / cols);
    return transposed() ? [y, x] : [x, y];
  }
  /** One label size per map: as soon as some robot has a two-digit label, all labels use the smaller size. */
  function fontFor() { return (String(S.map.starts.length).length >= 2 ? 0.34 : 0.42) * V.unit; }

  function buildView() {
    var svgEl = $('view-svg');
    clear(svgEl, true);
    buildGeometry();
    var u = V.unit, map = S.map, gr = map.graph, editing = !!(map.custom && S.arrange);
    svgEl.setAttribute('viewBox', V.vb.map(r3).join(' '));
    $('view').style.setProperty('--view-aspect', String(r3(V.vb[2] / V.vb[3])));
    $('view').classList.toggle('editing', editing);
    applyZoom();
    var root = svg('g', null, svgEl);
    V.root = root;
    var bg = svg('rect', { x: V.vb[0], y: V.vb[1], width: V.vb[2], height: V.vb[3], fill: 'transparent', class: 'bg' }, root);
    bg.addEventListener('click', function () { if (S.pinned) setHighlight(-1, false); });
    V.gCells = svg('g', null, root);
    var gEdges = svg('g', null, root), gVerts = svg('g', null, root);
    V.gHl = svg('g', null, root);
    V.gInter = svg('g', null, root);
    V.gGoals = svg('g', null, root);
    V.gRobots = svg('g', null, root);
    V.gInterTop = svg('g', null, root);         // owner tags of covered intermediate goals
    V.gHlTop = svg('g', null, root);
    var gLabels = svg('g', null, root);          // vertex ids and names: above the robots, with a halo
    V.gEdit = svg('g', null, root);
    // obstacles: grid cells without a vertex
    var hasObst = drawObstacles(V.gCells, u);
    $('legend-obstacle').hidden = !hasObst && !editing;
    // edges and vertices
    drawEdges(gEdges, gVerts, u, 0.07, 0.075);
    for (var v = 0; v < S.g.n; v++) {
      var a = V.pts[v];
      if (!a) continue;
      if (S.view.ids) {
        vtext(gLabels, a[0] - 0.38 * u, a[1] - 0.4 * u, 0.2 * u, String(v), { class: 'g-label', 'stroke-width': 0.05 * u });
      }
      if (map.vertex_names && map.vertex_names[v]) {
        var left = a[0] < V.midX - 1e-9;
        vtext(gLabels, a[0] + (left ? -0.44 : 0.44) * u, a[1] + 0.09 * u, 0.24 * u, map.vertex_names[v],
              { class: 'g-name', 'text-anchor': left ? 'end' : 'start' });
      }
    }
    V.robotEls = []; V.goalEls = [];
    if (editing) {
      buildEditLayer();
      $('view-empty').hidden = true;
      showPieceLegend(false);
      clear($('pb-ticks'));
      $('pb-piece').hidden = true;
      buildInset();
      updateHlOptions();
      updateViewSub();
      setPlaybarEnabled(false);
      return;
    }
    // goals (unshaded discs, as the paper draws them)
    map.goals.forEach(function (gv, i) {
      var p = V.pts[gv];
      var gEl = svg('g', { class: 'goal', transform: 'translate(' + r3(p[0]) + ' ' + r3(p[1]) + ')' }, V.gGoals);
      svg('circle', { r: r3(0.86 * V.r), 'stroke-width': r3(0.04 * u) }, gEl);
      vtext(gEl, 0, 0, 0.86 * fontFor(), String(i + 1));
      V.goalEls.push(gEl);
    });
    V.gGoals.style.display = S.view.goals ? '' : 'none';
    // robots
    var empty = $('view-empty');
    if (!S.paths) {
      empty.hidden = false;
      empty.textContent = canTryLive() && S.source !== 'paper' ? 'No plan yet. Use “Solve live in your browser” to compute one.' : 'No plan for this choice.';
    } else empty.hidden = true;
    map.starts.forEach(function (sv, i) {
      var p = V.pts[sv];
      var rEl = svg('g', { class: 'robot', transform: 'translate(' + r3(p[0]) + ' ' + r3(p[1]) + ')' }, V.gRobots);
      var title = svg('title', null, rEl);
      title.textContent = 'Robot ' + (i + 1) + ': vertex ' + sv + ' → ' + map.goals[i] +
        (map.vertex_names ? ' (' + map.vertex_names[sv] + ' → ' + map.vertex_names[map.goals[i]] + ')' : '');
      svg('circle', { r: r3(V.r), 'stroke-width': r3(0.05 * u) }, rEl);
      vtext(rEl, 0, 0, fontFor(), String(i + 1));
      rEl.addEventListener('pointerenter', function () { if (!S.pinned) setHighlight(i, false); });
      rEl.addEventListener('pointerleave', function () { if (!S.pinned) setHighlight(-1, false); });
      rEl.addEventListener('click', function (e) {
        e.stopPropagation();
        if (S.pinned && S.hl === i) setHighlight(-1, false); else setHighlight(i, true);
      });
      if (!S.paths) rEl.style.display = 'none';
      V.robotEls.push(rEl);
    });
    buildInset();
    if (S.view.colors) applyRobotColors();
    showPieceLegend(!!(S.paths && S.inter && S.inter.length));
    renderTicks();
    updateHlOptions();
    setHighlight(S.hl, S.pinned);
    setPlaybarEnabled(!!S.paths);
    updateViewSub();
  }

  function drawObstacles(parent, u) {
    var gr = S.map.graph, has = false;
    if (!V.isGrid) return false;
    var present = {};
    gr.xy.forEach(function (p) { if (p) present[p[1] * gr.cols + p[0]] = true; });
    for (var c = 0; c < gr.rows * gr.cols; c++) {
      if (present[c]) continue;
      has = true;
      var q = cellPoint(c);
      svg('rect', { class: 'g-obstacle', x: r3(q[0] - 0.42 * u), y: r3(q[1] - 0.42 * u), width: r3(0.84 * u), height: r3(0.84 * u),
                    rx: r3(0.1 * u), 'stroke-width': r3(0.03 * u) }, parent);
    }
    return has;
  }
  function drawEdges(gEdges, gVerts, u, ew, vr) {
    for (var v = 0; v < S.g.n; v++) {
      var a = V.pts[v];
      if (!a) continue;
      S.g.adj[v].forEach(function (w) {
        if (w <= v || !V.pts[w]) return;
        var b = V.pts[w];
        svg('line', { class: 'g-edge', x1: a[0], y1: a[1], x2: b[0], y2: b[1], 'stroke-width': r3(ew * u) }, gEdges);
      });
      if (gVerts) svg('circle', { class: 'g-vertex', cx: a[0], cy: a[1], r: r3(vr * u) }, gVerts);
    }
  }

  /** Is the map so crowded that the robots hide most goal discs? Then show the goal configuration beside the view. */
  function wantsInset() {
    var m = S.map;
    return !(m.custom && S.arrange) && S.view.goals && S.g.n <= 64 && m.starts.length * 2 >= S.g.n;
  }
  function buildInset() {
    var fig = $('goal-inset'), el = $('goal-svg');
    V.insetGoalEls = [];
    clear(el, true);
    if (!wantsInset()) { fig.hidden = true; return; }
    fig.hidden = false;
    var u = V.unit, map = S.map;
    // a tight box around the graph (the main view pads small graphs)
    var minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    V.pts.forEach(function (p) {
      if (!p) return;
      minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]); minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
    });
    if (V.isGrid) {
      var tr = transposed(), gr = map.graph;
      minX = Math.min(minX, 0); minY = Math.min(minY, 0);
      maxX = Math.max(maxX, (tr ? gr.rows : gr.cols) - 1); maxY = Math.max(maxY, (tr ? gr.cols : gr.rows) - 1);
    }
    var pad = 0.55 * u, vb = [minX - pad, minY - pad, maxX - minX + 2 * pad, maxY - minY + 2 * pad];
    el.setAttribute('viewBox', vb.map(r3).join(' '));
    // about 40 px per grid step, within 140..300 px beside the caption; on phones the caption goes below
    // (CSS) and the inset may take the whole width of the card, so that the labels stay readable
    var unitsW = vb[2] / u, unitsH = vb[3] / u;
    var cardW = fig.parentNode.clientWidth || 600, narrow = cardW < 520;
    fig.classList.toggle('stacked', narrow);
    var avail = narrow ? Math.max(100, cardW - 24) : Math.max(100, cardW - 190);
    var px = clamp((narrow ? 36 : 40) * Math.max(unitsW, unitsH), narrow ? 90 : 140, Math.min(narrow ? 360 : 300, avail));
    var sc = px / Math.max(unitsW, unitsH);
    el.style.width = Math.round(unitsW * sc) + 'px';
    el.style.height = Math.round(unitsH * sc) + 'px';
    // the note counts what the main view hides at t = 0 (a goal disc under a robot), not a guess from the robot count
    var A = map.starts.length, onStart = {}, covered = 0;
    map.starts.forEach(function (v) { onStart[v] = true; });
    map.goals.forEach(function (v) { if (onStart[v]) covered++; });
    $('goal-inset-note').textContent = A === S.g.n
      ? 'Every vertex holds a robot, so in the main view the goal discs are hidden under the robots.'
      : 'Robots occupy ' + A + ' of the ' + S.g.n + ' vertices; at t = 0 they hide ' +
        (covered === 0 ? 'no goal disc in the main view, but they cover goals as they move.'
         : (covered === A ? (A === 2 ? 'both' : 'all ' + A) + ' goal discs' : covered + ' of the ' + A + ' goal discs') +
           ' in the main view.');
    var g = svg('g', null, el);
    drawObstacles(g, u);
    drawEdges(g, null, u, 0.06, 0);
    var fs = (String(map.starts.length).length >= 2 ? 0.4 : 0.48) * u;
    map.goals.forEach(function (gv, i) {
      var p = V.pts[gv];
      var gEl = svg('g', { class: 'gi-goal', transform: 'translate(' + r3(p[0]) + ' ' + r3(p[1]) + ')' }, g);
      svg('circle', { r: r3(0.42 * u), 'stroke-width': r3(0.05 * u) }, gEl);
      vtext(gEl, 0, 0, fs, String(i + 1));
      if (S.view.colors) gEl.querySelector('circle').style.stroke = flowColor(i);
      gEl.classList.toggle('hl', S.hl === i);
      gEl.classList.toggle('dim', S.hl >= 0 && S.hl !== i);
      V.insetGoalEls.push(gEl);
    });
  }

  function applyZoom() {
    var view = $('view'), el = $('view-svg'), z = S.zoom || 1;
    view.classList.toggle('zoomed', z > 1);
    if (z <= 1 || !V.vb) { el.style.width = ''; el.style.height = ''; sizeBadges(); return; }
    el.style.width = ''; el.style.height = '';
    var cw = view.clientWidth || 300, ch = view.clientHeight || 300;
    var sc = Math.min(cw / V.vb[2], ch / V.vb[3]) * z;
    el.style.width = Math.round(V.vb[2] * sc) + 'px';
    el.style.height = Math.round(V.vb[3] * sc) + 'px';
    sizeBadges();
  }

  function applyRobotColors() {
    V.robotEls.forEach(function (el, i) {
      el.querySelector('circle').style.fill = S.view.colors ? robotFill(i) : '';
    });
    V.goalEls.forEach(function (el, i) {
      el.querySelector('circle').style.stroke = S.view.colors ? flowColor(i) : '';
    });
    (V.insetGoalEls || []).forEach(function (el, i) {
      el.querySelector('circle').style.stroke = S.view.colors ? flowColor(i) : '';
    });
  }

  function updateViewSub() {
    var info = S.info, sub = $('view-sub'), txt;
    if (S.map.custom && S.arrange) {
      txt = S.edit.tool === 'obstacles' ? 'Editing: click a cell to add or remove an obstacle'
          : 'Editing: drag the ' + (S.edit.tool === 'goals' ? 'goals' : 'starts');
    }
    else if (!info || info.kind === 'none') txt = 'no plan';
    else {
      var src = info.kind === 'paper' ? 'paper’s figure' : info.kind === 'live' ? 'live result' : 'precomputed';
      var what = info.kind === 'paper' ? '' : OBJ_BY_ID[S.objective].label.toLowerCase() + (S.k > 1 ? ', ' + S.k + '-way split' : '') + ', ';
      txt = what + src + ' · ' + plural(S.T, 'step');
    }
    if (S.zoom > 1) txt += ' · zoomed ' + S.zoom + '×: scroll (or Shift + arrow keys) to pan';
    sub.textContent = txt;
    // the keys that work here: editing has no plan to play, but the obstacle grid and the discs take the arrow keys
    // (txt already says how to pan a zoomed view)
    var keys = '';
    if (S.map.custom && S.arrange) {
      keys = S.edit.tool === 'obstacles'
        ? 'Tab to the cell grid, move between the cells with the arrow keys and press Enter to add or remove an obstacle'
        : 'Tab to a disc and use the arrow keys to move it';
    } else if (S.paths) {
      keys = 'Press space to play or pause, the arrow keys to step' + (S.zoom > 1 ? ', Shift + arrow keys to pan' : '');
    }
    $('view').setAttribute('aria-label', S.map.title + ': ' + txt + '.' + (keys ? ' ' + keys + '.' : ''));
    updateViewHint();
  }

  var VIEW_HINT = null;   // the static hint of index.html (plan mode)
  /** The hint under the view: how to highlight a robot while a plan plays; how to edit in edit mode. */
  function updateViewHint() {
    var h = $('view-hint');
    if (!h) return;
    if (VIEW_HINT === null) VIEW_HINT = h.textContent.replace(/\s+/g, ' ').trim();
    var editing = !!(S.map.custom && S.arrange), txt = VIEW_HINT;
    if (editing && S.edit.tool === 'obstacles') {
      txt = 'Click or tap a cell to add or remove an obstacle (with the keyboard: Tab to the grid, move between the cells ' +
            'with the arrow keys and press Enter). ' +
            'Solve live to get a plan to play.';
    } else if (editing) {
      txt = 'Drag a ' + (S.edit.tool === 'goals' ? 'goal (unshaded disc)' : 'start (shaded disc)') + ' to another vertex; ' +
            'dropping it on another robot’s disc swaps the two. With the keyboard, focus a disc and use the arrow keys. ' +
            'Solve live to get a plan to play.';
    }
    if (h.textContent !== txt) h.textContent = txt;
    // highlighting a robot's path needs a plan: no Highlight menu (and no legend entry for it) while the instance is edited
    $('hl-select').parentNode.hidden = editing;
    $('legend-hl').hidden = editing;
  }

  function updateHlOptions() {
    var sel = $('hl-select'), n = S.map.starts.length;
    if (sel.options.length !== n + 1) {
      clear(sel);
      html('option', { value: '-1' }, 'no robot', sel);
      for (var i = 0; i < n; i++) html('option', { value: String(i) }, 'robot ' + (i + 1), sel);
    }
    sel.value = String(S.hl);
  }

  function setHighlight(i, pinned) {
    if (i >= S.map.starts.length) i = -1;
    S.hl = i; S.pinned = i >= 0 && !!pinned;
    var gHl = V.gHl;
    if (!gHl) return;
    clear(gHl);
    if (V.gHlTop) clear(V.gHlTop);
    // the other robots turn opaque gray (nothing underneath shows through) and their goals are hidden
    V.robotEls.forEach(function (el, j) {
      el.classList.toggle('hl', j === i);
      el.classList.toggle('dim', i >= 0 && j !== i);
    });
    V.goalEls.forEach(function (el, j) {
      el.classList.toggle('hl', j === i);
      el.classList.toggle('dim', i >= 0 && j !== i);
    });
    (V.insetGoalEls || []).forEach(function (el, j) {
      el.classList.toggle('hl', j === i);
      el.classList.toggle('dim', i >= 0 && j !== i);
    });
    if (i >= 0 && S.paths && !(S.map.custom && S.arrange)) {
      var p = S.paths[i], pts = [], u = V.unit;
      for (var t = 0; t < p.length; t++) if (t === 0 || p[t] !== p[t - 1]) pts.push(V.pts[p[t]]);
      var s = V.pts[p[0]];
      svg('circle', { class: 'hl-start', cx: s[0], cy: s[1], r: r3(0.44 * u), 'stroke-width': r3(0.05 * u) }, gHl);
      if (pts.length > 1) {
        svg('polyline', { class: 'hl-path', points: pts.map(function (q) { return r3(q[0]) + ',' + r3(q[1]); }).join(' '),
                          'stroke-width': r3(0.11 * u) }, gHl);
      }
      // its goal: a dashed ring above the robots, so it stays visible when another robot sits on it
      var gq = V.pts[S.map.goals[i]];
      svg('circle', { class: 'hl-goal', cx: gq[0], cy: gq[1], r: r3(0.45 * u), 'stroke-width': r3(0.06 * u),
                      'stroke-dasharray': r3(0.14 * u) + ' ' + r3(0.09 * u) }, V.gHlTop);
    }
    var sel = $('hl-select');
    if (sel) sel.value = String(S.hl);
    S.interIdx = -2;
    if (V.gInter && S.paths) { drawInter(S.t); updateInterTags(S.t); }
    highlightFlows();
  }

  /** The legend entries of a split plan: intermediate goals, their owner tags and the piece ticks. */
  function showPieceLegend(on) {
    $('legend-inter').hidden = !on;
    $('legend-inter-tag').hidden = !on;
    $('legend-ticks').hidden = !(on && S.bounds && S.bounds.length);
  }

  /** Piece boundaries under the time bar (explained by a legend entry and the "piece j of k" text). */
  function renderTicks() {
    var box = $('pb-ticks');
    clear(box);
    if (!S.paths || !S.T) return;
    S.bounds.forEach(function (b) {
      var s = html('span', null, null, box);
      s.style.left = (100 * b / S.T) + '%';
    });
  }

  /** "piece j of k" next to the time label while a split plan plays (a boundary time starts the next piece). */
  function pieceText(t) {
    if (!S.paths || !S.bounds || !S.bounds.length) return '';
    var j = 0;
    while (j < S.bounds.length && S.bounds[j] <= t + 1e-9) j++;
    return 'piece ' + (j + 1) + ' of ' + (S.bounds.length + 1);
  }

  function smooth(f) { return f * f * (3 - 2 * f); }

  /** Draw the robots at time S.t. */
  function frame() {
    var T = S.T, t = clamp(S.t, 0, T);
    // reduced motion: the robots jump from step to step, so the label, the scrubber, the intermediate goals
    // and the network cursor all show the whole time step the robots are drawn at
    var still = !!reduceMotion.matches;
    if (still) t = Math.floor(t + 1e-9);
    var lab = $('pb-label'), range = $('pb-time');
    range.step = still ? '1' : '0.01';
    range.max = String(Math.max(T, 0.001));
    range.value = String(t);
    var shown = (Math.abs(t - Math.round(t)) < 1e-9) ? String(Math.round(t)) : t.toFixed(1);
    // arranging an edited instance: no plan is played (the discs stand at the starts), so no time is shown
    var playable = S.paths && !(S.map.custom && S.arrange);
    lab.textContent = playable ? 't = ' + shown + ' / ' + T : 't = –';
    var piece = pieceText(t), pEl = $('pb-piece');
    pEl.hidden = !piece;
    if (pEl.textContent !== piece) pEl.textContent = piece;
    range.setAttribute('aria-valuetext', playable ? 'time ' + shown + ' of ' + T + (piece ? ', ' + piece : '') : 'no plan');
    if (!S.paths || !V.robotEls || !V.robotEls.length || (S.map.custom && S.arrange)) { updateNetCursor(t); return; }
    var i0 = Math.floor(t), f = t - i0;
    if (i0 >= T) { i0 = T; f = 0; }
    var e = smooth(f), i1 = Math.min(i0 + 1, T);
    for (var i = 0; i < S.paths.length; i++) {
      var p = S.paths[i], a = V.pts[p[i0]], b = V.pts[p[i1]];
      var x = a[0] + (b[0] - a[0]) * e, y = a[1] + (b[1] - a[1]) * e;
      var el = V.robotEls[i];
      el.setAttribute('transform', 'translate(' + r3(x) + ' ' + r3(y) + ')');
      // at its goal only once it got there: not while it is still moving along the edge into it
      var gl = S.map.goals[i];
      el.classList.toggle('home', p[i0] === gl && (f === 0 || p[i1] === gl));
    }
    drawInter(t);
    updateInterTags(t);
    updateNetCursor(t);
  }

  /** The next intermediate goals of a split plan: brown dotted discs with a centred label (Fig. 8(b)), drawn
   *  under the goal discs and the robots (the ring stays visible around them; an owner tag names it then). With a robot highlighted, only its intermediate goal. */
  function drawInter(t) {
    var g = V.gInter;
    if (!g) return;
    var idx = -1;
    if (S.inter && S.inter.length && S.view.inter) {
      idx = 0;
      while (idx < S.bounds.length && S.bounds[idx] <= t + 1e-9) idx++;
      if (idx >= S.inter.length) idx = -1;
    }
    if (idx === S.interIdx) return;
    S.interIdx = idx;
    clear(g);
    if (V.gInterTop) clear(V.gInterTop);
    V.interTags = [];
    if (idx < 0) return;
    var u = V.unit, cfg = S.inter[idx], fs = 0.8 * fontFor(), tfs = 0.27 * u;
    cfg.forEach(function (v, r) {
      if (S.hl >= 0 && r !== S.hl) return;
      var p = V.pts[v];
      var gg = svg('g', { class: 'inter', transform: 'translate(' + r3(p[0]) + ' ' + r3(p[1]) + ')' }, g);
      svg('circle', { r: r3(0.42 * u), 'stroke-width': r3(0.05 * u), 'stroke-dasharray': '0 ' + r3(0.105 * u) }, gg);
      vtext(gg, 0, 0, fs, String(r + 1));
      // the owner's number on a small tag at the ring's upper right, above the robots: shown while another
      // robot or another robot's goal disc covers the ring (and so its centred label)
      if (!V.gInterTop) return;
      var lab = String(r + 1), w = (0.62 * lab.length + 0.6) * tfs, h = 1.25 * tfs;
      var tx = p[0] + 0.3 * u, ty = p[1] - 0.4 * u;
      var tg = svg('g', { class: 'inter-tag', transform: 'translate(' + r3(tx) + ' ' + r3(ty) + ')' }, V.gInterTop);
      svg('rect', { x: r3(-w / 2), y: r3(-h / 2), width: r3(w), height: r3(h), rx: r3(h / 2),
                    'stroke-width': r3(0.025 * u) }, tg);
      vtext(tg, 0, 0, tfs, lab);
      tg.style.display = 'none';
      V.interTags.push({ r: r, v: v, el: tg });
    });
  }

  /** Show the owner tag of each intermediate goal whose centred label is covered at time t by a robot other than
   *  its owner, or by the (shown) goal disc of another robot. */
  function updateInterTags(t) {
    var tags = V.interTags;
    if (!tags || !tags.length || !S.paths) return;
    var T = S.T, i0 = Math.floor(clamp(t, 0, T)), f = clamp(t, 0, T) - i0;
    if (i0 >= T) { i0 = T; f = 0; }
    var i1 = f > 0 ? Math.min(i0 + 1, T) : i0, cover = {};
    S.paths.forEach(function (p, r) {
      [p[Math.min(i0, p.length - 1)], p[Math.min(i1, p.length - 1)]].forEach(function (v) {
        if (cover[v] === undefined) cover[v] = [];
        if (cover[v].indexOf(r) < 0) cover[v].push(r);
      });
    });
    if (S.view.goals) {
      S.map.goals.forEach(function (v, r) {
        if (S.hl >= 0 && r !== S.hl) return;      // the other goals are hidden while a robot is highlighted
        if (cover[v] === undefined) cover[v] = [];
        if (cover[v].indexOf(r) < 0) cover[v].push(r);
      });
    }
    tags.forEach(function (o) {
      var c = cover[o.v] || [], show = c.some(function (r) { return r !== o.r; });
      o.el.style.display = show ? '' : 'none';
    });
  }

  // ------------------------------------------------------------------ playback
  var raf = 0, lastTs = 0;
  function setPlaybarEnabled(on) {
    ['pb-reset', 'pb-back', 'pb-play', 'pb-fwd', 'pb-time'].forEach(function (id) { $(id).disabled = !on; });
  }
  function updatePlayButton() {
    var b = $('pb-play');
    b.classList.toggle('playing', S.playing);
    b.setAttribute('aria-label', S.playing ? 'Pause' : 'Play');
  }
  function play() {
    if (!S.paths || S.T === 0) return;
    if (S.t >= S.T - 1e-9) S.t = 0;
    S.playing = true;
    updatePlayButton();
    lastTs = 0;
    raf = window.requestAnimationFrame(tick);
  }
  function stopPlay() {
    S.playing = false;
    // reduced motion shows whole steps only: pause on the step that is drawn
    if (reduceMotion.matches && S.paths) S.t = Math.floor(clamp(S.t, 0, S.T) + 1e-9);
    if (raf) window.cancelAnimationFrame(raf);
    raf = 0;
    updatePlayButton();
  }
  function tick(ts) {
    if (!S.playing) return;
    if (!lastTs) lastTs = ts;
    var dt = Math.min(0.1, (ts - lastTs) / 1000);
    lastTs = ts;
    S.t = Math.min(S.T, S.t + dt * S.speed);
    frame();
    if (S.t >= S.T) { stopPlay(); announceTime(); return; }
    raf = window.requestAnimationFrame(tick);
  }
  function stepBy(d) {
    stopPlay();
    if (!S.paths) return;
    // reduced motion draws the whole step floor(S.t) (frame()); step from that shown step, not from a fractional
    // pause time, or the first step back would land on the step already on screen
    var t = reduceMotion.matches ? Math.floor(S.t + 1e-9) : S.t;
    S.t = d > 0 ? Math.min(S.T, Math.floor(t + 1e-9) + 1) : Math.max(0, Math.ceil(t - 1e-9) - 1);
    frame();
  }
  function seek(t) { stopPlay(); S.t = clamp(t, 0, S.T); frame(); }
  /** Say the time once, in a polite live region, after a step, a jump, a pause or the end of playback. The label
   *  next to the scrubber is not live: playback rewrites it on every animation frame, which would flood a screen
   *  reader. The scrubber says the time itself (aria-valuetext) while it has the focus. */
  function announceTime() {
    var a = $('pb-announce');
    if (!a || S.playing || document.activeElement === $('pb-time')) return;
    a.textContent = $('pb-time').getAttribute('aria-valuetext') || '';
  }
  function togglePlay() { if (S.playing) stopPlay(); else play(); announceTime(); }

  // ------------------------------------------------------------------ live solving
  var W = { worker: null, broken: false, nextId: 1, pending: {}, coolUntil: 0, coolTimer: 0, coolWhy: '' };
  /** After a worker is terminated it can stay alive for a few seconds while its (large) heap is released: about
   *  2 s for a 7419 x 3602 model in Chrome. A new solve started at once would build a second large model next to
   *  it, and quick Solve / Cancel clicks stacked several and crashed the tab, so Solve waits for this long
   *  (twice the measured time per million dense numbers, at least 0.3 s, at most 6 s). */
  function cooldownMs(job) {
    var e = job.entries;
    return e === null || e === undefined ? 3000 : clamp(300 + 30 * e / 1e6, 300, 6000);
  }
  function coolingDown() { return W.coolUntil > nowMs(); }
  /** why: what stopped the solve, said in the live card while Solve waits (a cancel the user did not press). */
  function startCooldown(ms, why) {
    W.coolUntil = nowMs() + ms;
    W.coolWhy = why || '';
    window.clearTimeout(W.coolTimer);
    W.coolTimer = window.setTimeout(function () {
      W.coolUntil = 0;
      W.coolWhy = '';
      var lost = liveFocusLost();
      if (S.map) renderLive();
      if (lost) focusLive();
    }, ms + 20);
  }
  /** Focus left the live card's controls (a disabled or hidden button drops it to <body>). */
  function liveFocusLost() {
    var a = document.activeElement;
    return !a || a === document.body || a === $('live-btn') || a === $('live-cancel') || a === $('live-result');
  }
  /** Put the focus back on the live card: Solve when it can be pressed, else the result text. */
  function focusLive() {
    var b = $('live-btn'), c = $('live-cancel');
    if (S.solving && !c.hidden && !c.disabled) c.focus();
    else if (!b.disabled) b.focus();
    else $('live-result').focus();
  }
  function errorFrom(o) {
    var e = new Error(o && o.message ? o.message : String(o));
    e.name = o && o.name ? o.name : 'Error';
    return e;
  }
  function getWorker() {
    if (W.worker) return W.worker;
    if (W.broken || typeof window.Worker !== 'function' || typeof window.Blob !== 'function' || !MPP.createWorker) return null;
    try {
      var w = MPP.createWorker();
      w.onmessage = function (ev) {
        var d = ev.data || {}, pend = W.pending[d.id];
        if (!pend) return;
        delete W.pending[d.id];
        if (d.ok) pend.resolve(d.result); else pend.reject(errorFrom(d.error));
      };
      w.onerror = function (ev) {
        if (ev && ev.preventDefault) ev.preventDefault();
        W.broken = true;
        try { w.terminate(); } catch (e) { /* already gone */ }
        W.worker = null;
        var pend = W.pending;
        W.pending = {};
        Object.keys(pend).forEach(function (id) { runMain(pend[id].job, pend[id].resolve, pend[id].reject); });
      };
      W.worker = w;
      return w;
    } catch (e) {
      W.broken = true;
      return null;
    }
  }
  function runJob(job) {
    return new Promise(function (resolve, reject) {
      var w = getWorker();
      if (!w) { runMain(job, resolve, reject); return; }
      job.mode = 'worker';
      var id = W.nextId++;
      job.id = id;
      W.pending[id] = { resolve: resolve, reject: reject, job: job };
      w.postMessage({ id: id, fn: job.fn, args: job.args, seed: job.seed });
    });
  }
  /** Main-thread fallback. A k-way split runs piece by piece (as MPP.planSplit does), yielding between pieces;
   *  the time limit covers the whole split (timeLimitPerPiece: false), as in the worker, and every piece gets
   *  the job's lp options (the memory guard maxEntries and the wall-clock deadlineMs), as MPP.planSplit passes them. */
  function runMain(job, resolve, reject) {
    var args = jsonClone(job.args), rng = new MPP.MathRandom(job.seed);
    var opts = args[args.length - 1];
    if (job.fn === 'planSplit' && job.k > 1) { job.mode = 'chunked'; planSplitChunked(job, args, rng, resolve, reject); renderLive(); return; }
    job.mode = 'main';
    renderLive();
    window.setTimeout(function () {
      if (job.cancelled) { reject(errorFrom({ name: 'Cancelled', message: 'cancelled' })); return; }
      try {
        opts.rng = rng;
        resolve(jsonClone(MPP[job.fn].apply(null, args)));
      } catch (e) { reject(e); }
    }, 60);
  }
  function planSplitChunked(job, args, rng, resolve, reject) {
    var g = args[0], starts = args[1], goals = args[2], k = args[3], tl = args[4].timeLimit, lp = args[4].lp;
    var splits = Math.round(Math.log2(k)), msg = null, pieces = [], solves = [], failures = [], i = 0, t0 = nowMs();
    var outOfTime = false, skipped = 0;
    function next() {
      if (job.cancelled) { reject(errorFrom({ name: 'Cancelled', message: 'cancelled' })); return; }
      if (i === k) { finish(); return; }
      var lim = tl - (nowMs() - t0) / 1000;
      if (tl > 0 && lim <= 0) { outOfTime = true; skipped = k - i; finish(); return; }
      job.progress = 'piece ' + (i + 1) + ' of ' + k;
      renderLive();
      window.setTimeout(function () {
        // a Cancel (or an edit of the instance) handled after next() queued this piece: do not start it
        if (job.cancelled) { reject(errorFrom({ name: 'Cancelled', message: 'cancelled' })); return; }
        try {
          // a piece over the memory budget throws ModelTooLarge (rejected below, reported as in the worker)
          var r = MPP.minMakespan(g, msg[i], msg[i + 1], { timeLimit: tl > 0 ? lim : null, rng: rng, lp: lp });
          solves.push(r);
          var pp = r.paths;
          if (pp !== null && MPP.pathSetError(pp, g, msg[i], msg[i + 1]) !== null) pp = null;
          pieces[i] = pp;
        } catch (e) {
          if (!(e instanceof MPP.Infeasible)) { reject(e); return; }
          failures.push('piece ' + i + ': ' + e.message);
          pieces[i] = null;
        }
        i++;
        if (pieces[i - 1] === null) { skipped = k - i; finish(); return; }   // the merged plan is lost: stop here
        next();
      }, 0);
    }
    function finish() {
      var paths = pieces[0] === undefined ? null : pieces[0];
      for (var j = 1; j < k; j++) paths = MPP.mergePaths(paths, pieces[j] === undefined ? null : pieces[j]);
      var status = paths === null ? (outOfTime || solves.some(function (x) { return x.status === 'time_limit'; }) ? 'time_limit' : 'failed') : 'feasible';
      var attempts = [];
      solves.forEach(function (x) { x.attempts.forEach(function (a) { attempts.push(a); }); });
      var stats = MPP.attemptStats(attempts);   // as MPP.planSplit
      stats.skipped_pieces = skipped;
      stats.seconds = (nowMs() - t0) / 1000;
      resolve(jsonClone({ paths: paths, status: status, k: k, intermediateGoals: msg === null ? null : msg.slice(1, k), failures: failures,
                          attempts: attempts, stats: stats }));
    }
    window.setTimeout(function () {
      if (job.cancelled) { reject(errorFrom({ name: 'Cancelled', message: 'cancelled' })); return; }
      try {
        MPP.splitPrecheck(g, starts, goals);
        msg = MPP.splitConfigurations(g, starts, goals, splits, { rng: rng, failures: failures });
      } catch (e) { reject(e); return; }
      if (job.cancelled) { reject(errorFrom({ name: 'Cancelled', message: 'cancelled' })); return; }
      if (msg === null) { finish(); return; }   // the split heuristic found no intermediate configuration
      next();
    }, 30);
  }

  function startSolve() {
    if (S.solving || S.source === 'paper' || coolingDown()) return;
    if (S.objective !== 'makespan' && S.k > 1) return;    // the engine splits the makespan only
    if (S.map.custom && tooBigForBrowser(check()) && !(S.objective === 'makespan' && S.k > 1)) return;
    var refocus = liveFocusLost();
    var map = S.map, key = planKey(), obj = S.objective, k = S.k, p = prePlan(key);
    var seed = p && p.rng_seed !== undefined && p.rng_seed !== null ? p.rng_seed : 1;
    // The time limit covers the whole solve: a k-way split gives each piece the time left, not the full limit,
    // and every LP gets one absolute deadline (lp.js deadlineMs, wall clock), so that building the models
    // (not counted by the engine's per-model limits) cannot push the solve past the limit either.
    // maxEntries: every model the solve builds (each horizon, each piece) is checked against the memory budget
    var opts = { timeLimit: S.timeLimit, lp: { deadlineMs: Date.now() + S.timeLimit * 1000, maxEntries: LIVE_MAX_ENTRIES } };
    if (obj === 'makespan') opts.timeLimitPerPiece = false;
    var g = plainGraph(S.g), s = map.starts.slice(), t = map.goals.slice();
    var args = obj === 'makespan' ? [g, s, t, k, opts] : [g, s, t, opts];
    var job = { key: key, mapRef: map, objective: obj, k: k, fn: SOLVER_FN[obj], args: args, seed: seed, t0: nowMs(), mode: null,
                entries: checkEntries(check()) };
    S.solving = job;
    job.timer = window.setInterval(function () {
      $('live-elapsed').textContent = ((nowMs() - job.t0) / 1000).toFixed(1) + ' s';
    }, 100);
    $('live-elapsed').textContent = '0.0 s';
    // runJob sets job.mode synchronously (worker, main or chunked), so the progress text is right from the start
    runJob(job).then(function (res) { solveDone(job, res, null); }, function (err) { solveDone(job, null, err); });
    renderLive();
    if (refocus) focusLive();         // the Solve button is disabled now: move to Cancel
  }
  function cancelSolve(why) {
    var job = S.solving;
    if (!job) return;
    job.cancelled = true;
    if (job.mode === 'worker' && W.worker) {
      try { W.worker.terminate(); } catch (e) { /* ignore */ }
      W.worker = null;
      startCooldown(cooldownMs(job), why);
      var pend = W.pending[job.id];
      delete W.pending[job.id];
      if (pend) pend.reject(errorFrom({ name: 'Cancelled', message: 'cancelled' }));
    }
  }
  /** The engine's Infeasible message as a sentence after "No plan.", without restating it: "the instance has no
   *  solution for any horizon (joint-state search)" already says that the instance has no solution. */
  function infeasibleText(m) {
    m = String(m || '').replace(/\.$/, '');
    var cap = m.charAt(0).toUpperCase() + m.slice(1);
    if (/^the instance has no solution for any horizon \(joint-state search\)$/i.test(m)) {
      return 'The instance has no solution for any horizon (proved by the joint-state search).';
    }
    if (/^(the instance has )?no (solution|plan)\b/i.test(m)) return cap + '.';
    return 'The instance has no solution: ' + m + '.';
  }
  // Where the optimality of a live plan comes from when it is not an ILP: the makespan loop's joint-state search
  // (the plan of the smallest feasible horizon), or the horizon search's joint-state stage (the optimum over all
  // horizons). null otherwise (the facts list the models).
  function proofText(res, entry, job) {
    var hs = res.extra && res.extra.horizon_search;
    var jT = [];
    (entry.phases || []).forEach(function (ph, i) { if (ph === 'joint') jT.push(entry.horizons[i]); });
    if (hs && hs.proof === 'joint-state search') {
      return 'proved over all horizons by the joint-state search (an exact search over joint configurations)';
    }
    if (job.objective === 'makespan' && job.k === 1 && jT.length && res.status === 'optimal') {
      return 'proved by the joint-state search: T = ' + jT[0] + ' is the smallest feasible horizon, and the plan is the search’s own';
    }
    if (job.k > 1 && jT.length) {
      return 'none (a split); ' + (jT.length === 1 ? 'one piece comes' : jT.length + ' pieces come') +
             ' from the joint-state search, not from an ILP';
    }
    return null;
  }
  function solveDone(job, res, err) {
    window.clearInterval(job.timer);
    var refocus = liveFocusLost();
    if (S.solving === job) S.solving = null;
    if (job.stale) { renderLive(); if (refocus) focusLive(); return; }
    var map = job.mapRef, g = map === S.map ? S.g : graphOf(map);
    var seconds = (nowMs() - job.t0) / 1000;
    var entry = { objective: job.objective, k: job.k, seed: job.seed, seconds: seconds, mode: job.mode, paths: null, valid: false,
                  timeLimit: job.args[job.args.length - 1].timeLimit };
    if (err) {
      entry.cancelled = err.name === 'Cancelled';
      entry.tooLarge = err.name === 'ModelTooLarge';
      entry.status = entry.cancelled ? 'cancelled' : entry.tooLarge ? 'too_large' : 'error';
      entry.error = entry.cancelled ? 'The solve was stopped after ' + fmtSec(seconds) + '.' :
        entry.tooLarge ? 'After ' + fmtSec(seconds) + ' the page stopped the solve before the browser runs out of memory: ' +
                         err.message + '.' :
        (err.name === 'Infeasible' ? infeasibleText(err.message) : err.name + ': ' + err.message);
      // the page passes no t_max and no extra steps, so an Infeasible proof covers every horizon, objective and k
      if (err.name === 'Infeasible') { entry.infeasible = true; entry.status = 'infeasible'; }
    } else {
      entry.status = res.status;
      entry.horizons = res.stats && res.stats.horizons ? res.stats.horizons : (res.attempts || []).map(function (a) { return a.T; });
      if (res.attempts && res.attempts.length === entry.horizons.length) {
        entry.phases = res.attempts.map(function (a) { return a.phase === 'lp' || a.phase === 'joint' ? a.phase : 'ilp'; });
      }
      entry.proof = proofText(res, entry, job);
      entry.intermediate_goals = res.intermediateGoals || null;
      // a horizon search cut by the time limit after its first horizon: optimal at T0 only (engine horizonSearch)
      if (res.extra && res.extra.time_limit_at_T_hi) {
        var hs = res.extra.horizon_search || {};
        entry.cutAtT0 = hs.T0 !== undefined && hs.T0 !== null ? hs.T0 : '?';
        // the model at T_hi, if the limit stopped it before it found a plan: listed, but not solved
        var la = res.attempts && res.attempts.length ? res.attempts[res.attempts.length - 1] : null;
        if (la && la.T === hs.T_hi && entry.horizons.length && entry.horizons[entry.horizons.length - 1] === la.T &&
            (la.objective === null || la.objective === undefined)) entry.stoppedLast = true;
      }
      if (res.paths) {
        // Trim trailing steps in which every robot only waits, so that the makespan (paths length - 1, as
        // mppflow.plan_metrics computes it) is the latest arrival time of T-RO Sec. II-B.
        var P = trimmed(res.paths);
        var e2 = MPP.pathSetError(P, g, map.starts, map.goals);
        entry.paths = P;
        entry.valid = e2 === null;
        entry.error = e2;
        try { entry.metrics = MPP.metrics(P, map.goals); } catch (e) { entry.metrics = null; }
        try { entry.rotations = MPP.hasCycles(P); } catch (e) { entry.rotations = null; }
      }
    }
    var st = S.liveByMap[map.id] || (S.liveByMap[map.id] = {});
    st[job.key] = entry;
    // a solve the user cancelled has no result to point to (the card would send them to look at a stopped solve)
    if (!entry.cancelled) S.lastDone = { mapRef: map, key: job.key, objective: job.objective, k: job.k, entry: entry };
    if (map === S.map) {
      if (planKey() === job.key && entry.paths && entry.valid) {
        S.play = 'live';
        if (map.custom) S.arrange = false;
      }
      var keepT = S.info && S.info.kind !== 'live' && !(entry.paths && entry.valid && planKey() === job.key);
      var tKeep = S.t;
      planChanged();
      if (keepT) { S.t = Math.min(tKeep, S.T); frame(); }
      if (map.custom) renderEditPanel();
    } else renderLive();
    if (refocus) focusLive();         // the Cancel button is hidden now: back to Solve (or the result)
  }

  // ------------------------------------------------------------------ edit mode
  function editFromMap(map) {
    var gr = map.graph, rows = gr.rows, cols = gr.cols, present = {};
    gr.xy.forEach(function (p) { present[p[1] * cols + p[0]] = true; });
    var blocked = new Set();
    for (var c = 0; c < rows * cols; c++) if (!present[c]) blocked.add(c);
    function cell(v) { return gr.xy[v][1] * cols + gr.xy[v][0]; }
    return { rows: rows, cols: cols, blocked: blocked, starts: map.starts.map(cell), goals: map.goals.map(cell),
             origin: 'copied from “' + map.title + '” (' + (map.source || '') + ')', tool: 'starts' };
  }
  function editFromProblem(prob, rows, cols, origin) {
    var present = {}, g = prob.graph;
    g.xy.forEach(function (p) { present[p[1] * cols + p[0]] = true; });
    var blocked = new Set();
    for (var c = 0; c < rows * cols; c++) if (!present[c]) blocked.add(c);
    function cell(v) { return g.xy[v][1] * cols + g.xy[v][0]; }
    return { rows: rows, cols: cols, blocked: blocked, starts: prob.starts.map(cell), goals: prob.goals.map(cell), origin: origin,
             tool: 'starts' };
  }
  function randomEdit(rows, cols, frac, n, seed) {
    var prob = MPP.createGridProblem(rows, cols, frac, n, new MPP.MathRandom(seed));
    return editFromProblem(prob, rows, cols, 'regenerated, seed ' + seed + ': createGridProblem(' + rows + ', ' + cols + ', ' + frac +
                           ', ' + n + ', MathRandom(' + seed + '))');
  }
  function compileEdit() {
    var E = S.edit, rows = E.rows, cols = E.cols, idx = new Array(rows * cols), xy = [];
    for (var c = 0; c < rows * cols; c++) {
      if (E.blocked.has(c)) { idx[c] = -1; continue; }
      idx[c] = xy.length;
      xy.push([c % cols, Math.floor(c / cols)]);
    }
    var adj = [];
    for (c = 0; c < rows * cols; c++) {
      if (idx[c] < 0) continue;
      var x = c % cols, y = Math.floor(c / cols), nb = [];
      if (y > 0 && idx[c - cols] >= 0) nb.push(idx[c - cols]);
      if (x > 0 && idx[c - 1] >= 0) nb.push(idx[c - 1]);
      nb.push(idx[c]);
      if (x < cols - 1 && idx[c + 1] >= 0) nb.push(idx[c + 1]);
      if (y < rows - 1 && idx[c + cols] >= 0) nb.push(idx[c + cols]);
      adj.push(nb);
    }
    var g = MPP.makeGraph(adj, xy, rows, cols);
    var starts = E.starts.map(function (c2) { return idx[c2]; }), goals = E.goals.map(function (c2) { return idx[c2]; });
    var lb = MPP.lowerBounds(g, starts, goals);
    E.idx = idx;
    return {
      id: 'custom', custom: true, title: 'Your instance',
      caption: 'Your own instance on a ' + rows + ' × ' + cols + ' grid.', source: E.origin,
      graph: { n: g.n, adj: adj, xy: xy, rows: rows, cols: cols }, _g: g,
      starts: starts, goals: goals, lower_bounds: lb, plans: {}, live_plans: [], live: true,
    };
  }
  /** A live solve of the edited instance is moot once the instance changes or edit mode ends: stop it, and say so
   *  (why) in the live card while Solve waits for the stopped worker. */
  function staleCustomSolve(why) {
    if (S.solving && S.solving.mapRef && S.solving.mapRef.custom) {
      S.solving.stale = true;
      cancelSolve(why);
    }
  }
  function refreshEdit(focus) {
    staleCustomSolve('Changing the instance stopped the live solve of the earlier one.');
    S.liveByMap.custom = {};
    S.arrange = true;
    S.play = 'pre';
    var m = compileEdit();
    var keepObj = S.objective, keepK = S.k;
    S.map = m; S.g = m._g;
    S.checks.custom = computeCheck(m, S.g);
    S.objective = keepObj; S.k = keepK;
    S.hl = -1; S.pinned = false;
    renderAll();
    if (focus) {
      var el = V.gEdit && V.gEdit.querySelector('[data-robot="' + focus.i + '"][data-layer="' + focus.layer + '"]');
      if (el) el.focus();
    }
  }
  function editMsg(text, ok) {
    var m = $('edit-msg');
    m.textContent = text || '';
    m.classList.toggle('ok', !!ok);
  }
  /** Back to Zoom: fit (as selectMap does for a new map). */
  function resetZoom() {
    if (S.zoom !== 1) { S.zoom = 1; $('view-zoom').value = '1'; }
  }
  function enterEdit() {
    var m = S.map, gr = m.graph;
    stopPlay();
    resetZoom();      // the 24 x 18 map opens auto-zoomed on phones: the editing grid starts at "fit"
    try {
      if (gr.rows && gr.cols && gr.rows <= EDIT_MAX_SIDE && gr.cols <= EDIT_MAX_SIDE) S.edit = editFromMap(m);
      else S.edit = randomEdit(5, 5, 0.2, 6, 1);
    } catch (e) {
      S.edit = { rows: 4, cols: 4, blocked: new Set(), starts: [0, 15], goals: [15, 0], origin: 'a 4 × 4 grid', tool: 'starts' };
    }
    S.edit.tool = 'starts';
    editMsg('');
    syncEditInputs();
    resetRandomInputs();
    refreshEdit();
    $('tool-starts').focus();
  }
  function exitEdit() {
    staleCustomSolve('Leaving edit mode stopped the live solve of your instance.');
    S.edit = null; S.arrange = false;
    selectMap(S.lastDataMap || DATA.maps[0]);
    $('edit-btn').focus();
  }
  /** Rows and Columns show the edited grid. Robots, Obstacles and Seed are the settings of Randomize only
   *  (grouped under "New random instance"), so later edits leave them as the user set them, except that Robots
   *  is lowered to the cells of a smaller grid. */
  function syncEditInputs() {
    var E = S.edit, n = $('ed-n'), max = E.rows * E.cols;
    $('ed-rows').value = String(E.rows);
    $('ed-cols').value = String(E.cols);
    n.max = String(max);
    if (+n.value > max) n.value = String(max);
  }
  /** Start the Randomize settings from the instance (on entering edit mode). */
  function resetRandomInputs() {
    $('ed-n').value = String(S.edit.starts.length);
  }
  function syncPlayTool() {
    if (!S.edit) return;
    var l = liveFor(planKey()), has = !!(l && l.paths && l.valid);
    var tp = $('tool-play');
    tp.disabled = !has;
    if (!S.arrange && has) { tp.checked = true; }
    else {
      if (!has && !S.arrange) S.arrange = true;
      $('tool-' + S.edit.tool).checked = true;
    }
  }
  function renderEditPanel() {
    var E = S.edit, m = S.map, bits = [];
    bits.push(plural(m.graph.n, 'vertex', 'vertices'));
    bits.push(plural(E.blocked.size, 'obstacle'));
    bits.push(plural(m.starts.length, 'robot'));
    var lb = m.lower_bounds;
    if (lb.makespan === null) bits.push('some goal is unreachable');
    else bits.push('makespan ≥ ' + lb.makespan);
    $('edit-stats').textContent = bits.join(' · ');
    syncEditInputs();
    syncPlayTool();
  }

  /** The obstacle grid's one tab stop moves to cell c. */
  function rovingCell(c) {
    if (!S.edit || S.edit.focusCell === c) return;
    var g = V.gEdit, old = g.querySelector('.edit-cell[tabindex="0"]'), cell = g.querySelector('.edit-cell[data-cell="' + c + '"]');
    if (old) old.setAttribute('tabindex', '-1');
    if (cell) cell.setAttribute('tabindex', '0');
    S.edit.focusCell = c;
  }
  function buildEditLayer() {
    var E = S.edit, u = V.unit, map = S.map, tool = E.tool;
    var g = V.gEdit;
    // cells for the obstacle tool: on top of the graph (the discs above them are passive), with a dashed outline
    if (tool === 'obstacles') {
      // one tab stop for the whole grid (a roving tabindex): the arrow keys move between the cells, Enter or Space
      // toggles the focused one, and Tab leaves the grid
      var nCells = E.rows * E.cols;
      if (!(E.focusCell >= 0 && E.focusCell < nCells)) E.focusCell = 0;
      for (var c = 0; c < nCells; c++) {
        (function (c) {
          var q = cellPoint(c), blocked = E.blocked.has(c);
          var r = svg('rect', { class: 'edit-cell' + (blocked ? ' blocked' : ''), x: r3(q[0] - 0.46 * u), y: r3(q[1] - 0.46 * u),
                                width: r3(0.92 * u), height: r3(0.92 * u), rx: r3(0.1 * u),
                                tabindex: c === E.focusCell ? '0' : '-1', role: 'button',
                                'stroke-width': r3(0.03 * u), 'data-cell': String(c),
                                'aria-label': 'Cell row ' + (Math.floor(c / E.cols) + 1) + ', column ' + (c % E.cols + 1) + ': ' +
                                  (blocked ? 'obstacle, press Enter to clear' : 'free, press Enter to make an obstacle') }, g);
          r.addEventListener('click', function () { E.focusCell = c; toggleObstacle(c); });
          r.addEventListener('focus', function () { rovingCell(c); });
          r.addEventListener('keydown', function (e) {
            if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); toggleObstacle(c, true); return; }
            var x = c % E.cols, y = Math.floor(c / E.cols);
            var d = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] }[e.key];
            var to = -1;
            if (d && !e.shiftKey) {
              var nx = x + d[0], ny = y + d[1];
              to = nx >= 0 && nx < E.cols && ny >= 0 && ny < E.rows ? ny * E.cols + nx : c;
            } else if (e.key === 'Home') to = e.ctrlKey ? 0 : y * E.cols;
            else if (e.key === 'End') to = e.ctrlKey ? nCells - 1 : y * E.cols + E.cols - 1;
            if (to < 0) return;
            e.preventDefault();
            var cell = g.querySelector('.edit-cell[data-cell="' + to + '"]');
            if (cell) cell.focus();
          });
        })(c);
      }
    }
    // The active layer (the one being dragged) is drawn at full size. The passive layer sits on its vertex too;
    // where an active disc covers that vertex, it becomes a small badge on the disc's lower-right rim, drawn on top.
    var activeLayer = tool === 'goals' ? 'goals' : tool === 'starts' ? 'starts' : null;
    var topLayer = tool === 'goals' ? 'goals' : 'starts', topAt = {};
    (topLayer === 'starts' ? map.starts : map.goals).forEach(function (v, i) { topAt[v] = { i: i }; });
    function disc(layer, v, i, mode) {      // mode: 'active' | 'under' | 'badge'
      var p = V.pts[v], off = mode === 'badge' ? 0.25 * u : 0;
      var rr = mode === 'badge' ? 0.17 * u : layer === 'starts' ? V.r : 0.86 * V.r;
      var el = svg('g', { class: (layer === 'starts' ? 'robot' : 'goal') + ' edit-disc' + (mode === 'active' ? '' : ' passive') +
                                 (mode === 'badge' ? ' badge' : ''),
                          transform: 'translate(' + r3(p[0] + off) + ' ' + r3(p[1] + off) + ')',
                          'data-robot': String(i), 'data-layer': layer }, g);
      svg('circle', { r: r3(rr), 'stroke-width': r3((mode === 'badge' ? 0.03 : 0.045) * u) }, el);
      var fs = mode === 'badge' ? 0.5 * fontFor() : layer === 'goals' && mode !== 'active' ? 0.86 * fontFor() : fontFor();
      vtext(el, 0, 0, fs, String(i + 1));
      if (S.view.colors && layer === 'starts') el.querySelector('circle').style.fill = robotFill(i);
      if (mode === 'active') {
        var xy0 = map.graph.xy[v];
        el.setAttribute('tabindex', '0');
        el.setAttribute('role', 'button');
        el.setAttribute('aria-label', 'Robot ' + (i + 1) + ' ' + (layer === 'starts' ? 'start' : 'goal') + ' at row ' + (xy0[1] + 1) +
                        ', column ' + (xy0[0] + 1) + '. Arrow keys move it.');
        bindDrag(el, i, layer);
      }
      if (mode === 'badge') el._at = [p[0], p[1], off];
      else if (topAt[v] && layer === topLayer) topAt[v].el = el;
      return el;
    }
    var badges = [];
    var order = tool === 'goals' ? ['starts', 'goals'] : ['goals', 'starts'];
    order.forEach(function (layer) {
      var arr = layer === 'starts' ? map.starts : map.goals;
      arr.forEach(function (v, i) {
        if (layer === topLayer) disc(layer, v, i, layer === activeLayer ? 'active' : 'under');
        else if (topAt[v]) badges.push([layer, v, i]);
        else disc(layer, v, i, 'under');
      });
    });
    V.editBadges = badges.map(function (b) {
      // the covering disc names the robot of the badge (its number can be small on a phone)
      var top = topAt[b[1]] && topAt[b[1]].el;
      if (top) {
        var also = 'robot ' + (b[2] + 1) + '’s ' + (b[0] === 'starts' ? 'start' : 'goal') + ' is on this vertex too';
        if (top.getAttribute('aria-label')) top.setAttribute('aria-label', top.getAttribute('aria-label').replace(/\.( Arrow keys)/, '; ' + also + '.$1'));
        svg('title', null, top).textContent = 'Robot ' + (topAt[b[1]].i + 1) + ' ' + (topLayer === 'starts' ? 'start' : 'goal') + '; ' + also;
      }
      return disc(b[0], b[1], b[2], 'badge');
    });
    sizeBadges();
    V.dropRing = svg('circle', { class: 'drop-target', r: r3(0.46 * u), 'stroke-width': r3(0.05 * u), cx: -99, cy: -99 }, g);
    V.dropRing.style.display = 'none';
  }

  /** The badges' numbers are drawn at half the disc font, about 6 px on a phone: scale each badge up to
   *  BADGE_MIN_PX on screen (at most 1.6 times, so it stays on the disc's rim). Called after a build and a zoom. */
  var BADGE_MIN_PX = 9;
  function sizeBadges() {
    var b = V.editBadges, el = $('view-svg');
    if (!b || !b.length || !el || !V.vb) return;
    var w = el.getBoundingClientRect().width, pxPerUnit = w > 0 ? w / V.vb[2] : 0;
    var fpx = 0.5 * fontFor() * pxPerUnit;
    var f = fpx > 0 && fpx < BADGE_MIN_PX ? Math.min(1.6, BADGE_MIN_PX / fpx) : 1;
    b.forEach(function (g) {
      if (!g._at) return;
      var off = g._at[2] * (1 + 0.5 * (f - 1));
      g.setAttribute('transform', 'translate(' + r3(g._at[0] + off) + ' ' + r3(g._at[1] + off) + ')' +
                     (f > 1 ? ' scale(' + r3(f) + ')' : ''));
    });
  }

  function svgPoint(evt) {
    var s = $('view-svg'), pt = s.createSVGPoint();
    pt.x = evt.clientX; pt.y = evt.clientY;
    var m = s.getScreenCTM();
    if (!m) return { x: 0, y: 0 };
    var q = pt.matrixTransform(m.inverse());
    return { x: q.x, y: q.y };
  }
  function nearestVertex(x, y, maxD) {
    var best = -1, bd = maxD;
    for (var v = 0; v < V.pts.length; v++) {
      var p = V.pts[v];
      if (!p) continue;
      var d = Math.hypot(p[0] - x, p[1] - y);
      if (d < bd) { bd = d; best = v; }
    }
    return best;
  }
  function bindDrag(el, i, layer) {
    var drag = null;
    el.addEventListener('pointerdown', function (e) {
      if (e.button > 0) return;
      e.preventDefault();
      try { el.focus({ preventScroll: true }); } catch (x) { el.focus(); }
      drag = { id: e.pointerId, moved: false, p0: svgPoint(e) };
      if (el.nextSibling && el.nextSibling !== V.dropRing) V.gEdit.insertBefore(el, V.dropRing);   // on top, before capture
      try { el.setPointerCapture(e.pointerId); } catch (x) { /* ignore */ }
      el.classList.add('dragging');
    });
    el.addEventListener('pointermove', function (e) {
      if (!drag || e.pointerId !== drag.id) return;
      var p = svgPoint(e);
      if (Math.hypot(p.x - drag.p0.x, p.y - drag.p0.y) > 0.12 * V.unit) drag.moved = true;
      if (!drag.moved) return;
      el.setAttribute('transform', 'translate(' + r3(p.x) + ' ' + r3(p.y) + ')');
      var v = nearestVertex(p.x, p.y, 0.6 * V.unit);
      if (v >= 0) {
        V.dropRing.style.display = '';
        V.dropRing.setAttribute('cx', V.pts[v][0]); V.dropRing.setAttribute('cy', V.pts[v][1]);
      } else V.dropRing.style.display = 'none';
    });
    function end(e, cancel) {
      if (!drag || e.pointerId !== drag.id) return;
      var moved = drag.moved;
      drag = null;
      el.classList.remove('dragging');
      if (cancel || !moved) { if (moved) refreshEdit(); return; }
      var p = svgPoint(e), v = nearestVertex(p.x, p.y, 0.6 * V.unit);
      if (v < 0) { refreshEdit({ i: i, layer: layer }); return; }
      moveRobotTo(i, layer, vertexCell(v));
    }
    el.addEventListener('pointerup', function (e) { end(e, false); });
    el.addEventListener('pointercancel', function (e) { end(e, true); });
    el.addEventListener('keydown', function (e) {
      var dir = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] }[e.key];
      if (!dir) return;
      e.preventDefault();
      var arr = layer === 'starts' ? S.map.starts : S.map.goals, v = arr[i], a = V.pts[v], best = -1;
      S.g.adj[v].forEach(function (w) {
        if (w === v) return;
        var b = V.pts[w], dx = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(dx, dy) || 1;
        if ((dx * dir[0] + dy * dir[1]) / L > 0.7) best = w;
      });
      if (best >= 0) { moveRobotTo(i, layer, vertexCell(best)); return; }
      // no edge that way (an obstacle or the border): jump over the obstacles to the nearest vertex on the same row
      // or column in that direction, if any, as a pointer can drop the disc on any vertex
      var far = Infinity;
      for (var w = 0; w < V.pts.length; w++) {
        if (w === v || !V.pts[w]) continue;
        var b = V.pts[w], dx = b[0] - a[0], dy = b[1] - a[1];
        var along = dx * dir[0] + dy * dir[1], across = Math.abs(dx * dir[1] - dy * dir[0]);
        if (along > 1e-6 && across < 0.25 * V.unit && along < far) { far = along; best = w; }
      }
      if (best < 0) { editMsg('No vertex in that direction.'); return; }
      moveRobotTo(i, layer, vertexCell(best), 'Jumped over ' + plural(Math.max(1, Math.round(far / V.unit) - 1), 'obstacle') + '.');
    });
  }
  function vertexCell(v) { var p = S.map.graph.xy[v]; return p[1] * S.edit.cols + p[0]; }
  /** Move robot i's start or goal (layer) to the cell, swapping with the robot there; note: said first (a jump). */
  function moveRobotTo(i, layer, cell, note) {
    var arr = S.edit[layer], j = arr.indexOf(cell);
    if (j === i) { refreshEdit({ i: i, layer: layer }); return; }
    var old = arr[i];
    if (j >= 0) arr[j] = old;
    arr[i] = cell;
    markEdited();
    var swapped = j >= 0 ? 'Swapped the ' + (layer === 'starts' ? 'starts' : 'goals') + ' of robots ' + (i + 1) + ' and ' + (j + 1) + '.' : '';
    editMsg((note ? note + (swapped ? ' ' : '') : '') + swapped, true);
    refreshEdit({ i: i, layer: layer });
  }
  function markEdited() {
    if (!/\(edited\)$/.test(S.edit.origin)) S.edit.origin += ' (edited)';
  }
  function toggleObstacle(c, keepFocus) {
    var E = S.edit;
    if (E.starts.indexOf(c) >= 0 || E.goals.indexOf(c) >= 0) { editMsg('A robot starts or ends on that cell; move it first.'); return; }
    if (E.blocked.has(c)) E.blocked.delete(c);
    else {
      if (E.rows * E.cols - E.blocked.size <= E.starts.length) { editMsg('Keep at least one free cell per robot.'); return; }
      E.blocked.add(c);
    }
    markEdited();
    refreshEdit();
    editMsg(S.map.lower_bounds.makespan === null ? 'Some goal is now unreachable: the graph is disconnected.' : '', false);
    if (keepFocus) {
      var cell = V.gEdit.querySelector('.edit-cell[data-cell="' + c + '"]');
      if (cell) cell.focus();
    }
  }
  function addRobot() {
    var E = S.edit, n = E.rows * E.cols, s = -1, t = -1, c;
    for (c = 0; c < n; c++) if (!E.blocked.has(c) && E.starts.indexOf(c) < 0) { s = c; break; }
    for (c = n - 1; c >= 0; c--) if (!E.blocked.has(c) && E.goals.indexOf(c) < 0) { t = c; break; }
    if (s < 0 || t < 0) { editMsg('Every free cell is taken.'); return; }
    E.starts.push(s); E.goals.push(t);
    markEdited();
    editMsg('Added robot ' + E.starts.length + '; drag its start and goal.', true);
    refreshEdit();
  }
  function removeRobot() {
    var E = S.edit;
    if (E.starts.length <= 1) { editMsg('Keep at least one robot.'); return; }
    E.starts.pop(); E.goals.pop();
    markEdited();
    editMsg('');
    refreshEdit();
  }
  function swapStartsGoals() {
    var E = S.edit, tmp = E.starts;
    E.starts = E.goals; E.goals = tmp;
    markEdited();
    editMsg('Starts and goals swapped.', true);
    refreshEdit();
  }
  function resizeEdit(rows, cols) {
    var E = S.edit, oc = E.cols;
    function remap(c) {
      var x = c % oc, y = Math.floor(c / oc);
      return x < cols && y < rows ? y * cols + x : -1;
    }
    var blocked = new Set(), starts = [], goals = [], dropped = [], renumbered = false;
    E.blocked.forEach(function (c) { var d = remap(c); if (d >= 0) blocked.add(d); });
    // an instance needs at least one free cell (for at least one robot): refuse a size that leaves only obstacles
    if (blocked.size >= rows * cols) {
      $('ed-rows').value = String(E.rows); $('ed-cols').value = String(E.cols);
      editMsg('In a ' + rows + ' × ' + cols + ' grid every cell would be an obstacle, so the grid stays ' +
              E.rows + ' × ' + E.cols + '. Clear some obstacles first.');
      return;
    }
    E.starts.forEach(function (c, i) {
      var a = remap(c), b = remap(E.goals[i]);
      if (a >= 0 && b >= 0) { starts.push(a); goals.push(b); if (dropped.length) renumbered = true; }
      else dropped.push(i + 1);
    });
    // enlarge: new cells are free
    E.rows = rows; E.cols = cols; E.blocked = blocked; E.starts = starts; E.goals = goals;
    var kept = starts.length;
    if (!starts.length) {
      for (var c = 0; c < rows * cols; c++) if (!blocked.has(c)) { starts.push(c); break; }
      for (c = rows * cols - 1; c >= 0; c--) if (!blocked.has(c)) { goals.push(c); break; }
    }
    markEdited();
    // the robots outside the new grid are gone for good (enlarging it again does not bring them back): say so
    var msg = '';
    if (dropped.length) {
      msg = 'Removed ' + (dropped.length === 1 ? 'robot ' : 'robots ') + numberList(dropped) + ': ' +
            (dropped.length === 1 ? 'its' : 'their') + ' start or goal is outside the ' + rows + ' × ' + cols + ' grid.';
      if (!kept) msg += ' Added one new robot, since an instance needs at least one.';
      else if (renumbered) msg += ' The other robots are renumbered ' + (kept === 1 ? '1' : '1–' + kept) + ', in their old order.';
    }
    editMsg(msg);
    refreshEdit();
  }
  /** [1, 3, 5, 6, 7] -> "1, 3 and 5–7". */
  function numberList(a) {
    var runs = [];
    a.forEach(function (x) {
      var r = runs[runs.length - 1];
      if (r && x === r[1] + 1) r[1] = x; else runs.push([x, x]);
    });
    var t = runs.map(function (r) { return r[0] === r[1] ? String(r[0]) : r[1] === r[0] + 1 ? r[0] + ', ' + r[1] : r[0] + '–' + r[1]; });
    return t.length === 1 ? t[0] : t.slice(0, -1).join(', ') + ' and ' + t[t.length - 1];
  }
  function randomizeEdit() {
    var rows = +$('ed-rows').value, cols = +$('ed-cols').value, frac = +$('ed-frac').value;
    var n = Math.round(+$('ed-n').value), seed = Math.round(+$('ed-seed').value);
    if (!(n >= 1)) { editMsg('Pick at least one robot.'); return; }
    // the seed actually used goes back into the field (a negative or empty one is 0)
    seed = seed >= 0 ? Math.min(seed, 2147483647) : 0;
    $('ed-seed').value = String(seed);
    $('ed-n').value = String(n);
    var free = rows * cols - Math.floor(rows * cols * frac);
    if (n > free) { editMsg('At most ' + free + ' robots fit on this grid with these obstacles.'); return; }
    try {
      var tool = S.edit.tool;
      S.edit = randomEdit(rows, cols, frac, n, seed);
      S.edit.tool = tool === 'play' ? 'starts' : tool;
    } catch (e) { editMsg('Could not generate: ' + e.message); return; }
    editMsg('Generated with seed ' + seed + ' (the same generator as the paper’s grid instances).', true);
    refreshEdit();
  }

  // ------------------------------------------------------------------ time-expanded network
  var NET = {};
  /** The largest horizon a drawing shows on this map: at most NET_MAX_NODES vertex copies, so that it stays
   *  legible (0: the map has too many vertices for that drawing). */
  function netMaxT(variant) {
    var n = S.g.n, perStep = variant === 'full' ? 2 : 1;
    if (n > NET_MAX_VERTICES[variant]) return 0;
    return Math.max(2, Math.min(NET_MAX_T[variant], Math.floor((NET_MAX_NODES[variant] / n - 1) / perStep)));
  }
  function netMarker(defs, id, cls) {
    var m = svg('marker', { id: id, viewBox: '0 0 10 10', refX: '9', refY: '5', markerWidth: '6', markerHeight: '6',
                            orient: 'auto-start-reverse', markerUnits: 'userSpaceOnUse' }, defs);
    m.setAttribute('markerWidth', '7'); m.setAttribute('markerHeight', '7');
    svg('path', { d: 'M0 0L10 5L0 10z', class: cls }, m);
  }
  function renderNet() {
    var svgEl = $('net-svg'), info = $('net-info'), map = S.map, variant = S.net.variant;
    clear(svgEl, true);
    NET = {};
    $('netv-compact').checked = variant === 'compact';
    $('netv-full').checked = variant === 'full';
    $('net-card').classList.toggle('v-compact', variant === 'compact');
    $('net-card').classList.toggle('v-full', variant === 'full');
    var n = S.g.n, A = map.starts.length, lbT = map.lower_bounds.makespan;
    var maxT = netMaxT(variant) || NET_MAX_T[variant];
    var slider = $('net-T');
    var hi = clamp(Math.max(S.paths ? S.T : 0, lbT || 1) + 3, 2, maxT);
    slider.min = '1'; slider.max = String(hi);
    S.net.T = clamp(S.net.T, 1, hi);
    slider.value = String(S.net.T);
    $('net-T-out').textContent = String(S.net.T);
    if (n > NET_MAX_VERTICES[variant]) {
      slider.disabled = true;
      info.textContent = 'This map has ' + n + ' vertices: its network (' + n + ' rows × ' +
        (variant === 'full' ? 2 * S.net.T + 1 : S.net.T + 1) + ' copies) is too large to draw legibly. Pick a map with at most ' + NET_MAX_VERTICES[variant] + ' vertices' +
        (variant === 'full' ? ' (or the compact drawing, up to ' + NET_MAX_VERTICES.compact + ')' : '') + ', or edit a small grid.';
      svgEl.setAttribute('width', '0'); svgEl.setAttribute('height', '0');
      $('net-scroll').hidden = true;
      updateNetHint();
      return;
    }
    $('net-scroll').hidden = false;
    slider.disabled = false;
    var T = S.net.T, net;
    try { net = MPP.network(S.g, map.starts, map.goals, T, { variant: variant }); }
    catch (e) { info.textContent = 'Could not build the network: ' + e.message; return; }
    // ---- layout
    var full = variant === 'full';
    var dy = n <= 9 ? 30 : n <= 16 ? 24 : 19;
    var maxCol = full ? 2 * T + 0.5 : T;
    var loopBand = 16 + 5 * Math.ceil(A / 2);
    // the row labels sit left of the loopback curves, which reach 22 + 3 k px left of column 0
    var labLen = 1;
    for (var v0 = 0; v0 < n; v0++) labLen = Math.max(labLen, String(map.vertex_names ? map.vertex_names[v0] : v0).length);
    var loopOut = 22 + 3 * Math.ceil(A / 2) + 6;
    var labelX = 8 + 7 * labLen;
    var padL = labelX + 6 + loopOut, padR = 34 + 3 * Math.ceil(A / 2), padT = 14 + loopBand, padB = 30 + loopBand;
    var avail = Math.max(200, ($('net-scroll').clientWidth || 600) - 2);
    // phones: squeeze the columns further, so that a short horizon fits the width (scrolling hides t = T)
    var narrowNet = avail < 480;
    var dx = clamp((avail - padL - padR) / maxCol, full ? (narrowNet ? 30 : 44) : (narrowNet ? 38 : 60), full ? 110 : 150);
    var W = padL + maxCol * dx + padR, H = padT + (n - 1) * dy + padB;
    svgEl.setAttribute('width', String(Math.round(W)));
    svgEl.setAttribute('height', String(Math.round(H)));
    svgEl.setAttribute('viewBox', '0 0 ' + Math.round(W) + ' ' + Math.round(H));
    function X(col) { return padL + col * dx; }
    function Y(row) { return padT + row * dy; }
    NET.X = function (t) { return X(full ? 2 * t : t); };
    NET.top = padT - 8; NET.bottom = padT + (n - 1) * dy + 8;
    var defs = svg('defs', null, svgEl);
    netMarker(defs, 'nm-green', 'nm-green');
    netMarker(defs, 'nm-blue', 'nm-blue');
    netMarker(defs, 'nm-gadget', 'nm-gadget');
    netMarker(defs, 'nm-loop', 'nm-loop');
    // gadget nodes sharing a midpoint get spread out
    var rowOf = {};
    if (full) {
      var groups = {};
      net.edges.forEach(function (e) { var k = String((e[0] + e[1]) / 2); (groups[k] = groups[k] || []).push(e); });
      Object.keys(groups).forEach(function (k) {
        var es = groups[k];
        es.forEach(function (e, j) { rowOf[e[0] + ',' + e[1]] = (e[0] + e[1]) / 2 + (j - (es.length - 1) / 2) * 0.34; });
      });
    }
    var pos = net.nodes.map(function (nd) {
      var row = nd.row;
      if (full && nd.edge) row = rowOf[Math.min(nd.edge[0], nd.edge[1]) + ',' + Math.max(nd.edge[0], nd.edge[1])];
      return [X(nd.col), Y(row)];
    });
    var key = {};
    net.nodes.forEach(function (nd, i) {
      if (!full) key['c' + nd.v + ',' + nd.t] = i;
      else if (nd.kind === 'copy' || nd.kind === "copy'") key['c' + nd.v + ',' + nd.t + ',' + (nd.kind === "copy'" ? 1 : 0)] = i;
      else key[(nd.kind === 'merge' ? 'm' : 's') + Math.min(nd.edge[0], nd.edge[1]) + ',' + Math.max(nd.edge[0], nd.edge[1]) + ',' + nd.t] = i;
    });
    // ---- axis labels
    var gl = svg('g', null, svgEl);
    for (var v = 0; v < n; v++) {
      svg('text', { class: 'n-label v', x: labelX, y: Y(v) }, gl).textContent = map.vertex_names ? map.vertex_names[v] : String(v);
    }
    var ty = padT + (n - 1) * dy + 18;
    for (var t = 0; t <= T; t++) {
      svg('text', { class: 'n-label t', x: X(full ? 2 * t : t), y: ty }, gl).textContent = String(t);
      if (full && t > 0) svg('text', { class: 'n-label t', x: X(2 * t + 0.5), y: ty }, gl).textContent = t + '′';
    }
    // ---- arcs
    var ga = svg('g', null, svgEl), loops = [];
    function shorten(a, b, d) {
      var vx = b[0] - a[0], vy = b[1] - a[1], L = Math.hypot(vx, vy) || 1;
      return [a[0] + vx / L * d, a[1] + vy / L * d, b[0] - vx / L * d, b[1] - vy / L * d];
    }
    net.arcs.forEach(function (arc) {
      if (arc.kind === 'loopback') { loops.push(arc); return; }
      var a = pos[arc.tail], b = pos[arc.head], s = shorten(a, b, 3.5);
      svg('line', { class: 'nt-' + arc.kind, x1: r3(s[0]), y1: r3(s[1]), x2: r3(s[2]), y2: r3(s[3]),
                    'marker-end': 'url(#nm-' + arc.kind + ')' }, ga);
    });
    function loopPath(arc, r) {
      var a = pos[arc.tail], b = pos[arc.head], up = r % 2 === 0, k = Math.floor(r / 2);
      var yE = up ? padT - 12 - 5 * k : padT + (n - 1) * dy + 30 + 5 * k;
      var ox = 22 + 3 * k;
      return 'M' + r3(a[0]) + ' ' + r3(a[1]) + ' C' + r3(a[0] + ox) + ' ' + r3(a[1]) + ' ' + r3(a[0] + ox) + ' ' + r3(yE) + ' ' + r3(a[0]) + ' ' + r3(yE) +
             ' L' + r3(b[0]) + ' ' + r3(yE) + ' C' + r3(b[0] - ox) + ' ' + r3(yE) + ' ' + r3(b[0] - ox) + ' ' + r3(b[1]) + ' ' + r3(b[0] - 4) + ' ' + r3(b[1]);
    }
    var loopD = [];
    loops.forEach(function (arc, r) {
      var d = loopPath(arc, r);
      loopD[r] = d;
      svg('path', { class: 'nt-loop', d: d, 'marker-end': 'url(#nm-loop)' }, ga);
    });
    // ---- nodes
    var gn = svg('g', null, svgEl);
    net.nodes.forEach(function (nd, i) {
      var p = pos[i], gadget = full && (nd.kind === 'merge' || nd.kind === 'split');
      svg('circle', { class: 'n-node' + (gadget ? ' g' : ''), cx: r3(p[0]), cy: r3(p[1]), r: gadget ? 2.4 : 2.8 }, gn);
    });
    // ---- flows
    var gf = svg('g', null, svgEl);
    NET.flows = [];
    var fits = S.paths && S.T <= T;
    if (S.net.overlay && fits) {
      S.paths.forEach(function (p0, r) {
        var p = p0.slice();
        while (p.length < T + 1) p.push(p[p.length - 1]);
        var ids = [];
        if (!full) for (t = 0; t <= T; t++) ids.push(key['c' + p[t] + ',' + t]);
        else {
          ids.push(key['c' + p[0] + ',0,0']);
          for (t = 0; t < T; t++) {
            var u = p[t], w = p[t + 1];
            if (u !== w) {
              var e = Math.min(u, w) + ',' + Math.max(u, w) + ',' + t;
              ids.push(key['m' + e], key['s' + e]);
            }
            ids.push(key['c' + w + ',' + (t + 1) + ',0'], key['c' + w + ',' + (t + 1) + ',1']);
          }
        }
        if (ids.some(function (x) { return x === undefined; })) return;
        var pl = svg('polyline', { class: 'n-flow', points: ids.map(function (id) { return r3(pos[id][0]) + ',' + r3(pos[id][1]); }).join(' ') }, gf);
        pl.style.stroke = flowColor(r);
        var lp = svg('path', { class: 'n-flow loop', d: loopD[r] }, gf);
        lp.style.stroke = flowColor(r);
        var tt = svg('title', null, pl); tt.textContent = 'Robot ' + (r + 1) + ' flow';
        [pl, lp].forEach(function (el) {
          el.addEventListener('pointerenter', function () { if (!S.pinned) setHighlight(r, false); });
          el.addEventListener('pointerleave', function () { if (!S.pinned) setHighlight(-1, false); });
          el.addEventListener('click', function () { if (S.pinned && S.hl === r) setHighlight(-1, false); else setHighlight(r, true); });
        });
        NET.flows.push({ r: r, els: [pl, lp] });
      });
    }
    NET.cursor = svg('line', { class: 'n-cursor', x1: 0, x2: 0, y1: NET.top, y2: NET.bottom }, svgEl);
    NET.cursorLine = svg('line', { class: 'n-cursor-line', x1: 0, x2: 0, y1: NET.top, y2: NET.bottom }, svgEl);
    highlightFlows();
    updateNetCursor(S.t);
    // ---- info line
    var bits = [];
    // one format for both drawings: the arcs of the network, then the loopback arcs (full: arcs_total counts them)
    if (full) bits.push(net.counts.vertices_total + ' nodes, ' + (net.counts.arcs_total - net.counts.by_kind.loop) + ' arcs + ' + net.counts.by_kind.loop + ' loopbacks');
    else bits.push(net.counts.vertices + ' nodes, ' + net.counts.arcs_total + ' arcs + ' + A + ' loopbacks');
    try {
      var m = MPP.buildModel('makespan', S.g, map.starts, map.goals, T);
      bits.push('makespan ILP after reachability pruning: ' + m.model.numColumns + ' columns × ' + m.model.numRows + ' rows');
    } catch (e) { /* the model is optional here */ }
    if (lbT !== null && T < lbT) bits.push('T is below the lower bound ' + lbT + ': no flow of value ' + A + ' exists');
    else if (S.paths && S.T > maxT) {
      // the slider stops at maxT: say that the plan is too long for this drawing, not "set T" to a value it cannot reach
      var cT = full ? netMaxT('compact') : 0;
      bits.push('the played plan has ' + plural(S.T, 'step') + ', more than this drawing shows (at most T = ' + maxT + '), ' +
                'so its flow is not drawn' + (cT >= S.T ? '; the compact drawing can show it' : ''));
    }
    else if (S.paths && S.T > T) bits.push('the played plan needs T ≥ ' + S.T + ' to be drawn (move the Horizon slider)');
    else if (S.paths && S.net.overlay) bits.push('drawn: the played plan (' + plural(S.T, 'step') + ')');
    info.textContent = bits.join(' · ') + '.';
    updateNetHint();
  }
  /** A drawing wider than its box: say so (phones hide the scrollbar until it is touched) and fade the cut edges. */
  function updateNetHint() {
    var sc = $('net-scroll'), wrap = $('net-wrap'), hint = $('net-hint');
    var over = !sc.hidden && sc.scrollWidth > sc.clientWidth + 1;
    var left = over && sc.scrollLeft > 1, right = over && sc.scrollLeft + sc.clientWidth < sc.scrollWidth - 1;
    wrap.classList.toggle('more-left', left);
    wrap.classList.toggle('more-right', right);
    hint.hidden = !over;
    hint.textContent = !over ? '' : right ? 'The drawing is wider than this box: scroll it sideways to see up to t = ' + S.net.T + ' →'
                                       : '← Scroll back to t = 0.';
  }
  function highlightFlows() {
    if (!NET.flows) return;
    NET.flows.forEach(function (f) {
      f.els.forEach(function (el) {
        el.classList.toggle('dim', S.hl >= 0 && f.r !== S.hl);
        el.classList.toggle('hl', f.r === S.hl);
      });
    });
  }
  function updateNetCursor(t) {
    if (!NET.cursor) return;
    var show = S.paths && t <= S.net.T + 1e-9 && !(S.map.custom && S.arrange);
    [NET.cursor, NET.cursorLine].forEach(function (el) { if (el) el.style.display = show ? '' : 'none'; });
    if (!show) return;
    var x = r3(NET.X(t));
    [NET.cursor, NET.cursorLine].forEach(function (el) { if (el) { el.setAttribute('x1', x); el.setAttribute('x2', x); } });
  }

  // ------------------------------------------------------------------ URL hash (map and plan, or the edited instance)
  var lastHash = null;
  function hashNow() {
    var plan = '&plan=' + encodeURIComponent(planKey());
    if (S.map.custom && S.edit) {
      var E = S.edit, blocked = [];
      E.blocked.forEach(function (c) { blocked.push(c); });
      blocked.sort(function (x, y) { return x - y; });
      return '#edit=' + E.rows + 'x' + E.cols + '&o=' + blocked.join('.') + '&s=' + E.starts.join('.') + '&g=' + E.goals.join('.') + plan;
    }
    return '#map=' + encodeURIComponent(S.map.id) + plan;
  }
  function saveHash() {
    if (!S.map) return;
    var h = hashNow();
    lastHash = h;
    if (window.location.hash === h) return;
    try { window.history.replaceState(null, '', h); } catch (e) { /* file:// in some browsers */ }
  }
  function readHash() {
    var out = {};
    (window.location.hash || '').replace(/^#/, '').split('&').forEach(function (kv) {
      var i = kv.indexOf('=');
      if (i > 0) {
        try { out[kv.slice(0, i).toLowerCase()] = decodeURIComponent(kv.slice(i + 1)); } catch (e) { /* ignore */ }
      }
    });
    return out;
  }
  function applyHashPlan(plan) {
    if (!plan) return;
    if (plan === 'paper') { S.source = 'paper'; return; }
    var m = /^([a-z_]+)\/k(\d+)$/.exec(plan);
    if (m && OBJ_BY_ID[m[1]] && KS.indexOf(+m[2]) >= 0) { S.objective = m[1]; S.k = +m[2]; S.source = 'computed'; }
  }
  /** The edited instance in a hash (#edit=RxC&o=..&s=..&g=..), or {error: why it is not a valid one}. */
  function editFromHash(h) {
    var m = /^(\d+)x(\d+)$/.exec(h.edit || '');
    if (!m) return { error: 'the grid size “' + (h.edit || '') + '” is not of the form ROWSxCOLUMNS' };
    var rows = +m[1], cols = +m[2];
    if (rows < 2 || cols < 2 || rows > EDIT_MAX_SIDE || cols > EDIT_MAX_SIDE) {
      return { error: 'the grid is ' + rows + ' × ' + cols + ', and edited grids have 2 to ' + EDIT_MAX_SIDE + ' rows and columns' };
    }
    var bad = null;
    function list(x, what) {
      if (!x) return [];
      var parts = x.split('.');
      for (var i = 0; i < parts.length; i++) {
        // plain decimal cell numbers only (Number() would also take "0x1", "-0" or "")
        if (!/^\d+$/.test(parts[i]) || +parts[i] >= rows * cols) {
          bad = bad || what + ' has the cell “' + parts[i] + '”, not a number from 0 to ' + (rows * cols - 1);
          return null;
        }
      }
      return parts.map(Number);
    }
    var o = list(h.o, 'the obstacle list'), st = list(h.s, 'the start list'), go = list(h.g, 'the goal list');
    if (bad) return { error: bad };
    if (!st.length) return { error: 'it has no robots' };
    if (st.length !== go.length) return { error: 'it has ' + plural(st.length, 'start') + ' but ' + plural(go.length, 'goal') };
    var blocked = new Set(o);
    function check(a, what) {
      if (new Set(a).size !== a.length) return 'two ' + what + ' share a cell';
      for (var i = 0; i < a.length; i++) if (blocked.has(a[i])) return 'a ' + what.replace(/s$/, '') + ' sits on an obstacle';
      return null;
    }
    var why = check(st, 'starts') || check(go, 'goals');
    if (why) return { error: why };
    return { rows: rows, cols: cols, blocked: blocked, starts: st, goals: go, origin: 'from the page link (#edit=…)', tool: 'starts' };
  }
  /** "the max distance plan (k = 1)" for a plan key of a link, or the key itself when it is not one. */
  function planPhrase(key) {
    if (key === 'paper') return planWords('paper');
    var m = /^([a-z_]+)\/k(\d+)$/.exec(key);
    return m && OBJ_BY_ID[m[1]] && KS.indexOf(+m[2]) >= 0 ? planWords(m[1], +m[2]) : 'the plan “' + key + '”';
  }
  /** After following a link: a note when the shown plan is not the one the link asks for. */
  function hashPlanNote(asked, where) {
    if (!asked || asked === planKey()) return '';
    return 'This link asks for ' + planPhrase(asked) + ', which ' + where + ' does not offer; showing ' +
           planPhrase(planKey()) + '.';
  }
  /** A notice above the demo when a link could not be followed (cleared by the next choice of a map). */
  function hashNote(text) {
    var n = $('hash-note');
    if (!n) return;
    n.hidden = !text;
    n.textContent = text || '';
  }
  /** Is the edited instance E the one on screen (the same grid, obstacles, starts and goals)? */
  function sameEdit(E) {
    var C = S.map && S.map.custom ? S.edit : null;
    if (!C || C.rows !== E.rows || C.cols !== E.cols || C.blocked.size !== E.blocked.size) return false;
    var same = true;
    E.blocked.forEach(function (c) { if (!C.blocked.has(c)) same = false; });
    function eq(a, b) { return a.length === b.length && a.every(function (x, i) { return x === b[i]; }); }
    return same && eq(C.starts, E.starts) && eq(C.goals, E.goals);
  }
  /** Follow the hash: on load, and when it is edited or another link of the page is opened in this tab. */
  function applyHash() {
    if (lastHash !== null && window.location.hash === lastHash) return;
    var h = readHash(), editWhy = null;
    if (h.edit) {
      var E = editFromHash(h);
      if (E.error) editWhy = E.error;
      else if (sameEdit(E)) {
        // only the plan differs from the instance on screen: keep its live results, a running solve and its origin
        hashNote('');
        applyHashPlan(h.plan);
        S.play = 'pre';
        planChanged();
        hashNote(hashPlanNote(h.plan, 'this edited instance'));
        return;
      } else {
        hashNote('');
        stopPlay();
        if (!S.map || !S.map.custom) S.lastDataMap = S.lastDataMap || (S.map && !S.map.custom ? S.map : DATA.maps[0]);
        S.edit = E;
        applyHashPlan(h.plan);
        editMsg('');
        syncEditInputs();
        resetRandomInputs();
        resetZoom();      // a zoom chosen (or auto-zoomed) for the earlier map does not fit the new grid
        refreshEdit();
        hashNote(hashPlanNote(h.plan, 'this edited instance'));
        return;
      }
    }
    var map = DATA.maps.filter(function (m) { return m.id === h.map; })[0];
    // the map that will be shown: the linked one, else the one on screen, else the first
    var shown = map || S.map || DATA.maps[0];
    var shownName = shown.custom ? 'the edited instance it had' : '“' + shown.title + '”';
    var note = editWhy !== null
      ? 'This link’s edited instance is not valid (' + editWhy + '), so the page shows ' + shownName + ' instead.' : '';
    // a link to a map the demo does not have (a typo, or a map renamed since): say so rather than silently show another
    if (!map && h.map !== undefined) {
      note += (note ? ' ' : '') + 'This link names the map “' + h.map + '”, which this demo does not have; ' +
              (S.map ? 'the page keeps ' + (S.map.custom ? 'the edited instance.' : '“' + S.map.title + '”.')
                     : 'showing “' + DATA.maps[0].title + '”.');
    }
    if (!map && S.map) {
      // no usable map: keep the page as it is. A link without map= and edit= (e.g. #plan=paper) picks a plan of the
      // map on screen; one that names an unknown map or an invalid instance changes nothing
      if (h.map === undefined && editWhy === null && h.plan) {
        applyHashPlan(h.plan);
        S.play = 'pre';
        planChanged();
        var where = S.map.custom ? 'this edited instance' : '“' + S.map.title + '”';
        note = hashPlanNote(h.plan, where);
      } else saveHash();
      hashNote(note);
      return;
    }
    if (S.edit) { staleCustomSolve('Opening this link left edit mode and stopped the live solve of your instance.'); S.edit = null; S.arrange = false; }
    applyHashPlan(h.plan);
    selectMap(map || DATA.maps[0], true);
    var pn = hashPlanNote(h.plan, '“' + S.map.title + '”');
    hashNote(note + (note && pn ? ' ' : '') + pn);
  }

  // ------------------------------------------------------------------ wiring
  function bind() {
    var sel = $('map-select');
    DATA.maps.forEach(function (m) { html('option', { value: m.id }, mapOptionText(m), sel); });
    // links of the page that open a map (#map=...): the hashchange switches the map; bring the view into sight
    Array.prototype.forEach.call(document.querySelectorAll('a[href^="#map="]'), function (a) {
      a.addEventListener('click', function () {
        window.setTimeout(function () {
          $('view-title').scrollIntoView({ block: 'start', behavior: reduceMotion.matches ? 'auto' : 'smooth' });
        }, 0);
      });
    });
    sel.addEventListener('change', function () {
      var m = DATA.maps.filter(function (x) { return x.id === sel.value; })[0];
      hashNote('');
      if (m) selectMap(m);
    });
    $('src-set').addEventListener('change', function (e) { S.source = e.target.value; S.play = 'pre'; planChanged(); });
    $('obj-set').addEventListener('change', function (e) { S.objective = e.target.value; S.play = 'pre'; planChanged(); });
    $('k-set').addEventListener('change', function (e) { S.k = +e.target.value; S.play = 'pre'; planChanged(); });
    $('play-set').addEventListener('change', function (e) { S.play = e.target.value; planChanged(); });
    $('pb-play').addEventListener('click', togglePlay);
    $('pb-reset').addEventListener('click', function () { seek(0); announceTime(); });
    $('pb-back').addEventListener('click', function () { stepBy(-1); announceTime(); });
    $('pb-fwd').addEventListener('click', function () { stepBy(1); announceTime(); });
    var range = $('pb-time');
    range.addEventListener('input', function () { seek(+range.value); });
    range.addEventListener('keydown', function (e) {
      if (e.key === 'ArrowRight' || e.key === 'ArrowUp') { e.preventDefault(); stepBy(1); }
      else if (e.key === 'ArrowLeft' || e.key === 'ArrowDown') { e.preventDefault(); stepBy(-1); }
      else if (e.key === 'Home') { e.preventDefault(); seek(0); }
      else if (e.key === 'End') { e.preventDefault(); seek(S.T); }
    });
    var speed = $('pb-speed');
    speed.value = String(S.speed);
    if (speed.value !== String(S.speed)) { S.speed = 1; speed.value = '1'; }
    speed.addEventListener('change', function () { S.speed = +speed.value; savePrefs(); });
    $('hl-select').addEventListener('change', function (e) { var i = +e.target.value; setHighlight(i, i >= 0); });
    $('view').addEventListener('keydown', function (e) {
      if (e.target !== $('view')) return;
      var vw = $('view'), PAN = 60;
      if (e.shiftKey && /^Arrow/.test(e.key) && vw.scrollWidth + vw.scrollHeight > vw.clientWidth + vw.clientHeight + 2) {
        // zoomed in: Shift + arrow keys pan the view (the plain arrow keys step the time)
        e.preventDefault();
        vw.scrollBy(e.key === 'ArrowLeft' ? -PAN : e.key === 'ArrowRight' ? PAN : 0,
                    e.key === 'ArrowUp' ? -PAN : e.key === 'ArrowDown' ? PAN : 0);
        return;
      }
      // arranging an edited instance: the playbar is off and the discs stand at the starts, so the play keys do nothing
      if (S.map.custom && S.arrange) { if (e.key === 'Escape') setHighlight(-1, false); return; }
      if (e.key === ' ' || e.key === 'k') { e.preventDefault(); togglePlay(); }
      else if (e.key === 'ArrowRight') { e.preventDefault(); stepBy(1); announceTime(); }
      else if (e.key === 'ArrowLeft') { e.preventDefault(); stepBy(-1); announceTime(); }
      else if (e.key === 'Home') { e.preventDefault(); seek(0); announceTime(); }
      else if (e.key === 'End') { e.preventDefault(); seek(S.T); announceTime(); }
      else if (e.key === 'Escape') setHighlight(-1, false);
    });
    function opt(id, k, after) {
      var c = $(id);
      c.checked = S.view[k];
      c.addEventListener('change', function () { S.view[k] = c.checked; savePrefs(); after(); });
    }
    opt('opt-goals', 'goals', function () {
      if (V.gGoals) V.gGoals.style.display = S.view.goals ? '' : 'none';
      buildInset();
      if (S.view.colors) applyRobotColors();
      setHighlight(S.hl, S.pinned);
    });
    var zoom = $('view-zoom');
    zoom.addEventListener('change', function () {
      S.zoom = +zoom.value || 1;
      applyZoom();
      updateViewSub();
      if (S.zoom > 1) {   // keep the middle in view
        var vw = $('view');
        vw.scrollLeft = (vw.scrollWidth - vw.clientWidth) / 2;
        vw.scrollTop = (vw.scrollHeight - vw.clientHeight) / 2;
      }
    });
    var rsz = 0;
    window.addEventListener('resize', function () {
      window.clearTimeout(rsz);
      rsz = window.setTimeout(function () { if (S.map) { applyZoom(); buildInset(); setHighlight(S.hl, S.pinned); renderNet(); updateTableHint(); } }, 150);
    });
    window.addEventListener('hashchange', applyHash);
    $('net-scroll').addEventListener('scroll', updateNetHint, { passive: true });
    $('metrics-scroll').addEventListener('scroll', updateTableHint, { passive: true });
    opt('opt-inter', 'inter', function () { S.interIdx = -2; frame(); });
    opt('opt-colors', 'colors', function () { if (S.map.custom && S.arrange) buildView(); else applyRobotColors(); });
    opt('opt-ids', 'ids', function () { buildView(); frame(); });
    opt('opt-transpose', 'transpose', function () { buildView(); frame(); });
    // live
    $('live-btn').addEventListener('click', startSolve);
    $('live-cancel').addEventListener('click', function () { cancelSolve(); });
    $('live-tl').addEventListener('change', function (e) { S.timeLimit = +e.target.value; savePrefs(); });
    // edit
    var rs = $('ed-rows'), cs = $('ed-cols');
    for (var i = 2; i <= EDIT_MAX_SIDE; i++) { html('option', { value: String(i) }, String(i), rs); html('option', { value: String(i) }, String(i), cs); }
    $('edit-btn').addEventListener('click', function () { hashNote(''); enterEdit(); });
    $('edit-done').addEventListener('click', exitEdit);
    $('edit-tool').addEventListener('change', function (e) {
      var v = e.target.value;
      if (v === 'play') { S.arrange = false; }
      else { S.edit.tool = v; S.arrange = true; }
      stopPlay();
      buildView();
      frame();
      renderNet();
    });
    rs.addEventListener('change', function () { resizeEdit(+rs.value, S.edit.cols); });
    cs.addEventListener('change', function () { resizeEdit(S.edit.rows, +cs.value); });
    $('ed-random').addEventListener('click', randomizeEdit);
    ['ed-n', 'ed-seed'].forEach(function (id) {   // Enter in a Randomize setting runs Randomize
      $(id).addEventListener('keydown', function (e) { if (e.key === 'Enter') { e.preventDefault(); randomizeEdit(); } });
    });
    $('ed-add').addEventListener('click', addRobot);
    $('ed-remove').addEventListener('click', removeRobot);
    $('ed-swap').addEventListener('click', swapStartsGoals);
    // network. The statistics line is not a live region: renderNet also runs on map, plan, theme and edit changes,
    // and a screen reader would read it after each of them (every arrow-key move of an edited robot, say). It is
    // said once after a change of the drawing or the horizon, when the user is at these controls.
    var netSayTimer = 0;
    function netSay() {
      window.clearTimeout(netSayTimer);
      netSayTimer = window.setTimeout(function () { $('net-announce').textContent = $('net-info').textContent; }, 400);
    }
    $('net-variant').addEventListener('change', function (e) { S.net.variant = e.target.value; savePrefs(); renderNet(); netSay(); });
    var nT = $('net-T');
    nT.addEventListener('input', function () { S.net.T = +nT.value; renderNet(); netSay(); });
    var ov = $('net-overlay');
    ov.checked = S.net.overlay;
    ov.addEventListener('change', function () { S.net.overlay = ov.checked; renderNet(); });
    onMq(reduceMotion, function () { frame(); });
    document.addEventListener('visibilitychange', function () { if (document.hidden) stopPlay(); });
  }

  function fail(msg) {
    var box = $('load-error');
    if (box) { box.hidden = false; box.textContent = msg; }
  }

  function init() {
    if (!MPP || !MPP.lp || !MPP.network) { fail('The engine (js/engine.js, js/lp.js) did not load.'); return; }
    if (!DATA || !DATA.maps || !DATA.maps.length) { fail('The demo data (data/demo-data.js) did not load.'); return; }
    loadPrefs();
    initTheme();
    bind();
    applyHash();
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
  else init();
})();
