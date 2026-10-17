#!/usr/bin/env node
/*
 * tools/check_demo.mjs -- end-to-end check of the static browser demo (docs/index.html).
 *
 * Zero dependencies: needs Node >= 22 (global WebSocket) and a Chrome/Chromium binary.
 * Launches Chrome headless with --remote-debugging-port, drives the page over the Chrome
 * DevTools Protocol and opens docs/index.html from file:// (no server, no extra flags), then
 *
 *   - records console errors, uncaught exceptions (page and Web Workers), log errors, failed
 *     loads and every request whose URL is not file:, data:, blob: or about: (the demo must not
 *     touch the network);
 *   - validates every precomputed plan in window.MPP_DEMO with the page's own engine
 *     (MPP.validate, MPP.metrics);
 *   - for every map x plan (objective/k and the paper's plan) selects it with real mouse clicks
 *     on the controls (the map <select> gets its value set plus a change event), plays it to the
 *     end with the play button and checks every robot sits on its goal (robot transforms vs goal
 *     transforms, the "home" class, the t = T / T label and the metrics table);
 *   - runs one live solve (Web Worker) on each live map, checks the progress text names the Web
 *     Worker, waits for it and plays the result; checks that a solve still running after a map
 *     switch is named by its map on the other map's card, and by its objective on the card of
 *     another objective of the same map (with the keyboard focus moved to Cancel and back, and
 *     Solve held until the cancelled worker is gone, and the cancelled solve not reported as a
 *     result to look at); checks the memory guard (an edited copy of the crowded 8 x 8 map is not
 *     offered at k = 1, and a split piece over the budget stops the solve with a message), also on
 *     the main thread with window.Worker hidden (a k-way split then runs piece by piece: the Fig. 8
 *     split, a Cancel click queued during a piece that stops the split before the next piece, and
 *     the memory guard);
 *   - toggles the theme (and checks it persists across a reload), the network panel (drawing
 *     variant, flow overlay, horizon slider), transpose, and edit mode (drag a start, randomize,
 *     obstacles, add a robot, live solve, play, back);
 *   - follows the URL hash: editing it switches the map, an #edit= link opens (and a reload
 *     restores) an edited instance, and an invalid #edit= link, an unknown map and a plan the
 *     map does not have are reported; shrinking an edited grid reports the robots it removes, and a
 *     size that leaves only obstacles is refused;
 *   - live result texts: an infeasible instance is reported once, and the page then says that no
 *     objective or split has a plan (no Solve button); a plan whose build-time Python check stopped
 *     at its iteration budget does not quote that run as the same solve; a plan (or a split's piece)
 *     taken from the joint-state search is listed as such in the facts; a link that changes only the
 *     plan keeps the edited instance's live results;
 *   - preferences: a stored time limit that the menu does not offer is ignored; fmtSec near 10 s;
 *   - links: an invalid edit= next to a valid map= names the map shown, a #plan= link without map=
 *     picks the plan of the map on screen and clears the old notice; edit mode hides the highlight
 *     legend entry, starts at Zoom: fit (from the auto-zoomed 24 x 18 map on a phone) and draws
 *     its badges' numbers at least 8.5 px tall at 360 px;
 *   - screen reader and keyboard: the time label and the network statistics are not live regions (the time
 *     is said after a step and at the end of playback, the statistics after a change of the network's
 *     controls), sliders have names, an arrow key moves an edited disc over an obstacle, the view's
 *     name gives the edit keys in edit mode, the play keys do nothing while an edit tool is on after
 *     a live solve, and the obstacle grid is one tab stop with arrow-key movement; with reduced motion, one step back after a pause between steps moves one step;
 *   - checks the view and the play controls fit on one screen (1280 x 900, 390 x 844, 360 x 740),
 *     that every split plan in the data records its pieces, and the goal inset's note;
 *   - checks there is no horizontal page scroll at 360 px (and 390 px) for every map, and that the
 *     obstacles, edges, vertices, the highlighted robot's path and rings and the Obstacles tool's
 *     free-cell outlines have at least 3:1 contrast against the view in both themes (and the network
 *     panel's plan-flow lines, move arcs and time cursor against the panel);
 *   - writes screenshots at 1280 x 900 and 390 x 844 in light and dark.
 *
 * Prints a JSON report on stdout and exits 1 on any failure (2 if Chrome cannot be started).
 *
 * Usage (from anywhere; the default screenshot directory is never inside this repository):
 *   node tools/check_demo.mjs [--shots DIR] [--chrome PATH] [--fast] [--headed]
 *        [--only MAP_ID[,MAP_ID...]] [--no-live] [--timeout-live SECONDS]
 *   --shots DIR   screenshot directory (default: ./build/demo-shots under the current directory, or
 *                 <os temp dir>/mppflow-demo-shots when the current directory is inside the repository)
 *   --fast        jump to the end with the End key instead of playing at 8 steps/s
 *   --docs DIR    check another copy of docs/ (default: the docs/ next to this tools/ directory)
 *   env CHROME    alternative to --chrome
 */
import { spawn } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath, pathToFileURL } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const REPO = path.resolve(HERE, '..');
let DOCS = path.join(REPO, 'docs');

// ------------------------------------------------------------------ arguments
/** The default screenshot directory: ./build/demo-shots, unless that would put the PNGs inside the repository
 *  (which a careless `git add .` would then publish); then a directory under the OS temp directory. */
function defaultShots() {
  const cwd = path.resolve(process.cwd()), rel = path.relative(REPO, cwd);
  const inside = rel === '' || (!rel.startsWith('..') && !path.isAbsolute(rel));
  return inside ? path.join(os.tmpdir(), 'mppflow-demo-shots') : path.join(cwd, 'build', 'demo-shots');
}

function parseArgs(argv) {
  const o = { shots: defaultShots(), chrome: process.env.CHROME || null, fast: false,
              headed: false, only: null, live: true, liveTimeout: 120 };
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    if (a === '--shots') o.shots = path.resolve(argv[++i]);
    else if (a === '--chrome') o.chrome = argv[++i];
    else if (a === '--fast') o.fast = true;
    else if (a === '--headed') o.headed = true;
    else if (a === '--only') o.only = argv[++i].split(',');
    else if (a === '--no-live') o.live = false;
    else if (a === '--timeout-live') o.liveTimeout = +argv[++i];
    else if (a === '--docs') DOCS = path.resolve(argv[++i]);
    else if (a === '-h' || a === '--help') {
      process.stdout.write(fs.readFileSync(fileURLToPath(import.meta.url), 'utf8').split('*/')[0] + '*/\n');
      process.exit(0);
    } else { process.stderr.write('unknown argument ' + a + '\n'); process.exit(2); }
  }
  return o;
}
const OPT = parseArgs(process.argv.slice(2));
const INDEX = path.join(DOCS, 'index.html');

// ------------------------------------------------------------------ report
const T0 = Date.now();
const report = {
  ok: false, page: pathToFileURL(INDEX).href, chrome: null, seconds: 0,
  summary: {}, failures: [], warnings: [],
  console: [], exceptions: [], bad_requests: [], failed_loads: [],
  data_checks: [], plans: [], live: [], theme: {}, network_panel: {}, edit: {}, scroll: [], screenshots: [],
};
let currentStep = 'setup';
function fail(msg, extra) {
  report.failures.push(Object.assign({ step: currentStep, message: msg }, extra || {}));
  process.stderr.write('FAIL [' + currentStep + '] ' + msg + '\n');
}
function warn(msg) { report.warnings.push({ step: currentStep, message: msg }); }
function log(msg) { process.stderr.write(msg + '\n'); }
async function step(name, fn) {
  const prev = currentStep;
  currentStep = name;
  try { await fn(); } catch (e) { fail('exception: ' + (e && e.stack ? e.stack.split('\n').slice(0, 3).join(' | ') : e)); }
  currentStep = prev;
}
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const J = JSON.stringify;

// ------------------------------------------------------------------ demo data (read in node, the same file the page loads)
function loadDemoData() {
  const ctx = { window: {} };
  vm.createContext(ctx);
  vm.runInContext(fs.readFileSync(path.join(DOCS, 'data', 'demo-data.js'), 'utf8'), ctx, { filename: 'demo-data.js' });
  return ctx.window.MPP_DEMO;
}
function padPaths(paths) {
  let L = 1;
  for (const p of paths) L = Math.max(L, p.length);
  return paths.map((p) => { const q = p.slice(); while (q.length < L) q.push(q[q.length - 1]); return q; });
}
const OBJ_METRIC = { makespan: 'makespan', total_time: 'totalTime', total_distance: 'totalDistance', max_distance: 'maxDistance' };
const OBJ_ROW = { makespan: 'Makespan', total_time: 'Total arrival time', total_distance: 'Total distance', max_distance: 'Max distance' };

// ------------------------------------------------------------------ chrome + CDP
function findChrome() {
  const cands = [OPT.chrome, 'google-chrome', 'google-chrome-stable', 'chromium', 'chromium-browser',
                 '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'].filter(Boolean);
  for (const c of cands) {
    if (c.includes('/')) { if (fs.existsSync(c)) return c; continue; }
    for (const dir of (process.env.PATH || '').split(path.delimiter)) {
      const f = path.join(dir, c);
      if (fs.existsSync(f)) return f;
    }
  }
  return null;
}

function launchChrome(bin, userDir) {
  const args = [
    OPT.headed ? null : '--headless=new', '--remote-debugging-port=0', '--user-data-dir=' + userDir,
    '--no-first-run', '--no-default-browser-check', '--disable-extensions', '--disable-background-networking',
    '--disable-component-update', '--disable-sync', '--metrics-recording-only', '--mute-audio',
    '--disable-background-timer-throttling', '--disable-renderer-backgrounding', '--disable-backgrounding-occluded-windows',
    '--hide-scrollbars', '--window-size=1280,900', 'about:blank',
  ].filter(Boolean);
  if (process.getuid && process.getuid() === 0) args.unshift('--no-sandbox');
  const proc = spawn(bin, args, { stdio: ['ignore', 'ignore', 'pipe'] });
  return new Promise((resolve, reject) => {
    let buf = '';
    const timer = setTimeout(() => reject(new Error('Chrome did not print a DevTools URL within 30 s')), 30000);
    proc.stderr.on('data', (d) => {
      buf += d.toString();
      const m = /DevTools listening on (ws:\/\/\S+)/.exec(buf);
      if (m) { clearTimeout(timer); resolve({ proc, ws: m[1] }); }
    });
    proc.on('exit', (code) => { clearTimeout(timer); reject(new Error('Chrome exited with code ' + code + ': ' + buf.slice(-500))); });
  });
}

// Close Chrome, then remove its profile directory. Removing the profile right after a SIGKILL
// of the browser is not enough: its child processes keep writing the profile (the disk-cache
// index) for a moment and the directory comes back. So the browser is asked to close (CDP
// Browser.close, then SIGTERM, then SIGKILL), its exit is awaited, and the removal is retried
// until the directory stays gone. Returns whether it is gone.
async function closeChrome(chrome, client, userDir) {
  const proc = chrome && chrome.proc;
  const exited = () => !proc || proc.exitCode !== null || proc.signalCode !== null;
  const waitExit = (ms) => (exited() ? Promise.resolve(true) : new Promise((res) => {
    const t = setTimeout(() => res(exited()), ms);
    proc.once('exit', () => { clearTimeout(t); res(true); });
  }));
  if (!exited()) {
    try { if (client) await client.send('Browser.close', {}, undefined, 3000); } catch (e) { /* closing or gone */ }
    if (!(await waitExit(5000))) {
      try { proc.kill('SIGTERM'); } catch (e) { /* gone */ }
      if (!(await waitExit(3000))) { try { proc.kill('SIGKILL'); } catch (e) { /* gone */ } await waitExit(2000); }
    }
  }
  try { if (client) client.ws.close(); } catch (e) { /* closed */ }
  for (let i = 0; i < 10; i++) {
    try { fs.rmSync(userDir, { recursive: true, force: true }); } catch (e) { /* retried */ }
    await sleep(200);
    if (!fs.existsSync(userDir)) return true;
  }
  return false;
}

class CDP {
  constructor(url) {
    this.nextId = 1; this.pending = new Map(); this.listeners = [];
    this.ws = new WebSocket(url);
    this.ws.onmessage = (ev) => {
      const m = JSON.parse(typeof ev.data === 'string' ? ev.data : Buffer.from(ev.data).toString());
      if (m.id !== undefined) {
        const p = this.pending.get(m.id);
        if (!p) return;
        this.pending.delete(m.id);
        clearTimeout(p.timer);
        if (m.error) p.reject(new Error(p.method + ': ' + m.error.message)); else p.resolve(m.result);
      } else for (const fn of this.listeners) { try { fn(m); } catch (e) { log('listener error ' + e); } }
    };
    this.opened = new Promise((res, rej) => { this.ws.onopen = res; this.ws.onerror = () => rej(new Error('websocket error')); });
    this.ws.onclose = () => { for (const p of this.pending.values()) p.reject(new Error('websocket closed')); this.pending.clear(); };
  }
  send(method, params = {}, sessionId, timeoutMs = 60000) {
    const id = this.nextId++;
    const msg = { id, method, params };
    if (sessionId) msg.sessionId = sessionId;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => { this.pending.delete(id); reject(new Error(method + ' timed out')); }, timeoutMs);
      this.pending.set(id, { resolve, reject, method, timer });
      this.ws.send(J(msg));
    });
  }
  on(fn) { this.listeners.push(fn); }
}

// ------------------------------------------------------------------ page helpers
let cdp, SID;
const send = (m, p, t) => cdp.send(m, p, SID, t);

async function ev(expr, timeoutMs = 60000) {
  const r = await send('Runtime.evaluate', { expression: expr, awaitPromise: true, returnByValue: true, userGesture: true },
                       timeoutMs);
  if (r.exceptionDetails) {
    const d = r.exceptionDetails;
    throw new Error('evaluate failed: ' + (d.exception && d.exception.description ? d.exception.description.split('\n')[0] : d.text));
  }
  return r.result.value;
}
async function waitFor(expr, timeoutMs, what, interval = 100) {
  const t0 = Date.now();
  let last;
  while (Date.now() - t0 < timeoutMs) {
    last = await ev(expr);
    if (last) return last;
    await sleep(interval);
  }
  throw new Error('timed out after ' + timeoutMs + ' ms waiting for ' + (what || expr));
}
async function setViewport(width, height, mobile) {
  await send('Emulation.setDeviceMetricsOverride', { width, height, deviceScaleFactor: 1, mobile: !!mobile });
  await sleep(150);
}
async function emulateScheme(scheme) {
  await send('Emulation.setEmulatedMedia', { features: [{ name: 'prefers-color-scheme', value: scheme },
                                                         { name: 'prefers-reduced-motion', value: 'no-preference' }] });
}

/** A real mouse click at the centre of the element (scrolled into view). Reports covered targets. */
async function click(sel, opts = {}) {
  const r = await ev(`(() => {
    const e = document.querySelector(${J(sel)});
    if (!e) return { err: 'no element ' + ${J(sel)} };
    e.scrollIntoView({ block: 'center', inline: 'center' });
    const b = e.getBoundingClientRect();
    if (!b.width || !b.height) return { err: 'element ' + ${J(sel)} + ' is not visible (0 size)' };
    const x = b.left + b.width / 2, y = b.top + b.height / 2;
    const h = document.elementFromPoint(x, y);
    const desc = (n) => n ? n.nodeName.toLowerCase() + (n.id ? '#' + n.id : '') + (n.className && n.className.baseVal === undefined && n.className ? '.' + String(n.className).split(' ').join('.') : '') : 'nothing';
    return { x, y, hit: !!h && (h === e || e.contains(h) || (e.htmlFor && h.id === e.htmlFor)), what: desc(h) };
  })()`);
  if (r.err) throw new Error(r.err);
  if (!r.hit && !opts.allowCovered) fail('click on ' + sel + ' lands on ' + r.what + ' (element covered or off-screen)');
  for (const type of ['mouseMoved', 'mousePressed', 'mouseReleased']) {
    await send('Input.dispatchMouseEvent', { type, x: r.x, y: r.y, button: type === 'mouseMoved' ? 'none' : 'left',
                                             buttons: type === 'mousePressed' ? 1 : 0, clickCount: type === 'mouseMoved' ? 0 : 1 });
  }
  await sleep(opts.wait === undefined ? 40 : opts.wait);
}
async function drag(from, to, steps = 8) {
  await send('Input.dispatchMouseEvent', { type: 'mouseMoved', x: from.x, y: from.y, button: 'none', buttons: 0 });
  await send('Input.dispatchMouseEvent', { type: 'mousePressed', x: from.x, y: from.y, button: 'left', buttons: 1, clickCount: 1 });
  for (let i = 1; i <= steps; i++) {
    const x = from.x + (to.x - from.x) * i / steps, y = from.y + (to.y - from.y) * i / steps;
    await send('Input.dispatchMouseEvent', { type: 'mouseMoved', x, y, button: 'left', buttons: 1 });
    await sleep(15);
  }
  await send('Input.dispatchMouseEvent', { type: 'mouseReleased', x: to.x, y: to.y, button: 'left', buttons: 0, clickCount: 1 });
  await sleep(80);
}
async function pressKey(key, code, keyCode) {
  await send('Input.dispatchKeyEvent', { type: 'keyDown', key, code, windowsVirtualKeyCode: keyCode });
  await send('Input.dispatchKeyEvent', { type: 'keyUp', key, code, windowsVirtualKeyCode: keyCode });
  await sleep(40);
}
/** A <select> (or range input): set the value as the user would and fire the event the page listens to. */
async function setControl(id, value, eventName = 'change') {
  const r = await ev(`(() => {
    const e = document.getElementById(${J(id)});
    if (!e) return 'no element #' + ${J(id)};
    if (e.disabled) return '#' + ${J(id)} + ' is disabled';
    if (e.tagName === 'SELECT' && ![...e.options].some((o) => o.value === ${J(String(value))})) return '#' + ${J(id)} + ' has no option ' + ${J(String(value))};
    e.scrollIntoView({ block: 'center' });
    e.focus();
    e.value = ${J(String(value))};
    e.dispatchEvent(new Event(${J(eventName)}, { bubbles: true }));
    return null;
  })()`);
  if (r) throw new Error(r);
  await sleep(40);
}
async function screenshot(name, fullPage = true) {
  fs.mkdirSync(OPT.shots, { recursive: true });
  const params = { format: 'png', captureBeyondViewport: fullPage };
  if (fullPage) {
    const m = await send('Page.getLayoutMetrics');
    const cs = m.cssContentSize || m.contentSize;
    params.clip = { x: 0, y: 0, width: Math.ceil(cs.width), height: Math.min(Math.ceil(cs.height), 16000), scale: 1 };
  }
  const r = await send('Page.captureScreenshot', params, 60000);
  const f = path.join(OPT.shots, name + '.png');
  fs.writeFileSync(f, Buffer.from(r.data, 'base64'));
  report.screenshots.push(f);
}
async function navigate(url) {
  const loaded = new Promise((res) => { pageLoadWaiters.push(res); });
  await send('Page.navigate', { url });
  await Promise.race([loaded, sleep(20000)]);
  await waitFor(`!!(window.MPP && window.MPP_DEMO && document.readyState === 'complete' &&
                    (document.querySelector('#view-svg .robot') || !document.getElementById('load-error').hidden))`, 20000, 'demo init');
  await sleep(100);
}
let pageLoadWaiters = [];

// ------------------------------------------------------------------ demo-specific state readers
const STATE_JS = `(() => {
  const parse = (e) => { const m = /translate\\(\\s*([-\\d.e]+)[ ,]+([-\\d.e]+)\\s*\\)/.exec(e.getAttribute('transform') || ''); return m ? [+m[1], +m[2]] : null; };
  const robots = [...document.querySelectorAll('#view-svg .robot:not(.edit-disc)')];
  const goals = [...document.querySelectorAll('#view-svg .goal:not(.edit-disc)')];
  const play = document.getElementById('pb-play');
  const sel = document.querySelector('#metrics tbody tr.sel');
  const head = [...document.querySelectorAll('#metrics thead th')].map((x) => x.textContent.replace(/\u00ad/g, ''));
  return {
    title: document.getElementById('view-title').textContent,
    sub: document.getElementById('view-sub').textContent,
    label: document.getElementById('pb-label').textContent,
    playing: play.classList.contains('playing'),
    playDisabled: play.disabled,
    robots: robots.map((e) => ({ p: parse(e), home: e.classList.contains('home'), shown: e.style.display !== 'none' })),
    goals: goals.map(parse),
    status: document.getElementById('plan-status').textContent,
    metricsHead: head,
    selRow: sel ? [...sel.children].map((x) => x.textContent) : null,
    loadError: document.getElementById('load-error').hidden ? null : document.getElementById('load-error').textContent,
    viewBox: document.getElementById('view-svg').getAttribute('viewBox'),
    hash: location.hash,
  };
})()`;
const state = () => ev(STATE_JS);
/** (selector, 'fill' | 'stroke', id of the background element) => the lowest WCAG contrast of the matching
 *  elements' colours (composited at their opacity) against that element's background, or null if none match. */
const CONTRAST_JS = `((sel, prop, bgId) => { const rgb = (x) => x.match(/[\\d.]+/g).slice(0, 3).map(Number);
  const lum = (c) => { const l = c.map((v) => { v /= 255; return v <= 0.03928 ? v / 12.92 : Math.pow((v + 0.055) / 1.055, 2.4); }); return 0.2126 * l[0] + 0.7152 * l[1] + 0.0722 * l[2]; };
  const bg = rgb(getComputedStyle(document.getElementById(bgId)).backgroundColor), lb = lum(bg);
  let low = null;
  for (const e of document.querySelectorAll(sel)) {
    const cs = getComputedStyle(e), a = (+cs.opacity) * (cs[prop + 'Opacity'] === undefined ? 1 : +cs[prop + 'Opacity']);
    const c = rgb(cs[prop]).map((v, i) => a * v + (1 - a) * bg[i]), l = lum(c);
    const r = Math.round(((Math.max(l, lb) + 0.05) / (Math.min(l, lb) + 0.05)) * 100) / 100;
    if (low === null || r < low) low = r;
  }
  return low; })`;
const close = (a, b) => a && b && Math.abs(a[0] - b[0]) < 2e-3 && Math.abs(a[1] - b[1]) < 2e-3;

/** Robots at the vertices `verts` of the map (no transpose)? */
function atVertices(st, map, verts) {
  const bad = [];
  verts.forEach((v, i) => {
    const want = map.graph.xy[v], got = st.robots[i] && st.robots[i].p;
    if (!close(got, want)) bad.push('robot ' + (i + 1) + ' at ' + J(got) + ' not at vertex ' + v + ' ' + J(want));
  });
  return bad;
}

async function selectMap(map) {
  const cur = await ev(`document.getElementById('map-select').value`);
  if (cur !== map.id || !(await ev(`!document.getElementById('map-pick').hidden`))) await setControl('map-select', map.id);
  await waitFor(`document.getElementById('view-title').textContent === ${J(map.title)}`, 5000, 'map ' + map.id);
}
async function radioChecked(id) { return ev(`document.getElementById(${J(id)}).checked`); }
async function chooseRadio(id) {
  const st = await ev(`(() => { const e = document.getElementById(${J(id)}); return e ? { disabled: e.disabled, checked: e.checked, hidden: !!e.closest('[hidden]') } : null; })()`);
  if (!st) throw new Error('no radio #' + id);
  if (st.checked) return true;
  if (st.hidden) { fail('radio #' + id + ' is in a hidden group'); return false; }
  if (st.disabled) { fail('radio #' + id + ' is disabled although a plan exists for it'); return false; }
  await click(`label[for="${id}"]`);
  if (!(await radioChecked(id))) { fail('clicking the label of #' + id + ' did not check it'); return false; }
  return true;
}
async function selectPlan(key) {
  if (key === 'paper') return chooseRadio('src-paper');
  if (!(await ev(`document.getElementById('src-set').hidden`))) { if (!(await chooseRadio('src-computed'))) return false; }
  const [obj, kk] = key.split('/k');
  if (!(await chooseRadio('obj-' + obj))) return false;
  return chooseRadio('k-' + kk);
}

/** Play the shown plan from t = 0 to its end and check starts and goals. */
async function playToEnd(map, paths, tag) {
  const out = { ok: true, problems: [] };
  const P = padPaths(paths), T = P[0].length - 1;
  out.T = T;
  await click('#pb-reset');
  let st = await state();
  if (st.robots.length !== map.starts.length) out.problems.push(st.robots.length + ' robots drawn, expected ' + map.starts.length);
  if (st.robots.some((r) => !r.shown)) out.problems.push('some robot is hidden');
  if (st.label !== 't = 0 / ' + T) out.problems.push('label "' + st.label + '" at reset, expected "t = 0 / ' + T + '"');
  out.problems.push(...atVertices(st, map, P.map((p) => p[0])));
  if (T > 0) {
    const t0 = Date.now();
    if (OPT.fast) {
      await ev(`document.getElementById('view').focus()`);
      await pressKey('End', 'End', 35);
    } else {
      await click('#pb-play');
      // wait until it is playing, then until it stops at the end
      const startedPlaying = await waitFor(`document.getElementById('pb-play').classList.contains('playing') || /^t = ${T} \\//.test(document.getElementById('pb-label').textContent)`,
                                           3000, 'playback start').catch(() => false);
      if (!startedPlaying) out.problems.push('pressing play did not start playback');
      const budget = Math.ceil(T / 8 * 1000 * 2 + 5000);
      await waitFor(`!document.getElementById('pb-play').classList.contains('playing')`, budget, 'playback end ' + tag).catch((e) => out.problems.push(e.message));
    }
    out.play_seconds = (Date.now() - t0) / 1000;
  }
  st = await state();
  if (st.label !== 't = ' + T + ' / ' + T) out.problems.push('label "' + st.label + '" at the end, expected "t = ' + T + ' / ' + T + '"');
  out.problems.push(...atVertices(st, map, map.goals));
  const notHome = st.robots.map((r, i) => (r.home ? 0 : i + 1)).filter(Boolean);
  if (notHome.length) out.problems.push('robots without the "home" class at the end: ' + notHome.join(', '));
  st.robots.forEach((r, i) => { if (!close(r.p, st.goals[i])) out.problems.push('robot ' + (i + 1) + ' not on its goal disc'); });
  if (st.loadError) out.problems.push('load error shown: ' + st.loadError);
  out.ok = out.problems.length === 0;
  return out;
}

// ------------------------------------------------------------------ main
async function main() {
  const DATA = loadDemoData();
  if (!DATA || !Array.isArray(DATA.maps)) { fail('docs/data/demo-data.js does not define MPP_DEMO.maps'); return; }
  const MAPS = DATA.maps.filter((m) => !OPT.only || OPT.only.includes(m.id));
  const bin = findChrome();
  if (!bin) { report.failures.push({ step: 'setup', message: 'no Chrome binary found (use --chrome or CHROME)' }); finish(2); }
  const userDir = fs.mkdtempSync(path.join(os.tmpdir(), 'mppflow-check-demo-'));
  let chrome;
  try { chrome = await launchChrome(bin, userDir); } catch (e) {
    report.failures.push({ step: 'setup', message: String(e.message || e) });
    try { fs.rmSync(userDir, { recursive: true, force: true }); } catch (x) { /* ignore */ }
    finish(2);
  }
  let closed = false;
  const cleanup = () => {  // last resort on an early exit (closeChrome below is the normal way)
    if (closed) return;
    try { chrome.proc.kill('SIGKILL'); } catch (e) { /* gone */ }
    try { fs.rmSync(userDir, { recursive: true, force: true }); } catch (e) { /* ignore */ }
  };
  process.on('exit', cleanup);
  try {
    cdp = new CDP(chrome.ws);
    await cdp.opened;
    const ver = await cdp.send('Browser.getVersion');
    report.chrome = ver.product;
    const { targetId } = await cdp.send('Target.createTarget', { url: 'about:blank' });
    SID = (await cdp.send('Target.attachToTarget', { targetId, flatten: true })).sessionId;
    const workerSessions = new Set();
    const pageUrl = report.page;

    // ---- event collection: page and workers
    cdp.on((m) => {
      const fromWorker = m.sessionId && workerSessions.has(m.sessionId);
      if (m.sessionId !== SID && !fromWorker) return;
      const where = fromWorker ? 'worker' : 'page';
      const p = m.params || {};
      switch (m.method) {
        case 'Page.loadEventFired': { const w = pageLoadWaiters; pageLoadWaiters = []; w.forEach((f) => f()); break; }
        case 'Runtime.consoleAPICalled': {
          const text = (p.args || []).map((a) => (a.value !== undefined ? String(a.value) : a.description || a.type)).join(' ');
          const entry = { where, type: p.type, text: text.slice(0, 500), step: currentStep };
          if (p.type === 'error' || p.type === 'assert') { report.console.push(entry); fail('console.' + p.type + ' (' + where + '): ' + entry.text); }
          else if (p.type === 'warning') { report.console.push(entry); warn('console.warn (' + where + '): ' + entry.text); }
          break;
        }
        case 'Runtime.exceptionThrown': {
          const d = p.exceptionDetails || {};
          const text = (d.exception && d.exception.description) || d.text || 'exception';
          report.exceptions.push({ where, text: text.slice(0, 800), url: d.url, line: d.lineNumber, step: currentStep });
          fail('uncaught exception (' + where + '): ' + text.split('\n')[0]);
          break;
        }
        case 'Log.entryAdded': {
          const e = p.entry || {};
          if (e.level === 'error') { report.console.push({ where, type: 'log-' + e.source, text: e.text, url: e.url, step: currentStep }); fail('browser log error (' + e.source + '): ' + e.text + (e.url ? ' ' + e.url : '')); }
          else if (e.level === 'warning') warn('browser log warning (' + e.source + '): ' + e.text);
          break;
        }
        case 'Network.requestWillBeSent': {
          const url = (p.request && p.request.url) || '';
          if (!/^(file:|data:|blob:|about:)/i.test(url)) {
            report.bad_requests.push({ where, url, type: p.type, step: currentStep });
            fail('network request to ' + url + ' (only file:, data: and blob: are allowed)');
          }
          break;
        }
        case 'Network.loadingFailed': {
          if (p.canceled) break;
          report.failed_loads.push({ where, requestId: p.requestId, error: p.errorText, type: p.type, blocked: p.blockedReason, step: currentStep });
          fail('a resource failed to load (' + p.type + '): ' + p.errorText);
          break;
        }
        case 'Page.javascriptDialogOpening':
          fail('a JavaScript dialog opened: ' + p.message);
          send('Page.handleJavaScriptDialog', { accept: false }).catch(() => {});
          break;
        case 'Target.attachedToTarget': {
          const s = p.sessionId;
          workerSessions.add(s);
          report.workers_attached = (report.workers_attached || 0) + 1;
          (async () => {
            try { await cdp.send('Runtime.enable', {}, s); } catch (e) { /* not all targets */ }
            try { await cdp.send('Network.enable', {}, s); } catch (e) { /* not all targets */ }
            try { await cdp.send('Log.enable', {}, s); } catch (e) { /* not all targets */ }
            try { await cdp.send('Runtime.runIfWaitingForDebugger', {}, s); } catch (e) { /* ignore */ }
          })();
          break;
        }
        default: break;
      }
    });
    for (const d of ['Page', 'Runtime', 'Log', 'Network']) await send(d + '.enable');
    await send('Target.setAutoAttach', { autoAttach: true, waitForDebuggerOnStart: true, flatten: true });
    await setViewport(1280, 900, false);
    await emulateScheme('light');

    // ------------------------------------------------------------ load
    await step('load', async () => {
      await navigate(pageUrl);
      const st = await state();
      if (st.loadError) fail('the page shows a load error: ' + st.loadError);
      const opts = await ev(`[...document.getElementById('map-select').options].map((o) => o.value)`);
      const want = DATA.maps.map((m) => m.id);
      if (J(opts) !== J(want)) fail('map select lists ' + J(opts) + ', expected ' + J(want));
      const env = await ev(`({ dataVersion: window.MPP_DEMO.version, hasWorker: typeof Worker === 'function', theme: document.documentElement.getAttribute('data-theme'),
                               scripts: [...document.scripts].map((s) => s.src ? s.src.replace(/^.*\\/docs\\//, '') : '(inline)'),
                               modules: [...document.scripts].filter((s) => s.type === 'module').length })`);
      report.environment = env;
      if (env.modules) fail('the page uses ES module scripts');
    });

    // ------------------------------------------------------------ data validated with the page's own engine
    await step('data', async () => {
      const res = await ev(`(() => {
        const out = [];
        const pad = (ps) => { let L = 1; ps.forEach((p) => { L = Math.max(L, p.length); }); return ps.map((p) => { const q = p.slice(); while (q.length < L) q.push(q[q.length - 1]); return q; }); };
        for (const m of window.MPP_DEMO.maps) {
          const g = MPP.fromAdjacency(m.graph.adj, m.graph.xy);
          for (const [key, p] of Object.entries(m.plans || {})) {
            if (!p.paths) { out.push({ map: m.id, key, ok: false, error: 'no paths' }); continue; }
            const P = pad(p.paths);
            const err = MPP.pathSetError(P, g, m.starts, m.goals);
            const met = MPP.metrics(P, m.goals);
            const metOk = !p.metrics || ['makespan', 'totalTime', 'totalDistance', 'maxDistance'].every((k) => p.metrics[k] === met[k]);
            out.push({ map: m.id, key, ok: err === null && metOk, error: err, metrics: met, stored: p.metrics || null });
          }
        }
        return out;
      })()`);
      for (const r of res) {
        report.data_checks.push({ map: r.map, key: r.key, ok: r.ok });
        if (!r.ok) fail('plan ' + r.map + ' ' + r.key + ': ' + (r.error || 'metrics ' + J(r.metrics) + ' != stored ' + J(r.stored)));
      }
      // every split plan (any objective) records where its pieces meet: the page draws the piece ticks from them
      for (const m of DATA.maps) {
        for (const [key, p] of Object.entries(m.plans || {})) {
          if (!(p.k > 1)) continue;
          if (!p.intermediate_goals || p.intermediate_goals.length !== p.k - 1 || !p.pieces || p.pieces.length !== p.k) {
            fail('split plan ' + m.id + ' ' + key + ' lacks its intermediate goals or pieces');
          }
        }
      }
    });

    // ------------------------------------------------------------ theme: toggle, persistence, system preference
    await step('theme', async () => {
      const bg = `(() => { const c = getComputedStyle(document.body).backgroundColor; const m = c.match(/[\\d.]+/g).map(Number);
                         const L = (0.2126 * m[0] + 0.7152 * m[1] + 0.0722 * m[2]) / 255; return { c, L, attr: document.documentElement.getAttribute('data-theme'),
                         pressed: document.getElementById('theme-toggle').getAttribute('aria-pressed'), aria: document.getElementById('theme-toggle').getAttribute('aria-label'), label: document.querySelector('#theme-toggle .theme-label').textContent,
                         stored: (() => { try { return localStorage.getItem('mppflow-demo-theme'); } catch (e) { return 'blocked'; } })() }; })()`;
      const th = report.theme;
      th.initial = await ev(bg);
      if (th.initial.attr) fail('fresh profile already has data-theme=' + th.initial.attr);
      if (!(th.initial.L > 0.5)) fail('light system theme but the body background is dark (' + th.initial.c + ')');
      await click('#theme-toggle');
      th.afterToggle = await ev(bg);
      if (th.afterToggle.attr !== 'dark' || !(th.afterToggle.L < 0.5) || th.afterToggle.aria !== 'Use light theme' || th.afterToggle.pressed !== null) fail('theme toggle did not switch to dark (or its button is not an action button labelled "Use light theme" without aria-pressed): ' + J(th.afterToggle));
      if (th.afterToggle.stored !== 'dark') fail('the theme was not stored in localStorage (' + th.afterToggle.stored + ')');
      await navigate(pageUrl);
      th.afterReload = await ev(bg);
      if (th.afterReload.attr !== 'dark' || !(th.afterReload.L < 0.5)) fail('the dark theme did not persist across a reload: ' + J(th.afterReload));
      await click('#theme-toggle');
      th.toggledBack = await ev(bg);
      if (th.toggledBack.attr !== 'light' || !(th.toggledBack.L > 0.5) || th.toggledBack.label !== 'Dark') fail('toggling back did not give the light theme: ' + J(th.toggledBack));
      // system preference: dark scheme, nothing stored
      await ev(`(() => { try { localStorage.removeItem('mppflow-demo-theme'); } catch (e) {} })()`);
      await emulateScheme('dark');
      await navigate(pageUrl);
      th.systemDark = await ev(bg);
      if (th.systemDark.attr || !(th.systemDark.L < 0.5) || th.systemDark.aria !== 'Use light theme') fail('prefers-color-scheme: dark is not followed: ' + J(th.systemDark));
      await emulateScheme('light');
      await sleep(100);
      th.systemLight = await ev(bg);
      if (!(th.systemLight.L > 0.5) || th.systemLight.aria !== 'Use dark theme') fail('switching the system scheme back to light is not followed: ' + J(th.systemLight));
      // blocked storage must not break the page (the theme button still works)
      th.ok = !report.failures.some((f) => f.step === 'theme');
    });

    // speed 8 steps/s for the playback checks
    await step('speed', async () => { await setControl('pb-speed', '8'); });

    // ------------------------------------------------------------ a stored time limit outside the menu; seconds near 10
    await step('preferences and seconds', async () => {
      const P = report.prefs = {};
      const src = fs.readFileSync(path.join(DOCS, 'js', 'demo.js'), 'utf8');
      const m = /function fmtSec\(s\) \{[\s\S]*?\n  \}/.exec(src);
      if (!m) fail('fmtSec not found in demo.js');
      else {
        const fmtSec = vm.runInNewContext('(' + m[0] + ')');
        P.fmtSec = { 9.994: fmtSec(9.994), 9.996: fmtSec(9.996), 10: fmtSec(10), 0.004: fmtSec(0.004) };
        if (P.fmtSec[9.994] !== '9.99 s' || P.fmtSec[9.996] !== '10.0 s' || P.fmtSec[10] !== '10.0 s') fail('fmtSec near 10 s: ' + J(P.fmtSec));
      }
      const old = await ev(`localStorage.getItem('mppflow-demo-prefs')`);
      await ev(`localStorage.setItem('mppflow-demo-prefs', JSON.stringify(Object.assign(JSON.parse(localStorage.getItem('mppflow-demo-prefs') || '{}'), { timeLimit: 5 })))`);
      await send('Page.navigate', { url: 'about:blank' });
      await sleep(200);
      await navigate(pageUrl);
      P.timeLimit = await ev(`({ value: document.getElementById('live-tl').value, index: document.getElementById('live-tl').selectedIndex })`);
      if (P.timeLimit.value !== '30' || P.timeLimit.index < 0) fail('a stored time limit of 5 s (not in the menu) leaves the menu at ' + J(P.timeLimit));
      await ev(old === null ? `localStorage.removeItem('mppflow-demo-prefs')` : `localStorage.setItem('mppflow-demo-prefs', ${J(old)})`);
      await send('Page.navigate', { url: 'about:blank' });
      await sleep(200);
      await navigate(pageUrl);
      await setControl('pb-speed', '8');
    });

    // ------------------------------------------------------------ every map x plan
    for (const map of MAPS) {
      const keys = Object.keys(map.plans || {}).filter((k) => map.plans[k] && map.plans[k].paths);
      keys.sort((a, b) => (a === 'paper') - (b === 'paper') || a.localeCompare(b, 'en', { numeric: true }));
      for (const key of keys) {
        await step('plan ' + map.id + ' ' + key, async () => {
          const plan = map.plans[key];
          const rec = { map: map.id, key, ok: false };
          report.plans.push(rec);
          await selectMap(map);
          if (!(await selectPlan(key))) { rec.problems = ['could not select the plan']; return; }
          let st = await state();
          rec.sub = st.sub;
          const problems = [];
          if (key === 'paper') {
            if (!/paper/.test(st.sub)) problems.push('view subtitle "' + st.sub + '" does not say it plays the paper plan');
          } else {
            const [obj, kk] = key.split('/k');
            if (!/precomputed/.test(st.sub)) problems.push('view subtitle "' + st.sub + '" does not say precomputed');
            if (+kk > 1 && st.sub.indexOf(kk + '-way split') < 0) problems.push('view subtitle "' + st.sub + '" does not mention the ' + kk + '-way split');
            if (st.hash !== '#map=' + encodeURIComponent(map.id) + '&plan=' + encodeURIComponent(key)) problems.push('URL hash ' + st.hash + ' does not match the selection');
            const col = st.metricsHead.indexOf('Precomp.');
            if (!st.selRow || st.selRow[0] !== OBJ_ROW[obj]) problems.push('metrics table: selected row ' + J(st.selRow && st.selRow[0]) + ', expected ' + OBJ_ROW[obj]);
            else if (col < 0) problems.push('metrics table has no Precomp. column');
            else if (plan.metrics && st.selRow[col] !== String(plan.metrics[OBJ_METRIC[obj]])) problems.push('metrics table shows ' + st.selRow[col] + ' for ' + obj + ', the plan has ' + plan.metrics[OBJ_METRIC[obj]]);
          }
          const r = await playToEnd(map, plan.paths, map.id + ' ' + key);
          rec.T = r.T; rec.play_seconds = r.play_seconds;
          problems.push(...r.problems);
          rec.problems = problems;
          rec.ok = problems.length === 0;
          if (!rec.ok) fail(map.id + ' ' + key + ': ' + problems.join('; '));
        });
      }
    }

    // ------------------------------------------------------------ network panel
    await step('network panel', async () => {
      const NP = report.network_panel;
      const netState = `(() => ({ nodes: document.querySelectorAll('#net-svg .n-node').length, gadgets: document.querySelectorAll('#net-svg .n-node.g').length,
        flows: document.querySelectorAll('#net-svg .n-flow').length, loops: document.querySelectorAll('#net-svg .nt-loop').length,
        info: document.getElementById('net-info').textContent, hidden: document.getElementById('net-scroll').hidden,
        T: +document.getElementById('net-T').value, fullLegendShown: getComputedStyle(document.querySelector('.net-legend .full-only')).display !== 'none' }))()`;
      const small = MAPS.find((m) => m.id === 'tor16-fig1-9puzzle') || MAPS.find((m) => m.graph.n <= 16);
      if (small) {
        await selectMap(small);
        await selectPlan('makespan/k1');
        if (!(await radioChecked('netv-compact'))) await chooseRadio('netv-compact');
        const A = small.starts.length;
        NP.compact = await ev(netState);
        if (NP.compact.hidden || NP.compact.nodes === 0) fail('compact network not drawn for ' + small.id);
        if (NP.compact.loops !== A) fail('compact network: ' + NP.compact.loops + ' loopback arcs, expected ' + A);
        if (NP.compact.flows !== 2 * A) fail('compact network: ' + NP.compact.flows + ' flow overlays, expected ' + 2 * A);
        await chooseRadio('netv-full');
        NP.full = await ev(netState);
        if (NP.full.hidden || NP.full.gadgets === 0 || NP.full.nodes <= NP.compact.nodes) fail('full network not drawn with gadgets: ' + J(NP.full));
        if (!NP.full.fullLegendShown) fail('the vertex-capacity legend entry is not shown in the full drawing');
        if (NP.full.flows !== 2 * A) fail('full network: ' + NP.full.flows + ' flow overlays, expected ' + 2 * A);
        await screenshot('desktop-network-full', true);
        await click('#net-overlay');
        NP.noOverlay = await ev(netState);
        if (NP.noOverlay.flows !== 0) fail('turning off the plan flow left ' + NP.noOverlay.flows + ' flow overlays');
        await click('#net-overlay');
        const lb = small.lower_bounds.makespan;
        if (lb > 1) {
          await setControl('net-T', String(lb - 1), 'input');
          NP.belowBound = await ev(netState);
          if (NP.belowBound.T !== lb - 1 || !/below the lower bound/.test(NP.belowBound.info)) fail('horizon below the bound not reported: ' + NP.belowBound.info);
        }
        await setControl('net-T', String(lb + 1), 'input');
        NP.aboveBound = await ev(netState);
        if (NP.aboveBound.T !== lb + 1 || NP.aboveBound.flows !== 2 * A) fail('horizon T = ' + (lb + 1) + ' not drawn with the plan flows: ' + J(NP.aboveBound));
        // a plan longer than the full drawing can show (T <= 8): say so, and point to the compact drawing
        const long = Object.keys(small.plans).find((k) => small.plans[k].metrics && small.plans[k].metrics.makespan > 8 && small.plans[k].metrics.makespan <= 16);
        if (long) {
          await selectPlan(long);
          NP.tooLong = await ev(`({ info: document.getElementById('net-info').textContent, max: +document.getElementById('net-T').max })`);
          if (NP.tooLong.max >= small.plans[long].metrics.makespan || !/more than this drawing shows/.test(NP.tooLong.info) ||
              !/compact drawing can show it/.test(NP.tooLong.info) || /needs T ≥/.test(NP.tooLong.info)) {
            fail('full drawing, plan ' + long + ' longer than the slider allows: ' + J(NP.tooLong));
          }
          await selectPlan('makespan/k1');
        }
        await chooseRadio('netv-compact');
      }
      const big = MAPS.find((m) => m.graph.n > 30);
      if (big) {
        await selectMap(big);
        NP.large = await ev(netState);
        if (!NP.large.hidden || !/too large/.test(NP.large.info)) fail('the network of ' + big.id + ' (' + big.graph.n + ' vertices) is not replaced by a note');
        const m = / × (\d+) copies/.exec(NP.large.info), variant = (await radioChecked('netv-full')) ? 'full' : 'compact';
        const want = variant === 'full' ? 2 * NP.large.T + 1 : NP.large.T + 1;
        if (!m || +m[1] !== want) fail('the too-large note quotes ' + (m ? m[1] : 'no') + ' copies per vertex for the ' + variant + ' drawing at T = ' + NP.large.T + ', expected ' + want);
      }
    });

    // ------------------------------------------------------------ transpose on a non-square grid
    await step('transpose', async () => {
      const rect = MAPS.find((m) => m.graph.rows && m.graph.cols && m.graph.rows !== m.graph.cols);
      if (!rect) return;
      await selectMap(rect);
      const shown = await ev(`!document.getElementById('opt-transpose').parentNode.hidden`);
      if (!shown) { fail('transpose option hidden on the ' + rect.graph.rows + ' x ' + rect.graph.cols + ' grid'); return; }
      const vb0 = (await state()).viewBox.split(' ').map(Number);
      await click('#opt-transpose');
      const vb1 = (await state()).viewBox.split(' ').map(Number);
      if (Math.abs(vb0[2] - vb1[3]) > 1e-6 || Math.abs(vb0[3] - vb1[2]) > 1e-6) fail('transpose did not swap the view box: ' + J(vb0) + ' -> ' + J(vb1));
      const firstKey = Object.keys(rect.plans)[0];
      if (rect.plans[firstKey] && rect.plans[firstKey].paths) {
        await selectPlan(firstKey);
        await click('#pb-reset');
        await ev(`document.getElementById('view').focus()`);
        await pressKey('End', 'End', 35);
        const st = await state();
        const bad = rect.goals.filter((v, i) => !close(st.robots[i].p, [rect.graph.xy[v][1], rect.graph.xy[v][0]]));
        if (bad.length) fail('transposed view: ' + bad.length + ' robots not on their transposed goals');
      }
      await click('#opt-transpose');
    });

    // ------------------------------------------------------------ live solves (Web Worker)
    async function liveSolve(map, key, label) {
      const rec = { map: map.id, key, ok: false };
      report.live.push(rec);
      if (!(await selectPlan(key))) { rec.problems = ['could not select ' + key]; return rec; }
      const btn = await ev(`({ disabled: document.getElementById('live-btn').disabled, text: document.getElementById('live-btn').textContent, info: document.getElementById('live-info').textContent })`);
      rec.button = btn.text;
      if (btn.disabled) { fail(label + ': the live button is disabled (' + btn.info + ')'); rec.problems = ['button disabled']; return rec; }
      const t0 = Date.now();
      await click('#live-btn');
      const started = await ev(`({ shown: !document.getElementById('live-progress').hidden || document.getElementById('live-result').textContent.length > 0,
                                   running: !document.getElementById('live-progress').hidden, text: document.getElementById('live-result').textContent })`);
      if (!started.shown) fail(label + ': no progress shown after clicking the live button');
      if (started.running) {
        rec.progress = started.text.slice(0, 200);
        if (!/in a Web Worker/.test(started.text)) fail(label + ': the progress text does not say the solve runs in a Web Worker: ' + rec.progress);
      }
      await waitFor(`document.getElementById('live-progress').hidden && !document.getElementById('live-btn').disabled`, OPT.liveTimeout * 1000, 'live solve ' + label, 200);
      rec.wall_seconds = (Date.now() - t0) / 1000;
      const res = await ev(`({ result: document.getElementById('live-result').textContent, facts: document.getElementById('facts').textContent,
                               playSet: !document.getElementById('play-set').hidden, sub: document.getElementById('view-sub').textContent })`);
      rec.result = res.result.slice(0, 400);
      rec.mode = /Web Worker/.test(res.facts) ? 'worker' : /main thread/.test(res.facts) ? 'main thread' : '?';
      const problems = [];
      if (!/Valid plan/.test(res.result)) problems.push('no valid live plan: ' + res.result.slice(0, 200));
      if (rec.mode !== 'worker') problems.push('the live solve did not run in a Web Worker (' + rec.mode + ')');
      if (!/live result/.test(res.sub)) problems.push('the view does not play the live result after the solve (' + res.sub + ')');
      rec.problems = problems;
      return rec;
    }
    if (OPT.live) {
      for (const map of MAPS.filter((m) => m.live)) {
        await step('live ' + map.id, async () => {
          await selectMap(map);
          const key = (map.live_plans && map.live_plans[0]) || 'makespan/k1';
          const rec = await liveSolve(map, key, map.id + ' ' + key);
          if (rec.problems && rec.problems.length === 0) {
            // play the live result: its paths are the page's, the goal check is the same
            const T = await ev(`+document.getElementById('pb-label').textContent.split('/')[1]`);
            const r = await playToEndLive(map, T);
            rec.T = T;
            rec.problems.push(...r);
            if (map.plans[key] && map.plans[key].metrics) {
              const mk = key.split('/k')[0];
              const { selRow: sel, metricsHead: head } = await state();
              const li = head.indexOf('Live');
              if (li < 0) rec.problems.push('no Live column in the metrics table');
              else if (sel && sel[li] !== String(map.plans[key].metrics[OBJ_METRIC[mk]]) && /optimal/.test(map.plans[key].status || '')) {
                rec.problems.push('live ' + mk + ' ' + sel[li] + ' differs from the optimal precomputed ' + map.plans[key].metrics[OBJ_METRIC[mk]]);
              }
            }
            if (await ev(`!document.getElementById('play-set').hidden`)) {
              await chooseRadio('play-pre');
              const sub = await ev(`document.getElementById('view-sub').textContent`);
              if (!/precomputed/.test(sub)) rec.problems.push('choosing "Precomputed" does not switch back (' + sub + ')');
              await chooseRadio('play-live');
            }
          }
          rec.ok = rec.problems.length === 0;
          if (!rec.ok) fail('live ' + map.id + ': ' + rec.problems.join('; '));
        });
      }
      // the Fig. 8 example split live: k = 2 must give makespan 5
      const fig8 = MAPS.find((m) => m.id === 'tor16-fig8-split');
      if (fig8 && fig8.live) {
        await step('live split ' + fig8.id, async () => {
          await selectMap(fig8);
          const rec = await liveSolve(fig8, 'makespan/k2', fig8.id + ' makespan/k2');
          if (rec.problems && !rec.problems.length) {
            const T = await ev(`+document.getElementById('pb-label').textContent.split('/')[1]`);
            rec.T = T;
            if (T !== 5) rec.problems.push('live 2-way split makespan ' + T + ', expected 5 (TOR Fig. 8)');
            rec.problems.push(...(await playToEndLive(fig8, T)));
          }
          rec.ok = !!rec.problems && rec.problems.length === 0;
          if (!rec.ok) fail('live split ' + fig8.id + ': ' + (rec.problems || []).join('; '));
        });
      }
      // a live solve keeps running when the map changes: the other map's card must name the map being solved
      const p9 = MAPS.find((m) => m.id === 'tor16-fig1-9puzzle'), tree = MAPS.find((m) => m.id === 'tor16-fig4a-tree');
      if (p9 && p9.live && tree) {
        await step('live solve seen from another map', async () => {
          await selectMap(p9);
          await selectPlan('makespan/k1');
          await click('#live-btn');
          await selectMap(tree);
          const st = await ev(`({ res: document.getElementById('live-result').textContent, running: !document.getElementById('live-cancel').hidden,
                                 btn: document.getElementById('live-btn').disabled, title: document.getElementById('live-btn').title })`);
          report.live_other_map = st;
          if (!st.running) warn('the 9-puzzle solve ended before the map switch; the other-map text was not checked');
          else if (st.res.indexOf('“' + p9.title + '”') < 0 || st.title.indexOf(p9.title) < 0) {
            fail('during a live solve of ' + p9.id + ', the card of ' + tree.id + ' does not name the map being solved: ' + J(st));
          }
          await waitFor(`document.getElementById('live-cancel').hidden`, 120000, 'the ' + p9.id + ' live solve').catch((e) => fail(e.message));
          if (await ev(`document.getElementById('live-btn').disabled`)) fail('the live button of ' + tree.id + ' stays disabled after the other solve ended');
        });
      }
      // another objective of the same map while a solve runs: the card names the solve's own choice; the keyboard
      // focus moves to Cancel and back; a cancel makes Solve wait for the stopped worker to go (no stacked workers)
      if (p9 && p9.live && (p9.live_plans || []).includes('max_distance/k1')) {
        await step('live solve of another plan on the same map', async () => {
          const R = report.live_same_map = {};
          await selectMap(p9);
          await selectPlan('max_distance/k1');
          await ev(`document.getElementById('live-btn').focus()`);
          for (const type of ['keyDown', 'keyUp']) await send('Input.dispatchKeyEvent', { type, key: 'Enter', code: 'Enter', windowsVirtualKeyCode: 13, text: type === 'keyDown' ? '\r' : undefined });
          await sleep(60);
          R.focusSolving = await ev(`document.activeElement ? document.activeElement.id || document.activeElement.nodeName : null`);
          if (R.focusSolving !== 'live-cancel') fail('Enter on the Solve button leaves the focus on ' + R.focusSolving + ', not on Cancel');
          await selectPlan('makespan/k1');
          R.during = await ev(`({ res: document.getElementById('live-result').textContent, cancel: document.getElementById('live-cancel').textContent,
                                  running: !document.getElementById('live-cancel').hidden, info: document.getElementById('live-info').textContent })`);
          if (!R.during.running) { warn('the max distance solve ended before the switch; the same-map text was not checked'); return; }
          if (!/^Solving the max distance plan \(k = 1\) of this map/.test(R.during.res) || R.during.cancel !== 'Cancel that solve' ||
              !/wait for the solve of the max distance plan/.test(R.during.info)) {
            fail('during a max distance solve, the makespan card does not name the solve: ' + J(R.during));
          }
          await click('#live-cancel');
          await sleep(50);
          R.after = await ev(`({ res: document.getElementById('live-result').textContent, btn: document.getElementById('live-btn').disabled })`);
          // the user stopped that solve: the card does not report it as finished, nor send them to look at it
          if (/finished: cancelled|to see it/.test(R.after.res)) {
            fail('after the cancel, the makespan card points to the cancelled max distance solve: ' + J(R.after));
          }
          await waitFor(`!document.getElementById('live-btn').disabled`, 7000, 'Solve after the cancel cooldown').catch((e) => fail(e.message));
          await sleep(60);
          R.focusAfter = await ev(`document.activeElement ? document.activeElement.id || document.activeElement.nodeName : null`);
          if (R.focusAfter !== 'live-btn') fail('after the cancelled solve the focus is on ' + R.focusAfter + ', not on the Solve button');
          R.workers = (await cdp.send('Target.getTargets')).targetInfos.filter((t) => t.type === 'worker').length;
          if (R.workers > 1) fail(R.workers + ' solver workers alive after a cancel');
        });
      }
      // memory: an edited copy of the crowded 8 x 8 map (14346 x 6008 at T = 10) would crash the tab, so it is refused;
      // its k-way splits are offered, and a piece over the budget stops the solve with a message
      const crowdedMap = MAPS.find((m) => m.id === 'crowded-8x8-40-seed1');
      if (crowdedMap) {
        await step('live memory guard', async () => {
          const R = report.live_memory = {};
          const cell = (v) => crowdedMap.graph.xy[v][1] * crowdedMap.graph.cols + crowdedMap.graph.xy[v][0];
          await ev(`location.hash = ${J('#edit=8x8&o=&s=' + crowdedMap.starts.map(cell).join('.') + '&g=' + crowdedMap.goals.map(cell).join('.') + '&plan=makespan%2Fk1')}`);
          await waitFor(`document.getElementById('view-title').textContent === 'Your instance'`, 5000, 'edited crowded instance');
          R.k1 = await ev(`({ btn: document.getElementById('live-btn').disabled, info: document.getElementById('live-info').textContent })`);
          if (!R.k1.btn || !/too large to solve in the browser/.test(R.k1.info)) fail('the 40-robot 8 x 8 copy is offered live at k = 1: ' + J(R.k1));
          await chooseRadio('k-2');
          R.k2 = await ev(`({ btn: document.getElementById('live-btn').disabled, info: document.getElementById('live-info').textContent })`);
          if (R.k2.btn || /much smaller|about T\//.test(R.k2.info)) fail('the 2-way split of the crowded copy: ' + J(R.k2));
          // leaving edit mode stops a running solve of the edited instance: the card of the map the page returns to
          // says so while Solve waits for the stopped worker, and the note goes when Solve is back
          await click('#live-btn');
          await sleep(800);
          R.staleRunning = await ev(`!document.getElementById('live-cancel').hidden`);
          await click('#edit-done');
          R.stale = await ev(`({ res: document.getElementById('live-result').textContent, btn: document.getElementById('live-btn').disabled })`);
          if (!R.staleRunning) warn('the split of the crowded copy ended within 0.8 s; the note after leaving edit mode was not checked');
          else if (!/Leaving edit mode stopped the live solve of your instance\. Solve is available again in a moment/.test(R.stale.res) || !R.stale.btn) {
            fail('leaving edit mode during a live solve: the card does not say the solve was stopped: ' + J(R.stale));
          }
          await waitFor(`!/Solve is available again/.test(document.getElementById('live-result').textContent)`, 7000, 'the end of the cooldown note').catch((e) => fail(e.message));
          R.staleAfterBtn = await ev(`document.getElementById('live-btn').disabled`);
          // 64 robots on 8 x 8: the first piece's model (40301 columns) is over the budget, checked inside the worker
          const h64 = await ev(`(() => { const p = MPP.createGridProblem(8, 8, 0, 64, new MPP.MathRandom(1)), c = (v) => p.graph.xy[v][1] * 8 + p.graph.xy[v][0];
            return '#edit=8x8&o=&s=' + p.starts.map(c).join('.') + '&g=' + p.goals.map(c).join('.') + '&plan=makespan%2Fk2'; })()`);
          await ev(`location.hash = ${J(h64)}`);
          await waitFor(`document.getElementById('edit-stats').textContent.indexOf('64 robots') >= 0`, 5000, '64-robot instance');
          await click('#live-btn');
          await waitFor(`document.getElementById('live-cancel').hidden`, 30000, '64-robot split').catch((e) => fail(e.message));
          R.k2big = await ev(`document.getElementById('live-result').textContent`);
          if (!/Too large for the browser/.test(R.k2big)) fail('a split piece over the memory budget is not stopped with a message: ' + R.k2big.slice(0, 300));
          await click('#edit-done');
        });
      }
      // the result texts: an infeasible instance says so once, and a plan whose build-time Python check stopped at
      // its iteration budget does not quote that run's time as the time of "the same solve"
      await step('live result texts', async () => {
        const R = report.live_texts = {};
        const waitDone = (what, ms) => waitFor(`document.getElementById('live-progress').hidden && !document.getElementById('live-btn').disabled`, ms, what, 200);
        await ev(`location.hash = '#edit=2x2&o=&s=0.1.2.3&g=1.0.2.3&plan=makespan%2Fk1'`);
        await waitFor(`/4 robots/.test(document.getElementById('edit-stats').textContent)`, 5000, 'fully occupied 2 x 2 instance');
        await waitDone('the Solve button', 10000);
        await click('#live-btn');
        // the Solve button stays disabled after the proof: wait for the result instead
        await waitFor(`document.getElementById('live-progress').hidden && /No plan/.test(document.getElementById('live-result').textContent)`, 30000, 'the infeasible solve', 200);
        R.infeasible = await ev(`document.getElementById('live-result').textContent`);
        if (!/^No plan\. The instance has no solution for any horizon \(proved by the joint-state search\)\.$/.test(R.infeasible)) {
          fail('the infeasible 2 x 2 transposition reads ' + J(R.infeasible));
        }
        // the proof covers every objective and k: the page says so and offers no further solve
        const infState = `({ status: document.getElementById('plan-status').textContent, table: document.getElementById('metrics').textContent,
                             btn: document.getElementById('live-btn').disabled, info: document.getElementById('live-info').textContent })`;
        R.infeasibleState = { same: await ev(infState) };
        await chooseRadio('obj-total_time');
        R.infeasibleState.other = await ev(infState);
        for (const [what, x] of Object.entries(R.infeasibleState)) {
          if (!/^infeasibleNo plan exists/i.test(x.status) || !/No plan exists for this instance/.test(x.table) || !x.btn ||
              !/no solution for any horizon/.test(x.info)) {
            fail('after the infeasible solve (' + what + ' choice) the page still offers to solve it: ' + J(x));
          }
        }
        await chooseRadio('obj-makespan');
        await click('#edit-done');
        const p9 = MAPS.find((m) => m.id === 'tor16-fig1-9puzzle');
        const pl = p9 && p9.plans['total_time/k1'];
        if (pl && pl.live_check && pl.live_check.value === null) {
          await selectMap(p9);
          await selectPlan('total_time/k1');
          await waitDone('the Solve button', 10000);
          await click('#live-btn');
          await waitDone('the 9-puzzle total-time solve', OPT.liveTimeout * 1000);
          R.p9tt = await ev(`document.getElementById('live-result').textContent`);
          if (/Valid plan/.test(R.p9tt) && (/The same solve with the Python builtin solver took/.test(R.p9tt) ||
                                            !/the Python builtin solver did not finish this solve/.test(R.p9tt))) {
            fail('the 9-puzzle total-time live result compares with an unfinished Python run: ' + J(R.p9tt.slice(0, 500)));
          } else if (!/Valid plan/.test(R.p9tt)) warn('the 9-puzzle total-time live solve gave no plan: ' + R.p9tt.slice(0, 200));
        }
        // after k = 1 and k = 2 both stop at the time limit, neither result suggests the other one again
        await ev(`location.hash = '#edit=6x6&o=&s=0.1.2.3.4.5.6.7.8.9.10.11&g=35.34.33.32.31.30.29.28.27.26.25.24&plan=makespan%2Fk1'`);
        await waitFor(`/12 robots/.test(document.getElementById('edit-stats').textContent)`, 5000, '6 x 6 instance with 12 robots');
        const tl0 = await ev(`document.getElementById('live-tl').value`);
        await setControl('live-tl', '10');
        const res = {};
        for (const k of ['1', '2']) {
          await chooseRadio('k-' + k);
          await waitDone('the Solve button', 10000);
          await click('#live-btn');
          await waitDone('the 6 x 6 solve with k = ' + k, 40000);
          res[k] = await ev(`document.getElementById('live-result').textContent`);
        }
        await chooseRadio('k-1');
        res['1 again'] = await ev(`document.getElementById('live-result').textContent`);
        R.advice = res;
        if (/Stopped at the time limit/.test(res['1']) && /Stopped at the time limit/.test(res['2'])) {
          if (/Try the exact solve/.test(res['2']) || !/Already tried here without a plan, for the makespan: k = 1 /.test(res['2'])) {
            fail('the k = 2 advice suggests the exact solve that already stopped at the time limit: ' + J(res['2']));
          }
          if (/k = 2, 4/.test(res['1 again']) || !/Already tried here without a plan, for the makespan: k = 2 /.test(res['1 again'])) {
            fail('the k = 1 advice suggests the 2-way split that already stopped at the time limit: ' + J(res['1 again']));
          }
        } else warn('the 6 x 6 instance did not time out at k = 1 and k = 2 within 10 s; the advice was not checked');
        await setControl('live-tl', tl0);
        await click('#edit-done');
      });
      // a plan from the joint-state search (no model solved at its horizon) is listed as such; a link that changes
      // only the plan keeps the instance's live results
      await step('live plan sources and plan-only links', async () => {
        const R = report.live_sources = {};
        const waitDone = (what, ms) => waitFor(`document.getElementById('live-progress').hidden && !document.getElementById('live-btn').disabled`, ms, what, 200);
        const facts = `(() => { const o = {}; const dl = document.getElementById('facts'); let k = null;
          [...dl.children].forEach((e) => { if (e.tagName === 'DT') k = e.textContent; else if (k) o[k] = e.textContent; }); return o; })()`;
        for (const [hh, re] of [['#edit=3x3&o=&s=2.0&g=0.1&plan=makespan%2Fk1', /^1 \(T = 2 \(LP\), 3 \(joint-state search\)\); \(LP\): an LP relaxation, solved first /],
                                ['#edit=3x3&o=&s=2.0&g=3.5&plan=makespan%2Fk2', /^2 \(T = 2, 3 \(joint-state search\), 2\);.* both pieces, in order/]]) {
          await ev(`location.hash = ${J(hh)}`);
          await waitFor(`document.getElementById('view-title').textContent === 'Your instance'`, 5000, 'edited instance ' + hh);
          await waitDone('the Solve button', 10000);
          await click('#live-btn');
          await waitDone('the solve of ' + hh, 30000);
          const f = await ev(facts), st = await ev(`document.getElementById('plan-status').textContent`);
          (R.cases = R.cases || []).push({ hash: hh, facts: f, status: st });
          if (!re.test(f['Models solved'] || '') || !f.Optimality || !/joint-state search/.test(st)) {
            fail('the live facts do not say that the plan comes from the joint-state search (' + hh + '): ' + J({ f, st }));
          }
        }
        const inst = '#edit=4x4&o=5&s=0.15&g=15.0';
        await ev(`location.hash = ${J(inst + '&plan=makespan%2Fk1')}`);
        await waitFor(`/4 × 4|4 x 4/.test(document.getElementById('edit-stats').textContent) || /2 robots/.test(document.getElementById('edit-stats').textContent)`, 5000, 'edited 4 x 4 instance');
        await waitDone('the Solve button', 10000);
        await click('#live-btn');
        await waitDone('the 4 x 4 solve', 30000);
        const src0 = await ev(`document.getElementById('map-source').textContent`);
        await ev(`location.hash = ${J(inst + '&plan=total_time%2Fk1')}`);
        await waitFor(`/total_time/.test(location.hash) && document.getElementById('obj-total_time').checked`, 3000, 'the plan-only link');
        await ev(`location.hash = ${J(inst + '&plan=makespan%2Fk1')}`);
        await waitFor(`document.getElementById('obj-makespan').checked`, 3000, 'the plan-only link back');
        R.planOnly = await ev(`({ result: document.getElementById('live-result').textContent.slice(0, 200), source: document.getElementById('map-source').textContent,
                                  note: document.getElementById('hash-note').hidden ? '' : document.getElementById('hash-note').textContent })`);
        if (!/Valid plan/.test(R.planOnly.result) || R.planOnly.source !== src0 || R.planOnly.note) {
          fail('a link that changes only the plan reset the edited instance: ' + J(Object.assign({ before: src0 }, R.planOnly)));
        }
        await click('#edit-done');
      });
      // without Web Workers (or when the worker fails to start) the page solves on the main thread, a k-way split
      // piece by piece: the Fig. 8 split must still give makespan 5, and a piece over the memory budget must still be
      // refused with a message (not freeze the tab)
      if (fig8 && fig8.live) {
        await step('live without a Web Worker', async () => {
          const R = report.live_no_worker = {};
          const h64 = await ev(`(() => { const p = MPP.createGridProblem(8, 8, 0, 64, new MPP.MathRandom(1)), c = (v) => p.graph.xy[v][1] * 8 + p.graph.xy[v][0];
            return '#edit=8x8&o=&s=' + p.starts.map(c).join('.') + '&g=' + p.goals.map(c).join('.') + '&plan=makespan%2Fk2'; })()`);
          const { identifier } = await send('Page.addScriptToEvaluateOnNewDocument', { source: 'window.Worker = undefined;' });
          const reloadAt = async (hash) => { await send('Page.navigate', { url: 'about:blank' }); await sleep(200); await navigate(pageUrl + hash); };
          const solveAndWait = async (what, ms) => {
            await click('#live-btn');
            await waitFor(`document.getElementById('live-progress').hidden && !document.getElementById('live-btn').disabled`, ms, what, 200);
            return ev(`({ result: document.getElementById('live-result').textContent, facts: document.getElementById('facts').textContent,
                          label: document.getElementById('pb-label').textContent })`);
          };
          try {
            await reloadAt('#map=' + fig8.id + '&plan=makespan%2Fk2');
            R.worker = await ev(`typeof window.Worker`);
            if (R.worker !== 'undefined') { fail('could not hide window.Worker for the main-thread check (' + R.worker + ')'); return; }
            const a = await solveAndWait('main-thread split of ' + fig8.id, OPT.liveTimeout * 1000);
            R.fig8 = { result: a.result.slice(0, 300), label: a.label, mainThread: /main thread/.test(a.facts) };
            if (!/Valid plan/.test(a.result) || !R.fig8.mainThread || !/\/ 5$/.test(a.label)) {
              fail('the 2-way split of ' + fig8.id + ' on the main thread: ' + J(R.fig8));
            }
            // a Cancel click queued while a piece runs stops the split before the next piece starts (it does not
            // run that piece, freezing the page, only to drop its plan)
            const p9c = MAPS.find((m) => m.id === 'tor16-fig1-9puzzle');
            if (p9c && p9c.live) {
              await reloadAt('#map=' + p9c.id + '&plan=makespan%2Fk2');
              await ev(`(() => { window.__clog = []; window.__ct0 = performance.now(); const t = () => Math.round(performance.now() - window.__ct0);
                new MutationObserver(() => { window.__clog.push([t(), document.getElementById('live-result').textContent.slice(0, 80)]); })
                  .observe(document.getElementById('live-result'), { childList: true, subtree: true, characterData: true });
                document.getElementById('live-cancel').addEventListener('click', () => window.__clog.push([t(), 'CANCEL']), true); })()`);
              const cb = await ev(`(() => { const e = document.getElementById('live-cancel'); e.hidden = false; e.scrollIntoView({ block: 'center' });
                const r = e.getBoundingClientRect(); e.hidden = true; return { x: r.left + r.width / 2, y: r.top + r.height / 2 }; })()`);
              await ev(`window.__ct0 = performance.now(); setTimeout(() => document.getElementById('live-btn').click(), 0); true`);
              await sleep(300);
              // the three events queue at once while the main thread is busy with a piece
              for (const type of ['mouseMoved', 'mousePressed', 'mouseReleased']) {
                send('Input.dispatchMouseEvent', { type, x: cb.x, y: cb.y, button: type === 'mouseMoved' ? 'none' : 'left', buttons: type === 'mousePressed' ? 1 : 0, clickCount: 1 }).catch(() => {});
              }
              await waitFor(`document.getElementById('live-cancel').hidden`, OPT.liveTimeout * 1000, 'the cancelled main-thread split', 200).catch((e) => fail(e.message));
              const lg = R.chunkCancel = await ev(`window.__clog`);
              const ci = lg.findIndex((x) => x[1] === 'CANCEL'), last = lg[lg.length - 1];
              if (ci < 0) warn('the queued Cancel click was not handled; the main-thread cancel was not checked');
              else if (!/^Cancelled\./.test(last[1])) {
                if (/Valid plan/.test(last[1]) && ci === lg.length - 2) warn('the main-thread split ended before the Cancel click was handled');
                else fail('a Cancel during a main-thread split piece: ' + J(lg));
              } else if (last[0] - lg[ci][0] > 1500) {
                fail('a Cancel during a main-thread split piece let the next piece run for ' + (last[0] - lg[ci][0]) + ' ms: ' + J(lg));
              }
            }
            await reloadAt(h64);
            // ev waits up to 60 s for an answer: a frozen main thread fails here instead of hanging the check
            const b = await solveAndWait('main-thread split of the 64-robot instance', 30000);
            R.k2big = b.result.slice(0, 400);
            if (!/Too large for the browser/.test(b.result)) fail('without a Web Worker, a split piece over the memory budget is not stopped with a message: ' + R.k2big);
          } finally {
            await send('Page.removeScriptToEvaluateOnNewDocument', { identifier });
            await reloadAt('');
          }
        });
      }
    }

    /** Play whatever plan is shown (live) and check robots end on their goals. */
    async function playToEndLive(map, T) {
      const problems = [];
      await click('#pb-reset');
      let st = await state();
      problems.push(...atVertices(st, map, map.starts));
      if (T > 0) {
        if (OPT.fast) { await ev(`document.getElementById('view').focus()`); await pressKey('End', 'End', 35); }
        else {
          await click('#pb-play');
          await waitFor(`!document.getElementById('pb-play').classList.contains('playing')`, Math.ceil(T / 8 * 2000 + 5000), 'live playback end').catch((e) => problems.push(e.message));
        }
      }
      st = await state();
      if (st.label !== 't = ' + T + ' / ' + T) problems.push('label "' + st.label + '" at the end');
      problems.push(...atVertices(st, map, map.goals));
      if (st.robots.some((r) => !r.home)) problems.push('some robot lacks the "home" class at the end');
      return problems;
    }

    // ------------------------------------------------------------ edit mode
    await step('edit mode', async () => {
      const E = report.edit;
      const base = MAPS.find((m) => m.id === 'tor16-fig8-split') || MAPS.find((m) => m.graph.rows && m.graph.rows <= 8 && m.graph.cols <= 8);
      if (!base) { warn('no small grid map to edit'); return; }
      await selectMap(base);
      await click('#edit-btn');
      const panel = `(() => ({ edit: !document.getElementById('edit-panel').hidden, pick: !document.getElementById('map-pick').hidden,
        title: document.getElementById('view-title').textContent, stats: document.getElementById('edit-stats').textContent,
        msg: document.getElementById('edit-msg').textContent, discs: document.querySelectorAll('#view-svg .edit-disc:not(.passive)').length,
        source: document.getElementById('map-source').textContent, playTool: document.getElementById('tool-play').disabled ? 'disabled' : document.getElementById('tool-play').checked ? 'checked' : 'enabled' }))()`;
      E.entered = await ev(panel);
      if (!E.entered.edit || E.entered.pick || E.entered.title !== 'Your instance') fail('edit mode did not open: ' + J(E.entered));
      if (E.entered.discs !== base.starts.length) fail('edit mode shows ' + E.entered.discs + ' start discs, expected ' + base.starts.length);
      await screenshot('desktop-edit', false);
      // drag robot 1's start to a free neighbouring vertex
      const dragPlan = await ev(`(() => {
        const disc = document.querySelector('#view-svg .edit-disc[data-layer="starts"][data-robot="0"]');
        const occupied = [...document.querySelectorAll('#view-svg .edit-disc[data-layer="starts"]')].map((d) => { const b = d.getBoundingClientRect(); return [b.left + b.width / 2, b.top + b.height / 2]; });
        disc.scrollIntoView({ block: 'center' });
        const b = disc.getBoundingClientRect(), c = [b.left + b.width / 2, b.top + b.height / 2];
        const vs = [...document.querySelectorAll('#view-svg .g-vertex')].map((v) => { const r = v.getBoundingClientRect(); return [r.left + r.width / 2, r.top + r.height / 2]; });
        let unit = Infinity;
        vs.forEach((v) => { const d = Math.hypot(v[0] - c[0], v[1] - c[1]); if (d > 2 && d < unit) unit = d; });
        const target = vs.find((v) => { const d = Math.hypot(v[0] - c[0], v[1] - c[1]); return d > 2 && d < unit * 1.05 && !occupied.some((o) => Math.hypot(o[0] - v[0], o[1] - v[1]) < unit / 3); });
        return target ? { from: { x: c[0], y: c[1] }, to: { x: target[0], y: target[1] } } : null;
      })()`);
      if (dragPlan) {
        await drag(dragPlan.from, dragPlan.to);
        const after = await ev(`(() => { const d = document.querySelector('#view-svg .edit-disc[data-layer="starts"][data-robot="0"]'); const b = d.getBoundingClientRect(); return { x: b.left + b.width / 2, y: b.top + b.height / 2, source: document.getElementById('map-source').textContent }; })()`);
        E.drag = { from: dragPlan.from, to: dragPlan.to, now: { x: after.x, y: after.y } };
        if (Math.hypot(after.x - dragPlan.to.x, after.y - dragPlan.to.y) > 3) fail('dragging robot 1\'s start did not move it to the target vertex: ' + J(E.drag));
      } else warn('no free neighbouring vertex to drag robot 1 to');
      // randomize
      await setControl('ed-rows', '5'); await setControl('ed-cols', '5');
      await ev(`(() => { const n = document.getElementById('ed-n'); n.value = '4'; const s = document.getElementById('ed-seed'); s.value = '3'; })()`);
      await click('#ed-random');
      E.randomized = await ev(panel);
      if (!/Generated with seed 3/.test(E.randomized.msg) || E.randomized.discs !== 4) fail('Randomize did not generate 4 robots with seed 3: ' + J(E.randomized));
      // obstacles: click a free cell
      await chooseRadio('tool-obstacles');
      const cellPick = await ev(`(() => {
        const discs = [...document.querySelectorAll('#view-svg .edit-disc')].map((d) => { const b = d.getBoundingClientRect(); return [b.left + b.width / 2, b.top + b.height / 2]; });
        const cells = [...document.querySelectorAll('#view-svg .edit-cell')];
        const desc = (n) => n ? n.nodeName.toLowerCase() + (n.getAttribute('class') ? '.' + n.getAttribute('class').split(' ')[0] : '') : 'nothing';
        for (const c of cells) {
          if (!/free/.test(c.getAttribute('aria-label'))) continue;
          c.scrollIntoView({ block: 'center' });
          const b = c.getBoundingClientRect();
          if (discs.some((d) => d[0] >= b.left && d[0] <= b.right && d[1] >= b.top && d[1] <= b.bottom)) continue;   // a robot starts or ends here
          const centre = desc(document.elementFromPoint(b.left + b.width / 2, b.top + b.height / 2));
          for (const fy of [0.5, 0.3, 0.7, 0.2, 0.8]) for (const fx of [0.5, 0.3, 0.7, 0.2, 0.8]) {
            const x = b.left + b.width * fx, y = b.top + b.height * fy;
            if (document.elementFromPoint(x, y) === c) return { x, y, fx, fy, centre, label: c.getAttribute('aria-label') };
          }
        }
        return null;
      })()`);
      if (cellPick) {
        E.obstacleClick = cellPick;
        if (cellPick.fx !== 0.5 || cellPick.fy !== 0.5) {
          fail('obstacle tool: the centre of a free cell is covered by ' + cellPick.centre + ' (not pointer-events: none), so a click on the cell centre does nothing; clicked at (' + cellPick.fx + ', ' + cellPick.fy + ') of the cell instead');
        }
        const before = E.randomized.stats;
        for (const type of ['mouseMoved', 'mousePressed', 'mouseReleased']) {
          await send('Input.dispatchMouseEvent', { type, x: cellPick.x, y: cellPick.y, button: type === 'mouseMoved' ? 'none' : 'left', buttons: type === 'mousePressed' ? 1 : 0, clickCount: type === 'mouseMoved' ? 0 : 1 });
        }
        await sleep(60);
        E.obstacle = await ev(panel);
        if (E.obstacle.stats === before) fail('clicking a free cell with the obstacle tool did not change the instance (' + before + ')');
      } else fail('no free cell without a robot that a click can reach with the obstacle tool');
      await chooseRadio('tool-starts');
      await click('#ed-add');
      E.added = await ev(panel);
      const nAfter = E.added.discs;
      if (!/Added robot/.test(E.added.msg)) fail('Add robot did not report: ' + E.added.msg);
      // live solve of the edited instance, then play it
      if (OPT.live) {
        if (!(await radioChecked('obj-makespan'))) await chooseRadio('obj-makespan');
        if (!(await radioChecked('k-1'))) await chooseRadio('k-1');
        const btnOff = await ev(`document.getElementById('live-btn').disabled`);
        if (btnOff) fail('the live button is disabled in edit mode');
        else {
          const custom = await ev(`(() => { const s = [...document.querySelectorAll('#view-svg .edit-disc[data-layer="starts"]')].length; return s; })()`);
          await click('#live-btn');
          await waitFor(`document.getElementById('live-progress').hidden && !document.getElementById('live-btn').disabled`, OPT.liveTimeout * 1000, 'edit-mode live solve', 200);
          E.live = await ev(`({ result: document.getElementById('live-result').textContent.slice(0, 300), play: document.getElementById('tool-play').checked,
                                playDisabled: document.getElementById('tool-play').disabled, robots: document.querySelectorAll('#view-svg .robot:not(.edit-disc)').length,
                                label: document.getElementById('pb-label').textContent })`);
          if (!/Valid plan/.test(E.live.result)) fail('edit-mode live solve: ' + E.live.result);
          else {
            if (!E.live.play || E.live.playDisabled) fail('after the live solve the "Play plan" tool is not selected');
            if (E.live.robots !== custom || custom !== nAfter) fail('edit-mode live plan draws ' + E.live.robots + ' robots, the instance has ' + custom);
            // goals: read the goal discs drawn by the plan view
            const T = +E.live.label.split('/')[1];
            await click('#pb-reset');
            if (OPT.fast) { await ev(`document.getElementById('view').focus()`); await pressKey('End', 'End', 35); }
            else {
              await click('#pb-play');
              await waitFor(`!document.getElementById('pb-play').classList.contains('playing')`, Math.ceil(T / 8 * 2000 + 5000), 'edit playback').catch((e) => fail(e.message));
            }
            const st = await state();
            E.liveEnd = st.label;
            if (st.label !== 't = ' + T + ' / ' + T) fail('edit-mode playback ended at "' + st.label + '"');
            const off = st.robots.filter((r, i) => !r.home || !close(r.p, st.goals[i])).length;
            if (off) fail('edit-mode live plan: ' + off + ' robots not on their goals at the end');
            await chooseRadio('tool-starts');
            const back = await ev(panel);
            if (back.discs !== nAfter) fail('back to the starts tool shows ' + back.discs + ' discs');
          }
        }
      }
      await click('#edit-done');
      E.exited = await ev(panel);
      if (E.exited.edit || !E.exited.pick || E.exited.title !== base.title) fail('"Back to the paper\'s maps" did not return to ' + base.title + ': ' + J(E.exited));
    });

    // ------------------------------------------------------------ URL hash: same-tab navigation and the edited instance
    await step('hash', async () => {
      const H = report.hash = {};
      const a = MAPS.find((m) => m.id === 'tor16-fig8-split') || MAPS[0];
      const b = MAPS.find((m) => m.id === 'tor16-fig1-9puzzle' && m !== a) || MAPS.find((m) => m !== a);
      if (!a || !b) return;
      await selectMap(a);
      await ev(`location.hash = ${J('#map=' + b.id + '&plan=makespan%2Fk1')}`);
      await waitFor(`document.getElementById('view-title').textContent === ${J(b.title)}`, 3000, 'hashchange to ' + b.id).catch((e) => fail('editing the hash does not switch the map: ' + e.message));
      H.afterHashchange = await ev(`document.getElementById('view-title').textContent`);
      const edit = '#edit=4x4&o=5&s=0.1&g=15.12&plan=makespan%2Fk1';
      await ev(`location.hash = ${J(edit)}`);
      await waitFor(`document.getElementById('view-title').textContent === 'Your instance'`, 3000, 'hashchange to an edited instance').catch((e) => fail('an #edit= hash does not open the instance: ' + e.message));
      H.edit = await ev(`({ stats: document.getElementById('edit-stats').textContent, discs: document.querySelectorAll('#view-svg .edit-disc[data-layer="starts"]').length, hash: location.hash })`);
      if (H.edit.discs !== 2 || !/1 obstacle/.test(H.edit.stats)) fail('the #edit= hash gave ' + J(H.edit));
      H.editHint = await ev(`document.getElementById('view-hint').textContent`);
      if (!/Drag a start/.test(H.editHint)) fail('edit mode: the hint under the view does not explain dragging: ' + J(H.editHint));
      await click('#ed-add');
      H.afterAdd = await ev(`location.hash`);
      if (!/^#edit=4x4&/.test(H.afterAdd) || H.afterAdd === edit) fail('the hash does not follow the edited instance: ' + H.afterAdd);
      await send('Page.navigate', { url: 'about:blank' });   // a real reload, not a same-document navigation
      await sleep(200);
      await navigate(pageUrl + H.afterAdd);
      H.reload = await ev(`({ title: document.getElementById('view-title').textContent, discs: document.querySelectorAll('#view-svg .edit-disc[data-layer="starts"]').length })`);
      if (H.reload.title !== 'Your instance' || H.reload.discs !== 3) fail('reloading an #edit= link does not restore the instance: ' + J(H.reload));
      // shrinking the grid drops the robots outside it: say so (all three here, so one new robot is placed)
      await setControl('ed-rows', '2');
      H.resize = await ev(`({ msg: document.getElementById('edit-msg').textContent, n: +document.getElementById('ed-n').value,
                              transpose: document.getElementById('opt-transpose').parentNode.hidden })`);
      if (!/^Removed robots 1–3: their start or goal is outside the 2 × 4 grid\. Added one new robot/.test(H.resize.msg) || H.resize.n > 8) {
        fail('shrinking the edited grid does not report the removed robots: ' + J(H.resize));
      }
      if (!H.resize.transpose) fail('the transpose box is offered while editing a 2 x 4 grid');
      // a size that leaves only obstacles is refused, and the grid keeps its size and its robot
      await ev(`location.hash = '#edit=4x4&o=0.1.4.5&s=15&g=10&plan=makespan%2Fk1'`);
      await waitFor(`/4 obstacles/.test(document.getElementById('edit-stats').textContent)`, 3000, 'edited 4 x 4 instance with 4 obstacles');
      await setControl('ed-rows', '2');
      await setControl('ed-cols', '2');
      H.resizeAllBlocked = await ev(`({ msg: document.getElementById('edit-msg').textContent, stats: document.getElementById('edit-stats').textContent,
                                        cols: document.getElementById('ed-cols').value, hash: location.hash })`);
      if (!/every cell would be an obstacle, so the grid stays 2 × 4/.test(H.resizeAllBlocked.msg) || H.resizeAllBlocked.cols !== '4' ||
          !/ 1 robot/.test(H.resizeAllBlocked.stats) || !/^#edit=2x4&o=0\.1\.4\.5&s=\d+&g=\d+/.test(H.resizeAllBlocked.hash)) {
        fail('shrinking the grid to obstacles only: ' + J(H.resizeAllBlocked));
      }
      await click('#edit-done');
      // links the page cannot follow exactly: an unknown map, a plan the map does not have
      for (const [hh, re] of [['#map=no-such-map&plan=makespan%2Fk1', /names the map “no-such-map”, which this demo does not have/],
                              ['#map=tor16-fig8-split&plan=paper', /asks for the paper’s plan, which “Split example” does not offer; showing the makespan plan \(k = 1\)/]]) {
        await ev(`location.hash = ${J(hh)}`);
        await sleep(200);
        const note = await ev(`document.getElementById('hash-note').hidden ? '' : document.getElementById('hash-note').textContent`);
        (H.notes = H.notes || []).push({ hash: hh, note });
        if (!re.test(note)) fail('the link ' + hh + ' is not followed with a notice: ' + J(note));
      }
      await selectMap(b);
      if (!(await ev(`document.getElementById('hash-note').hidden`))) fail('the link notice stays after choosing a map');
      // an invalid #edit= link (3 starts, 1 goal) is reported, and the notice goes with the next choice of a map
      await ev(`location.hash = '#edit=4x4&o=&s=0.1.2&g=3'`);
      await waitFor(`!document.getElementById('hash-note').hidden`, 3000, 'invalid #edit= notice').catch((e) => fail('an invalid #edit= link is dropped without a notice: ' + e.message));
      H.invalid = await ev(`document.getElementById('hash-note').textContent`);
      await selectMap(a);
      if (!(await ev(`document.getElementById('hash-note').hidden`))) fail('the invalid-link notice stays after choosing a map');
      // an invalid edit= with a valid map=: the notice names the map that is shown
      const p25 = MAPS.find((m) => m.id === 'tor16-fig14-25puzzle');
      if (p25) {
        await ev(`location.hash = '#map=tor16-fig14-25puzzle&plan=paper&edit=bad'`);
        await waitFor(`document.getElementById('view-title').textContent === ${J(p25.title)}`, 3000, 'the 25-puzzle link');
        H.invalidWithMap = await ev(`document.getElementById('hash-note').textContent`);
        if (!H.invalidWithMap.includes('the page shows “' + p25.title + '” instead')) fail('an invalid edit= with map= names the wrong instance: ' + J(H.invalidWithMap));
      }
      // a link without map= (only plan=): it picks the plan of the map on screen, and the earlier notice goes
      if (b.plans.paper && b.plans['total_time/k1'] && !b.plans['total_time/k2']) {
        await ev(`location.hash = ${J('#map=' + b.id + '&plan=total_time%2Fk2')}`);
        await waitFor(`!document.getElementById('hash-note').hidden`, 3000, 'the plan notice');
        await ev(`location.hash = '#plan=paper'`);
        await sleep(200);
        H.planOnlyLink = await ev(`({ note: document.getElementById('hash-note').hidden ? '' : document.getElementById('hash-note').textContent,
                                      paper: document.getElementById('src-paper') ? document.getElementById('src-paper').checked : null, hash: location.hash })`);
        if (H.planOnlyLink.note || !H.planOnlyLink.paper) fail('the link #plan=paper keeps the old notice or ignores the plan: ' + J(H.planOnlyLink));
      }
      // edit mode has no highlight: its legend entry goes with the Highlight menu
      await ev(`location.hash = '#edit=4x4&o=5&s=0.15&g=15.0&plan=makespan%2Fk1'`);
      await waitFor(`document.getElementById('view-title').textContent === 'Your instance'`, 3000, 'edited instance');
      H.legendHl = await ev(`document.getElementById('legend-hl').hidden`);
      if (!H.legendHl) fail('edit mode still lists the highlighted-robot legend entry');
      await click('#edit-done');
      if (await ev(`document.getElementById('legend-hl').hidden`)) fail('the highlighted-robot legend entry stays hidden after edit mode');
    });

    // ------------------------------------------------------------ screen readers and the keyboard
    // live regions speak only when asked: the time label is rewritten on every animation frame and the network's
    // statistics line on every edit, so neither is live; the time is said after a step and at the end of playback,
    // the statistics after a change of the network's controls. Sliders have a name; an arrow key moves an edited
    // disc over an obstacle.
    await step('screen reader and keyboard', async () => {
      const A = report.a11y = {};
      const p9 = MAPS.find((m) => m.id === 'tor16-fig1-9puzzle') || MAPS[0];
      await selectMap(p9);
      await selectPlan('makespan/k1');
      A.live = await ev(`(() => { const live = (id) => { const e = document.getElementById(id); return e.getAttribute('aria-live') || (/^(OUTPUT)$/.test(e.nodeName) || /^(status|log|alert)$/.test(e.getAttribute('role') || '') ? 'implicit' : null); };
        return { pbLabel: live('pb-label'), netInfo: live('net-info'), netT: live('net-T-out') }; })()`);
      for (const [k, v] of Object.entries(A.live)) if (v && v !== 'off') fail('#' + k + ' is a live region (' + v + '), so a screen reader reads every update of it');
      await click('#pb-reset');
      await click('#pb-fwd');
      A.afterStep = await ev(`document.getElementById('pb-announce').textContent`);
      const T = await ev(`+document.getElementById('pb-label').textContent.split('/')[1]`);
      if (A.afterStep !== 'time 1 of ' + T) fail('a step does not say the time: #pb-announce = ' + J(A.afterStep));
      await setControl('pb-speed', '8');
      await ev(`(() => { window.__said = 0; new MutationObserver(() => { window.__said++; }).observe(document.getElementById('pb-announce'), { childList: true, characterData: true, subtree: true }); })()`);
      await click('#pb-play');
      await sleep(300);
      A.saidWhilePlaying = await ev(`window.__said`);
      if (A.saidWhilePlaying > 0) fail('the time is announced ' + A.saidWhilePlaying + ' times during playback');
      await waitFor(`!document.getElementById('pb-play').classList.contains('playing')`, T * 1000 + 3000, 'playback end');
      A.atEnd = await ev(`document.getElementById('pb-announce').textContent`);
      if (A.atEnd !== 'time ' + T + ' of ' + T) fail('the end of playback is not announced: ' + J(A.atEnd));
      // every slider has a name that reads well (no dangling "=")
      const ax = await send('Accessibility.getFullAXTree', {});
      A.sliders = (ax.nodes || []).filter((n) => n.role && n.role.value === 'slider' && !n.ignored)
        .map((n) => ({ name: n.name ? String(n.name.value) : '', valuetext: ((n.properties || []).find((q) => q.name === 'valuetext') || {}).value }));
      for (const sl of A.sliders) if (!sl.name.trim() || /[=:]\s*$/.test(sl.name)) fail('a slider is named ' + J(sl.name));
      // edited instance: robot 1 at row 2, column 1 with an obstacle to its right
      await ev(`location.hash = '#edit=4x4&o=5&s=4.15&g=15.4&plan=makespan%2Fk1'`);
      await waitFor(`document.getElementById('view-title').textContent === 'Your instance' && !!document.querySelector('#view-svg .edit-disc[data-layer="starts"][data-robot="0"]')`, 5000, 'edited 4 x 4 instance');
      await ev(`(() => { window.__netSaid = 0; new MutationObserver(() => { window.__netSaid++; }).observe(document.getElementById('net-announce'), { childList: true, characterData: true, subtree: true }); })()`);
      await ev(`document.querySelector('#view-svg .edit-disc[data-layer="starts"][data-robot="0"]').focus()`);
      await pressKey('ArrowRight', 'ArrowRight', 39);
      await sleep(500);
      A.jump = await ev(`({ hash: location.hash, msg: document.getElementById('edit-msg').textContent,
                            focus: document.activeElement && document.activeElement.getAttribute('aria-label'), netSaid: window.__netSaid })`);
      if (!/&s=6\.15&/.test(A.jump.hash) || !/^Jumped over 1 obstacle\./.test(A.jump.msg)) fail('ArrowRight next to an obstacle does not jump over it: ' + J(A.jump));
      if (!/row 2, column 3/.test(A.jump.focus || '')) fail('after the jump the focus is not on the moved disc: ' + J(A.jump.focus));
      if (A.jump.netSaid > 0) fail('moving an edited disc announces the network statistics');
      // the view's name in edit mode gives the edit keys, not the playback keys (there is no plan to play)
      A.editViewName = await ev(`document.getElementById('view').getAttribute('aria-label')`);
      if (/Press space/.test(A.editViewName) || !/Tab to a disc and use the arrow keys/.test(A.editViewName)) {
        fail('edit mode: the view is named ' + J(A.editViewName));
      }
      // the obstacle grid is one tab stop: the arrow keys move between the cells, Enter toggles the focused one
      await chooseRadio('tool-obstacles');
      A.cells = await ev(`(() => { const z = [...document.querySelectorAll('#view-svg .edit-cell[tabindex="0"]')];
        return { stops: z.length, cell: z[0] && z[0].getAttribute('data-cell'), name: document.getElementById('view').getAttribute('aria-label') }; })()`);
      if (A.cells.stops !== 1) fail('the obstacle grid has ' + A.cells.stops + ' tab stops, expected 1');
      if (!/move between the cells with the arrow keys/.test(A.cells.name || '')) fail('obstacle tool: the view is named ' + J(A.cells.name));
      await ev(`document.querySelector('#view-svg .edit-cell[data-cell="0"]').focus()`);
      await pressKey('ArrowRight', 'ArrowRight', 39);
      await pressKey('ArrowDown', 'ArrowDown', 40);
      A.cellMove = await ev(`({ focus: document.activeElement && document.activeElement.getAttribute('data-cell'),
                                stops: [...document.querySelectorAll('#view-svg .edit-cell[tabindex="0"]')].map((e) => e.getAttribute('data-cell')) })`);
      if (A.cellMove.focus !== '5' || A.cellMove.stops.join() !== '5') fail('arrow keys in the obstacle grid: ' + J(A.cellMove) + ', expected focus and tab stop on cell 5');
      await pressKey('ArrowDown', 'ArrowDown', 40);
      await pressKey('Enter', 'Enter', 13);
      await sleep(200);
      A.cellToggle = await ev(`({ focus: document.activeElement && document.activeElement.getAttribute('data-cell'), hash: location.hash })`);
      if (A.cellToggle.focus !== '9' || !/^#edit=4x4&o=5\.9&/.test(A.cellToggle.hash)) fail('Enter in the obstacle grid: ' + J(A.cellToggle));
      await chooseRadio('tool-starts');
      await setControl('net-T', '5', 'input');
      await sleep(600);
      A.netSay = await ev(`({ said: document.getElementById('net-announce').textContent, info: document.getElementById('net-info').textContent })`);
      if (!A.netSay.said || A.netSay.said !== A.netSay.info) fail('a change of the horizon does not say the network statistics: ' + J(A.netSay));
      // after a live solve, an edit tool shows the starts with the playbar off: the play keys must not play the plan
      // unseen (the time label and the announcement would count a time that the view does not show)
      await click('#live-btn');
      await waitFor(`document.getElementById('live-progress').hidden && !document.getElementById('live-btn').disabled && /Valid plan/.test(document.getElementById('live-result').textContent)`,
                    30000, 'the live solve of the edited 4 x 4 instance', 200).catch((e) => fail(e.message));
      await chooseRadio('tool-starts');
      const said0 = await ev(`document.getElementById('pb-announce').textContent`);
      await ev(`document.getElementById('view').focus()`);
      await pressKey('End', 'End', 35);
      await pressKey(' ', 'Space', 32);
      await sleep(400);
      A.arrangeKeys = await ev(`({ label: document.getElementById('pb-label').textContent, said: document.getElementById('pb-announce').textContent,
                                   playing: document.getElementById('pb-play').classList.contains('playing'), disabled: document.getElementById('pb-play').disabled })`);
      if (A.arrangeKeys.label !== 't = –' || A.arrangeKeys.said !== said0 || A.arrangeKeys.playing || !A.arrangeKeys.disabled) {
        fail('edit tool after a live solve: the play keys still act on the hidden plan: ' + J(A.arrangeKeys));
      }
      await click('#edit-done');
    });

    // ------------------------------------------------------------ reduced motion: pausing mid-step, then stepping back
    await step('reduced motion', async () => {
      const R = report.reducedMotion = {};
      const p9 = MAPS.find((m) => m.id === 'tor16-fig1-9puzzle') || MAPS[0];
      await send('Emulation.setEmulatedMedia', { features: [{ name: 'prefers-color-scheme', value: 'light' },
                                                             { name: 'prefers-reduced-motion', value: 'reduce' }] });
      try {
        await selectMap(p9);
        await selectPlan('makespan/k1');
        await setControl('pb-speed', '1');
        await click('#pb-reset');
        await click('#pb-play');
        // pause between two steps: t = 1.4 is drawn (and labelled) as step 1
        await waitFor(`/^t = [1-9]/.test(document.getElementById('pb-label').textContent)`, 5000, 'playback past t = 1');
        await sleep(400);
        await click('#pb-play');
        R.paused = await ev(`document.getElementById('pb-label').textContent`);
        await click('#pb-back');
        R.back = await ev(`({ label: document.getElementById('pb-label').textContent, said: document.getElementById('pb-announce').textContent })`);
        const t0 = +R.paused.split(' ')[2], t1 = +R.back.label.split(' ')[2];
        if (!(t0 >= 1) || t1 !== t0 - 1) fail('reduced motion: paused at ' + J(R.paused) + ', one step back shows ' + J(R.back));
      } finally {
        await emulateScheme('light');
        await setControl('pb-speed', '8');
      }
    });

    // ------------------------------------------------------------ no horizontal scroll at phone widths
    const scrollJs = `(() => { const se = document.scrollingElement; const W = window.innerWidth;
      const wide = []; if (se.scrollWidth > W) { for (const e of document.querySelectorAll('body *')) { const r = e.getBoundingClientRect(); if (r.right > W + 0.5 && r.width > 0) { let p = e.parentElement, clipped = false;
        while (p && p !== document.body) { const o = getComputedStyle(p).overflowX; if (o !== 'visible') { clipped = true; break; } p = p.parentElement; }
        if (!clipped) wide.push((e.id ? '#' + e.id : e.nodeName.toLowerCase() + (typeof e.className === 'string' && e.className ? '.' + e.className.split(' ')[0] : '')) + ' right=' + Math.round(r.right)); if (wide.length > 8) break; } } }
      return { scrollWidth: se.scrollWidth, innerWidth: W, ok: se.scrollWidth <= W, wide }; })()`;
    for (const [w, h] of [[360, 740], [390, 844]]) {
      await step('scroll ' + w, async () => {
        await setViewport(w, h, true);
        await sleep(200);
        const variants = [];
        for (const map of MAPS) {
          await selectMap(map);
          variants.push(['map ' + map.id, await ev(scrollJs)]);
        }
        const small = MAPS.find((m) => m.graph.n <= 16 && m.plans['makespan/k1']);
        if (small) {
          await selectMap(small);
          await chooseRadio('netv-full');
          variants.push(['full network ' + small.id, await ev(scrollJs)]);
          await chooseRadio('netv-compact');
          await click('#edit-btn');
          variants.push(['edit mode', await ev(scrollJs)]);
          await chooseRadio('tool-obstacles');
          variants.push(['edit mode, obstacle tool', await ev(scrollJs)]);
          await click('#edit-done');
        }
        for (const [what, r] of variants) {
          report.scroll.push({ width: w, what, scrollWidth: r.scrollWidth, innerWidth: r.innerWidth, ok: r.ok, wide: r.wide.length ? r.wide : undefined });
          if (!r.ok) fail('horizontal scroll at ' + w + ' px (' + what + '): scrollWidth ' + r.scrollWidth + ' > ' + r.innerWidth + (r.wide.length ? ' [' + r.wide.join(', ') + ']' : ''));
        }
      });
    }

    // ------------------------------------------------------------ phone readability (360 px): network width, zoom, goal inset
    await step('phone readability', async () => {
      await setViewport(360, 740, true);
      await sleep(200);
      const R = report.phone = {};
      const nine = MAPS.find((m) => m.id === 'tor16-fig1-9puzzle');
      if (nine) {
        await selectMap(nine);
        await ev(`window.scrollTo(0, 0)`);
        const netJs = `(() => { const e = document.getElementById('net-scroll'), h = document.getElementById('net-hint');
          return { T: +document.getElementById('net-T').value, sw: e.scrollWidth, cw: e.clientWidth, hint: !h.hidden, text: h.textContent }; })()`;
        R.net_lb = await ev(netJs);
        if (R.net_lb.sw > R.net_lb.cw + 1) fail('9-puzzle network at T = ' + R.net_lb.T + ' does not fit 360 px (' + R.net_lb.sw + ' > ' + R.net_lb.cw + ')');
        const hi = await ev(`document.getElementById('net-T').max`);
        await setControl('net-T', hi, 'input');
        R.net_hi = await ev(netJs);
        if (R.net_hi.sw > R.net_hi.cw + 1 && !R.net_hi.hint) fail('network wider than its box at T = ' + R.net_hi.T + ' without a scroll hint');
        await setControl('net-T', String(nine.lower_bounds.makespan), 'input');
      }
      // the shown disc sizes (px): robots in the main view, goals in the inset
      const discJs = (sel) => `(() => { const e = document.querySelector(${J(sel)}); return e ? Math.round(e.getBoundingClientRect().width) : null; })()`;
      const big = MAPS.find((m) => m.id === 'tor16-fig9-24x18-seed1');
      if (big) {
        await selectMap(big);
        R.big = { zoom: await ev(`document.getElementById('view-zoom').value`), robot_px: await ev(discJs('#view-svg .robot circle')) };
        if (!(R.big.robot_px >= 16)) fail('24x18 map at 360 px: robot discs are ' + R.big.robot_px + ' px wide (zoom ' + R.big.zoom + ')');
        // the dimmed-k note must not blame the makespan when the dimmed plans are simply not precomputed
        if (big.plans['total_distance/k4']) {
          await selectPlan('total_distance/k4');
          R.big.note = await ev(`document.getElementById('pick-note').textContent`);
          if (/planned for the makespan/.test(R.big.note)) fail('24x18 total distance: the dimmed-k note says the split is makespan-only: ' + R.big.note);
        }
      }
      const crowded = MAPS.find((m) => m.id === 'crowded-8x8-40-seed1');
      if (crowded) {
        await selectMap(crowded);
        R.crowded = { inset_goal_px: await ev(discJs('#goal-svg .gi-goal circle')), robot_px: await ev(discJs('#view-svg .robot circle')) };
        if (!(R.crowded.inset_goal_px >= 20)) fail('crowded 8x8 at 360 px: goal-inset discs are ' + R.crowded.inset_goal_px + ' px wide');
        // edit mode: the numbers of the badges (a goal under a start) stay readable, and the covering disc names them
        await click('#edit-btn');
        R.crowded.badges = await ev(`(() => { const t = [...document.querySelectorAll('#view-svg .edit-disc.badge text')];
          const a = document.querySelector('#view-svg .edit-disc:not(.passive) title');
          return { n: t.length, min_px: t.length ? Math.min(...t.map((e) => e.getBoundingClientRect().height)) : null, title: a ? a.textContent : null }; })()`);
        if (R.crowded.badges.n && !(R.crowded.badges.min_px >= 8.5)) fail('crowded 8x8 edit mode at 360 px: badge numbers are ' + R.crowded.badges.min_px + ' px tall');
        if (R.crowded.badges.n && !/is on this vertex too/.test(R.crowded.badges.title || '')) fail('crowded 8x8 edit mode: a covering disc does not name its badge: ' + J(R.crowded.badges.title));
        await click('#edit-done');
      }
      if (big) {
        // the 24 x 18 map opens zoomed in; the editing grid starts at "fit"
        await selectMap(big);
        const z0 = await ev(`document.getElementById('view-zoom').value`);
        await click('#edit-btn');
        R.editZoom = { before: z0, after: await ev(`document.getElementById('view-zoom').value`),
                       over: await ev(`(() => { const v = document.getElementById('view'); return v.scrollWidth - v.clientWidth; })()`) };
        if (R.editZoom.after !== '1' || R.editZoom.over > 1) fail('edit mode keeps the zoom of the 24 x 18 map: ' + J(R.editZoom));
        await click('#edit-done');
      }
      // the metrics table (up to five columns before a live solve, Ratio last) fits the card at 360 px; on a
      // narrower screen it scrolls sideways with a hint, since phones hide scroll bars
      const tableJs = `(() => { const e = document.getElementById('metrics-scroll'), h = document.getElementById('metrics-hint');
        return { sw: e.scrollWidth, cw: e.clientWidth, cols: document.querySelectorAll('#metrics thead th').length, hint: !h.hidden }; })()`;
      R.tables = [];
      for (const map of MAPS) {
        await selectMap(map);
        for (const key of ['makespan/k1', 'paper']) {
          if (!map.plans[key]) continue;
          await selectPlan(key);
          const t = Object.assign({ map: map.id, key }, await ev(tableJs));
          R.tables.push(t);
          if (t.sw > t.cw + 1) fail('360 px, ' + map.id + ' ' + key + ': the metrics table (' + t.cols + ' columns) is wider than its card (' + t.sw + ' > ' + t.cw + ')');
        }
      }
      const p25 = MAPS.find((m) => m.id === 'tor16-fig14-25puzzle');
      if (p25 && p25.plans.paper) {
        await setViewport(300, 640, true);
        await sleep(300);
        await selectMap(p25);
        await selectPlan('paper');
        R.table300 = await ev(tableJs);
        if (R.table300.sw > R.table300.cw + 1 && !R.table300.hint) fail('300 px: the metrics table is cut without a hint that it scrolls: ' + J(R.table300));
      }
      await setViewport(1280, 900, false);
    });

    // ------------------------------------------------------------ the "home" ring only once a robot has arrived; k limited by the makespan bound
    await step('home ring and k limits', async () => {
      const R = report.home_k = { mid: [], k: [] };
      const nine = MAPS.find((m) => m.id === 'tor16-fig1-9puzzle');
      if (nine && nine.plans['makespan/k1']) {
        await selectMap(nine);
        await selectPlan('makespan/k1');
        const P = padPaths(nine.plans['makespan/k1'].paths), T = P[0].length - 1;
        for (let i0 = 0; i0 < T; i0++) {
          await setControl('pb-time', String(i0 + 0.5), 'input');
          const home = await ev(`[...document.querySelectorAll('#view-svg .robot:not(.edit-disc)')].map((e) => e.classList.contains('home'))`);
          const bad = home.map((h, r) => (h && !(P[r][i0] === nine.goals[r] && P[r][i0 + 1] === nine.goals[r]) ? r + 1 : 0)).filter(Boolean);
          R.mid.push({ t: i0 + 0.5, bad });
          if (bad.length) fail('at t = ' + (i0 + 0.5) + ' robots ' + bad.join(', ') + ' have the "home" ring while still moving or away');
        }
      }
      const kState = `[1, 2, 4, 8, 16].map((k) => !document.getElementById('k-' + k).disabled)`;
      for (const map of MAPS.filter((m) => m.live && m.lower_bounds.makespan !== null)) {
        await selectMap(map);
        await selectPlan('makespan/k1');
        const on = await ev(kState), lb = map.lower_bounds.makespan;
        const bad = [1, 2, 4, 8, 16].filter((k, j) => k > 1 && k > lb && on[j] && !map.plans['makespan/k' + k]);
        R.k.push({ map: map.id, lb, enabled: on });
        if (bad.length) fail(map.id + ': k = ' + bad.join(', ') + ' enabled although the makespan lower bound is ' + lb);
      }
      // edit mode: the same limit (a 4x4 swap of two corners has makespan lower bound 6)
      await ev(`location.hash = '#edit=4x4&o=&s=0.15&g=15.0'`);
      await waitFor(`!!document.querySelector('#view-svg .edit-disc')`, 5000, 'edit instance from the hash');
      const onE = await ev(kState);
      R.k.push({ map: 'edit 4x4', lb: 6, enabled: onE });
      if (onE[3] || onE[4]) fail('edit mode: k = 8 or 16 enabled on an instance with makespan lower bound 6');
      if (!onE[2]) fail('edit mode: k = 4 disabled on an instance with makespan lower bound 6');
      await click('#edit-done');
      await waitFor(`!document.getElementById('map-pick').hidden`, 5000, 'back to the paper\'s maps');
    });

    // ------------------------------------------------------------ the play controls on one screen with the view; the inset note
    await step('play controls next to the view', async () => {
      const R = report.layout = [];
      const geo = `(() => { const r = (id) => { const b = document.getElementById(id).getBoundingClientRect(); return [Math.round(b.top + scrollY), Math.round(b.bottom + scrollY)]; };
        const v = r('view'), pb = r('playbar'); return { view: v, playbar: pb, span: pb[1] - v[0], H: innerHeight }; })()`;
      for (const [w, h, mobile] of [[1280, 900, false], [390, 844, true], [360, 740, true]]) {
        await setViewport(w, h, mobile);
        await sleep(150);
        for (const map of MAPS) {
          await selectMap(map);
          const g = await ev(geo);
          R.push(Object.assign({ width: w, map: map.id }, g));
          if (g.span > h) fail(w + ' x ' + h + ', ' + map.id + ': the view and the play controls do not fit on one screen (' + g.span + ' px)');
          if (!mobile && g.playbar[1] > h) fail(w + ' x ' + h + ', ' + map.id + ': the play controls are below the first screen (y = ' + g.playbar[1] + ')');
        }
      }
      await setViewport(1280, 900, false);
      // the goal inset's note counts the goal discs hidden at t = 0 (Star swap: both, though only half the vertices hold robots)
      const star = MAPS.find((m) => m.id === 'main-case3-star');
      if (star) {
        await selectMap(star);
        const note = await ev(`document.getElementById('goal-inset').hidden ? null : document.getElementById('goal-inset-note').textContent`);
        report.inset_note = note;
        if (note !== null && !/Robots occupy 2 of the 4 vertices; at t = 0 they hide both goal discs/.test(note)) fail('Star swap goal-inset note: ' + J(note));
      }
    });

    // ------------------------------------------------------------ screenshots: 1280 x 900 and 390 x 844, light and dark
    await step('screenshots', async () => {
      const showcase = MAPS.find((m) => m.id === 'tor16-fig1-9puzzle') || MAPS[0];
      const big = MAPS.find((m) => m.id === 'tor16-fig9-24x18-seed1');
      for (const [name, w, h, mobile] of [['desktop', 1280, 900, false], ['mobile', 390, 844, true]]) {
        await setViewport(w, h, mobile);
        for (const theme of ['light', 'dark']) {
          const cur = await ev(`document.documentElement.getAttribute('data-theme') || (matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light')`);
          if (cur !== theme) await click('#theme-toggle');
          await selectMap(showcase);
          if (showcase.plans['makespan/k2']) await selectPlan('makespan/k2'); else await selectPlan('makespan/k1');
          await click('#pb-reset');
          await click('#pb-fwd'); await click('#pb-fwd');
          await ev(`window.scrollTo(0, 0)`);
          await sleep(150);
          await screenshot(name + '-' + theme, true);
          await screenshot(name + '-' + theme + '-viewport', false);
          if (name === 'desktop') {
            // the highlight layer (the only mark of the highlighted route) against the view's background
            await setControl('hl-select', '0');
            for (const [what, sel] of [['highlight_path', '#view-svg .hl-path'], ['highlight_rings', '#view-svg .hl-start, #view-svg .hl-goal']]) {
              const c = await ev(`${CONTRAST_JS}(${J(sel)}, 'stroke', 'view')`);
              report.theme[what + '_contrast_' + theme] = c;
              if (c === null) fail('no ' + what.replace('_', ' ') + ' drawn after highlighting robot 1 of ' + showcase.id);
              else if (c < 3) fail('the ' + what.replace('_', ' ') + ' has ' + c + ':1 contrast against the view in the ' + theme + ' theme (need 3:1)');
            }
            await setControl('hl-select', '-1');
          }
          if (big) {
            await selectMap(big);
            await selectPlan('makespan/k1');
            await click('#pb-reset');
            for (let i = 0; i < 10; i++) await click('#pb-fwd', { wait: 10 });
            await ev(`document.getElementById('view').scrollIntoView({ block: 'start' })`);
            await sleep(150);
            await screenshot(name + '-' + theme + '-24x18', false);
            if (name === 'desktop') {
              // the graphics the view needs (obstacles, edges, vertices) against the view's background
              for (const [what, sel, prop] of [['obstacles', '#view-svg .g-obstacle', 'fill'], ['edges', '#view-svg .g-edge', 'stroke'], ['vertices', '#view-svg .g-vertex', 'fill']]) {
                const c = await ev(`${CONTRAST_JS}(${J(sel)}, ${J(prop)}, 'view')`);
                report.theme[what + '_contrast_' + theme] = c;
                if (c !== null && c < 3) fail(what + ' have ' + c + ':1 contrast against the view in the ' + theme + ' theme (need 3:1)');
              }
            }
          }
          // the network panel's plan-flow lines (every robot's colour, at the lines' opacity) and move arcs, on the 25-puzzle
          const p25 = MAPS.find((m) => m.id === 'tor16-fig14-25puzzle');
          if (name === 'desktop' && p25) {
            await selectMap(p25);
            await selectPlan('makespan/k1');
            for (const [what, sel] of [['plan_flows', '#net-scroll .n-flow:not(.dim)'], ['move_arcs', '#net-scroll .nt-gadget'], ['time_cursor', '#net-scroll .n-cursor-line']]) {
              const c = await ev(`${CONTRAST_JS}(${J(sel)}, 'stroke', 'net-scroll')`);
              report.theme[what + '_contrast_' + theme] = c;
              if (c === null) fail('no ' + what.replace('_', ' ') + ' drawn in the network panel of ' + p25.id);
              else if (c < 3) fail('the network panel\'s ' + what.replace('_', ' ') + ' have ' + c + ':1 contrast (lowest) in the ' + theme + ' theme (need 3:1)');
            }
          }
          // the Obstacles tool's outlines of the clickable free cells against the view
          if (name === 'desktop') {
            await ev(`location.hash = '#edit=4x4&o=5.6&s=0.15&g=15.0&plan=makespan%2Fk1'`);
            await waitFor(`document.getElementById('view-title').textContent === 'Your instance'`, 5000, 'edited 4 x 4 instance');
            await chooseRadio('tool-obstacles');
            await ev(`document.activeElement && document.activeElement.blur()`);
            const c = await ev(`${CONTRAST_JS}('#view-svg .edit-cell:not(.blocked)', 'stroke', 'view')`);
            report.theme['edit_cells_contrast_' + theme] = c;
            if (c === null) fail('no free-cell outlines drawn in the Obstacles tool');
            else if (c < 3) fail('the Obstacles tool\'s free-cell outlines have ' + c + ':1 contrast against the view in the ' + theme + ' theme (need 3:1)');
            await click('#edit-done');
          }
        }
      }
      // leave the stored theme as light
      await setViewport(1280, 900, false);
      if (await ev(`document.documentElement.getAttribute('data-theme') === 'dark'`)) await click('#theme-toggle');
    });

    await sleep(300);   // late console messages
  } finally {
    if (!(await closeChrome(chrome, cdp, userDir))) log('warning: could not remove the Chrome profile ' + userDir);
    closed = true;
  }
}

function finish(code) {
  report.seconds = Math.round((Date.now() - T0) / 100) / 10;
  const plansOk = report.plans.filter((p) => p.ok).length;
  report.summary = {
    plans_checked: report.plans.length, plans_ok: plansOk,
    data_plans_valid: report.data_checks.filter((d) => d.ok).length + ' / ' + report.data_checks.length,
    live_solves: report.live.map((l) => l.map + ' ' + l.key + ': ' + (l.ok ? 'ok' : 'FAIL') + (l.wall_seconds !== undefined ? ' (' + l.wall_seconds.toFixed(1) + ' s)' : '')),
    console_errors: report.console.filter((c) => c.type === 'error' || c.type === 'assert' || /^log-/.test(c.type)).length,
    exceptions: report.exceptions.length, bad_requests: report.bad_requests.length, failed_loads: report.failed_loads.length,
    scroll_checks: report.scroll.filter((s) => s.ok).length + ' / ' + report.scroll.length + ' without horizontal scroll',
    workers_attached: report.workers_attached || 0,
    screenshots: report.screenshots.length, failures: report.failures.length, warnings: report.warnings.length,
  };
  report.ok = report.failures.length === 0 && code === undefined;
  process.stdout.write(J(report, null, 2) + '\n');
  process.exit(code !== undefined ? code : report.ok ? 0 : 1);
}

main().then(() => finish(), (e) => { report.failures.push({ step: currentStep, message: 'fatal: ' + (e && e.stack || e) }); finish(1); });
