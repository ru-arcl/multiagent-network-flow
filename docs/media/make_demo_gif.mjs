#!/usr/bin/env node
/*
 * docs/media/make_demo_gif.mjs -- capture the frames of the README demo GIF from the real demo page.
 *
 * Zero dependencies: Node >= 22 (global WebSocket) and a Chrome/Chromium binary. Opens
 * docs/index.html from file:// in headless Chrome (light theme, fixed viewport), drives it over the
 * Chrome DevTools Protocol and saves PNG screenshots of the page's own drawing of each plan:
 *
 *   scene "puzzle": the 9-puzzle of T-RO 2016 Fig. 1 and our optimal 4-step plan (Fig. 3);
 *   scene "net":    the Fig. 4(a) tree with its compact time-expanded network and the plan's flow,
 *                   the time cursor sweeping it; robots coloured as their flows (the page's
 *                   "color robots" option);
 *   scene "split":  the 24 x 18 grid with 50 robots, the exact plan (k = 1, makespan 29) and the
 *                   4-way split plan (k = 4, makespan 35), captured at the same times so that
 *                   make_demo_gif.py can show them side by side.
 *
 * Capture-only changes to the page (nothing in docs/ is modified): the subtitle of each plan
 * (#view-sub) is replaced by a plain one written from docs/data/demo-data.js (objective value and,
 * for the 24 x 18 grid, the build's solve time), set in the text colour; for the network scene the
 * network is drawn 600 CSS pixels wide (so it can fill the GIF's width) and its legend is stacked.
 *
 * Time is set with the page's own time slider (#pb-time, an "input" event, exactly what dragging it
 * does), so the frames are evenly spaced and do not depend on the machine's speed. Every robot
 * position is drawn by docs/js/demo.js from the plans in docs/data/demo-data.js.
 *
 * The page is rendered at a 1280 x 900 CSS viewport and every clip is captured at 2x; the .py
 * downscales. PACE below sets the frames per time step and the playback speed of each scene.
 * The script fails if the page reports an error, or if a plan does not end with every robot on its goal.
 *
 * Writes <out>/<scene>-<clip>-NNNN.png and <out>/frames.json (scenes, clip and element boxes, frame
 * files, durations, the page's time labels).
 * make_demo_gif.py turns them into docs/media/demo.gif; see docs/media/README.md.
 *
 * Usage: node docs/media/make_demo_gif.mjs --out DIR [--chrome PATH] [--probe]
 *   --out DIR   frame directory (required; keep it outside the repository)
 *   --probe     only write a full-page screenshot per scene and print element boxes
 *   env CHROME  alternative to --chrome
 */
import { spawn } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const DOCS = path.resolve(HERE, '..');
const INDEX = path.join(DOCS, 'index.html');

// ------------------------------------------------------------------ arguments
const OPT = { out: null, chrome: process.env.CHROME || null, probe: false };
for (let i = 2; i < process.argv.length; i++) {
  const a = process.argv[i];
  if (a === '--out') OPT.out = path.resolve(process.argv[++i]);
  else if (a === '--chrome') OPT.chrome = process.argv[++i];
  else if (a === '--probe') OPT.probe = true;
  else if (a === '-h' || a === '--help') {
    process.stdout.write(fs.readFileSync(fileURLToPath(import.meta.url), 'utf8').split('*/')[0] + '*/\n');
    process.exit(0);
  } else { process.stderr.write('unknown argument ' + a + '\n'); process.exit(2); }
}
if (!OPT.out) { process.stderr.write('--out DIR is required\n'); process.exit(2); }

const log = (m) => process.stderr.write(m + '\n');
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const J = JSON.stringify;

// ------------------------------------------------------------------ chrome + CDP (from tools/check_demo.mjs)
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
    '--headless=new', '--remote-debugging-port=0', '--user-data-dir=' + userDir,
    '--no-first-run', '--no-default-browser-check', '--disable-extensions', '--disable-background-networking',
    '--disable-component-update', '--disable-sync', '--metrics-recording-only', '--mute-audio',
    '--hide-scrollbars', '--force-color-profile=srgb', '--window-size=1280,900', 'about:blank',
  ];
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
      } else for (const fn of this.listeners) fn(m);
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

let cdp, SID;
const send = (m, p, t) => cdp.send(m, p, SID, t);
const errors = [];

async function ev(expr) {
  const r = await send('Runtime.evaluate', { expression: expr, awaitPromise: true, returnByValue: true, userGesture: true });
  if (r.exceptionDetails) {
    const d = r.exceptionDetails;
    throw new Error('evaluate failed: ' + (d.exception && d.exception.description ? d.exception.description.split('\n')[0] : d.text));
  }
  return r.result.value;
}
async function waitFor(expr, timeoutMs, what) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    const v = await ev(expr);
    if (v) return v;
    await sleep(100);
  }
  throw new Error('timed out waiting for ' + (what || expr));
}
let loadWaiters = [];
async function navigate(url) {
  const loaded = new Promise((res) => { loadWaiters.push(res); });
  await send('Page.navigate', { url });
  await Promise.race([loaded, sleep(20000)]);
  await waitFor(`!!(window.MPP && window.MPP_DEMO && document.readyState === 'complete' && document.querySelector('#view-svg .robot'))`,
                20000, 'demo init');
  await sleep(200);
}

// ------------------------------------------------------------------ page helpers
const PAGE = pathToFileURL(INDEX).href;
const VIEW = { width: 1280, height: 900, scale: 2 };   // CSS viewport; screenshots at 2x, downscaled by the .py

/** Show map `id` with plan `plan` through the page's own URL hash (#map=..&plan=..). */
async function show(id, plan, title) {
  await ev(`(() => { location.hash = ${J('#map=' + encodeURIComponent(id) + '&plan=' + encodeURIComponent(plan))}; })()`);
  await waitFor(`document.getElementById('view-title').textContent === ${J(title)} &&
                 /^t = 0 \\/ /.test(document.getElementById('pb-label').textContent)`, 10000, 'map ' + id + ' ' + plan);
  await sleep(200);
}
/** Set the time with the page's own slider, as dragging it does. */
async function seek(t) {
  await ev(`(() => { const r = document.getElementById('pb-time'); r.value = ${J(String(t))};
                     r.dispatchEvent(new Event('input', { bubbles: true })); })()`);
}
async function rect(sel) {
  return ev(`(() => { const e = document.querySelector(${J(sel)}); if (!e) return null;
                      const b = e.getBoundingClientRect(); return { x: b.left + scrollX, y: b.top + scrollY, width: b.width, height: b.height }; })()`);
}
async function capture(clip, file, scale = 1) {
  const r = await send('Page.captureScreenshot', { format: 'png', captureBeyondViewport: true,
    clip: { x: clip.x, y: clip.y, width: clip.width, height: clip.height, scale } });
  fs.writeFileSync(file, Buffer.from(r.data, 'base64'));
}
async function readout() {
  return ev(`({ label: document.getElementById('pb-label').textContent,
               piece: document.getElementById('pb-piece').hidden ? '' : document.getElementById('pb-piece').textContent,
               title: document.getElementById('view-title').textContent })`);
}
async function fullShot(file) {
  const m = await send('Page.getLayoutMetrics');
  const cs = m.cssContentSize || m.contentSize;
  await capture({ x: 0, y: 0, width: Math.ceil(cs.width), height: Math.ceil(cs.height) }, file);
}

// ------------------------------------------------------------------ main
const SCENES = {
  puzzle: { id: 'tor16-fig1-9puzzle', title: '9-puzzle' },
  net: { id: 'tor16-fig4a-tree', title: 'Two robots on a tree' },
  split: { id: 'tor16-fig9-24x18-seed1', title: '24 × 18 grid, 50 robots' },
};

/** A plan's objective value and solve time from docs/data/demo-data.js (as the page has them). */
async function planFacts(id, plan) {
  return ev(`(() => { const m = MPP_DEMO.maps.find((x) => x.id === ${J(id)}); const p = m.plans[${J(plan)}];
                      return { makespan: p.metrics.makespan, seconds: p.seconds, k: p.k, backend: MPP_DEMO.backend_version }; })()`);
}
const fmtSec = (s) => (s >= 10 ? String(Math.round(s)) : s.toFixed(1)) + ' s';
/** Replace the page's subtitle (and optionally its title) by plain text, in the text colour. */
async function relabel(sub, title) {
  await ev(`(() => { const s = document.getElementById('view-sub'); s.textContent = ${J(sub)};
                     s.style.color = 'var(--text)'; s.style.fontWeight = '600';
                     ${title ? `document.getElementById('view-title').textContent = ${J(title)};` : ''} })()`);
}
/** The network scene's capture-only layout: a 600 px wide network (redrawn by the page's resize handler)
 *  and a stacked legend. */
async function netLayout(on) {
  await ev(`(() => { let st = document.getElementById('gif-style');
                     if (${on} && !st) { st = document.createElement('style'); st.id = 'gif-style';
                       st.textContent = '#net-scroll { width: 600px; } .net-legend { flex-direction: column; width: max-content; }';
                       document.head.appendChild(st); }
                     if (!${on} && st) st.remove();
                     window.dispatchEvent(new Event('resize')); })()`);
  await sleep(600);
}
/** The network legend, for the capture only: the page draws the loopback swatch as one red dashed line,
 *  but with "color robots" each loopback is dashed in its robot's colour. So the swatch shows the two
 *  robots' loopback colours (the page's legend already lists the time cursor, a translucent band with a centre line).
 *  `on = false` restores the page's legend. */
async function netLegend(on) {
  await ev(`(() => { const li = document.querySelector('.net-legend li[data-gif-loop]')
                       || [...document.querySelectorAll('.net-legend li')].find((e) => e.querySelector('.na-loop'));
                     li.setAttribute('data-gif-loop', '');
                     const sw = li.querySelector('svg');
                     if (${on}) {
                       if (window.__gifLoopSwatch === undefined) window.__gifLoopSwatch = sw.innerHTML;
                       const cols = [...document.querySelectorAll('#net-svg .n-flow.loop')].map((e) => e.style.stroke);
                       if (cols.length < 2) throw new Error('netLegend: expected two loopback flows, found ' + cols.length);
                       sw.innerHTML = cols.slice(0, 2).map((c, i) => '<line x1="1" x2="19" y1="' + (2 + 4 * i) + '" y2="' + (2 + 4 * i)
                         + '" style="stroke:' + c + ';stroke-width:2;stroke-dasharray:4 3;fill:none"/>').join('');
                     } else {
                       if (window.__gifLoopSwatch !== undefined) sw.innerHTML = window.__gifLoopSwatch;
                       li.removeAttribute('data-gif-loop');
                     } })()`);
}
/** The page's "color robots" option (robot i in the hue of its flow). */
async function robotColors(on) {
  await ev(`(() => { const c = document.getElementById('opt-colors');
                     if (c.checked !== ${on}) c.click(); })()`);
  await sleep(200);
}

async function main() {
  const bin = findChrome();
  if (!bin) throw new Error('no Chrome binary found (use --chrome or CHROME)');
  fs.mkdirSync(OPT.out, { recursive: true });
  const userDir = fs.mkdtempSync(path.join(os.tmpdir(), 'mppflow-demo-gif-'));
  const chrome = await launchChrome(bin, userDir);
  let closed = false;
  const cleanup = () => {  // last resort on an early exit (shutdown is the normal way)
    if (closed) return;
    try { chrome.proc.kill('SIGKILL'); } catch (e) { /* gone */ }
    try { fs.rmSync(userDir, { recursive: true, force: true }); } catch (e) { /* ignore */ }
  };
  process.on('exit', cleanup);
  shutdown = async () => {
    if (!(await closeChrome(chrome, cdp, userDir))) log('warning: could not remove the Chrome profile ' + userDir);
    closed = true;
  };
  cdp = new CDP(chrome.ws);
  await cdp.opened;
  const { targetId } = await cdp.send('Target.createTarget', { url: 'about:blank' });
  SID = (await cdp.send('Target.attachToTarget', { targetId, flatten: true })).sessionId;
  cdp.on((m) => {
    if (m.sessionId !== SID) return;
    const p = m.params || {};
    if (m.method === 'Page.loadEventFired') { const w = loadWaiters; loadWaiters = []; w.forEach((f) => f()); }
    else if (m.method === 'Runtime.exceptionThrown') errors.push('exception: ' + ((p.exceptionDetails || {}).text || ''));
    else if (m.method === 'Runtime.consoleAPICalled' && p.type === 'error') errors.push('console.error: ' + J((p.args || []).map((a) => a.value)));
  });
  for (const d of ['Page', 'Runtime']) await send(d + '.enable');
  await send('Emulation.setDeviceMetricsOverride', { width: VIEW.width, height: VIEW.height, deviceScaleFactor: 1, mobile: false });
  await send('Emulation.setEmulatedMedia', { features: [{ name: 'prefers-color-scheme', value: 'light' },
                                                         { name: 'prefers-reduced-motion', value: 'no-preference' }] });
  await navigate(PAGE + '#map=' + SCENES.puzzle.id + '&plan=makespan%2Fk1');
  // light theme regardless of a stored preference (a fresh profile has none)
  await ev(`(() => { try { localStorage.removeItem('mppflow-demo-theme'); } catch (e) {} document.documentElement.setAttribute('data-theme', 'light'); })()`);

  if (OPT.probe) {
    for (const [name, sc] of Object.entries(SCENES)) {
      await show(sc.id, 'makespan/k1', sc.title);
      const boxes = {};
      for (const s of PARTS) boxes[s] = await rect(s);
      log(name + ' ' + J(boxes));
      await fullShot(path.join(OPT.out, 'probe-' + name + '.png'));
    }
    return;
  }

  const manifest = { page: 'docs/index.html', scale: VIEW.scale, viewport: [VIEW.width, VIEW.height], scenes: [] };
  // ---- scene 1: the 9-puzzle, our optimal 4-step plan (it is the plan of Fig. 3)
  manifest.scenes.push(await record('puzzle', SCENES.puzzle, 'makespan/k1', {
    times: timeline(4, PACE.puzzle, 1.0, 1.6), clips: { stage: STAGE },
    prepare: () => relabel('exact ILP: makespan 4, the plan of Fig. 3'),
  }));
  // ---- scene 2: the Fig. 4(a) tree and its compact time-expanded network with the plan's flow
  await netLayout(true);
  await robotColors(true);
  manifest.scenes.push(await record('net', SCENES.net, 'makespan/k1', {
    times: timeline(4, PACE.net, 0.8, 2.0),
    clips: { stage: STAGE, net: '#net-scroll', nethead: '#net-h', legend: '.net-legend' },
    prepare: async () => { await relabel('exact ILP: makespan 4, one unit flow per robot'); await netLegend(true); },
  }));
  await netLegend(false);
  await robotColors(false);
  await netLayout(false);
  // ---- scene 3: the 24 x 18 grid, exact (k = 1) and 4-way split (k = 4), at the same times
  const f1 = await planFacts(SCENES.split.id, 'makespan/k1'), f4 = await planFacts(SCENES.split.id, 'makespan/k4');
  const splitTitle = SCENES.split.title + ', solve times with ' + f1.backend;
  const times = timeline(f4.makespan, PACE.split, 1.0, 2.5);   // the k = 4 plan is the longer one: 35 steps
  const k1 = await record('split-k1', SCENES.split, 'makespan/k1', { times, clips: { stage: STAGE },
    prepare: () => relabel('exact ILP (k = 1): makespan ' + f1.makespan + ', ' + fmtSec(f1.seconds), splitTitle) });
  const k4 = await record('split-k4', SCENES.split, 'makespan/k4', { times, clips: { stage: STAGE },
    prepare: () => relabel(f4.k + '-way split: makespan ' + f4.makespan + ', ' + fmtSec(f4.seconds), splitTitle) });
  manifest.scenes.push({ name: 'split', layout: 'split', panels: [k1, k4],
                         frames: k1.frames.map((f, i) => ({ duration: f.duration, t: f.t })) });
  fs.writeFileSync(path.join(OPT.out, 'frames.json'), J(manifest, null, 1) + '\n');
  log('wrote ' + manifest.scenes.reduce((a, s) => a + s.frames.length, 0) + ' frames to ' + OPT.out);
}

/** The element boxes the .py crops (relative to the clip they lie in). */
const PARTS = ['.stage-head', '#view-title', '#view-sub', '#view', '#view-svg rect.bg', '#goal-svg', '#playbar', '.pb-scrub', '.pb-status', '#net-svg', '#net-scroll', '#net-h',
               '#goal-inset figcaption strong', '.net-legend'];
/** The stage clip: from the card's heading to the bottom of the play bar, or of the goal inset (below it) when shown. */
const STAGE = 'stage';

/** Playback pace per scene: frames per time step and time steps per second. The GIF budget (< 1.8 MB) is set
 *  mostly by how many pixels change per frame, so the 50-robot scene uses fewer frames per step. */
const PACE = {
  puzzle: { framesPerStep: 6, stepsPerSecond: 1.25 },
  split: { framesPerStep: 1, stepsPerSecond: 4 },
  net: { framesPerStep: 6, stepsPerSecond: 1.25 },
};

/** Frame times for a plan of `T` steps: a hold at t = 0, the playback, a hold at the end. Each entry is
 *  [t, duration in ms]; the holds are single frames with long durations. */
function timeline(T, { framesPerStep, stepsPerSecond }, holdStart, holdEnd) {
  const n = T * framesPerStep, ms = Math.round(1000 / (framesPerStep * stepsPerSecond));
  const out = [[0, Math.round(holdStart * 1000)]];
  for (let i = 1; i < n; i++) out.push([+(i / framesPerStep).toFixed(4), ms]);
  out.push([T, Math.round(holdEnd * 1000)]);
  return out;
}

async function record(name, sc, plan, { times, clips, prepare }) {
  await show(sc.id, plan, sc.title);
  if (prepare) await prepare();
  const boxes = {};
  for (const s of PARTS) boxes[s] = await rect(s);
  const head = boxes['.stage-head'], bar = boxes['#playbar'];
  const clipRects = {};
  for (const [key, sel] of Object.entries(clips)) {
    const gs = boxes['#goal-svg'], inset = gs && gs.height > 0 ? boxes['#goal-inset figcaption strong'] : null;
    const bottom = Math.max(bar.y + bar.height, gs && gs.height > 0 ? gs.y + gs.height : 0, inset ? inset.y + inset.height : 0);
    const r = sel === STAGE ? { x: head.x - 8, y: head.y - 8, width: head.width + 16, height: bottom + 8 - (head.y - 8) } : boxes[sel];
    clipRects[key] = { x: Math.floor(r.x), y: Math.floor(r.y), width: Math.ceil(r.width), height: Math.ceil(r.height) };
  }
  const frames = [];
  for (let i = 0; i < times.length; i++) {
    const [t, duration] = times[i];
    await seek(t);
    await ev(`new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r)))`);
    const files = {};
    for (const [key, c] of Object.entries(clipRects)) {
      const f = name + '-' + key + '-' + String(i).padStart(4, '0') + '.png';
      await capture(c, path.join(OPT.out, f), VIEW.scale);
      files[key] = f;
    }
    const ro = await readout();
    frames.push({ t, duration, files, label: ro.label, piece: ro.piece });
  }
  // the page must show the plan finished: every robot on its goal (the page's "home" class) at t = T / T
  const end = await ev(`({ home: [...document.querySelectorAll('#view-svg .robot')].every((e) => e.classList.contains('home')),
                           robots: document.querySelectorAll('#view-svg .robot').length })`);
  const last = frames[frames.length - 1].label, m = /^t = (\d+) \/ (\d+)$/.exec(last);
  if (!end.home || !end.robots || !m || m[1] !== m[2]) throw new Error(name + ': the plan does not end with every robot home (' + last + ')');
  const sub = await ev(`document.getElementById('view-sub').textContent`);
  log(name + ': ' + frames.length + ' frames, "' + sub + '", last label "' + frames[frames.length - 1].label + '"');
  return { name, map: sc.id, plan, sub, clips: clipRects, boxes, frames };
}

let shutdown = async () => {};  // set by main once Chrome runs
main().then(async () => {
  await shutdown();
  if (errors.length) { log('page errors:\n  ' + errors.join('\n  ')); process.exit(1); }
  process.exit(0);
}, async (e) => { log('fatal: ' + (e && e.stack || e)); await shutdown(); process.exit(1); });
