/* ============================================================
 * horstSIM – Hauptcontroller (UI ↔ Engine ↔ Renderer)
 * ============================================================ */

import { SimEngine } from './engine.js';
import { SceneRenderer } from './renderer.js';
import { duplicateFreeBody, removeFreeBody } from './sceneedit.js';
import { PickController } from './pick.js';
import { attachInteraction } from './interaction.js';
import { SCENES } from './scenes.js';
import { mountWizard } from './wizard.js';

const $ = s => document.querySelector(s);
const $$ = s => [...document.querySelectorAll(s)];

let engine, renderer, wizard;
let selectedBody = -1, tracking = false;
let copiedName = null, pasteCount = 0;
let pick = null;
let recorder = null, recChunks = [];
let fps = 0, frames = 0, fpsT0 = performance.now();

/* ---------- Boot ---------- */
(async function boot() {
  globalThis.__bootlog?.('Module geladen – Engine wird initialisiert …');
  if (location.protocol === 'file:' && !globalThis.__HORST_WASM) {
    document.body.insertAdjacentHTML('beforeend',
      '<div style="position:fixed;inset:0;z-index:99;display:grid;place-items:center;background:#031e20;color:#e8fbfa;font:600 15px/1.6 system-ui;padding:24px;text-align:center">' +
      'horstSIM kann nicht direkt über file:// laufen (Browser blockiert Module/WASM).<br>' +
      'Bitte per lokalem Server starten – z.&nbsp;B. <code style="color:#5adbd6">npx serve .</code> – ' +
      'oder die Einzeldatei <b>horstsim_single.html</b> verwenden.</div>');
    return;
  }
  const ov = $('#loading');
  try {
    engine = await SimEngine.create();
  } catch (e) {
    ov.innerHTML = `<div class="load-card err"><h2>MuJoCo konnte nicht geladen werden</h2><p>${e}</p></div>`;
    return;
  }
  renderer = new SceneRenderer($('#viewport'));
  pick = new PickController(engine);
  globalThis.horst = { engine, pick };              // Debug-Zugriff (Konsole/Tests)
  addEventListener('error', ev => toast('Fehler: ' + (ev.message || 'siehe Konsole'), true));
  addEventListener('unhandledrejection', () => toast('Fehler: unbehandelte Promise-Ablehnung', true));
  engine.onReload = onModelLoaded;
  attachInteraction(renderer.canvas, engine, { onSelect: onBodySelected });

  $('#mjversion').textContent = 'MuJoCo ' + engine.version;
  buildStaticPanels();
  wizard = mountWizard($('#wizard'), {
    onFinish(xml) { loadXML(xml, 'Assistent'); },
  });

  await loadScene(SCENES[0]);
  ov.classList.add('hide');
  requestAnimationFrame(loop);
  new ResizeObserver(() => renderer.resize()).observe($('#viewport'));
})();

/* ---------- Laden ---------- */
async function loadScene(entry) {
  let xml = entry.make ? entry.make() : (globalThis.__HORST_FILES?.[entry.url] ?? await (await fetch(entry.url)).text());
  loadXML(xml, entry.name);
}

function loadXML(xml, sourceName = '') {
  const res = engine.loadXML(xml);
  const err = $('#xmlerror');
  if (!res.ok) {
    err.textContent = res.error;
    err.classList.add('show');
    toast(`Kompilierfehler – Modell unverändert`, true);
    return false;
  }
  err.classList.remove('show');
  if (sourceName) toast(`Geladen: ${sourceName}`);
  return true;
}

function onModelLoaded() {
  renderer.onModelChanged();
  renderer.robotMesh.rebuild(engine);   // async, Primitive bleiben Fallback
  selectedBody = -1; tracking = false;
  $('#trackBtn').classList.remove('on');
  $('#selname').textContent = '–';
  $('#xmleditor').value = engine.xml;
  buildActuators();
  buildSensors();
  buildSceneTree();
  selectBody(-1);
  pick.configure();
  for (const id of ['pickRotBtn', 'pickBlauBtn', 'pickKugelBtn', 'pickAlleBtn']) $('#' + id).disabled = !pick.ok;
  syncPhysicsInputs();
  syncVisInputs();
  const m = engine.model;
  $('#modelstats').textContent =
    `${m.nbody} Körper · ${m.njnt} Gelenke · ${m.nu} Aktoren · ${m.ngeom} Geome · nq=${m.nq}`;
}

/* ---------- Render-Loop ---------- */
let lastT = performance.now(), statT = 0;
function loop(t) {
  const dt = (t - lastT) / 1000; lastT = t;
  engine.update(dt);
  pick?.tick(dt);
  if (tracking && selectedBody > 0) engine.cam.trackbodyid = selectedBody;
  renderer.render(engine);

  frames++;
  if (t - fpsT0 > 500) { fps = Math.round(frames * 1000 / (t - fpsT0)); frames = 0; fpsT0 = t; }
  statT += dt;
  if (statT > 0.12) { statT = 0; refreshStatus(); }
  requestAnimationFrame(loop);
}

/* ---------- Panels (statisch) ---------- */
function buildStaticPanels() {
  // Szenen
  const sel = $('#sceneSelect');
  SCENES.forEach((s, i) => sel.append(new Option(s.name, i)));
  sel.onchange = () => loadScene(SCENES[+sel.value]);
  $('#wizardBtn').onclick = () => wizard.open();
  $('#fileBtn').onclick = () => $('#fileInput').click();
  $('#fileInput').onchange = async ev => {
    const f = ev.target.files[0]; if (!f) return;
    loadXML(await f.text(), f.name); ev.target.value = '';
  };

  // Drag & Drop
  const vp = $('#viewport');
  vp.addEventListener('dragover', e => { e.preventDefault(); vp.classList.add('drop'); });
  vp.addEventListener('dragleave', () => vp.classList.remove('drop'));
  vp.addEventListener('drop', async e => {
    e.preventDefault(); vp.classList.remove('drop');
    const f = e.dataTransfer.files[0];
    if (f) loadXML(await f.text(), f.name);
  });

  // Transport
  $('#playBtn').onclick = togglePause;
  $('#stepBtn').onclick = () => { engine.singleStep(); };
  $('#resetBtn').onclick = () => { engine.reset(true); };
  $('#speed').oninput = ev => {
    engine.speed = +ev.target.value;
    $('#speedVal').textContent = engine.speed.toFixed(1) + '×';
  };
  $('#camBtn').onclick = () => {
    engine.mujoco.mjv_defaultFreeCamera(engine.model, engine.cam);
    tracking = false; $('#trackBtn').classList.remove('on');
  };
  $('#trackBtn').onclick = () => {
    if (selectedBody <= 0) { toast('Erst Körper per Doppelklick auswählen', true); return; }
    tracking = !tracking;
    $('#trackBtn').classList.toggle('on', tracking);
    const mj = engine.mujoco;
    engine.cam.type = tracking ? mj.mjtCamera.mjCAMERA_TRACKING.value : mj.mjtCamera.mjCAMERA_FREE.value;
    if (tracking) engine.cam.trackbodyid = selectedBody;
  };
  $('#shotBtn').onclick = () => {
    const a = document.createElement('a');
    a.href = renderer.screenshotURL(); a.download = 'horstSIM.png'; a.click();
  };
  $('#recBtn').onclick = toggleRecording;

  // Snapshots
  let snap = null;
  $('#snapSave').onclick = () => { snap = engine.snapshot(); toast('Zustand gespeichert'); };
  $('#snapLoad').onclick = () => {
    if (!engine.restore(snap)) toast('Kein passender Zustand gespeichert', true);
  };

  // Physik
  $$('#panelPhysik [data-phys]').forEach(inp => inp.onchange = applyPhysicsInputs);
  $$('#panelPhysik [data-dsbl]').forEach(inp => inp.onchange = applyPhysicsInputs);

  // Visualisierung
  $$('#panelVis [data-vis]').forEach(inp => inp.onchange = applyVisInputs);
  $('#visFrame').onchange = applyVisInputs;
  $('#pickRotBtn').onclick = () => pick.start('rot');
  $('#pickBlauBtn').onclick = () => pick.start('blau');
  $('#pickKugelBtn').onclick = () => pick.start('kugel');
  $('#pickAlleBtn').onclick = () => pick.start('alle');
  $('#pickStopBtn').onclick = () => pick.stop();
  $('#pickSpeed').oninput = () => {
    const v = +$('#pickSpeed').value;
    if (pick) pick.speed = v;
    $('#pickSpeedVal').textContent = '×' + v.toFixed(1);
  };
  $('#spawnBtn').onclick = () => spawnParts();
  $('#clearBtn').onclick = () => clearLooseParts();
  $('#transToggle').onclick = () => {
    const t = document.querySelector('.transport');
    const min = t.classList.toggle('min');
    const b = $('#transToggle');
    b.textContent = min ? '⌃' : '⌄';
    b.title = min ? 'Leiste ausklappen' : 'Leiste minimieren';
  };
  if (matchMedia('(max-width: 940px)').matches) $('#transToggle').onclick();
  $('#dupBtn').onclick = () => duplicateSelected();
  $('#delBtn').onclick = () => deleteSelected();
  $('#focusBtn').onclick = () => {
    if (selectedBody > 0) {
      const p = selectedBody * 3, xp = engine.data.xpos;
      engine.cam.lookat[0] = xp[p]; engine.cam.lookat[1] = xp[p + 1]; engine.cam.lookat[2] = xp[p + 2];
    }
  };
  window.addEventListener('keydown', onEditKeys);
  $('#visCad').onchange = ev => renderer.robotMesh.setActive(ev.target.checked);
  $('#visShadow').onchange = ev => renderer.setShadows(ev.target.checked);
  $('#visWire').onchange = ev => renderer.setWireframe(ev.target.checked);

  // XML
  $('#xmlApply').onclick = () => loadXML($('#xmleditor').value, 'XML-Editor');
  $('#xmlRevert').onclick = () => { $('#xmleditor').value = engine.xml; $('#xmlerror').classList.remove('show'); };
  $('#xmlDownload').onclick = () => {
    const blob = new Blob([$('#xmleditor').value], { type: 'application/xml' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob); a.download = 'horstOS_szene.xml'; a.click();
    URL.revokeObjectURL(a.href);
  };
  $('#xmleditor').addEventListener('keydown', e => {
    if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') { e.preventDefault(); $('#xmlApply').click(); }
  });

  // Tastatur
  window.addEventListener('keydown', e => {
    if (e.target.tagName === 'TEXTAREA' || e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    if (e.code === 'Space') { e.preventDefault(); togglePause(); }
    else if (e.key === 's' || e.key === 'ArrowRight') engine.singleStep();
    else if (e.key === 'r') engine.reset(true);
  });

  // Panel-Auf/Zuklappen
  $$('.panel > h3').forEach(h => h.onclick = () => h.parentElement.classList.toggle('closed'));
}

function togglePause() {
  engine.paused = !engine.paused;
  $('#playBtn').innerHTML = engine.paused ? '&#9654;' : '&#10074;&#10074;';
  $('#playBtn').title = engine.paused ? 'Fortsetzen (Leertaste)' : 'Pause (Leertaste)';
}

/* ---------- Aktuatoren ---------- */
function buildActuators() {
  const host = $('#actList');
  host.innerHTML = '';
  const acts = engine.listActuators();
  if (!acts.length) { host.innerHTML = '<p class="empty">Modell hat keine Aktoren.</p>'; return; }
  const jByName = new Map(engine.listJoints().map(j => [j.name, j]));
  const R2D = 180 / Math.PI;
  let lastPrefix = null;
  for (const a of acts) {
    const mm = a.name.match(/^(.*)A(\d+)$/);
    const prefix = mm ? mm[1] : null;
    const joint = mm ? jByName.get(`${prefix}j${mm[2]}`) : null;
    if (mm && prefix !== lastPrefix) {
      lastPrefix = prefix;
      const h = document.createElement('div');
      h.className = 'axgroup';
      h.textContent = prefix ? `Roboter ${prefix.replace(/_$/, '')}` : 'HORST600 · Achsen';
      host.append(h);
    }
    const deg = !!joint;
    const min = deg ? a.min * R2D : a.min, max = deg ? a.max * R2D : a.max;
    const val = deg ? a.value * R2D : a.value;
    const row = document.createElement('div');
    row.className = 'act-row';
    row.innerHTML = `
      <span class="act-name" title="${a.name}${deg ? ` · ${min.toFixed(0)}° … ${max.toFixed(0)}°` : ''}">${mm ? 'A' + mm[2] : a.name}</span>
      <input type="range" min="${min.toFixed(2)}" max="${max.toFixed(2)}" step="${deg ? 0.5 : 0.001}" value="${val.toFixed(2)}">
      <output>${val.toFixed(deg ? 1 : 2)}${deg ? '°' : ''}</output>`;
    const slider = row.querySelector('input'), out = row.querySelector('output');
    slider.dataset.act = a.i;
    if (deg) { slider.dataset.deg = '1'; slider.dataset.qadr = joint.qadr; }
    slider.oninput = () => {
      const v = +slider.value, rad = deg ? v / R2D : v;
      engine.data.ctrl[a.i] = rad;
      out.textContent = v.toFixed(deg ? 1 : 2) + (deg ? '°' : '');
      if (engine.paused && joint) {           // im Standbild direkt posieren
        engine.data.qpos[joint.qadr] = rad;
        engine.data.qvel.fill(0);
        engine.mujoco.mj_forward(engine.model, engine.data);
      }
    };
    slider.onpointerdown = () => { slider.dataset.hold = '1'; };
    slider.onpointerup = () => { delete slider.dataset.hold; };
    host.append(row);
  }
}

function refreshActuatorSliders(force = false) {
  const R2D = 180 / Math.PI;
  $$('#actList input[type=range]').forEach(sl => {
    const i = +sl.dataset.act, deg = sl.dataset.deg === '1';
    const qadr = sl.dataset.qadr !== undefined ? +sl.dataset.qadr : -1;
    const ist = deg && qadr >= 0 ? engine.data.qpos[qadr] * R2D : engine.data.ctrl[i];
    if ((force || !sl.dataset.hold) && document.activeElement !== sl) sl.value = ist;
    sl.nextElementSibling.textContent = ist.toFixed(deg ? 1 : 2) + (deg ? '°' : '');
  });
}

/* ---------- Physik ---------- */
function syncPhysicsInputs() {
  const o = engine.model.opt, mj = engine.mujoco;
  $('#pGx').value = o.gravity[0]; $('#pGy').value = o.gravity[1]; $('#pGz').value = o.gravity[2];
  $('#pTs').value = o.timestep;
  $('#pInt').value = o.integrator;
  $('#pIter').value = o.iterations;
  $('#pCone').value = o.cone;
  const d = o.disableflags, B = mj.mjtDisableBit;
  const map = { pdContact: B.mjDSBL_CONTACT, pdLimit: B.mjDSBL_LIMIT, pdAct: B.mjDSBL_ACTUATION, pdFric: B.mjDSBL_FRICTIONLOSS, pdGrav: B.mjDSBL_GRAVITY };
  for (const [id, bit] of Object.entries(map)) { const el = $('#' + id); if (el && bit) el.checked = !!(d & bit.value); }
}

function applyPhysicsInputs() {
  const o = engine.model.opt, mj = engine.mujoco;
  o.gravity[0] = +$('#pGx').value; o.gravity[1] = +$('#pGy').value; o.gravity[2] = +$('#pGz').value;
  o.timestep = Math.min(0.02, Math.max(0.00005, +$('#pTs').value));
  o.integrator = +$('#pInt').value;
  o.iterations = Math.max(1, +$('#pIter').value | 0);
  o.cone = +$('#pCone').value;
  const B = mj.mjtDisableBit;
  let d = 0;
  if ($('#pdContact').checked) d |= B.mjDSBL_CONTACT.value;
  if ($('#pdLimit').checked) d |= B.mjDSBL_LIMIT.value;
  if ($('#pdAct').checked) d |= B.mjDSBL_ACTUATION.value;
  if ($('#pdFric').checked) d |= B.mjDSBL_FRICTIONLOSS.value;
  if ($('#pdGrav').checked) d |= B.mjDSBL_GRAVITY.value;
  o.disableflags = d;
}

/* ---------- Visualisierung ---------- */
const VIS_MAP = {
  vJoint: 'mjVIS_JOINT', vCPoint: 'mjVIS_CONTACTPOINT', vCForce: 'mjVIS_CONTACTFORCE',
  vInertia: 'mjVIS_INERTIA', vCom: 'mjVIS_COM', vPert: 'mjVIS_PERTFORCE',
  vTransp: 'mjVIS_TRANSPARENT', vActuator: 'mjVIS_ACTUATOR', vHull: 'mjVIS_CONVEXHULL',
};

function syncVisInputs() {
  const mj = engine.mujoco, f = engine.vopt.flags;
  for (const [id, name] of Object.entries(VIS_MAP)) {
    const e = mj.mjtVisFlag[name]; const el = $('#' + id);
    if (e && el) el.checked = !!f[e.value];
  }
  $('#visFrame').value = engine.vopt.frame;
}

function applyVisInputs() {
  const mj = engine.mujoco, f = engine.vopt.flags;
  for (const [id, name] of Object.entries(VIS_MAP)) {
    const e = mj.mjtVisFlag[name]; const el = $('#' + id);
    if (e && el) f[e.value] = el.checked ? 1 : 0;
  }
  engine.vopt.frame = +$('#visFrame').value;
}

/* ---------- Sensorik / Status ---------- */
let sensorRows = [];
function buildSensors() {
  const host = $('#sensorList');
  host.innerHTML = '';
  sensorRows = [];
  const sensors = engine.listSensors();
  if (!sensors.length) { host.innerHTML = '<p class="empty">Keine Sensoren im Modell.</p>'; return; }
  for (const s of sensors) {
    const row = document.createElement('div');
    row.className = 'kv';
    row.innerHTML = `<span>${s.name}</span><b>–</b>`;
    host.append(row);
    sensorRows.push({ ...s, out: row.querySelector('b') });
  }
}

function refreshStatus() {
  if (!engine.loaded) return;
  const d = engine.data;
  $('#stTime').textContent = d.time.toFixed(2) + ' s';
  $('#stSteps').textContent = engine.paused ? 'Pause' : engine.stepsPerSecond + ' /s';
  $('#stFps').textContent = fps + ' fps';
  $('#stCon').textContent = d.ncon;
  const E = d.energy;
  $('#stEnergy').textContent = `${E[0].toFixed(2)} | ${E[1].toFixed(2)} J`;
  refreshActuatorSliders();
  if (pick) $('#pickStatus').textContent = pick.status;
  if (selectedBody > 0) {
    const p = selectedBody * 3, xp = d.xpos;
    $('#selInfoPos').textContent = `${xp[p].toFixed(3)}, ${xp[p + 1].toFixed(3)}, ${xp[p + 2].toFixed(3)} m`;
  }

  for (const s of sensorRows) {
    const vals = [];
    for (let k = 0; k < s.dim; k++) vals.push(d.sensordata[s.adr + k].toFixed(3));
    s.out.textContent = vals.join('  ');
  }

  const cons = engine.listContacts(6);
  $('#contactList').innerHTML = cons.length
    ? cons.map(c => `<div class="kv"><span>${c.a} ↔ ${c.b}</span><b>${(c.dist * 1000).toFixed(2)} mm</b></div>`).join('')
    : '<p class="empty">Keine Kontakte.</p>';
}

function isFreeBody(i) {
  if (i <= 0) return false;
  const md = engine.model;
  if (md.body_jntnum[i] !== 1) return false;
  const FREE = engine.mujoco.mjtJoint?.mjJNT_FREE?.value ?? 0;
  return md.jnt_type[md.body_jntadr[i]] === FREE;
}

function selectBody(body) {
  selectedBody = body;
  engine.pert.select = body > 0 ? body : 0;
  $('#selname').textContent = body > 0 ? engine.bodyName(body) : '–';
  $$('#scenetree .trow').forEach(el => el.classList.toggle('sel', +el.dataset.body === body));
  const free = body > 0 && isFreeBody(body);
  $('#selInfoName').textContent = body > 0 ? engine.bodyName(body) : '–';
  $('#selInfoType').textContent = body > 0 ? (free ? 'Freies Objekt' : 'Struktur / Roboter') : '–';
  if (body <= 0) $('#selInfoPos').textContent = '–';
  $('#dupBtn').disabled = !free;
  $('#delBtn').disabled = !free;
  $('#focusBtn').disabled = body <= 0;
  if (tracking && body > 0) engine.cam.trackbodyid = body;
}

function buildSceneTree() {
  const host = $('#scenetree');
  host.innerHTML = '';
  const md = engine.model;
  const grp = t => { const h = document.createElement('div'); h.className = 'tgroup'; h.textContent = t; return h; };
  const row = (body, icon, label, color) => {
    const d = document.createElement('div');
    d.className = 'trow'; d.dataset.body = body;
    d.innerHTML = `<i${color ? ` style="color:${color}"` : ''}>${icon}</i><span title="${label}">${label}</span>`;
    d.onclick = () => selectBody(body);
    return d;
  };
  const robots = [];
  for (let i = 1; i < md.nbody; i++) {
    const n = engine.bodyName(i) || '';
    if (n.endsWith('horst_basis')) robots.push({ base: i, prefix: n.slice(0, -'horst_basis'.length) });
  }
  const robotBodies = new Set();
  for (const r of robots) {
    robotBodies.add(r.base);
    for (let i = 1; i < md.nbody; i++) {
      const n = engine.bodyName(i) || '';
      if (r.prefix ? n.startsWith(r.prefix) : /^link_\d$/.test(n)) robotBodies.add(i);
    }
  }
  if (robots.length) {
    host.append(grp('Roboter'));
    for (const r of robots) host.append(row(r.base, '⛭', `HORST600 ${r.prefix.replace(/_$/, '')}`.trim()));
  }
  const icons = { 2: '●', 3: '▯', 5: '▮', 6: '■' };
  const free = [], stat = [];
  for (let i = 1; i < md.nbody; i++) {
    if (robotBodies.has(i)) continue;
    const gi = md.body_geomadr[i], hasGeom = gi >= 0 && md.body_geomnum[i] > 0;
    const gt = hasGeom ? md.geom_type[gi] : -1;
    const color = hasGeom ? `rgb(${[0, 1, 2].map(k => Math.round(md.geom_rgba[gi * 4 + k] * 255)).join(',')})` : '';
    const item = { i, name: engine.bodyName(i) || `#${i}`, icon: icons[gt] ?? '◆', color };
    (isFreeBody(i) ? free : stat).push(item);
  }
  if (free.length) { host.append(grp(`Objekte (${free.length})`)); for (const o of free) host.append(row(o.i, o.icon, o.name, o.color)); }
  if (stat.length) { host.append(grp('Statisch')); for (const o of stat) host.append(row(o.i, o.icon, o.name, o.color)); }
}

function loadXMLKeepState(xml, qposArr) {
  const ok = loadXML(xml, 'Bearbeitung');
  if (!ok) return false;
  if (qposArr && qposArr.length === engine.data.qpos.length) {
    engine.data.qpos.set(qposArr);
    engine.data.qvel.fill(0);
    engine.mujoco.mj_forward(engine.model, engine.data);
  }
  return true;
}

function bodyIdByName(name) {
  for (let i = 0; i < engine.model.nbody; i++) if (engine.bodyName(i) === name) return i;
  return -1;
}

/** Teile-Spawner: erzeugt Teile aus den Vorlagen in Fallhoehe ueber dem Tisch. */
function clearLooseParts() {
  if (!engine.loaded) return;
  if (pick && pick.phase !== 'idle') pick.stop();
  const md = engine.model;
  const loose = [];
  for (let i = 1; i < md.nbody; i++) {
    if (!isFreeBody(i)) continue;
    loose.push({ name: engine.bodyName(i), qadr: md.jnt_qposadr[md.body_jntadr[i]] });
  }
  if (!loose.length) { toast('Keine losen Teile in der Szene.', true); return; }
  loose.sort((a, b) => b.qadr - a.qadr);            // von hinten löschen → vordere Adressen bleiben gültig
  let xml = engine.xml;
  const q = Array.from(engine.data.qpos);
  try {
    for (const o of loose) { xml = removeFreeBody(xml, o.name, o.qadr); q.splice(o.qadr, 7); }
  } catch (e) { toast('Entfernen: ' + e.message, true); return; }
  copiedName = null;
  if (!loadXMLKeepState(xml, q)) return;
  selectBody(-1);
  toast(loose.length + ' Teile entfernt');
}

function spawnParts() {
  if (!engine.loaded) return;
  const want = [['spawnRot', 'kiste_rot_1'], ['spawnBlau', 'kiste_blau_1'], ['spawnKugel', 'kugel'], ['spawnWuerfel', 'fallwuerfel_1']];
  let xml = engine.xml;
  const q = Array.from(engine.data.qpos);
  let total = 0, made = 0;
  for (const [id, tpl] of want) {
    const n = Math.max(0, Math.min(15, Math.round(+$('#' + id).value || 0)));
    for (let i = 0; i < n && total < 30; i++, total++) {
      const ang = Math.random() * Math.PI * 2, r = 0.16 + Math.random() * 0.40;
      const pos = [
        Math.max(-0.26, Math.min(0.62, Math.cos(ang) * r + 0.18)),
        Math.max(-0.34, Math.min(0.34, Math.sin(ang) * r * 0.8)),
        0.95 + Math.random() * 0.5,
      ];
      let quat = [Math.random() * 2 - 1, Math.random() * 2 - 1, Math.random() * 2 - 1, Math.random() * 2 - 1];
      const nq = Math.hypot(...quat) || 1;
      quat = quat.map(x => x / nq);
      if (quat[0] < 0) quat = quat.map(x => -x);
      let res;
      try { res = duplicateFreeBody(xml, tpl, { pos, quat }); }
      catch (e) { toast('Spawner: ' + e.message, true); return; }
      xml = res.xml;
      q.push(...pos, ...quat);
      made++;
    }
  }
  if (!made) { toast('Stückzahlen wählen.', true); return; }
  if (!loadXMLKeepState(xml, q)) return;
  toast(made + ' Teile abgeworfen');
  if (engine.paused) engine.paused = false;
}

function duplicateSelected(srcName) {
  const name = srcName ?? (selectedBody > 0 ? engine.bodyName(selectedBody) : null);
  const body = name ? bodyIdByName(name) : -1;
  if (body <= 0 || !isFreeBody(body)) { toast('Nur freie Objekte lassen sich duplizieren.', true); return; }
  pasteCount++;
  const d = engine.data, p = body * 3, q = body * 4;
  const off = 0.06 + 0.012 * (pasteCount % 5);
  const pose = {
    pos: [d.xpos[p] + off, d.xpos[p + 1] + off * 0.6, d.xpos[p + 2] + 0.02],
    quat: [d.xquat[q], d.xquat[q + 1], d.xquat[q + 2], d.xquat[q + 3]],
  };
  const oldQ = Array.from(d.qpos);
  let res;
  try { res = duplicateFreeBody(engine.xml, name, pose); }
  catch (e) { toast('Duplizieren: ' + e.message, true); return; }
  if (!loadXMLKeepState(res.xml, [...oldQ, ...pose.pos, ...pose.quat])) return;
  const nb = bodyIdByName(res.newName);
  if (nb > 0) selectBody(nb);
  toast(`„${res.newName}" eingefügt`);
}

function deleteSelected() {
  if (selectedBody <= 0 || !isFreeBody(selectedBody)) { toast('Nur freie Objekte lassen sich löschen.', true); return; }
  const name = engine.bodyName(selectedBody);
  const md = engine.model;
  const qadr = md.jnt_qposadr[md.body_jntadr[selectedBody]];
  const oldQ = Array.from(engine.data.qpos);
  const newQ = [...oldQ.slice(0, qadr), ...oldQ.slice(qadr + 7)];
  let xml2;
  try { xml2 = removeFreeBody(engine.xml, name, qadr); }
  catch (e) { toast('Löschen: ' + e.message, true); return; }
  if (!loadXMLKeepState(xml2, newQ)) return;
  if (copiedName === name) copiedName = null;
  selectBody(-1);
  toast(`„${name}" entfernt`);
}

function onEditKeys(e) {
  const t = e.target;
  if (t && /input|textarea|select/i.test(t.tagName)) return;
  const mod = e.ctrlKey || e.metaKey;
  const k = e.key.toLowerCase();
  if (mod && k === 'c') {
    if (selectedBody > 0 && isFreeBody(selectedBody)) {
      copiedName = engine.bodyName(selectedBody);
      toast(`„${copiedName}" kopiert – Strg+V fügt ein`);
    }
  } else if (mod && k === 'v') {
    if (copiedName) duplicateSelected(copiedName);
  } else if (mod && k === 'd') {
    e.preventDefault();
    duplicateSelected();
  } else if (e.key === 'Delete' || e.key === 'Backspace') {
    if (selectedBody > 0 && isFreeBody(selectedBody)) { e.preventDefault(); deleteSelected(); }
  }
}

function onBodySelected(body) { selectBody(body); }

/* ---------- Aufnahme ---------- */
function toggleRecording() {
  const btn = $('#recBtn');
  if (recorder) {
    recorder.stop();
    return;
  }
  const stream = renderer.captureStream(30);
  const mime = MediaRecorder.isTypeSupported('video/webm;codecs=vp9') ? 'video/webm;codecs=vp9' : 'video/webm';
  recChunks = [];
  recorder = new MediaRecorder(stream, { mimeType: mime, videoBitsPerSecond: 8_000_000 });
  recorder.ondataavailable = e => { if (e.data.size) recChunks.push(e.data); };
  recorder.onstop = () => {
    const blob = new Blob(recChunks, { type: 'video/webm' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob); a.download = 'horstSIM_aufnahme.webm'; a.click();
    URL.revokeObjectURL(a.href);
    recorder = null;
    btn.classList.remove('on'); btn.title = 'Video aufnehmen';
    toast('Aufnahme gespeichert');
  };
  recorder.start();
  btn.classList.add('on'); btn.title = 'Aufnahme beenden';
  toast('Aufnahme läuft …');
}

/* ---------- Toast ---------- */
let toastTimer;
function toast(msg, isErr = false) {
  const t = $('#toast');
  t.textContent = msg;
  t.classList.toggle('err', isErr);
  t.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove('show'), 2600);
}
