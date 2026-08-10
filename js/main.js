/* ============================================================
 * horstSIM – Hauptcontroller (UI ↔ Engine ↔ Renderer)
 * ============================================================ */

import { SimEngine } from './engine.js';
import { SceneRenderer } from './renderer.js';
import { attachInteraction } from './interaction.js';
import { SCENES } from './scenes.js';
import { mountWizard } from './wizard.js';

const $ = s => document.querySelector(s);
const $$ = s => [...document.querySelectorAll(s)];

let engine, renderer, wizard;
let selectedBody = -1, tracking = false;
let recorder = null, recChunks = [];
let fps = 0, frames = 0, fpsT0 = performance.now();

/* ---------- Boot ---------- */
(async function boot() {
  const ov = $('#loading');
  try {
    engine = await SimEngine.create();
  } catch (e) {
    ov.innerHTML = `<div class="load-card err"><h2>MuJoCo konnte nicht geladen werden</h2><p>${e}</p></div>`;
    return;
  }
  renderer = new SceneRenderer($('#viewport'));
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
  let xml = entry.make ? entry.make() : await (await fetch(entry.url)).text();
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
  $('#visCad').onchange = ev => renderer.robotMesh.setActive(ev.target.checked);
  $('#visShadow').onchange = ev => renderer.setShadows(ev.target.checked);
  $('#visWire').onchange = ev => renderer.setWireframe(ev.target.checked);

  // XML
  $('#xmlApply').onclick = () => loadXML($('#xmleditor').value, 'XML-Editor');
  $('#xmlRevert').onclick = () => { $('#xmleditor').value = engine.xml; $('#xmlerror').classList.remove('show'); };
  $('#xmlDownload').onclick = () => {
    const blob = new Blob([$('#xmleditor').value], { type: 'application/xml' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob); a.download = 'horstSIM_szene.xml'; a.click();
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
  for (const a of acts) {
    const row = document.createElement('div');
    row.className = 'act-row';
    row.innerHTML = `
      <span class="act-name" title="${a.name}">${a.name}</span>
      <input type="range" min="${a.min}" max="${a.max}" step="0.001" value="${a.value}">
      <output>${a.value.toFixed(2)}</output>`;
    const slider = row.querySelector('input'), out = row.querySelector('output');
    slider.oninput = () => { engine.data.ctrl[a.i] = +slider.value; out.textContent = (+slider.value).toFixed(2); };
    slider.dataset.act = a.i;
    host.append(row);
  }
}

function refreshActuatorSliders() {
  $$('#actList input[type=range]').forEach(sl => {
    const i = +sl.dataset.act;
    const v = engine.data.ctrl[i];
    if (document.activeElement !== sl) sl.value = v;
    sl.nextElementSibling.textContent = v.toFixed(2);
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

function onBodySelected(body) {
  selectedBody = body;
  $('#selname').textContent = body > 0 ? engine.bodyName(body) : '–';
  if (tracking && body > 0) engine.cam.trackbodyid = body;
}

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
