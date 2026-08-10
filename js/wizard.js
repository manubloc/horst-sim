/* ============================================================
 * horstSIM – Einrichtungs-Assistent
 * Führt in 5 Schritten zu einer eigenen Simulationszelle und
 * erzeugt daraus vollständiges MJCF.
 * ============================================================ */

import { horstBody, sceneHeader, buildKeyframe, HORST600_HOME, HORST } from './scenes.js';

const DEFAULTS = () => ({
  robots: 1,
  spacing: 0.9,
  mount: 'tisch',            // 'tisch' | 'boden' | 'sockel'
  tray: true,
  objects: { box: 3, sphere: 1, cylinder: 1, size: 0.024, mass: 0.12 },
  physics: { gravity: -9.81, timestep: 0.002, integrator: 'implicitfast' },
  name: 'Meine Zelle',
});

export function generateCellXML(cfg) {
  const phys = cfg.physics || {};
  const h = sceneHeader({
    timestep: phys.timestep ?? 0.002,
    integrator: phys.integrator ?? 'implicitfast',
    gravity: `0 0 ${phys.gravity ?? -9.81}`,
    floorSize: Math.max(3, (cfg.robots ?? 1) * (cfg.spacing ?? 1.4) + 2),
  });

  let world = '', actuators = '', sensors = '';
  const robots = [], freeBodies = [];
  const mountH = cfg.mount === 'boden' ? 0 : cfg.mount === 'sockel' ? 0.30 : 0.37;

  for (let r = 0; r < cfg.robots; r++) {
    const x = (r - (cfg.robots - 1) / 2) * cfg.spacing;
    const prefix = cfg.robots > 1 ? `r${r + 1}_` : '';
    if (cfg.mount === 'tisch') {
      world += `
    <body name="${prefix}tisch" pos="${x + 0.18} 0 0">
      <geom type="box" size="0.42 0.30 0.018" pos="0 0 0.352" rgba="0.20 0.22 0.23 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="0.39 0.27 0.176" rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="0.39 -0.27 0.176" rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="-0.39 0.27 0.176" rgba="0.12 0.13 0.14 1"/>
      <geom type="box" size="0.02 0.02 0.176" pos="-0.39 -0.27 0.176" rgba="0.12 0.13 0.14 1"/>
    </body>`;
    } else if (cfg.mount === 'sockel') {
      world += `
    <geom type="box" size="0.11 0.11 0.15" pos="${x} 0 0.15" rgba="0.12 0.13 0.14 1"/>`;
    }
    const rb = horstBody({ prefix, pos: [x, 0, mountH] });
    world += rb.body;
    actuators += (actuators ? '\n    ' : '') + rb.actuators;
    sensors += rb.sensors;
    robots.push({ home: HORST600_HOME });
  }

  if (cfg.tray) {
    world += `
    <body name="ablage" pos="0.30 0.22 ${mountH + 0.002}">
      <geom type="box" size="0.07 0.07 0.002" rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.07 0.004 0.014" pos="0 0.066 0.012" rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.07 0.004 0.014" pos="0 -0.066 0.012" rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.004 0.07 0.014" pos="0.066 0 0.012" rgba="0.05 0.35 0.36 1"/>
      <geom type="box" size="0.004 0.07 0.014" pos="-0.066 0 0.012" rgba="0.05 0.35 0.36 1"/>
    </body>`;
  }

  const kinds = [
    ['box', cfg.objects.box, s => `size="${s} ${s} ${s}"`, '0.92 0.55 0.15 1'],
    ['sphere', cfg.objects.sphere, s => `size="${s}"`, '0.74 1 0.99 1'],
    ['cylinder', cfg.objects.cylinder, s => `size="${(s * 0.8).toFixed(4)} ${(s * 1.3).toFixed(4)}"`, '0.63 0.63 0.69 1'],
  ];
  let oi = 0;
  for (const [type, count, sizeAttr, rgba] of kinds) {
    for (let k = 0; k < count; k++, oi++) {
      const px = 0.34 + 0.055 * (oi % 3);
      const py = -0.16 + 0.075 * Math.floor(oi / 3);
      const pz = mountH + 0.05 + cfg.objects.size;
      world += `
    <body name="objekt_${type}_${k + 1}" pos="${px} ${py} ${pz}">
      <freejoint/>
      <geom type="${type}" ${sizeAttr(cfg.objects.size)} rgba="${rgba}" mass="${cfg.objects.mass}" friction="0.85 0.004 0.0001"/>
    </body>`;
      freeBodies.push({ pos: [px, py, pz] });
    }
  }

  const key = buildKeyframe(robots, freeBodies);
  return h.open + world + '\n' + h.close(actuators, sensors, key);
}

/* ---------------- UI ---------------- */

export function mountWizard(root, { onFinish }) {
  let cfg = DEFAULTS();
  let step = 0;
  const steps = ['Roboter', 'Montage', 'Objekte', 'Physik', 'Zusammenfassung'];

  function num(label, value, set, { min = 0, max = 10, stepv = 1, unit = '' } = {}) {
    return `<label class="wz-field"><span>${label}</span>
      <input type="number" value="${value}" min="${min}" max="${max}" step="${stepv}" data-set="${set}">
      ${unit ? `<em>${unit}</em>` : ''}</label>`;
  }

  function render() {
    const s = step;
    let body = '';
    if (s === 0) body = `
      <p class="wz-hint">Wie viele HORST600-G2-Roboter soll die Zelle enthalten?</p>
      ${num('Anzahl Roboter', cfg.robots, 'robots', { min: 1, max: 3 })}
      ${num('Abstand', cfg.spacing, 'spacing', { min: 0.6, max: 2, stepv: 0.1, unit: 'm' })}`;
    if (s === 1) body = `
      <p class="wz-hint">Wo wird der Roboter montiert?</p>
      <div class="wz-choices">
        ${[['tisch', 'Auf Arbeitstisch'], ['sockel', 'Auf Sockel'], ['boden', 'Direkt am Boden']]
          .map(([v, t]) => `<button class="wz-choice ${cfg.mount === v ? 'on' : ''}" data-mount="${v}">${t}</button>`).join('')}
      </div>
      <label class="wz-field wz-check"><input type="checkbox" ${cfg.tray ? 'checked' : ''} data-set="tray"><span>Ablageschale hinzufügen</span></label>`;
    if (s === 2) body = `
      <p class="wz-hint">Frei bewegliche Objekte zum Greifen, Schieben und Testen der Kontaktphysik.</p>
      ${num('Kisten', cfg.objects.box, 'objects.box', { max: 8 })}
      ${num('Kugeln', cfg.objects.sphere, 'objects.sphere', { max: 8 })}
      ${num('Zylinder', cfg.objects.cylinder, 'objects.cylinder', { max: 8 })}
      ${num('Größe', cfg.objects.size, 'objects.size', { min: 0.012, max: 0.06, stepv: 0.002, unit: 'm' })}
      ${num('Masse', cfg.objects.mass, 'objects.mass', { min: 0.02, max: 2, stepv: 0.02, unit: 'kg' })}`;
    if (s === 3) body = `
      <p class="wz-hint">Physik-Grundeinstellungen (später jederzeit im Physik-Panel änderbar).</p>
      ${num('Gravitation z', cfg.physics.gravity, 'physics.gravity', { min: -30, max: 0, stepv: 0.01, unit: 'm/s²' })}
      ${num('Zeitschritt', cfg.physics.timestep, 'physics.timestep', { min: 0.0002, max: 0.01, stepv: 0.0002, unit: 's' })}
      <label class="wz-field"><span>Integrator</span>
        <select data-set="physics.integrator">
          ${['implicitfast', 'implicit', 'Euler', 'RK4'].map(o => `<option ${o === cfg.physics.integrator ? 'selected' : ''}>${o}</option>`).join('')}
        </select></label>`;
    if (s === 4) body = `
      <p class="wz-hint">Alles bereit. Die Zelle wird als MJCF erzeugt, in den XML-Editor übernommen und geladen.</p>
      <ul class="wz-summary">
        <li>${cfg.robots}× HORST600 G2, Abstand ${cfg.spacing} m</li>
        <li>Montage: ${cfg.mount}${cfg.tray ? ' · mit Ablageschale' : ''}</li>
        <li>Objekte: ${cfg.objects.box} Kisten, ${cfg.objects.sphere} Kugeln, ${cfg.objects.cylinder} Zylinder</li>
        <li>g = ${cfg.physics.gravity} m/s² · Δt = ${cfg.physics.timestep} s · ${cfg.physics.integrator}</li>
      </ul>`;

    root.innerHTML = `
      <div class="wz-card">
        <div class="wz-head">
          <h2>Einrichtungs-Assistent</h2>
          <button class="wz-close" title="Schließen">×</button>
        </div>
        <ol class="wz-steps">${steps.map((t, i) =>
          `<li class="${i === s ? 'act' : i < s ? 'done' : ''}"><i>${i + 1}</i>${t}</li>`).join('')}</ol>
        <div class="wz-body">${body}</div>
        <div class="wz-foot">
          <button class="btn ghost" data-nav="-1" ${s === 0 ? 'disabled' : ''}>Zurück</button>
          <button class="btn primary" data-nav="1">${s === steps.length - 1 ? 'Zelle erzeugen' : 'Weiter'}</button>
        </div>
      </div>`;

    root.querySelector('.wz-close').onclick = () => { root.classList.remove('open'); };
    root.querySelectorAll('[data-nav]').forEach(b => b.onclick = () => {
      const d = +b.dataset.nav;
      if (step === steps.length - 1 && d > 0) {
        root.classList.remove('open');
        onFinish(generateCellXML(cfg), cfg);
        return;
      }
      step = Math.max(0, step + d); render();
    });
    root.querySelectorAll('[data-mount]').forEach(b => b.onclick = () => { cfg.mount = b.dataset.mount; render(); });
    root.querySelectorAll('[data-set]').forEach(inp => inp.onchange = () => {
      const path = inp.dataset.set.split('.');
      let o = cfg; while (path.length > 1) o = o[path.shift()];
      o[path[0]] = inp.type === 'checkbox' ? inp.checked
        : inp.tagName === 'SELECT' ? inp.value : +inp.value;
    });
  }

  return {
    open() { cfg = DEFAULTS(); step = 0; render(); root.classList.add('open'); },
  };
}
