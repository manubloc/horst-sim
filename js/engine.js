/* ============================================================
 * horstSIM – SimEngine
 * Kapselt die offiziellen MuJoCo-WASM-Bindings (@mujoco/mujoco):
 * Modell laden, Echtzeit-Stepping, Parameter, Snapshots, Perturb.
 * ============================================================ */

import loadMujoco from '../vendor/mujoco/mujoco.js';

/* --- Kleine Quaternion-Helfer (w,x,y,z) für den Greif-Attach --- */
function mat3ToQuat(m, o) {   // m: row-major 9er, o: offset
  const t = m[o] + m[o + 4] + m[o + 8];
  let w, x, y, z;
  if (t > 0) { const s = Math.sqrt(t + 1) * 2; w = s / 4; x = (m[o + 7] - m[o + 5]) / s; y = (m[o + 2] - m[o + 6]) / s; z = (m[o + 3] - m[o + 1]) / s; }
  else if (m[o] > m[o + 4] && m[o] > m[o + 8]) { const s = Math.sqrt(1 + m[o] - m[o + 4] - m[o + 8]) * 2; w = (m[o + 7] - m[o + 5]) / s; x = s / 4; y = (m[o + 1] + m[o + 3]) / s; z = (m[o + 2] + m[o + 6]) / s; }
  else if (m[o + 4] > m[o + 8]) { const s = Math.sqrt(1 + m[o + 4] - m[o] - m[o + 8]) * 2; w = (m[o + 2] - m[o + 6]) / s; x = (m[o + 1] + m[o + 3]) / s; y = s / 4; z = (m[o + 5] + m[o + 7]) / s; }
  else { const s = Math.sqrt(1 + m[o + 8] - m[o] - m[o + 4]) * 2; w = (m[o + 3] - m[o + 1]) / s; x = (m[o + 2] + m[o + 6]) / s; y = (m[o + 5] + m[o + 7]) / s; z = s / 4; }
  return [w, x, y, z];
}
const quatMul = (a, b) => [
  a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
  a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
  a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
  a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
];
const quatInv = (q) => [q[0], -q[1], -q[2], -q[3]];

export class SimEngine {
  static async create() {
    globalThis.__bootlog?.('MuJoCo-WASM wird geladen (≈10 MB, erster Aufruf dauert) …');
    const mujoco = await loadMujoco(globalThis.__HORST_WASM ? { wasmBinary: globalThis.__HORST_WASM } : undefined);
    globalThis.__bootlog?.('Physik-Engine bereit – Szene wird kompiliert …');
    return new SimEngine(mujoco);
  }

  constructor(mujoco) {
    this.attachInfo = null;   // Vakuumgreifer: { qadr, dofadr, siteId, relp, relq }
    this.onPreStep = null;    // zusätzlicher Antrieb vor jedem Schritt (z. B. Förderband)
    this._ikData = null;      // Scratch-MjData für die IK (pick.js)
    this.mujoco = mujoco;
    this.model = null;
    this.data = null;
    this.scene = null;      // MjvScene
    this.vopt = null;       // MjvOption
    this.pert = null;       // MjvPerturb
    this.cam = null;        // MjvCamera
    this.paused = false;
    this.speed = 1.0;       // Echtzeitfaktor
    this.stepsPerSecond = 0;
    this._stepCounter = 0;
    this._stepCountT0 = performance.now();
    this._accum = 0;
    this.xml = '';
    this.onReload = null;   // Callback nach erfolgreichem Laden
  }

  get version() { return this.mujoco.mj_versionString(); }
  get loaded() { return !!this.model; }

  /** MJCF/URDF-String kompilieren. Bei Fehler bleibt das alte Modell aktiv. */
  loadXML(xml) {
    const mj = this.mujoco;
    let model;
    try {
      model = mj.MjModel.from_xml_string(xml);
    } catch (e) {
      return { ok: false, error: String(e?.message ?? e) };
    }
    // Alte Objekte freigeben
    const oldFlags = this.vopt ? Array.from(this.vopt.flags) : null;
    const oldFrame = this.vopt ? this.vopt.frame : null;
    this._freeAll();

    this.model = model;
    this.data = new mj.MjData(model);
    this._ikData?.delete?.();
    this._ikData = new mj.MjData(model);
    this.scene = new mj.MjvScene(model, 4000);
    this.vopt = new mj.MjvOption();
    this.pert = new mj.MjvPerturb();
    this.cam = new mj.MjvCamera();
    mj.mjv_defaultOption(this.vopt);
    mj.mjv_defaultPerturb(this.pert);
    mj.mjv_defaultFreeCamera(model, this.cam);
    if (oldFlags) { oldFlags.forEach((f, i) => { this.vopt.flags[i] = f; }); this.vopt.frame = oldFrame; }

    // Energieberechnung aktivieren (für Statusanzeige)
    this.model.opt.enableflags |= mj.mjtEnableBit.mjENBL_ENERGY.value;

    this.xml = xml;
    this._accum = 0;
    if (this.model.nkey > 0) mj.mj_resetDataKeyframe(this.model, this.data, 0);
    mj.mj_forward(this.model, this.data);
    this.onReload?.();
    return { ok: true };
  }

  _freeAll() {
    this._ikData?.delete?.();
    this._ikData = null;
    this.attachInfo = null;
    for (const k of ['scene', 'vopt', 'pert', 'cam', 'data', 'model']) {
      if (this[k]) { try { this[k].delete(); } catch (_) {} this[k] = null; }
    }
  }

  /** Echtzeit-Stepping mit Akkumulator; Perturbationskräfte werden je Step angewandt. */
  /** Objekt am TCP "ansaugen": relative Pose im Site-Frame merken. */
  attachBody(bodyId, siteId) {
    const md = this.model, d = this.data;
    const j = md.body_jntadr[bodyId];
    const qadr = md.jnt_qposadr[j], dofadr = md.jnt_dofadr[j];
    const s3 = siteId * 3, s9 = siteId * 9, b3 = bodyId * 3, b4 = bodyId * 4;
    const sm = d.site_xmat, sp = d.site_xpos;
    const dx = [d.xpos[b3] - sp[s3], d.xpos[b3 + 1] - sp[s3 + 1], d.xpos[b3 + 2] - sp[s3 + 2]];
    const relp = [                                    // R_site^T · dx
      sm[s9] * dx[0] + sm[s9 + 3] * dx[1] + sm[s9 + 6] * dx[2],
      sm[s9 + 1] * dx[0] + sm[s9 + 4] * dx[1] + sm[s9 + 7] * dx[2],
      sm[s9 + 2] * dx[0] + sm[s9 + 5] * dx[1] + sm[s9 + 8] * dx[2],
    ];
    const qs = mat3ToQuat(sm, s9);
    const relq = quatMul(quatInv(qs), [d.xquat[b4], d.xquat[b4 + 1], d.xquat[b4 + 2], d.xquat[b4 + 3]]);
    this.attachInfo = { qadr, dofadr, siteId, relp, relq };
  }

  releaseBody() { this.attachInfo = null; }

  /** Vor jedem Physikschritt: Greifer-Kopplung und angemeldete Antriebe. */
  _preStep() {
    this._applyAttach();
    if (this.onPreStep) this.onPreStep();
  }

  _applyAttach() {
    const a = this.attachInfo;
    if (!a || !this.loaded) return;
    const d = this.data, s3 = a.siteId * 3, s9 = a.siteId * 9;
    const sm = d.site_xmat, sp = d.site_xpos, r = a.relp;
    d.qpos[a.qadr]     = sp[s3]     + sm[s9] * r[0]     + sm[s9 + 1] * r[1] + sm[s9 + 2] * r[2];
    d.qpos[a.qadr + 1] = sp[s3 + 1] + sm[s9 + 3] * r[0] + sm[s9 + 4] * r[1] + sm[s9 + 5] * r[2];
    d.qpos[a.qadr + 2] = sp[s3 + 2] + sm[s9 + 6] * r[0] + sm[s9 + 7] * r[1] + sm[s9 + 8] * r[2];
    const q = quatMul(mat3ToQuat(sm, s9), a.relq);
    d.qpos[a.qadr + 3] = q[0]; d.qpos[a.qadr + 4] = q[1]; d.qpos[a.qadr + 5] = q[2]; d.qpos[a.qadr + 6] = q[3];
    for (let k = 0; k < 6; k++) d.qvel[a.dofadr + k] = 0;
  }

  update(dtWall) {
    if (!this.loaded || this.paused) return 0;
    const mj = this.mujoco;
    const ts = this.model.opt.timestep;
    this._accum += Math.min(dtWall, 0.25) * this.speed;
    let steps = 0;
    const maxSteps = Math.max(1, Math.ceil((1 / 30) / ts) * 5); // Spiralschutz
    while (this._accum >= ts && steps < maxSteps) {
      this.data.xfrc_applied.fill(0);
      if (this.pert.active) mj.mjv_applyPerturbForce(this.model, this.data, this.pert);
      this._preStep();
      mj.mj_step(this.model, this.data);
      this._accum -= ts;
      steps++;
    }
    if (steps === maxSteps) this._accum = 0;
    this._stepCounter += steps;
    const now = performance.now();
    if (now - this._stepCountT0 > 500) {
      this.stepsPerSecond = Math.round(this._stepCounter * 1000 / (now - this._stepCountT0));
      this._stepCounter = 0; this._stepCountT0 = now;
    }
    return steps;
  }

  /** Einzelschritt (auch im Pausenmodus). */
  singleStep() {
    if (!this.loaded) return;
    this.data.xfrc_applied.fill(0);
    if (this.pert.active) this.mujoco.mjv_applyPerturbForce(this.model, this.data, this.pert);
    this._preStep();
    this.mujoco.mj_step(this.model, this.data);
  }

  /** Bei Pause: Perturbation als Pose anwenden (wie in MuJoCo simulate). */
  applyPausedPerturb() {
    if (!this.loaded || !this.pert.active) return;
    this.mujoco.mjv_applyPerturbPose(this.model, this.data, this.pert, 1);
    this.mujoco.mj_forward(this.model, this.data);
  }

  reset(keyframe = true) {
    if (!this.loaded) return;
    const mj = this.mujoco;
    if (keyframe && this.model.nkey > 0) mj.mj_resetDataKeyframe(this.model, this.data, 0);
    else mj.mj_resetData(this.model, this.data);
    mj.mj_forward(this.model, this.data);
    this._accum = 0;
  }

  snapshot() {
    if (!this.loaded) return null;
    return {
      qpos: Array.from(this.data.qpos),
      qvel: Array.from(this.data.qvel),
      ctrl: Array.from(this.data.ctrl),
      time: this.data.time,
    };
  }

  restore(s) {
    if (!this.loaded || !s || s.qpos.length !== this.data.qpos.length) return false;
    this.data.qpos.set(s.qpos); this.data.qvel.set(s.qvel); this.data.ctrl.set(s.ctrl);
    this.mujoco.mj_forward(this.model, this.data);
    return true;
  }

  /* ---------- Introspektion für die UI ---------- */

  listActuators() {
    const out = [];
    if (!this.loaded) return out;
    // Binding-Bug-Umgehung: bool-Properties (z. B. actuator_ctrllimited) sind als
    // nicht registrierter memory_view<bool> gebunden und werfen beim Zugriff.
    // "limited" wird deshalb aus der ctrlrange abgeleitet (lo < hi).
    const mj = this.mujoco, cr = this.model.actuator_ctrlrange;
    for (let i = 0; i < this.model.nu; i++) {
      const lo = cr[2 * i], hi = cr[2 * i + 1];
      const limited = lo < hi;
      out.push({
        i, name: mj.mj_id2name(this.model, mj.mjtObj.mjOBJ_ACTUATOR.value, i) || `Aktor ${i + 1}`,
        min: limited ? lo : -1, max: limited ? hi : 1,
        value: this.data.ctrl[i],
      });
    }
    return out;
  }

  listJoints() {
    const out = [];
    if (!this.loaded) return out;
    const mj = this.mujoco;
    for (let i = 0; i < this.model.njnt; i++) {
      out.push({
        i, name: mj.mj_id2name(this.model, mj.mjtObj.mjOBJ_JOINT.value, i) || `Gelenk ${i + 1}`,
        type: this.model.jnt_type[i], qadr: this.model.jnt_qposadr[i],
      });
    }
    return out;
  }

  listSensors() {
    const out = [];
    if (!this.loaded) return out;
    for (let i = 0; i < this.model.nsensor; i++) {
      const s = this.model.sensor(i);
      out.push({ i, name: s.name || `Sensor ${i + 1}`, adr: this.model.sensor_adr[i], dim: this.model.sensor_dim[i] });
    }
    return out;
  }

  /** Kontakte (Kopie, max. n Einträge). */
  listContacts(n = 10) {
    const out = [];
    if (!this.loaded || this.data.ncon === 0) return out;
    const vec = this.data.contact;
    try {
      const count = Math.min(this.data.ncon, n);
      for (let i = 0; i < count; i++) {
        const c = vec.get(i);
        const g1 = this.model.geom(c.geom1), g2 = this.model.geom(c.geom2);
        out.push({ a: g1.name || `geom${c.geom1}`, b: g2.name || `geom${c.geom2}`, dist: c.dist });
        c.delete();
      }
    } finally { vec.delete(); }
    return out;
  }

  bodyName(id) {
    if (!this.loaded || id < 0 || id >= this.model.nbody) return '';
    return this.model.body(id).name || `Körper ${id}`;
  }

  dispose() { this._freeAll(); }
}
