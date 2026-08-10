/* ============================================================
 * horstSIM – SimEngine
 * Kapselt die offiziellen MuJoCo-WASM-Bindings (@mujoco/mujoco):
 * Modell laden, Echtzeit-Stepping, Parameter, Snapshots, Perturb.
 * ============================================================ */

import loadMujoco from '../vendor/mujoco/mujoco.js';

export class SimEngine {
  static async create() {
    globalThis.__bootlog?.('MuJoCo-WASM wird geladen (≈10 MB, erster Aufruf dauert) …');
    const mujoco = await loadMujoco(globalThis.__HORST_WASM ? { wasmBinary: globalThis.__HORST_WASM } : undefined);
    globalThis.__bootlog?.('Physik-Engine bereit – Szene wird kompiliert …');
    return new SimEngine(mujoco);
  }

  constructor(mujoco) {
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
    for (const k of ['scene', 'vopt', 'pert', 'cam', 'data', 'model']) {
      if (this[k]) { try { this[k].delete(); } catch (_) {} this[k] = null; }
    }
  }

  /** Echtzeit-Stepping mit Akkumulator; Perturbationskräfte werden je Step angewandt. */
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
    const cr = this.model.actuator_ctrlrange, lim = this.model.actuator_ctrllimited;
    for (let i = 0; i < this.model.nu; i++) {
      out.push({
        i, name: this.model.actuator(i).name || `Aktor ${i + 1}`,
        min: lim[i] ? cr[2 * i] : -1, max: lim[i] ? cr[2 * i + 1] : 1,
        value: this.data.ctrl[i],
      });
    }
    return out;
  }

  listJoints() {
    const out = [];
    if (!this.loaded) return out;
    for (let i = 0; i < this.model.njnt; i++) {
      const j = this.model.joint(i);
      out.push({ i, name: j.name || `Gelenk ${i + 1}`, type: this.model.jnt_type[i], qadr: this.model.jnt_qposadr[i] });
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
