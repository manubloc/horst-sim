/* ============================================================
 * horstSIM – Pick & Place
 * Autonome Greifsequenz: Roboter fährt farbige Teile an, saugt
 * sie am TCP an (engine.attachBody) und legt sie auf dem
 * passenden Ablage-Pad ab.
 *
 * Bausteine:
 *  - Numerische IK (Damped Least Squares, Jacobian per finiter
 *    Differenz auf einem MjData-Scratch): Ziel ist die TCP-
 *    Position plus "Werkzeug senkrecht nach unten"
 *    (Site-X-Achse → (0,0,−1), da die Kette entlang X läuft).
 *  - Rampenfahrten über die vorhandenen Positionsaktoren
 *    (ctrl wird smooth interpoliert – echte Physik, kein Teleport).
 *  - Sequencer: hover → absenken → greifen → heben → Pad → ablegen.
 * ============================================================ */

import { HORST600_HOME } from './scenes.js';

const ZIEL = {
  rot:   [0.10, 0.32],
  blau:  [0.10, -0.32],
  kugel: [0.30, 0.22],   // vorhandene Ablage-Wanne fängt rollende Kugeln
  rest:  [0.40, -0.28],
};
const HOVER_Z = 0.535;      // sichere Anfahrhöhe (Welt)
const DROP_Z = 0.445;
const SETTLE = 0.12;        // s Nachlauf je Phase

export class PickController {
  constructor(engine) {
    this.e = engine;
    this.phase = 'idle';
    this.status = 'bereit';
    this.queue = [];
    this.mode = null;
    this.ramp = null;
    this.wait = 0;
    this.dropCount = { rot: 0, blau: 0, kugel: 0, rest: 0 };
    this.speed = 1;
    this.ok = false;
  }

  /** Nach jedem Modell-Load aufrufen. */
  configure() {
    this.stop(true);
    this.ok = false;
    const e = this.e;
    if (!e.loaded) return;
    const mj = e.mujoco, md = e.model;
    let prefix = null;
    for (let i = 1; i < md.nbody; i++) {
      const n = e.bodyName(i) || '';
      if (n.endsWith('horst_basis')) { prefix = n.slice(0, -'horst_basis'.length); break; }
    }
    if (prefix === null) { this.status = 'kein Roboter'; return; }
    this.prefix = prefix;
    this.siteId = mj.mj_name2id(md, mj.mjtObj.mjOBJ_SITE.value, prefix + 'tcp');
    if (this.siteId < 0) { this.status = 'kein TCP'; return; }
    this.joints = [];
    const jmap = new Map(e.listJoints().map(j => [j.name, j]));
    const amap = new Map(e.listActuators().map(a => [a.name, a]));
    for (let n = 1; n <= 6; n++) {
      const j = jmap.get(`${prefix}j${n}`), a = amap.get(`${prefix}A${n}`);
      if (!j || !a) { this.status = 'Achsen unvollständig'; return; }
      this.joints.push({ qadr: j.qadr, ctrl: a.i, lo: md.jnt_range[2 * j.i], hi: md.jnt_range[2 * j.i + 1] });
    }
    this.baseId = mj.mj_name2id(md, mj.mjtObj.mjOBJ_BODY.value, prefix + 'horst_basis');
    this.ok = true;
    this.status = 'bereit';
  }

  q() { return this.joints.map(j => this.e.data.qpos[j.qadr]); }

  /** Sorte eines freien Objekts: rot / blau (Name), kugel (Geometrie), sonst rest. */
  _classify(bodyId) {
    const e = this.e, n = e.bodyName(bodyId) || '';
    if (n.includes('_rot')) return 'rot';
    if (n.includes('_blau')) return 'blau';
    const g = e.model.body_geomadr[bodyId];
    const SPH = e.mujoco.mjtGeom?.mjGEOM_SPHERE?.value ?? 2;
    if (g >= 0 && e.model.geom_type[g] === SPH) return 'kugel';
    return 'rest';
  }

  /** Halbe Hoehe des ersten Geoms (fuer die Greifhoehe ueber der Oberkante). */
  _topHalf(bodyId) {
    const md = this.e.model, g = md.body_geomadr[bodyId];
    if (g < 0) return 0.022;
    const t = md.geom_type[g], s = md.geom_size;
    const gt = this.e.mujoco.mjtGeom ?? {};
    const SPH = gt.mjGEOM_SPHERE?.value ?? 2, CAP = gt.mjGEOM_CAPSULE?.value ?? 3, CYL = gt.mjGEOM_CYLINDER?.value ?? 5;
    if (t === SPH) return s[3 * g];
    if (t === CYL) return s[3 * g + 1];
    if (t === CAP) return s[3 * g] + s[3 * g + 1];
    return s[3 * g + 2];
  }

  /* ---------- IK ---------- */
  _fk(ik, q) {
    const e = this.e;
    ik.qpos.set(e.data.qpos);
    for (let k = 0; k < 6; k++) ik.qpos[this.joints[k].qadr] = q[k];
    e.mujoco.mj_forward(e.model, ik);
    const s3 = this.siteId * 3, s9 = this.siteId * 9;
    return {
      p: [ik.site_xpos[s3], ik.site_xpos[s3 + 1], ik.site_xpos[s3 + 2]],
      ax: [ik.site_xmat[s9], ik.site_xmat[s9 + 3], ik.site_xmat[s9 + 6]],  // Site-X in Welt
    };
  }

  _err(f, target, soll = [0, 0, -1]) {
    // Richtungs-Residuum (soll − ax): überall linear, kein Null-Gradient
    // bei 90°-Abweichung (cross-Fehler versagt dort, a_x ändert nur quadratisch).
    const w = this._wCur ?? 0.35;
    return [
      target[0] - f.p[0], target[1] - f.p[1], target[2] - f.p[2],
      (soll[0] - f.ax[0]) * w, (soll[1] - f.ax[1]) * w, (soll[2] - f.ax[2]) * w,
    ];
  }

  solveIK(target, q0, tiltDeg = 0) {
    // tiltDeg > 0: Werkzeug radial vom Fuß weg neigen – der Roboter
    // "streckt sich" und erreicht ~6–8 cm weiter entfernte Ziele.
    const b3s = this.baseId * 3, xps = this.e.data.xpos;
    const phi = Math.atan2(target[1] - xps[b3s + 1], target[0] - xps[b3s]);
    const t = tiltDeg * Math.PI / 180;
    const soll = [Math.sin(t) * Math.cos(phi), Math.sin(t) * Math.sin(phi), -Math.cos(t)];
    // Stufe 1: Werkzeug senkrecht. Stufe 2 (Fallback): Neigung zulassen,
    // damit auch weit außen liegende Teile erreichbar bleiben (Vakuum hält schräg).
    return this._solve(target, q0, 0.35, true, soll) ?? this._solve(target, q0, 0.10, false, soll);
  }

  _solve(target, q0, oriW, needOri, soll = [0, 0, -1]) {
    this._wCur = oriW;
    const e = this.e;
    const ik = e._ikData;
    if (!ik) return null;
    let q = q0 ? q0.slice() : this.q();
    const h = 1e-3, lam = 0.03;
    for (let iter = 0; iter < 90; iter++) {
      const f0 = this._fk(ik, q);
      const err = this._err(f0, target, soll);
      const epos = Math.hypot(err[0], err[1], err[2]);
      const eori = Math.hypot(err[3], err[4], err[5]);
      if (epos < 0.0015 && (!needOri || eori < 0.02)) return q;
      const J = [];
      for (let k = 0; k < 6; k++) {
        const qk = q.slice(); qk[k] += h;
        const fk = this._fk(ik, qk);
        const ek = this._err(fk, target, soll);
        J.push(err.map((v, r) => (v - ek[r]) / h));   // ∂e/∂q  (Spalte k)
      }
      // Damped Least Squares: (JᵀJ + λI)·dq = Jᵀ·e
      const JT_J = Array.from({ length: 6 }, () => new Array(6).fill(0));
      const JT_e = new Array(6).fill(0);
      for (let a = 0; a < 6; a++) {
        for (let b = 0; b < 6; b++) {
          let s = 0;
          for (let r = 0; r < 6; r++) s += J[a][r] * J[b][r];
          JT_J[a][b] = s;
        }
        let s = 0;
        for (let r = 0; r < 6; r++) s += J[a][r] * err[r];
        JT_e[a] = s;
        JT_J[a][a] += lam;
      }
      const dq = gauss6(JT_J, JT_e);
      if (!dq || dq.some(v => !Number.isFinite(v))) return null;
      for (let k = 0; k < 6; k++) {
        q[k] += Math.max(-0.25, Math.min(0.25, dq[k]));
        q[k] = Math.max(this.joints[k].lo + 0.02, Math.min(this.joints[k].hi - 0.02, q[k]));
      }
    }
    const f = this._fk(ik, q);
    const err = this._err(f, target, soll);
    return !needOri && Math.hypot(err[0], err[1], err[2]) < 0.004 ? q : null;
  }

  /** IK mit Rückfallstufen gegen lokale Minima und Reichweitengrenze. */
  _solveSmart(target) {
    return this.solveIK(target)
        ?? this.solveIK(target, HORST600_HOME.slice())
        ?? this.solveIK(target, undefined, 26)
        ?? this.solveIK(target, HORST600_HOME.slice(), 26);
  }

  /* ---------- Sequenz ---------- */
  start(mode) {
    if (!this.ok) { this.status = 'nicht bereit'; return; }
    const e = this.e;
    const FREE = e.mujoco.mjtJoint?.mjJNT_FREE?.value ?? 0;
    this.queue = [];
    for (let i = 1; i < e.model.nbody; i++) {
      if (e.model.body_jntnum[i] !== 1) continue;
      const j = e.model.body_jntadr[i];
      if (e.model.jnt_type[j] !== FREE) continue;
      const key = this._classify(i);
      if (mode !== 'alle' && key !== mode) continue;
      this.queue.push(e.bodyName(i));
    }
    if (!this.queue.length) { this.status = 'keine passenden Teile gefunden'; return; }
    // sortiert: erst Sorten in fester Reihenfolge, innerhalb nahe Teile zuerst
    const order = { rot: 0, blau: 1, kugel: 2, rest: 3 };
    this.queue.sort((a, b) => {
      const ka = order[this._classify(this._body(a))], kb = order[this._classify(this._body(b))];
      return ka !== kb ? ka - kb : this._dist(a) - this._dist(b);
    });
    this.mode = mode;
    this.dropCount = { rot: 0, blau: 0, kugel: 0, rest: 0 };
    this.phase = 'next';
    this.status = this.queue.length + ' Teile ...';
    if (e.paused) e.paused = false;
  }

  _dist(name) {
    const e = this.e, i = this._body(name);
    if (i < 0) return 9e9;
    const b3 = this.baseId * 3, p = i * 3, xp = e.data.xpos;
    return Math.hypot(xp[p] - xp[b3], xp[p + 1] - xp[b3 + 1]);
  }

  _body(name) {
    for (let i = 0; i < this.e.model.nbody; i++) if (this.e.bodyName(i) === name) return i;
    return -1;
  }

  stop(silent) {
    this.phase = 'idle';
    this.queue = [];
    this.ramp = null;
    this.e.releaseBody?.();
    if (this.ok) for (const j of this.joints) this.e.data.ctrl[j.ctrl] = this.e.data.qpos[j.qadr];
    if (!silent) this.status = 'gestoppt';
  }

  _rampTo(q, dur, nextPhase) {
    this.ramp = { from: this.joints.map(j => this.e.data.ctrl[j.ctrl]), to: q, t: 0, dur: Math.max(0.25, dur / this.speed), next: nextPhase };
    this.phase = 'ramp';
  }

  _ikOrSkip(target, dur, nextPhase) {
    const q = this._solveSmart(target);
    if (!q) { this.status = `„${this.target}" außer Reichweite – übersprungen`; this.e.releaseBody(); this.phase = 'next'; return; }
    this._rampTo(q, dur, nextPhase);
  }

  tick(dt) {
    if (this.phase === 'idle' || !this.ok || !this.e.loaded) return;
    const e = this.e;

    if (this.phase === 'ramp') {
      const r = this.ramp;
      r.t += dt;
      const s = Math.min(1, r.t / r.dur);
      const k = s * s * (3 - 2 * s);                 // smoothstep
      for (let i = 0; i < 6; i++) e.data.ctrl[this.joints[i].ctrl] = r.from[i] + (r.to[i] - r.from[i]) * k;
      if (s >= 1) {
        const errQ = Math.max(...this.joints.map((j, i) => Math.abs(e.data.qpos[j.qadr] - r.to[i])));
        r.settle = (r.settle ?? 0) + dt;
        if (errQ < 0.03 || r.settle > r.dur + 1.2) { this.phase = r.next; this.ramp = null; this.wait = SETTLE; }
      }
      return;
    }
    if (this.wait > 0) { this.wait -= dt; return; }

    switch (this.phase) {
      case 'next': {
        e.releaseBody();
        const name = this.queue.shift();
        if (!name) { this.status = 'zurück zur Home-Pose'; this._rampTo(HORST600_HOME.slice(), 1.4, 'done'); return; }
        const b = this._body(name);
        if (b < 0) { this.phase = 'next'; return; }
        this.target = name;
        this.targetBody = b;
        const p = b * 3, xp = e.data.xpos;
        this.status = `fahre zu „${name}"`;
        this._ikOrSkip([xp[p], xp[p + 1], HOVER_Z], 1.2, 'descend');
        return;
      }
      case 'descend': {
        const b = this._body(this.target);
        if (b < 0) { this.phase = 'next'; return; }
        const p = b * 3, xp = e.data.xpos;
        this.status = `greife „${this.target}"`;
        this._ikOrSkip([xp[p], xp[p + 1], xp[p + 2] + this._topHalf(b) + 0.010], 0.8, 'grab');
        return;
      }
      case 'grab': {
        const b = this._body(this.target);
        this.targetKey = b > 0 ? this._classify(b) : 'rest';
        if (b > 0) e.attachBody(b, this.siteId);
        this.status = `hebe „${this.target}"`;
        const s3 = this.siteId * 3;
        this._ikOrSkip([e.data.site_xpos[s3], e.data.site_xpos[s3 + 1], HOVER_Z], 0.7, 'toPad');
        return;
      }
      case 'toPad': {
        const key = this.targetKey ?? 'rest';
        const pad = ZIEL[key];
        const n = this.dropCount[key]++;
        const off = [(n % 3) * 0.035 - 0.035, (Math.floor(n / 3) % 3) * 0.03 - 0.03];
        this.status = 'lege ab: ' + key;
        this._ikOrSkip([pad[0] + off[0], pad[1] + off[1], DROP_Z], 1.3, 'release');
        return;
      }
      case 'release': {
        e.releaseBody();
        this.status = `„${this.target}" abgelegt`;
        this.wait = 0.25;
        this.phase = 'next';
        return;
      }
      case 'done': {
        this.phase = 'idle';
        this.status = 'fertig ✓';
        return;
      }
    }
  }
}

function gauss6(A, b) {
  // Klassischer Gauss mit Teilpivotisierung + Rücksubstitution.
  // (Die frühere Gauss-Jordan-Variante lieferte mit denselben Zahlen NaN.)
  const n = 6;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[piv][c])) piv = r;
    if (Math.abs(M[piv][c]) < 1e-12) return null;
    [M[c], M[piv]] = [M[piv], M[c]];
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  const x = new Array(n);
  for (let i = n - 1; i >= 0; i--) {
    let s = M[i][n];
    for (let k = i + 1; k < n; k++) s -= M[i][k] * x[k];
    x[i] = s / M[i][i];
  }
  return x;
}
