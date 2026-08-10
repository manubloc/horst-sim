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
  kugel: [0.40, 0.28],   // Wanne mit umlaufendem Rand – hält rollende Kugeln
  rest:  [0.40, -0.28],
};
/* Scanmutti: Kennzahlen der Paketzelle (siehe sceneScanmutti in scenes.js). */
const SCAN = {
  bandZiel: [0.16, -0.34], bandZ: 0.455,          // Ablagepunkt auf dem Förderband
  bandY: -0.34, bandHalbY: 0.11, bandX0: -0.10, bandX1: 0.56,
  bandV: 0.11,                                     // Bandgeschwindigkeit [m/s]
  zone: { yMin: 0.15, yMax: 0.40, xMin: 0.05, xMax: 0.56, zMax: 0.47 },
  spawn: { x: 0.30, y: 0.79, z: 0.70, tilt: 0.48 },
  takt: 1.8,                                       // s zwischen zwei Nachschub-Paketen
};

const HOVER_Z = 0.535;      // sichere Anfahrhöhe (Welt)
const DROP_Z = 0.445;
const SETTLE = 0.12;        // s Nachlauf je Phase

/** Quaternionen-Produkt (w,x,y,z) – lokal, damit pick.js eigenständig bleibt. */
function qMul(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}

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
    this.scanCount = 0; this.flipCount = 0;
    this._nachschub = 0; this._flip = null;
    this._bandDofs = [];
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
    const SPH = gt.mjGEOM_SPHERE?.value ?? 2, CAP = gt.mjGEOM_CAPSULE?.value ?? 3;
    const CYL = gt.mjGEOM_CYLINDER?.value ?? 5, BOX = gt.mjGEOM_BOX?.value ?? 6;
    if (t === SPH) return s[3 * g];
    if (t === CAP) return s[3 * g] + s[3 * g + 1];
    if (t === BOX) {
      // Höhe der gedrehten Box: Σ |R[2][k]|·halb[k] – sonst greift der Roboter
      // bei flach liegenden Paketen weit über dem Deckel ins Leere.
      const m = this.e.data.xmat, o = bodyId * 9;
      return Math.abs(m[o + 2]) * s[3 * g] + Math.abs(m[o + 5]) * s[3 * g + 1] + Math.abs(m[o + 8]) * s[3 * g + 2];
    }
    if (t === CYL) return s[3 * g + 1];
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

  /**
   * Ein Regelschritt fürs Handverfahren (Bahngeschwindigkeitsregelung):
   * verschiebt den TCP um `delta` und hält die Werkzeuglage dabei weich fest.
   *
   * Bewusst KEINE Ziel-IK: die verlangt Konvergenz und liefert an
   * Reichweitengrenzen gar nichts, der Arm bliebe beim Tippen stehen.
   * Ein gedämpfter Jacobi-Schritt bewegt dort stattdessen anteilig weiter.
   *
   * Gerechnet wird auf den SOLLWERTEN `qFrom`, nicht auf der Istlage – sonst
   * hebt der Schleppfehler des Lagereglers die Vorgabe jedes Bild wieder auf.
   */
  jogStep(delta, qFrom) {
    const ik = this.e._ikData;
    if (!ik) return null;
    const q = qFrom ? qFrom.slice() : this.q();
    const fStart = this._fk(ik, q);
    const soll = fStart.ax.slice();                 // aktuelle Werkzeugachse halten
    const ziel = [fStart.p[0] + delta[0], fStart.p[1] + delta[1], fStart.p[2] + delta[2]];
    this._wCur = 0.25;
    // Ein einzelner gedämpfter Schritt liefert wegen der Dämpfung nur einen
    // Bruchteil des Wegs – deshalb bis zum Ziel iterieren (wenige Schritte).
    for (let iter = 0; iter < 14; iter++) {
      const dq = this._jogIter(ik, q, ziel, soll);
      if (!dq) return iter ? q : null;
      const f = this._fk(ik, q);
      if (Math.hypot(ziel[0] - f.p[0], ziel[1] - f.p[1], ziel[2] - f.p[2]) < 0.0002) break;
    }
    return q;
  }

  /** Ein gedämpfter Least-Squares-Schritt; verändert q direkt. */
  _jogIter(ik, q, ziel, soll) {
    const f0 = this._fk(ik, q);
    const err = this._err(f0, ziel, soll);
    const h = 1e-3, lam = 0.02;
    const J = [];
    for (let k = 0; k < 6; k++) {
      const qk = q.slice(); qk[k] += h;
      const ek = this._err(this._fk(ik, qk), ziel, soll);
      J.push(err.map((v, r) => (v - ek[r]) / h));
    }
    const A = Array.from({ length: 6 }, () => new Array(6).fill(0));
    const bv = new Array(6).fill(0);
    for (let a = 0; a < 6; a++) {
      for (let b = 0; b < 6; b++) {
        let s = 0;
        for (let r = 0; r < 6; r++) s += J[a][r] * J[b][r];
        A[a][b] = s;
      }
      let s = 0;
      for (let r = 0; r < 6; r++) s += J[a][r] * err[r];
      bv[a] = s;
      A[a][a] += lam;
    }
    const dq = gauss6(A, bv);
    if (!dq || dq.some(v => !Number.isFinite(v))) return null;
    for (let k = 0; k < 6; k++) {
      q[k] += Math.max(-0.08, Math.min(0.08, dq[k]));
      q[k] = Math.max(this.joints[k].lo + 0.01, Math.min(this.joints[k].hi - 0.01, q[k]));
    }
    return dq;
  }

  /** IK mit Rückfallstufen gegen lokale Minima und Reichweitengrenze. */
  _solveSmart(target) {
    return this.solveIK(target)
        ?? this.solveIK(target, HORST600_HOME.slice())
        ?? this.solveIK(target, undefined, 26)
        ?? this.solveIK(target, HORST600_HOME.slice(), 26);
  }

  /** Freier Körper (genau ein Freigelenk)? */
  _istFrei(i) {
    const md = this.e.model;
    if (md.body_jntnum[i] !== 1) return false;
    const FREE = this.e.mujoco.mjtJoint?.mjJNT_FREE?.value ?? 0;
    return md.jnt_type[md.body_jntadr[i]] === FREE;
  }

  /* ---------- Scanmutti ---------- */

  /** Alle Pakete der Zelle (freie Körper mit Namenspräfix „paket"). */
  _pakete() {
    const e = this.e, out = [];
    for (let i = 1; i < e.model.nbody; i++) {
      const n = e.bodyName(i) || '';
      if (n.startsWith('paket') && this._istFrei(i)) out.push(i);
    }
    return out;
  }

  startScan() {
    if (!this.ok) { this.status = 'nicht bereit'; return; }
    if (!this._pakete().length) { this.status = 'keine Pakete – Scanmutti-Zelle laden'; return; }
    this.stop(true);
    this.scanCount = 0; this.flipCount = 0;
    this.phase = 'scanNext';
    this.status = 'Paketzuführung läuft';
    this.e.onPreStep = () => this._bandAntrieb();   // Band vor jedem Physikschritt antreiben
    if (this.e.paused) this.e.paused = false;
  }

  /** Wird vor JEDEM Physikschritt gerufen. Einmal pro Bild reicht nicht:
   *  die Zwischenschritte dämpfen die Bandgeschwindigkeit weg (gemessen 2,8 statt 11 cm/s). */
  _bandAntrieb() {
    const d = this.e.data;
    for (const dofadr of this._bandDofs) { d.qvel[dofadr] = SCAN.bandV; d.qvel[dofadr + 1] = 0; }
  }

  /** Förderband + Nachschub laufen unabhängig vom Programm. */
  _bandLauf(dt) {
    this._bandDofs.length = 0;
    const e = this.e;
    if (!e.loaded) return;
    const pak = this._pakete();
    if (!pak.length) return;
    const md = e.model, d = e.data;
    const gehalten = e.attachInfo ? e.attachInfo.qadr : -1;
    this._nachschub -= dt;
    let recycelt = false;
    for (const b of pak) {
      const j = md.body_jntadr[b];
      const qadr = md.jnt_qposadr[j], dofadr = md.jnt_dofadr[j];
      if (qadr === gehalten) continue;
      const p = b * 3;
      const x = d.xpos[p], y = d.xpos[p + 1], z = d.xpos[p + 2];
      const aufBand = Math.abs(y - SCAN.bandY) < SCAN.bandHalbY && z > 0.40 && z < 0.55
        && x > SCAN.bandX0 && x < SCAN.bandX1;
      if (aufBand) this._bandDofs.push(dofadr);      // Antrieb läuft vor jedem Physikschritt
      const fertig = (x >= SCAN.bandX1 && Math.abs(y - SCAN.bandY) < 0.2) || z < 0.15;
      if (fertig && !recycelt && this._nachschub <= 0) {
        this._neuAufRampe(qadr, dofadr);
        this._nachschub = SCAN.takt;
        recycelt = true;
      }
    }
  }

  /** Paket oberhalb der Rampe neu einsetzen – jedes Mal etwas anders. */
  _neuAufRampe(qadr, dofadr) {
    const d = this.e.data, s = SCAN.spawn;
    const gier = (Math.random() - 0.5) * 0.9;
    const kipp = s.tilt + (Math.random() < 0.5 ? Math.PI : 0);   // Etikett zufällig oben/unten
    const qz = [Math.cos(gier / 2), 0, 0, Math.sin(gier / 2)];
    const qx = [Math.cos(kipp / 2), Math.sin(kipp / 2), 0, 0];
    const q = qMul(qz, qx);
    d.qpos[qadr] = s.x + (Math.random() - 0.5) * 0.11;
    d.qpos[qadr + 1] = s.y + (Math.random() - 0.5) * 0.04;
    d.qpos[qadr + 2] = s.z + Math.random() * 0.05;
    for (let k = 0; k < 4; k++) d.qpos[qadr + 3 + k] = q[k];
    for (let k = 0; k < 6; k++) d.qvel[dofadr + k] = 0;
  }

  /** Vorderstes greifbares Paket in der Zone vor dem Roboter. */
  _paketInZone() {
    const e = this.e, z = SCAN.zone;
    let best = -1, bestY = 9;
    for (const b of this._pakete()) {
      const p = b * 3, d = e.data;
      const x = d.xpos[p], y = d.xpos[p + 1], zz = d.xpos[p + 2];
      if (y < z.yMin || y > z.yMax || x < z.xMin || x > z.xMax || zz > z.zMax || zz < 0.30) continue;
      if (y < bestY) { bestY = y; best = b; }
    }
    return best;
  }

  /** Etikett oben? Lokale +Z-Achse des Pakets in der Welt. */
  _etikettOben(bodyId) { return this.e.data.xmat[bodyId * 9 + 8] > 0.5; }

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
    // Kugeln zuerst: sie rollen bei jeder Armbewegung weiter und sind sonst
    // nach den Kisten längst aus der Reichweite gerollt (gemessen).
    const order = { kugel: 0, rot: 1, blau: 2, rest: 3 };
    this.queue.sort((a, b) => {
      const ka = order[this._classify(this._body(a))], kb = order[this._classify(this._body(b))];
      return ka !== kb ? ka - kb : this._dist(a) - this._dist(b);
    });
    this.mode = mode;
    this.dropCount = { rot: 0, blau: 0, kugel: 0, rest: 0 };
    this.scanCount = 0; this.flipCount = 0;
    this._nachschub = 0; this._flip = null;
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

  /** Grundstellung anfahren (Manual Control). */
  goHome() {
    if (!this.ok) return;
    this.stop(true);
    if (this.e.paused) this.e.paused = false;
    this.status = 'fahre Home-Pose';
    this._rampTo(HORST600_HOME.slice(), 1.4, 'done');
  }

  stop(silent) {
    this.e.onPreStep = null;
    this._bandDofs.length = 0;
    this.phase = 'idle';
    this._flip = null;
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
    this._bandLauf(dt);
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
        // Weiter gestreutes 3×3-Raster: sonst stoßen sich die Teile auf dem
        // kleinen Feld gegenseitig wieder herunter (gemessen an Rest-Ablagen).
        // Streuung an die Feldgröße koppeln: bei der kleineren Kugelwanne
        // landeten die Teile sonst auf deren Rand und sprangen wieder heraus.
        const sx = key === 'kugel' ? 0.030 : 0.058;
        const sy = key === 'kugel' ? 0.026 : 0.050;
        const off = [(n % 3) * sx - sx, (Math.floor(n / 3) % 3) * sy - sy];
        // Fallhöhe aus der Objektgröße: knapp über der Auflage loslassen statt
        // aus 3–4 cm fallen zu lassen (Hüpfer trieben die Teile vom Feld).
        const b = this._body(this.target);
        const z = 0.379 + (b > 0 ? this._topHalf(b) : 0.024) + 0.030;   // über dem Feldrand freikommen
        this.status = 'lege ab: ' + key;
        this._ikOrSkip([pad[0] + off[0], pad[1] + off[1], z], 1.3, 'release');
        return;
      }
      case 'release': {
        e.releaseBody();
        this.status = `„${this.target}" abgelegt`;
        this.wait = 0.45;                            // kurz stehen bleiben, damit das Teil zur Ruhe kommt
        this.phase = 'next';
        return;
      }
      case 'scanNext': {
        e.releaseBody();
        this._flip = null;
        const b = this._paketInZone();
        if (b < 0) { this.status = `warte auf Nachschub · ${this.scanCount} auf dem Band`; this.wait = 0.35; return; }
        this.targetBody = b;
        this.target = e.bodyName(b);
        const p = b * 3, xp = e.data.xpos;
        this.status = `fahre zu „${this.target}"`;
        this._ikOrSkip([xp[p], xp[p + 1], HOVER_Z], 1.1, 'scanDescend');
        return;
      }
      case 'scanDescend': {
        const b = this._body(this.target);
        if (b < 0) { this.phase = 'scanNext'; return; }
        const p = b * 3, xp = e.data.xpos;
        this.status = `greife „${this.target}"`;
        this._ikOrSkip([xp[p], xp[p + 1], xp[p + 2] + this._topHalf(b) + 0.006], 0.7, 'scanGrab');
        return;
      }
      case 'scanGrab': {
        const b = this._body(this.target);
        if (b < 0) { this.phase = 'scanNext'; return; }
        this._etikettWarOben = this._etikettOben(b);
        e.attachBody(b, this.siteId);
        this.status = this._etikettWarOben
          ? 'Etikett oben ✓ – direkt aufs Band'
          : 'Etikett unten – Paket wird gewendet';
        const s3 = this.siteId * 3;
        this._ikOrSkip([e.data.site_xpos[s3], e.data.site_xpos[s3 + 1], HOVER_Z], 0.7, 'scanPruefen');
        return;
      }
      case 'scanPruefen': {
        if (this._etikettWarOben) { this.phase = 'scanZumBand'; return; }
        if (!e.attachInfo) { this.phase = 'scanNext'; return; }
        this._flip = { t: 0, dur: 1.0 / this.speed, q0: e.attachInfo.relq.slice() };
        this.flipCount++;
        this.phase = 'scanWenden';
        return;
      }
      case 'scanWenden': {
        const f = this._flip;
        if (!f || !e.attachInfo) { this.phase = 'scanZumBand'; return; }
        f.t += dt;
        const s = Math.min(1, f.t / f.dur);
        const a = Math.PI * (s * s * (3 - 2 * s));      // Wendeeinheit dreht um die Greifer-Y-Achse
        e.attachInfo.relq = qMul([Math.cos(a / 2), 0, Math.sin(a / 2), 0], f.q0);
        const stufe = Math.round(s * 12) * 15;
        this.status = `wende „${this.target}" · ${stufe}°`;
        if (s >= 1) { this._flip = null; this.phase = 'scanZumBand'; }
        return;
      }
      case 'scanZumBand': {
        this.status = `lege „${this.target}" aufs Förderband`;
        this._ikOrSkip([SCAN.bandZiel[0], SCAN.bandZiel[1], SCAN.bandZ], 1.2, 'scanAbgeben');
        return;
      }
      case 'scanAbgeben': {
        e.releaseBody();
        this.scanCount++;
        this.status = `${this.scanCount} Pakete auf dem Band · ${this.flipCount}× gewendet`;
        this.wait = 0.3;
        this.phase = 'scanNext';
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
