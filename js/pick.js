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

import { HORST600_HOME, SCAN_CONFIG } from './scenes.js';

/* Zielorte der Anwendungen (siehe scenes.js). */
const KASTEN = { rot: [0.16, 0.36], blau: [0.16, -0.36] };
const KASTEN_ABWURF = 0.462;      // TCP fährt in die Kastenöffnung: Kugel wird gelegt, nicht geworfen
                                  // (Rand liegt bei 0,438 m; Flansch bleibt darüber)
const PALETTE = { rot: [0.16, 0.36], blau: [0.16, -0.36] };
const PAL_Z0 = 0.382;             // Oberkante Palettendeck
const WUERFEL_HW = 0.026;         // halbe Würfelkante
const WENDE = [0.420, -0.10];          // Wendetisch
const WENDE_Z0 = 0.386;                // Oberkante des Wendetischs
/* Scanmutti: Kennzahlen der Paketzelle (siehe sceneScanmutti in scenes.js). */
const SCAN = {
  bandZiel: [0.20, -0.34], bandZ: 0.470,           // Ablagepunkt auf dem Förderband
  bandY: -0.34, bandHalbY: 0.12, bandX0: -0.12, bandX1: 1.14,
  bandV: 0.16,                                     // Bandgeschwindigkeit [m/s]
  zone: { yMin: 0.13, yMax: 0.38, xMin: 0.05, xMax: 0.55, zMax: 0.48 },
  spawn: { x: 0.30, y: 0.95, z: 0.80, tilt: 0.42 },
  takt: 2.2,                                       // s zwischen zwei Nachschub-Paketen
  box: [1.42, -0.34], boxMax: 8,                   // Zielbox geradeaus: voll = Programmende
  weicheX: 0.70,                                   // hier verzweigt das Band
  bahnHalb: 0.09,                                  // halbe Breite der Abzweigbahnen
  linksEnde: 0.44,                                 // y, ab dem der Rundlauf greift
  rechtsBox: [0.70, -1.06],                        // Sammelbox der rechten Bahn
  /* Routenwahl nach Größenklasse: kleine Pakete drehen die Runde,
     mittlere gehen geradeaus in die Zielbox, große nach rechts. */
  route: { XS: 'links', S: 'links', M: 'gerade', L: 'rechts' },
};

const HOVER_Z = 0.560;      // sichere Anfahrhöhe (Welt)
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
    this._seq = [];            // Schrittkette des laufenden Programms
    this._planer = null;       // plant den jeweils nächsten Arbeitsgang nach
    this.mode = null;
    this.ramp = null;
    this.wait = 0;
    this.zaehler = 0;
    this.zielName = null;
    this.lagen = { rot: 0, blau: 0 };
    this.scanCount = 0; this.flipCount = 0;
    this._nachschub = 0; this._flip = null;
    this._bandDofs = [];
    this.speed = 1;
    this.ok = false;
  }

  /** Nach jedem Modell-Load aufrufen. */
  configure(wunschPrefix = null) {
    this.stop(true);
    this.ok = false;
    const e = this.e;
    if (!e.loaded) return;
    const mj = e.mujoco, md = e.model;
    let prefix = wunschPrefix;
    if (prefix === null || prefix === undefined) {
      prefix = null;
      for (let i = 1; i < md.nbody; i++) {
        const n = e.bodyName(i) || '';
        if (n.endsWith('horst_basis')) { prefix = n.slice(0, -'horst_basis'.length); break; }
      }
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

  /** sollFest: Werkzeugachse fest vorgeben (z. B. [-1,0,0] = waagerecht in −X).
   *  Ohne Angabe wird sie aus tiltDeg radial zur Basis abgeleitet. */
  solveIK(target, q0, tiltDeg = 0, sollFest = null) {
    // tiltDeg > 0: Werkzeug radial vom Fuß weg neigen – der Roboter
    // "streckt sich" und erreicht ~6–8 cm weiter entfernte Ziele.
    const b3s = this.baseId * 3, xps = this.e.data.xpos;
    const phi = Math.atan2(target[1] - xps[b3s + 1], target[0] - xps[b3s]);
    const t = tiltDeg * Math.PI / 180;
    const soll = sollFest ?? [Math.sin(t) * Math.cos(phi), Math.sin(t) * Math.sin(phi), -Math.cos(t)];
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

  /** IK mit fest vorgegebener Werkzeugneigung (90° = Werkzeugachse waagerecht). */
  _solveTilt(target, tiltDeg) {
    return this.solveIK(target, undefined, tiltDeg)
        ?? this.solveIK(target, HORST600_HOME.slice(), tiltDeg)
        ?? this.solveIK(target, this.q().map((v, i) => v + (i === 4 ? 0.5 : 0)), tiltDeg);
  }

  /** Waagerechte Werkzeugrichtung am Wendetisch (radial vom Fuß weg). */
  _wendeAchse() {
    const b3 = this.baseId * 3, xp = this.e.data.xpos;
    const phi = Math.atan2(WENDE[1] - xp[b3 + 1], WENDE[0] - xp[b3]);
    return [Math.cos(phi), Math.sin(phi)];
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
  /** Wird vor JEDEM Physikschritt gerufen. Einmal pro Bild reicht nicht:
   *  die Zwischenschritte dämpfen die Bandgeschwindigkeit weg (gemessen 2,8 statt 11 cm/s). */
  _bandAntrieb() {
    const d = this.e.data;
    for (const [dofadr, vx, vy] of this._bandDofs) { d.qvel[dofadr] = vx; d.qvel[dofadr + 1] = vy; }
  }

  /** Förderband und Nachschub laufen unabhängig vom Programmablauf. */
  /** Größenklasse aus dem Körpernamen (paket_M_3 → M). */
  _klasse(bodyId) {
    const t = (this.e.bodyName(bodyId) || '').split('_');
    return t.length > 2 ? t[1] : 'M';
  }

  _route(bodyId) {
    if (!SCAN_CONFIG.rundlauf && SCAN.route[this._klasse(bodyId)] === 'links') return 'gerade';
    return SCAN.route[this._klasse(bodyId)] ?? 'gerade';
  }

  _bandLauf(dt) {
    const e = this.e;
    if (!e.loaded) return;
    this._bandDofs.length = 0;
    const md = e.model, d = e.data;
    const gehalten = e.attachments.length ? e.attachments[0].qadr : -1;
    this._nachschub -= dt;
    let recycelt = false;
    for (const b of this._pakete()) {
      const j = md.body_jntadr[b];
      const qadr = md.jnt_qposadr[j], dofadr = md.jnt_dofadr[j];
      if (qadr === gehalten) continue;
      const x = d.xpos[b * 3], y = d.xpos[b * 3 + 1], z = d.xpos[b * 3 + 2];
      const aufHoehe = z > 0.40 && z < 0.60;
      const aufHauptband = aufHoehe && Math.abs(y - SCAN.bandY) < SCAN.bandHalbY
        && x > SCAN.bandX0 && x < SCAN.bandX1;
      const inBahn = aufHoehe && Math.abs(x - SCAN.weicheX) < SCAN.bahnHalb;
      const route = this._route(b);

      if (aufHauptband && (x < SCAN.weicheX - 0.02 || route === 'gerade')) {
        this._bandDofs.push([dofadr, SCAN.bandV, 0]);            // geradeaus (+X)
      } else if (inBahn && route === 'links' && y > SCAN.bandY - 0.02) {
        this._bandDofs.push([dofadr, 0, SCAN.bandV]);            // Abzweig links (+Y)
      } else if (inBahn && route === 'rechts' && y < SCAN.bandY + 0.02) {
        this._bandDofs.push([dofadr, 0, -SCAN.bandV]);           // Abzweig rechts (−Y)
      }

      // Rundlauf: was die linke Bahn durchlaufen hat, startet oben auf der Rampe neu.
      if (route === 'links' && y > SCAN.linksEnde && aufHoehe && !recycelt) {
        this._neuAufRampe(qadr, dofadr);
        this.rundCount = (this.rundCount ?? 0) + 1;
        recycelt = true;
        continue;
      }
      // Sicherheitsnetz: was neben die Zelle fällt, kommt oben auf der Rampe zurück.
      const inKiste = (Math.abs(x - SCAN.box[0]) < 0.19 && Math.abs(y - SCAN.box[1]) < 0.17)
        || (Math.abs(x - SCAN.rechtsBox[0]) < 0.17 && Math.abs(y - SCAN.rechtsBox[1]) < 0.16);
      if (z < 0.12 && !inKiste && !recycelt && this._nachschub <= 0) {
        this._neuAufRampe(qadr, dofadr);
        this._nachschub = SCAN.takt;
        recycelt = true;
      }
    }
  }

  /** Paket oberhalb der Rampe neu einsetzen – jedes Mal etwas anders. */
  _neuAufRampe(qadr, dofadr) {
    const d = this.e.data, s = SCAN.spawn;
    const gier = (Math.random() - 0.5) * 0.8;
    const kipp = s.tilt + (Math.random() < 0.5 ? Math.PI : 0);   // Etikett zufällig oben/unten
    const qz = [Math.cos(gier / 2), 0, 0, Math.sin(gier / 2)];
    const qx = [Math.cos(kipp / 2), Math.sin(kipp / 2), 0, 0];
    const q = qMul(qz, qx);
    d.qpos[qadr] = s.x + (Math.random() - 0.5) * 0.12;
    d.qpos[qadr + 1] = s.y + (Math.random() - 0.5) * 0.05;
    d.qpos[qadr + 2] = s.z + Math.random() * 0.06;
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

  /** Etikett oben? Lokale +Z-Achse des Pakets zeigt in der Welt nach oben. */
  _etikettOben(bodyId) { return this.e.data.xmat[bodyId * 9 + 8] > 0.5; }

  /* ================= Ablaufsteuerung =================
   * Jedes Programm ist eine Kette kleiner Schritte. Zwei Regeln sind fest
   * eingebaut und gelten für alle Anwendungen:
   *   1. Angefahren wird immer aus der Sicherheitshöhe SENKRECHT nach unten.
   *   2. Nach Greifen und nach Loslassen geht es zuerst SENKRECHT nach oben,
   *      erst danach seitlich weiter – so streift nichts an Kastenwänden,
   *      Bandführungen oder Nachbarteilen.
   * Ziele dürfen Funktionen sein; sie werden erst bei der Ausführung
   * ausgewertet, weil sich Teile bis dahin noch bewegen.
   * ==================================================== */

  /** Schrittkette anhängen. */
  _add(...schritte) { this._seq.push(...schritte.flat()); }

  /** Senkrechter Weg nach oben von der aktuellen TCP-Lage. */
  _hoch(z = HOVER_Z, dur = 0.6) {
    return { t: 'move', to: () => { const s = this.siteId * 3, d = this.e.data;
      return [d.site_xpos[s], d.site_xpos[s + 1], z]; }, dur };
  }

  /** Teil aufnehmen: über das Teil, senkrecht runter, saugen, senkrecht hoch. */
  _holen(bodyFn, text) {
    let b = -1;
    return [
      { t: 'call', fn: () => { b = bodyFn(); this.zielName = b > 0 ? this.e.bodyName(b) : '?';
                               this.status = text ? text() : `fahre zu „${this.zielName}"`; } },
      { t: 'move', to: () => { const p = b * 3, d = this.e.data; return [d.xpos[p], d.xpos[p + 1], HOVER_Z]; }, dur: 1.1 },
      { t: 'move', to: () => { const p = b * 3, d = this.e.data;
                               return [d.xpos[p], d.xpos[p + 1], d.xpos[p + 2] + this._topHalf(b) + 0.010]; }, dur: 0.7 },
      { t: 'grab', body: () => b },
      this._hoch(HOVER_Z, 0.7),
    ];
  }

  /** Teil ablegen: über den Zielpunkt, senkrecht runter, loslassen, senkrecht hoch. */
  _ablegen(zielFn, abwurfZ, text) {
    return [
      { t: 'call', fn: () => { if (text) this.status = text(); } },
      { t: 'move', to: () => { const z = zielFn(); return [z[0], z[1], HOVER_Z]; }, dur: 1.2 },
      { t: 'move', to: () => { const z = zielFn(); return [z[0], z[1], typeof abwurfZ === 'function' ? abwurfZ() : abwurfZ]; }, dur: 0.8 },
      { t: 'release' },
      { t: 'wait', s: 0.22 },
      this._hoch(HOVER_Z, 0.7),
    ];
  }

  /* ---------- Programm 1: Kugeln in die Kästen ---------- */
  startKugeln(farbe) {
    if (!this._bereit()) return;
    this._neu('kugeln');
    this.farbe = farbe;                              // 'rot' | 'blau' | 'alle'
    this.status = 'Kugeln sortieren';
    this._planer = () => this._planKugel();
    this._planer();
  }

  _kugeln(farbe) {
    const out = [];
    for (let i = 1; i < this.e.model.nbody; i++) {
      const n = this.e.bodyName(i) || '';
      if (!n.startsWith('kugel_') || !this._istFrei(i)) continue;
      const f = n.includes('_rot') ? 'rot' : 'blau';
      if (farbe !== 'alle' && f !== farbe) continue;
      if (this._imKasten(i, f)) continue;             // liegt schon richtig
      out.push({ i, f });
    }
    const b3 = this.baseId * 3, xp = this.e.data.xpos;
    out.sort((a, b) => Math.hypot(xp[a.i * 3] - xp[b3], xp[a.i * 3 + 1] - xp[b3 + 1])
                     - Math.hypot(xp[b.i * 3] - xp[b3], xp[b.i * 3 + 1] - xp[b3 + 1]));
    return out;
  }

  _imKasten(bodyId, farbe) {
    const k = KASTEN[farbe], p = bodyId * 3, d = this.e.data;
    return Math.abs(d.xpos[p] - k[0]) < 0.10 && Math.abs(d.xpos[p + 1] - k[1]) < 0.085;
  }

  _planKugel() {
    const rest = this._kugeln(this.farbe);
    if (!rest.length) { this._abschluss(`fertig ✓ · ${this.zaehler} Kugeln sortiert`); return; }
    const { i, f } = rest[0];
    this.zaehler++;
    this._add(
      this._holen(() => i, () => `hole ${f === 'rot' ? 'rote' : 'blaue'} Kugel (${rest.length} offen)`),
      // Abwurf deutlich über der Kastenoberkante: der Greifer bleibt außerhalb,
      // die Kugel fällt die letzten Zentimeter selbst hinein.
      this._ablegen(() => this._kastenPunkt(f), KASTEN_ABWURF, () => `lege in Kasten ${f}`),
    );
  }

  /** Streupunkt im Kasten, damit die Kugeln sich verteilen statt zu stapeln. */
  _kastenPunkt(farbe) {
    const k = KASTEN[farbe], n = this.zaehler;
    // Streuung bewusst eng: der Flansch (r ≈ 32 mm) muss im lichten Innenmaß
    // (95 × 80 mm) bleiben, wenn der Greifer in den Kasten eintaucht.
    return [k[0] + ((n % 3) - 1) * 0.042, k[1] + (((n / 3) | 0) % 2 ? 0.028 : -0.028)];
  }

  /* ---------- Programm 2: Würfel palettieren ---------- */
  startPalettieren(farbe) {
    if (!this._bereit()) return;
    this._neu('palette');
    this.farbe = farbe;
    this.lagen = { rot: 0, blau: 0 };
    this.status = 'Palettieren';
    this._planer = () => this._planPalette();
    this._planer();
  }

  _wuerfel(farbe) {
    const out = [];
    for (let i = 1; i < this.e.model.nbody; i++) {
      const n = this.e.bodyName(i) || '';
      if (!n.startsWith('wuerfel_') || !this._istFrei(i)) continue;
      const f = n.includes('_rot') ? 'rot' : 'blau';
      if (farbe !== 'alle' && f !== farbe) continue;
      const p = i * 3, d = this.e.data;
      const pal = PALETTE[f];
      if (Math.abs(d.xpos[p] - pal[0]) < 0.11 && Math.abs(d.xpos[p + 1] - pal[1]) < 0.10) continue;
      out.push({ i, f });
    }
    const b3 = this.baseId * 3, xp = this.e.data.xpos;
    out.sort((a, b) => Math.hypot(xp[a.i * 3] - xp[b3], xp[a.i * 3 + 1] - xp[b3 + 1])
                     - Math.hypot(xp[b.i * 3] - xp[b3], xp[b.i * 3 + 1] - xp[b3 + 1]));
    return out;
  }

  _planPalette() {
    const rest = this._wuerfel(this.farbe);
    if (!rest.length) { this._abschluss(`fertig ✓ · ${this.zaehler} Würfel palettiert`); return; }
    const { i, f } = rest[0];
    const n = this.lagen[f]++;                        // laufende Nummer je Palette
    const platz = n % 4, lage = (n / 4) | 0;
    const dx = (platz % 2 ? 1 : -1) * 0.029;
    const dy = (platz < 2 ? 1 : -1) * 0.029;
    const pal = PALETTE[f];
    this.zaehler++;
    // Ablagehöhe wächst mit jeder Lage; 12 mm Luft, dann fällt der Würfel sauber auf.
    const z = () => PAL_Z0 + lage * (2 * WUERFEL_HW) + WUERFEL_HW + 0.030;
    this._add(
      this._holen(() => i, () => `palettiere ${f} · Lage ${lage + 1}, Platz ${platz + 1}`),
      this._ablegen(() => [pal[0] + dx, pal[1] + dy], z, () => `setze auf Palette ${f}`),
    );
  }

  /* ---------- Programm 3: Scanmutti ---------- */
  startScan() {
    if (!this._bereit()) return;
    if (!this._pakete().length) { this.status = 'keine Pakete – Scanmutti-Zelle laden'; return; }
    this._neu('scan');
    this.scanCount = 0; this.flipCount = 0;
    this.status = 'Paketzuführung läuft';
    this.e.onPreStep = () => this._bandAntrieb();
    this._planer = () => this._planScan();
    this._planer();
  }

  _planScan() {
    if (this._boxVoll()) { this._abschluss(`Zielbox voll ✓ · ${this.scanCount} Pakete verpackt`); return; }
    const b = this._paketInZone();
    if (b < 0) {                                      // Rampe braucht einen Moment
      this.status = `warte auf Nachschub · ${this.scanCount} im Karton`;
      this._add({ t: 'wait', s: 0.4 }, { t: 'call', fn: () => this._planer() });
      return;
    }
    this._bearbeite(b, 0);
  }

  /**
   * Ein Paket abarbeiten. Liegt das Etikett unten, kann ein Sauggreifer es
   * nicht in der Hand wenden: das Paket wird mit Überstand auf die Wendekante
   * gelegt, kippt dort über die Kante und wird anschließend an seiner neuen
   * Oberseite wieder gegriffen – so oft, bis das Etikett oben liegt.
   */
  _bearbeite(b, versuch) {
    this._add(this._holen(() => b, () => versuch
      ? `greife „${this.e.bodyName(b)}" neu (Wendung ${versuch})`
      : `greife „${this.e.bodyName(b)}"`));
    this._add({ t: 'call', fn: () => {
      if (this._etikettOben(b) || versuch >= 6) {
        this.status = this._etikettOben(b) ? 'Etikett oben ✓ – aufs Band' : 'Wendung erschöpft – trotzdem aufs Band';
        this._add(this._ablegen(() => this._bandPunkt(), SCAN.bandZ, () => 'lege aufs Förderband'),
                  { t: 'call', fn: () => { this.scanCount++; this._planer(); } });
      } else {
        this.flipCount++;
        this.status = 'Etikett unten – Paket wird gewendet';
        this._add(this._wendeSchritte(b),
                  { t: 'call', fn: () => this._bearbeite(b, versuch + 1) });
      }
    } });
  }

  /**
   * Wenden ohne Drehung in der Hand: Der Roboter stellt das Werkzeug auf 90°
   * an und setzt das Paket damit auf eine SEITENFLÄCHE des Wendetischs ab.
   * Es liegt dann exakt 90° gedreht da und wird von der neuen Oberseite
   * wieder gegriffen – zwei Durchgänge ergeben die volle Wendung um 180°.
   */
  _wendeSchritte(b) {
    const ax = () => this._wendeAchse();
    // Beim angestellten Werkzeug hängt das Paket seitlich neben dem TCP:
    // Ablagepunkt deshalb um den Greifabstand entgegen der Werkzeugachse versetzen.
    const d = () => this._topHalf(b) + 0.012;
    const hoehe = () => WENDE_Z0 + this._langHalb(b) + 0.008;
    const tcp = (dz = 0) => { const a = ax(), dd = d();
      return [WENDE[0] - a[0] * dd, WENDE[1] - a[1] * dd, hoehe() + dz]; };
    return [
      { t: 'call', fn: () => { this.status = 'stelle das Werkzeug an und setze das Paket auf die Seite'; } },
      { t: 'move', to: () => [WENDE[0] - ax()[0] * 0.14, WENDE[1] - ax()[1] * 0.14, HOVER_Z], dur: 1.1 },
      { t: 'move', to: () => tcp(0.10), tilt: 90, dur: 1.0 },     // Werkzeug waagerecht anstellen
      { t: 'move', to: () => tcp(0), tilt: 90, dur: 0.8 },        // senkrecht absetzen
      { t: 'release' },
      { t: 'wait', s: 0.35 },
      // Rückzug ORTHOGONAL zur Greiffläche: erst waagerecht weg, dann hoch.
      { t: 'move', to: () => { const a = ax(), s3 = this.siteId * 3, dd = this.e.data;
          return [dd.site_xpos[s3] - a[0] * 0.10, dd.site_xpos[s3 + 1] - a[1] * 0.10, dd.site_xpos[s3 + 2]]; },
        tilt: 90, dur: 0.7 },
      this._hoch(HOVER_Z, 0.8),
      { t: 'wait', s: 0.4 },
    ];
  }

  /** Größtes halbes Kantenmaß – so hoch steht das Paket nach dem Wenden höchstens. */
  _langHalb(bodyId) {
    const md = this.e.model, g = md.body_geomadr[bodyId];
    if (g < 0) return 0.03;
    const s = md.geom_size;
    return Math.max(s[3 * g], s[3 * g + 1], s[3 * g + 2]);
  }

  _bandPunkt() {
    const n = this.scanCount % 3;
    return [SCAN.bandZiel[0] + n * 0.055 - 0.055, SCAN.bandZiel[1]];
  }

  /** Pakete in der Zielbox zählen (Programmende, wenn sie voll ist). */
  _boxVoll() { return this._inBox() >= SCAN.boxMax; }

  _inBox() {
    const d = this.e.data;
    let n = 0;
    for (const b of this._pakete()) {
      const p = b * 3;
      const inZiel = Math.abs(d.xpos[p] - SCAN.box[0]) < 0.17 && Math.abs(d.xpos[p + 1] - SCAN.box[1]) < 0.15;
      const inRechts = Math.abs(d.xpos[p] - SCAN.rechtsBox[0]) < 0.15 && Math.abs(d.xpos[p + 1] - SCAN.rechtsBox[1]) < 0.14;
      if ((inZiel || inRechts) && d.xpos[p + 2] < 0.36) n++;
    }
    return n;
  }

  /* ---------- gemeinsame Steuerung ---------- */
  _bereit() {
    if (!this.ok) { this.status = 'nicht bereit'; return false; }
    return true;
  }

  _neu(mode) {
    this.stop(true);
    this.mode = mode;
    this.zaehler = 0;
    this._seq = [];
    this.phase = 'run';
    if (this.e.paused) this.e.paused = false;
  }

  /** Laufendes Programm beschreiben – für die Wiederaufnahme nach einem
   *  Modellwechsel (z. B. wenn währenddessen Teile abgeworfen werden). */
  laufendesProgramm() {
    if (this.phase === 'idle' || !this.mode) return null;
    return { mode: this.mode, farbe: this.farbe, zaehler: this.zaehler };
  }

  nimmWiederAuf(z) {
    if (!z || !this.ok) return false;
    if (z.mode === 'kugeln') this.startKugeln(z.farbe);
    else if (z.mode === 'palette') this.startPalettieren(z.farbe);
    else if (z.mode === 'scan') this.startScan();
    else return false;
    this.zaehler = z.zaehler ?? 0;
    return true;
  }

  _abschluss(text) {
    this._add({ t: 'call', fn: () => { this.status = 'zurück zur Home-Pose'; } },
              { t: 'moveQ', q: HORST600_HOME.slice(), dur: 1.4 },
              { t: 'call', fn: () => { this.phase = 'idle'; this.status = text; this._planer = null; } });
  }

  goHome() {
    if (!this.ok) return;
    this.stop(true);
    this.phase = 'run';
    this._seq = [{ t: 'moveQ', q: HORST600_HOME.slice(), dur: 1.2 },
                 { t: 'call', fn: () => { this.phase = 'idle'; this.status = 'Home-Pose'; } }];
    if (this.e.paused) this.e.paused = false;
  }

  stop(silent) {
    this.phase = 'idle';
    this._seq = [];
    this._planer = null;
    this.ramp = null;
    this.e.onPreStep = null;
    this._bandDofs.length = 0;
    this.e.releaseBody?.();
    if (this.ok) for (const j of this.joints) this.e.data.ctrl[j.ctrl] = this.e.data.qpos[j.qadr];
    if (!silent) this.status = 'gestoppt';
  }

  /** Rampenfahrt auf Gelenkwinkel. */
  _rampTo(q, dur) {
    this.ramp = { from: this.joints.map(j => this.e.data.ctrl[j.ctrl]), to: q,
                  t: 0, dur: Math.max(0.25, dur / this.speed), settle: 0 };
  }

  tick(dt) {
    if (!this.ok || !this.e.loaded) return;
    if (this.mode === 'scan') this._bandLauf(dt);
    if (this.phase !== 'run') return;
    const e = this.e;

    if (this.ramp) {                                  // laufende Fahrt bedienen
      const r = this.ramp;
      r.t += dt;
      const s = Math.min(1, r.t / r.dur), k = s * s * (3 - 2 * s);
      for (let i = 0; i < 6; i++) e.data.ctrl[this.joints[i].ctrl] = r.from[i] + (r.to[i] - r.from[i]) * k;
      if (s >= 1) {
        r.settle += dt;
        const err = Math.max(...this.joints.map((j, i) => Math.abs(e.data.qpos[j.qadr] - r.to[i])));
        if (err < 0.03 || r.settle > 1.2) this.ramp = null;
      }
      return;
    }
    if (this.wait > 0) { this.wait -= dt; return; }

    const s = this._seq.shift();
    if (!s) { if (this._planer) this._planer(); else this.phase = 'idle'; return; }

    switch (s.t) {
      case 'move': {
        const ziel = typeof s.to === 'function' ? s.to() : s.to;
        const q = s.tilt ? this._solveTilt(ziel, s.tilt) : this._solveSmart(ziel);
        if (!q) {                                     // unerreichbar: Teil überspringen
          this.status = `„${this.zielName ?? '?'}" außer Reichweite – übersprungen`;
          this.e.releaseBody();
          this._seq = [];
          this._add(this._hoch(HOVER_Z, 0.6), { t: 'call', fn: () => this._planer?.() });
          return;
        }
        this._rampTo(q, s.dur ?? 1.0);
        return;
      }
      case 'moveQ': this._rampTo(s.q, s.dur ?? 1.2); return;
      case 'grab': {
        const b = s.body();
        if (b > 0) e.attachBody(b, this.siteId);
        return;
      }
      case 'release': e.releaseBody(); return;
      case 'wait': this.wait = s.s / this.speed; return;
      case 'call': s.fn(); return;
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
