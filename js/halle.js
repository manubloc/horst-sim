/* ============================================================
 * horstOS – Hallenlogistik
 *
 * Auf einem Rundlauf-Förderband kreisen Pakete. Eine fahrbare
 * Hubplattform mit zwei HORST-Armen fährt an die Übernahmestelle,
 * greift dort entweder zwei kleine Pakete (je Arm eines) oder ein
 * großes Paket beidseitig – wie ein Mensch, der einen Karton links
 * und rechts anfasst – und stellt es in eines der Regale.
 *
 * Der Umlauf wird kinematisch angetrieben: Pakete auf dem Band
 * bekommen vor jedem Physikschritt die Bahngeschwindigkeit ihres
 * Abschnitts. Alles andere ist echte Physik.
 * ============================================================ */

import { PickController } from './pick.js';

const BAHN = { x0: 1.15, x1: 3.15, y0: -0.95, y1: 0.95, z: 0.62, bw: 0.24 };
const BAND_V = 0.30;                       // Umlaufgeschwindigkeit [m/s]
const STATION = { x: 1.25, y: -1.30 };     // Standplatz der Plattform vor der Bandkante
const UEBERNAHME = { x: 1.55, y: -0.95 };  // Punkt auf der vorderen Bandkante
const GREIF_TOL = 0.26;                    // Fangbereich um die Übernahmestelle

/* Regalfächer: x, y, Ebenenhöhen, Anfahrt-Standplatz der Plattform */
/* Standplätze aus der Geometrie gerechnet: Das Regal steht bei y = −1,55 mit
 * 0,20 m halber Tiefe, seine vorderen Steher liegen bei y = −1,37. Die Plattform
 * ist 0,28 m halbtief, also darf ihre Mitte nicht südlicher als −1,04 stehen.
 * Der Arm sitzt 0,26 m seitlich der Plattformmitte, die Basis 0,255 m + Hub hoch. */
/* stand = Px, sodass ARM A über der Regalmitte steht:
   Arm A liegt bei x = 0,30 (Aggregat) + Px + 0,26 (Armversatz) = Px + 0,56. */
const ARM_A_VERSATZ = 0.56;
const REGALE = [
  { name: 'Regal 1', x: 0.35 },
  { name: 'Regal 2', x: 1.55 },
  { name: 'Regal 3', x: 2.75 },
].map(r => ({ ...r, stand: +(r.x - ARM_A_VERSATZ).toFixed(3) }));
const REGAL_Y = -2.00;                     // Mittellinie der Regalreihe
const STAND_Y = -1.55;                     // Standplatz davor (Plattform 0,20 halbtief)
const FACH_Y = -1.86;                      // Ablagetiefe im Fach (hinter der Vorderkante)
const EBENEN = [0.34, 0.72, 1.10];         // Fachhöhen im Regal

export class HalleController {
  constructor(engine) {
    this.e = engine;
    this.A = new PickController(engine);     // linker Arm
    this.B = new PickController(engine);     // rechter Arm
    this.phase = 'idle';
    this.status = 'bereit';
    this.speed = 1;
    this.eingelagert = 0;
    this.ok = false;
    this._seq = [];
    this._bandDofs = [];
    this._ramp = null;
    this.wait = 0;
  }

  configure() {
    this.stop(true);
    this.ok = false;
    const e = this.e;
    if (!e.loaded) return;
    this.A.configure('A_');
    this.B.configure('B_');
    if (!this.A.ok || !this.B.ok) { this.status = 'keine Doppelarm-Zelle'; return; }
    const mj = e.mujoco, md = e.model;
    const akt = new Map(e.listActuators().map(a => [a.name, a.i]));
    const jnt = new Map(e.listJoints().map(j => [j.name, j]));
    if (!akt.has('APx') || !jnt.has('Px')) { this.status = 'keine Hubplattform'; return; }
    this.platt = ['Px', 'Py', 'Pz'].map((n, k) => ({
      qadr: jnt.get(n).qadr, ctrl: akt.get(['APx', 'APy', 'APz'][k]),
      lo: md.jnt_range[2 * jnt.get(n).i], hi: md.jnt_range[2 * jnt.get(n).i + 1],
    }));
    this.ok = true;
    this.status = 'bereit';
  }

  /* ---------- Rundlauf ---------- */

  _hpakete() {
    const out = [];
    for (let i = 1; i < this.e.model.nbody; i++) {
      const n = this.e.bodyName(i) || '';
      if (n.startsWith('hpaket_')) out.push(i);
    }
    return out;
  }

  /** Nächster Punkt auf der Bahnmitte samt Laufrichtung (gegen den Uhrzeigersinn).
   *  Die Pakete werden auf diese Linie geregelt – sonst fliegen sie an den Ecken
   *  geradeaus vom Band (gemessen: nach 5 s war keines mehr auf der Bahn). */
  _fuehrung(x, y) {
    const B = BAHN;
    const kanten = [
      { a: [B.x0, B.y0], b: [B.x1, B.y0] },        // vorn, nach +X
      { a: [B.x1, B.y0], b: [B.x1, B.y1] },        // rechts, nach +Y
      { a: [B.x1, B.y1], b: [B.x0, B.y1] },        // hinten, nach −X
      { a: [B.x0, B.y1], b: [B.x0, B.y0] },        // links, nach −Y
    ];
    let best = null, bestD = Infinity;
    for (const k of kanten) {
      const dx = k.b[0] - k.a[0], dy = k.b[1] - k.a[1];
      const len2 = dx * dx + dy * dy;
      let t = ((x - k.a[0]) * dx + (y - k.a[1]) * dy) / len2;
      t = Math.max(0, Math.min(1, t));
      const px = k.a[0] + dx * t, py = k.a[1] + dy * t;
      const d = Math.hypot(x - px, y - py);
      if (d < bestD) { bestD = d; const L = Math.hypot(dx, dy); best = { px, py, tx: dx / L, ty: dy / L }; }
    }
    return best;
  }


  _aufBand(bodyId) {
    const d = this.e.data, p = bodyId * 3;
    const x = d.xpos[p], y = d.xpos[p + 1], z = d.xpos[p + 2];
    if (z < BAHN.z - 0.02 || z > BAHN.z + 0.22) return false;
    const B = BAHN, r = B.bw + 0.02;
    const imRahmen = x > B.x0 - r && x < B.x1 + r && y > B.y0 - r && y < B.y1 + r;
    if (!imRahmen) return false;
    const innen = x > B.x0 + r && x < B.x1 - r && y > B.y0 + r && y < B.y1 - r;
    return !innen;
  }

  /** Vor jedem Physikschritt: Bahngeschwindigkeit aufprägen. */
  _bandAntrieb() {
    const d = this.e.data;
    for (const { dofadr, vx, vy } of this._bandDofs) {
      d.qvel[dofadr] = vx; d.qvel[dofadr + 1] = vy;
    }
  }

  _bandLauf() {
    this._bandDofs.length = 0;
    const md = this.e.model, d = this.e.data;
    const gehalten = new Set(this.e.attachments.map(a => a.bodyId));
    for (const b of this._hpakete()) {
      if (gehalten.has(b)) continue;
      if (!this._aufBand(b)) continue;
      const p = b * 3;
      const f = this._fuehrung(d.xpos[p], d.xpos[p + 1]);
      const kx = Math.max(-0.5, Math.min(0.5, f.px - d.xpos[p]));      // sanft auf die Bahnmitte ziehen
      const ky = Math.max(-0.5, Math.min(0.5, f.py - d.xpos[p + 1]));
      const dofadr = md.jnt_dofadr[md.body_jntadr[b]];
      this._bandDofs.push({ dofadr, vx: f.tx * BAND_V + kx * 2.2, vy: f.ty * BAND_V + ky * 2.2 });
    }
  }

  /** Paket, das gerade an der Übernahmestelle vorbeikommt. */
  _inUebernahme(nurGross) {
    const d = this.e.data;
    let best = -1, bestD = 9;
    for (const b of this._hpakete()) {
      const n = this.e.bodyName(b) || '';
      if (nurGross !== null && (n.includes('gross') !== nurGross)) continue;
      if (!this._aufBand(b)) continue;
      const p = b * 3;
      const dd = Math.hypot(d.xpos[p] - UEBERNAHME.x, d.xpos[p + 1] - UEBERNAHME.y);
      if (dd < GREIF_TOL && dd < bestD) { best = b; bestD = dd; }
    }
    return best;
  }

  /* ---------- Bewegungsbausteine ---------- */

  _add(...s) { this._seq.push(...s.flat()); }

  /** Plattform verfahren (x, y, Hub). null = Wert halten. */
  _fahre(x, y, z, dur = 2.2, text) {
    return {
      t: 'platt', dur, text,
      ziel: () => [x, y, z].map((w, k) => w === null || w === undefined
        ? this.e.data.qpos[this.platt[k].qadr] : w),
    };
  }

  /** Beide Arme auf eine Gelenkstellung rampen. */
  _arme(qA, qB, dur = 1.4, text) { return { t: 'arme', qA, qB, dur, text }; }

  _armeIK(zielA, zielB, dur, text, tilt = 90) {
    return { t: 'armeIK', zielA, zielB, dur, text, tilt };
  }

  /* ---------- Programm ---------- */

  start() {
    if (!this.ok) { this.status = 'nicht bereit'; return; }
    this.stop(true);
    this.phase = 'run';
    this.eingelagert = 0;
    this.status = 'Rundlauf läuft';
    this.e.onPreStep = () => this._bandAntrieb();
    if (this.e.paused) this.e.paused = false;
    this._planer = () => this._planZyklus();
    this._planer();
  }

  stop(silent) {
    this.phase = 'idle';
    this._seq = [];
    this._ramp = null;
    this._planer = null;
    this._bandDofs.length = 0;
    this.e.onPreStep = null;
    this.e.releaseBody();
    if (this.ok) {
      for (const j of this.platt) this.e.data.ctrl[j.ctrl] = this.e.data.qpos[j.qadr];
      for (const p of [this.A, this.B]) for (const j of p.joints) this.e.data.ctrl[j.ctrl] = this.e.data.qpos[j.qadr];
    }
    if (!silent) this.status = 'gestoppt';
  }

  _planZyklus() {
    const regal = REGALE[this.eingelagert % REGALE.length];
    const ebene = EBENEN[Math.floor(this.eingelagert / REGALE.length) % EBENEN.length];
    let ziel = -1, gross = false;

    this._add(
      this._fahre(STATION.x, STATION.y, 0.30, 2.0, () => 'fahre zur Übernahme'),
      { t: 'warte', bed: () => {
          const g = this._inUebernahme(true), k = this._inUebernahme(false);
          if (g > 0) { ziel = g; gross = true; return true; }
          if (k > 0) { ziel = k; gross = false; return true; }
          return false;
        }, text: () => 'warte auf das nächste Paket' },
      { t: 'call', fn: () => {
          this.status = gross ? 'großes Paket – beide Arme greifen seitlich' : 'kleines Paket – ein Arm greift';
          this._zielName = this.e.bodyName(ziel);
        } },
      // Beidseitig anfahren: Werkzeuge waagerecht, je eine Seitenfläche
      { t: 'greifen', body: () => ziel, gross: () => gross },
      this._fahre(null, null, 0.52, 1.0, () => 'Paket anheben'),
      this._fahre(regal.stand, STAND_Y, 0.52, 3.0, () => `fahre zu ${regal.name}`),
      { t: 'call', fn: () => { this.status = `Hub auf Fachebene ${(ebene * 100) | 0} cm`; } },
      this._fahre(null, null, Math.max(0, Math.min(0.75, ebene - 0.24)), 1.6),
      { t: 'einlagern', regal: () => regal, ebene: () => ebene, body: () => ziel },
      { t: 'call', fn: () => { this.eingelagert++; this.status = `${this.eingelagert} Pakete eingelagert`; } },
      { t: 'weiter' },
    );
  }

  /* ---------- Ablaufsteuerung ---------- */

  tick(dt) {
    if (this.phase === 'idle' || !this.ok || !this.e.loaded) return;
    const s = dt * this.speed;
    this._bandLauf();

    if (this._ramp) {
      const r = this._ramp;
      r.t += s;
      const k = Math.min(1, r.t / r.dur), w = k * k * (3 - 2 * k);
      for (const b of r.bahnen) this.e.data.ctrl[b.ctrl] = b.von + (b.nach - b.von) * w;
      if (k >= 1) {
        // Erst weiter, wenn die Achsen wirklich stehen. Vorher lief die Folge
        // schon los, während die Plattform noch einen halben Meter unterwegs war.
        r.warte = (r.warte ?? 0) + s;
        const fehler = Math.max(...r.bahnen.map(b => Math.abs(this.e.data.qpos[b.qadr] - b.nachQ)));
        if (fehler < r.tol || r.warte > r.gedulden) this._ramp = null;
      }
      return;
    }
    if (this.wait > 0) { this.wait -= s; return; }

    const schritt = this._seq.shift();
    if (!schritt) { if (this._planer) this._planer(); return; }
    this._fuehreAus(schritt);
  }

  _rampe(bahnen, dur, tol = 0.05) {
    const weg = Math.max(...bahnen.map(b => Math.abs(b.nachQ - b.von)));
    this._ramp = { t: 0, dur: Math.max(0.15, dur / this.speed), bahnen, warte: 0, tol,
                   gedulden: 2.5 + weg * 2.5 };
  }

  _bahn(j, ziel) {
    return { ctrl: j.ctrl, qadr: j.qadr, von: this.e.data.ctrl[j.ctrl],
             nach: Math.max(j.lo ?? -9, Math.min(j.hi ?? 9, ziel)), nachQ: ziel };
  }

  _fuehreAus(s) {
    const e = this.e;
    switch (s.t) {
      case 'call': s.fn(); return;
      case 'weiter': if (this._planer) this._planer(); return;
      case 'warte':
        if (s.text) this.status = s.text();
        if (s.bed()) return;
        this._seq.unshift(s);                       // weiter warten
        this.wait = 0.1;
        return;
      case 'platt': {
        if (s.text) this.status = s.text();
        const ziel = s.ziel();
        const bahnen = this.platt.map((j, k) => this._bahn(j, ziel[k]));
        const weg = Math.max(...bahnen.map(b => Math.abs(b.nachQ - b.von)));
        this._rampe(bahnen, Math.max(s.dur ?? 1.2, weg / 0.55), 0.025);   // ≈ 0,55 m/s Fahrgeschwindigkeit
        return;
      }
      case 'arme': {
        if (s.text) this.status = s.text();
        const bahnen = [...this.A.joints.map((j, k) => this._bahn(j, s.qA[k])),
                        ...this.B.joints.map((j, k) => this._bahn(j, s.qB[k]))];
        this._rampe(bahnen, s.dur);
        return;
      }
      case 'greifen': {
        const b = s.body(), gross = s.gross();
        if (b <= 0) { this._seq.length = 0; if (this._planer) this._planer(); return; }
        const halb = this._halb(b);
        const p = b * 3, d = e.data;
        const mitte = [d.xpos[p], d.xpos[p + 1], d.xpos[p + 2]];
        // Arm A greift die +Y-Seite, Arm B die −Y-Seite (Plattform steht quer zum Band)
        // Arm A fasst die +X-Seite, Arm B die −X-Seite – beide waagerecht.
        const zielA = [mitte[0] + halb[0] + 0.014, mitte[1], mitte[2]];
        const zielB = [mitte[0] - halb[0] - 0.014, mitte[1], mitte[2]];
        // Werkzeugachsen fest waagerecht aufeinander zu – so fassen beide Arme
        // die gegenüberliegenden Seitenflächen flächig an.
        const qA = this.A.solveIK(zielA, undefined, 90, [-1, 0, 0]);
        const qB = gross ? this.B.solveIK(zielB, undefined, 90, [1, 0, 0]) : null;
        if (!qA) { this.status = 'Paket außer Reichweite – nächstes'; this._seq.length = 0; if (this._planer) this._planer(); return; }
        this._seq.unshift(
          { t: 'arme', qA, qB: qB ?? this.B.q(), dur: 1.3, text: () => this.status },
          { t: 'call', fn: () => {
              e.attachBody(b, this.A.siteId);
              this._gehalten = b;
            } },
          { t: 'wait', s: 0.25 },
        );
        return;
      }
      case 'einlagern': {
        const regal = s.regal(), ebene = s.ebene(), b = s.body();
        const halb = this._halb(b);
        // Der Greifer hält die Seitenfläche; im Fach steht er neben dem Paket,
        // Werkzeugachse waagerecht nach −Y. Erst davor, dann hinein, nie von oben
        // durch den Regalboden.
        const ziel = [regal.x + (this.eingelagert % 2 ? 0.15 : -0.15) + halb[0] + 0.014,
                      FACH_Y, ebene + halb[2] + 0.030];   // 3 cm Luft über dem Fachboden
        void regal;
        const achse = [0, -1, 0];
        const q1 = this.A.solveIK([ziel[0], ziel[1] + 0.22, ziel[2] + 0.06], undefined, 90, achse);
        const q2 = this.A.solveIK(ziel, undefined, 90, achse);
        if (!q1 || !q2) { this.status = 'Regalfach nicht erreichbar'; e.releaseBody(); return; }

        this._seq.unshift(
          { t: 'arme', qA: q1, qB: this.B.q(), dur: 1.6, text: () => 'vor das Regalfach' },
          { t: 'arme', qA: q2, qB: this.B.q(), dur: 1.4, text: () => 'Paket einstellen' },
          { t: 'tcpWarten', ziel: () => ziel },
          { t: 'call', fn: () => e.releaseBody() },
          { t: 'wait', s: 0.35 },
          { t: 'arme', qA: q1, qB: this.B.q(), dur: 1.1, text: () => 'Arme zurückziehen' },
        );
        return;
      }
      case 'tcpWarten': {
        // Erst loslassen, wenn der Greifer wirklich am Ablagepunkt steht.
        // Die Gelenktoleranz allein reichte nicht: die Arme sacken unter Last,
        // die Rampe lief in die Zeitgrenze und das Paket fiel unterwegs herunter.
        const z = s.ziel();
        const s3 = this.A.siteId * 3, d = e.data;
        const fehler = Math.hypot(d.site_xpos[s3] - z[0], d.site_xpos[s3 + 1] - z[1], d.site_xpos[s3 + 2] - z[2]);
        s.t = (s.t ?? 0) + 0.1;
        if (fehler < 0.035) { this._tcpFehler = fehler; return; }
        if (s.t > 4) { this._tcpFehler = fehler; this.status = `Ablage ungenau (${(fehler * 1000) | 0} mm)`; return; }
        this._seq.unshift(s);
        this.wait = 0.1;
        return;
      }
      case 'wait': this.wait = s.s / this.speed; return;
      default: return;
    }
  }

  _halb(bodyId) {
    const md = this.e.model, g = md.body_geomadr[bodyId];
    if (g < 0) return [0.05, 0.04, 0.03];
    return [md.geom_size[3 * g], md.geom_size[3 * g + 1], md.geom_size[3 * g + 2]];
  }
}
