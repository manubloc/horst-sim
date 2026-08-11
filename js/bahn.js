/* ============================================================
 * horstOS – Bandantrieb für den großen Rundlauf
 *
 * Die Pakete werden kinematisch auf einer Polylinie geführt:
 * Bahngeschwindigkeit entlang der Tangente plus eine sanfte
 * Rückstellung auf die Bahnmitte. Ohne diese Rückstellung fliegen
 * sie an jeder Ecke geradeaus vom Band (gemessen: nach 5 s war
 * kein Paket mehr auf der Bahn).
 *
 * Am Dreiwege-Modul wird reihum verteilt: Kiste A, Kiste B,
 * geradeaus weiter im Kreis.
 * ============================================================ */

export class BahnAntrieb {
  constructor(engine, bahn) {
    this.e = engine;
    this.B = bahn;
    this.route = new Map();        // bodyId → 'haupt' | 'A' | 'B'
    this.verteilt = new Map();     // bodyId → true, sobald an der Weiche entschieden
    this.zaehler = 0;
    this.inKiste = { A: 0, B: 0 };
    this._dofs = [];
  }

  reset() { this.route.clear(); this.verteilt.clear(); this.zaehler = 0; this.inKiste = { A: 0, B: 0 }; }

  /** Nächster Punkt auf einer Polylinie samt Tangente und Bogenlänge. */
  static naechster(pfad, p) {
    let best = null, bestD = Infinity, bogen = 0, vorher = 0;
    for (let i = 0; i < pfad.length - 1; i++) {
      const a = pfad[i], b = pfad[i + 1];
      const d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
      const len2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
      const len = Math.sqrt(len2);
      let t = ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1] + (p[2] - a[2]) * d[2]) / len2;
      t = Math.max(0, Math.min(1, t));
      const q = [a[0] + d[0] * t, a[1] + d[1] * t, a[2] + d[2] * t];
      const dist = Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
      if (dist <= bestD) {                      // Gleichstand: das spätere Segment gewinnt
        bestD = dist;
        best = { q, t: [d[0] / len, d[1] / len, d[2] / len], seg: i, bogen: vorher + len * t, rest: len * (1 - t) };
      }
      vorher += len;
      bogen = vorher;
    }
    if (best) {
      best.gesamt = bogen;
      // Am Segmentende die Richtung des Folgesegments übernehmen, sonst wird
      // das Paket über die Ecke hinaus geradeaus weitergeschoben.
      if (best.rest < 0.05 && best.seg < pfad.length - 2) {
        const a = pfad[best.seg + 1], c = pfad[best.seg + 2];
        const d2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        const l2 = Math.hypot(d2[0], d2[1], d2[2]) || 1;
        best.t = [d2[0] / l2, d2[1] / l2, d2[2] / l2];
      }
    }
    return best;
  }

  _pfadVon(bodyId) {
    const r = this.route.get(bodyId);
    if (r === 'A') return this.B.zweigA;
    if (r === 'B') return this.B.zweigB;
    return this.B.haupt;
  }

  /** Einmal je Bild: entscheiden, wer wohin fährt. */
  plane(pakete, gehalten) {
    this._dofs.length = 0;
    const md = this.e.model, d = this.e.data;
    const W = this.B.weiche;
    for (const b of pakete) {
      if (gehalten.has(b)) { this.route.delete(b); this.verteilt.delete(b); continue; }
      const p3 = b * 3;
      const p = [d.xpos[p3], d.xpos[p3 + 1], d.xpos[p3 + 2]];

      // In einer Kiste gelandet? Dann nicht mehr antreiben.
      if (p[2] < this.B.z - 0.10) { this.route.delete(b); continue; }

      const pfad = this._pfadVon(b);
      const n = BahnAntrieb.naechster(pfad, p);
      if (!n) continue;
      const quer = Math.hypot(p[0] - n.q[0], p[1] - n.q[1]);
      if (quer > this.B.bw + 0.14 || p[2] < n.q[2] - 0.14 || p[2] > n.q[2] + 0.26) continue;

      // Dreiwege-Modul: beim Erreichen der Weiche einmalig zuteilen
      if (!this.verteilt.get(b) && Math.abs(p[0] - W[0]) < 0.055 && Math.abs(p[1] - W[1]) < this.B.bw) {
        const wahl = ['A', 'B', 'haupt'][this.zaehler++ % 3];
        this.route.set(b, wahl);
        this.verteilt.set(b, true);
        continue;                                      // im nächsten Bild auf dem neuen Pfad
      }
      // Auf der Hauptbahn hinter der Weiche wieder freigeben (für die nächste Runde)
      if (this.verteilt.get(b) && this.route.get(b) === 'haupt' && p[0] > W[0] + 0.35) this.verteilt.delete(b);

      // Am Ende eines Zweigs: auslaufen lassen, damit das Paket in die Kiste kippt
      const letzte = n.seg === pfad.length - 2;
      const amEnde = n.rest < 0.06 && letzte;
      // Vor der Übergabe abbremsen, sonst schießt das Paket über die Rampe hinaus
      // (gemessen: 44 cm Überlauf bei voller Bandgeschwindigkeit).
      const tempo = letzte && n.rest < 0.30 ? this.B.v * (0.25 + 0.75 * (n.rest / 0.30)) : this.B.v;
      const dofadr = md.jnt_dofadr[md.body_jntadr[b]];
      if (amEnde && this.route.get(b) !== 'haupt') {
        this._dofs.push({ dofadr, v: [n.t[0] * this.B.v * 1.2, n.t[1] * this.B.v * 1.2, 0] });
        continue;
      }
      if (amEnde) continue;                            // Ende der Hauptbahn: auf die Rampe fallen lassen

      // Sollhöhe ist die Bandoberfläche PLUS die halbe Pakethöhe. Ohne diesen
      // Versatz zog die Rückstellung die Pakete in das Band hinein; an der
      // Steigung rutschten sie darunter durch statt hinaufzufahren.
      const halbH = md.geom_size[3 * md.body_geomadr[b] + 2] || 0.03;
      const sollZ = n.q[2] + halbH + 0.006;
      const k = 2.4;
      this._dofs.push({ dofadr, v: [
        n.t[0] * tempo + Math.max(-0.4, Math.min(0.4, n.q[0] - p[0])) * k,
        n.t[1] * tempo + Math.max(-0.4, Math.min(0.4, n.q[1] - p[1])) * k,
        n.t[2] * tempo + Math.max(-0.35, Math.min(0.35, sollZ - p[2])) * (k * 1.6),
      ] });
    }
  }

  /** Vor JEDEM Physikschritt – einmal je Bild reicht nicht, die
   *  Zwischenschritte dämpfen die Bandgeschwindigkeit sonst weg. */
  antreibe() {
    const d = this.e.data;
    for (const { dofadr, v } of this._dofs) {
      d.qvel[dofadr] = v[0]; d.qvel[dofadr + 1] = v[1]; d.qvel[dofadr + 2] = v[2];
    }
  }

  zaehleKisten(pakete) {
    const d = this.e.data, K = this.B;
    let a = 0, b2 = 0;
    for (const b of pakete) {
      const p3 = b * 3;
      if (d.xpos[p3 + 2] > 0.34) continue;
      if (Math.abs(d.xpos[p3] - K.kisteA[0]) < 0.20 && Math.abs(d.xpos[p3 + 1] - K.kisteA[1]) < 0.16) a++;
      else if (Math.abs(d.xpos[p3] - K.kisteB[0]) < 0.20 && Math.abs(d.xpos[p3 + 1] - K.kisteB[1]) < 0.16) b2++;
    }
    this.inKiste = { A: a, B: b2 };
    return this.inKiste;
  }
}
