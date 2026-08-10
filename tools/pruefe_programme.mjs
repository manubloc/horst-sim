/* ============================================================
 * Prüfstand für die drei Anwendungen.
 * Misst nicht nur das Ergebnis, sondern liest die echten Kontakte
 * der Physik aus: berührt der Roboter jemals Kasten, Palette,
 * Rampe, Band oder Zielbox, gilt der Lauf als kollisionsbehaftet.
 * ============================================================ */
import { SimEngine } from '../app/js/engine.js';
import { SCENES } from '../app/js/scenes.js';
import { PickController } from '../app/js/pick.js';

globalThis.__bootlog = () => {};

const HINDERNIS = /kasten|palette|rampe|band|zielbox|wendetisch/;

function waechter(e) {
  const md = e.model;
  const roboter = new Set();
  for (let i = 0; i < md.nbody; i++) {
    const n = e.bodyName(i) || '';
    if (n.includes('link_') || n.includes('horst_basis')) roboter.add(i);
  }
  const treffer = new Map();
  let zaehler = 0;
  return {
    pruefe() {
      // Stichprobe statt jedes Bild: jeder Kontaktzugriff legt in der Bindung
      // ein Wrapper-Objekt an, das nicht freigegeben wird (sonst 2-GB-Limit).
      if ((zaehler = (zaehler + 1) % 12) !== 0) return;
      const d = e.data;
      const n = Math.min(d.ncon, 80);
      for (let c = 0; c < n; c++) {
        let k = null;
        try {
          k = d.contact[c];
          if (!k) continue;
          const b1 = md.geom_bodyid[k.geom1], b2 = md.geom_bodyid[k.geom2];
          const r1 = roboter.has(b1), r2 = roboter.has(b2);
          if (r1 === r2) continue;
          const gegner = e.bodyName(r1 ? b2 : b1) || '?';
          if (!HINDERNIS.test(gegner)) continue;
          if (k.dist > -0.0015) continue;               // Streifkontakte ignorieren
          treffer.set(gegner, Math.min(treffer.get(gegner) ?? 0, k.dist));
        } catch { /* Kontakt zwischenzeitlich ungültig */ }
        finally { k?.delete?.(); }                      // Wrapper freigeben, sonst wächst der Heap
      }
    },
    bericht() {
      const t = [...treffer.entries()];
      return t.length
        ? `KOLLISION: ${t.map(([n, d]) => `${n} (${(d * 1000).toFixed(1)} mm)`).join(', ')}`
        : 'kollisionsfrei ✓';
    },
  };
}

export async function laufe(szeneId, starter, sekunden, auswertung) {
  const e = await SimEngine.create();
  e.loadXML(SCENES.find(s => s.id === szeneId).make());
  const pick = new PickController(e);
  pick.configure();
  pick.speed = 2;
  const w = waechter(e);
  starter(pick);
  let letzter = '';
  const meilensteine = [];
  for (let i = 0; i < sekunden * 60 && pick.phase !== 'idle'; i++) {
    e.update(1 / 60);
    pick.tick(1 / 60);
    w.pruefe();
    if (pick.status !== letzter) { letzter = pick.status; meilensteine.push(`[${(i / 60).toFixed(0)}s] ${letzter}`); }
  }
  return { e, pick, kollision: w.bericht(), meilensteine, ergebnis: auswertung ? auswertung(e, pick) : null };
}

export function nameById(e) {
  return (n) => { for (let i = 0; i < e.model.nbody; i++) if (e.bodyName(i) === n) return i; return -1; };
}
