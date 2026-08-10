/* ============================================================
 * horstSIM – Szenen-Bearbeitung (reine String-Funktionen)
 * Dupliziert/entfernt freie Objekte (Top-Level-Bodies mit
 * <freejoint/>) direkt im MJCF-XML und hält den "home"-Keyframe
 * konsistent (qpos-Länge muss nq entsprechen!).
 *
 * Konventionen:
 *  - Freie Objekte haben KEINE Kind-Bodies (Guard eingebaut).
 *  - Neue Bodies werden ans Ende von <worldbody> gehängt, ihre
 *    7 Freejoint-qpos-Werte damit ans Ende des Keyframes.
 *  - Beim Entfernen liefert der Aufrufer qadr (qpos-Startadresse
 *    des Freejoints aus dem kompilierten Modell).
 * ============================================================ */

function findBodyBlock(xml, name) {
  const openTag = new RegExp(`<body\\s+name="${name}"[^>]*>`);
  const m = openTag.exec(xml);
  if (!m) return null;
  const start = m.index;
  const end = xml.indexOf('</body>', start);
  if (end < 0) return null;
  const inner = xml.slice(start + m[0].length, end);
  if (inner.includes('<body')) throw new Error(`"${name}" hat Kind-Körper – nur freie Objekte sind bearbeitbar.`);
  return { start, end: end + '</body>'.length, openTag: m[0], block: xml.slice(start, end + '</body>'.length) };
}

export function uniqueName(xml, srcName) {
  const base = srcName.replace(/_\d+$/, '');
  let n = 2;
  while (xml.includes(`name="${base}_${n}"`)) n++;
  return `${base}_${n}`;
}

const f5 = (x) => (+x).toFixed(5).replace(/\.?0+$/, '') || '0';

/** Freies Objekt klonen; pose = { pos:[x,y,z], quat:[w,x,y,z] }. */
export function duplicateFreeBody(xml, srcName, pose) {
  const hit = findBodyBlock(xml, srcName);
  if (!hit) throw new Error(`Körper "${srcName}" nicht im XML gefunden.`);
  const newName = uniqueName(xml, srcName);

  let open = hit.openTag.replace(`name="${srcName}"`, `name="${newName}"`);
  const posStr = pose.pos.map(f5).join(' ');
  const quatStr = pose.quat.map(f5).join(' ');
  open = /\spos="[^"]*"/.test(open) ? open.replace(/\spos="[^"]*"/, ` pos="${posStr}"`) : open.replace('>', ` pos="${posStr}">`);
  open = /\squat="[^"]*"/.test(open) ? open.replace(/\squat="[^"]*"/, ` quat="${quatStr}"`) : open.replace('>', ` quat="${quatStr}">`);
  const clone = open + hit.block.slice(hit.openTag.length);

  const wbEnd = xml.lastIndexOf('</worldbody>');
  if (wbEnd < 0) throw new Error('Kein </worldbody> gefunden.');
  let out = xml.slice(0, wbEnd) + '    ' + clone + '\n  ' + xml.slice(wbEnd);

  // Keyframe "home": qpos um die 7 Freejoint-Werte des Klons erweitern
  out = out.replace(/(<key\s[^>]*qpos=")([^"]*)(")/, (all, a, q, c) =>
    a + (q.trim() + ' ' + [...pose.pos, ...pose.quat].map(f5).join(' ')).trim() + c);
  return { xml: out, newName };
}

/** Freies Objekt entfernen; qadr = qpos-Startadresse seines Freejoints. */
export function removeFreeBody(xml, name, qadr) {
  const hit = findBodyBlock(xml, name);
  if (!hit) throw new Error(`Körper "${name}" nicht im XML gefunden.`);
  let out = xml.slice(0, hit.start) + xml.slice(hit.end);
  out = out.replace(/(<key\s[^>]*qpos=")([^"]*)(")/, (all, a, q, c) => {
    const vals = q.trim().split(/\s+/);
    if (qadr >= 0 && qadr + 7 <= vals.length) vals.splice(qadr, 7);
    return a + vals.join(' ') + c;
  });
  return out;
}

/** Alle freien Top-Level-Objekte (Name + Blockgrenzen) auflisten. */
export function listFreeBodies(xml) {
  const out = [];
  const re = /<body\s+name="([^"]+)"[^>]*>/g;
  let m;
  while ((m = re.exec(xml))) {
    const end = xml.indexOf('</body>', m.index);
    if (end < 0) continue;
    const inner = xml.slice(m.index + m[0].length, end);
    if (!inner.includes('<body') && inner.includes('<freejoint')) out.push(m[1]);
  }
  return out;
}
