/* ============================================================
 * horstSIM – Maus-/Touch-Interaktion
 * Verhalten wie in MuJoCo "simulate":
 *   Links ziehen        → Kamera rotieren (Shift: horizontal)
 *   Rechts/Mitte ziehen → Kamera verschieben
 *   Mausrad             → Zoom
 *   Doppelklick         → Körper auswählen
 *   Strg + Links ziehen → Körper ziehen (Kraft-Perturbation)
 *   Strg + Rechts       → Körper rotieren
 * ============================================================ */

export function attachInteraction(canvas, engine, { onSelect } = {}) {
  const mj = engine.mujoco;
  let dragging = false, button = 0, lastX = 0, lastY = 0, perturbing = false;

  const rel = (dx, dy) => [dx / canvas.clientHeight, dy / canvas.clientHeight];

  function select(ev) {
    if (!engine.loaded) return -1;
    const r = canvas.getBoundingClientRect();
    const relx = (ev.clientX - r.left) / r.width;
    const rely = 1.0 - (ev.clientY - r.top) / r.height;
    const aspect = r.width / r.height;
    const selpnt = new mj.DoubleBuffer(3);
    const gid = new mj.IntBuffer(1), fid = new mj.IntBuffer(1), sid = new mj.IntBuffer(1);
    let body = -1;
    try {
      body = mj.mjv_select(engine.model, engine.data, engine.vopt,
        aspect, relx, rely, engine.scene, selpnt, gid, fid, sid);
      if (body >= 0) {
        const p = selpnt.GetView();
        engine.pert.select = body;
        engine.pert.flexselect = -1;
        engine.pert.skinselect = -1;
        const xp = engine.data.xpos;
        // Weltpunkt → körperlokale Koordinaten (nur Translation; Rotation macht initPerturb)
        const b3 = body * 3;
        const dxl = [p[0] - xp[b3], p[1] - xp[b3 + 1], p[2] - xp[b3 + 2]];
        const xmat = engine.data.xmat; const m9 = body * 9;
        engine.pert.localpos[0] = xmat[m9] * dxl[0] + xmat[m9 + 3] * dxl[1] + xmat[m9 + 6] * dxl[2];
        engine.pert.localpos[1] = xmat[m9 + 1] * dxl[0] + xmat[m9 + 4] * dxl[1] + xmat[m9 + 7] * dxl[2];
        engine.pert.localpos[2] = xmat[m9 + 2] * dxl[0] + xmat[m9 + 5] * dxl[1] + xmat[m9 + 8] * dxl[2];
      } else {
        engine.pert.select = 0;
      }
    } finally { selpnt.delete(); gid.delete(); fid.delete(); sid.delete(); }
    onSelect?.(body);
    return body;
  }

  canvas.addEventListener('contextmenu', e => e.preventDefault());

  canvas.addEventListener('pointerdown', ev => {
    canvas.setPointerCapture(ev.pointerId);
    dragging = true; button = ev.button; lastX = ev.clientX; lastY = ev.clientY;
    if ((ev.ctrlKey || ev.metaKey) && engine.loaded) {
      const body = select(ev);
      if (body > 0) {
        engine.pert.active = ev.button === 2
          ? mj.mjtPertBit.mjPERT_ROTATE.value
          : mj.mjtPertBit.mjPERT_TRANSLATE.value;
        mj.mjv_initPerturb(engine.model, engine.data, engine.scene, engine.pert);
        perturbing = true;
      }
    }
  });

  canvas.addEventListener('pointermove', ev => {
    if (!dragging || !engine.loaded) return;
    const [dx, dy] = rel(ev.clientX - lastX, ev.clientY - lastY);
    lastX = ev.clientX; lastY = ev.clientY;
    const M = mj.mjtMouse;
    if (perturbing && engine.pert.active) {
      const action = engine.pert.active === mj.mjtPertBit.mjPERT_ROTATE.value
        ? (ev.shiftKey ? M.mjMOUSE_ROTATE_H.value : M.mjMOUSE_ROTATE_V.value)
        : (ev.shiftKey ? M.mjMOUSE_MOVE_H.value : M.mjMOUSE_MOVE_V.value);
      mj.mjv_movePerturb(engine.model, engine.data, action, dx, dy, engine.scene, engine.pert);
      if (engine.paused) engine.applyPausedPerturb();
    } else {
      let action;
      if (button === 0) action = ev.shiftKey ? M.mjMOUSE_ROTATE_H.value : M.mjMOUSE_ROTATE_V.value;
      else action = ev.shiftKey ? M.mjMOUSE_MOVE_H.value : M.mjMOUSE_MOVE_V.value;
      mj.mjv_moveCamera(engine.model, action, dx, dy, engine.cam);
    }
  });

  const endDrag = ev => {
    dragging = false;
    if (perturbing) { engine.pert.active = 0; perturbing = false; }
    try { canvas.releasePointerCapture(ev.pointerId); } catch (_) {}
  };
  canvas.addEventListener('pointerup', endDrag);
  canvas.addEventListener('pointercancel', endDrag);

  canvas.addEventListener('dblclick', ev => { if (!ev.ctrlKey) select(ev); });

  canvas.addEventListener('wheel', ev => {
    if (!engine.loaded) return;
    ev.preventDefault();
    const step = -Math.sign(ev.deltaY) * 0.05;
    mj.mjv_moveCamera(engine.model, mj.mjtMouse.mjMOUSE_ZOOM.value, 0, step, engine.cam);
  }, { passive: false });
}
