# horstSIM – MuJoCo-Robotersimulation im Browser

Web-basiertes Simulationstool im horstOS-Design. Physik: offizielle
**MuJoCo-WebAssembly-Bindings** von Google DeepMind (`@mujoco/mujoco`,
Engine-Version 3.11), Rendering: three.js. Keine Build-Toolchain nötig –
reine statische Website.

## Starten (lokal)

Ein beliebiger statischer HTTP-Server genügt (WASM lädt nicht per `file://`):

```bash
npx serve .          # oder: python3 -m http.server 8000
```

Dann `http://localhost:3000` (bzw. `:8000`) öffnen. Deployment: Ordner
unverändert auf Cloudflare Pages / GitHub Pages / beliebiges Static Hosting.

## Funktionsumfang

- **HORST600 G2** als parametrisches MJCF-Modell – Kinematik (Gelenk-Offsets,
  Achsen, Limits, Geschwindigkeiten) exakt aus `horst600-g2-pt1-chain.xacro`
  übernommen; Geometrie als Primitiv-Approximation in Original-Farbschema.
- **Einrichtungs-Assistent**: Zelle in 5 Schritten (Roboteranzahl, Montage,
  Objekte, Physik) → erzeugt vollständiges MJCF.
- **Echtzeit-Simulation** mit Pause/Einzelschritt/Reset, Echtzeitfaktor 0,1–4×.
- **Interaktion wie in MuJoCo `simulate`**: Kamera (Drehen/Verschieben/Zoom),
  Doppelklick-Auswahl, **Strg+Ziehen** übt physikalische Kräfte auf Körper aus
  (`mjv_select` / `mjv_movePerturb` / `mjv_applyPerturbForce`).
- **Aktuator-Panel**: Positionsservos je Gelenk mit Live-Reglern.
- **Physik-Panel**: Gravitation, Zeitschritt, Integrator (Euler/RK4/implicit/
  implicitfast), Solver-Iterationen, Reibkegel, Disable-Flags (Kontakte,
  Gravitation, Limits, Aktuatorik, Reibungsverlust) – alles zur Laufzeit.
- **Visualisierung** über die native mjv-Abstraktion: Kontaktpunkte,
  Kontaktkraft-Pfeile, Gelenkachsen, Trägheitsboxen, Schwerpunkte,
  Koordinatenrahmen, Transparenz, konvexe Hüllen; zusätzlich Schatten und
  Drahtgitter im Renderer.
- **Sensorik & Status**: TCP-Position/-Geschwindigkeit (framepos/framelinvel),
  Gelenkwinkel, Antriebsmomente, Energie, Kontaktliste – live.
- **MJCF/XML-Editor** mit Kompilierfehler-Anzeige (Strg+Enter = laden),
  Download, Datei-Upload und Drag&Drop (MJCF und meshfreies URDF).
- **Kamera-Tracking**, Screenshot (PNG) und **Video-Aufnahme** (WebM).
- Beispielszenen: Arbeitszelle, Solo, Pendelkette, Kistenstapel sowie der
  DeepMind-**Humanoid** (`models/humanoid.xml`, Apache-2.0, © DeepMind
  Technologies Limited).

## Bedienung

| Aktion | Eingabe |
|---|---|
| Kamera drehen / verschieben | Linke / rechte Maustaste ziehen |
| Zoom | Mausrad |
| Körper auswählen | Doppelklick |
| Körper ziehen / rotieren | Strg + linke / rechte Maustaste ziehen |
| Pause · Einzelschritt · Reset | Leertaste · S · R |

## Projektstruktur

```
index.html            App-Shell (horstOS-Design, Reiter „Simulation")
css/app.css           Designsystem (Tokens aus horstOS übernommen)
js/engine.js          MuJoCo-Wrapper (Laden, Stepping, Snapshots, Introspektion)
js/renderer.js        mjvScene → three.js (Geometrie-Pooling, Meshes, Pfeile)
js/interaction.js     Kamera, Picking, Kraft-Perturbation
js/scenes.js          HORST600-Generator + Beispielszenen
js/wizard.js          Einrichtungs-Assistent + MJCF-Generator
js/main.js            UI-Controller
vendor/mujoco/        Offizielle WASM-Bindings (Google DeepMind, Apache-2.0)
vendor/three/         three.js (MIT)
models/               horst600_g2.xml, humanoid.xml
```

## Bekannte Grenzen / nächste Schritte

- Die STL-Meshes aus `visuals.xacro` lagen nicht bei – sobald vorhanden,
  können sie als MJCF-`<mesh>`-Assets eingebunden werden (Mesh-Rendering ist
  im Renderer bereits implementiert).
- Höhenfelder (hfield) und Flex-Körper werden noch nicht gezeichnet.
- Multithread-Build (`@mujoco/mujoco/mt`) benötigt COOP/COEP-Header; die
  Single-Thread-Variante läuft überall ohne Sonderkonfiguration.


## CAD-Meshes

Die Optik nutzt die echten HORST600-G2-Geometrien: `tools/build_meshes.mjs`
bereitet die fruitcore-GLB-Exporte auf (Struktur-Meshes liegen dort bereits
im Roboter-Basisframe in mm/Nullpose; Deko wird relativ zur posierten
Träger-Baugruppe zurückgerechnet), fasst pro Link nach Material zusammen
und simplifiziert per meshoptimizer (23 MB → 5,4 MB). Zur Laufzeit legt
`js/robotmesh.js` die 7 Link-Meshes über die MuJoCo-Körper
(`M = [R|p]·T(−t0)`); die Kollisions-Primitive laufen unverändert weiter
und werden nur ausgeblendet (Checkbox „CAD-Meshes"). `horst800_visual.glb`
liegt vorbereitet bei, wird aber erst mit einer HORST800-Kinematik nutzbar.
