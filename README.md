# STL → STEP B-rep Converter

A browser-based tool that converts triangulated mesh files (STL / OBJ / 3MF) into true
analytic B-rep **STEP** files — no installation required, all processing happens locally
in your browser.

Instead of exporting a tessellated mesh as STEP (which most converters do), this tool
runs full geometric analysis: it segments the mesh into face groups, fits planes /
cylinders / spheres to each group using PCA, then writes proper analytic OCCT surfaces
via an in-browser OpenCASCADE WASM build.

---

## How to Run

> **⚠️ ES module scripts require an HTTP server — opening `index.html` directly
> as a `file://` URL is unreliable across browsers.**

### Option 1 — Node.js server (recommended, zero dependencies)

```bash
node server.js
```

Then open **<http://localhost:3000>** in Chrome or Edge.

### Option 2 — Python built-in server

```bash
# Python 3
python -m http.server 3000
# Python 2
python -m SimpleHTTPServer 3000
```

Then open **<http://localhost:3000>**.

### Option 3 — VS Code Live Server extension

Install the [Live Server](https://marketplace.visualstudio.com/items?itemName=ritwickdey.LiveServer)
extension, right-click `index.html` → **Open with Live Server**.

---

## Workflow

1. **Load a model** — drag-and-drop an `.stl`, `.obj`, or `.3mf` onto the viewport,
   or use the **Load Model…** button in the sidebar.
2. **Detect Groups** — adjust the *Crease Angle* slider and click **Detect Groups**.
   The tool uses BFS flood-fill to segment triangles into face groups whose dihedral
   angles stay within the threshold. Groups are shown in random colours.
3. **Fit Surfaces** — click **Fit Surfaces**. Each group is classified as a
   **Plane**, **Cylinder**, **Sphere**, or **NURBS** (fallback) using
   PCA / least-squares fitting. You can override individual groups via the dropdown.
4. **Export STEP** — click **Export STEP**. On the first run this downloads the
   ~25 MB OpenCASCADE WASM bundle from the CDN and caches it in the browser;
   subsequent exports are instant. A `.stp` file is saved to your downloads folder.

---

## Features

- **Analytic B-rep output** — planes, cylinders, and spheres become true OCCT analytic
  surfaces, not faceted meshes.
- **NURBS fallback** — groups that don't fit a standard quadric use a wire-based face.
- **B-rep sewing** — OpenCASCADE's `BRepBuilderAPI_Sewing` joins adjacent faces into a
  manifold shell; `ShapeFix_Solid` attempts promotion to a solid.
- **STEP schema choice** — AP214 (recommended) or AP203 (maximum CAD compatibility).
- **Sewing tolerance control** — adjustable from 1e-6 to 1e-2 mm.
- **3D viewer** — orthographic camera, orbit / pan / zoom, wireframe overlay,
  bounding-box dimensions, coordinate axes.
- **Light / Dark theme** — respects OS preference, persisted per browser.
- **English / German UI** — auto-detected from browser language.

---

## File Support

| Format | Notes |
|--------|-------|
| `.stl` | Binary and ASCII |
| `.obj` | Multi-mesh OBJ; meshes are merged before analysis |
| `.3mf` | Bambu Studio / PrusaSlicer multi-file 3MF via custom ZIP parser |

---

## Project Structure

```
index.html          — Main entry point
style.css           — Styles (light / dark theme)
server.js           — Zero-dependency Node.js dev server
js/
  main.js           — App bootstrap & UI wiring
  viewer.js         — Three.js scene / camera / controls
  stlLoader.js      — STL / OBJ / 3MF loaders
  faceGrouper.js    — BFS face segmentation by dihedral angle
  surfaceFitter.js  — PCA surface classification (plane / cylinder / sphere)
  brepBuilder.js    — OpenCASCADE B-rep assembly + STEPControl_Writer
  exclusion.js      — Adjacency graph & bucket-fill utilities
  exporter.js       — Binary STL export helper (unused in STEP branch)
  i18n.js           — EN / DE translations
vendor/
  three/            — Three.js v0.170.0 (bundled locally for file:// compatibility)
  fflate/           — fflate v0.8.2 (ZIP decompression for .3mf)
```

---

## Dependencies

| Library | Version | How loaded |
|---------|---------|------------|
| [Three.js](https://threejs.org/) | 0.170.0 | Bundled in `vendor/` |
| [fflate](https://github.com/101arrowz/fflate) | 0.8.2 | Bundled in `vendor/` |
| [opencascade.js](https://github.com/donalffons/opencascade.js) | 2.0.0-pre.4 | Fetched from CDN on first STEP export (~25 MB, then cached) |

All processing runs entirely in the browser — no data is uploaded to any server.

---

## License

MIT — see [LICENSE](LICENSE).
