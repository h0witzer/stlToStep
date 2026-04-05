/**
 * main.js — STL → B-rep STEP converter entry point
 *
 * Workflow:
 *   1. Load STL / OBJ / 3MF  →  Three.js BufferGeometry for preview
 *   2. "Detect Groups"        →  faceGrouper.js (dihedral-angle segmentation)
 *   3. "Fit Surfaces"         →  surfaceFitter.js (plane / cylinder / cone / sphere)
 *   4. "Export STEP"          →  brepBuilder.js  (opencascade.js + STEPControl_Writer)
 */

import * as THREE from 'three';
import {
  initViewer, loadGeometry, setMeshMaterial, setWireframe,
  getCamera, getCurrentMesh, showFaceGroupColors, setViewerTheme,
  setGroupHighlight, setGroupHoverCallback,
} from './viewer.js';
import { loadModelFile, computeBounds, getTriangleCount } from './stlLoader.js';
import { t, initLang, setLang, getLang, applyTranslations } from './i18n.js';
import { groupFaces } from './faceGrouper.js';
import { fitAllGroups } from './surfaceFitter.js';
import { initOC, buildAndExportSTEP, downloadSTEP, BUILD_VERSION } from './brepBuilder.js';

// ── State ─────────────────────────────────────────────────────────────────────

let currentGeometry = null;
let currentStlName  = 'model';
let currentGroups   = null;   // array of { triangleIndices, surface? }
let isBusy          = false;

// Reverse map: triangleIndex → groupIndex (rebuilt after each Detect Groups run)
let _triangleToGroup = null;
// The currently highlighted group index (-1 = none)
let _highlightedGroup = -1;

// ── DOM refs ─────────────────────────────────────────────────────────────────

const canvas        = document.getElementById('viewport');
const dropZone      = document.getElementById('drop-zone');
const dropHint      = document.getElementById('drop-hint');
const stlFileInput  = document.getElementById('stl-file-input');
const meshInfo      = document.getElementById('mesh-info');
const wireframeToggle = document.getElementById('wireframe-toggle');

// Face detection panel
const creaseSlider  = document.getElementById('crease-angle');
const creaseVal     = document.getElementById('crease-angle-val');
const detectBtn     = document.getElementById('detect-btn');
const groupCount    = document.getElementById('group-count');

// Surface fitting panel
const fitBtn        = document.getElementById('fit-btn');
const surfaceList   = document.getElementById('surface-list');

// Export panel
const schemaSelect  = document.getElementById('step-schema');
const sewTolSlider  = document.getElementById('sew-tolerance');
const sewTolVal     = document.getElementById('sew-tolerance-val');
const exportBtn     = document.getElementById('export-btn');
const exportProgress = document.getElementById('export-progress');
const exportProgBar  = document.getElementById('export-progress-bar');
const exportProgLbl  = document.getElementById('export-progress-label');

// ── Init ──────────────────────────────────────────────────────────────────────

// Stamp the build version badge in the header
(function () {
  const badge = document.getElementById('build-badge');
  if (badge) badge.textContent = BUILD_VERSION;
})();

initViewer(canvas);

// Wire up the canvas hover callback: maps triangle index → group index → highlight
setGroupHoverCallback((triangleIdx) => {
  if (!_triangleToGroup || !currentGroups) return;
  const groupIdx = triangleIdx >= 0 ? _triangleToGroup[triangleIdx] : -1;
  if (groupIdx === _highlightedGroup) return;
  _highlightedGroup = groupIdx;
  // Update viewport overlay
  if (groupIdx >= 0 && currentGroups[groupIdx]) {
    setGroupHighlight(currentGroups[groupIdx].triangleIndices, currentGeometry);
  } else {
    setGroupHighlight(null, null);
  }
  // Sync the sidebar row highlight
  document.querySelectorAll('.surface-row').forEach((row, i) => {
    row.classList.toggle('viewport-hover', i === groupIdx);
  });
  // Scroll the highlighted row into view if it's offscreen
  if (groupIdx >= 0) {
    const rows = document.querySelectorAll('.surface-row');
    if (rows[groupIdx]) rows[groupIdx].scrollIntoView({ block: 'nearest' });
  }
});

// Apply saved theme
setViewerTheme(document.documentElement.getAttribute('data-theme') === 'light');

// Language
initLang();
(function () {
  const lang = getLang();
  document.querySelectorAll('.lang-btn').forEach(btn =>
    btn.classList.toggle('active', btn.dataset.langCode === lang));
})();

// Theme toggle
document.getElementById('theme-toggle').addEventListener('click', () => {
  const isLight = document.documentElement.getAttribute('data-theme') !== 'light';
  document.documentElement.setAttribute('data-theme', isLight ? 'light' : 'dark');
  localStorage.setItem('stlt-theme', isLight ? 'light' : 'dark');
  setViewerTheme(isLight);
});

// Language buttons
document.querySelectorAll('.lang-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    const lang = btn.dataset.langCode;
    setLang(lang);
    document.querySelectorAll('.lang-btn').forEach(b =>
      b.classList.toggle('active', b.dataset.langCode === lang));
  });
});

wireEvents();

// ── Model loading ─────────────────────────────────────────────────────────────

async function handleModelFile(file) {
  if (isBusy) return;
  currentStlName = file.name.replace(/\.[^.]+$/, '');
  try {
    const { geometry } = await loadModelFile(file);
    currentGeometry = geometry;
    currentGroups = null;

    loadGeometry(geometry);
    dropHint.classList.add('loaded');   // hide hint via CSS (does not affect the file input)

    const triCount = getTriangleCount(geometry);
    geometry.computeBoundingBox();
    const box  = geometry.boundingBox;
    const size = new THREE.Vector3();
    box.getSize(size);
    meshInfo.textContent =
      `${triCount.toLocaleString()} triangles · ` +
      `${size.x.toFixed(1)} × ${size.y.toFixed(1)} × ${size.z.toFixed(1)} mm`;

    resetPanels();
    detectBtn.disabled = false;
    exportBtn.disabled = true;
  } catch (err) {
    alert(`Could not load file: ${err.message}`);
  }
}

function resetPanels() {
  groupCount.textContent = '';
  surfaceList.innerHTML  = '';
  exportProgress.classList.add('hidden');
  fitBtn.disabled = true;
  exportBtn.disabled = true;
  // Clear any lingering hover state
  setGroupHighlight(null, null);
  _triangleToGroup = null;
  _highlightedGroup = -1;
}

// Build a fast reverse-lookup array: triangleIndex → groupIndex.
// Called after groupFaces() so the canvas hover can identify the hovered group.
function _buildTriangleGroupMap() {
  if (!currentGeometry || !currentGroups) return;
  const triCount = currentGeometry.attributes.position.count / 3;
  _triangleToGroup = new Int32Array(triCount).fill(-1);
  currentGroups.forEach((g, gi) => {
    for (const t of g.triangleIndices) _triangleToGroup[t] = gi;
  });
}

// ── Face detection ────────────────────────────────────────────────────────────

async function handleDetect() {
  if (!currentGeometry || isBusy) return;
  setBusy(true, detectBtn);

  await yieldToUI();
  const creaseAngle = parseFloat(creaseSlider.value);

  try {
    currentGroups = groupFaces(currentGeometry, creaseAngle);
    _buildTriangleGroupMap();
    showFaceGroupColors(currentGeometry, currentGroups);
    groupCount.textContent = `${currentGroups.length} group${currentGroups.length !== 1 ? 's' : ''} detected`;
    fitBtn.disabled = false;
    exportBtn.disabled = true;
    surfaceList.innerHTML = '';
  } catch (err) {
    console.error('Face detection failed:', err);
    alert(`Face detection failed: ${err.message}`);
  } finally {
    setBusy(false, detectBtn);
  }
}

// ── Surface fitting ───────────────────────────────────────────────────────────

async function handleFit() {
  if (!currentGroups || isBusy) return;
  setBusy(true, fitBtn);

  await yieldToUI();

  try {
    fitAllGroups(currentGroups, currentGeometry);
    renderSurfaceList();
    exportBtn.disabled = false;
  } catch (err) {
    console.error('Surface fitting failed:', err);
    alert(`Surface fitting failed: ${err.message}`);
  } finally {
    setBusy(false, fitBtn);
  }
}

const TYPE_LABELS = { plane: 'Plane', cylinder: 'Cylinder', cone: 'Cone', sphere: 'Sphere', nurbs: 'NURBS' };
const TYPE_COLORS = { plane: '#4a9eff', cylinder: '#ff9a3c', cone: '#f97316', sphere: '#7dd67d', nurbs: '#c084fc' };

function renderSurfaceList() {
  surfaceList.innerHTML = '';

  // Tally surface types
  const tally = {};
  for (const g of currentGroups) {
    const type = g.surface?.type ?? 'unknown';
    tally[type] = (tally[type] || 0) + 1;
  }

  // Summary chips
  const chips = document.createElement('div');
  chips.className = 'surface-chips';
  for (const [type, count] of Object.entries(tally)) {
    const chip = document.createElement('span');
    chip.className = 'surface-chip';
    chip.style.setProperty('--chip-color', TYPE_COLORS[type] ?? '#aaa');
    chip.textContent = `${count} × ${TYPE_LABELS[type] ?? type}`;
    chips.appendChild(chip);
  }
  surfaceList.appendChild(chips);

  // Per-group rows (cap at 200 to avoid DOM overload)
  const MAX_ROWS = 200;
  const shown = currentGroups.slice(0, MAX_ROWS);
  shown.forEach((g, i) => {
    const type  = g.surface?.type ?? 'unknown';
    const label = TYPE_LABELS[type] ?? type;
    const color = TYPE_COLORS[type] ?? '#aaa';
    const rms   = g.surface?.rms ?? 0;

    const row = document.createElement('div');
    row.className = 'surface-row';

    const dot = document.createElement('span');
    dot.className = 'surface-dot';
    dot.style.background = color;

    const name = document.createElement('span');
    name.className = 'surface-name';
    name.textContent = `Group ${i + 1} — ${label}`;

    const rmsEl = document.createElement('span');
    rmsEl.className = 'surface-rms';
    rmsEl.textContent = isFinite(rms) ? `RMS ${rms.toFixed(4)}` : '';

    // Manual override select
    const sel = document.createElement('select');
    sel.className = 'surface-override';
    ['auto', 'plane', 'cylinder', 'cone', 'sphere', 'nurbs'].forEach(opt => {
      const o = document.createElement('option');
      o.value = opt; o.textContent = opt === 'auto' ? 'auto' : TYPE_LABELS[opt];
      if ((type === opt) || (opt === 'auto' && !g._override)) o.selected = true;
      sel.appendChild(o);
    });
    sel.addEventListener('change', () => {
      const val = sel.value;
      if (val === 'auto') { delete g._override; g.surface = g._autoSurface; }
      else {
        g._autoSurface = g._autoSurface ?? g.surface;
        g._override    = val;
        g.surface      = { ...g.surface, type: val };
      }
    });
    // Remember the auto classification
    g._autoSurface = g._autoSurface ?? g.surface;

    row.append(dot, name, rmsEl, sel);

    // Sidebar → viewport hover: highlight this group in the 3D viewer
    row.addEventListener('mouseenter', () => {
      setGroupHighlight(g.triangleIndices, currentGeometry);
      _highlightedGroup = i;
      row.classList.add('sidebar-hover');
    });
    row.addEventListener('mouseleave', () => {
      setGroupHighlight(null, null);
      _highlightedGroup = -1;
      row.classList.remove('sidebar-hover');
    });

    surfaceList.appendChild(row);
  });

  if (currentGroups.length > MAX_ROWS) {
    const more = document.createElement('p');
    more.className = 'surface-more';
    more.textContent = `… and ${currentGroups.length - MAX_ROWS} more groups`;
    surfaceList.appendChild(more);
  }
}

// ── STEP export ───────────────────────────────────────────────────────────────

async function handleExport() {
  if (!currentGroups || isBusy) return;
  const hasSurfaces = currentGroups.some(g => g.surface);
  if (!hasSurfaces) {
    alert('Run "Fit Surfaces" before exporting.');
    return;
  }

  setBusy(true, exportBtn);
  exportProgress.classList.remove('hidden');
  setProgress('', 0);

  try {
    const schema = schemaSelect.value;
    const sewTol = parseSewTol();

    const stepContent = await buildAndExportSTEP(
      currentGroups,
      currentGeometry,
      { schema, sewTol },
      (msg, pct) => setProgress(msg, pct),
    );

    downloadSTEP(stepContent, `${currentStlName}.stp`);
  } catch (err) {
    console.error('Export failed:', err);
    alert(`Export failed: ${err.message}`);
  } finally {
    setBusy(false, exportBtn);
    setTimeout(() => exportProgress.classList.add('hidden'), 2000);
  }
}

function parseSewTol() {
  const v = parseFloat(sewTolSlider.value);
  return isFinite(v) ? Math.pow(10, v) : 1e-4;
}

function setProgress(msg, pct) {
  exportProgBar.style.width = `${Math.max(0, Math.min(100, pct))}%`;
  exportProgLbl.textContent = msg;
}

// ── Event wiring ──────────────────────────────────────────────────────────────

function wireEvents() {
  // Model loading
  stlFileInput.addEventListener('change', e => {
    if (e.target.files[0]) handleModelFile(e.target.files[0]);
  });

  // Drag & drop
  dropZone.addEventListener('dragover', e => { e.preventDefault(); dropZone.classList.add('drag-over'); });
  dropZone.addEventListener('dragleave', () => dropZone.classList.remove('drag-over'));
  dropZone.addEventListener('drop', e => {
    e.preventDefault(); dropZone.classList.remove('drag-over');
    const file = [...e.dataTransfer.files].find(f => /\.(stl|obj|3mf)$/i.test(f.name));
    if (file) handleModelFile(file);
  });

  // Clicking the viewport (canvas) or the drop-zone background opens the file picker.
  // The canvas is positioned absolute on top of the drop-zone, so we listen on the
  // canvas directly.  We suppress the click during OrbitControls drags by checking
  // that the pointer hasn't moved more than a few pixels between pointerdown and click.
  let _pointerDownPos = null;
  canvas.addEventListener('pointerdown', e => {
    _pointerDownPos = { x: e.clientX, y: e.clientY };
  });
  canvas.addEventListener('click', e => {
    if (!currentGeometry && _pointerDownPos) {
      const dx = e.clientX - _pointerDownPos.x;
      const dy = e.clientY - _pointerDownPos.y;
      if (dx * dx + dy * dy < 25) stlFileInput.click(); // < 5px movement = genuine click
    }
    _pointerDownPos = null;
  });

  // Crease angle slider
  creaseSlider.addEventListener('input', () => {
    creaseVal.value = creaseSlider.value;
  });
  creaseVal.addEventListener('change', () => {
    creaseSlider.value = Math.max(0, Math.min(90, parseFloat(creaseVal.value) || 30));
    creaseVal.value = creaseSlider.value;
  });

  // Action buttons
  detectBtn.addEventListener('click', handleDetect);
  fitBtn.addEventListener('click', handleFit);
  exportBtn.addEventListener('click', handleExport);

  // Sewing tolerance display
  sewTolSlider.addEventListener('input', () => {
    const exp = parseFloat(sewTolSlider.value);
    sewTolVal.textContent = `1e${exp}`;
  });

  // Wireframe
  wireframeToggle.addEventListener('change', () => setWireframe(wireframeToggle.checked));

  // License overlay
  const licenseLink    = document.getElementById('license-link');
  const licenseOverlay = document.getElementById('license-overlay');
  const licenseClose   = document.getElementById('license-close');
  if (licenseLink) {
    licenseLink.addEventListener('click', () => licenseOverlay.classList.remove('hidden'));
    licenseClose.addEventListener('click', () => licenseOverlay.classList.add('hidden'));
    licenseOverlay.addEventListener('click', e => {
      if (e.target === licenseOverlay) licenseOverlay.classList.add('hidden');
    });
  }
}

// ── Utilities ─────────────────────────────────────────────────────────────────

function setBusy(busy, primaryBtn) {
  isBusy = busy;
  if (primaryBtn) primaryBtn.disabled = busy;
}

function yieldToUI() {
  return new Promise(resolve => setTimeout(resolve, 10));
}
