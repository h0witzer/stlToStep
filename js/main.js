/**
 * main.js — STL → B-rep STEP converter entry point
 *
 * Workflow:
 *   1. Load STL / OBJ / 3MF  →  Three.js BufferGeometry for preview
 *   2. "Detect Groups"        →  faceGrouper.js (dihedral-angle segmentation)
 *   3. "Fit Surfaces"         →  surfaceFitter.js (plane / cylinder / sphere)
 *   4. "Export STEP"          →  brepBuilder.js  (opencascade.js + STEPControl_Writer)
 */

import * as THREE from 'three';
import {
  initViewer, loadGeometry, setMeshMaterial, setWireframe,
  getCamera, getCurrentMesh, showFaceGroupColors, setViewerTheme,
} from './viewer.js';
import { loadModelFile, computeBounds, getTriangleCount } from './stlLoader.js';
import { t, initLang, setLang, getLang, applyTranslations } from './i18n.js';
import { groupFaces } from './faceGrouper.js';
import { fitAllGroups } from './surfaceFitter.js';
import { initOC, buildAndExportSTEP, downloadSTEP } from './brepBuilder.js';

// ── State ─────────────────────────────────────────────────────────────────────

let currentGeometry = null;
let currentStlName  = 'model';
let currentGroups   = null;   // array of { triangleIndices, surface? }
let isBusy          = false;

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

initViewer(canvas);

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
    dropHint.style.display = 'none';

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
}

// ── Face detection ────────────────────────────────────────────────────────────

async function handleDetect() {
  if (!currentGeometry || isBusy) return;
  setBusy(true, detectBtn);

  await yieldToUI();
  const creaseAngle = parseFloat(creaseSlider.value);

  try {
    currentGroups = groupFaces(currentGeometry, creaseAngle);
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

const TYPE_LABELS = { plane: 'Plane', cylinder: 'Cylinder', sphere: 'Sphere', nurbs: 'NURBS' };
const TYPE_COLORS = { plane: '#4a9eff', cylinder: '#ff9a3c', sphere: '#7dd67d', nurbs: '#c084fc' };

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
    ['auto', 'plane', 'cylinder', 'sphere', 'nurbs'].forEach(opt => {
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
  dropZone.addEventListener('click', e => { if (e.target === dropZone) stlFileInput.click(); });

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
