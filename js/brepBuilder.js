/**
 * brepBuilder.js — B-rep topology assembly via opencascade.js + STEP export
 *
 * Pipeline per face group:
 *   1. Extract ordered boundary loop from mesh edges (faceGrouper.js)
 *   2. Snap boundary vertices onto the fitted analytic surface
 *   3. Build an OCCT wire from straight 3D edges
 *   4. Build an OCCT face: plane uses analytic gp_Pln;
 *      cylinder/cone/sphere/NURBS fall back to BRepBuilderAPI_MakeFace (wire-only)
 *   5. Assemble all faces into a TopoDS_Compound
 *   6. Write STEP via STEPControl_Writer; chdir('/') first so the bare filename
 *      is always resolved to '/', then pre-create the inode for O_WRONLY safety
 *
 * opencascade.js is loaded lazily via dynamic import() when the user first
 * clicks "Export STEP" so the 35 MB WASM does not block page load.
 */

import { extractBoundaryLoop } from './faceGrouper.js';

// ── Build version ─────────────────────────────────────────────────────────────

/** Increment this string with each release to verify live-site deployments. */
export const BUILD_VERSION = 'v0.4.3';

// ── OpenCASCADE lazy loader ───────────────────────────────────────────────────

// jsdelivr returns HTTP 403 for the .wasm binary; unpkg serves all file types.
const OC_CDN = 'https://unpkg.com/opencascade.js@1.1.4/dist/';

let _oc = null;
let _ocPromise = null;

/**
 * Lazily load the opencascade.js WASM bundle.
 * Safe to call multiple times — only loads once.
 *
 * @param {function(string):void} [onStatus]  progress callback
 * @returns {Promise<object>}  the OCCT module
 */
export async function initOC(onStatus) {
  if (_oc) return _oc;
  if (_ocPromise) return _ocPromise;

  _ocPromise = (async () => {
    onStatus?.('Downloading OpenCASCADE geometry engine (~25 MB, first load only)…');

    // opencascade.js@1.1.4 ships as an ES module, so dynamic import() is the
    // correct loader. The module's default export is the factory function.
    const ocMod = await import(/* @vite-ignore */ OC_CDN + 'opencascade.wasm.js');
    const factory = ocMod.default ?? ocMod.opencascade;
    if (typeof factory !== 'function') {
      throw new Error(
        'Could not find the opencascade factory in the loaded module. ' +
        'Check that unpkg.com is reachable and the correct version is being loaded.'
      );
    }

    onStatus?.('Initialising OpenCASCADE…');
    _oc = await factory({
      locateFile: (file) => OC_CDN + file,
      print:    () => {},
      printErr: () => {},
    });
    onStatus?.('OpenCASCADE ready.');
    return _oc;
  })();
  _ocPromise.catch(() => { _ocPromise = null; });

  return _ocPromise;
}

// ── Geometry helpers ─────────────────────────────────────────────────────────

function makePnt(oc, p)  { return new oc.gp_Pnt_3(p[0], p[1], p[2]); }
function makeDir(oc, v)  {
  const n = Math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
  return new oc.gp_Dir_4(v[0]/n, v[1]/n, v[2]/n);
}

function makeAx3(oc, origin, normal) {
  return new oc.gp_Ax3_4(makePnt(oc, origin), makeDir(oc, normal));
}

// ── Surface snapping helpers ─────────────────────────────────────────────────

/** Project point p onto the plane defined by (origin, normal). */
function snapToPlane(p, origin, normal) {
  const d = (p[0]-origin[0])*normal[0]+(p[1]-origin[1])*normal[1]+(p[2]-origin[2])*normal[2];
  return [p[0]-d*normal[0], p[1]-d*normal[1], p[2]-d*normal[2]];
}

/** Project point p onto the cylinder surface (axisPoint, axis, radius). */
function snapToCylinder(p, axisPoint, axis, radius) {
  const dp = [p[0]-axisPoint[0], p[1]-axisPoint[1], p[2]-axisPoint[2]];
  const axComp = dp[0]*axis[0]+dp[1]*axis[1]+dp[2]*axis[2];
  const perp = [dp[0]-axComp*axis[0], dp[1]-axComp*axis[1], dp[2]-axComp*axis[2]];
  const dist = Math.sqrt(perp[0]*perp[0]+perp[1]*perp[1]+perp[2]*perp[2]);
  if (dist < 1e-14) return p;
  const s = radius / dist;
  return [
    axisPoint[0] + axComp*axis[0] + perp[0]*s,
    axisPoint[1] + axComp*axis[1] + perp[1]*s,
    axisPoint[2] + axComp*axis[2] + perp[2]*s,
  ];
}

/** Project point p onto the sphere surface (center, radius). */
function snapToSphere(p, center, radius) {
  const dp = [p[0]-center[0], p[1]-center[1], p[2]-center[2]];
  const dist = Math.sqrt(dp[0]*dp[0]+dp[1]*dp[1]+dp[2]*dp[2]);
  if (dist < 1e-14) return p;
  const s = radius / dist;
  return [center[0]+dp[0]*s, center[1]+dp[1]*s, center[2]+dp[2]*s];
}

/** Project point p onto the cone surface (apex, axis, halfAngle). */
function snapToCone(p, apex, axis, halfAngle) {
  const tanA = Math.tan(halfAngle);
  const dp = [p[0]-apex[0], p[1]-apex[1], p[2]-apex[2]];
  const az = dp[0]*axis[0]+dp[1]*axis[1]+dp[2]*axis[2];
  const rx = dp[0]-az*axis[0], ry = dp[1]-az*axis[1], rz = dp[2]-az*axis[2];
  const r = Math.sqrt(rx*rx+ry*ry+rz*rz);
  const targetR = Math.abs(az) * tanA;
  if (r < 1e-14) return p; // on axis — can't determine radial direction
  const s = targetR / r;
  return [
    apex[0] + az*axis[0] + rx*s,
    apex[1] + az*axis[1] + ry*s,
    apex[2] + az*axis[2] + rz*s,
  ];
}

// ── Wire builder ─────────────────────────────────────────────────────────────

/**
 * Build an OCCT wire from an ordered list of 3D points (linear edges).
 * Skips zero-length edges.  Returns null if the wire cannot be built.
 *
 * @param {object} oc       OCCT module
 * @param {Array}  loop     [[x,y,z], …]
 * @param {object[]} toDelete  objects that need .delete() later
 * @returns {object|null}   TopoDS_Wire or null
 */
function buildWire(oc, loop, toDelete) {
  if (!loop || loop.length < 3) return null;

  const wireMaker = new oc.BRepBuilderAPI_MakeWire_1();
  toDelete.push(wireMaker);

  let edgeCount = 0;
  for (let i = 0; i < loop.length; i++) {
    const a = loop[i], b = loop[(i + 1) % loop.length];
    const dx=b[0]-a[0], dy=b[1]-a[1], dz=b[2]-a[2];
    if (dx*dx+dy*dy+dz*dz < 1e-20) continue; // skip zero-length

    const pa = makePnt(oc, a); toDelete.push(pa);
    const pb = makePnt(oc, b); toDelete.push(pb);
    const edgeMaker = new oc.BRepBuilderAPI_MakeEdge_3(pa, pb);
    toDelete.push(edgeMaker);
    if (!edgeMaker.IsDone()) continue;
    const edge = edgeMaker.Edge();
    wireMaker.Add_1(edge);
    edgeCount++;
  }

  if (edgeCount < 3 || !wireMaker.IsDone()) return null;
  return wireMaker.Wire();
}

// ── Face builder ─────────────────────────────────────────────────────────────

/**
 * Build a single OCCT face from a classified face group.
 *
 * @returns {object|null}  TopoDS_Face or null
 */
function buildFace(oc, group, geometry, toDelete) {
  const { type, params } = group.surface;

  // Extract boundary loop from mesh topology
  const rawLoop = extractBoundaryLoop(geometry, group.triangleIndices);
  if (!rawLoop || rawLoop.length < 3) return null;

  // Snap vertices onto fitted surface so they lie exactly on it
  let snappedLoop;
  if (type === 'plane') {
    const { origin, normal } = params;
    snappedLoop = rawLoop.map(p => snapToPlane(p, origin, normal));
  } else if (type === 'cylinder') {
    const { axisPoint, axis, radius } = params;
    snappedLoop = rawLoop.map(p => snapToCylinder(p, axisPoint, axis, radius));
  } else if (type === 'cone') {
    const { apex, axis, halfAngle } = params;
    snappedLoop = rawLoop.map(p => snapToCone(p, apex, axis, halfAngle));
  } else if (type === 'sphere') {
    const { center, radius } = params;
    snappedLoop = rawLoop.map(p => snapToSphere(p, center, radius));
  } else {
    snappedLoop = rawLoop; // NURBS: use raw boundary
  }

  // Deduplicate consecutive snapped points
  const dedupLoop = [snappedLoop[0]];
  for (let i = 1; i < snappedLoop.length; i++) {
    const a = dedupLoop[dedupLoop.length-1], b = snappedLoop[i];
    const dx=b[0]-a[0], dy=b[1]-a[1], dz=b[2]-a[2];
    if (dx*dx+dy*dy+dz*dz > 1e-20) dedupLoop.push(b);
  }
  if (dedupLoop.length < 3) return null;

  if (type === 'plane') {
    return _buildPlaneFace(oc, params, dedupLoop, toDelete);
  } else if (type === 'cylinder') {
    return _buildCylinderFace(oc, params, dedupLoop, toDelete);
  } else if (type === 'cone') {
    return _buildConeFace(oc, params, dedupLoop, toDelete);
  } else if (type === 'sphere') {
    return _buildSphereFace(oc, params, dedupLoop, toDelete);
  } else {
    return _buildFallbackFace(oc, dedupLoop, toDelete);
  }
}

function _buildPlaneFace(oc, params, loop, toDelete) {
  const { origin, normal } = params;
  const pln = new oc.gp_Pln_3(makePnt(oc, origin), makeDir(oc, normal));
  toDelete.push(pln);

  const wire = buildWire(oc, loop, toDelete);
  if (!wire) return null;

  try {
    const mf = new oc.BRepBuilderAPI_MakeFace_16(pln, wire, true);
    toDelete.push(mf);
    if (!mf.IsDone()) return _buildFallbackFace(oc, loop, toDelete);
    return mf.Face();
  } catch {
    return _buildFallbackFace(oc, loop, toDelete);
  }
}

function _buildCylinderFace(oc, params, loop, toDelete) {
  // Straight 3D line segments between snapped vertices do not lie on the
  // cylinder surface, so BRepBuilderAPI_MakeFace_17 would compute degenerate
  // PCurves that silently pass IsDone() but crash inside STEPControl_Writer.
  // Use a planar best-fit face (wire-only) instead.
  return _buildFallbackFace(oc, loop, toDelete);
}

function _buildConeFace(oc, params, loop, toDelete) {
  // Straight 3D edges between snapped vertices are not generators of the cone,
  // so they don't lie exactly on the cone surface. BRepBuilderAPI_MakeFace with
  // a Geom_ConicalSurface would produce degenerate PCurves. Use the wire-only
  // planar fallback — correct boundary shape, no null-PCurve crash.
  return _buildFallbackFace(oc, loop, toDelete);
}

function _buildSphereFace(oc, params, loop, toDelete) {
  // Same reasoning as cylinder: straight edges between snapped vertices lie
  // inside the sphere, not on it. Skip MakeFace_19 and use the planar fallback.
  return _buildFallbackFace(oc, loop, toDelete);
}

/**
 * Fallback: build a best-fit planar face from the wire boundary alone.
 * OCCT automatically computes the best-fit plane from the wire's vertices.
 */
function _buildFallbackFace(oc, loop, toDelete) {
  const wire = buildWire(oc, loop, toDelete);
  if (!wire) return null;
  try {
    const mf = new oc.BRepBuilderAPI_MakeFace_15(wire, false);
    toDelete.push(mf);
    return mf.IsDone() ? mf.Face() : null;
  } catch {
    return null;
  }
}

// ── Main export: build B-rep + STEP ─────────────────────────────────────────

/**
 * Build a B-rep shell/solid from annotated face groups and export as STEP.
 *
 * @param {Array<{triangleIndices,surface}>} groups   annotated by fitAllGroups()
 * @param {THREE.BufferGeometry}             geometry  source mesh
 * @param {{ schema: 'AP203'|'AP214', sewTol: number }} options
 * @param {function(string,number):void} [onStatus]   (message, pct 0–100)
 * @returns {Promise<string>}  STEP file content as UTF-8 string
 */
export async function buildAndExportSTEP(groups, geometry, options = {}, onStatus) {
  const { schema = 'AP214' } = options;

  const oc = await initOC(msg => onStatus?.(msg, 5));

  onStatus?.('Building B-rep faces…', 20);

  const toDelete = []; // OCCT objects to free after export
  const faces = [];

  for (let i = 0; i < groups.length; i++) {
    const g = groups[i];
    if (!g.surface) continue;
    try {
      const face = buildFace(oc, g, geometry, toDelete);
      if (face) faces.push(face);
    } catch (err) {
      console.warn(`Face ${i} (${g.surface.type}) failed:`, err);
    }
    if (i % 50 === 0) onStatus?.(`Building faces… ${i}/${groups.length}`, 20 + 30 * i / groups.length);
  }

  if (faces.length === 0) throw new Error('No valid B-rep faces could be constructed.');

  onStatus?.(`Assembling ${faces.length} faces…`, 55);

  // Assemble all faces into a compound. All faces are built with MakeFace_15 (wire-only
  // best-fit plane) or MakeFace_16 (analytic plane), so they have valid PCurves and can
  // be transferred to STEP in a single call without triggering OCCT null-pointer traps.
  const builder = new oc.BRep_Builder();
  const compound = new oc.TopoDS_Compound();
  toDelete.push(compound);
  builder.MakeCompound(compound);
  for (const f of faces) builder.Add(compound, f);

  onStatus?.('Writing STEP file…', 80);

  // Set STEP schema
  try {
    oc.Interface_Static_SetCVal(
      'write.step.schema',
      schema === 'AP214' ? 'AP214IS' : 'AP203',
    );
  } catch { /* optional */ }

  const writer = new oc.STEPControl_Writer_1();
  toDelete.push(writer);

  // IFSelect_RetDone = 1 in all OCCT versions; also accept the enum object form.
  const DONE = oc.IFSelect_ReturnStatus?.IFSelect_RetDone ?? 1;

  const transferResult = writer.Transfer(
    compound,
    oc.STEPControl_StepModelType?.STEPControl_AsIs ?? 0,
    true,
  );

  if (transferResult !== DONE) {
    throw new Error(`STEPControl_Writer.Transfer failed (status ${transferResult}).`);
  }

  // Pin the process CWD to '/' so bare filenames resolve predictably.
  // In some OCCT/Emscripten builds the CWD is initialised to a non-root path
  // (e.g. '/home/web_user') which causes Write to produce a file that is
  // invisible from a '/' lookup.
  const stepFile = 'brep_export.stp';
  const stepPath = '/' + stepFile;
  const savedCwd = typeof oc.FS.cwd === 'function' ? oc.FS.cwd() : '/';
  try { oc.FS.chdir('/'); } catch { /* not fatal — best effort */ }

  // Clean up any leftover from a previous failed export, then pre-create the
  // inode so that OSD_File can open it with O_WRONLY even if OCCT's libc open()
  // omits O_CREAT in the Emscripten build.
  try { oc.FS.unlink(stepPath); } catch { /* ignore */ }
  oc.FS.writeFile(stepPath, '');

  const writeResult = writer.Write(stepFile);

  // Restore the CWD regardless of outcome
  try { if (savedCwd !== '/') oc.FS.chdir(savedCwd); } catch { /* ignore */ }

  if (writeResult !== DONE) {
    throw new Error(`STEPControl_Writer.Write failed (status ${writeResult}).`);
  }

  let stepContent;
  try {
    const raw = oc.FS.readFile(stepPath);
    // readFile returns a Uint8Array by default in Emscripten; decode it.
    const text = typeof raw === 'string' ? raw : new TextDecoder().decode(raw);
    // Allow for a leading BOM (U+FEFF) or stray whitespace before the header.
    if (text && text.includes('ISO-10303')) {
      stepContent = text;
    }
  } catch { /* fall through to error below */ }

  try { oc.FS.unlink(stepPath); } catch { /* ignore */ }

  if (!stepContent) {
    // Diagnostic: log MEMFS state so developers can pinpoint the issue
    try {
      const cwd = typeof oc.FS.cwd === 'function' ? oc.FS.cwd() : '?';
      console.error('STEP Write diagnostics — CWD:', cwd,
        '/ contents:', oc.FS.readdir('/'));
    } catch { /* ignore */ }
    throw new Error('STEP export produced empty or invalid output. Transfer returned DONE but no ISO-10303 header found.');
  }

  onStatus?.('Cleaning up…', 95);

  // Free C++ objects (reverse order to respect OCCT ownership)
  for (let i = toDelete.length - 1; i >= 0; i--) {
    try { toDelete[i].delete?.(); } catch { /* ignore */ }
  }

  onStatus?.('Done.', 100);
  return stepContent;
}

/**
 * Trigger a browser download of the given text content as a .stp file.
 *
 * @param {string} content   STEP file text
 * @param {string} filename  e.g. 'model.stp'
 */
export function downloadSTEP(content, filename = 'model.stp') {
  const blob = new Blob([content], { type: 'application/octet-stream' });
  const url  = URL.createObjectURL(blob);
  const a    = document.createElement('a');
  a.href = url; a.download = filename; a.style.display = 'none';
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  setTimeout(() => URL.revokeObjectURL(url), 10_000);
}
