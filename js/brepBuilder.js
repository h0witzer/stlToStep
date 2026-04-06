/**
 * brepBuilder.js — B-rep topology assembly via opencascade.js + STEP export
 *
 * Pipeline:
 *   1. Build one generously oversized analytical face per fitted surface group.
 *      All surface types (plane, cylinder, cone, sphere) use _buildLargePatch()
 *      with margins proportional to the model bounding-box diagonal so every
 *      face extends well beyond any possible intersection with its neighbours.
 *   2. Compute exact analytical intersection curves between every pair of
 *      adjacent faces using BRepAlgoAPI_Section.  These are true surface–
 *      surface intersections (circles for plane∩cylinder, lines for
 *      plane∩plane, conics for cone cases, etc.) — no mesh data involved.
 *   3. Trim each face using the section curves:
 *        • Plane faces: assemble section edges into closed wires, then
 *          BRepBuilderAPI_MakeFace(gp_Pln, outerWire).  Inner wires (from
 *          holes) are added via MakeFace.Add().
 *        • Curved faces (cylinder, cone, sphere): project section edges onto
 *          the surface axis to determine V-parameter bounds, then
 *          MakeFace(gp_Cylinder/Cone/Sphere, 0, 2π, Vmin, Vmax).
 *      This generalises to fillets and arbitrary NURBS surfaces — section
 *      edges give exact trim curves for any pair of analytical or freeform
 *      surfaces.
 *   4. Sew the trimmed faces into a watertight shell → solid.
 *   5. Write STEP via STEPControl_Writer with the /tmp CWD strategy.
 *
 * If BOPAlgo_MakerVolume is available in the opencascade.js build, it is used
 * as a fast-path (it implements steps 2–4 internally).  The Section-based
 * path is the robust fallback that relies only on core Boolean APIs.
 *
 * opencascade.js is loaded lazily via dynamic import() when the user first
 * clicks "Export STEP" so the 35 MB WASM does not block page load.
 */

// ── Build version ─────────────────────────────────────────────────────────────

/** Increment this string with each release to verify live-site deployments. */
export const BUILD_VERSION = 'v0.2.5';

// ── OpenCASCADE lazy loader ───────────────────────────────────────────────────

// opencascade.js 2.0 beta — full OCCT 7.6.2 bindings with proper TypeScript types.
// BOPAlgo_MakerVolume_1() is supported here (it was RED/unsupported in 1.1.4).
const OC_CDN = 'https://unpkg.com/opencascade.js@2.0.0-beta.b5ff984/dist/';

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
    onStatus?.('Downloading OpenCASCADE geometry engine (~35 MB, first load only)…');

    // opencascade.js 2.0 beta ships opencascade.full.js (Emscripten glue) +
    // opencascade.full.wasm.  Dynamic import() fetches the glue; locateFile
    // maps the .wasm request back to the CDN.
    const ocMod = await import(/* @vite-ignore */ OC_CDN + 'opencascade.full.js');
    const factory = ocMod.default ?? ocMod.opencascade;
    if (typeof factory !== 'function') {
      throw new Error(
        'Could not find the opencascade factory in the loaded module. ' +
        'Check that unpkg.com is reachable and the correct version is being loaded.'
      );
    }

    onStatus?.('Initialising OpenCASCADE…');
    _oc = await factory({
      locateFile: (file) => {
        // The wasm binary is always named opencascade.full.wasm in 2.0 beta.
        if (file.endsWith('.wasm')) return OC_CDN + 'opencascade.full.wasm';
        return OC_CDN + file;
      },
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

/**
 * Return a new Message_ProgressRange for passing to OCCT Perform()/Build()
 * calls that require one in opencascade.js 2.0 beta (OCCT 7.5+).
 * The range is "dead" (no reporter attached) so OCCT treats it as a no-op
 * progress indicator.
 */
function _mkRange(oc) {
  try { return new oc.Message_ProgressRange_1(); } catch { return null; }
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

// ── Group adjacency ─────────────────────────────────────────────────────────

/**
 * Build a symmetric adjacency map between face groups by scanning every mesh
 * edge. Two groups are adjacent when a mesh edge (identified by its quantised
 * vertex keys) belongs to a triangle in each group.
 *
 * @param {object[]} groups   array of { triangleIndices }
 * @param {THREE.BufferGeometry} geometry
 * @returns {Map<number, Set<number>>}  groupIdx → set of adjacent groupIdxs
 */
export function buildGroupAdjacencyMap(groups, geometry) {
  const posAttr = geometry.attributes.position;
  const QUANT = 1e4;
  const qk = (x, y, z) =>
    `${Math.round(x * QUANT)}_${Math.round(y * QUANT)}_${Math.round(z * QUANT)}`;

  // Map: edge-key → first group index that claimed this edge
  const edgeOwner = new Map();
  const result = new Map();
  for (let i = 0; i < groups.length; i++) result.set(i, new Set());

  for (let gi = 0; gi < groups.length; gi++) {
    for (const t of groups[gi].triangleIndices) {
      for (let e = 0; e < 3; e++) {
        const ai = t * 3 + e;
        const bi = t * 3 + (e + 1) % 3;
        const ka = qk(posAttr.getX(ai), posAttr.getY(ai), posAttr.getZ(ai));
        const kb = qk(posAttr.getX(bi), posAttr.getY(bi), posAttr.getZ(bi));
        const key = ka < kb ? `${ka}|${kb}` : `${kb}|${ka}`;
        const prev = edgeOwner.get(key);
        if (prev === undefined) {
          edgeOwner.set(key, gi);
        } else if (prev !== gi && prev !== -1) {
          result.get(gi).add(prev);
          result.get(prev).add(gi);
          edgeOwner.set(key, -1); // mark as claimed by multiple groups
        }
      }
    }
  }

  return result;
}

// ── Vector math utilities ────────────────────────────────────────────────────

function _d3(a, b)  { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
function _x3(a, b)  {
  return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
}
function _n3(v)     { return Math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
function _u3(v)     { const n = _n3(v); return n > 1e-14 ? [v[0]/n, v[1]/n, v[2]/n] : v; }

// ── Mesh boundary extraction ─────────────────────────────────────────────────

/**
 * Single-pass extraction of per-group boundary edge pairs from the mesh.
 *
 * An edge is a "boundary edge" for group i when it is shared by exactly one
 * triangle from group i and exactly one triangle from a different group.
 * (Outer mesh edges — shared by only one triangle across the whole mesh —
 * are not boundary edges between groups and are ignored.)
 *
 * Runs in O(total triangles) time — one pass over all groups, no per-group
 * inner loops.
 *
 * @param {object[]} groups    with `.triangleIndices`
 * @param {THREE.BufferGeometry} geometry
 * @returns {Map<number, Array<[number,number]>>}
 *   groupIdx → array of [a_idx, b_idx] flat position-buffer index pairs
 */
function _buildBoundaryEdgeMap(groups, geometry) {
  const pos   = geometry.attributes.position;
  const QUANT = 1e4;
  const qk    = (i) => {
    const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
    return `${Math.round(x * QUANT)}_${Math.round(y * QUANT)}_${Math.round(z * QUANT)}`;
  };

  // Record every directed half-edge: undirected key → [{gi, a, b}]
  const edgeData = new Map();
  for (let gi = 0; gi < groups.length; gi++) {
    for (const t of groups[gi].triangleIndices) {
      for (let e = 0; e < 3; e++) {
        const a  = t * 3 + e;
        const b  = t * 3 + (e + 1) % 3;
        const ka = qk(a), kb = qk(b);
        const key = ka < kb ? `${ka}|${kb}` : `${kb}|${ka}`;
        if (!edgeData.has(key)) edgeData.set(key, []);
        edgeData.get(key).push({ gi, a, b });
      }
    }
  }

  // An edge shared by exactly two different groups is a boundary edge.
  const result = new Map();
  for (let gi = 0; gi < groups.length; gi++) result.set(gi, []);
  for (const entries of edgeData.values()) {
    if (entries.length !== 2) continue;
    const [e0, e1] = entries;
    if (e0.gi === e1.gi) continue;
    result.get(e0.gi).push([e0.a, e0.b]);
    result.get(e1.gi).push([e1.a, e1.b]);
  }
  return result;
}

/**
 * Walk an unordered set of boundary edge vertex-index pairs into a single
 * ordered polygon loop by following the adjacency graph.
 *
 * @param {Array<[number,number]>} edgePairs  [a_idx, b_idx] position indices
 * @param {THREE.BufferAttribute}  posAttr
 * @returns {Array<[number,number,number]>|null}  ordered [x,y,z] points, or null
 */
function _orderBoundaryLoop(edgePairs, posAttr) {
  if (edgePairs.length < 3) return null;

  const QUANT = 1e4;
  const qk  = (i) => {
    const x = posAttr.getX(i), y = posAttr.getY(i), z = posAttr.getZ(i);
    return `${Math.round(x * QUANT)}_${Math.round(y * QUANT)}_${Math.round(z * QUANT)}`;
  };
  const vtx = (i) => [posAttr.getX(i), posAttr.getY(i), posAttr.getZ(i)];

  // Build adjacency: vertexKey → [{nextKey, nextIdx}]
  const adj  = new Map();
  const vidx = new Map(); // vertexKey → a representative buffer index
  for (const [a, b] of edgePairs) {
    const ka = qk(a), kb = qk(b);
    if (!adj.has(ka))  { adj.set(ka, []);  vidx.set(ka, a); }
    if (!adj.has(kb))  { adj.set(kb, []);  vidx.set(kb, b); }
    adj.get(ka).push({ key: kb, idx: b });
    adj.get(kb).push({ key: ka, idx: a });
  }

  // Walk from an arbitrary starting vertex.
  const startKey = [...adj.keys()][0];
  const loop    = [];
  const visited = new Set();
  let curKey = startKey;
  let curIdx = vidx.get(startKey);

  while (!visited.has(curKey)) {
    visited.add(curKey);
    loop.push(vtx(curIdx));
    let nextKey = null, nextIdx = null;
    for (const { key, idx } of (adj.get(curKey) ?? [])) {
      if (!visited.has(key)) { nextKey = key; nextIdx = idx; break; }
    }
    if (nextKey === null) break;
    curKey = nextKey;
    curIdx = nextIdx;
  }

  return loop.length >= 3 ? loop : null;
}

/**
 * Scan all triangle vertices in a group to find the axial V extent.
 * Projects vertices onto the surface axis to find the parameter range.
 */
function _cylinderVExtentsFromVertices(group, geometry, axisPoint, axis) {
  const ax  = _u3(axis);
  const pos = geometry.attributes.position;
  let vmin = Infinity, vmax = -Infinity;
  for (const t of group.triangleIndices) {
    for (let j = 0; j < 3; j++) {
      const i = t * 3 + j;
      const v = (pos.getX(i) - axisPoint[0]) * ax[0] +
                (pos.getY(i) - axisPoint[1]) * ax[1] +
                (pos.getZ(i) - axisPoint[2]) * ax[2];
      if (v < vmin) vmin = v;
      if (v > vmax) vmax = v;
    }
  }
  if (!isFinite(vmin) || vmax - vmin < 1e-10) return null;
  return { vmin, vmax };
}

/**
 * Scan all triangle vertices in a sphere group to find the latitude (V) extent.
 */
function _sphereVExtentsFromVertices(group, geometry, center, radius) {
  const pos  = geometry.attributes.position;
  let vmin =  Math.PI / 2;
  let vmax = -Math.PI / 2;
  for (const t of group.triangleIndices) {
    for (let j = 0; j < 3; j++) {
      const i   = t * 3 + j;
      const dz  = pos.getZ(i) - center[2];
      const lat = Math.asin(Math.max(-1, Math.min(1, dz / radius)));
      if (lat < vmin) vmin = lat;
      if (lat > vmax) vmax = lat;
    }
  }
  if (vmax - vmin < 1e-10) return null;
  return { vmin, vmax };
}

// ── Public pure-JS boundary computation (no OCCT) ────────────────────────────

/**
 * Compute analytical boundary data for every fitted group without loading
 * OpenCASCADE.  The result can be used for viewport face-shape preview and
 * to pre-compute V extents for the STEP builder.
 *
 * @param {object[]} groups   groups with `.surface` already set by fitAllGroups()
 * @param {THREE.BufferGeometry} geometry
 * @returns {{ adjacency: Map, boundaries: Map<number, object> }}
 *   `boundaries` maps groupIdx → { type, loop? (plane corners),
 *                                   vmin?, vmax? (cylinder/cone) }
 */
export function computeAnalyticalBoundaries(groups, geometry) {
  const adjacency = buildGroupAdjacencyMap(groups, geometry);
  const boundaries = new Map();

  for (let gi = 0; gi < groups.length; gi++) {
    const g = groups[gi];
    if (!g.surface) continue;
    const { type, params } = g.surface;

    if (type === 'plane') {
      // For viewport preview: compute a bounding rectangle in the plane's local frame
      // using mesh vertices. The boundary shown in the preview is approximate.
      const nu = _u3(params.normal);
      const ref = Math.abs(nu[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
      const xA = _u3(_x3(nu, ref));
      const yA = _x3(nu, xA);
      const pos = geometry.attributes.position;
      let umin = Infinity, umax = -Infinity, vmin = Infinity, vmax = -Infinity;
      for (const t of g.triangleIndices) {
        for (let j = 0; j < 3; j++) {
          const i = t * 3 + j;
          const dx = pos.getX(i) - params.origin[0];
          const dy = pos.getY(i) - params.origin[1];
          const dz = pos.getZ(i) - params.origin[2];
          const u = dx*xA[0] + dy*xA[1] + dz*xA[2];
          const v = dx*yA[0] + dy*yA[1] + dz*yA[2];
          if (u < umin) umin = u; if (u > umax) umax = u;
          if (v < vmin) vmin = v; if (v > vmax) vmax = v;
        }
      }
      if (isFinite(umin)) {
        const o = params.origin;
        const loop = [
          [o[0]+umin*xA[0]+vmin*yA[0], o[1]+umin*xA[1]+vmin*yA[1], o[2]+umin*xA[2]+vmin*yA[2]],
          [o[0]+umax*xA[0]+vmin*yA[0], o[1]+umax*xA[1]+vmin*yA[1], o[2]+umax*xA[2]+vmin*yA[2]],
          [o[0]+umax*xA[0]+vmax*yA[0], o[1]+umax*xA[1]+vmax*yA[1], o[2]+umax*xA[2]+vmax*yA[2]],
          [o[0]+umin*xA[0]+vmax*yA[0], o[1]+umin*xA[1]+vmax*yA[1], o[2]+umin*xA[2]+vmax*yA[2]],
        ];
        boundaries.set(gi, { type: 'plane', loop });
      }
    } else if (type === 'cylinder') {
      const vr = _cylinderVExtentsFromVertices(g, geometry, params.axisPoint, params.axis);
      boundaries.set(gi, { type: 'cylinder', vmin: vr?.vmin ?? null, vmax: vr?.vmax ?? null });
    } else if (type === 'cone') {
      const vr = _cylinderVExtentsFromVertices(g, geometry, params.apex, params.axis);
      boundaries.set(gi, { type: 'cone', vmin: vr?.vmin ?? null, vmax: vr?.vmax ?? null });
    } else if (type === 'sphere') {
      boundaries.set(gi, { type: 'sphere' });
    }
  }

  return { adjacency, boundaries };
}

// ── Wire builder ─────────────────────────────────────────────────────────────

/**
 * Build an OCCT wire from an ordered list of 3D points (linear edges).
 * Skips zero-length edges.  Returns null if the wire cannot be built.
 */
function buildWire(oc, loop, toDelete) {
  if (!loop || loop.length < 3) return null;

  const wireMaker = new oc.BRepBuilderAPI_MakeWire_1();
  toDelete.push(wireMaker);

  let edgeCount = 0;
  for (let i = 0; i < loop.length; i++) {
    const a = loop[i], b = loop[(i + 1) % loop.length];
    const dx=b[0]-a[0], dy=b[1]-a[1], dz=b[2]-a[2];
    if (dx*dx+dy*dy+dz*dz < 1e-20) continue;

    const pa = makePnt(oc, a); toDelete.push(pa);
    const pb = makePnt(oc, b); toDelete.push(pb);
    const edgeMaker = new oc.BRepBuilderAPI_MakeEdge_3(pa, pb);
    toDelete.push(edgeMaker);
    if (!edgeMaker.IsDone()) continue;
    wireMaker.Add_1(edgeMaker.Edge());
    edgeCount++;
  }

  if (edgeCount < 3 || !wireMaker.IsDone()) return null;
  return wireMaker.Wire();
}

// ── Face builder ─────────────────────────────────────────────────────────────

/**
 * Build a large analytical patch face for any surface type.
 * UV extents come from mesh vertices plus a generous margin — used as the
 * primary face shape for curved surfaces and as a fallback for planes.
 *
 * @param {object}   oc
 * @param {object}   group    { triangleIndices, surface: {type, params} }
 * @param {object}   geometry THREE.BufferGeometry
 * @param {object[]} toDelete
 * @param {number}   [modelDiag]  bounding-box diagonal for model-scale margins
 * @returns {object|null}  TopoDS_Face or null
 */
function _buildLargePatch(oc, group, geometry, toDelete, modelDiag) {
  const { type, params } = group.surface;

  try {
    if (type === 'plane') {
      const { origin, normal } = params;
      const nu = _u3(normal);
      const pln = new oc.gp_Pln_3(makePnt(oc, origin), makeDir(oc, nu));
      toDelete.push(pln);

      // Local frame for projecting vertices — consistent JS frame, doesn't need
      // to match OCCT's internal UV axes because we build a 3D wire, not UV bounds.
      const ref = Math.abs(nu[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
      const xA = _u3(_x3(nu, ref));
      const yA = _x3(nu, xA);

      const pos = geometry.attributes.position;
      let umin = Infinity, umax = -Infinity, vmin = Infinity, vmax = -Infinity;
      for (const t of group.triangleIndices) {
        for (let j = 0; j < 3; j++) {
          const i = t * 3 + j;
          const dx = pos.getX(i) - origin[0];
          const dy = pos.getY(i) - origin[1];
          const dz = pos.getZ(i) - origin[2];
          const u = dx*xA[0] + dy*xA[1] + dz*xA[2];
          const v = dx*yA[0] + dy*yA[1] + dz*yA[2];
          if (u < umin) umin = u; if (u > umax) umax = u;
          if (v < vmin) vmin = v; if (v > vmax) vmax = v;
        }
      }
      if (!isFinite(umin)) return null;
      const m = Math.max(Math.max(umax - umin, vmax - vmin) * 0.5, (modelDiag || 0) * 0.5) + 1e-2;
      umin -= m; umax += m; vmin -= m; vmax += m;

      const corners = [
        [origin[0]+umin*xA[0]+vmin*yA[0], origin[1]+umin*xA[1]+vmin*yA[1], origin[2]+umin*xA[2]+vmin*yA[2]],
        [origin[0]+umax*xA[0]+vmin*yA[0], origin[1]+umax*xA[1]+vmin*yA[1], origin[2]+umax*xA[2]+vmin*yA[2]],
        [origin[0]+umax*xA[0]+vmax*yA[0], origin[1]+umax*xA[1]+vmax*yA[1], origin[2]+umax*xA[2]+vmax*yA[2]],
        [origin[0]+umin*xA[0]+vmax*yA[0], origin[1]+umin*xA[1]+vmax*yA[1], origin[2]+umin*xA[2]+vmax*yA[2]],
      ];
      const wire = buildWire(oc, corners, toDelete);
      if (!wire) return null;
      const mf = new oc.BRepBuilderAPI_MakeFace_16(pln, wire, true);
      toDelete.push(mf);
      return mf.IsDone() ? mf.Face() : null;
    }

    if (type === 'cylinder') {
      const { axisPoint, axis, radius } = params;
      const vr = _cylinderVExtentsFromVertices(group, geometry, axisPoint, axis);
      if (!vr) return null;
      const ax3 = makeAx3(oc, axisPoint, axis);
      toDelete.push(ax3);
      const cyl = new oc.gp_Cylinder_2(ax3, radius);
      toDelete.push(cyl);
      const pad = Math.max((vr.vmax - vr.vmin) * 0.25, (modelDiag || 0) * 0.25);
      const mf = new oc.BRepBuilderAPI_MakeFace_10(cyl, 0.0, 2*Math.PI, vr.vmin - pad, vr.vmax + pad);
      toDelete.push(mf);
      return mf.IsDone() ? mf.Face() : null;
    }

    if (type === 'cone') {
      const { apex, axis, halfAngle } = params;
      const vr = _cylinderVExtentsFromVertices(group, geometry, apex, axis);
      if (!vr || vr.vmin < -1e-6) return null;
      const ax3 = makeAx3(oc, apex, axis);
      toDelete.push(ax3);
      const cone = new oc.gp_Cone_2(ax3, halfAngle, 0.0);
      toDelete.push(cone);
      const pad = Math.max((vr.vmax - vr.vmin) * 0.25, (modelDiag || 0) * 0.25);
      const mf = new oc.BRepBuilderAPI_MakeFace_11(cone, 0.0, 2*Math.PI, Math.max(0, vr.vmin - pad), vr.vmax + pad);
      toDelete.push(mf);
      return mf.IsDone() ? mf.Face() : null;
    }

    if (type === 'sphere') {
      const { center, radius } = params;
      const vr = _sphereVExtentsFromVertices(group, geometry, center, radius);
      if (!vr) return null;
      const ax3 = new oc.gp_Ax3_4(makePnt(oc, center), makeDir(oc, [0, 0, 1]));
      toDelete.push(ax3);
      const sph = new oc.gp_Sphere_2(ax3, radius);
      toDelete.push(sph);
      // Sphere latitude is bounded to [-π/2, π/2]; the model-scale factor is
      // smaller (0.1 vs 0.25) to avoid pushing past the pole singularities.
      const pad = Math.max((vr.vmax - vr.vmin) * 0.25, (modelDiag || 0) * 0.1);
      const mf = new oc.BRepBuilderAPI_MakeFace_12(
        sph, 0.0, 2*Math.PI,
        Math.max(-Math.PI/2, vr.vmin - pad),
        Math.min( Math.PI/2, vr.vmax + pad),
      );
      toDelete.push(mf);
      return mf.IsDone() ? mf.Face() : null;
    }

  } catch (e) {
    console.warn('[brepBuilder] _buildLargePatch:', group.surface.type, e?.message ?? e);
  }
  return null;
}

// ── BRepAlgoAPI_Section wrapper ──────────────────────────────────────────────

/**
 * Compute the intersection curves between two shapes using BRepAlgoAPI_Section.
 * Probes multiple constructor overloads for opencascade.js compatibility.
 *
 * @param {object}   oc
 * @param {object}   s1     TopoDS_Shape
 * @param {object}   s2     TopoDS_Shape
 * @param {object[]} toDelete
 * @returns {object|null}  resulting TopoDS_Shape (compound of edges) or null
 */
function _section(oc, s1, s2, toDelete) {
  // opencascade.js 2.0 beta TypeScript declaration (verified from d.ts):
  //   BRepAlgoAPI_Section_1()                         — empty ctor
  //   BRepAlgoAPI_Section_2(PaveFiller)               — with filler
  //   BRepAlgoAPI_Section_3(S1, S2, PerformNow)       — two shapes ← use this
  //   BRepAlgoAPI_Section_4(S1, S2, PaveFiller, Now)  — with filler
  //   BRepAlgoAPI_Section_5(S1, gp_Pln, Now)
  //   BRepAlgoAPI_Section_6(S1, Geom_Surface, Now)
  //   Build(Message_ProgressRange)                     — required in 2.0 beta

  // ── Attempt 1: three-arg (S1, S2, PerformNow=true) ───────────────────────
  // _3 is the (S1, S2, PerformNow) overload in both 1.1.4 and 2.0 beta.
  // Also try _2 which was (S1, S2, PerformNow) in older builds.
  for (const name of ['BRepAlgoAPI_Section_3', 'BRepAlgoAPI_Section_2']) {
    if (typeof oc[name] !== 'function') continue;
    try {
      const sec = new oc[name](s1, s2, true);
      toDelete.push(sec);
      if (typeof sec.IsDone === 'function' && !sec.IsDone()) continue;
      const shape = sec.Shape();
      if (!shape || (typeof shape.IsNull === 'function' && shape.IsNull())) continue;
      return shape;
    } catch { continue; }
  }

  // ── Attempt 2: empty ctor → Init1/Init2 → Build ──────────────────────────
  if (typeof oc.BRepAlgoAPI_Section_1 === 'function') {
    try {
      const sec = new oc.BRepAlgoAPI_Section_1();
      toDelete.push(sec);
      const init1 = sec.Init1_1 ?? sec.Init1;
      const init2 = sec.Init2_1 ?? sec.Init2;
      if (typeof init1 === 'function') init1.call(sec, s1);
      if (typeof init2 === 'function') init2.call(sec, s2);
      if (typeof sec.Build === 'function') {
        const range = _mkRange(oc);
        try { sec.Build(range); } catch { try { sec.Build(); } catch {} }
      }
      if (typeof sec.IsDone === 'function' && !sec.IsDone()) return null;
      const shape = sec.Shape();
      if (!shape || (typeof shape.IsNull === 'function' && shape.IsNull())) return null;
      return shape;
    } catch { /* fall through */ }
  }

  console.warn('BRepAlgoAPI_Section: no working constructor found.');
  return null;
}

// ── Edge extraction ─────────────────────────────────────────────────────────

/**
 * Extract all TopoDS_Edge objects from a shape using TopExp_Explorer.
 */
function _extractEdges(oc, shape, toDelete) {
  const edges = [];
  const EDGE_T = oc.TopAbs_ShapeEnum?.TopAbs_EDGE ?? 5;
  const SHAPE_T = oc.TopAbs_ShapeEnum?.TopAbs_SHAPE ?? 0;
  try {
    const explorer = new oc.TopExp_Explorer_2(shape, EDGE_T, SHAPE_T);
    toDelete.push(explorer);
    while (explorer.More()) {
      // explorer.Current() returns TopoDS_Shape; cast to TopoDS_Edge so that
      // BRepAdaptor_Curve_2 and BRepBuilderAPI_MakeWire.Add_1 receive the
      // correct type (required in opencascade.js 2.0 beta).
      try {
        const e = oc.TopoDS.Edge_1(explorer.Current());
        edges.push(e);
      } catch {
        edges.push(explorer.Current()); // fallback for older builds
      }
      explorer.Next();
    }
  } catch (e) {
    console.warn('_extractEdges failed:', e?.message ?? e);
  }
  return edges;
}

// ── Edge endpoint extraction ────────────────────────────────────────────────

/**
 * Get the 3D start and end points of an OCCT edge, plus whether it is closed.
 * Uses BRepAdaptor_Curve to evaluate the edge's underlying curve.
 *
 * @param {object} oc
 * @param {object} edge   TopoDS_Edge
 * @param {object[]} toDelete
 * @param {number} tolerance  distance below which start≈end means "closed"
 * @returns {{ start: number[], end: number[], closed: boolean }|null}
 */
function _edgePoints(oc, edge, toDelete, tolerance = 1e-6) {
  try {
    const adaptor = new oc.BRepAdaptor_Curve_2(edge);
    toDelete.push(adaptor);
    const p1 = adaptor.Value(adaptor.FirstParameter());
    const p2 = adaptor.Value(adaptor.LastParameter());
    toDelete.push(p1, p2);
    const start = [p1.X(), p1.Y(), p1.Z()];
    const end   = [p2.X(), p2.Y(), p2.Z()];
    const dx = end[0]-start[0], dy = end[1]-start[1], dz = end[2]-start[2];
    const closed = Math.sqrt(dx*dx+dy*dy+dz*dz) < tolerance;
    return { start, end, closed };
  } catch {
    return null;
  }
}

// ── Edge → wire grouping ────────────────────────────────────────────────────

/**
 * Group a set of OCCT edges into closed wires by endpoint connectivity.
 *
 * Edges whose start and end coincide (within tolerance) are treated as
 * standalone closed wires (e.g. circles from plane∩cylinder sections).
 *
 * Remaining edges are grouped into connected chains by matching endpoints
 * and assembled into BRepBuilderAPI_MakeWire.
 *
 * @param {object}   oc
 * @param {object[]} edges      TopoDS_Edge objects
 * @param {number}   tolerance
 * @param {object[]} toDelete
 * @returns {object[]}  array of TopoDS_Wire
 */
function _groupEdgesIntoWires(oc, edges, tolerance, toDelete) {
  if (edges.length === 0) return [];
  const tol2 = tolerance * tolerance;
  const ptsClose = (a, b) => {
    const dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
    return dx*dx+dy*dy+dz*dz < tol2;
  };

  // Get endpoint data for every edge.
  const infos = [];
  for (const edge of edges) {
    const pts = _edgePoints(oc, edge, toDelete, tolerance);
    if (!pts) continue;
    infos.push({ edge, ...pts });
  }

  const wires = [];
  const used = new Set();

  // Pass 1 — closed edges (circles, full ellipses, etc.) are standalone wires.
  for (let i = 0; i < infos.length; i++) {
    if (!infos[i].closed) continue;
    used.add(i);
    try {
      const wm = new oc.BRepBuilderAPI_MakeWire_1();
      toDelete.push(wm);
      wm.Add_1(infos[i].edge);
      if (wm.IsDone()) wires.push(wm.Wire());
    } catch { /* skip malformed edge */ }
  }

  // Pass 2 — chain open edges by endpoint proximity.
  while (true) {
    // Find an unused open edge to start a new chain.
    let seedIdx = -1;
    for (let i = 0; i < infos.length; i++) {
      if (!used.has(i)) { seedIdx = i; break; }
    }
    if (seedIdx === -1) break;

    const chain = [seedIdx];
    used.add(seedIdx);
    let head = infos[seedIdx].start;
    let tail = infos[seedIdx].end;

    // Grow the chain in both directions.
    let progress = true;
    while (progress) {
      progress = false;
      for (let i = 0; i < infos.length; i++) {
        if (used.has(i)) continue;
        const { start, end } = infos[i];
        if (ptsClose(tail, start)) {
          chain.push(i); used.add(i); tail = end; progress = true;
        } else if (ptsClose(tail, end)) {
          chain.push(i); used.add(i); tail = start; progress = true;
        } else if (ptsClose(head, end)) {
          chain.unshift(i); used.add(i); head = start; progress = true;
        } else if (ptsClose(head, start)) {
          chain.unshift(i); used.add(i); head = end; progress = true;
        }
      }
    }

    if (chain.length === 0) continue;
    try {
      const wm = new oc.BRepBuilderAPI_MakeWire_1();
      toDelete.push(wm);
      for (const idx of chain) wm.Add_1(infos[idx].edge);
      if (wm.IsDone()) wires.push(wm.Wire());
      else console.warn(`Wire from ${chain.length} edges did not complete.`);
    } catch (e) {
      console.warn('Wire construction failed:', e?.message ?? e);
    }
  }

  return wires;
}

// ── Analytical V-bounds from adjacent planes ────────────────────────────────

/**
 * Compute the V-parameter range of a curved surface (cylinder, cone, sphere)
 * by finding where each adjacent plane intersects the surface axis.
 *
 * This is a pure analytical computation using only fitted surface parameters —
 * no mesh data, no OCCT edges.  For each adjacent plane with known origin
 * and normal, the axis intersection parameter is:
 *
 *   V = dot(planeOrigin − axisPoint, planeNormal) / dot(axis, planeNormal)
 *
 * @param {object}   surfaceParams
 * @param {string}   surfaceType    'cylinder' | 'cone' | 'sphere'
 * @param {number[]} adjacentGroupIndices
 * @param {object[]} groups
 * @returns {{ vmin: number, vmax: number }|null}
 */
function _vBoundsFromAdjacentPlanes(surfaceParams, surfaceType,
                                    adjacentGroupIndices, groups) {
  let vmin = Infinity, vmax = -Infinity;

  let axisPoint, axis;
  if (surfaceType === 'cylinder') {
    axisPoint = surfaceParams.axisPoint;
    axis = _u3(surfaceParams.axis);
  } else if (surfaceType === 'cone') {
    axisPoint = surfaceParams.apex;
    axis = _u3(surfaceParams.axis);
  } else if (surfaceType === 'sphere') {
    axisPoint = surfaceParams.center;
    // OCCT gp_Sphere_2(Ax3, R) always uses the Ax3 main direction as the
    // sphere's pole axis.  We construct it with makeDir([0,0,1]) in
    // _buildTrimmedFace, so V (latitude) is measured relative to Z.
    axis = [0, 0, 1];
  } else {
    return null;
  }

  for (const adjIdx of adjacentGroupIndices) {
    const adj = groups[adjIdx];
    if (!adj?.surface || adj.surface.type !== 'plane') continue;
    const { origin, normal } = adj.surface.params;
    const nu = _u3(normal);
    const denom = _d3(axis, nu);
    if (Math.abs(denom) < 1e-14) continue; // plane parallel to axis
    const diff = [origin[0]-axisPoint[0], origin[1]-axisPoint[1],
                  origin[2]-axisPoint[2]];
    const t = _d3(diff, nu) / denom;

    if (surfaceType === 'sphere') {
      // t is the signed distance along the Z-axis from the sphere centre
      // to the plane intersection.  Dividing by radius gives sin(latitude),
      // so asin(t/R) converts it to the OCCT sphere V parameter.
      const lat = Math.asin(Math.max(-1, Math.min(1, t / surfaceParams.radius)));
      if (lat < vmin) vmin = lat;
      if (lat > vmax) vmax = lat;
    } else {
      if (t < vmin) vmin = t;
      if (t > vmax) vmax = t;
    }
  }

  return isFinite(vmin) && vmax - vmin > 1e-10 ? { vmin, vmax } : null;
}

// ── Trimmed face builder ────────────────────────────────────────────────────

/**
 * Build a properly trimmed face from section edges (computed via
 * BRepAlgoAPI_Section with all neighbours).
 *
 * • Plane faces: section edges are assembled into wires.  The largest wire
 *   becomes the outer boundary; smaller wires become holes.
 *
 * • Curved faces (cylinder, cone, sphere): V-parameter bounds are computed
 *   analytically from adjacent plane positions — no mesh or OCCT edges.
 *
 * This generalises to fillets and NURBS: section edges provide exact trim
 * curves for any surface pair, and MakeFace(Geom_Surface, wire) will produce
 * the trimmed face.
 */
function _buildTrimmedFace(oc, group, groupIdx, sectionEdges, adjacentIndices,
                           groups, modelDiag, tolerance, toDelete) {
  const { type, params } = group.surface;

  try {
    // ── Plane: build face from section-edge wires ──────────────────────────
    if (type === 'plane') {
      if (sectionEdges.length === 0) return null;

      const wires = _groupEdgesIntoWires(oc, sectionEdges, tolerance, toDelete);
      if (wires.length === 0) return null;

      // Identify the outer wire (largest bounding-box diagonal).
      let outerIdx = 0;
      if (wires.length > 1) {
        let maxDiag = -1;
        for (let i = 0; i < wires.length; i++) {
          try {
            const box = new oc.Bnd_Box_1();
            toDelete.push(box);
            oc.BRepBndLib.Add(wires[i], box, false);
            const cMin = box.CornerMin();
            const cMax = box.CornerMax();
            toDelete.push(cMin, cMax);
            const dx = cMax.X()-cMin.X(), dy = cMax.Y()-cMin.Y(), dz = cMax.Z()-cMin.Z();
            const diag = dx*dx + dy*dy + dz*dz;
            if (diag > maxDiag) { maxDiag = diag; outerIdx = i; }
          } catch { /* keep default */ }
        }
      }

      const { origin, normal } = params;
      const nu = _u3(normal);
      const pln = new oc.gp_Pln_3(makePnt(oc, origin), makeDir(oc, nu));
      toDelete.push(pln);

      const mf = new oc.BRepBuilderAPI_MakeFace_16(pln, wires[outerIdx], true);
      toDelete.push(mf);
      if (!mf.IsDone()) return null;

      // Add inner wires (holes).
      for (let i = 0; i < wires.length; i++) {
        if (i === outerIdx) continue;
        try { mf.Add(wires[i]); } catch { /* skip */ }
      }

      return mf.Face();
    }

    // ── Cylinder: V-bounds from adjacent planes ───────────────────────────
    if (type === 'cylinder') {
      const vr = _vBoundsFromAdjacentPlanes(params, 'cylinder',
        [...adjacentIndices], groups);
      if (!vr) return null;
      const ax3 = makeAx3(oc, params.axisPoint, params.axis);
      toDelete.push(ax3);
      const cyl = new oc.gp_Cylinder_2(ax3, params.radius);
      toDelete.push(cyl);
      const mf = new oc.BRepBuilderAPI_MakeFace_10(
        cyl, 0.0, 2*Math.PI, vr.vmin, vr.vmax);
      toDelete.push(mf);
      return mf.IsDone() ? mf.Face() : null;
    }

    // ── Cone: V-bounds from adjacent planes ───────────────────────────────
    if (type === 'cone') {
      const vr = _vBoundsFromAdjacentPlanes(params, 'cone',
        [...adjacentIndices], groups);
      if (!vr) return null;
      const ax3 = makeAx3(oc, params.apex, params.axis);
      toDelete.push(ax3);
      const cone = new oc.gp_Cone_2(ax3, params.halfAngle, 0.0);
      toDelete.push(cone);
      const mf = new oc.BRepBuilderAPI_MakeFace_11(
        cone, 0.0, 2*Math.PI, Math.max(0, vr.vmin), vr.vmax);
      toDelete.push(mf);
      return mf.IsDone() ? mf.Face() : null;
    }

    // ── Sphere: V-bounds from adjacent planes ─────────────────────────────
    if (type === 'sphere') {
      const vr = _vBoundsFromAdjacentPlanes(params, 'sphere',
        [...adjacentIndices], groups);
      if (!vr) return null;
      const ax3 = new oc.gp_Ax3_4(
        makePnt(oc, params.center), makeDir(oc, [0, 0, 1]));
      toDelete.push(ax3);
      const sph = new oc.gp_Sphere_2(ax3, params.radius);
      toDelete.push(sph);
      const mf = new oc.BRepBuilderAPI_MakeFace_12(
        sph, 0.0, 2*Math.PI,
        Math.max(-Math.PI/2, vr.vmin),
        Math.min( Math.PI/2, vr.vmax));
      toDelete.push(mf);
      return mf.IsDone() ? mf.Face() : null;
    }

  } catch (e) {
    console.warn(`_buildTrimmedFace (${type}):`, e?.message ?? e);
  }
  return null;
}

// ── Sew faces into a solid ──────────────────────────────────────────────────

/**
 * Sew an array of properly trimmed faces into a watertight solid.
 */
function _sewIntoSolid(oc, faces, sewTol, toDelete) {
  if (faces.length === 0) return null;
  try {
    // opencascade.js 2.0 beta (verified from d.ts):
    //   BRepBuilderAPI_Sewing(tolerance, option1, option2, option3, option4)
    //   — ALL 5 args required; no numbered suffix (single overload exported).
    //   Perform(Message_ProgressRange) — progress range is required.
    const sewing = new oc.BRepBuilderAPI_Sewing(sewTol, true, true, true, false);
    toDelete.push(sewing);

    for (const f of faces) sewing.Add(f);
    sewing.Perform(_mkRange(oc));

    const sewn = sewing.SewedShape();
    const shapeType = sewn.ShapeType?.() ?? -1;
    const SOLID_T = oc.TopAbs_ShapeEnum?.TopAbs_SOLID ?? 3;
    const SHELL_T = oc.TopAbs_ShapeEnum?.TopAbs_SHELL ?? 4;
    const COMP_T  = oc.TopAbs_ShapeEnum?.TopAbs_COMPOUND ?? 0;

    if (shapeType === SOLID_T) return sewn;

    if (shapeType === SHELL_T) {
      // BRepBuilderAPI_MakeSolid_3 takes TopoDS_Shell — cast from TopoDS_Shape.
      const shell = oc.TopoDS.Shell_1 ? oc.TopoDS.Shell_1(sewn) : sewn;
      const mkSolid = new oc.BRepBuilderAPI_MakeSolid_3(shell);
      toDelete.push(mkSolid);
      return mkSolid.IsDone() ? mkSolid.Solid() : null;
    }

    if (shapeType === COMP_T) {
      const expShell = new oc.TopExp_Explorer_2(
        sewn, SHELL_T, oc.TopAbs_ShapeEnum?.TopAbs_SHAPE ?? 0);
      toDelete.push(expShell);
      if (expShell.More()) {
        const shellShape = expShell.Current();
        const shell = oc.TopoDS.Shell_1 ? oc.TopoDS.Shell_1(shellShape) : shellShape;
        const mkSolid = new oc.BRepBuilderAPI_MakeSolid_3(shell);
        toDelete.push(mkSolid);
        return mkSolid.IsDone() ? mkSolid.Solid() : null;
      }
    }
  } catch (e) {
    console.warn('_sewIntoSolid failed:', e?.message ?? e);
  }
  return null;
}

// ── BOPAlgo_MakerVolume fast-path ───────────────────────────────────────────

// ── Mesh point-in-solid helpers (pure JS, no OCCT) ──────────────────────────

/**
 * Test whether a +Z ray from (px, py, pz) intersects triangle (A, B, C).
 * Returns true when the intersection lies strictly above the ray origin
 * (i.e. at t > 0 along the +Z axis).
 *
 * Uses Möller–Trumbore with D = (0,0,1) expanded at compile time for speed.
 */
function _rayTriangleHitAbove(px, py, pz,
                               ax, ay, az, bx, by, bz, cx, cy, cz) {
  const e1x = bx - ax, e1y = by - ay, e1z = bz - az;
  const e2x = cx - ax, e2y = cy - ay, e2z = cz - az;
  // H = (0,0,1) × E2  →  hx = -e2y, hy = e2x, hz = 0
  const hx = -e2y, hy = e2x;
  const det = e1x * hx + e1y * hy;           // E1 · H  (hz term = 0)
  if (Math.abs(det) < 1e-14) return false;   // ray parallel to triangle
  const inv = 1.0 / det;
  const sx = px - ax, sy = py - ay, sz = pz - az;
  const u = inv * (sx * hx + sy * hy);
  if (u < 0.0 || u > 1.0) return false;
  // Q = S × E1
  const qx = sy * e1z - sz * e1y;
  const qy = sz * e1x - sx * e1z;
  const qz = sx * e1y - sy * e1x;
  const v = inv * qz;                         // (0,0,1) · Q = Q.z
  if (v < 0.0 || u + v > 1.0) return false;
  const t = inv * (e2x * qx + e2y * qy + e2z * qz);
  return t > 0.0;                             // hit strictly above origin
}

/**
 * Return true when the point (px, py, pz) is inside the closed triangular
 * mesh described by a THREE.BufferGeometry, using the parity (ray-casting)
 * rule with a +Z ray.
 *
 * Handles indexed and non-indexed geometries.  Assumes a manifold, watertight
 * mesh — the same requirement as the STL format.
 */
function _pointInMesh(geometry, px, py, pz) {
  const pos = geometry.attributes.position;
  const idx = geometry.index;
  const triCount = idx ? idx.count / 3 : pos.count / 3;
  let hits = 0;
  for (let t = 0; t < triCount; t++) {
    const i0 = idx ? idx.getX(t * 3)     : t * 3;
    const i1 = idx ? idx.getX(t * 3 + 1) : t * 3 + 1;
    const i2 = idx ? idx.getX(t * 3 + 2) : t * 3 + 2;
    if (_rayTriangleHitAbove(
          px, py, pz,
          pos.getX(i0), pos.getY(i0), pos.getZ(i0),
          pos.getX(i1), pos.getY(i1), pos.getZ(i1),
          pos.getX(i2), pos.getY(i2), pos.getZ(i2))) hits++;
  }
  return (hits & 1) === 1;
}

// ── Per-solid sample-point finder ────────────────────────────────────────────

/**
 * Find a point that is verified to be in the interior of the given solid.
 *
 * The center of mass is correct for simply-connected cells, but can lie
 * outside a cell that is concave (horseshoe / crescent shaped).  We always
 * confirm with BRepClass3d_SolidClassifier before returning.  If the center
 * of mass fails we fall back to the vertex centroid and then to individual
 * vertices offset toward that centroid, so we find a valid interior point
 * even for strongly non-convex cells.
 *
 * @returns {number[]|null}  [x, y, z] inside the solid, or null if not found
 */
function _getSamplePointInsideSolid(oc, solid, tol, toDelete) {
  const classTol = Math.max(tol, 1e-7);
  const IN  = oc.TopAbs_State?.TopAbs_IN  ?? 0;
  const ON  = oc.TopAbs_State?.TopAbs_ON  ?? 2;

  const _isIn = p => {
    try {
      const clf = new oc.BRepClass3d_SolidClassifier_3(
        solid, makePnt(oc, p), classTol);
      toDelete.push(clf);
      const s = clf.State();
      return s === IN || s === ON
        || (typeof s === 'object' &&
            (s === oc.TopAbs_State?.TopAbs_IN ||
             s === oc.TopAbs_State?.TopAbs_ON));
    } catch { return false; }
  };

  // 1. GProp centre of mass (exact, fast, correct for convex/simply-connected).
  try {
    const props = new oc.GProp_GProps_1();
    toDelete.push(props);
    oc.BRepGProp.VolumeProperties_1(solid, props, false, false, false);
    const com = props.CentreOfMass();
    const p = [com.X(), com.Y(), com.Z()];
    if (_isIn(p)) return p;
  } catch { /* fall through */ }

  // 2. Vertex centroid — always inside convex cells; may be outside horseshoe.
  const VERTEX_T = oc.TopAbs_ShapeEnum?.TopAbs_VERTEX ?? 7;
  const SHAPE_T  = oc.TopAbs_ShapeEnum?.TopAbs_SHAPE  ?? 0;
  const verts = [];
  try {
    const exp = new oc.TopExp_Explorer_2(solid, VERTEX_T, SHAPE_T);
    toDelete.push(exp);
    while (exp.More() && verts.length < 64) {
      const vtx = oc.TopoDS.Vertex_1(exp.Current());
      const pt  = oc.BRep_Tool.Pnt(vtx);
      verts.push([pt.X(), pt.Y(), pt.Z()]);
      exp.Next();
    }
  } catch { /* fall through */ }

  if (verts.length === 0) return null;

  const cx = verts.reduce((s, v) => s + v[0], 0) / verts.length;
  const cy = verts.reduce((s, v) => s + v[1], 0) / verts.length;
  const cz = verts.reduce((s, v) => s + v[2], 0) / verts.length;
  const vc = [cx, cy, cz];
  if (_isIn(vc)) return vc;

  // 3. Vertices offset toward the vertex centroid — works for horseshoe cells
  //    where the centroid lands in the concave gap.
  for (const [vx, vy, vz] of verts) {
    for (const frac of [0.1, 0.3, 0.5, 0.7]) {
      const p = [
        vx + (cx - vx) * frac,
        vy + (cy - vy) * frac,
        vz + (cz - vz) * frac,
      ];
      if (_isIn(p)) return p;
    }
  }

  return null;
}

// ── MakerVolume solid filter ─────────────────────────────────────────────────

/**
 * Use OCCT's BOPAlgo_MakerVolume to build a solid from oversized faces.
 * MakerVolume partitions all of space into closed cells — we then keep every
 * cell whose interior lies within the original STL mesh, using a pure-JS
 * ray-casting point-in-mesh test.
 *
 * @param {object} geometry  THREE.BufferGeometry of the original mesh
 */
function _buildSolidViaMakerVolume(oc, faces, fuzzyTol, geometry, toDelete) {
  if (typeof oc.BOPAlgo_MakerVolume_1 !== 'function') {
    console.warn('BOPAlgo_MakerVolume_1 not found in this opencascade.js build.');
    return null;
  }
  try {
    const maker = new oc.BOPAlgo_MakerVolume_1();
    toDelete.push(maker);
    const argList = new oc.TopTools_ListOfShape_1();
    toDelete.push(argList);
    for (const f of faces) argList.Append_1(f);
    maker.SetArguments(argList);
    maker.SetIntersect(true);
    if (typeof maker.SetAvoidInternalShapes === 'function')
      maker.SetAvoidInternalShapes(true);
    if (typeof maker.SetFuzzyValue === 'function' && fuzzyTol > 0)
      maker.SetFuzzyValue(fuzzyTol);
    if (typeof maker.SetRunParallel === 'function')
      maker.SetRunParallel(false);
    maker.Perform(_mkRange(oc));
    if (typeof maker.HasErrors === 'function' && maker.HasErrors()) {
      console.warn('BOPAlgo_MakerVolume: Perform() completed with errors.');
      return null;
    }
    const result = maker.Shape();
    if (!result || (typeof result.IsNull === 'function' && result.IsNull())) {
      console.warn('BOPAlgo_MakerVolume: Perform() returned null/empty shape.');
      return null;
    }

    // ── Keep every cell that lies inside the original mesh ──────────────────
    // MakerVolume creates one closed solid for every bounded region of space
    // in the surface arrangement — interior cells, exterior octants, and all
    // cross-cut slivers.  We keep only the cells that are physically inside
    // the original part by testing a verified-interior sample point from each
    // cell against the STL mesh with a ray-casting parity test.
    const kept = _filterSolidsInsideMesh(oc, result, geometry, fuzzyTol, toDelete);
    if (!kept) {
      console.warn('BOPAlgo_MakerVolume: no cells classified as inside the mesh.');
      return null;
    }
    console.info('BOPAlgo_MakerVolume succeeded.');
    return kept;
  } catch (e) {
    console.warn('BOPAlgo_MakerVolume failed:', e?.message ?? e);
    return null;
  }
}

/**
 * From a MakerVolume result compound, collect every solid whose interior
 * lies within the original STL mesh and return them as a compound (or as a
 * single solid if only one survives).
 *
 * For each solid we find a point that is verifiably inside the solid
 * (handling non-convex/horseshoe-shaped cells) and then test it against the
 * mesh using Möller–Trumbore ray-casting parity.
 *
 * @param {object} geometry  THREE.BufferGeometry of the original mesh
 * @returns {object|null}  TopoDS_Compound of kept solids, or a single
 *   TopoDS_Solid, or null when nothing survived the filter
 */
function _filterSolidsInsideMesh(oc, shape, geometry, tol, toDelete) {
  const SOLID_T = oc.TopAbs_ShapeEnum?.TopAbs_SOLID ?? 3;
  const SHAPE_T = oc.TopAbs_ShapeEnum?.TopAbs_SHAPE ?? 0;

  // Collect all solids from the shape.
  const solids = [];
  try {
    const exp = new oc.TopExp_Explorer_2(shape, SOLID_T, SHAPE_T);
    toDelete.push(exp);
    while (exp.More()) {
      solids.push(oc.TopoDS.Solid_1
        ? oc.TopoDS.Solid_1(exp.Current())
        : exp.Current());
      exp.Next();
    }
  } catch (e) {
    console.warn('_filterSolidsInsideMesh: explorer failed:', e?.message ?? e);
  }

  if (solids.length === 0) return null;
  if (solids.length === 1) return solids[0];  // no filtering needed

  console.info(`BOPAlgo_MakerVolume: filtering ${solids.length} cells against mesh boundary…`);

  const kept = [];
  for (const solid of solids) {
    const pt = _getSamplePointInsideSolid(oc, solid, tol, toDelete);
    if (pt && _pointInMesh(geometry, pt[0], pt[1], pt[2])) kept.push(solid);
  }

  console.info(`BOPAlgo_MakerVolume: kept ${kept.length} / ${solids.length} cells.`);

  if (kept.length === 0) return null;
  if (kept.length === 1) return kept[0];

  // Multiple interior cells → return a compound so the STEP writer can
  // export them all as separate (correctly bounded) solids.
  try {
    const compound = new oc.TopoDS_Compound();
    const bb = new oc.BRep_Builder();
    toDelete.push(bb);
    bb.MakeCompound(compound);
    toDelete.push(compound);
    for (const s of kept) bb.Add(compound, s);
    return compound;
  } catch (e) {
    console.warn('_filterSolidsInsideMesh: compound build failed:', e?.message ?? e);
    return kept[0];  // best-effort single solid
  }
}

// ── Section-based solid builder ─────────────────────────────────────────────

/**
 * Build a solid by computing analytical intersection curves between adjacent
 * face pairs, trimming each face with those curves, and sewing the trimmed
 * faces into a watertight solid.
 *
 * The mesh is used ONLY for the adjacency map (which faces are neighbours).
 * All intersection curves and trim boundaries are computed analytically by
 * OCCT's geometry kernel — no mesh-derived geometry appears in the output.
 */
function _buildSolidViaSections(oc, faceEntries, adjacency, groups,
                                modelDiag, sewTol, toDelete) {
  const idxToEntry = new Map();
  for (const e of faceEntries) idxToEntry.set(e.groupIdx, e);

  const tolerance = sewTol;

  // Phase 1 — For each face, compute its section edges against all neighbours
  // in a single BRepAlgoAPI_Section call so that OCCT produces consistent
  // vertex topology at triple-point intersections.
  const faceSectionEdges = new Map();

  for (const { face, groupIdx } of faceEntries) {
    const adjSet = adjacency.get(groupIdx);
    if (!adjSet || adjSet.size === 0) {
      faceSectionEdges.set(groupIdx, []);
      continue;
    }

    const adjFaces = [];
    for (const adjIdx of adjSet) {
      const adjEntry = idxToEntry.get(adjIdx);
      if (adjEntry) adjFaces.push(adjEntry.face);
    }
    if (adjFaces.length === 0) {
      faceSectionEdges.set(groupIdx, []);
      continue;
    }

    const adjCompound = new oc.TopoDS_Compound();
    const bb = new oc.BRep_Builder();
    toDelete.push(bb);
    bb.MakeCompound(adjCompound);
    for (const af of adjFaces) bb.Add(adjCompound, af);
    toDelete.push(adjCompound);

    const sectionShape = _section(oc, face, adjCompound, toDelete);
    const edges = sectionShape ? _extractEdges(oc, sectionShape, toDelete) : [];
    faceSectionEdges.set(groupIdx, edges);

    if (edges.length === 0) {
      console.warn(`No section edges for group ${groupIdx} (${groups[groupIdx]?.surface?.type}).`);
    }
  }

  // Phase 2 — Build a trimmed face for each group.
  const trimmedFaces = [];
  for (const { group, groupIdx } of faceEntries) {
    const edges = faceSectionEdges.get(groupIdx) || [];
    const adjSet = adjacency.get(groupIdx) || new Set();
    const trimmed = _buildTrimmedFace(
      oc, group, groupIdx, edges, adjSet, groups,
      modelDiag, tolerance, toDelete);
    if (trimmed) trimmedFaces.push(trimmed);
    else console.warn(`Could not trim group ${groupIdx} (${group.surface?.type}).`);
  }

  if (trimmedFaces.length === 0) return null;

  // Phase 3 — Sew trimmed faces into a solid.
  const solid = _sewIntoSolid(oc, trimmedFaces, sewTol, toDelete);
  if (solid) {
    console.info(`Section-based solid: ${trimmedFaces.length} trimmed faces.`);
  }
  return solid;
}

// ── Main solid builder ──────────────────────────────────────────────────────

/**
 * Build a watertight solid from oversized analytical faces.
 *
 * Two implementations of the same face-based analytical-trimming approach:
 *   1. BOPAlgo_MakerVolume — ideal single-call (if available in this build).
 *   2. Section-based trimming — robust path using only core OCCT APIs.
 *
 * Both produce the same result: faces trimmed at exact analytical intersection
 * curves, assembled into a watertight solid.  No mesh-derived boundaries.
 *
 * @param {object} geometry  THREE.BufferGeometry — used to filter MakerVolume
 *   cells to only those that lie inside the original mesh boundary
 */
function _buildSolid(oc, faceEntries, adjacency, groups,
                     modelDiag, sewTol, geometry, toDelete) {
  // Fast path: BOPAlgo_MakerVolume.
  const faces = faceEntries.map(e => e.face);
  const mv = _buildSolidViaMakerVolume(oc, faces, sewTol, geometry, toDelete);
  if (mv) return mv;

  // Robust path: Section-based analytical trimming.
  console.info('BOPAlgo_MakerVolume did not produce a result — using Section-based analytical trimming.');
  return _buildSolidViaSections(oc, faceEntries, adjacency, groups,
                                modelDiag, sewTol, toDelete);
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

  const toDelete   = [];

  // ── Compute model bounding-box diagonal ─────────────────────────────────────
  // Used for adaptive sewing tolerance and for sizing the oversized analytical
  // patches in _buildLargePatch().
  const posAttr = geometry.attributes.position;
  let xmin =  Infinity, ymin =  Infinity, zmin =  Infinity;
  let xmax = -Infinity, ymax = -Infinity, zmax = -Infinity;
  for (let i = 0; i < posAttr.count; i++) {
    const x = posAttr.getX(i), y = posAttr.getY(i), z = posAttr.getZ(i);
    if (x < xmin) xmin = x; if (x > xmax) xmax = x;
    if (y < ymin) ymin = y; if (y > ymax) ymax = y;
    if (z < zmin) zmin = z; if (z > zmax) zmax = z;
  }
  const modelDiag = Math.sqrt((xmax-xmin)**2 + (ymax-ymin)**2 + (zmax-zmin)**2);

  let sewTol = options.sewTol ?? 0;
  if (sewTol <= 0) sewTol = Math.max(1e-6, modelDiag * 5e-3);

  // ── Build oversized analytical patches for every surface group ──────────────
  //
  // ALL surface types (plane, cylinder, cone, sphere) go through the same code
  // path — _buildLargePatch() — with model-scale margins so that each face
  // extends well beyond the model bounding box.  Trimming is handled by OCCT's
  // geometry kernel (MakerVolume or Section-based trimming) — never by mesh
  // boundaries.
  //
  // We track the groupIdx because the Section-based trimming path uses the
  // mesh-derived adjacency map to know which faces are neighbours, and the
  // fitted surface parameters to compute analytical V-bounds.

  const faceEntries = [];
  for (let i = 0; i < groups.length; i++) {
    const g = groups[i];
    if (!g.surface) continue;
    if (i % 50 === 0) onStatus?.(`Building faces… ${i}/${groups.length}`, 20 + 20 * i / groups.length);

    try {
      const face = _buildLargePatch(oc, g, geometry, toDelete, modelDiag);
      if (face) faceEntries.push({ face, group: g, groupIdx: i });
    } catch (e) {
      console.warn(`Patch ${i} (${g.surface?.type}):`, e?.message ?? e);
    }
  }

  if (faceEntries.length === 0) throw new Error('No valid B-rep faces could be constructed.');

  onStatus?.(`Trimming ${faceEntries.length} faces and building solid…`, 40);

  // ── Compute mesh adjacency ─────────────────────────────────────────────────
  // The mesh is used ONLY here — to determine which surface groups are
  // neighbours.  All intersection curves and trim boundaries are computed
  // analytically by OCCT's geometry kernel.
  const adjacency = buildGroupAdjacencyMap(groups, geometry);

  // ── Build a watertight solid ────────────────────────────────────────────────
  // One approach (face-based analytical trimming) with two implementations:
  //   • BOPAlgo_MakerVolume — ideal, single-call (if available in this build)
  //   • Section-based trimming — robust path using only core OCCT APIs
  // No fallback to untrimmed faces, no compound dumping.
  const topShape = _buildSolid(
    oc, faceEntries, adjacency, groups, modelDiag, sewTol, geometry, toDelete);

  if (!topShape) {
    throw new Error(
      'Solid construction failed.  Neither BOPAlgo_MakerVolume nor ' +
      'Section-based trimming could produce a watertight solid from ' +
      `${faceEntries.length} analytical faces.`);
  }

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
    topShape,
    oc.STEPControl_StepModelType?.STEPControl_AsIs ?? 0,
    true,
    _mkRange(oc),
  );

  if (transferResult !== DONE) {
    throw new Error(`STEPControl_Writer.Transfer failed (status ${transferResult}).`);
  }

  // ── STEP file write ─────────────────────────────────────────────────────────
  //
  // OCCT's OSD_Path resolves bare filenames differently across Emscripten builds.
  // Evidence (observed diagnostics): writing to CWD='/' with a bare filename
  // results in an empty file at the expected inode — the actual content lands in
  // a garbled/unexpected MEMFS entry elsewhere in '/'.
  //
  // Strategy:
  //  1. Use '/tmp' as the write directory; it is always present in OCCT's MEMFS.
  //  2. Pre-create the inode (in case OSD_File uses O_WRONLY without O_CREAT).
  //  3. Delete the C++ writer object BEFORE reading so fclose() flushes the
  //     FILE* buffer to MEMFS (STEPControl_Writer uses C stdio internally).
  //  4. After flushing, try to read the file.  If the expected path is empty,
  //     scan a fixed set of candidate directories; the writer may have resolved
  //     the path differently.
  //  5. Log the full MEMFS state BEFORE cleanup so the diagnostic is useful.

  const stepFile = 'brep_export.stp';
  const savedCwd = typeof oc.FS.cwd === 'function' ? oc.FS.cwd() : '/';

  // Prefer '/tmp' — it exists in all OCCT Emscripten builds and is a real
  // directory node (not root), which avoids the root-path resolution quirk.
  let writeCwd = '/tmp';
  try {
    oc.FS.chdir('/tmp');
    writeCwd = typeof oc.FS.cwd === 'function' ? oc.FS.cwd() : '/tmp';
  } catch {
    // '/tmp' unavailable — fall back to root
    writeCwd = '/';
    try { oc.FS.chdir('/'); } catch { /* best effort */ }
    writeCwd = typeof oc.FS.cwd === 'function' ? oc.FS.cwd() : writeCwd;
  }

  const stepPath = writeCwd.endsWith('/') ? writeCwd + stepFile : writeCwd + '/' + stepFile;

  // Clean up any leftover from a previous failed export, then pre-create the
  // inode so that OSD_File can open it with O_WRONLY even if OCCT's libc open()
  // omits O_CREAT in the Emscripten build.
  try { oc.FS.unlink(stepPath); } catch { /* ignore */ }
  oc.FS.writeFile(stepPath, '');

  const writeResult = writer.Write(stepFile);

  // Restore the CWD regardless of outcome.
  try { if (writeCwd !== savedCwd) oc.FS.chdir(savedCwd); } catch { /* ignore */ }

  if (writeResult !== DONE) {
    throw new Error(`STEPControl_Writer.Write failed (status ${writeResult}).`);
  }

  // STEPControl_Writer::Write() uses buffered C FILE* I/O internally.  The
  // write buffer is only flushed to Emscripten MEMFS when fclose() is called,
  // which happens inside the C++ destructor.  Explicitly delete the writer
  // object HERE — before reading the file — so that the destructor runs and
  // all buffered bytes are committed to the in-memory filesystem before we
  // attempt to read them back.
  const writerIdx = toDelete.indexOf(writer);
  if (writerIdx !== -1) toDelete.splice(writerIdx, 1);
  try { writer.delete(); } catch { /* ignore */ }

  // ── Read back the written file ──────────────────────────────────────────────
  // Helper: decode and validate a candidate path.
  const tryRead = (p) => {
    try {
      const raw = oc.FS.readFile(p);
      const text = typeof raw === 'string' ? raw : new TextDecoder().decode(raw);
      return (text && text.includes('ISO-10303')) ? text : null;
    } catch { return null; }
  };

  // Build a list of candidate locations to check in priority order.
  // The writer may resolve the bare filename relative to its internal CWD,
  // HOME, or some other Emscripten path rather than the POSIX CWD we set.
  const candidatePaths = [
    stepPath,                          // our explicit target (e.g. /tmp/brep_export.stp)
    '/' + stepFile,                    // root fallback
    '/home/web_user/' + stepFile,      // OCCT Emscripten HOME default
    '/home/' + stepFile,
  ];

  let stepContent = null;
  let usedPath = null;
  for (const p of candidatePaths) {
    const c = tryRead(p);
    if (c) { stepContent = c; usedPath = p; break; }
  }

  // Last resort: walk the filesystem (depth-limited) looking for any file
  // that contains the ISO-10303 header.  This handles exotic path resolution.
  if (!stepContent) {
    const scanDir = (dir, depth) => {
      if (depth > 2) return null;
      let entries;
      try { entries = oc.FS.readdir(dir); } catch { return null; }
      for (const e of entries) {
        if (e === '.' || e === '..') continue;
        const full = dir === '/' ? '/' + e : `${dir}/${e}`;
        try {
          const st = oc.FS.stat(full);
          if (oc.FS.isDir(st.mode)) {
            const found = scanDir(full, depth + 1);
            if (found) return found;
          } else if (st.size > 50) {
            const c = tryRead(full);
            if (c) return { path: full, content: c };
          }
        } catch { /* skip */ }
      }
      return null;
    };
    const found = scanDir('/', 0);
    if (found) { stepContent = found.content; usedPath = found.path; }
  }

  // Log the MEMFS state BEFORE cleanup so the diagnostic is actionable.
  if (!stepContent) {
    try {
      console.error('STEP Write diagnostics',
        '| writeCwd:', writeCwd,
        '| stepPath:', stepPath,
        '| candidatePaths:', candidatePaths,
        '| /tmp contents:', oc.FS.readdir('/tmp'),
        '| / contents:', oc.FS.readdir('/'),
      );
    } catch { /* ignore */ }
  }

  // Clean up all candidate inodes.
  for (const p of [...candidatePaths, ...(usedPath ? [usedPath] : [])]) {
    try { oc.FS.unlink(p); } catch { /* ignore */ }
  }

  if (!stepContent) {
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
