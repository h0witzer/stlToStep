/**
 * brepBuilder.js — B-rep topology assembly via opencascade.js + STEP export
 *
 * Pipeline:
 *   1. Build one generously oversized analytical face per fitted surface group.
 *      All surface types (plane, cylinder, cone, sphere) use _buildLargePatch()
 *      with margins proportional to the model bounding-box diagonal so every
 *      face extends well beyond any possible intersection with its neighbours.
 *   2. Feed all oversized faces into BRepAlgoAPI_Splitter as both Arguments
 *      and Tools.  The kernel finds every surface–surface intersection (lines,
 *      circles, ellipses, conics, …) simultaneously and trims every face into
 *      the exact fragments bounded by those intersections — no surface-type
 *      enumeration, no manual wire-building, no vertex math.
 *   3. For each input group, keep the one output fragment whose centroid (via
 *      BRepGProp.SurfaceProperties) is closest to the group's surface origin.
 *      This discards the "exterior" flaps from the oversized patches.
 *   4. Sew the kept fragments into a watertight shell → solid.
 *   5. Write STEP via STEPControl_Writer.
 *
 * opencascade.js is loaded lazily via dynamic import() when the user first
 * clicks "Export STEP" so the 35 MB WASM does not block page load.
 */

// ── Build version ─────────────────────────────────────────────────────────────

/** Increment this string with each release to verify live-site deployments. */
export const BUILD_VERSION = 'v0.2.13';

// ── OpenCASCADE lazy loader ───────────────────────────────────────────────────

// opencascade.js 2.0 beta — full OCCT 7.6.2 bindings with proper TypeScript types.
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

/**
 * Partition face groups into disconnected mesh islands using BFS over the
 * group adjacency map.  Each island is the set of group indices that are
 * transitively connected by shared mesh edges.
 *
 * A multi-body STL (e.g. a part + a separate washer) produces multiple
 * islands.  Processing each island independently avoids spurious cross-body
 * face intersections inside BRepAlgoAPI_Section.
 *
 * @param {Map<number, Set<number>>} adjacency  from buildGroupAdjacencyMap()
 * @param {number} groupCount  total number of groups (indices 0…groupCount-1)
 * @returns {Array<number[]>}  one entry per island; each entry is a sorted
 *   array of group indices belonging to that island
 */
export function findMeshIslands(adjacency, groupCount) {
  const visited = new Set();
  const islands = [];

  for (let start = 0; start < groupCount; start++) {
    if (visited.has(start)) continue;

    // BFS from this unvisited group.
    const island = [];
    const queue = [start];
    visited.add(start);
    let head = 0;  // read-index pointer avoids O(N) shift()

    while (head < queue.length) {
      const cur = queue[head++];
      island.push(cur);
      for (const nb of (adjacency.get(cur) ?? [])) {
        if (!visited.has(nb)) {
          visited.add(nb);
          queue.push(nb);
        }
      }
    }

    islands.push(island);
  }

  return islands;
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

// ── Splitter-based solid builder ────────────────────────────────────────────

/**
 * Compute the centroid of a TopoDS_Face using BRepGProp.SurfaceProperties.
 *
 * @param {object}   oc
 * @param {object}   face     TopoDS_Face
 * @param {object[]} toDelete
 * @returns {number[]|null}  [x, y, z] centroid or null on failure
 */
function _faceGPropCentroid(oc, face, toDelete) {
  try {
    const props = new oc.GProp_GProps_1();
    toDelete.push(props);
    oc.BRepGProp.SurfaceProperties_1(face, props, false, false);
    const com = props.CentreOfMass();
    toDelete.push(com);
    return [com.X(), com.Y(), com.Z()];
  } catch {
    return null;
  }
}

/**
 * Build a watertight solid using BRepAlgoAPI_Splitter:
 *
 *  1. Feed all oversized analytical patches as both Arguments and Tools so the
 *     Splitter fragments every surface by every other surface simultaneously.
 *     OCCT handles all curve types (lines, circles, ellipses, conics) and all
 *     angle combinations internally — no surface-type enumeration needed.
 *
 *  2. Filter the resulting face fragments: keep only the fragment whose
 *     centroid (via BRepGProp.SurfaceProperties) is closest to the original
 *     mesh group's average vertex position.  One winning fragment per group.
 *
 *  3. Sew the winning fragments into a watertight shell → solid.
 *
 * @param {object}   oc
 * @param {object[]} faceEntries  [{face, group, groupIdx}]
 * @param {number}   sewTol
 * @param {object[]} toDelete
 * @returns {object|null}  TopoDS_Solid/Shell or null
 */
function _buildSolidViaSplitter(oc, faceEntries, sewTol, toDelete, geometry) {
  if (faceEntries.length === 0) return null;

  // ── Phase 1: Run the Splitter ───────────────────────────────────────────
  // BRepAlgoAPI_Splitter (OCCT 7.3+) takes Arguments (shapes to be split)
  // and Tools (shapes that do the splitting).  By adding every oversized face
  // as both an Argument and a Tool every surface is cut by every other surface
  // in one pass — the kernel finds all intersections at once and produces
  // shared edges at every junction.
  let splitter = null;
  let allPieces = null;

  // opencascade.js 2.0 beta exports the default ctor as BRepAlgoAPI_Splitter_1.
  const SplitterCtor = oc.BRepAlgoAPI_Splitter_1 ?? oc.BRepAlgoAPI_Splitter;
  if (typeof SplitterCtor !== 'function') {
    console.warn('BRepAlgoAPI_Splitter not available in this opencascade.js build.');
    return null;
  }

  try {
    splitter = new SplitterCtor();
    toDelete.push(splitter);

    // BRepAlgoAPI_Splitter inherits from BRepAlgoAPI_BuilderShape.
    // In OCCT 7.x the argument API is:
    //   • AddArgument(TopoDS_Shape) / AddTool(TopoDS_Shape) — on some subclasses
    //   • SetArguments(TopTools_ListOfShape) / SetTools(TopTools_ListOfShape) — always present
    // opencascade.js 2.0 beta only exports SetArguments/SetTools for this class.
    // Try AddArgument first (forward-compat), fall back to list API.
    if (typeof splitter.AddArgument === 'function') {
      for (const { face } of faceEntries) {
        splitter.AddArgument(face);
        splitter.AddTool(face);
      }
    } else {
      // Build TopTools_ListOfShape lists and use SetArguments / SetTools.
      const ListCtor = oc.TopTools_ListOfShape_1 ?? oc.TopTools_ListOfShape;
      if (typeof ListCtor !== 'function') {
        console.warn('TopTools_ListOfShape not available; cannot set Splitter arguments.');
        return null;
      }
      const argList  = new ListCtor();
      const toolList = new ListCtor();
      toDelete.push(argList, toolList);
      for (const { face } of faceEntries) {
        // Append_1 is the single-element overload in opencascade.js 2.0 beta.
        const append = typeof argList.Append_1 === 'function' ? 'Append_1' : 'Append';
        argList[append](face);
        toolList[append](face);
      }
      splitter.SetArguments(argList);
      splitter.SetTools(toolList);
    }

    splitter.SetRunParallel(false);
    const range = _mkRange(oc);
    try { splitter.Build(range ?? undefined); } catch { splitter.Build(); }

    if (!splitter.IsDone()) {
      console.warn('BRepAlgoAPI_Splitter.Build() failed (IsDone=false).');
      return null;
    }

    allPieces = splitter.Shape();
  } catch (e) {
    console.warn('BRepAlgoAPI_Splitter error:', e?.message ?? e);
    return null;
  }

  if (!allPieces || (typeof allPieces.IsNull === 'function' && allPieces.IsNull())) {
    return null;
  }

  // ── Phase 2: Filter face fragments by proximity to mesh group centroids ──
  // For each input group, compute its centroid from mesh triangle vertices
  // (the most accurate representative point on each original surface patch).
  // Fall back to the surface params origin if geometry is unavailable.
  const pos = geometry?.attributes?.position;
  const groupCentroids = faceEntries.map(({ group, groupIdx }) => {
    let cx = 0, cy = 0, cz = 0, n = 0;
    if (pos && group.triangleIndices) {
      for (const t of group.triangleIndices) {
        for (let v = 0; v < 3; v++) {
          const i = t * 3 + v;
          cx += pos.getX(i); cy += pos.getY(i); cz += pos.getZ(i); n++;
        }
      }
    }
    if (n === 0) {
      // Fallback: use surface params origin / axisPoint / apex / center.
      const p = group.surface.params;
      const pt = p.origin ?? p.axisPoint ?? p.apex ?? p.center ?? [0, 0, 0];
      cx = pt[0]; cy = pt[1]; cz = pt[2]; n = 1;
    }
    return { groupIdx, cx: cx/n, cy: cy/n, cz: cz/n };
  });

  const FACE_T  = oc.TopAbs_ShapeEnum?.TopAbs_FACE  ?? 4;
  const SHAPE_T = oc.TopAbs_ShapeEnum?.TopAbs_SHAPE ?? 0;

  // Map groupIdx → {face, distSq} — best (closest) fragment per group.
  const best = new Map();
  for (const { groupIdx } of faceEntries) best.set(groupIdx, { face: null, distSq: Infinity });

  try {
    const exp = new oc.TopExp_Explorer_2(allPieces, FACE_T, SHAPE_T);
    toDelete.push(exp);
    while (exp.More()) {
      let fragFace;
      try { fragFace = oc.TopoDS.Face_1 ? oc.TopoDS.Face_1(exp.Current()) : exp.Current(); }
      catch { fragFace = exp.Current(); }

      const c = _faceGPropCentroid(oc, fragFace, toDelete);
      if (c) {
        // Find the nearest group centroid.
        let bestGroupIdx = -1, bestDist = Infinity;
        for (const g of groupCentroids) {
          const d = (c[0]-g.cx)**2 + (c[1]-g.cy)**2 + (c[2]-g.cz)**2;
          if (d < bestDist) { bestDist = d; bestGroupIdx = g.groupIdx; }
        }
        if (bestGroupIdx >= 0) {
          const slot = best.get(bestGroupIdx);
          if (slot && bestDist < slot.distSq) {
            slot.face = fragFace;
            slot.distSq = bestDist;
          }
        }
      }

      exp.Next();
    }
  } catch (e) {
    console.warn('Splitter face exploration error:', e?.message ?? e);
    return null;
  }

  const keptFaces = [];
  for (const { face, distSq } of best.values()) {
    if (face && isFinite(distSq)) keptFaces.push(face);
  }

  if (keptFaces.length === 0) {
    console.warn('Splitter produced no usable face fragments.');
    return null;
  }

  console.info(`Splitter: ${keptFaces.length}/${faceEntries.length} face fragments kept.`);

  // ── Phase 3: Sew into solid ─────────────────────────────────────────────
  return _sewIntoSolid(oc, keptFaces, sewTol, toDelete);
}

/**
 * Sew an array of properly trimmed faces into a watertight solid.
 */
function _sewIntoSolid(oc, faces, sewTol, toDelete) {
  if (faces.length === 0) return null;
  try {
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

// ── Main solid builder ──────────────────────────────────────────────────────

/**
 * Build a watertight solid from oversized analytical faces via
 * BRepAlgoAPI_Splitter: all faces shatter each other simultaneously, then
 * the fragment closest to each original mesh group centroid is kept and
 * the kept fragments are sewn into a solid.
 */
function _buildSolid(oc, faceEntries, adjacency, groups,
                     modelDiag, sewTol, toDelete, geometry) {
  return _buildSolidViaSplitter(oc, faceEntries, sewTol, toDelete, geometry);
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
  // geometry kernel via Section-based trimming — never by mesh
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

  // ── Detect mesh islands ─────────────────────────────────────────────────────
  // A multi-body STL (e.g. a part + a separate washer) has multiple connected
  // components in the group adjacency graph.  We process each island completely
  // independently so that face patches from one body never enter the boolean
  // operations of another.
  const islands = findMeshIslands(adjacency, groups.length);
  console.info(`Mesh island detection: ${islands.length} island(s).`);

  // Map groupIdx → faceEntry for O(1) lookup when splitting by island.
  const faceEntryByGroup = new Map(faceEntries.map(e => [e.groupIdx, e]));

  // ── Build one solid per island ──────────────────────────────────────────────
  // Each island is run through the full solid-building pipeline independently.
  // Results are transferred to a single STEPControl_Writer so the output file
  // contains all bodies together (as separate product solids).
  const islandSolids = [];
  for (let ii = 0; ii < islands.length; ii++) {
    const islandGroupIdxs = islands[ii];
    const islandFaceEntries = islandGroupIdxs
      .map(gi => faceEntryByGroup.get(gi))
      .filter(Boolean);

    if (islandFaceEntries.length === 0) continue;

    onStatus?.(
      `Building solid ${ii + 1}/${islands.length} (${islandFaceEntries.length} faces)…`,
      40 + 35 * ii / islands.length,
    );

    const solid = _buildSolid(
      oc, islandFaceEntries, adjacency, groups, modelDiag, sewTol, toDelete, geometry);

    if (solid) {
      islandSolids.push(solid);
    } else {
      console.warn(`Island ${ii + 1}/${islands.length}: solid construction failed — skipped.`);
    }
  }

  if (islandSolids.length === 0) {
    throw new Error(
      'Solid construction failed for all islands.  Section-based trimming ' +
      'could not produce a watertight solid from ' +
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

  // Transfer each island solid into the writer.  STEPControl_Writer accumulates
  // multiple Transfer() calls before a single Write() — each Transfer becomes
  // one product shape in the output file.
  let transferred = 0;
  for (const solid of islandSolids) {
    const transferResult = writer.Transfer(
      solid,
      oc.STEPControl_StepModelType?.STEPControl_AsIs ?? 0,
      true,
      _mkRange(oc),
    );
    if (transferResult === DONE) {
      transferred++;
    } else {
      console.warn(`STEPControl_Writer.Transfer returned status ${transferResult} for an island — skipping.`);
    }
  }

  if (transferred === 0) {
    throw new Error(`STEPControl_Writer.Transfer failed for all ${islandSolids.length} island(s).`);
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
