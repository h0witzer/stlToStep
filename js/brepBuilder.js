/**
 * brepBuilder.js — B-rep topology assembly via opencascade.js + STEP export
 *
 * Pipeline per face group:
 *   1. Build group adjacency map (groups that share mesh edges).
 *   2. Build one face per surface group:
 *        • Planar faces — compute exact corner points by intersecting every
 *          triple (this plane, neighbour_j, neighbour_k) where j and k are
 *          mutually adjacent.  Sort corners by angle in the local 2-D frame,
 *          build a wire, then call MakeFace_16(gp_Pln, wire).  Adjacent
 *          planar faces share the same corner coordinates (same 3-plane
 *          intersection result on both sides), so sewing succeeds without
 *          any tolerance relaxation.
 *        • Curved faces (cylinder / cone / sphere) — build a UV-bounded
 *          analytical patch from mesh vertex projections.  Full-revolution
 *          surfaces use U = 0…2π; the V range comes from axial projections.
 *        • Fallback — a large bounding-rectangle patch is used for planar
 *          faces whose neighbours are not all planes (e.g. a cap adjacent
 *          only to a cylinder) or whenever the analytical build fails.
 *   3. Assemble: BRepBuilderAPI_Sewing → BRep_Builder shell (fallback) →
 *        TopoDS_Compound (last resort).
 *   4. Write STEP via STEPControl_Writer with the /tmp CWD strategy.
 *
 * opencascade.js is loaded lazily via dynamic import() when the user first
 * clicks "Export STEP" so the 35 MB WASM does not block page load.
 */

// ── Build version ─────────────────────────────────────────────────────────────

/** Increment this string with each release to verify live-site deployments. */
export const BUILD_VERSION = 'v0.6.0';

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

// ── Plane–plane intersection utilities ─────────────────────────────────────

/**
 * Find the common point of three planes, each given as (unit normal, origin).
 * Uses the cross-product form of Cramer's rule:
 *   x = (d1*(n2×n3) + d2*(n3×n1) + d3*(n1×n2)) / det,  det = n1·(n2×n3)
 *
 * Returns null when the planes are not in general position (parallel or
 * forming a prismatic pencil rather than a single intersection point).
 *
 * @param {number[]} n1,n2,n3  unit normals
 * @param {number[]} o1,o2,o3  reference points on each plane
 * @returns {number[]|null}
 */
function _intersect3Planes(n1, o1, n2, o2, n3, o3) {
  const d1  = _d3(n1, o1);
  const d2  = _d3(n2, o2);
  const d3  = _d3(n3, o3);
  const c23 = _x3(n2, n3);
  const det = _d3(n1, c23);
  if (Math.abs(det) < 1e-12) return null;
  const c31 = _x3(n3, n1);
  const c12 = _x3(n1, n2);
  return [
    (d1 * c23[0] + d2 * c31[0] + d3 * c12[0]) / det,
    (d1 * c23[1] + d2 * c31[1] + d3 * c12[1]) / det,
    (d1 * c23[2] + d2 * c31[2] + d3 * c12[2]) / det,
  ];
}

/**
 * Compute the exact boundary polygon for a planar face group.
 *
 * For every pair of mutually-adjacent planar neighbours (j, k) of face gi,
 * intersect the three planes (gi, j, k) to obtain a corner vertex.
 * The corners are sorted CCW by angle in the face's local 2-D frame and
 * returned as an ordered polygon loop.
 *
 * Returns null when fewer than 3 unique corners are found — the caller should
 * fall back to a mesh-vertex bounding-rectangle patch in that case.
 *
 * @param {number}   gi
 * @param {object[]} groups    with `.surface = { type, params }` set
 * @param {Map}      adjacency  from buildGroupAdjacencyMap()
 * @returns {Array<[number,number,number]>|null}
 */
function _analyticalPlaneBoundary(gi, groups, adjacency) {
  const { params: pi } = groups[gi].surface;
  const ni  = _u3(pi.normal);
  const oi  = pi.origin;
  const nbrs = [...(adjacency.get(gi) ?? [])];
  const corners = [];

  for (let a = 0; a < nbrs.length; a++) {
    for (let b = a + 1; b < nbrs.length; b++) {
      const j = nbrs[a], k = nbrs[b];
      // Both j and k must also be adjacent to each other to form a corner.
      if (!adjacency.get(j)?.has(k)) continue;

      const gj = groups[j], gk = groups[k];
      if (!gj?.surface || !gk?.surface) continue;
      // Only plane–plane–plane triples yield a point corner.
      if (gj.surface.type !== 'plane' || gk.surface.type !== 'plane') continue;

      const nj = _u3(gj.surface.params.normal);
      const nk = _u3(gk.surface.params.normal);
      const pt = _intersect3Planes(ni, oi, nj, gj.surface.params.origin,
                                             nk, gk.surface.params.origin);
      if (!pt) continue;

      // Deduplicate: same triple may appear from different orderings.
      if (!corners.some(c =>
        Math.abs(c[0] - pt[0]) < 1e-8 &&
        Math.abs(c[1] - pt[1]) < 1e-8 &&
        Math.abs(c[2] - pt[2]) < 1e-8,
      )) corners.push(pt);
    }
  }

  if (corners.length < 3) return null;

  // Sort CCW around the face normal so they form a proper polygon.
  const ref = Math.abs(ni[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
  const xA  = _u3(_x3(ni, ref));
  const yA  = _x3(ni, xA);
  corners.sort((a, b) => {
    const au = (a[0]-oi[0])*xA[0] + (a[1]-oi[1])*xA[1] + (a[2]-oi[2])*xA[2];
    const av = (a[0]-oi[0])*yA[0] + (a[1]-oi[1])*yA[1] + (a[2]-oi[2])*yA[2];
    const bu = (b[0]-oi[0])*xA[0] + (b[1]-oi[1])*xA[1] + (b[2]-oi[2])*xA[2];
    const bv = (b[0]-oi[0])*yA[0] + (b[1]-oi[1])*yA[1] + (b[2]-oi[2])*yA[2];
    return Math.atan2(av, au) - Math.atan2(bv, bu);
  });
  return corners;
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
 * @returns {object|null}  TopoDS_Face or null
 */
function _buildLargePatch(oc, group, geometry, toDelete) {
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
      const m = Math.max(umax - umin, vmax - vmin) * 0.25 + 1e-2;
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
      const pad = (vr.vmax - vr.vmin) * 0.25 + 1e-2;
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
      const pad = (vr.vmax - vr.vmin) * 0.25 + 1e-2;
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
      const pad = Math.max((vr.vmax - vr.vmin) * 0.25 + 1e-2, 0);
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

  const adjacency = buildGroupAdjacencyMap(groups, geometry);
  const toDelete  = [];

  // ── Build one face per surface group ─────────────────────────────────────────
  //
  // Planar faces: compute exact boundary corners from pairwise plane–plane–plane
  // intersections.  Adjacent faces share the same corner coordinates (same
  // 3-plane intersection result computed on both sides), so sewing succeeds.
  // Falls back to the oversized large-patch rectangle when the analytical build
  // fails (e.g. a flat cap whose only non-plane neighbour is a cylinder).
  //
  // Curved faces (cylinder / cone / sphere): build a UV-bounded patch from
  // mesh vertex projections.  Full-revolution surfaces use U = 0…2π; the V
  // range is derived from axial vertex projections.
  const faces = [];
  for (let i = 0; i < groups.length; i++) {
    const g = groups[i];
    if (!g.surface) continue;
    if (i % 50 === 0) onStatus?.(`Building faces… ${i}/${groups.length}`, 20 + 20 * i / groups.length);

    let face = null;

    // For planar faces, prefer the exact analytical boundary.
    if (g.surface.type === 'plane') {
      const loop = _analyticalPlaneBoundary(i, groups, adjacency);
      if (loop) {
        try {
          const { origin, normal } = g.surface.params;
          const nu  = _u3(normal);
          const pln = new oc.gp_Pln_3(makePnt(oc, origin), makeDir(oc, nu));
          toDelete.push(pln);
          const wire = buildWire(oc, loop, toDelete);
          if (wire) {
            const mf = new oc.BRepBuilderAPI_MakeFace_16(pln, wire, true);
            toDelete.push(mf);
            if (mf.IsDone()) face = mf.Face();
          }
        } catch (e) {
          console.warn(`Analytical planar face ${i}:`, e?.message ?? e);
        }
      }
    }

    // Fallback: large UV-bounded patch (primary path for curved surfaces).
    if (!face) {
      try {
        face = _buildLargePatch(oc, g, geometry, toDelete);
      } catch (e) {
        console.warn(`Patch ${i} (${g.surface?.type}):`, e?.message ?? e);
      }
    }

    if (face) faces.push(face);
  }

  if (faces.length === 0) throw new Error('No valid B-rep faces could be constructed.');

  onStatus?.(`Sewing ${faces.length} faces into a solid…`, 40);

  // ── Build a watertight solid ────────────────────────────────────────────────
  //
  // Analytically-bounded planar faces share exact corner coordinates so
  // BRepBuilderAPI_Sewing will merge their common edges and produce a true
  // manifold shell for all-planar models (polyhedra, prismatic parts, …).

  const sewTol = options.sewTol ?? 1e-6;
  let topShape = null;

  // ── Strategy 1: BRepBuilderAPI_Sewing ──────────────────────────────────────
  try {
    // BRepBuilderAPI_Sewing constructor: _1() default tol, _2(tol, opts…).
    // Use typeof guard to avoid accidentally calling an undefined constructor.
    const SewCtor = typeof oc.BRepBuilderAPI_Sewing_2 === 'function'
      ? oc.BRepBuilderAPI_Sewing_2
      : typeof oc.BRepBuilderAPI_Sewing_1 === 'function'
        ? oc.BRepBuilderAPI_Sewing_1
        : oc.BRepBuilderAPI_Sewing;
    const sewing = new SewCtor(sewTol);
    toDelete.push(sewing);

    for (const f of faces) sewing.Add(f);

    // Perform() has an optional Handle<Message_ProgressIndicator> default arg.
    // Some Emscripten builds handle the no-arg call; others throw — we catch.
    sewing.Perform();

    const sewn = sewing.SewedShape();

    // SewedShape() may return a Shell, Solid, or Compound.
    const shapeType = sewn.ShapeType?.() ?? -1;
    const SOLID_T   = oc.TopAbs_ShapeEnum?.TopAbs_SOLID   ?? 3;
    const SHELL_T   = oc.TopAbs_ShapeEnum?.TopAbs_SHELL   ?? 4;
    const COMP_T    = oc.TopAbs_ShapeEnum?.TopAbs_COMPOUND ?? 0;

    if (shapeType === SOLID_T) {
      topShape = sewn;
    } else if (shapeType === SHELL_T) {
      const mkSolid = new oc.BRepBuilderAPI_MakeSolid_3(sewn);
      toDelete.push(mkSolid);
      if (mkSolid.IsDone()) topShape = mkSolid.Solid();
    } else if (shapeType === COMP_T) {
      // Sewing produced a compound (multiple disconnected shells).
      // Try to find the largest shell inside it and wrap that.
      try {
        const expShell = new oc.TopExp_Explorer_2(
          sewn,
          oc.TopAbs_ShapeEnum?.TopAbs_SHELL ?? 4,
          oc.TopAbs_ShapeEnum?.TopAbs_SHAPE ?? 0,
        );
        toDelete.push(expShell);
        let bestShell = null;
        while (expShell.More()) {
          const s = expShell.Current();
          if (!bestShell) bestShell = s;
          expShell.Next();
        }
        if (bestShell) {
          const mkSolid = new oc.BRepBuilderAPI_MakeSolid_3(bestShell);
          toDelete.push(mkSolid);
          if (mkSolid.IsDone()) topShape = mkSolid.Solid();
        }
      } catch { /* fall through to strategy 2 */ }
    }

    if (topShape) console.info(`Sewing succeeded — solid built from sewn shape (type ${shapeType}).`);
  } catch (sewErr) {
    console.warn('BRepBuilderAPI_Sewing unavailable or failed:', sewErr?.message ?? sewErr);
  }

  // ── Strategy 2: manual shell → solid (no sewing) ───────────────────────────
  if (!topShape) {
    try {
      const brepBuilder = new oc.BRep_Builder();
      const shell = new oc.TopoDS_Shell();
      toDelete.push(shell);
      brepBuilder.MakeShell(shell);
      for (const f of faces) brepBuilder.Add(shell, f);

      const mkSolid = new oc.BRepBuilderAPI_MakeSolid_3(shell);
      toDelete.push(mkSolid);
      if (mkSolid.IsDone()) {
        topShape = mkSolid.Solid();
        console.info('Using unsewn shell→solid fallback.');
      }
    } catch (shErr) {
      console.warn('Shell→Solid failed:', shErr?.message ?? shErr);
    }
  }

  // ── Strategy 3: bare compound (last resort, still imports as surfaces) ──────
  if (!topShape) {
    console.warn('All solid strategies failed — falling back to TopoDS_Compound.');
    const brepBuilder = new oc.BRep_Builder();
    const compound = new oc.TopoDS_Compound();
    toDelete.push(compound);
    brepBuilder.MakeCompound(compound);
    for (const f of faces) brepBuilder.Add(compound, f);
    topShape = compound;
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
