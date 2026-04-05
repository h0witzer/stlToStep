/**
 * brepBuilder.js — B-rep topology assembly via opencascade.js + STEP export
 *
 * Pipeline per face group:
 *   1. Build group adjacency map (groups that share mesh edges)
 *   2. Face boundaries derived PURELY from analytical surface parameters:
 *        plane cap on cylinder/cone/sphere → circle/ellipse from axis–plane
 *          intersection (pure JS, no IntAna_QuadQuadGeo dependency)
 *        other planar face → polygon from 3-plane intersections
 *        cylinder / cone → MakeFace_10/11 (UV-bounds only); V extents from
 *          axis–plane intersections with neighbouring plane groups, falling
 *          back to a triangle-vertex scan of the group itself
 *        sphere → MakeFace_12 (UV-bounds); latitude from vertex scan
 *      Mesh edge topology is NOT used at this stage.
 *   3. Build an OCCT face from the analytical parameters only.
 *   4. Sew all faces into a TopoDS_Shell → TopoDS_Solid (MANIFOLD_SOLID_BREP)
 *   5. Write STEP via STEPControl_Writer; chdir('/') first so the bare filename
 *      is always resolved to '/', then pre-create the inode for O_WRONLY safety
 *
 * opencascade.js is loaded lazily via dynamic import() when the user first
 * clicks "Export STEP" so the 35 MB WASM does not block page load.
 */

// ── Build version ─────────────────────────────────────────────────────────────

/** Increment this string with each release to verify live-site deployments. */
export const BUILD_VERSION = 'v0.4.8';

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

// ── Analytical boundary helpers ──────────────────────────────────────────────

function _d3(a, b)  { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
function _x3(a, b)  {
  return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
}
function _n3(v)     { return Math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
function _u3(v)     { const n = _n3(v); return n > 1e-14 ? [v[0]/n, v[1]/n, v[2]/n] : v; }

/**
 * Compute the unique point at the intersection of three planes.
 * Each plane is given by (unitNormal, pointOnPlane).
 * Returns null when the determinant is below threshold (parallel planes).
 */
function _intersect3Planes(n1, o1, n2, o2, n3, o3) {
  const d1 = _d3(n1, o1), d2 = _d3(n2, o2), d3 = _d3(n3, o3);
  const c23 = _x3(n2, n3);
  const det = _d3(n1, c23);
  if (Math.abs(det) < 1e-10) return null;
  const c31 = _x3(n3, n1);
  const c12 = _x3(n1, n2);
  return [
    (d1*c23[0] + d2*c31[0] + d3*c12[0]) / det,
    (d1*c23[1] + d2*c31[1] + d3*c12[1]) / det,
    (d1*c23[2] + d2*c31[2] + d3*c12[2]) / det,
  ];
}

/**
 * Compute the analytical boundary loop for a planar face.
 *
 * For each pair of planar neighbours (j, k) that are BOTH adjacent to face i
 * AND adjacent to each other, the three planes (i, j, k) share exactly one
 * corner point. Collecting all such corners and sorting them by azimuth
 * around the face normal yields the correct boundary polygon.
 *
 * This completely bypasses mesh edge topology so the resulting face extent is
 * defined by the intersection of analytical surfaces, not by triangulation
 * artefacts.
 *
 * @param {number}   gi         group index of the planar face
 * @param {object[]} groups     all groups with fitted surfaces
 * @param {Map}      adjacency  output of buildGroupAdjacencyMap()
 * @returns {Array<[number,number,number]>|null}  ordered corners, or null
 */
function _analyticalPlaneBoundary(gi, groups, adjacency) {
  const { params: pi } = groups[gi].surface;
  const ni = pi.normal, oi = pi.origin;

  // Only planar neighbours can contribute straight-line boundaries.
  const planeNeighbors = [...(adjacency.get(gi) ?? [])]
    .filter(j => groups[j].surface?.type === 'plane');

  if (planeNeighbors.length < 3) return null;

  const rawCorners = [];

  for (let a = 0; a < planeNeighbors.length; a++) {
    for (let b = a + 1; b < planeNeighbors.length; b++) {
      const j = planeNeighbors[a];
      const k = planeNeighbors[b];
      // j and k must be directly adjacent to each other to form a corner
      if (!adjacency.get(j)?.has(k)) continue;
      const pj = groups[j].surface.params;
      const pk = groups[k].surface.params;
      const pt = _intersect3Planes(ni, oi, pj.normal, pj.origin, pk.normal, pk.origin);
      if (pt) rawCorners.push(pt);
    }
  }

  if (rawCorners.length < 3) return null;

  // Deduplicate numerically coincident corners
  const CTOL2 = 1e-8; // squared distance threshold
  const corners = [];
  for (const pt of rawCorners) {
    if (!corners.some(c => {
      const dx=pt[0]-c[0], dy=pt[1]-c[1], dz=pt[2]-c[2];
      return dx*dx+dy*dy+dz*dz < CTOL2;
    })) corners.push(pt);
  }

  if (corners.length < 3) return null;

  // Sort corners by azimuth angle around the face normal so the polygon is
  // wound consistently (required by OCCT's wire builder).
  const cx = corners.reduce((s, p) => s + p[0], 0) / corners.length;
  const cy = corners.reduce((s, p) => s + p[1], 0) / corners.length;
  const cz = corners.reduce((s, p) => s + p[2], 0) / corners.length;
  const nu = _u3(ni);
  // Local X axis: pick a reference direction perpendicular to the face normal
  const ref = Math.abs(nu[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
  const lx = _u3(_x3(nu, ref));
  const ly = _x3(nu, lx); // second in-plane axis (nu × lx)

  corners.sort((a, b) => {
    const [dax, day, daz] = [a[0]-cx, a[1]-cy, a[2]-cz];
    const [dbx, dby, dbz] = [b[0]-cx, b[1]-cy, b[2]-cz];
    return Math.atan2(_d3([dax,day,daz], ly), _d3([dax,day,daz], lx))
         - Math.atan2(_d3([dbx,dby,dbz], ly), _d3([dbx,dby,dbz], lx));
  });

  return corners;
}

// ── Analytical cylinder boundary helpers ─────────────────────────────────────

/**
 * Compute the cylinder's axial V extents analytically by projecting each
 * adjacent planar group's origin onto the cylinder axis.
 *
 * This replaces mesh-topology V computation so that the cylinder face ends
 * exactly where neighbouring flat faces lie — making BRepBuilderAPI_Sewing
 * able to stitch the cylinder to those planes.
 *
 * @param {number}   gi         group index of the cylinder
 * @param {object[]} groups     all groups
 * @param {Map}      adjacency
 * @param {number[]} axisPoint  [x,y,z] point on the cylinder axis
 * @param {number[]} axis       unit vector along the axis
 * @returns {{ vmin: number, vmax: number } | null}
 */
function _cylinderVExtentsFromNeighbors(gi, groups, adjacency, axisPoint, axis) {
  const ax = _u3(axis);
  const vs = [];
  for (const j of adjacency.get(gi) ?? []) {
    const s = groups[j]?.surface;
    if (!s) continue;
    if (s.type === 'plane') {
      // Find where the cylinder/cone axis pierces this plane.
      // This is the V value at which the analytical cap circle sits, so using
      // it here ensures the cylinder face circles align exactly with the cap.
      //   axis · (axisPoint + t*ax - planeOrigin) · n = 0
      //   t = (planeOrigin - axisPoint) · n / (ax · n)
      const n = _u3(s.params.normal);
      const denom = ax[0]*n[0] + ax[1]*n[1] + ax[2]*n[2];
      if (Math.abs(denom) < 1e-6) continue; // axis parallel to plane → not a cap
      const o = s.params.origin;
      const t = ((o[0]-axisPoint[0])*n[0] +
                 (o[1]-axisPoint[1])*n[1] +
                 (o[2]-axisPoint[2])*n[2]) / denom;
      vs.push(t);
    } else if (s.type === 'cylinder') {
      // Co-axial cylinder neighbour: project its axis-point for approximate bound
      const refPt = s.params.axisPoint;
      vs.push(
        (refPt[0] - axisPoint[0]) * ax[0] +
        (refPt[1] - axisPoint[1]) * ax[1] +
        (refPt[2] - axisPoint[2]) * ax[2],
      );
    }
  }
  if (vs.length < 1) return null;
  return { vmin: Math.min(...vs), vmax: Math.max(...vs) };
}

/**
 * Scan all triangle vertices in a group to find the axial V extent.
 * Used as a last-resort fallback when no planar neighbours are available.
 * Unlike extractBoundaryLoop, this uses only vertex positions (no edge topology).
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
 * Latitude is measured from the Z-up frame centred at the sphere's centre,
 * matching the gp_Ax3 used in _buildSphereFace.
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
      const loop = _analyticalPlaneBoundary(gi, groups, adjacency);
      if (loop) boundaries.set(gi, { type: 'plane', loop });
    } else if (type === 'cylinder') {
      const vr = _cylinderVExtentsFromNeighbors(gi, groups, adjacency,
        params.axisPoint, params.axis);
      boundaries.set(gi, {
        type: 'cylinder',
        vmin: vr?.vmin ?? null,
        vmax: vr?.vmax ?? null,
      });
    } else if (type === 'cone') {
      const { apex, axis } = params;
      const ax = _u3(axis);
      let vmin = Infinity, vmax = -Infinity;
      for (const j of adjacency.get(gi) ?? []) {
        const s = groups[j]?.surface;
        if (!s) continue;
        let refPt = null;
        if (s.type === 'plane') refPt = s.params.origin;
        if (!refPt) continue;
        const v = (refPt[0]-apex[0])*ax[0] + (refPt[1]-apex[1])*ax[1] + (refPt[2]-apex[2])*ax[2];
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
      }
      boundaries.set(gi, {
        type: 'cone',
        vmin: isFinite(vmin) ? vmin : null,
        vmax: isFinite(vmax) ? vmax : null,
      });
    } else if (type === 'sphere') {
      boundaries.set(gi, { type: 'sphere' });
    }
  }

  return { adjacency, boundaries };
}



/**
 * Remove collinear intermediate vertices from a closed 3-D polygon loop.
 *
 * A vertex is considered collinear (and removed) when the sine of the turn
 * angle between the incoming and outgoing edge is below `angleTol`.  Multiple
 * passes are made until the loop is stable.  Returns the original loop if the
 * reduced version would have fewer than 3 vertices.
 *
 * @param {Array<[number,number,number]>} loop
 * @param {number} [angleTol=1e-4]  sine-of-angle threshold
 * @returns {Array<[number,number,number]>}
 */
function simplifyLoop(loop, angleTol = 1e-4) {
  if (loop.length <= 3) return loop;
  let pts = loop.slice();
  let changed = true;
  while (changed && pts.length > 3) {
    changed = false;
    const next = [];
    const n = pts.length;
    for (let i = 0; i < n; i++) {
      const prev = pts[(i + n - 1) % n];
      const curr = pts[i];
      const nxt  = pts[(i + 1) % n];
      const d1x = curr[0]-prev[0], d1y = curr[1]-prev[1], d1z = curr[2]-prev[2];
      const d2x = nxt[0]-curr[0],  d2y = nxt[1]-curr[1],  d2z = nxt[2]-curr[2];
      const l1 = Math.sqrt(d1x*d1x+d1y*d1y+d1z*d1z);
      const l2 = Math.sqrt(d2x*d2x+d2y*d2y+d2z*d2z);
      if (l1 < 1e-14 || l2 < 1e-14) { next.push(curr); continue; }
      // |cross(d1,d2)| / (|d1|*|d2|) = sin of turn angle
      const cx = d1y*d2z-d1z*d2y, cy = d1z*d2x-d1x*d2z, cz = d1x*d2y-d1y*d2x;
      const sinA = Math.sqrt(cx*cx+cy*cy+cz*cz) / (l1*l2);
      if (sinA > angleTol) {
        next.push(curr);
      } else {
        changed = true; // drop this collinear vertex
      }
    }
    if (next.length < 3) return pts; // safety: don't over-reduce
    pts = next;
  }
  return pts;
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
 * All boundaries are derived purely from the analytical surface parameters —
 * no mesh edge topology is used at this stage.  If the analytical geometry is
 * insufficient to build a face, the face is skipped (returns null) rather than
 * falling back to mesh-polygon approximations.  Mixing analytical circles with
 * mesh-polygon approximations during sewing would cause OCCT to split the
 * analytical circle edges into hundreds of tiny edgelets to match the polygon
 * vertices — exactly the artefact this architecture is designed to prevent.
 *
 * @param {object} oc
 * @param {object} group          { triangleIndices, surface: {type, params} }
 * @param {THREE.BufferGeometry} geometry
 * @param {object[]} toDelete
 * @param {number}   [groupIdx]   index of this group in allGroups
 * @param {object[]} [allGroups]  all groups (needed for analytical boundary)
 * @param {Map}      [adjacency]  output of buildGroupAdjacencyMap()
 * @returns {object|null}  TopoDS_Face or null
 */
function buildFace(oc, group, geometry, toDelete, groupIdx, allGroups, adjacency) {
  const { type, params } = group.surface;

  // ── Planar faces ────────────────────────────────────────────────────────────
  if (type === 'plane') {
    if (allGroups && adjacency) {
      // Priority 1: Planar face adjacent to a curved surface → the shared
      // boundary is an analytic circle/ellipse computed by IntAna_QuadQuadGeo.
      for (const j of adjacency.get(groupIdx) ?? []) {
        const s = allGroups[j]?.surface;
        if (!s || !['cylinder', 'cone', 'sphere'].includes(s.type)) continue;
        const face = _buildAnalyticalCapFace(oc, params, s, toDelete);
        if (face) return face;
      }

      // Priority 2: Planar face bounded only by other planes →
      // corners from 3-plane intersections, sorted by azimuth.
      const analyticalLoop = _analyticalPlaneBoundary(groupIdx, allGroups, adjacency);
      if (analyticalLoop) {
        return _buildPlaneFace(oc, params, analyticalLoop, toDelete);
      }
    }
    // Cannot build this planar face analytically — skip rather than use mesh.
    return null;
  }

  // ── Cylinder face ───────────────────────────────────────────────────────────
  if (type === 'cylinder') {
    const { axisPoint, axis } = params;
    let vRange = null;
    if (allGroups && adjacency) {
      vRange = _cylinderVExtentsFromNeighbors(groupIdx, allGroups, adjacency, axisPoint, axis);
    }
    // Fallback: scan own triangle vertices — pure geometry, no edge topology
    if (!vRange || vRange.vmax - vRange.vmin < 1e-10) {
      vRange = _cylinderVExtentsFromVertices(group, geometry, axisPoint, axis);
    }
    return _buildCylinderFace(oc, params, toDelete, vRange);
  }

  // ── Cone face ───────────────────────────────────────────────────────────────
  if (type === 'cone') {
    const { apex, axis } = params;
    let vRange = null;
    if (allGroups && adjacency) {
      vRange = _cylinderVExtentsFromNeighbors(groupIdx, allGroups, adjacency, apex, axis);
    }
    if (!vRange || vRange.vmax - vRange.vmin < 1e-10) {
      vRange = _cylinderVExtentsFromVertices(group, geometry, apex, axis);
    }
    return _buildConeFace(oc, params, toDelete, vRange);
  }

  // ── Sphere face ─────────────────────────────────────────────────────────────
  if (type === 'sphere') {
    const vRange = _sphereVExtentsFromVertices(group, geometry, params.center, params.radius);
    return _buildSphereFace(oc, params, toDelete, vRange);
  }

  return null; // NURBS and unknown types: no analytical face
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
    if (!mf.IsDone()) return null;
    return mf.Face();
  } catch {
    return null;
  }
}

/**
 * Build a circular (disk) planar face bounded by a single full-circle edge.
 * Used for cylinder/cone end caps where the boundary between the flat face
 * and the curved surface is an analytic circle, not a polygon.
 *
 * @param {object}   oc
 * @param {object}   planeParams  { origin, normal }
 * @param {number[]} circCenter   [x,y,z] circle centre (already on the plane)
 * @param {number}   radius
 * @param {object[]} toDelete
 * @returns {object|null}  TopoDS_Face or null
 */
function _buildCircularPlaneFace(oc, planeParams, circCenter, radius, toDelete) {
  try {
    const { origin, normal } = planeParams;
    // gp_Ax2_3(P, N): Z axis of the frame = circle normal = plane normal.
    // The circle will be wound right-hand relative to N.
    const ax2 = new oc.gp_Ax2_3(makePnt(oc, circCenter), makeDir(oc, normal));
    toDelete.push(ax2);
    const circ = new oc.gp_Circ_2(ax2, radius);
    toDelete.push(circ);

    // BRepBuilderAPI_MakeEdge_8(gp_Circ) → full circle edge
    const edgeMaker = new oc.BRepBuilderAPI_MakeEdge_8(circ);
    toDelete.push(edgeMaker);
    if (!edgeMaker.IsDone()) {
      console.warn('[brepBuilder] circular cap: MakeEdge_8 !IsDone()');
      return null;
    }
    const edge = edgeMaker.Edge();

    // BRepBuilderAPI_MakeWire_2(TopoDS_Edge) → single-edge wire
    const wireMaker = new oc.BRepBuilderAPI_MakeWire_2(edge);
    toDelete.push(wireMaker);
    if (!wireMaker.IsDone()) {
      console.warn('[brepBuilder] circular cap: MakeWire_2 !IsDone()');
      return null;
    }
    const wire = wireMaker.Wire();

    const pln = new oc.gp_Pln_3(makePnt(oc, origin), makeDir(oc, normal));
    toDelete.push(pln);
    const mf = new oc.BRepBuilderAPI_MakeFace_16(pln, wire, true);
    toDelete.push(mf);
    if (!mf.IsDone()) {
      console.warn('[brepBuilder] circular cap: MakeFace_16 !IsDone()');
      return null;
    }
    return mf.Face();
  } catch (e) {
    console.warn('[brepBuilder] circular cap: exception in _buildCircularPlaneFace', e);
    return null;
  }
}

/**
 * Build a planar cap face whose boundary is the exact analytical intersection
 * circle (or ellipse) between the plane and the adjacent curved surface,
 * computed by IntAna_QuadQuadGeo — OCCT's purpose-built quadric intersection
 * package.  All three types are bound in opencascade.js@1.1.4:
 *
 *   cylinder → IntAna_QuadQuadGeo_4(gp_Pln, gp_Cylinder, tolAng, tol)
 *   cone     → IntAna_QuadQuadGeo_5(gp_Pln, gp_Cone,     tolAng, tol)
 *   sphere   → IntAna_QuadQuadGeo_3(gp_Pln, gp_Sphere)
 *
 * IntAna returns the intersection as a gp_Circ / gp_Elips which becomes a
 * single analytic edge → wire → face.
 *
 * OCCT 7.4 IntAna_ResultType integer values (sequential from 0):
 *   IntAna_Same=0, IntAna_Point=1, IntAna_Line=2, IntAna_Circle=3,
 *   IntAna_PointAndCircle=4, IntAna_TwoCircles=5, IntAna_Ellipse=6, ...
 *
 * @param {object}   oc
 * @param {object}   planeParams  { origin, normal }
 * @param {object}   adjSurface   { type, params }  — the adjacent curved surface
 * @param {object[]} toDelete
 * @returns {object|null}  TopoDS_Face or null
 */
function _buildAnalyticalCapFace(oc, planeParams, adjSurface, toDelete) {
  const { origin, normal } = planeParams;
  const n = _u3(normal);

  try {
    const pln = new oc.gp_Pln_3(makePnt(oc, origin), makeDir(oc, n));
    toDelete.push(pln);

    let inter = null;

    if (adjSurface.type === 'cylinder') {
      const { axisPoint, axis, radius } = adjSurface.params;
      const ax3 = makeAx3(oc, axisPoint, axis);
      toDelete.push(ax3);
      const cyl = new oc.gp_Cylinder_2(ax3, radius);
      toDelete.push(cyl);
      inter = new oc.IntAna_QuadQuadGeo_4(pln, cyl, 1e-7, 1e-7);
      toDelete.push(inter);

    } else if (adjSurface.type === 'cone') {
      const { apex, axis, halfAngle } = adjSurface.params;
      const ax3 = makeAx3(oc, apex, axis);
      toDelete.push(ax3);
      const cone = new oc.gp_Cone_2(ax3, halfAngle, 0.0);
      toDelete.push(cone);
      inter = new oc.IntAna_QuadQuadGeo_5(pln, cone, 1e-7, 1e-7);
      toDelete.push(inter);

    } else if (adjSurface.type === 'sphere') {
      const { center, radius } = adjSurface.params;
      const ax3 = makeAx3(oc, center, [0, 0, 1]);
      toDelete.push(ax3);
      const sph = new oc.gp_Sphere_2(ax3, radius);
      toDelete.push(sph);
      inter = new oc.IntAna_QuadQuadGeo_3(pln, sph);
      toDelete.push(inter);
    }

    if (!inter || !inter.IsDone() || inter.NbSolutions() < 1) return null;

    // IntAna_ResultType sequential enum values in OCCT 7.4:
    //   IntAna_Circle = 3, IntAna_Ellipse = 6
    // Use oc.IntAna_ResultType.IntAna_Circle when the namespace is bound;
    // fall back to the correct integer literals otherwise.
    const T_CIRCLE  = oc.IntAna_ResultType?.IntAna_Circle  ?? 3;
    const T_ELLIPSE = oc.IntAna_ResultType?.IntAna_Ellipse ?? 6;
    const typeInter = inter.TypeInter();

    let edgeMaker;
    if (typeInter === T_CIRCLE) {
      const circ = inter.Circle(1);
      toDelete.push(circ);
      edgeMaker = new oc.BRepBuilderAPI_MakeEdge_8(circ);
    } else if (typeInter === T_ELLIPSE) {
      const elips = inter.Ellipse(1);
      toDelete.push(elips);
      edgeMaker = new oc.BRepBuilderAPI_MakeEdge_12(elips);
    } else {
      console.warn('[brepBuilder] IntAna cap: unexpected intersection type', typeInter,
                   '(expected', T_CIRCLE, 'or', T_ELLIPSE, ')');
      return null;
    }

    toDelete.push(edgeMaker);
    if (!edgeMaker.IsDone()) {
      console.warn('[brepBuilder] IntAna cap: MakeEdge !IsDone(), typeInter=', typeInter);
      return null;
    }

    const wireMaker = new oc.BRepBuilderAPI_MakeWire_2(edgeMaker.Edge());
    toDelete.push(wireMaker);
    if (!wireMaker.IsDone()) {
      console.warn('[brepBuilder] IntAna cap: MakeWire_2 !IsDone()');
      return null;
    }

    const mf = new oc.BRepBuilderAPI_MakeFace_16(pln, wireMaker.Wire(), true);
    toDelete.push(mf);
    if (!mf.IsDone()) {
      console.warn('[brepBuilder] IntAna cap: MakeFace_16 !IsDone()');
      return null;
    }
    return mf.Face();

  } catch (e) {
    console.warn('[brepBuilder] IntAna cap: exception in _buildAnalyticalCapFace', e);
    return null;
  }
}

function _buildCylinderFace(oc, params, toDelete, neighborVRange) {
  // Build an analytical cylindrical face using UV parameter bounds.
  // BRepBuilderAPI_MakeFace_10(gp_Cylinder, UMin, UMax, VMin, VMax) creates a
  // proper Geom_CylindricalSurface face without requiring PCurves, avoiding the
  // null-PCurve crash that MakeFace_17 (cylinder + wire) triggers inside
  // STEPControl_Writer when the wire edges are straight 3-D line segments.
  const { axisPoint, axis, radius } = params;
  try {
    if (!neighborVRange || neighborVRange.vmax - neighborVRange.vmin < 1e-10) {
      console.warn('[brepBuilder] cylinder: no usable V range', neighborVRange);
      return null;
    }
    const { vmin, vmax } = neighborVRange;

    const ax3 = makeAx3(oc, axisPoint, axis);
    toDelete.push(ax3);
    const cyl = new oc.gp_Cylinder_2(ax3, radius);
    toDelete.push(cyl);

    // U: full circle (0 … 2π) — correct for a complete cylindrical hole or boss.
    // A small padding on V avoids degenerate edge artefacts at exact boundaries.
    const pad = (vmax - vmin) * 1e-6;
    const mf = new oc.BRepBuilderAPI_MakeFace_10(
      cyl, 0.0, 2 * Math.PI, vmin - pad, vmax + pad,
    );
    toDelete.push(mf);
    if (!mf.IsDone()) {
      console.warn('[brepBuilder] cylinder: MakeFace_11 !IsDone()', { axisPoint, axis, radius, vmin, vmax });
      return null;
    }
    return mf.Face();
  } catch (e) {
    console.warn('[brepBuilder] cylinder: exception in _buildCylinderFace', e);
    return null;
  }
}

function _buildConeFace(oc, params, toDelete, neighborVRange) {
  // Build an analytical conical face using UV parameter bounds.
  // BRepBuilderAPI_MakeFace_11(gp_Cone, UMin, UMax, VMin, VMax) avoids the
  // null-PCurve crash that occurs when using a wire of straight 3-D edges.
  const { apex, axis, halfAngle } = params;
  try {
    // V is the signed axial distance from the apex along the cone axis.
    if (!neighborVRange || neighborVRange.vmax - neighborVRange.vmin < 1e-10 ||
        neighborVRange.vmin < -1e-6) {
      console.warn('[brepBuilder] cone: no usable V range or apex issue', neighborVRange);
      return null;
    }
    const { vmin, vmax } = neighborVRange;

    const ax3 = makeAx3(oc, apex, axis);
    toDelete.push(ax3);
    // gp_Cone_2(Ax3, HalfAngle, RadiusAtOrigin): origin is at apex, radius=0 there.
    const cone = new oc.gp_Cone_2(ax3, halfAngle, 0.0);
    toDelete.push(cone);

    const pad = (vmax - vmin) * 1e-6;
    const mf = new oc.BRepBuilderAPI_MakeFace_11(
      cone, 0.0, 2 * Math.PI, vmin - pad, vmax + pad,
    );
    toDelete.push(mf);
    if (!mf.IsDone()) {
      console.warn('[brepBuilder] cone: MakeFace_12 !IsDone()', { apex, axis, halfAngle, vmin, vmax });
      return null;
    }
    return mf.Face();
  } catch (e) {
    console.warn('[brepBuilder] cone: exception in _buildConeFace', e);
    return null;
  }
}

function _buildSphereFace(oc, params, toDelete, vRange) {
  // Build an analytical spherical face using UV parameter bounds.
  // BRepBuilderAPI_MakeFace_12(gp_Sphere, UMin, UMax, VMin, VMax) avoids the
  // null-PCurve crash from straight-edge wires.  Latitude is computed using the
  // Z-axis of a default frame centered at the sphere's centre; for a full or
  // near-full hemisphere use -π/2 … π/2.
  const { center, radius } = params;
  try {
    if (!vRange || vRange.vmax - vRange.vmin < 1e-10) {
      console.warn('[brepBuilder] sphere: no usable latitude range', vRange);
      return null;
    }
    const { vmin, vmax } = vRange;

    // Place the sphere centre at `center` with default Z-up orientation
    const ax3 = new oc.gp_Ax3_4(makePnt(oc, center), makeDir(oc, [0, 0, 1]));
    toDelete.push(ax3);
    const sph = new oc.gp_Sphere_2(ax3, radius);
    toDelete.push(sph);

    const pad = Math.max((vmax - vmin) * 1e-6, 1e-8);
    const mf = new oc.BRepBuilderAPI_MakeFace_12(
      sph, 0.0, 2 * Math.PI, vmin - pad, vmax + pad,
    );
    toDelete.push(mf);
    if (!mf.IsDone()) {
      console.warn('[brepBuilder] sphere: MakeFace_13 !IsDone()', { center, radius, vmin, vmax });
      return null;
    }
    return mf.Face();
  } catch (e) {
    console.warn('[brepBuilder] sphere: exception in _buildSphereFace', e);
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

  // Pre-compute group adjacency so planar faces can use analytical boundary
  // construction (3-plane intersections) instead of mesh edge topology.
  const adjacency = buildGroupAdjacencyMap(groups, geometry);

  const toDelete = []; // OCCT objects to free after export
  const faces = [];

  for (let i = 0; i < groups.length; i++) {
    const g = groups[i];
    if (!g.surface) continue;
    try {
      const face = buildFace(oc, g, geometry, toDelete, i, groups, adjacency);
      if (face) faces.push(face);
    } catch (err) {
      console.warn(`Face ${i} (${g.surface.type}) failed:`, err);
    }
    if (i % 50 === 0) onStatus?.(`Building faces… ${i}/${groups.length}`, 20 + 30 * i / groups.length);
  }

  if (faces.length === 0) throw new Error('No valid B-rep faces could be constructed.');

  onStatus?.(`Sewing ${faces.length} faces into a solid…`, 55);

  // ── Build a watertight solid ────────────────────────────────────────────────
  //
  // Goal: produce a MANIFOLD_SOLID_BREP STEP entity so that Onshape and other
  // CAD importers recognise the result as a solid body rather than an assembly
  // of disconnected surfaces (which is what TopoDS_Compound would produce).
  //
  // Strategy:
  //  1. Try BRepBuilderAPI_Sewing — it walks the faces and identifies shared
  //     edges, producing a fully-connected TopoDS_Shell.  Wrap that shell in a
  //     TopoDS_Solid via BRepBuilderAPI_MakeSolid_3.
  //  2. If Sewing.Perform() throws (the Emscripten binding may not support the
  //     optional Handle<Message_ProgressIndicator> arg), fall back to manually
  //     inserting all faces into a TopoDS_Shell and wrapping it in a Solid.
  //     The faces will not share edge references but OCCT will still write a
  //     MANIFOLD_SOLID_BREP, which is enough for most importers to understand
  //     the intent and attempt their own healing.

  const sewTol = options.sewTol ?? 1e-6;
  let topShape = null;

  // ── Strategy 1: BRepBuilderAPI_Sewing ──────────────────────────────────────
  try {
    const sewing = new oc.BRepBuilderAPI_Sewing_1
      ? new oc.BRepBuilderAPI_Sewing_1()
      : new oc.BRepBuilderAPI_Sewing(sewTol);
    toDelete.push(sewing);

    for (const f of faces) sewing.Add(f);

    // Perform() has an optional Handle<Message_ProgressIndicator> default arg.
    // Some Emscripten builds expose this as a no-arg call; others don't register
    // the Handle type and throw.  We catch and fall through.
    sewing.Perform();

    const sewn = sewing.SewedShape();

    // SewedShape() may return a Shell, Solid, or Compound depending on what was
    // sewn.  Determine the shape type and wrap in a Solid if needed.
    const shapeType = sewn.ShapeType?.() ?? -1;
    // TopAbs_SHELL = 4, TopAbs_SOLID = 3, TopAbs_COMPOUND = 0
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
