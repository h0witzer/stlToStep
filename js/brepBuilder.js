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

// ── Vector math utilities ────────────────────────────────────────────────────

function _d3(a, b)  { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
function _x3(a, b)  {
  return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
}
function _n3(v)     { return Math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
function _u3(v)     { const n = _n3(v); return n > 1e-14 ? [v[0]/n, v[1]/n, v[2]/n] : v; }

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
//
// Architecture: two-phase build.
//
//  Phase 1 — _buildLargePatch(oc, group, geometry, toDelete)
//    Every surface type is treated identically: project mesh vertices into
//    the surface's natural UV space, compute a bounding extent with margin,
//    and call BRepBuilderAPI_MakeFace_X(surface, umin, umax, vmin, vmax).
//    No intersection computation, no adjacency lookup, no type enumeration.
//    For planes a rectangular wire face is used (same idea, different API).
//
//  Phase 2 — _buildSectionBoundedPlaneFace(oc, …, patches)
//    For each planar face, replace its coarse rectangular patch with a face
//    whose boundary is derived from the EXACT intersection with every adjacent
//    surface patch.  A single OCCT kernel call — BRepAlgoAPI_Section — handles
//    every surface-type pair (plane/cylinder, plane/cone, plane/sphere,
//    plane/plane, …) without any JS-level special-casing.  The returned edges
//    already carry PCurves on both surfaces; collecting them into a wire and
//    calling MakeFace_16(gp_Pln, wire, true) gives the precisely trimmed face.
//
//  Curved surfaces (cylinder, cone, sphere) use the large-patch faces directly.
//  BRepBuilderAPI_Sewing in buildAndExportSTEP then matches the UV-boundary
//  circles of the curved patches to the analytically exact edges of the planar
//  faces, stitching them into a manifold shell without any JS-written geometry.

/**
 * Build a large analytical patch face for any surface type.
 * UV extents come from mesh vertices plus a generous margin — no intersection
 * computation, no adjacent-surface enumeration.
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

/**
 * Use BRepAlgoAPI_Section to find the intersection edges between two faces.
 * The returned edges already carry PCurves on both surfaces — they can be fed
 * directly into BRepBuilderAPI_MakeWire and used as face boundaries.
 *
 * BRepAlgoAPI_Section(S1, S2, PerformNow=true) takes two TopoDS_Shape values
 * (no Handle<T>) and computes the section at construction time, avoiding the
 * need to call Build() separately.
 *
 * @param {object}   oc
 * @param {object}   shape1  TopoDS_Shape (first patch face)
 * @param {object}   shape2  TopoDS_Shape (second patch face)
 * @param {object[]} toDelete
 * @returns {object[]}  array of TopoDS_Edge (may be empty on failure)
 */
function _getSectionEdges(oc, shape1, shape2, toDelete) {
  // Try BRepAlgoAPI_Section constructor overloads until one works.
  // opencascade.js@1.1.4 numbers constructors sequentially; the non-Handle
  // (S1, S2, PerformNow) overload is typically _2 or _3.
  const ctors = ['BRepAlgoAPI_Section_3', 'BRepAlgoAPI_Section_2', 'BRepAlgoAPI_Section_4'];
  for (const ctorName of ctors) {
    if (typeof oc[ctorName] !== 'function') continue;
    try {
      const section = new oc[ctorName](shape1, shape2, true);
      toDelete.push(section);
      if (!section.IsDone()) continue;

      const result = section.Shape();
      const edges  = [];
      const EDGE_T  = oc.TopAbs_ShapeEnum?.TopAbs_EDGE  ?? 6;
      const SHAPE_T = oc.TopAbs_ShapeEnum?.TopAbs_SHAPE ?? 8;
      const exp = new oc.TopExp_Explorer_2(result, EDGE_T, SHAPE_T);
      toDelete.push(exp);
      while (exp.More()) {
        // Downcast TopoDS_Shape → TopoDS_Edge
        const shape = exp.Current();
        try {
          const edge = oc.TopoDS.Edge_1
            ? oc.TopoDS.Edge_1(shape)
            : shape; // fallback: pass shape directly
          edges.push(edge);
        } catch { edges.push(shape); }
        exp.Next();
      }
      return edges;
    } catch (e) {
      console.warn(`[brepBuilder] ${ctorName} failed:`, e?.message ?? e);
    }
  }
  return [];
}

/**
 * Build a planar face whose boundary is computed by intersecting the plane's
 * large patch with the large patches of all adjacent surfaces.
 *
 * One BRepAlgoAPI_Section call per adjacent pair — the kernel handles every
 * surface-type combination (plane/cylinder, plane/cone, plane/plane, …)
 * without any JS-level geometry enumeration.
 *
 * Falls back to the large rectangular patch when Section returns no edges.
 */
function _buildSectionBoundedPlaneFace(oc, params, planePatch, groupIdx, allGroups, adjacency, patches, toDelete) {
  const { origin, normal } = params;
  const pln = new oc.gp_Pln_3(makePnt(oc, origin), makeDir(oc, _u3(normal)));
  toDelete.push(pln);

  const wireMaker = new oc.BRepBuilderAPI_MakeWire_1();
  toDelete.push(wireMaker);
  let edgeCount = 0;

  for (const j of adjacency.get(groupIdx) ?? []) {
    const adjPatch = patches.get(j);
    if (!adjPatch) continue;
    const sectionEdges = _getSectionEdges(oc, planePatch, adjPatch, toDelete);
    for (const edge of sectionEdges) {
      try { wireMaker.Add_1(edge); edgeCount++; } catch { /* skip bad edge */ }
    }
  }

  if (edgeCount >= 3 && wireMaker.IsDone()) {
    try {
      const mf = new oc.BRepBuilderAPI_MakeFace_16(pln, wireMaker.Wire(), true);
      toDelete.push(mf);
      if (mf.IsDone()) return mf.Face();
    } catch (e) {
      console.warn('[brepBuilder] section-bounded plane: MakeFace_16 failed', e?.message ?? e);
    }
  }

  // Fall back to the large rectangular patch when Section is unavailable or
  // produces insufficient edges (e.g. BRepAlgoAPI_Section not bound).
  return planePatch;
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

  // ── Phase 1: build large analytical patches for every surface ────────────────
  // Each patch covers the mesh vertex extent with generous margin.
  // No intersection computation, no adjacency lookup, no type enumeration.
  const patches = new Map(); // groupIdx → TopoDS_Face (large patch)
  for (let i = 0; i < groups.length; i++) {
    const g = groups[i];
    if (!g.surface) continue;
    try {
      const patch = _buildLargePatch(oc, g, geometry, toDelete);
      if (patch) patches.set(i, patch);
    } catch (e) {
      console.warn(`Patch ${i} (${g.surface?.type}) failed:`, e?.message ?? e);
    }
    if (i % 50 === 0) onStatus?.(`Building patches… ${i}/${groups.length}`, 20 + 15 * i / groups.length);
  }

  // ── Phase 2: compute analytically exact faces ────────────────────────────────
  // • Planar faces: use BRepAlgoAPI_Section against every adjacent patch so that
  //   the boundary wire is the exact intersection curve — circle, ellipse, line,
  //   or any other curve the kernel returns — for any surface-type pair.
  // • Curved faces: the large UV-bounded patch already has correct analytical
  //   surface geometry; its boundary circles will be matched by sewing below.
  const faces = [];
  for (let i = 0; i < groups.length; i++) {
    const g = groups[i];
    if (!g.surface) continue;
    const patch = patches.get(i);
    if (!patch) continue;
    try {
      let face;
      if (g.surface.type === 'plane') {
        face = _buildSectionBoundedPlaneFace(
          oc, g.surface.params, patch, i, groups, adjacency, patches, toDelete,
        );
      } else {
        face = patch;
      }
      if (face) faces.push(face);
    } catch (e) {
      console.warn(`Face ${i} (${g.surface.type}) failed:`, e?.message ?? e);
    }
    if (i % 50 === 0) onStatus?.(`Building faces… ${i}/${groups.length}`, 35 + 15 * i / groups.length);
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
