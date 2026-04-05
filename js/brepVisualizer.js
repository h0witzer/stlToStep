/**
 * brepVisualizer.js — Three.js overlay for fitted analytic surfaces
 *
 * Two complementary overlays:
 *
 * buildBrepOverlay(groups, geometry, colorMap)
 *   Wireframe shape indicators (circles, bounding rectangles, etc.) for every
 *   fitted surface.  Quick to build; used right after "Fit Surfaces".
 *
 * buildAnalyticalFacesMesh(groups, analyticalBounds, colorMap)
 *   Actual face-shape meshes (semi-transparent, per surface type colour) built
 *   from the pure-JS analytical boundary data returned by
 *   computeAnalyticalBoundaries() in brepBuilder.js.  Gives the user a
 *   viewport "preview" of what the STEP solid will look like before running the
 *   full OCCT export pipeline.
 *
 *   Planes   → triangulated polygon from 3-plane-intersection corners
 *   Cylinders→ tube mesh oriented along the fitted axis with V extents from neighbours
 *   Cones    → cone mesh with V-extent ring from neighbours
 *   Spheres  → three-ring wireframe (radius from fitter)
 */

import * as THREE from 'three';

const CIRCLE_SEGMENTS = 64;

// ── geometry helpers ──────────────────────────────────────────────────────────

function perpTo(normal) {
  const n = new THREE.Vector3(...normal).normalize();
  const arb = Math.abs(n.x) < 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
  return new THREE.Vector3().crossVectors(n, arb).normalize();
}

function circlePositions(center, normal, radius) {
  const n  = new THREE.Vector3(...normal).normalize();
  const xh = perpTo(normal);
  const yh = new THREE.Vector3().crossVectors(n, xh).normalize();
  const pts = [];
  for (let i = 0; i <= CIRCLE_SEGMENTS; i++) {
    const θ = (i / CIRCLE_SEGMENTS) * Math.PI * 2;
    const c = Math.cos(θ), s = Math.sin(θ);
    pts.push(
      center[0] + radius * (c * xh.x + s * yh.x),
      center[1] + radius * (c * xh.y + s * yh.y),
      center[2] + radius * (c * xh.z + s * yh.z),
    );
  }
  return pts;
}

function makeLine(pts, mat) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  return new THREE.Line(geo, mat);
}

function makeLineSegments(pts, mat) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  return new THREE.LineSegments(geo, mat);
}

// ── vertex extraction ─────────────────────────────────────────────────────────

function groupVertices(group, geometry) {
  const pos = geometry.attributes.position;
  const idx = geometry.index;
  const out = [];
  for (const tri of group.triangleIndices) {
    for (let k = 0; k < 3; k++) {
      const i = idx ? idx.getX(tri * 3 + k) : tri * 3 + k;
      out.push(pos.getX(i), pos.getY(i), pos.getZ(i));
    }
  }
  return out;
}

function axialExtent(verts, axisPoint, axis) {
  const a  = new THREE.Vector3(...axis).normalize();
  const ap = new THREE.Vector3(...axisPoint);
  let min = Infinity, max = -Infinity;
  for (let i = 0; i < verts.length; i += 3) {
    const t = new THREE.Vector3(verts[i], verts[i + 1], verts[i + 2]).sub(ap).dot(a);
    if (t < min) min = t;
    if (t > max) max = t;
  }
  return [min, max];
}

// ── wireframe indicator helpers ───────────────────────────────────────────────

function vizPlane(surface, verts, mat) {
  const { origin, normal } = surface.params;
  const n  = new THREE.Vector3(...normal).normalize();
  const xh = perpTo(normal);
  const yh = new THREE.Vector3().crossVectors(n, xh).normalize();
  const o  = new THREE.Vector3(...origin);

  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (let i = 0; i < verts.length; i += 3) {
    const p = new THREE.Vector3(verts[i], verts[i + 1], verts[i + 2]).sub(o);
    const px = p.dot(xh), py = p.dot(yh);
    if (px < minX) minX = px;
    if (px > maxX) maxX = px;
    if (py < minY) minY = py;
    if (py > maxY) maxY = py;
  }

  const corners = [[minX, minY], [maxX, minY], [maxX, maxY], [minX, maxY], [minX, minY]];
  const rectPts = [];
  for (const [cx, cy] of corners) {
    rectPts.push(
      o.x + cx * xh.x + cy * yh.x,
      o.y + cx * xh.y + cy * yh.y,
      o.z + cx * xh.z + cy * yh.z,
    );
  }

  const diag = Math.sqrt((maxX - minX) ** 2 + (maxY - minY) ** 2);
  const arrowLen = diag * 0.12;
  const mcx = (minX + maxX) / 2, mcy = (minY + maxY) / 2;
  const base = new THREE.Vector3(
    o.x + mcx * xh.x + mcy * yh.x,
    o.y + mcx * xh.y + mcy * yh.y,
    o.z + mcx * xh.z + mcy * yh.z,
  );
  const tip = base.clone().addScaledVector(n, arrowLen);

  const g = new THREE.Group();
  g.add(makeLine(rectPts, mat));
  g.add(makeLine([base.x, base.y, base.z, tip.x, tip.y, tip.z], mat));
  return g;
}

function vizCylinder(surface, verts, mat) {
  const { axis, axisPoint, radius } = surface.params;
  const a  = new THREE.Vector3(...axis).normalize();
  const ap = new THREE.Vector3(...axisPoint);
  const [zMin, zMax] = axialExtent(verts, axisPoint, axis);

  const c1 = [ap.x + zMin * a.x, ap.y + zMin * a.y, ap.z + zMin * a.z];
  const c2 = [ap.x + zMax * a.x, ap.y + zMax * a.y, ap.z + zMax * a.z];
  const xh = perpTo(axis);
  const yh = new THREE.Vector3().crossVectors(a, xh).normalize();

  const g = new THREE.Group();
  g.add(makeLine(circlePositions(c1, axis, radius), mat));
  g.add(makeLine(circlePositions(c2, axis, radius), mat));

  for (let i = 0; i < 4; i++) {
    const θ = (i / 4) * Math.PI * 2;
    const dx = Math.cos(θ) * xh.x + Math.sin(θ) * yh.x;
    const dy = Math.cos(θ) * xh.y + Math.sin(θ) * yh.y;
    const dz = Math.cos(θ) * xh.z + Math.sin(θ) * yh.z;
    g.add(makeLine([
      c1[0] + radius * dx, c1[1] + radius * dy, c1[2] + radius * dz,
      c2[0] + radius * dx, c2[1] + radius * dy, c2[2] + radius * dz,
    ], mat));
  }
  return g;
}

function vizSphere(surface, mat) {
  const { center, radius } = surface.params;
  const g = new THREE.Group();
  g.add(makeLine(circlePositions(center, [1, 0, 0], radius), mat));
  g.add(makeLine(circlePositions(center, [0, 1, 0], radius), mat));
  g.add(makeLine(circlePositions(center, [0, 0, 1], radius), mat));
  return g;
}

function vizCone(surface, verts, mat) {
  const { axis, apex, halfAngle } = surface.params;
  const a     = new THREE.Vector3(...axis).normalize();
  const apexV = new THREE.Vector3(...apex);
  const tanA  = Math.tan(halfAngle);

  let tMin = Infinity, tMax = -Infinity;
  for (let i = 0; i < verts.length; i += 3) {
    const t = new THREE.Vector3(verts[i], verts[i + 1], verts[i + 2]).sub(apexV).dot(a);
    if (t < tMin) tMin = t;
    if (t > tMax) tMax = t;
  }
  if (!isFinite(tMin) || !isFinite(tMax)) return new THREE.Group();

  const xh = perpTo(axis);
  const yh = new THREE.Vector3().crossVectors(a, xh).normalize();

  const c1 = [apexV.x + tMin * a.x, apexV.y + tMin * a.y, apexV.z + tMin * a.z];
  const r1 = Math.abs(tMin) * tanA;
  const c2 = [apexV.x + tMax * a.x, apexV.y + tMax * a.y, apexV.z + tMax * a.z];
  const r2 = Math.abs(tMax) * tanA;

  const g = new THREE.Group();
  if (r1 > 1e-9) g.add(makeLine(circlePositions(c1, axis, r1), mat));
  g.add(makeLine(circlePositions(c2, axis, r2), mat));

  for (let i = 0; i < 4; i++) {
    const θ = (i / 4) * Math.PI * 2;
    const dx = Math.cos(θ) * xh.x + Math.sin(θ) * yh.x;
    const dy = Math.cos(θ) * xh.y + Math.sin(θ) * yh.y;
    const dz = Math.cos(θ) * xh.z + Math.sin(θ) * yh.z;
    g.add(makeLine([
      c1[0] + r1 * dx, c1[1] + r1 * dy, c1[2] + r1 * dz,
      c2[0] + r2 * dx, c2[1] + r2 * dy, c2[2] + r2 * dz,
    ], mat));
  }

  if (tMin <= 0 && tMax >= 0) {
    const sz = Math.max(r1, r2) * 0.06;
    g.add(makeLineSegments([
      apex[0] - sz, apex[1], apex[2], apex[0] + sz, apex[1], apex[2],
      apex[0], apex[1] - sz, apex[2], apex[0], apex[1] + sz, apex[2],
      apex[0], apex[1], apex[2] - sz, apex[0], apex[1], apex[2] + sz,
    ], mat));
  }

  return g;
}

// ── analytical face mesh builders ─────────────────────────────────────────────

/**
 * Build a semi-transparent Three.js mesh for a single PLANE face given its
 * analytically-derived corner polygon.
 *
 * @param {Array<number[]>} corners  ordered [x,y,z] points
 * @param {THREE.Material}  mat
 */
function meshPlane(corners, mat) {
  if (!corners || corners.length < 3) return null;

  // Fan triangulation from the first corner
  const positions = [];
  const n = corners.length;
  for (let i = 1; i < n - 1; i++) {
    positions.push(...corners[0], ...corners[i], ...corners[i + 1]);
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geo.computeVertexNormals();
  return new THREE.Mesh(geo, mat);
}

/**
 * Build a semi-transparent Three.js mesh for a CYLINDER face.
 *
 * @param {object} params    { axisPoint, axis, radius }
 * @param {number} vmin      axial start (from axisPoint along axis)
 * @param {number} vmax      axial end
 * @param {THREE.Material} mat
 */
function meshCylinder(params, vmin, vmax, mat) {
  const { axisPoint, axis, radius } = params;
  const height = vmax - vmin;
  if (height < 1e-10 || radius < 1e-10) return null;

  const SEG = 32;
  const ax = new THREE.Vector3(...axis).normalize();
  const xh = perpTo(axis);
  const yh = new THREE.Vector3().crossVectors(ax, xh).normalize();
  const ap = new THREE.Vector3(...axisPoint);

  // Build vertices in two rings at vmin and vmax
  const positions = [];
  const normals   = [];
  const indices   = [];

  for (let i = 0; i <= SEG; i++) {
    const θ = (i / SEG) * Math.PI * 2;
    const c = Math.cos(θ), s = Math.sin(θ);
    const nx = c * xh.x + s * yh.x;
    const ny = c * xh.y + s * yh.y;
    const nz = c * xh.z + s * yh.z;
    const bx = ap.x + radius * nx;
    const by = ap.y + radius * ny;
    const bz = ap.z + radius * nz;

    // bottom ring (v = vmin)
    positions.push(bx + vmin * ax.x, by + vmin * ax.y, bz + vmin * ax.z);
    normals.push(nx, ny, nz);
    // top ring (v = vmax)
    positions.push(bx + vmax * ax.x, by + vmax * ax.y, bz + vmax * ax.z);
    normals.push(nx, ny, nz);
  }

  for (let i = 0; i < SEG; i++) {
    const b0 = i * 2, b1 = (i + 1) * 2;
    indices.push(b0, b0 + 1, b1 + 1, b0, b1 + 1, b1);
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geo.setAttribute('normal',   new THREE.Float32BufferAttribute(normals,   3));
  geo.setIndex(indices);
  return new THREE.Mesh(geo, mat);
}

/**
 * Build a semi-transparent Three.js mesh for a CONE face.
 *
 * @param {object} params    { apex, axis, halfAngle }
 * @param {number} vmin      axial start from apex (should be > 0)
 * @param {number} vmax      axial end from apex
 * @param {THREE.Material} mat
 */
function meshCone(params, vmin, vmax, mat) {
  const { apex, axis, halfAngle } = params;
  const tanA = Math.tan(halfAngle);
  const height = vmax - vmin;
  if (height < 1e-10) return null;

  const SEG = 32;
  const ax = new THREE.Vector3(...axis).normalize();
  const xh = perpTo(axis);
  const yh = new THREE.Vector3().crossVectors(ax, xh).normalize();
  const apV = new THREE.Vector3(...apex);

  const positions = [];
  const normals   = [];
  const indices   = [];

  for (let i = 0; i <= SEG; i++) {
    const θ = (i / SEG) * Math.PI * 2;
    const c = Math.cos(θ), s = Math.sin(θ);
    const dirX = c * xh.x + s * yh.x;
    const dirY = c * xh.y + s * yh.y;
    const dirZ = c * xh.z + s * yh.z;

    for (const v of [vmin, vmax]) {
      const r = Math.max(0, v) * tanA;
      positions.push(
        apV.x + v * ax.x + r * dirX,
        apV.y + v * ax.y + r * dirY,
        apV.z + v * ax.z + r * dirZ,
      );
      // Outward normal perpendicular to the generator
      const cosA = Math.cos(halfAngle), sinA = Math.sin(halfAngle);
      normals.push(
        cosA * dirX - sinA * ax.x,
        cosA * dirY - sinA * ax.y,
        cosA * dirZ - sinA * ax.z,
      );
    }
  }

  for (let i = 0; i < SEG; i++) {
    const b0 = i * 2, b1 = (i + 1) * 2;
    indices.push(b0, b0 + 1, b1 + 1, b0, b1 + 1, b1);
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geo.setAttribute('normal',   new THREE.Float32BufferAttribute(normals,   3));
  geo.setIndex(indices);
  return new THREE.Mesh(geo, mat);
}

/**
 * Build a semi-transparent Three.js mesh for a SPHERE face.
 *
 * Uses Three.js SphereGeometry positioned at the fitted centre.
 */
function meshSphere(params, mat) {
  const { center, radius } = params;
  if (radius < 1e-10) return null;
  const geo = new THREE.SphereGeometry(radius, 32, 16);
  const mesh = new THREE.Mesh(geo, mat);
  mesh.position.set(...center);
  return mesh;
}

// ── public API ────────────────────────────────────────────────────────────────

/**
 * Build a THREE.Group containing analytic wireframe indicators for every
 * fitted surface group.  The wireframe is always built from the raw mesh
 * vertex extents and serves as a lightweight shape indicator that is
 * immediately available after "Fit Surfaces".
 *
 * @param {object[]} groups
 * @param {THREE.BufferGeometry} geometry
 * @param {Record<string,string>} colorMap
 * @returns {THREE.Group}
 */
export function buildBrepOverlay(groups, geometry, colorMap) {
  const overlay = new THREE.Group();
  overlay.name = 'brep-overlay';

  for (const g of groups) {
    if (!g.surface || g.surface.type === 'nurbs') continue;

    const hex = parseInt((colorMap[g.surface.type] ?? '#aaaaaa').replace('#', ''), 16);
    const mat = new THREE.LineBasicMaterial({
      color: hex,
      depthTest: false,
      depthWrite: false,
    });

    let verts;
    try { verts = groupVertices(g, geometry); } catch { continue; }

    let vizGroup;
    try {
      switch (g.surface.type) {
        case 'plane':    vizGroup = vizPlane(g.surface, verts, mat);    break;
        case 'cylinder': vizGroup = vizCylinder(g.surface, verts, mat); break;
        case 'sphere':   vizGroup = vizSphere(g.surface, mat);          break;
        case 'cone':     vizGroup = vizCone(g.surface, verts, mat);     break;
        default: continue;
      }
    } catch { continue; }

    if (vizGroup) overlay.add(vizGroup);
  }

  return overlay;
}

/**
 * Build a THREE.Group containing semi-transparent face meshes that preview
 * the analytical solid as it will appear in the STEP export — without loading
 * OpenCASCADE.  Requires the output of computeAnalyticalBoundaries() from
 * brepBuilder.js to provide exact surface extents.
 *
 * Plane   → fan-triangulated polygon from 3-plane-intersection corners
 * Cylinder→ tube mesh oriented along the fitted axis with V extents from neighbours
 * Cone    → ring frustum mesh with V extents from neighbours
 * Sphere  → Three.js SphereGeometry at the fitted centre/radius
 *
 * @param {object[]} groups
 * @param {THREE.BufferGeometry} geometry  (used as fallback for vertex extents)
 * @param {Map<number, object>}  analyticalBounds  from computeAnalyticalBoundaries()
 * @param {Record<string,string>} colorMap
 * @returns {THREE.Group}
 */
export function buildAnalyticalFacesMesh(groups, geometry, analyticalBounds, colorMap) {
  const overlay = new THREE.Group();
  overlay.name = 'brep-faces-overlay';

  for (let gi = 0; gi < groups.length; gi++) {
    const g = groups[gi];
    if (!g.surface || g.surface.type === 'nurbs') continue;

    const { type, params } = g.surface;
    const bound = analyticalBounds?.get(gi);

    const hex = parseInt((colorMap[type] ?? '#aaaaaa').replace('#', ''), 16);
    const meshMat = new THREE.MeshPhongMaterial({
      color: hex,
      opacity: 0.45,
      transparent: true,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    const edgeMat = new THREE.LineBasicMaterial({
      color: hex,
      depthTest: false,
      depthWrite: false,
    });

    let faceMesh = null;
    let edgeGroup = null;

    try {
      if (type === 'plane') {
        const loop = bound?.loop;
        if (loop && loop.length >= 3) {
          faceMesh = meshPlane(loop, meshMat);
          // Outline edge
          const pts = [...loop.flatMap(c => c), ...loop[0]];
          edgeGroup = makeLine(pts, edgeMat);
        }
      } else if (type === 'cylinder') {
        let vmin = bound?.vmin, vmax = bound?.vmax;
        // Fall back to mesh vertex projection if no analytical extents
        if (vmin == null || vmax == null) {
          const verts = groupVertices(g, geometry);
          [vmin, vmax] = axialExtent(verts, params.axisPoint, params.axis);
        }
        faceMesh  = meshCylinder(params, vmin, vmax, meshMat);
        // End-cap circles as edge indicators
        const ax = new THREE.Vector3(...params.axis).normalize();
        const ap = new THREE.Vector3(...params.axisPoint);
        const c1 = [ap.x + vmin * ax.x, ap.y + vmin * ax.y, ap.z + vmin * ax.z];
        const c2 = [ap.x + vmax * ax.x, ap.y + vmax * ax.y, ap.z + vmax * ax.z];
        edgeGroup = new THREE.Group();
        edgeGroup.add(makeLine(circlePositions(c1, params.axis, params.radius), edgeMat));
        edgeGroup.add(makeLine(circlePositions(c2, params.axis, params.radius), edgeMat));
      } else if (type === 'cone') {
        let vmin = bound?.vmin, vmax = bound?.vmax;
        if (vmin == null || vmax == null) {
          const verts = groupVertices(g, geometry);
          [vmin, vmax] = axialExtent(verts, params.apex, params.axis);
        }
        faceMesh = meshCone(params, vmin, vmax, meshMat);
      } else if (type === 'sphere') {
        faceMesh = meshSphere(params, meshMat);
      }
    } catch { /* skip on any error */ }

    if (faceMesh) overlay.add(faceMesh);
    if (edgeGroup) overlay.add(edgeGroup);
  }

  return overlay;
}


import * as THREE from 'three';

const CIRCLE_SEGMENTS = 64;

// ── geometry helpers ──────────────────────────────────────────────────────────

/**
 * Return a unit vector perpendicular to `normal`.
 * @param {number[]} normal  [nx,ny,nz]
 * @returns {THREE.Vector3}
 */
function perpTo(normal) {
  const n = new THREE.Vector3(...normal).normalize();
  const arb = Math.abs(n.x) < 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
  return new THREE.Vector3().crossVectors(n, arb).normalize();
}

/**
 * Build a flat (x,y,z, x,y,z, …) position array tracing a circle.
 * The first point is repeated at the end so it closes as a LineLoop.
 *
 * @param {number[]} center  [cx,cy,cz]
 * @param {number[]} normal  axis direction (not required to be unit)
 * @param {number}   radius
 * @returns {number[]}
 */
function circlePositions(center, normal, radius) {
  const n  = new THREE.Vector3(...normal).normalize();
  const xh = perpTo(normal);
  const yh = new THREE.Vector3().crossVectors(n, xh).normalize();
  const pts = [];
  for (let i = 0; i <= CIRCLE_SEGMENTS; i++) {
    const θ = (i / CIRCLE_SEGMENTS) * Math.PI * 2;
    const c = Math.cos(θ), s = Math.sin(θ);
    pts.push(
      center[0] + radius * (c * xh.x + s * yh.x),
      center[1] + radius * (c * xh.y + s * yh.y),
      center[2] + radius * (c * xh.z + s * yh.z),
    );
  }
  return pts;
}

function makeLine(pts, mat) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  return new THREE.Line(geo, mat);
}

function makeLineSegments(pts, mat) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
  return new THREE.LineSegments(geo, mat);
}

// ── vertex extraction ─────────────────────────────────────────────────────────

function groupVertices(group, geometry) {
  const pos = geometry.attributes.position;
  const idx = geometry.index;
  const out = [];
  for (const tri of group.triangleIndices) {
    for (let k = 0; k < 3; k++) {
      const i = idx ? idx.getX(tri * 3 + k) : tri * 3 + k;
      out.push(pos.getX(i), pos.getY(i), pos.getZ(i));
    }
  }
  return out;
}

/** Min/max projection of flat vertex array onto an axis through axisPoint. */
function axialExtent(verts, axisPoint, axis) {
  const a  = new THREE.Vector3(...axis).normalize();
  const ap = new THREE.Vector3(...axisPoint);
  let min = Infinity, max = -Infinity;
  for (let i = 0; i < verts.length; i += 3) {
    const t = new THREE.Vector3(verts[i], verts[i + 1], verts[i + 2]).sub(ap).dot(a);
    if (t < min) min = t;
    if (t > max) max = t;
  }
  return [min, max];
}

// ── per-type visualisers ──────────────────────────────────────────────────────

function vizPlane(surface, verts, mat) {
  const { origin, normal } = surface.params;
  const n  = new THREE.Vector3(...normal).normalize();
  const xh = perpTo(normal);
  const yh = new THREE.Vector3().crossVectors(n, xh).normalize();
  const o  = new THREE.Vector3(...origin);

  // Project every vertex onto the plane's local 2-D frame
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (let i = 0; i < verts.length; i += 3) {
    const p = new THREE.Vector3(verts[i], verts[i + 1], verts[i + 2]).sub(o);
    const px = p.dot(xh), py = p.dot(yh);
    if (px < minX) minX = px;
    if (px > maxX) maxX = px;
    if (py < minY) minY = py;
    if (py > maxY) maxY = py;
  }

  // Rectangle (closed)
  const corners = [[minX, minY], [maxX, minY], [maxX, maxY], [minX, maxY], [minX, minY]];
  const rectPts = [];
  for (const [cx, cy] of corners) {
    rectPts.push(
      o.x + cx * xh.x + cy * yh.x,
      o.y + cx * xh.y + cy * yh.y,
      o.z + cx * xh.z + cy * yh.z,
    );
  }

  // Normal arrow from centroid (12 % of bounding-box diagonal length)
  const diag = Math.sqrt((maxX - minX) ** 2 + (maxY - minY) ** 2);
  const arrowLen = diag * 0.12;
  const mcx = (minX + maxX) / 2, mcy = (minY + maxY) / 2;
  const base = new THREE.Vector3(
    o.x + mcx * xh.x + mcy * yh.x,
    o.y + mcx * xh.y + mcy * yh.y,
    o.z + mcx * xh.z + mcy * yh.z,
  );
  const tip = base.clone().addScaledVector(n, arrowLen);
  const arrowPts = [base.x, base.y, base.z, tip.x, tip.y, tip.z];

  const g = new THREE.Group();
  g.add(makeLine(rectPts, mat));
  g.add(makeLine(arrowPts, mat));
  return g;
}

function vizCylinder(surface, verts, mat) {
  const { axis, axisPoint, radius } = surface.params;
  const a  = new THREE.Vector3(...axis).normalize();
  const ap = new THREE.Vector3(...axisPoint);
  const [zMin, zMax] = axialExtent(verts, axisPoint, axis);

  const c1 = [ap.x + zMin * a.x, ap.y + zMin * a.y, ap.z + zMin * a.z];
  const c2 = [ap.x + zMax * a.x, ap.y + zMax * a.y, ap.z + zMax * a.z];

  const xh = perpTo(axis);
  const yh = new THREE.Vector3().crossVectors(a, xh).normalize();

  const g = new THREE.Group();
  g.add(makeLine(circlePositions(c1, axis, radius), mat));
  g.add(makeLine(circlePositions(c2, axis, radius), mat));

  // 4 axial lines connecting the two end circles
  for (let i = 0; i < 4; i++) {
    const θ = (i / 4) * Math.PI * 2;
    const dx = Math.cos(θ) * xh.x + Math.sin(θ) * yh.x;
    const dy = Math.cos(θ) * xh.y + Math.sin(θ) * yh.y;
    const dz = Math.cos(θ) * xh.z + Math.sin(θ) * yh.z;
    g.add(makeLine([
      c1[0] + radius * dx, c1[1] + radius * dy, c1[2] + radius * dz,
      c2[0] + radius * dx, c2[1] + radius * dy, c2[2] + radius * dz,
    ], mat));
  }
  return g;
}

function vizSphere(surface, mat) {
  const { center, radius } = surface.params;
  const g = new THREE.Group();
  g.add(makeLine(circlePositions(center, [1, 0, 0], radius), mat));
  g.add(makeLine(circlePositions(center, [0, 1, 0], radius), mat));
  g.add(makeLine(circlePositions(center, [0, 0, 1], radius), mat));
  return g;
}

function vizCone(surface, verts, mat) {
  const { axis, apex, halfAngle } = surface.params;
  const a     = new THREE.Vector3(...axis).normalize();
  const apexV = new THREE.Vector3(...apex);
  const tanA  = Math.tan(halfAngle);

  // Find the axial extent of the mesh section measured FROM the apex.
  // We need both the minimum and maximum signed projections so we can draw
  // circles at BOTH ends of the cone section — not just the farthest end.
  let tMin = Infinity, tMax = -Infinity;
  for (let i = 0; i < verts.length; i += 3) {
    const t = new THREE.Vector3(verts[i], verts[i + 1], verts[i + 2]).sub(apexV).dot(a);
    if (t < tMin) tMin = t;
    if (t > tMax) tMax = t;
  }
  if (!isFinite(tMin) || !isFinite(tMax)) return new THREE.Group();

  const xh = perpTo(axis);
  const yh = new THREE.Vector3().crossVectors(a, xh).normalize();

  // Two end circles at tMin and tMax (with correct radii at each axial slice)
  const c1 = [apexV.x + tMin * a.x, apexV.y + tMin * a.y, apexV.z + tMin * a.z];
  const r1 = Math.abs(tMin) * tanA;
  const c2 = [apexV.x + tMax * a.x, apexV.y + tMax * a.y, apexV.z + tMax * a.z];
  const r2 = Math.abs(tMax) * tanA;

  const g = new THREE.Group();
  if (r1 > 1e-9) g.add(makeLine(circlePositions(c1, axis, r1), mat));
  g.add(makeLine(circlePositions(c2, axis, r2), mat));

  // 4 generator lines connecting the two end circles (avoids lines to a
  // potentially distant apex outside the mesh bounds)
  for (let i = 0; i < 4; i++) {
    const θ = (i / 4) * Math.PI * 2;
    const dx = Math.cos(θ) * xh.x + Math.sin(θ) * yh.x;
    const dy = Math.cos(θ) * xh.y + Math.sin(θ) * yh.y;
    const dz = Math.cos(θ) * xh.z + Math.sin(θ) * yh.z;
    g.add(makeLine([
      c1[0] + r1 * dx, c1[1] + r1 * dy, c1[2] + r1 * dz,
      c2[0] + r2 * dx, c2[1] + r2 * dy, c2[2] + r2 * dz,
    ], mat));
  }

  // Apex marker — only when apex lies within the mesh's axial span (tMin ≤ 0 ≤ tMax)
  if (tMin <= 0 && tMax >= 0) {
    const sz = Math.max(r1, r2) * 0.06;
    g.add(makeLineSegments([
      apex[0] - sz, apex[1], apex[2], apex[0] + sz, apex[1], apex[2],
      apex[0], apex[1] - sz, apex[2], apex[0], apex[1] + sz, apex[2],
      apex[0], apex[1], apex[2] - sz, apex[0], apex[1], apex[2] + sz,
    ], mat));
  }

  return g;
}

// ── public API ────────────────────────────────────────────────────────────────

/**
 * Build a THREE.Group containing analytic wireframe indicators for every
 * fitted surface group.
 *
 * @param {Array<{triangleIndices: Set<number>, surface?: object}>} groups
 * @param {THREE.BufferGeometry} geometry  source mesh
 * @param {Record<string,string>} colorMap  e.g. { plane:'#4a9eff', … }
 * @returns {THREE.Group}
 */
export function buildBrepOverlay(groups, geometry, colorMap) {
  const overlay = new THREE.Group();
  overlay.name = 'brep-overlay';

  for (const g of groups) {
    if (!g.surface || g.surface.type === 'nurbs') continue;

    const hex = parseInt((colorMap[g.surface.type] ?? '#aaaaaa').replace('#', ''), 16);
    const mat = new THREE.LineBasicMaterial({
      color: hex,
      depthTest: false,   // always draw on top so hidden edges are visible
      depthWrite: false,
    });

    let verts;
    try {
      verts = groupVertices(g, geometry);
    } catch { continue; }

    let vizGroup;
    try {
      switch (g.surface.type) {
        case 'plane':    vizGroup = vizPlane(g.surface, verts, mat);    break;
        case 'cylinder': vizGroup = vizCylinder(g.surface, verts, mat); break;
        case 'sphere':   vizGroup = vizSphere(g.surface, mat);          break;
        case 'cone':     vizGroup = vizCone(g.surface, verts, mat);     break;
        default: continue;
      }
    } catch { continue; }

    if (vizGroup) overlay.add(vizGroup);
  }

  return overlay;
}
