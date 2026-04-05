/**
 * brepVisualizer.js — Three.js wireframe overlay for fitted analytic surfaces
 *
 * Call buildBrepOverlay(groups, geometry, colorMap) after fitAllGroups() to
 * get a THREE.Group containing line-geometry indicators for every fitted
 * surface in the model.  The returned group can be added to the scene and
 * toggled visible/invisible to inspect whether the analytic B-rep shapes
 * actually match the mesh before attempting a STEP export.
 *
 * Surface indicators:
 *   plane    — bounding rectangle in the plane + outward normal arrow
 *   cylinder — two end-cap circles + four axial lines
 *   sphere   — three orthogonal great circles
 *   cone     — base circle + midpoint circle + four apex-to-base lines
 *   nurbs    — skipped (no analytic representation)
 */

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
