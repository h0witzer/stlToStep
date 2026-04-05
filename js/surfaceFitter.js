/**
 * surfaceFitter.js — analytic surface classification and least-squares fitting
 *
 * For each face group produced by faceGrouper.js, this module attempts to
 * classify the surface as one of:
 *   plane | cylinder | sphere | nurbs (fallback)
 *
 * and returns the best-fit geometric parameters.
 *
 * All geometry math is pure JS — no OCCT dependency at this stage.
 */

import { extractGroupVertices, extractGroupNormals } from './faceGrouper.js';

// ── Math helpers ─────────────────────────────────────────────────────────────

function dot(a, b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
function cross(a, b) {
  return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
}
function norm(v) { return Math.sqrt(dot(v, v)); }
function normalize(v) {
  const n = norm(v);
  return n > 1e-14 ? [v[0]/n, v[1]/n, v[2]/n] : [0, 0, 1];
}
function sub(a, b) { return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }
function add(a, b) { return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]; }
function scale(v, s) { return [v[0]*s, v[1]*s, v[2]*s]; }

/** A vector perpendicular to v (arbitrary but consistent). */
function perp(v) {
  const abs = v.map(Math.abs);
  const minI = abs.indexOf(Math.min(...abs));
  const e = [0, 0, 0]; e[minI] = 1;
  return normalize(cross(v, e));
}

// ── 3×3 symmetric Jacobi eigensolver ────────────────────────────────────────

/**
 * Compute eigenvalues and eigenvectors of a 3×3 symmetric matrix via Jacobi
 * iteration.  Returns eigenvalues sorted ascending with matching eigenvectors.
 *
 * @param {number[]} m  [m00, m01, m02, m11, m12, m22]
 * @returns {{ values: number[], vectors: number[][] }}
 */
export function eigen3(m) {
  let a = [
    [m[0], m[1], m[2]],
    [m[1], m[3], m[4]],
    [m[2], m[4], m[5]],
  ];
  // Eigenvector matrix (starts as identity)
  let V = [[1,0,0],[0,1,0],[0,0,1]];

  for (let iter = 0; iter < 200; iter++) {
    // Find largest off-diagonal element
    let maxV = 0, p = 0, q = 1;
    for (let i = 0; i < 3; i++)
      for (let j = i+1; j < 3; j++)
        if (Math.abs(a[i][j]) > maxV) { maxV = Math.abs(a[i][j]); p = i; q = j; }
    if (maxV < 1e-14) break;

    const theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
    const t = (theta >= 0 ? 1 : -1) / (Math.abs(theta) + Math.sqrt(1 + theta*theta));
    const c = 1 / Math.sqrt(1 + t*t), s = t * c;

    const app = a[p][p], aqq = a[q][q], apq = a[p][q];
    a[p][p] = app - t * apq;
    a[q][q] = aqq + t * apq;
    a[p][q] = a[q][p] = 0;
    for (let r = 0; r < 3; r++) {
      if (r === p || r === q) continue;
      const apr = a[p][r], aqr = a[q][r];
      a[p][r] = a[r][p] = c * apr - s * aqr;
      a[q][r] = a[r][q] = s * apr + c * aqr;
    }
    for (let r = 0; r < 3; r++) {
      const vp = V[r][p], vq = V[r][q];
      V[r][p] = c * vp - s * vq;
      V[r][q] = s * vp + c * vq;
    }
  }

  const order = [0, 1, 2].sort((i, j) => a[i][i] - a[j][j]);
  return {
    values:  order.map(i => a[i][i]),
    vectors: order.map(i => [V[0][i], V[1][i], V[2][i]]),
  };
}

// ── Plane fitting ─────────────────────────────────────────────────────────────

/**
 * Fit a best-fit plane to a point cloud via PCA.
 * The normal is the eigenvector corresponding to the smallest eigenvalue of the
 * covariance matrix (direction of smallest spread = thickness direction = normal).
 *
 * @param {Float32Array} vertices  flat [x,y,z, …]
 * @returns {{ origin, normal, rms }}
 */
export function fitPlane(vertices) {
  const n = vertices.length / 3;
  let cx = 0, cy = 0, cz = 0;
  for (let i = 0; i < n; i++) {
    cx += vertices[i*3]; cy += vertices[i*3+1]; cz += vertices[i*3+2];
  }
  cx /= n; cy /= n; cz /= n;

  let c00=0, c01=0, c02=0, c11=0, c12=0, c22=0;
  for (let i = 0; i < n; i++) {
    const dx = vertices[i*3]-cx, dy = vertices[i*3+1]-cy, dz = vertices[i*3+2]-cz;
    c00+=dx*dx; c01+=dx*dy; c02+=dx*dz; c11+=dy*dy; c12+=dy*dz; c22+=dz*dz;
  }
  const scale = 1/n;
  const { vectors } = eigen3([c00*scale,c01*scale,c02*scale,c11*scale,c12*scale,c22*scale]);
  const normal = vectors[0]; // smallest eigenvalue → normal direction

  let rms = 0;
  for (let i = 0; i < n; i++) {
    const d = (vertices[i*3]-cx)*normal[0]+(vertices[i*3+1]-cy)*normal[1]+(vertices[i*3+2]-cz)*normal[2];
    rms += d*d;
  }
  return { origin: [cx,cy,cz], normal, rms: Math.sqrt(rms/n) };
}

// ── Circle fitting (2-D algebraic) ────────────────────────────────────────────

/**
 * Fit a circle to 2D points using Pratt's algebraic method.
 * Returns null if the system is degenerate.
 */
function fitCircle2D(pts) {
  const n = pts.length;
  if (n < 3) return null;
  let Sx=0, Sy=0, Sxx=0, Sxy=0, Syy=0, Sxxx=0, Sxxy=0, Sxyy=0, Syyy=0;
  for (const [x,y] of pts) {
    const x2=x*x, y2=y*y;
    Sx+=x; Sy+=y; Sxx+=x2; Sxy+=x*y; Syy+=y2;
    Sxxx+=x2*x; Sxxy+=x2*y; Sxyy+=x*y2; Syyy+=y2*y;
  }
  const A = [
    [2*(Sxx - Sx*Sx/n), 2*(Sxy - Sx*Sy/n)],
    [2*(Sxy - Sx*Sy/n), 2*(Syy - Sy*Sy/n)],
  ];
  const B = [
    Sxxx + Sxyy - Sxx*Sx/n - Sxy*Sy/n,
    Sxxy + Syyy - Sxy*Sx/n - Syy*Sy/n,
  ];
  const det = A[0][0]*A[1][1] - A[0][1]*A[1][0];
  if (Math.abs(det) < 1e-14) return null;
  const cx = (B[0]*A[1][1] - B[1]*A[0][1]) / det;
  const cy = (A[0][0]*B[1] - A[1][0]*B[0]) / det;
  let r = 0;
  for (const [x,y] of pts) r += Math.sqrt((x-cx)**2 + (y-cy)**2);
  return { cx, cy, r: r/n };
}

// ── Cylinder fitting ─────────────────────────────────────────────────────────

/**
 * Fit a cylinder to a vertex/normal cloud.
 *
 * Strategy:
 *  1. The cylinder axis is the direction perpendicular to all surface normals,
 *     i.e. the eigenvector of the normal covariance matrix with the SMALLEST
 *     eigenvalue.
 *  2. Project vertices onto the plane ⊥ axis, fit a circle to find axis point
 *     and radius.
 *
 * @param {Float32Array} vertices  flat [x,y,z, …]
 * @param {Float32Array} normals   flat [nx,ny,nz, …]
 * @returns {{ axis, axisPoint, radius, rms } | null}
 */
export function fitCylinder(vertices, normals) {
  const nv = vertices.length / 3;
  const nn = normals.length / 3;
  if (nn < 3) return null;

  // 1. Axis from normal covariance
  let c00=0,c01=0,c02=0,c11=0,c12=0,c22=0;
  for (let i = 0; i < nn; i++) {
    const x=normals[i*3], y=normals[i*3+1], z=normals[i*3+2];
    c00+=x*x; c01+=x*y; c02+=x*z; c11+=y*y; c12+=y*z; c22+=z*z;
  }
  const sc=1/nn;
  const { vectors } = eigen3([c00*sc,c01*sc,c02*sc,c11*sc,c12*sc,c22*sc]);
  const axis = normalize(vectors[0]); // smallest eigenvalue

  // 2. Build perpendicular frame
  const xhat = perp(axis);
  const yhat = normalize(cross(axis, xhat));

  // Vertex centroid
  let cx=0, cy2=0, cz=0;
  for (let i=0;i<nv;i++){cx+=vertices[i*3];cy2+=vertices[i*3+1];cz+=vertices[i*3+2];}
  cx/=nv; cy2/=nv; cz/=nv;

  // Project vertices to perpendicular plane
  const pts2d = [];
  for (let i=0;i<nv;i++){
    const dx=vertices[i*3]-cx, dy=vertices[i*3+1]-cy2, dz=vertices[i*3+2]-cz;
    const axComp = dx*axis[0]+dy*axis[1]+dz*axis[2];
    const rx=dx-axComp*axis[0], ry=dy-axComp*axis[1], rz=dz-axComp*axis[2];
    pts2d.push([rx*xhat[0]+ry*xhat[1]+rz*xhat[2],
                rx*yhat[0]+ry*yhat[1]+rz*yhat[2]]);
  }

  const circle = fitCircle2D(pts2d);
  if (!circle) return null;

  const axisPoint = [
    cx + circle.cx*xhat[0] + circle.cy*yhat[0],
    cy2+ circle.cx*xhat[1] + circle.cy*yhat[1],
    cz + circle.cx*xhat[2] + circle.cy*yhat[2],
  ];
  const radius = Math.abs(circle.r);

  // 3. RMS radial distance error
  let rms = 0;
  for (let i=0;i<nv;i++){
    const dx=vertices[i*3]-axisPoint[0], dy=vertices[i*3+1]-axisPoint[1], dz=vertices[i*3+2]-axisPoint[2];
    const axComp=dx*axis[0]+dy*axis[1]+dz*axis[2];
    const perpDist=Math.sqrt((dx-axComp*axis[0])**2+(dy-axComp*axis[1])**2+(dz-axComp*axis[2])**2);
    rms+=(perpDist-radius)**2;
  }
  return { axis, axisPoint, radius, rms: Math.sqrt(rms/nv) };
}

// ── Cone fitting ─────────────────────────────────────────────────────────────

/**
 * Fit a cone to a vertex/normal cloud.
 *
 * Strategy:
 *  1. The cone axis is the eigenvector of the normal covariance with the
 *     smallest eigenvalue (same starting point as cylinder fitting).
 *  2. Discriminate from cylinder by checking mean projection of normals onto
 *     that axis: for a cylinder ≈ 0; for a cone ≈ ±sin(halfAngle).
 *  3. Estimate the apex by linear regression: r_i = |z_i - z_apex| * tan(α).
 *
 * @param {Float32Array} vertices  flat [x,y,z, …]
 * @param {Float32Array} normals   flat [nx,ny,nz, …]
 * @returns {{ axis, apex, halfAngle, rms } | null}
 */
export function fitCone(vertices, normals) {
  const nn = normals.length / 3;
  const nv = vertices.length / 3;
  if (nn < 6 || nv < 6) return null;

  // 1. Axis estimate via normal covariance (smallest eigenvalue direction)
  let c00=0,c01=0,c02=0,c11=0,c12=0,c22=0;
  for (let i = 0; i < nn; i++) {
    const x=normals[i*3], y=normals[i*3+1], z=normals[i*3+2];
    c00+=x*x; c01+=x*y; c02+=x*z; c11+=y*y; c12+=y*z; c22+=z*z;
  }
  const sc = 1/nn;
  const { vectors } = eigen3([c00*sc,c01*sc,c02*sc,c11*sc,c12*sc,c22*sc]);
  let axis = normalize(vectors[0]); // smallest eigenvalue

  // 2. Mean projection of normals onto axis = ±sin(halfAngle) for a cone
  let meanProj = 0;
  for (let i = 0; i < nn; i++) {
    meanProj += normals[i*3]*axis[0]+normals[i*3+1]*axis[1]+normals[i*3+2]*axis[2];
  }
  meanProj /= nn;

  const absMeanProj = Math.abs(meanProj);
  // Discard if too close to cylinder (absMeanProj ≈ 0) or degenerate
  if (absMeanProj < 0.08 || absMeanProj > 0.98) return null;

  // Orient axis so normals project negatively (outward normals tilt away from axis)
  if (meanProj > 0) axis = [-axis[0], -axis[1], -axis[2]];

  const halfAngle = Math.asin(absMeanProj);
  const tanA = Math.tan(halfAngle);
  if (!isFinite(tanA) || tanA < 1e-6) return null;

  // 3. Vertex centroid
  let cx=0, cy=0, cz=0;
  for (let i=0;i<nv;i++){cx+=vertices[i*3];cy+=vertices[i*3+1];cz+=vertices[i*3+2];}
  cx/=nv; cy/=nv; cz/=nv;

  // 4. Apex estimation: z_apex = z_i - r_i/tanA (if apex below section)
  //    OR z_apex = z_i + r_i/tanA (if apex above section)
  //    Try both; pick the one with lower variance.
  let sum1=0, sum2=0;
  for (let i=0;i<nv;i++){
    const dx=vertices[i*3]-cx, dy=vertices[i*3+1]-cy, dz=vertices[i*3+2]-cz;
    const az = dx*axis[0]+dy*axis[1]+dz*axis[2];
    const rx=dx-az*axis[0], ry=dy-az*axis[1], rz=dz-az*axis[2];
    const r = Math.sqrt(rx*rx+ry*ry+rz*rz);
    sum1 += az - r/tanA;
    sum2 += az + r/tanA;
  }
  const zapex1=sum1/nv, zapex2=sum2/nv;

  let var1=0, var2=0;
  for (let i=0;i<nv;i++){
    const dx=vertices[i*3]-cx, dy=vertices[i*3+1]-cy, dz=vertices[i*3+2]-cz;
    const az = dx*axis[0]+dy*axis[1]+dz*axis[2];
    const rx=dx-az*axis[0], ry=dy-az*axis[1], rz=dz-az*axis[2];
    const r = Math.sqrt(rx*rx+ry*ry+rz*rz);
    var1 += (az-r/tanA-zapex1)**2;
    var2 += (az+r/tanA-zapex2)**2;
  }
  const zapex = var1 <= var2 ? zapex1 : zapex2;
  const apex = [cx+zapex*axis[0], cy+zapex*axis[1], cz+zapex*axis[2]];

  // 5. Vertex RMS (radial error on cone surface)
  let rms = 0;
  for (let i=0;i<nv;i++){
    const dx=vertices[i*3]-apex[0], dy=vertices[i*3+1]-apex[1], dz=vertices[i*3+2]-apex[2];
    const az = dx*axis[0]+dy*axis[1]+dz*axis[2];
    const rx=dx-az*axis[0], ry=dy-az*axis[1], rz=dz-az*axis[2];
    const r = Math.sqrt(rx*rx+ry*ry+rz*rz);
    rms += (r - Math.abs(az)*tanA)**2;
  }
  return { axis, apex, halfAngle, rms: Math.sqrt(rms/nv) };
}

// ── Sphere fitting ────────────────────────────────────────────────────────────

/**
 * Fit a sphere to a vertex cloud using linear least squares.
 * Solves: x²+y²+z² = a·x + b·y + c·z + d  →  center = [a,b,c]/2
 *
 * @param {Float32Array} vertices  flat [x,y,z, …]
 * @returns {{ center, radius, rms } | null}
 */
export function fitSphere(vertices) {
  const n = vertices.length / 3;
  if (n < 4) return null;

  // Normal equations for 4-parameter linear system [a,b,c,d]
  const A = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]];
  const b = [0,0,0,0];
  for (let i=0;i<n;i++){
    const x=vertices[i*3], y=vertices[i*3+1], z=vertices[i*3+2];
    const rhs = x*x+y*y+z*z;
    const row = [x,y,z,1];
    for (let j=0;j<4;j++){
      b[j]+=rhs*row[j];
      for (let k=0;k<4;k++) A[j][k]+=row[j]*row[k];
    }
  }

  const params = _solve4(A, b);
  if (!params) return null;
  const cx=params[0]/2, cy=params[1]/2, cz=params[2]/2;
  const r2 = cx*cx+cy*cy+cz*cz+params[3];
  if (r2<=0 || !isFinite(r2)) return null;
  const radius = Math.sqrt(r2);

  let rms=0;
  for (let i=0;i<n;i++){
    const dx=vertices[i*3]-cx, dy=vertices[i*3+1]-cy, dz=vertices[i*3+2]-cz;
    rms+=(Math.sqrt(dx*dx+dy*dy+dz*dz)-radius)**2;
  }
  return { center:[cx,cy,cz], radius, rms: Math.sqrt(rms/n) };
}

// ── 4×4 Gaussian elimination ─────────────────────────────────────────────────

function _solve4(A, b) {
  const M = A.map((row,i) => [...row, b[i]]);
  for (let col=0;col<4;col++){
    let maxRow=col;
    for (let r=col+1;r<4;r++) if (Math.abs(M[r][col])>Math.abs(M[maxRow][col])) maxRow=r;
    [M[col],M[maxRow]]=[M[maxRow],M[col]];
    if (Math.abs(M[col][col])<1e-14) return null;
    for (let r=col+1;r<4;r++){
      const f=M[r][col]/M[col][col];
      for (let c=col;c<=4;c++) M[r][c]-=f*M[col][c];
    }
  }
  const x=new Array(4);
  for (let i=3;i>=0;i--){
    x[i]=M[i][4];
    for (let j=i+1;j<4;j++) x[i]-=M[i][j]*x[j];
    x[i]/=M[i][i];
  }
  return x;
}

// ── Classification ────────────────────────────────────────────────────────────

/**
 * Classify a face group and return the best-fit surface parameters.
 *
 * @param {{ triangleIndices: Set<number> }} group
 * @param {THREE.BufferGeometry} geometry
 * @returns {{
 *   type: 'plane'|'cylinder'|'cone'|'sphere'|'nurbs',
 *   params: object,
 *   rms: number
 * }}
 */
export function classifyGroup(group, geometry) {
  const vertices = extractGroupVertices(geometry, group.triangleIndices);
  const normals  = extractGroupNormals(geometry, group.triangleIndices);
  const n = vertices.length / 3;

  // Compute bounding sphere radius for relative RMS thresholds
  const planeFit = fitPlane(vertices);

  // Orient the plane normal to agree with the mesh normals.
  // PCA yields an arbitrary sign; align it to the mean mesh-vertex normal so
  // OCCT always sees a consistently outward-pointing face normal.
  {
    const nn = normals.length / 3;
    let mx = 0, my = 0, mz = 0;
    for (let i = 0; i < nn; i++) {
      mx += normals[i*3]; my += normals[i*3+1]; mz += normals[i*3+2];
    }
    if (planeFit.normal[0]*mx + planeFit.normal[1]*my + planeFit.normal[2]*mz < 0) {
      planeFit.normal = [-planeFit.normal[0], -planeFit.normal[1], -planeFit.normal[2]];
    }
  }

  let maxDist = 0;
  for (let i=0;i<n;i++){
    const dx=vertices[i*3]-planeFit.origin[0],
          dy=vertices[i*3+1]-planeFit.origin[1],
          dz=vertices[i*3+2]-planeFit.origin[2];
    maxDist = Math.max(maxDist, Math.sqrt(dx*dx+dy*dy+dz*dz));
  }
  const scale = maxDist || 1;

  const candidates = [
    { type: 'plane', params: planeFit, rms: planeFit.rms },
  ];

  // Try cone (needs normals)
  if (n >= 6) {
    try {
      const coneFit = fitCone(vertices, normals);
      if (coneFit && coneFit.halfAngle > 0 && isFinite(coneFit.rms)) {
        candidates.push({ type: 'cone', params: coneFit, rms: coneFit.rms });
      }
    } catch { /* skip */ }
  }

  // Try cylinder (needs normals)
  if (n >= 3) {
    try {
      const cylFit = fitCylinder(vertices, normals);
      if (cylFit && cylFit.radius > 1e-6 && isFinite(cylFit.rms)) {
        candidates.push({ type: 'cylinder', params: cylFit, rms: cylFit.rms });
      }
    } catch { /* skip */ }
  }

  // Try sphere
  if (n >= 4) {
    try {
      const sphereFit = fitSphere(vertices);
      if (sphereFit && sphereFit.radius > 1e-6 && isFinite(sphereFit.rms)) {
        // Reject sphere when the fitted radius is much larger than the group's own
        // extent — this is the degenerate case where a near-flat patch is fit with a
        // huge-radius sphere that effectively approximates a plane but scores a lower
        // raw RMS because it has one extra free parameter.
        const radiusOk = sphereFit.radius <= scale * 5;

        // Reject sphere when the surface normals are nearly co-planar or co-linear.
        // A genuine sphere has normals spread in all three directions equally; a flat
        // or cylindrical face has normals concentrated in ≤ 2 directions.
        // We reuse the normal covariance we already have from fitCylinder's axis calc:
        // the eigenvalue ratio λ_min/λ_max of the normal cloud must exceed a threshold.
        let normalSpreadOk = true;
        const nn = normals.length / 3;
        if (nn >= 3) {
          let c00=0,c01=0,c02=0,c11=0,c12=0,c22=0;
          for (let i=0;i<nn;i++){
            const x=normals[i*3],y=normals[i*3+1],z=normals[i*3+2];
            c00+=x*x;c01+=x*y;c02+=x*z;c11+=y*y;c12+=y*z;c22+=z*z;
          }
          const sc=1/nn;
          const { values } = eigen3([c00*sc,c01*sc,c02*sc,c11*sc,c12*sc,c22*sc]);
          const lambdaMin = values[0], lambdaMax = values[2];
          // Require at least 15% of the dominant spread in the weakest direction
          normalSpreadOk = lambdaMax > 1e-12 && (lambdaMin / lambdaMax) >= 0.15;
        }

        if (radiusOk && normalSpreadOk) {
          candidates.push({ type: 'sphere', params: sphereFit, rms: sphereFit.rms });
        }
      }
    } catch { /* skip */ }
  }

  // Choose best candidate: lowest rms relative to scale
  // Apply a bias against complex surfaces (prefer plane > cyl > cone > sphere)
  const bias = { plane: 1.0, cylinder: 1.15, cone: 1.25, sphere: 1.5, nurbs: 9999 };
  candidates.sort((a, b) => (a.rms * bias[a.type]) - (b.rms * bias[b.type]));

  const best = candidates[0];

  // Accept best-fit surface if its relative RMS is below 2% of model scale.
  // Otherwise fall back to NURBS (the boundary geometry still drives the shape).
  const REL_TOL = 0.02;
  if (best.rms / scale > REL_TOL && best.type !== 'plane') {
    // If even cylinder/sphere don't fit well, try plane once more — planes are
    // acceptable even at moderate RMS because they're the most common case.
    if (planeFit.rms / scale < 0.05) {
      return { type: 'plane', params: planeFit, rms: planeFit.rms };
    }
    return { type: 'nurbs', params: { vertices, normals }, rms: Infinity };
  }

  return { type: best.type, params: best.params, rms: best.rms };
}

/**
 * Annotate every group in place with `.surface` = classifyGroup result.
 *
 * @param {Array<{triangleIndices: Set<number>}>} groups
 * @param {THREE.BufferGeometry} geometry
 */
export function fitAllGroups(groups, geometry) {
  for (const group of groups) {
    group.surface = classifyGroup(group, geometry);
  }
}

/**
 * Force a specific surface type for a group, re-running the appropriate fit
 * function so params match the declared type.  Returns a surface object with
 * the correct { type, params, rms } for that type, falling back to the auto-
 * classified surface when the forced fit is degenerate.
 *
 * @param {'plane'|'cylinder'|'cone'|'sphere'|'nurbs'} forcedType
 * @param {{ triangleIndices: Set<number> }} group
 * @param {THREE.BufferGeometry} geometry
 * @returns {{ type, params, rms }}
 */
export function classifyGroupAs(forcedType, group, geometry) {
  const vertices = extractGroupVertices(geometry, group.triangleIndices);
  const normals  = extractGroupNormals(geometry, group.triangleIndices);
  const n = vertices.length / 3;

  if (forcedType === 'plane') {
    const fit = fitPlane(vertices);
    // Orient to mesh normals
    const nn = normals.length / 3;
    let mx = 0, my = 0, mz = 0;
    for (let i = 0; i < nn; i++) { mx += normals[i*3]; my += normals[i*3+1]; mz += normals[i*3+2]; }
    if (fit.normal[0]*mx + fit.normal[1]*my + fit.normal[2]*mz < 0) {
      fit.normal = [-fit.normal[0], -fit.normal[1], -fit.normal[2]];
    }
    return { type: 'plane', params: fit, rms: fit.rms };
  }

  if (forcedType === 'cylinder') {
    try {
      const fit = fitCylinder(vertices, normals);
      if (fit && fit.radius > 1e-6 && isFinite(fit.rms)) return { type: 'cylinder', params: fit, rms: fit.rms };
    } catch { /* fall through */ }
  }

  if (forcedType === 'cone') {
    try {
      const fit = fitCone(vertices, normals);
      if (fit && fit.halfAngle > 0 && isFinite(fit.rms)) return { type: 'cone', params: fit, rms: fit.rms };
    } catch { /* fall through */ }
  }

  if (forcedType === 'sphere') {
    try {
      const fit = fitSphere(vertices);
      if (fit && fit.radius > 1e-6 && isFinite(fit.rms)) return { type: 'sphere', params: fit, rms: fit.rms };
    } catch { /* fall through */ }
  }

  if (forcedType === 'nurbs') {
    return { type: 'nurbs', params: { vertices, normals }, rms: Infinity };
  }

  // Forced fit was degenerate or didn't satisfy quality thresholds.
  // Rather than silently switching to the auto type (which would give the
  // visualizer params that don't match the declared type and cause a crash),
  // derive plausible fallback params for the forced type from a plane fit.
  // rms: Infinity signals to the caller that the fit was degenerate.
  const pf = fitPlane(vertices);
  const nn2 = normals.length / 3;
  let mx2 = 0, my2 = 0, mz2 = 0;
  for (let i = 0; i < nn2; i++) { mx2 += normals[i*3]; my2 += normals[i*3+1]; mz2 += normals[i*3+2]; }
  if (pf.normal[0]*mx2 + pf.normal[1]*my2 + pf.normal[2]*mz2 < 0) {
    pf.normal = [-pf.normal[0], -pf.normal[1], -pf.normal[2]];
  }
  let ext = 0;
  for (let i = 0; i < n; i++) {
    const dx = vertices[i*3] - pf.origin[0], dy = vertices[i*3+1] - pf.origin[1], dz = vertices[i*3+2] - pf.origin[2];
    ext = Math.max(ext, Math.sqrt(dx*dx + dy*dy + dz*dz));
  }
  const scale = ext > 1e-9 ? ext : 1;

  if (forcedType === 'cylinder') {
    return { type: 'cylinder', params: { axis: pf.normal, axisPoint: pf.origin, radius: scale * 0.5 }, rms: Infinity };
  }
  if (forcedType === 'cone') {
    const apexOffset = scale;
    return { type: 'cone', params: {
      axis: pf.normal,
      apex: [pf.origin[0] + pf.normal[0]*apexOffset, pf.origin[1] + pf.normal[1]*apexOffset, pf.origin[2] + pf.normal[2]*apexOffset],
      halfAngle: Math.PI / 6,  // 30° placeholder
    }, rms: Infinity };
  }
  if (forcedType === 'sphere') {
    return { type: 'sphere', params: { center: pf.origin, radius: scale }, rms: Infinity };
  }

  // Last resort: return auto classification
  return classifyGroup(group, geometry);
}
