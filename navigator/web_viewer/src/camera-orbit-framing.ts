import * as THREE from "three";

import type { ScenePath, SceneSnapshot } from "./scene-types";

const ORBIT_PLANE_ELEVATION_RAD = THREE.MathUtils.degToRad(20);

/** Velocity / orbit-direction targets share framing with their body counterpart. */
const ORBITAL_DIRECTION_FRAMING: Partial<Record<string, string>> = {
  sun_orbit: "sun",
  milky_way_orbit: "milky_way_center",
  cmb_dipole: "milky_way_center",
};

function framingPointingTarget(pointingTarget: string): string {
  return ORBITAL_DIRECTION_FRAMING[pointingTarget] ?? pointingTarget;
}

/** Scene path used to infer the orbit plane for each pointing target. */
const POINTING_TARGET_ORBIT_PATH: Partial<Record<string, string>> = {
  sun: "earth_orbit",
  sun_orbit: "earth_orbit",
  moon: "moon_orbit",
  iss: "iss_orbit",
  milky_way_center: "galactic_orbit",
  milky_way_orbit: "galactic_orbit",
  cmb_dipole: "galactic_orbit",
  earth_rotation: "earth_axis",
};

const ORBIT_PATH_CENTER_BODY: Partial<Record<string, string>> = {
  earth_orbit: "sun",
  moon_orbit: "earth",
  iss_orbit: "earth",
  galactic_orbit: "galactic_center",
};

const HEAVENLY_BODY_ORBIT_PATH: Partial<Record<string, string>> = {
  sun: "earth_orbit",
  moon: "moon_orbit",
  iss: "iss_orbit",
  galactic_center: "galactic_orbit",
};

/** Orbits around Earth (LEO, lunar); reject heliocentric paths misread as geocentric. */
const MAX_GEOCENTRIC_ORBIT_RADIUS_AU = 0.01;

const _heavenlyOrbitCenter = new THREE.Vector3();
const _heavenlyOrbitBody = new THREE.Vector3();
const _heavenlyOrbitMatrix = new THREE.Matrix4();

const BODY_FOR_TARGET: Partial<Record<string, string>> = {
  sun: "sun",
  moon: "moon",
  iss: "iss",
  milky_way_center: "galactic_center",
};

const WORLD_UP = new THREE.Vector3(0, 1, 0);
const EARTH_VIEWPORT_FILL = 0.9;
const ORBIT_VIEWPORT_FILL = 0.9;

export type OrbitCameraPose = {
  offset: THREE.Vector3;
  up: THREE.Vector3;
};

function pathPoints(path: ScenePath): THREE.Vector3[] {
  const matrix = new THREE.Matrix4().fromArray(path.matrix);
  const points: THREE.Vector3[] = [];
  for (const segment of path.segments) {
    for (const [x, y, z] of segment) {
      points.push(new THREE.Vector3(x, y, z).applyMatrix4(matrix));
    }
  }
  return points;
}

function newellPlaneNormal(points: THREE.Vector3[]): THREE.Vector3 | null {
  if (points.length < 3) {
    return null;
  }
  const normal = new THREE.Vector3();
  for (let i = 0; i < points.length; i += 1) {
    const current = points[i];
    const next = points[(i + 1) % points.length];
    normal.x += (current.y - next.y) * (current.z + next.z);
    normal.y += (current.z - next.z) * (current.x + next.x);
    normal.z += (current.x - next.x) * (current.y + next.y);
  }
  if (normal.lengthSq() < 1e-20) {
    return null;
  }
  return normal.normalize();
}

function lineDirection(points: THREE.Vector3[]): THREE.Vector3 | null {
  if (points.length < 2) {
    return null;
  }
  const direction = points[points.length - 1].clone().sub(points[0]);
  if (direction.lengthSq() < 1e-20) {
    return null;
  }
  return direction.normalize();
}

function meanPathRadius(path: ScenePath, center: THREE.Vector3): number | null {
  const points = pathPoints(path);
  if (points.length === 0) {
    return null;
  }
  let sum = 0;
  for (const point of points) {
    sum += point.distanceTo(center);
  }
  return sum / points.length;
}

function orbitPlaneNormal(
  snapshot: SceneSnapshot,
  pointingTarget: string,
  currentOffsetDirection: THREE.Vector3,
): THREE.Vector3 {
  const pathName = POINTING_TARGET_ORBIT_PATH[framingPointingTarget(pointingTarget)];
  if (pathName) {
    const path = snapshot.paths.find((entry) => entry.name === pathName);
    if (path) {
      const points = pathPoints(path);
      const axis = pathName === "earth_axis" ? lineDirection(points) : newellPlaneNormal(points);
      if (axis) {
        return axis.dot(currentOffsetDirection) >= 0 ? axis.clone() : axis.clone().negate();
      }
    }
  }

  const velocityArrow = snapshot.arrows.find((entry) => entry.name === "observer_velocity_arrow");
  if (velocityArrow) {
    const direction = new THREE.Vector3(...velocityArrow.direction).normalize();
    return direction.dot(currentOffsetDirection) >= 0 ? direction.clone() : direction.clone().negate();
  }

  return currentOffsetDirection.lengthSq() > 1e-20
    ? currentOffsetDirection.clone().normalize()
    : new THREE.Vector3(0, 0, 1);
}

function inPlaneHorizontal(normal: THREE.Vector3, observer: THREE.Vector3, toward: THREE.Vector3): THREE.Vector3 {
  const inPlane = toward.clone().sub(observer);
  inPlane.sub(normal.clone().multiplyScalar(inPlane.dot(normal)));
  if (inPlane.lengthSq() > 1e-16) {
    return inPlane.normalize();
  }

  const fallback = new THREE.Vector3().crossVectors(normal, WORLD_UP);
  if (fallback.lengthSq() > 1e-16) {
    return fallback.normalize();
  }
  return new THREE.Vector3().crossVectors(normal, new THREE.Vector3(1, 0, 0)).normalize();
}

function targetWorldPosition(
  snapshot: SceneSnapshot,
  pointingTarget: string,
  observer: THREE.Vector3,
  bodyWorldPosition: (name: string) => THREE.Vector3 | null,
): THREE.Vector3 {
  const bodyName = BODY_FOR_TARGET[framingPointingTarget(pointingTarget)];
  if (bodyName) {
    const bodyPosition = bodyWorldPosition(bodyName);
    if (bodyPosition) {
      return bodyPosition;
    }
  }
  if (pointingTarget === "cmb_dipole") {
    const cmb = snapshot.arrows.find((entry) => entry.name === "cmb_dipole_arrow");
    if (cmb) {
      const extent = Math.max(snapshot.default_camera_distance_au, snapshot.scene_scale);
      return new THREE.Vector3(...cmb.base).add(new THREE.Vector3(...cmb.direction).multiplyScalar(extent));
    }
  }
  const velocityArrow = snapshot.arrows.find((entry) => entry.name === "observer_velocity_arrow");
  if (velocityArrow) {
    const extent = Math.max(snapshot.default_camera_distance_au, snapshot.scene_scale);
    return observer.clone().add(new THREE.Vector3(...velocityArrow.direction).multiplyScalar(extent));
  }
  return observer.clone().add(new THREE.Vector3(0, 0, snapshot.default_camera_distance_au));
}

function bodyPositionFromSnapshot(snapshot: SceneSnapshot, name: string, target: THREE.Vector3): boolean {
  const entry = snapshot.bodies.find((body) => body.name === name);
  if (!entry) {
    return false;
  }
  _heavenlyOrbitMatrix.fromArray(entry.matrix);
  target.setFromMatrixPosition(_heavenlyOrbitMatrix);
  return true;
}

function geocentricOrbitRadiusFromSnapshot(bodyName: string, snapshot: SceneSnapshot): number | null {
  if (!bodyPositionFromSnapshot(snapshot, "earth", _heavenlyOrbitCenter)) {
    return null;
  }
  if (!bodyPositionFromSnapshot(snapshot, bodyName, _heavenlyOrbitBody)) {
    return null;
  }
  return _heavenlyOrbitBody.distanceTo(_heavenlyOrbitCenter);
}

function geocentricBodyOrbitRadiusAu(
  bodyName: string,
  snapshot: SceneSnapshot,
  bodyWorldPosition: (name: string) => THREE.Vector3 | null,
): number | null {
  const earth = bodyWorldPosition("earth");
  const body = bodyWorldPosition(bodyName);
  if (earth && body) {
    _heavenlyOrbitCenter.copy(earth);
    _heavenlyOrbitBody.copy(body);
    const liveRadius = _heavenlyOrbitBody.distanceTo(_heavenlyOrbitCenter);
    if (liveRadius > 1e-10) {
      return liveRadius;
    }
  }
  return geocentricOrbitRadiusFromSnapshot(bodyName, snapshot);
}

function geocentricPathOrbitRadiusAu(
  pathName: string,
  snapshot: SceneSnapshot,
  bodyWorldPosition: (name: string) => THREE.Vector3 | null,
): number | null {
  const path = snapshot.paths.find((entry) => entry.name === pathName);
  const centerBody = ORBIT_PATH_CENTER_BODY[pathName];
  const center = centerBody ? bodyWorldPosition(centerBody) : null;
  if (!path || !center) {
    return null;
  }
  _heavenlyOrbitCenter.copy(center);
  const radius = meanPathRadius(path, _heavenlyOrbitCenter);
  if (radius === null || radius <= 0) {
    return null;
  }
  if (centerBody === "earth" && radius > MAX_GEOCENTRIC_ORBIT_RADIUS_AU) {
    return null;
  }
  return radius;
}

export function heavenlyBodyOrbitRadiusAu(
  bodyName: string,
  snapshot: SceneSnapshot,
  bodyWorldPosition: (name: string) => THREE.Vector3 | null,
): number | null {
  if (bodyName === "earth" || bodyName === "observer") {
    return null;
  }
  const pathName = HEAVENLY_BODY_ORBIT_PATH[bodyName];
  if (pathName) {
    const pathRadius = geocentricPathOrbitRadiusAu(pathName, snapshot, bodyWorldPosition);
    if (pathRadius !== null) {
      return pathRadius;
    }
  }
  if (bodyName === "moon" || bodyName === "iss") {
    return geocentricBodyOrbitRadiusAu(bodyName, snapshot, bodyWorldPosition);
  }
  return snapshot.earth_orbit_radius_au;
}

function halfViewportFovRad(camera: THREE.PerspectiveCamera, fill: number): number {
  const halfFovY = THREE.MathUtils.degToRad(camera.fov / 2) * fill;
  const halfFovX = Math.atan(Math.tan(halfFovY) * camera.aspect) * fill;
  return Math.min(halfFovX, halfFovY);
}

function issOrbitPlaneNormalTowardObserver(
  snapshot: SceneSnapshot,
  earthCenter: THREE.Vector3,
  observer: THREE.Vector3,
  currentOffsetDirection: THREE.Vector3,
): THREE.Vector3 | null {
  const path = snapshot.paths.find((entry) => entry.name === "iss_orbit");
  if (!path) {
    return null;
  }
  const normal = newellPlaneNormal(pathPoints(path));
  if (!normal) {
    return null;
  }
  const observerSide = observer.clone().sub(earthCenter).dot(normal);
  if (observerSide < -1e-20) {
    normal.negate();
  } else if (Math.abs(observerSide) <= 1e-20 && currentOffsetDirection.dot(normal) < 0) {
    normal.negate();
  }
  return normal;
}

function minDistanceToFramePointsInView(
  camera: THREE.PerspectiveCamera,
  observer: THREE.Vector3,
  offsetDirection: THREE.Vector3,
  points: THREE.Vector3[],
  minDistance: number,
  maxDistance: number,
): number {
  if (points.length === 0) {
    return minDistance;
  }
  const halfFov = halfViewportFovRad(camera, ORBIT_VIEWPORT_FILL);
  const fits = (distance: number): boolean => {
    const cameraPosition = observer.clone().addScaledVector(offsetDirection, distance);
    const toPivot = observer.clone().sub(cameraPosition).normalize();
    for (const point of points) {
      const toPoint = point.clone().sub(cameraPosition);
      if (toPoint.lengthSq() < 1e-24) {
        continue;
      }
      if (toPoint.normalize().angleTo(toPivot) > halfFov) {
        return false;
      }
    }
    return true;
  };

  const upper = Number.isFinite(maxDistance) && maxDistance > 0 ? maxDistance : minDistance;
  if (!fits(upper)) {
    return upper;
  }
  if (fits(minDistance)) {
    return minDistance;
  }

  let lo = minDistance;
  let hi = upper;
  for (let step = 0; step < 48; step += 1) {
    const mid = (lo + hi) / 2;
    if (fits(mid)) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  return hi;
}

function desiredIssOrbitCameraPose(
  camera: THREE.PerspectiveCamera,
  snapshot: SceneSnapshot,
  pointingTarget: string,
  observer: THREE.Vector3,
  bodyWorldPosition: (name: string) => THREE.Vector3 | null,
  currentOffsetDirection: THREE.Vector3,
  maxDistance: number,
): OrbitCameraPose | null {
  const earthCenter = bodyWorldPosition("earth");
  if (!earthCenter) {
    return null;
  }
  const normal = issOrbitPlaneNormalTowardObserver(snapshot, earthCenter, observer, currentOffsetDirection);
  if (!normal) {
    return null;
  }

  const path = snapshot.paths.find((entry) => entry.name === "iss_orbit");
  if (!path) {
    return null;
  }
  const orbitPoints = pathPoints(path);
  const toward = targetWorldPosition(snapshot, pointingTarget, observer, bodyWorldPosition);
  const up = inPlaneHorizontal(normal, observer, toward);

  const minDistance = Math.max(snapshot.default_camera_distance_au * 0.01, 1e-14);
  let distance = minDistanceToFramePointsInView(camera, observer, normal, orbitPoints, minDistance, maxDistance);
  if (Number.isFinite(maxDistance) && maxDistance > 0) {
    distance = Math.min(distance, maxDistance);
  }
  distance = Math.max(distance, minDistance);

  return {
    offset: normal.clone().multiplyScalar(distance),
    up,
  };
}

function orbitRadiusAu(
  snapshot: SceneSnapshot,
  pointingTarget: string,
  bodyWorldPosition: (name: string) => THREE.Vector3 | null,
): number {
  const target = framingPointingTarget(pointingTarget);

  if (target === "moon" || target === "iss") {
    const bodyName = BODY_FOR_TARGET[target];
    if (bodyName) {
      const radius = geocentricBodyOrbitRadiusAu(bodyName, snapshot, bodyWorldPosition);
      if (radius !== null && radius > 0) {
        return radius;
      }
    }
  }

  const pathName = POINTING_TARGET_ORBIT_PATH[target];
  if (pathName && pathName !== "earth_axis") {
    const path = snapshot.paths.find((entry) => entry.name === pathName);
    const centerBody = ORBIT_PATH_CENTER_BODY[pathName];
    const center = centerBody ? bodyWorldPosition(centerBody) : null;
    if (path && center) {
      const radius = meanPathRadius(path, center);
      if (radius !== null && radius > 0) {
        return radius;
      }
    }
  }

  return snapshot.earth_orbit_radius_au;
}

function desiredEarthRotationCameraPose(
  camera: THREE.PerspectiveCamera,
  snapshot: SceneSnapshot,
  pointingTarget: string,
  observer: THREE.Vector3,
  bodyWorldPosition: (name: string) => THREE.Vector3 | null,
  currentOffsetDirection: THREE.Vector3,
): OrbitCameraPose | null {
  const earthCenter = bodyWorldPosition("earth");
  if (!earthCenter) {
    return null;
  }

  const earthRadius = snapshot.earth_radius_au;
  const fromCenter = observer.clone().sub(earthCenter);
  let outward = fromCenter.clone();
  if (outward.lengthSq() < 1e-20) {
    outward = currentOffsetDirection.clone().normalize();
  } else {
    outward.normalize();
  }

  let up = orbitPlaneNormal(snapshot, pointingTarget, currentOffsetDirection);
  if (up.dot(WORLD_UP) < 0) {
    up = up.negate();
  }
  if (Math.abs(up.dot(outward)) > 0.95) {
    const tangent = new THREE.Vector3().crossVectors(up, WORLD_UP);
    up =
      tangent.lengthSq() > 1e-16
        ? tangent.normalize()
        : new THREE.Vector3().crossVectors(up, new THREE.Vector3(1, 0, 0)).normalize();
  }

  const halfFov = halfViewportFovRad(camera, EARTH_VIEWPORT_FILL);

  const distance = Math.max(earthRadius / Math.tan(halfFov) - earthRadius, earthRadius * 0.25);

  return {
    offset: outward.multiplyScalar(distance),
    up,
  };
}

export function desiredOrbitTargetCameraPose(
  camera: THREE.PerspectiveCamera,
  snapshot: SceneSnapshot,
  pointingTarget: string,
  observer: THREE.Vector3,
  bodyWorldPosition: (name: string) => THREE.Vector3 | null,
  currentOffsetDirection: THREE.Vector3,
  maxDistance: number,
): OrbitCameraPose {
  if (framingPointingTarget(pointingTarget) === "iss") {
    const issPose = desiredIssOrbitCameraPose(
      camera,
      snapshot,
      pointingTarget,
      observer,
      bodyWorldPosition,
      currentOffsetDirection,
      maxDistance,
    );
    if (issPose) {
      return issPose;
    }
  }

  if (pointingTarget === "earth_rotation") {
    const earthPose = desiredEarthRotationCameraPose(
      camera,
      snapshot,
      pointingTarget,
      observer,
      bodyWorldPosition,
      currentOffsetDirection,
    );
    if (earthPose) {
      return earthPose;
    }
  }

  let normal = orbitPlaneNormal(snapshot, pointingTarget, currentOffsetDirection);
  if (normal.dot(WORLD_UP) < 0) {
    normal = normal.negate();
  }

  const toward = targetWorldPosition(snapshot, pointingTarget, observer, bodyWorldPosition);
  const horizontal = inPlaneHorizontal(normal, observer, toward);

  const elevation = ORBIT_PLANE_ELEVATION_RAD;
  const offsetDirection = horizontal
    .clone()
    .multiplyScalar(-Math.cos(elevation))
    .add(normal.clone().multiplyScalar(Math.sin(elevation)))
    .normalize();

  let distance = orbitRadiusAu(snapshot, pointingTarget, bodyWorldPosition);
  if (Number.isFinite(maxDistance) && maxDistance > 0) {
    distance = Math.min(distance, maxDistance);
  }
  distance = Math.max(distance, snapshot.default_camera_distance_au * 0.01);

  return {
    offset: offsetDirection.multiplyScalar(distance),
    up: normal,
  };
}

export function cameraPoseFromState(camera: THREE.PerspectiveCamera, pivot: THREE.Vector3): OrbitCameraPose {
  return {
    offset: camera.position.clone().sub(pivot),
    up: camera.up.clone().normalize(),
  };
}
