import * as THREE from "three";

import { sampleKeplerianOrbitPoints, type KeplerianOrbitParams } from "./keplerian-orbit";
import {
  DEFAULT_PARAMETRIC_ORBIT_SAMPLES,
  EARTH_ORBIT_PARAMETRIC_SAMPLES,
  GALACTIC_ORBIT_PARAMETRIC_SAMPLES,
} from "./scene-viewer-constants";
import { simulationTimeMsFromIso } from "./simulation-time";
import { sampleTleOrbitPoints, type TleOrbitParams } from "./tle-orbit";
import type { ScenePath, SceneSnapshot } from "./scene-types";

export function parametricOrbitSampleCount(orbitName: string): number {
  if (orbitName === "earth_orbit") {
    return EARTH_ORBIT_PARAMETRIC_SAMPLES;
  }
  if (orbitName === "galactic_orbit") {
    return GALACTIC_ORBIT_PARAMETRIC_SAMPLES;
  }
  return DEFAULT_PARAMETRIC_ORBIT_SAMPLES;
}

export function worldPointsFromScenePath(path: ScenePath, out: THREE.Vector3[]): void {
  const matrix = new THREE.Matrix4().fromArray(path.matrix);
  for (const segment of path.segments) {
    for (const [x, y, z] of segment) {
      out.push(new THREE.Vector3(x, y, z).applyMatrix4(matrix));
    }
  }
}

export function orbitWorldPointsFromSnapshot(
  snapshot: SceneSnapshot,
  pathName: string,
  timeMs: number,
  out: THREE.Vector3[],
): boolean {
  out.length = 0;
  const path = snapshot.paths.find((entry) => entry.name === pathName);
  if (path && path.segments.some((segment) => segment.length >= 2)) {
    worldPointsFromScenePath(path, out);
    return out.length > 0;
  }

  const orbit = snapshot.parametric_orbits.find((entry) => entry.name === pathName);
  if (!orbit) {
    return false;
  }

  const localPoints: THREE.Vector3[] = [];
  let origin: THREE.Vector3 | null = null;
  if (orbit.origin_body === "earth") {
    const anchor = orbit.origin_heliocentric_au;
    if (anchor === undefined) {
      return false;
    }
    origin = new THREE.Vector3(anchor[0], anchor[1], anchor[2]);
  }

  const samples = parametricOrbitSampleCount(pathName);
  if (orbit.kind === "keplerian") {
    sampleKeplerianOrbitPoints(orbit as KeplerianOrbitParams, timeMs, samples, origin, localPoints);
  } else {
    sampleTleOrbitPoints(orbit as TleOrbitParams, timeMs, samples, origin, localPoints);
  }

  const matrix = new THREE.Matrix4().fromArray(orbit.matrix);
  for (const point of localPoints) {
    out.push(point.clone().applyMatrix4(matrix));
  }
  return out.length > 0;
}

export function snapshotSimulationTimeMs(snapshot: SceneSnapshot): number {
  return simulationTimeMsFromIso(snapshot.simulation_time_iso);
}
