import * as THREE from "three";
import { propagate, twoline2satrec, type SatRec } from "satellite.js";

export type TleOrbitParams = {
  kind: "tle";
  name: string;
  color: [number, number, number];
  matrix: number[];
  line1: string;
  line2: string;
  epoch_iso: string;
  period_s: number;
  origin_body: string | null;
  eci_to_ecliptic: number[];
};

const KM_PER_AU = 149_597_870.7;
const MS_PER_SECOND = 1000;

function eclipticRotation(params: TleOrbitParams): THREE.Matrix3 {
  const rotation = new THREE.Matrix3();
  rotation.fromArray(params.eci_to_ecliptic);
  return rotation;
}

function geocentricEclipticAu(params: TleOrbitParams, satrec: SatRec, timeMs: number, target: THREE.Vector3): void {
  const date = new Date(timeMs);
  const propagation = propagate(satrec, date);
  if (!propagation.position || propagation.position === true) {
    target.set(0, 0, 0);
    return;
  }
  const { x, y, z } = propagation.position;
  const eciAu = new THREE.Vector3(x / KM_PER_AU, y / KM_PER_AU, z / KM_PER_AU);
  target.copy(eciAu).applyMatrix3(eclipticRotation(params));
}

export function sampleTleOrbitPoints(
  params: TleOrbitParams,
  timeMs: number,
  samples: number,
  earthPosition: THREE.Vector3 | null,
  out: THREE.Vector3[],
): void {
  out.length = 0;
  const satrec = twoline2satrec(params.line1, params.line2);
  const halfPeriodMs = (params.period_s * MS_PER_SECOND) / 2;
  const scratch = new THREE.Vector3();
  for (let index = 0; index < samples; index += 1) {
    const fraction = index / samples;
    const sampleMs = timeMs - halfPeriodMs + fraction * params.period_s * MS_PER_SECOND;
    geocentricEclipticAu(params, satrec, sampleMs, scratch);
    if (earthPosition) {
      scratch.add(earthPosition);
    }
    out.push(scratch.clone());
  }
}
