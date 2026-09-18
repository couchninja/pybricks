import * as THREE from "three";

import { simulationTimeMsFromIso } from "./simulation-time";

export type KeplerianOrbitParams = {
  kind: "keplerian";
  name: string;
  color: [number, number, number];
  matrix: number[];
  epoch_iso: string;
  period_days: number;
  a_au: number;
  e: number;
  i_rad: number;
  raan_rad: number;
  argp_rad: number;
  M0_rad: number;
  mu_au3_per_day2: number;
  origin_body: string | null;
};

const TWO_PI = 2 * Math.PI;
const MS_PER_DAY = 86_400_000;

function solveKepler(M: number, e: number): number {
  let E = e < 0.8 ? M : Math.PI;
  for (let i = 0; i < 64; i += 1) {
    const delta = (E - e * Math.sin(E) - M) / (1 - e * Math.cos(E));
    E -= delta;
    if (Math.abs(delta) < 1e-12) {
      break;
    }
  }
  return E;
}

function meanToTrueAnomaly(M: number, e: number): number {
  const E = solveKepler(M, e);
  const sinE = Math.sin(E);
  const cosE = Math.cos(E);
  const sinNu = (Math.sqrt(1 - e * e) * sinE) / (1 - e * cosE);
  const cosNu = (cosE - e) / (1 - e * cosE);
  let nu = Math.atan2(sinNu, cosNu);
  if (nu < 0) {
    nu += TWO_PI;
  }
  return nu;
}

function positionFromElements(
  a: number,
  e: number,
  i: number,
  raan: number,
  argp: number,
  nu: number,
  target: THREE.Vector3,
): void {
  const p = a * (1 - e * e);
  const r = p / (1 + e * Math.cos(nu));
  const xP = r * Math.cos(nu);
  const yP = r * Math.sin(nu);

  const cosRaan = Math.cos(raan);
  const sinRaan = Math.sin(raan);
  const cosI = Math.cos(i);
  const sinI = Math.sin(i);
  const cosArgp = Math.cos(argp);
  const sinArgp = Math.sin(argp);

  target.set(
    (cosRaan * cosArgp - sinRaan * sinArgp * cosI) * xP + (-cosRaan * sinArgp - sinRaan * cosArgp * cosI) * yP,
    (sinRaan * cosArgp + cosRaan * sinArgp * cosI) * xP + (-sinRaan * sinArgp + cosRaan * cosArgp * cosI) * yP,
    sinArgp * sinI * xP + cosArgp * sinI * yP,
  );
}

function meanAnomalyAtTime(params: KeplerianOrbitParams, timeMs: number): number {
  const epochMs = simulationTimeMsFromIso(params.epoch_iso);
  const deltaDays = (timeMs - epochMs) / MS_PER_DAY;
  const n = Math.sqrt(params.mu_au3_per_day2 / params.a_au ** 3);
  let M = params.M0_rad + n * deltaDays;
  M %= TWO_PI;
  if (M < 0) {
    M += TWO_PI;
  }
  return M;
}

export function sampleKeplerianOrbitPoints(
  params: KeplerianOrbitParams,
  timeMs: number,
  samples: number,
  origin: THREE.Vector3 | null,
  out: THREE.Vector3[],
): void {
  out.length = 0;
  const scratch = new THREE.Vector3();
  const MAtEpoch = meanAnomalyAtTime(params, timeMs);
  for (let index = 0; index < samples; index += 1) {
    const phase = (TWO_PI * index) / samples;
    let M = MAtEpoch + phase;
    M %= TWO_PI;
    const nu = meanToTrueAnomaly(M, params.e);
    positionFromElements(params.a_au, params.e, params.i_rad, params.raan_rad, params.argp_rad, nu, scratch);
    if (origin) {
      scratch.add(origin);
    }
    out.push(scratch.clone());
  }
}
