import { describe, expect, it } from "vitest";
import * as THREE from "three";

import { orbitWorldPointsFromSnapshot, snapshotSimulationTimeMs } from "./orbit-path-samples";
import { sampleTleOrbitPoints } from "./tle-orbit";
import type { SceneSnapshot, TleOrbitSnapshot } from "./scene-types";

const ISS_TLE: Pick<TleOrbitSnapshot, "line1" | "line2" | "period_s" | "eci_to_ecliptic"> = {
  line1: "1 25544U 98067A   26261.14280998  .00005718  00000+0  11125-3 0  9991",
  line2: "2 25544  51.6307 200.0361 0004822 152.4527 207.6718 15.49160218586162",
  period_s: 5577.215254826535,
  eci_to_ecliptic: [
    0.999999995133502, -9.865590290400186e-5, -9.391434674406852e-8, -6.463510082409228e-6, 0.917466516466091,
    -0.3978130102470242, -6.311079224114597e-6, 0.39769392055836267, 0.9175181445132783,
  ],
};

describe("sampleTleOrbitPoints", () => {
  it("stays geocentric when earth offset is omitted (ISS path matrix is near identity in root frame)", () => {
    const params: TleOrbitSnapshot = {
      kind: "tle",
      name: "iss_orbit",
      color: [100, 165, 255],
      matrix: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
      epoch_iso: "2026-09-18 15:59:32.189",
      origin_body: "earth",
      origin_heliocentric_au: [1.001549503750877, -0.0807266199955463, 1.4181899593204395e-6],
      ...ISS_TLE,
    };
    const timeMs = Date.parse("2026-09-18T15:59:32.189Z");
    const geocentric: THREE.Vector3[] = [];
    sampleTleOrbitPoints(params, timeMs, 128, null, geocentric);
    expect(geocentric.length).toBe(128);
    const maxRadiusAu = Math.max(...geocentric.map((point) => point.length()));
    expect(maxRadiusAu).toBeLessThan(0.001);
  });
});

describe("orbitWorldPointsFromSnapshot ISS TLE", () => {
  it("places the parametric loop near Earth in root frame", () => {
    const snapshot: SceneSnapshot = {
      simulation_time_iso: "2026-09-18T15:59:32.189Z",
      inertial_to_root_rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1],
      bodies: [],
      paths: [],
      parametric_orbits: [
        {
          kind: "tle",
          name: "iss_orbit",
          color: [100, 165, 255],
          matrix: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
          epoch_iso: "2026-09-18 15:59:32.189",
          origin_body: "earth",
          origin_heliocentric_au: [1.001549503750877, -0.0807266199955463, 1.4181899593204395e-6],
          ...ISS_TLE,
        },
      ],
      arrows: [],
    };
    const world: THREE.Vector3[] = [];
    const ok = orbitWorldPointsFromSnapshot(snapshot, "iss_orbit", snapshotSimulationTimeMs(snapshot), world);
    expect(ok).toBe(true);
    const maxRadiusAu = Math.max(...world.map((point) => point.length()));
    expect(maxRadiusAu).toBeLessThan(0.001);
  });
});
