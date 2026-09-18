import { describe, expect, it } from "vitest";
import * as THREE from "three";

import { desiredOrbitTargetCameraPose } from "./camera-orbit-framing";
import type { SceneSnapshot } from "./scene-types";

function translationMatrix(position: THREE.Vector3): number[] {
  return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, position.x, position.y, position.z, 1];
}

function baseSnapshot(overrides: Partial<SceneSnapshot> = {}): SceneSnapshot {
  return {
    inertial_to_root_rotation: [1, 0, 0, 0, 1, 0, 0, 0, 1],
    bodies: [],
    paths: [],
    arrows: [],
    ...overrides,
  };
}

describe("desiredOrbitTargetCameraPose for ISS", () => {
  it("places the camera on the observer side, perpendicular to the orbit plane", () => {
    const orbitRadius = 0.01;
    const earthCenter = new THREE.Vector3(0.02, 0, 0);
    const observer = new THREE.Vector3(0.02, 0, 0.005);
    const issPosition = new THREE.Vector3(0.02 + orbitRadius, 0, 0);

    const snapshot = baseSnapshot({
      bodies: [
        {
          name: "earth",
          radius: 0.001,
          color: [0, 0, 1],
          matrix: translationMatrix(earthCenter),
        },
        {
          name: "iss",
          radius: 0.0001,
          color: [0, 0, 1],
          matrix: translationMatrix(issPosition),
        },
      ],
      paths: [
        {
          name: "iss_orbit",
          color: [1, 1, 1],
          matrix: translationMatrix(new THREE.Vector3()),
          segments: [
            [
              [earthCenter.x + orbitRadius, 0, 0],
              [earthCenter.x, orbitRadius, 0],
              [earthCenter.x - orbitRadius, 0, 0],
              [earthCenter.x, -orbitRadius, 0],
            ],
          ],
        },
      ],
    });

    const camera = new THREE.PerspectiveCamera(45, 1, 0.0001, 100);
    const bodyWorldPosition = (name: string) => {
      if (name === "earth") {
        return earthCenter.clone();
      }
      if (name === "iss") {
        return issPosition.clone();
      }
      return null;
    };

    const pose = desiredOrbitTargetCameraPose(
      camera,
      snapshot,
      "iss",
      observer,
      bodyWorldPosition,
      new THREE.Vector3(0, 0, 1),
      100,
    );

    const offsetDirection = pose.offset.clone().normalize();
    expect(Math.abs(offsetDirection.z)).toBeGreaterThan(0.99);
    expect(offsetDirection.z).toBeGreaterThan(0);

    const cameraPosition = observer.clone().add(pose.offset);
    const toPivot = observer.clone().sub(cameraPosition).normalize();
    for (const [x, y, z] of snapshot.paths[0].segments[0]) {
      const point = new THREE.Vector3(x, y, z);
      const toPoint = point.clone().sub(cameraPosition).normalize();
      const halfFov = THREE.MathUtils.degToRad(camera.fov / 2) * 0.9;
      expect(toPoint.angleTo(toPivot)).toBeLessThanOrEqual(halfFov + 1e-6);
    }
  });
});
