import * as THREE from "three";
import { describe, expect, it } from "vitest";

import { orientFlatArrow, worldArrowShaftDirection } from "./billboard-arrow";

function makeCamera(position: THREE.Vector3): THREE.PerspectiveCamera {
  const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 1000);
  camera.position.copy(position);
  camera.lookAt(0, 0, 0);
  camera.updateMatrixWorld(true);
  return camera;
}

function expectDirectionClose(actual: THREE.Vector3, expected: THREE.Vector3, toleranceRad = 0.02): void {
  expect(actual.angleTo(expected)).toBeLessThan(toleranceRad);
}

describe("orientFlatArrow", () => {
  it("keeps the shaft on the world target direction", () => {
    const group = new THREE.Group();
    const direction = new THREE.Vector3(1, 0, 0);
    orientFlatArrow(group, new THREE.Vector3(0, 0, 0), direction, makeCamera(new THREE.Vector3(0, 0, 20)));
    expectDirectionClose(worldArrowShaftDirection(group), direction);
  });

  it("keeps oblique target direction when viewed from the side", () => {
    const group = new THREE.Group();
    const direction = new THREE.Vector3(1, 0, 1).normalize();
    orientFlatArrow(group, new THREE.Vector3(0, 0, 0), direction, makeCamera(new THREE.Vector3(0, 0, 15)));
    expectDirectionClose(worldArrowShaftDirection(group), direction);
  });

  it("uses world origin for camera-facing twist under a translated parent", () => {
    const parent = new THREE.Group();
    parent.position.set(-2.5, 1.25, -0.5);
    const group = new THREE.Group();
    parent.add(group);
    parent.updateMatrixWorld(true);

    const direction = new THREE.Vector3(0, 1, 0);
    orientFlatArrow(group, new THREE.Vector3(0, 0, 0), direction, makeCamera(new THREE.Vector3(-2.5, 1.25, 19.5)));
    expectDirectionClose(worldArrowShaftDirection(group), direction);
  });

  it("does not rotate the shaft when only the camera moves", () => {
    const direction = new THREE.Vector3(1, 2, 0.5).normalize();
    const origin = new THREE.Vector3(0, 0, 0);
    const groupA = new THREE.Group();
    const groupB = new THREE.Group();
    orientFlatArrow(groupA, origin, direction, makeCamera(new THREE.Vector3(0, 0, 20)));
    orientFlatArrow(groupB, origin, direction, makeCamera(new THREE.Vector3(15, 3, 4)));
    expectDirectionClose(worldArrowShaftDirection(groupA), direction);
    expectDirectionClose(worldArrowShaftDirection(groupB), direction);
  });

  it("twists so local +Z is toward the camera but stays perpendicular to the shaft", () => {
    const group = new THREE.Group();
    const direction = new THREE.Vector3(0, 1, 0);
    const camera = makeCamera(new THREE.Vector3(3, 4, 12));
    orientFlatArrow(group, new THREE.Vector3(1, -2, 0.5), direction, camera);

    group.updateMatrixWorld(true);
    const worldOrigin = new THREE.Vector3().setFromMatrixPosition(group.matrixWorld);
    const shaft = worldArrowShaftDirection(group);
    const faceNormal = new THREE.Vector3(0, 0, 1).transformDirection(group.matrixWorld).normalize();
    const toCamera = camera.position.clone().sub(worldOrigin).normalize();
    expect(Math.abs(faceNormal.dot(shaft))).toBeLessThan(0.02);
    expect(faceNormal.dot(toCamera)).toBeGreaterThan(0.85);
  });
});
