import * as THREE from "three";

const _toCamera = new THREE.Vector3();
const _shaft = new THREE.Vector3();
const _cardNormal = new THREE.Vector3();
const _worldOrigin = new THREE.Vector3();
const _basisX = new THREE.Vector3();
const _basisY = new THREE.Vector3();
const _basisZ = new THREE.Vector3();
const _basisMatrix = new THREE.Matrix4();
const _worldUp = new THREE.Vector3(0, 1, 0);
const _worldX = new THREE.Vector3(1, 0, 0);
const _localShaft = new THREE.Vector3(0, 1, 0);

export type FlatArrowMaterialOptions = {
  occludedOpacity: number;
  drawOccludedPass: boolean;
  visibleRenderOrder: number;
  occludedRenderOrder: number;
  visiblePolygonOffset: { factor: number; units: number };
  occludedPolygonOffset: { factor: number; units: number };
};

export function createFlatArrowGeometry(
  length: number,
  headLength: number,
  shaftWidth: number,
  headWidth: number,
): THREE.ExtrudeGeometry {
  const shaftLength = length - headLength;
  const halfShaft = shaftWidth / 2;
  const halfHead = headWidth / 2;
  const shape = new THREE.Shape();
  shape.moveTo(-halfShaft, 0);
  shape.lineTo(halfShaft, 0);
  shape.lineTo(halfShaft, shaftLength);
  shape.lineTo(halfHead, shaftLength);
  shape.lineTo(0, length);
  shape.lineTo(-halfHead, shaftLength);
  shape.lineTo(-halfShaft, shaftLength);
  shape.closePath();
  const depth = Math.max(length * 0.015, shaftWidth * 0.35);
  return new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false });
}

export function createFlatArrowGroup(
  length: number,
  color: THREE.Color,
  headLength: number,
  headWidth: number,
  shaftWidth: number,
  materialOptions: FlatArrowMaterialOptions,
): THREE.Group {
  const group = new THREE.Group();
  const geometry = createFlatArrowGeometry(length, headLength, shaftWidth, headWidth);
  const visibleMaterial = new THREE.MeshBasicMaterial({
    color,
    polygonOffset: true,
    polygonOffsetFactor: materialOptions.visiblePolygonOffset.factor,
    polygonOffsetUnits: materialOptions.visiblePolygonOffset.units,
  });
  const occludedMaterial = new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity: materialOptions.occludedOpacity,
    depthTest: true,
    depthFunc: THREE.GreaterDepth,
    depthWrite: false,
    polygonOffset: true,
    polygonOffsetFactor: materialOptions.occludedPolygonOffset.factor,
    polygonOffsetUnits: materialOptions.occludedPolygonOffset.units,
  });
  const visible = new THREE.Mesh(geometry, visibleMaterial);
  visible.renderOrder = materialOptions.visibleRenderOrder;
  visible.userData.arrowOutline = true;
  group.add(visible);

  if (materialOptions.drawOccludedPass) {
    const occluded = new THREE.Mesh(geometry, occludedMaterial);
    occluded.renderOrder = materialOptions.occludedRenderOrder;
    group.add(occluded);
  }

  group.frustumCulled = false;
  return group;
}

export function disposeFlatArrowGroup(group: THREE.Group): void {
  const geometries = new Set<THREE.BufferGeometry>();
  const materials = new Set<THREE.Material>();
  for (const child of group.children) {
    if (!(child instanceof THREE.Mesh)) {
      continue;
    }
    geometries.add(child.geometry);
    materials.add(child.material as THREE.Material);
  }
  for (const geometry of geometries) {
    geometry.dispose();
  }
  for (const material of materials) {
    material.dispose();
  }
}

/** Local +Y in world space (base toward tip). */
export function worldArrowShaftDirection(group: THREE.Object3D): THREE.Vector3 {
  group.updateMatrixWorld(true);
  const base = new THREE.Vector3().setFromMatrixPosition(group.matrixWorld);
  const tip = new THREE.Vector3(0, 1, 0).applyMatrix4(group.matrixWorld);
  return tip.sub(base).normalize();
}

/** Shaft locked to `direction`; twist around it so +Z faces the camera when possible. */
export function orientFlatArrow(
  group: THREE.Group,
  origin: THREE.Vector3,
  direction: THREE.Vector3,
  camera: THREE.PerspectiveCamera,
): void {
  group.position.copy(origin);

  if (direction.lengthSq() < 1e-20) {
    return;
  }
  _shaft.copy(direction).normalize();

  group.parent?.updateMatrixWorld(true);
  group.getWorldPosition(_worldOrigin);

  _toCamera.subVectors(camera.position, _worldOrigin);
  if (_toCamera.lengthSq() < 1e-20) {
    group.quaternion.setFromUnitVectors(_localShaft, _shaft);
    group.updateMatrixWorld(true);
    return;
  }
  _toCamera.normalize();

  _cardNormal.copy(_toCamera).addScaledVector(_shaft, -_toCamera.dot(_shaft));
  if (_cardNormal.lengthSq() < 1e-20) {
    _cardNormal.crossVectors(_shaft, _worldUp);
    if (_cardNormal.lengthSq() < 1e-20) {
      _cardNormal.crossVectors(_shaft, _worldX);
    }
  }
  _cardNormal.normalize();
  if (_cardNormal.dot(_toCamera) < 0) {
    _cardNormal.negate();
  }

  _basisY.copy(_shaft);
  _basisZ.copy(_cardNormal);
  _basisX.crossVectors(_basisY, _basisZ).normalize();
  _basisZ.crossVectors(_basisX, _basisY).normalize();

  _basisMatrix.makeBasis(_basisX, _basisY, _basisZ);
  group.quaternion.setFromRotationMatrix(_basisMatrix);
  group.updateMatrixWorld(true);
}
