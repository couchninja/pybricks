import * as THREE from "three";

const OCEAN = new THREE.Color(20 / 255, 70 / 255, 150 / 255);
const LAND = new THREE.Color(45 / 255, 130 / 255, 55 / 255);
const ICE = new THREE.Color(235 / 255, 240 / 255, 245 / 255);
const DESERT = new THREE.Color(170 / 255, 150 / 255, 90 / 255);

function wrapLongitude(lonDeg: number): number {
  return ((lonDeg + 180) % 360) - 180;
}

function inRegion(
  latDeg: number,
  lonDeg: number,
  latMin: number,
  latMax: number,
  lonMin: number,
  lonMax: number,
): boolean {
  return latDeg >= latMin && latDeg <= latMax && lonDeg >= lonMin && lonDeg <= lonMax;
}

function landMask(latDeg: number, lonDeg: number): boolean {
  const lon = wrapLongitude(lonDeg);
  let land =
    inRegion(latDeg, lon, 49, 72, -170, -55) ||
    inRegion(latDeg, lon, 25, 49, -125, -65) ||
    inRegion(latDeg, lon, 15, 30, -115, -80) ||
    inRegion(latDeg, lon, 7, 20, -92, -77) ||
    inRegion(latDeg, lon, -56, 15, -82, -34) ||
    inRegion(latDeg, lon, 36, 72, -25, 45) ||
    inRegion(latDeg, lon, -35, 37, -18, 52) ||
    inRegion(latDeg, lon, 5, 77, 40, 145) ||
    inRegion(latDeg, lon, 5, 77, 145, 180) ||
    inRegion(latDeg, lon, 5, 77, -180, -168) ||
    inRegion(latDeg, lon, -45, -10, 112, 154) ||
    inRegion(latDeg, lon, 60, 84, -58, -20) ||
    inRegion(latDeg, lon, -47, -34, 166, 179) ||
    inRegion(latDeg, lon, -6, 6, 95, 141) ||
    inRegion(latDeg, lon, -26, -12, 43, 50) ||
    inRegion(latDeg, lon, 50, 59, -11, 2) ||
    inRegion(latDeg, lon, 30, 46, 129, 146) ||
    inRegion(latDeg, lon, 5, 22, 99, 109) ||
    inRegion(latDeg, lon, 6, 13, -5, 2) ||
    inRegion(latDeg, lon, 62, 67, 20, 32) ||
    inRegion(latDeg, lon, 76, 82, -70, -12) ||
    latDeg <= -62;
  land &&= !inRegion(latDeg, lon, 18, 30, -98, -82);
  land &&= !inRegion(latDeg, lon, 53, 66, -90, -60);
  land &&= !inRegion(latDeg, lon, 56, 66, 20, 45);
  land &&= !inRegion(latDeg, lon, 12, 28, 32, 44);
  land &&= !inRegion(latDeg, lon, 41, 46, 26, 42);
  return land;
}

function desertMask(latDeg: number, lonDeg: number): boolean {
  const lon = wrapLongitude(lonDeg);
  return (
    inRegion(latDeg, lon, 15, 35, -17, 40) ||
    inRegion(latDeg, lon, 12, 32, 35, 55) ||
    inRegion(latDeg, lon, 25, 37, 70, 90) ||
    inRegion(latDeg, lon, 18, 30, 110, 125) ||
    inRegion(latDeg, lon, -30, -20, 115, 130)
  );
}

function faceColor(latDeg: number, lonDeg: number): THREE.Color {
  if (latDeg >= 66 || latDeg <= -60) {
    return ICE;
  }
  if (landMask(latDeg, lonDeg)) {
    if (desertMask(latDeg, lonDeg)) {
      return DESERT;
    }
    return LAND;
  }
  return OCEAN;
}

export function createEarthMesh(radius: number): THREE.Mesh {
  const geometry = new THREE.IcosahedronGeometry(radius, 4);
  const positions = geometry.getAttribute("position");
  const colors = new Float32Array(positions.count * 3);
  const scratch = new THREE.Vector3();
  for (let index = 0; index < positions.count; index += 1) {
    scratch.fromBufferAttribute(positions, index).normalize();
    const lat = Math.asin(scratch.z);
    const lon = Math.atan2(scratch.y, scratch.x);
    const latDeg = (lat * 180) / Math.PI;
    const lonDeg = (lon * 180) / Math.PI;
    const color = faceColor(latDeg, lonDeg);
    colors[index * 3] = color.r;
    colors[index * 3 + 1] = color.g;
    colors[index * 3 + 2] = color.b;
  }
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  const material = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.85 });
  return new THREE.Mesh(geometry, material);
}
