import * as THREE from "three";

/** NASA Visible Earth 57752 — Blue Marble land/shallow water/shaded topography (public domain). */
const EARTH_TEXTURE_PATH = "/textures/land_shallow_topo_2048.jpg";

const EARTH_SPHERE_WIDTH = 64;
const EARTH_SPHERE_HEIGHT = 32;

let earthMap: THREE.Texture | null = null;

function earthMapTexture(): THREE.Texture {
  if (earthMap === null) {
    earthMap = new THREE.TextureLoader().load(EARTH_TEXTURE_PATH);
    earthMap.colorSpace = THREE.SRGBColorSpace;
    earthMap.flipY = false;
    earthMap.wrapS = THREE.RepeatWrapping;
    earthMap.repeat.x = -1;
    earthMap.offset.x = 1;
  }
  return earthMap;
}

export function createEarthMesh(radius: number): THREE.Mesh {
  const geometry = new THREE.SphereGeometry(radius, EARTH_SPHERE_WIDTH, EARTH_SPHERE_HEIGHT);
  const material = new THREE.MeshStandardMaterial({
    map: earthMapTexture(),
    roughness: 0.85,
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.rotation.x = -Math.PI / 2;
  return mesh;
}
