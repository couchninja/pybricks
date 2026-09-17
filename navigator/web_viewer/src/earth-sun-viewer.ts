import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { CSS2DObject, CSS2DRenderer } from "three/examples/jsm/renderers/CSS2DRenderer.js";

import { createConstellationSky, skyOpacityForCameraDistance } from "./constellation-sky";
import { createEarthMesh } from "./earth-mesh";
import type {
  ArrowDistanceAnchor,
  Rgb,
  SceneArrow,
  SceneBody,
  ScenePath,
  SceneSnapshot,
} from "./scene-types";

const VIEWER_TARGET_FPS = 60;
const VIEWER_FRAME_MS = 1000 / VIEWER_TARGET_FPS;

const LABEL_OFFSET_BODY_RADII = 2.5;
const BODY_LABEL_COLORS: Record<string, string> = {
  sun: "rgb(255, 210, 60)",
  earth: "rgb(255, 255, 255)",
  moon: "rgb(210, 210, 205)",
  iss: "rgb(70, 130, 255)",
  observer: "rgb(255, 80, 40)",
  galactic_center: "rgb(240, 200, 255)",
};

function rgbToThree([r, g, b]: Rgb): THREE.Color {
  return new THREE.Color(r / 255, g / 255, b / 255);
}

function matrixFromSnapshot(values: number[]): THREE.Matrix4 {
  const matrix = new THREE.Matrix4();
  matrix.fromArray(values);
  return matrix;
}

function createSphereMesh(radius: number, color: Rgb): THREE.Mesh {
  const geometry = new THREE.IcosahedronGeometry(radius, 3);
  const material = new THREE.MeshStandardMaterial({
    color: rgbToThree(color),
    roughness: 0.75,
  });
  return new THREE.Mesh(geometry, material);
}

const ARROW_HEAD_LENGTH_FRACTION = 0.35;
const ARROW_HEAD_RADIUS_EARTH_RADII = 0.12;
/** Web view: longer than desktop so the arrow stays readable when zoomed out. */
const ARROW_LENGTH_BOOST = 2.5;
type ArrowFrame = {
  base: THREE.Vector3;
  direction: THREE.Vector3;
  color: Rgb;
  distanceAnchor: ArrowDistanceAnchor;
  displayLengthAu: number;
};

function disposeArrowHelper(arrow: THREE.ArrowHelper): void {
  arrow.line.geometry.dispose();
  (arrow.line.material as THREE.Material).dispose();
  arrow.cone.geometry.dispose();
  (arrow.cone.material as THREE.Material).dispose();
}

function cameraDistanceForArrow(
  camera: THREE.PerspectiveCamera,
  target: THREE.Vector3,
  anchor: ArrowDistanceAnchor,
  galacticCenter: THREE.Vector3,
): number {
  if (anchor === "galactic_center") {
    return camera.position.distanceTo(galacticCenter);
  }
  return camera.position.distanceTo(target);
}

function createLabel(text: string, color: string): CSS2DObject {
  const element = document.createElement("div");
  element.textContent = text;
  element.style.color = color;
  element.style.font = "600 14px system-ui, sans-serif";
  element.style.textShadow = "0 1px 3px rgba(0,0,0,0.85)";
  element.style.pointerEvents = "none";
  element.style.whiteSpace = "nowrap";
  return new CSS2DObject(element);
}

type BodyEntry = {
  root: THREE.Object3D;
  label: CSS2DObject | null;
  radius: number;
};

type PathEntry = {
  root: THREE.Object3D;
  signature: string;
};

export class EarthSunViewer {
  readonly root: HTMLElement;

  private readonly renderer: THREE.WebGLRenderer;
  private readonly labelRenderer: CSS2DRenderer;
  private readonly scene: THREE.Scene;
  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;
  private readonly bodies = new Map<string, BodyEntry>();
  private readonly paths = new Map<string, PathEntry>();
  private readonly arrows = new Map<string, THREE.ArrowHelper>();
  private readonly arrowFrames = new Map<string, ArrowFrame>();
  private readonly galacticCenter = new THREE.Vector3();
  private readonly constellationSky = createConstellationSky(1);
  private arrowMeshLengthAu = 0;
  private arrowEarthRadiusAu = 0;
  private arrowLengthCameraFraction = 0.05;
  private arrowMinLengthAu = 0;
  private readonly captionEl: HTMLDivElement;
  private readonly fpsEl: HTMLDivElement;
  private defaultCameraDistance = 1;
  private earthRadiusAu = 1;
  private earthOrbitRadiusAu = 1;
  private skyboxFadeOrbitMultiple = 10;
  private skyboxFadeSpanOrbitMultiple = 10;
  private skyOpacity = 1;
  private sceneScale = 1;
  private animationId = 0;
  private lastRenderTime = 0;
  private geometryPollIntervalMs = VIEWER_FRAME_MS;
  private cameraMotionUntil = 0;
  private fpsFrames = 0;
  private fpsIntervalStart = performance.now();
  private fpsDisplay = 0;
  private hasInitialCamera = false;

  constructor(mount: HTMLElement) {
    this.root = document.createElement("div");
    this.root.className = "earth-sun-viewer";
    mount.appendChild(this.root);

    const canvasHost = document.createElement("div");
    canvasHost.className = "earth-sun-viewer-canvas";
    this.root.appendChild(canvasHost);

    this.captionEl = document.createElement("div");
    this.captionEl.className = "earth-sun-viewer-caption";
    canvasHost.appendChild(this.captionEl);

    this.fpsEl = document.createElement("div");
    this.fpsEl.className = "earth-sun-viewer-fps";
    canvasHost.appendChild(this.fpsEl);

    this.scene = new THREE.Scene();
    this.scene.add(this.constellationSky.root);

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.001, 1000);
    this.camera.position.set(0, 0, 1);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    canvasHost.appendChild(this.renderer.domElement);

    this.labelRenderer = new CSS2DRenderer();
    this.labelRenderer.domElement.style.position = "absolute";
    this.labelRenderer.domElement.style.inset = "0";
    this.labelRenderer.domElement.style.pointerEvents = "none";
    canvasHost.appendChild(this.labelRenderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.target.set(0, 0, 0);
    this.controls.addEventListener("change", () => {
      this.cameraMotionUntil = performance.now() + 250;
      this.clampCameraDistance();
      this.syncClipPlanes();
      this.layoutArrows();
    });

    const ambient = new THREE.AmbientLight(0xffffff, 0.35);
    const sunLight = new THREE.DirectionalLight(0xfff2cc, 1.1);
    sunLight.position.set(3, 2, 4);
    this.scene.add(ambient, sunLight);

    window.addEventListener("resize", this.onResize);
    this.onResize();
    this.animate();
  }

  setGeometryPollIntervalMs(intervalMs: number): void {
    this.geometryPollIntervalMs = intervalMs;
  }

  dispose(): void {
    cancelAnimationFrame(this.animationId);
    window.removeEventListener("resize", this.onResize);
    this.renderer.dispose();
    this.root.remove();
  }

  applySnapshot(snapshot: SceneSnapshot): void {
    this.defaultCameraDistance = snapshot.default_camera_distance_au;
    this.earthRadiusAu = snapshot.earth_radius_au;
    this.earthOrbitRadiusAu = snapshot.earth_orbit_radius_au;
    this.skyboxFadeOrbitMultiple = snapshot.skybox_fade_camera_distance_orbit_multiple;
    this.skyboxFadeSpanOrbitMultiple = snapshot.skybox_fade_span_orbit_radius_multiple;
    this.arrowMeshLengthAu = snapshot.arrow_mesh_length_au;
    this.arrowEarthRadiusAu = snapshot.earth_radius_au;
    const fraction = snapshot.arrow_length_camera_distance_fraction;
    this.arrowLengthCameraFraction =
      typeof fraction === "number" && fraction > 0
        ? fraction
        : snapshot.arrow_mesh_length_au / snapshot.default_camera_distance_au;
    this.arrowMinLengthAu = snapshot.arrow_min_length_au;
    this.sceneScale = snapshot.scene_scale;
    this.constellationSky.setInertialToRootRotation(snapshot.inertial_to_root_rotation);
    if (snapshot.milky_way_diameter_au > 0) {
      this.controls.maxDistance = snapshot.milky_way_diameter_au;
    }
    this.clampCameraDistance();
    this.captionEl.textContent = `Earth-sun (${snapshot.time_iso})`;

    for (const body of snapshot.bodies) {
      this.updateBody(body);
    }
    for (const path of snapshot.paths) {
      this.updatePath(path);
    }
    for (const arrow of snapshot.arrows) {
      this.updateArrow(arrow);
    }
    this.layoutArrows();
    this.syncClipPlanes();
    this.updateLabels();
    if (!this.hasInitialCamera) {
      this.resetCamera();
      this.hasInitialCamera = true;
    }
    this.lastRenderTime = 0;
  }

  private updateBody(body: SceneBody): void {
    let entry = this.bodies.get(body.name);
    if (!entry) {
      const root = new THREE.Object3D();
      root.name = body.name;
      const label =
        body.name === "observer"
          ? null
          : createLabel(body.label, BODY_LABEL_COLORS[body.name] ?? "white");
      if (label) {
        root.add(label);
      }
      this.scene.add(root);
      entry = { root, label, radius: 0 };
      this.bodies.set(body.name, entry);
    }

    if (Math.abs(entry.radius - body.radius) > entry.radius * 1e-6) {
      entry.root.children
        .filter((child) => child instanceof THREE.Mesh)
        .forEach((mesh) => {
          entry.root.remove(mesh);
          (mesh as THREE.Mesh).geometry.dispose();
        });
      const mesh =
        body.name === "earth" ? createEarthMesh(body.radius) : createSphereMesh(body.radius, body.color);
      entry.root.add(mesh);
      entry.radius = body.radius;
    }

    entry.root.matrixAutoUpdate = false;
    entry.root.matrix.copy(matrixFromSnapshot(body.matrix));
    entry.root.matrixWorldNeedsUpdate = true;
    if (entry.label) {
      entry.label.position.set(0, 0, body.radius * LABEL_OFFSET_BODY_RADII);
    }
    if (body.name === "galactic_center") {
      this.galacticCenter.setFromMatrixPosition(entry.root.matrix);
    }
  }

  private updatePath(path: ScenePath): void {
    const signature = JSON.stringify(path.segments);
    const existing = this.paths.get(path.name);
    if (existing?.signature === signature) {
      return;
    }

    if (existing) {
      this.scene.remove(existing.root);
      existing.root.traverse((object) => {
        if (object instanceof THREE.Line) {
          object.geometry.dispose();
          (object.material as THREE.Material).dispose();
        }
      });
    }

    const root = new THREE.Object3D();
    root.name = path.name;
    for (const segment of path.segments) {
      if (segment.length < 2) {
        continue;
      }
      const points = segment.map(([x, y, z]) => new THREE.Vector3(x, y, z));
      const geometry = new THREE.BufferGeometry().setFromPoints(points);
      const material = new THREE.LineBasicMaterial({
        color: rgbToThree(path.color),
        transparent: true,
        opacity: 0.95,
      });
      root.add(new THREE.Line(geometry, material));
    }
    this.scene.add(root);
    this.paths.set(path.name, { root, signature });
  }

  private updateArrow(arrow: SceneArrow): void {
    this.arrowFrames.set(arrow.name, {
      base: new THREE.Vector3(...arrow.base),
      direction: new THREE.Vector3(...arrow.direction).normalize(),
      color: arrow.color,
      distanceAnchor: arrow.distance_anchor,
      displayLengthAu: -1,
    });
  }

  private arrowDisplayLengthAu(cameraDistanceAu: number): number {
    const length = Math.max(
      cameraDistanceAu * this.arrowLengthCameraFraction,
      this.arrowMinLengthAu,
    );
    return length * ARROW_LENGTH_BOOST;
  }

  private layoutArrows(): void {
    if (this.arrowMeshLengthAu <= 0) {
      return;
    }
    for (const [name, frame] of this.arrowFrames) {
      const cameraDistance = cameraDistanceForArrow(
        this.camera,
        this.controls.target,
        frame.distanceAnchor,
        this.galacticCenter,
      );
      const displayLength = this.arrowDisplayLengthAu(cameraDistance);
      const lengthDelta = Math.abs(displayLength - frame.displayLengthAu);
      const rebuild =
        lengthDelta > frame.displayLengthAu * 0.02 ||
        lengthDelta > this.arrowMeshLengthAu * 0.02;
      if (!rebuild && this.arrows.has(name)) {
        continue;
      }
      frame.displayLengthAu = displayLength;

      const existing = this.arrows.get(name);
      if (existing) {
        this.scene.remove(existing);
        disposeArrowHelper(existing);
      }

      const sizeScale = displayLength / this.arrowMeshLengthAu;
      const headLength = displayLength * ARROW_HEAD_LENGTH_FRACTION;
      const headWidth = 2 * ARROW_HEAD_RADIUS_EARTH_RADII * this.arrowEarthRadiusAu * sizeScale;
      const helper = new THREE.ArrowHelper(
        frame.direction,
        frame.base,
        displayLength,
        rgbToThree(frame.color),
        headLength,
        headWidth,
      );
      this.scene.add(helper);
      this.arrows.set(name, helper);
    }
  }

  private clampCameraDistance(): void {
    const maxDistance = this.controls.maxDistance;
    if (!Number.isFinite(maxDistance) || maxDistance <= 0) {
      return;
    }
    const offset = this.camera.position.clone().sub(this.controls.target);
    const distance = offset.length();
    if (distance <= maxDistance) {
      return;
    }
    offset.multiplyScalar(maxDistance / distance);
    this.camera.position.copy(this.controls.target).add(offset);
  }

  private syncClipPlanes(): void {
    const distance = this.camera.position.distanceTo(this.controls.target);
    const far = distance + this.sceneScale * 2;
    const near = Math.max(this.earthRadiusAu * 0.01, far / 5e7, 1e-5);
    this.camera.near = near;
    this.camera.far = far;
    this.camera.updateProjectionMatrix();
    this.constellationSky.setRadius(this.constellationSky.radiusForCameraFar(far));
    this.updateSkyOpacity();
  }

  private updateSkyOpacity(): void {
    const cameraDistance = this.camera.position.distanceTo(this.controls.target);
    const opacity = skyOpacityForCameraDistance(
      cameraDistance,
      this.earthOrbitRadiusAu,
      this.skyboxFadeOrbitMultiple,
      this.skyboxFadeSpanOrbitMultiple,
    );
    this.skyOpacity = opacity;
    this.constellationSky.setOpacity(opacity);
  }

  private updateLabels(): void {
    const hideWhenBehind = (label: CSS2DObject, visibleOpacity = 1): void => {
      const world = new THREE.Vector3();
      label.getWorldPosition(world);
      const projected = world.clone().project(this.camera);
      const behind = projected.z < -1 || projected.z > 1;
      label.element.style.opacity = behind ? "0" : String(visibleOpacity);
    };
    for (const entry of this.bodies.values()) {
      const label = entry.label;
      if (!label) {
        continue;
      }
      hideWhenBehind(label);
    }
    for (const label of this.constellationSky.labels) {
      hideWhenBehind(label, this.skyOpacity);
    }
  }

  private onResize = (): void => {
    const host = this.renderer.domElement.parentElement;
    if (!host) {
      return;
    }
    const width = host.clientWidth;
    const height = host.clientHeight;
    this.camera.aspect = width / Math.max(height, 1);
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
    this.labelRenderer.setSize(width, height);

    if (this.camera.position.lengthSq() < 1e-12) {
      this.camera.position.set(0, 0, this.defaultCameraDistance || 1);
      this.controls.update();
    }
  };

  private renderIntervalMs(now: number): number {
    if (now < this.cameraMotionUntil) {
      return VIEWER_FRAME_MS;
    }
    return this.geometryPollIntervalMs;
  }

  private animate = (): void => {
    this.animationId = requestAnimationFrame(this.animate);
    this.controls.update();
    const now = performance.now();
    if (now - this.lastRenderTime < this.renderIntervalMs(now)) {
      return;
    }
    this.lastRenderTime = now;
    this.layoutArrows();
    this.syncClipPlanes();
    this.updateLabels();
    this.renderer.render(this.scene, this.camera);
    this.labelRenderer.render(this.scene, this.camera);
    this.updateFps();
  };

  private updateFps(): void {
    this.fpsFrames += 1;
    const elapsed = performance.now() - this.fpsIntervalStart;
    if (elapsed < 1000) {
      return;
    }
    this.fpsDisplay = (this.fpsFrames * 1000) / elapsed;
    this.fpsFrames = 0;
    this.fpsIntervalStart = performance.now();
    this.fpsEl.textContent = `${this.fpsDisplay.toFixed(0)} FPS`;
  };

  resetCamera(): void {
    this.controls.target.set(0, 0, 0);
    this.camera.position.set(0, 0, this.defaultCameraDistance);
    this.controls.update();
  }
}
