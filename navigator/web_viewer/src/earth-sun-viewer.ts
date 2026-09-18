import * as THREE from "three";
import { WebGPURenderer } from "three/webgpu";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { CSS2DObject, CSS2DRenderer } from "three/examples/jsm/renderers/CSS2DRenderer.js";

import { createViewerOutlinePipeline, type ViewerOutlinePipeline } from "./viewer-outline-pipeline";

import {
  cameraPoseFromState,
  desiredOrbitTargetCameraPose,
  heavenlyBodyOrbitRadiusAu,
} from "./camera-orbit-framing";
import {
  cameraOrbitTweenActive,
  tweenCameraToOrbitPose,
  updateCameraTargetTweens,
  type CameraOrbitTweenHandle,
} from "./camera-target-orbit";
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
const _labelWorldCenter = new THREE.Vector3();
const _labelWorldAnchor = new THREE.Vector3();
const _cameraScreenDown = new THREE.Vector3();
const _galacticCenterWorld = new THREE.Vector3();
const _orbitPivot = new THREE.Vector3();
const SELF_LIT_BODY_EMISSIVE_INTENSITY: Partial<Record<string, number>> = {
  sun: 0.65,
  observer: 0.45,
  iss: 0.45,
};
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

function createSphereMesh(radius: number, color: Rgb, emissiveIntensity = 0): THREE.Mesh {
  const geometry = new THREE.IcosahedronGeometry(radius, 3);
  const threeColor = rgbToThree(color);
  const material = new THREE.MeshStandardMaterial({
    color: threeColor,
    roughness: 0.75,
    ...(emissiveIntensity > 0
      ? { emissive: threeColor.clone(), emissiveIntensity }
      : {}),
  });
  return new THREE.Mesh(geometry, material);
}

const ARROW_HEAD_LENGTH_FRACTION = 0.1;
const ARROW_HEAD_RADIUS_EARTH_RADII = 0.15;
const ARROW_SHAFT_RADIUS_EARTH_RADII = 0.07;
const ARROW_OCCLUDED_OPACITY = 0.45;
/** Temporary: set true to draw the faded behind-geometry arrow pass. */
const ARROW_OCCLUDED_PASS_ENABLED = true;
const ARROW_OCCLUDED_RENDER_ORDER = 1;
const ARROW_VISIBLE_RENDER_ORDER = 2;
/** Separate duplicate arrow passes in depth to avoid WebGPU z-fighting at large clip distances. */
const ARROW_VISIBLE_POLYGON_OFFSET = { factor: -2, units: -2 };
const ARROW_OCCLUDED_POLYGON_OFFSET = { factor: 2, units: 2 };
/** Web view: longer than desktop so the arrow stays readable when zoomed out. */
const ARROW_LENGTH_BOOST = 2.5;
/** Matches `OBSERVER_VELOCITY_ARROW_SHAFT_START_OBSERVER_RADIUS_MULTIPLE` in simulate/astronomy/constants.py */
const ARROW_SHAFT_START_OBSERVER_RADIUS_MULTIPLE = 50;
type ArrowFrame = {
  base: THREE.Vector3;
  direction: THREE.Vector3;
  color: Rgb;
  distanceAnchor: ArrowDistanceAnchor;
  displayLengthAu: number;
  displayGapAu: number;
};

function observerArrowShaftStartAu(
  observerRadiusAu: number,
  cameraDistanceAu: number,
  defaultCameraDistanceAu: number,
): number {
  if (defaultCameraDistanceAu <= 0) {
    return 0;
  }
  const defaultGapAu = ARROW_SHAFT_START_OBSERVER_RADIUS_MULTIPLE * observerRadiusAu;
  return defaultGapAu * (cameraDistanceAu / defaultCameraDistanceAu);
}

function addArrowPart(
  group: THREE.Group,
  geometry: THREE.BufferGeometry,
  positionY: number,
  visibleMaterial: THREE.Material,
  occludedMaterial: THREE.Material,
): void {
  const visible = new THREE.Mesh(geometry, visibleMaterial);
  visible.position.y = positionY;
  visible.renderOrder = ARROW_VISIBLE_RENDER_ORDER;
  visible.userData.arrowOutline = true;
  group.add(visible);

  if (!ARROW_OCCLUDED_PASS_ENABLED) {
    return;
  }

  const occluded = new THREE.Mesh(geometry, occludedMaterial);
  occluded.position.y = positionY;
  occluded.renderOrder = ARROW_OCCLUDED_RENDER_ORDER;
  group.add(occluded);
}

function createSceneArrow(
  direction: THREE.Vector3,
  origin: THREE.Vector3,
  length: number,
  color: THREE.Color,
  headLength: number,
  headRadius: number,
  shaftRadius: number,
): THREE.Group {
  const group = new THREE.Group();
  const visibleMaterial = new THREE.MeshBasicMaterial({
    color,
    polygonOffset: true,
    polygonOffsetFactor: ARROW_VISIBLE_POLYGON_OFFSET.factor,
    polygonOffsetUnits: ARROW_VISIBLE_POLYGON_OFFSET.units,
  });
  const occludedMaterial = new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity: ARROW_OCCLUDED_OPACITY,
    depthTest: true,
    depthFunc: THREE.GreaterDepth,
    depthWrite: false,
    polygonOffset: true,
    polygonOffsetFactor: ARROW_OCCLUDED_POLYGON_OFFSET.factor,
    polygonOffsetUnits: ARROW_OCCLUDED_POLYGON_OFFSET.units,
  });
  const shaftLength = length - headLength;
  const shaftGeometry = new THREE.CylinderGeometry(shaftRadius, shaftRadius, shaftLength, 12);
  addArrowPart(group, shaftGeometry, shaftLength / 2, visibleMaterial, occludedMaterial);
  const headGeometry = new THREE.ConeGeometry(headRadius, headLength, 12);
  addArrowPart(group, headGeometry, shaftLength + headLength / 2, visibleMaterial, occludedMaterial);
  group.position.copy(origin);
  group.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), direction);
  group.frustumCulled = false;
  return group;
}

function disposeSceneArrow(group: THREE.Group): void {
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

function cameraDistanceForArrow(
  camera: THREE.PerspectiveCamera,
  target: THREE.Vector3,
  anchor: ArrowDistanceAnchor,
  galacticCenterRoot: THREE.Object3D | null,
): number {
  if (anchor === "galactic_center") {
    if (!galacticCenterRoot) {
      return camera.position.distanceTo(target);
    }
    galacticCenterRoot.getWorldPosition(_galacticCenterWorld);
    return camera.position.distanceTo(_galacticCenterWorld);
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

  private renderer!: WebGPURenderer;
  private outlinePipeline: ViewerOutlinePipeline | null = null;
  private renderReady = false;
  private readonly labelRenderer: CSS2DRenderer;
  private readonly scene: THREE.Scene;
  /** Scene graph rebased each frame so the observer sits at the origin (GPU float32 precision). */
  private readonly contentRoot: THREE.Group;
  private readonly camera: THREE.PerspectiveCamera;
  private readonly controls: OrbitControls;
  private readonly bodies = new Map<string, BodyEntry>();
  private readonly paths = new Map<string, PathEntry>();
  private readonly arrows = new Map<string, THREE.Group>();
  private readonly arrowFrames = new Map<string, ArrowFrame>();
  private readonly sunPointLight: THREE.PointLight;
  private readonly observerPosition = new THREE.Vector3();
  private readonly defaultCameraOffset = new THREE.Vector3(0, 0, 1);
  private readonly constellationSky = createConstellationSky(1);
  private arrowMeshLengthAu = 0;
  private arrowEarthRadiusAu = 0;
  private arrowLengthCameraFraction = 0.05;
  private readonly canvasHost: HTMLDivElement;
  private readonly captionEl: HTMLDivElement;
  private readonly fpsEl: HTMLDivElement;
  private resizeObserver: ResizeObserver | null = null;
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
  private lastPointingTarget: string | null = null;
  private cameraOrbitTween: CameraOrbitTweenHandle | null = null;
  private cameraOrbitTweenUntil = 0;
  private sceneSnapshot: SceneSnapshot | null = null;

  constructor(mount: HTMLElement) {
    this.root = document.createElement("div");
    this.root.className = "earth-sun-viewer";
    mount.appendChild(this.root);

    this.canvasHost = document.createElement("div");
    this.canvasHost.className = "earth-sun-viewer-canvas";
    this.root.appendChild(this.canvasHost);

    this.captionEl = document.createElement("div");
    this.captionEl.className = "earth-sun-viewer-caption";
    this.canvasHost.appendChild(this.captionEl);

    this.fpsEl = document.createElement("div");
    this.fpsEl.className = "earth-sun-viewer-fps";
    this.canvasHost.appendChild(this.fpsEl);

    this.scene = new THREE.Scene();
    this.contentRoot = new THREE.Group();
    this.contentRoot.name = "content_root";
    this.scene.add(this.constellationSky.root, this.contentRoot);

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.001, 1000);
    this.camera.position.set(0, 0, 1);

    this.renderer = new WebGPURenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.canvasHost.appendChild(this.renderer.domElement);

    this.labelRenderer = new CSS2DRenderer();
    this.labelRenderer.domElement.style.position = "absolute";
    this.labelRenderer.domElement.style.inset = "0";
    this.labelRenderer.domElement.style.pointerEvents = "none";
    this.canvasHost.appendChild(this.labelRenderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.target.set(0, 0, 0);
    this.controls.addEventListener("change", () => {
      this.cameraMotionUntil = performance.now() + 250;
      this.clampCameraDistance();
      this.syncClipPlanes();
      this.layoutArrows();
      this.updateLabels();
      this.labelRenderer.render(this.scene, this.camera);
    });

    const ambient = new THREE.AmbientLight(0xffffff, 0.12);
    this.sunPointLight = new THREE.PointLight(0xfff2cc, 2.5, 0, 2);
    this.contentRoot.add(ambient, this.sunPointLight);

    this.resizeObserver = new ResizeObserver(() => {
      this.onResize();
    });
    this.resizeObserver.observe(this.canvasHost);
    void this.bootstrapRenderer();
  }

  private async bootstrapRenderer(): Promise<void> {
    await this.renderer.init();
    this.outlinePipeline = createViewerOutlinePipeline(this.renderer, this.scene, this.camera);
    this.syncOutlineSelection();
    this.renderReady = true;
    this.onResize();
    this.animate();
  }

  setGeometryPollIntervalMs(intervalMs: number): void {
    this.geometryPollIntervalMs = intervalMs;
  }

  dispose(): void {
    cancelAnimationFrame(this.animationId);
    this.resizeObserver?.disconnect();
    this.resizeObserver = null;
    this.outlinePipeline?.dispose();
    this.outlinePipeline = null;
    this.renderer.dispose();
    this.root.remove();
  }

  applySnapshot(snapshot: SceneSnapshot): void {
    this.sceneSnapshot = snapshot;
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
    this.removeArrowsExcept(snapshot.arrows.map((arrow) => arrow.name));
    this.layoutArrows();
    this.syncClipPlanes();
    this.updateLabels();
    this.syncOrbitPivot();
    const pointingTarget =
      snapshot.pointing_target ?? snapshot.pointing_target_label ?? "";
    const targetChanged =
      this.hasInitialCamera &&
      this.lastPointingTarget !== null &&
      pointingTarget !== "" &&
      pointingTarget !== this.lastPointingTarget;
    this.lastPointingTarget = pointingTarget;
    if (!this.hasInitialCamera) {
      this.resetCamera();
      this.hasInitialCamera = true;
    } else if (targetChanged) {
      this.animateCameraForPointingTarget(snapshot);
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
      this.contentRoot.add(root);
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
        body.name === "earth"
          ? createEarthMesh(body.radius)
          : createSphereMesh(
              body.radius,
              body.color,
              SELF_LIT_BODY_EMISSIVE_INTENSITY[body.name] ?? 0,
            );
      entry.root.add(mesh);
      entry.radius = body.radius;
    }

    entry.root.matrixAutoUpdate = false;
    entry.root.matrix.copy(matrixFromSnapshot(body.matrix));
    entry.root.matrixWorldNeedsUpdate = true;
    if (entry.label) {
      entry.label.position.set(0, 0, body.radius * LABEL_OFFSET_BODY_RADII);
    }
    if (body.name === "sun") {
      this.sunPointLight.position.setFromMatrixPosition(entry.root.matrix);
    }
  }

  private updatePath(path: ScenePath): void {
    const signature = JSON.stringify(path.segments);
    const existing = this.paths.get(path.name);
    if (existing?.signature === signature) {
      return;
    }

    if (existing) {
      this.contentRoot.remove(existing.root);
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
      if (path.name.endsWith("_orbit") && points.length >= 3) {
        points.push(points[0].clone());
      }
      const geometry = new THREE.BufferGeometry().setFromPoints(points);
      const material = new THREE.LineBasicMaterial({
        color: rgbToThree(path.color),
        transparent: true,
        opacity: 0.95,
      });
      root.add(new THREE.Line(geometry, material));
    }
    this.contentRoot.add(root);
    this.paths.set(path.name, { root, signature });
  }

  private updateArrow(arrow: SceneArrow): void {
    this.arrowFrames.set(arrow.name, {
      base: new THREE.Vector3(...arrow.base),
      direction: new THREE.Vector3(...arrow.direction).normalize(),
      color: arrow.color,
      distanceAnchor: arrow.distance_anchor,
      displayLengthAu: -1,
      displayGapAu: -1,
    });
  }

  private removeArrowsExcept(names: string[]): void {
    const keep = new Set(names);
    for (const name of this.arrowFrames.keys()) {
      if (keep.has(name)) {
        continue;
      }
      this.arrowFrames.delete(name);
      const existing = this.arrows.get(name);
      if (!existing) {
        continue;
      }
      this.contentRoot.remove(existing);
      disposeSceneArrow(existing);
      this.arrows.delete(name);
    }
  }

  private arrowDisplayLengthAu(cameraDistanceAu: number): number {
    return cameraDistanceAu * this.arrowLengthCameraFraction * ARROW_LENGTH_BOOST;
  }

  private arrowShaftStartAu(frame: ArrowFrame, cameraDistanceAu: number): number {
    if (frame.distanceAnchor !== "observer") {
      return 0;
    }
    const observer = this.bodies.get("observer");
    if (!observer || observer.radius <= 0) {
      return 0;
    }
    return observerArrowShaftStartAu(
      observer.radius,
      cameraDistanceAu,
      this.defaultCameraDistance,
    );
  }

  private arrowWorldOrigin(frame: ArrowFrame, shaftStartAu: number): THREE.Vector3 {
    if (frame.distanceAnchor === "observer") {
      return this.observerPosition.clone().addScaledVector(frame.direction, shaftStartAu);
    }
    return frame.base.clone();
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
        this.bodies.get("galactic_center")?.root ?? null,
      );
      const shaftStartAu = this.arrowShaftStartAu(frame, cameraDistance);
      const origin = this.arrowWorldOrigin(frame, shaftStartAu);
      const displayLength = this.arrowDisplayLengthAu(cameraDistance);
      const lengthDelta = Math.abs(displayLength - frame.displayLengthAu);
      const gapDelta = Math.abs(shaftStartAu - frame.displayGapAu);
      const rebuildLength =
        lengthDelta > frame.displayLengthAu * 0.02 ||
        lengthDelta > this.arrowMeshLengthAu * 0.02;
      const repositionGap =
        gapDelta > Math.max(frame.displayGapAu * 0.02, this.arrowEarthRadiusAu * 1e-4);
      const existing = this.arrows.get(name);
      if (!rebuildLength && !repositionGap && existing) {
        continue;
      }
      frame.displayLengthAu = displayLength;
      frame.displayGapAu = shaftStartAu;

      if (rebuildLength || !existing) {
        if (existing) {
          this.contentRoot.remove(existing);
          disposeSceneArrow(existing);
        }

        const sizeScale = displayLength / this.arrowMeshLengthAu;
        const headLength = displayLength * ARROW_HEAD_LENGTH_FRACTION;
        const headRadius = ARROW_HEAD_RADIUS_EARTH_RADII * this.arrowEarthRadiusAu * sizeScale;
        const shaftRadius = ARROW_SHAFT_RADIUS_EARTH_RADII * this.arrowEarthRadiusAu * sizeScale;
        const arrow = createSceneArrow(
          frame.direction,
          origin,
          displayLength,
          rgbToThree(frame.color),
          headLength,
          headRadius,
          shaftRadius,
        );
        this.contentRoot.add(arrow);
        this.arrows.set(name, arrow);
      } else if (existing) {
        existing.position.copy(origin);
      }
    }
    this.syncOutlineSelection();
  }

  private syncOutlineSelection(): void {
    this.outlinePipeline?.syncArrowOutline(this.arrows.values());
    this.outlinePipeline?.syncBodyOutline(this.bodies);
  }

  private syncOrbitPivot(): void {
    const entry = this.bodies.get("observer");
    if (!entry) {
      return;
    }
    this.observerPosition.setFromMatrixPosition(entry.root.matrix);
    this.syncFloatingOrigin();
  }

  private syncFloatingOrigin(): void {
    this.contentRoot.position.copy(this.observerPosition).negate();
    this.contentRoot.updateMatrixWorld(true);
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
    this.scene.updateMatrixWorld(true);
    _cameraScreenDown.set(0, -1, 0).applyQuaternion(this.camera.quaternion);
    const hideWhenBehind = (label: CSS2DObject, visibleOpacity = 1): void => {
      const world = new THREE.Vector3();
      label.getWorldPosition(world);
      const projected = world.clone().project(this.camera);
      const behind = projected.z < -1 || projected.z > 1;
      label.element.style.opacity = behind ? "0" : String(visibleOpacity);
    };
    const bodyWorldPosition = (name: string): THREE.Vector3 | null => {
      const bodyEntry = this.bodies.get(name);
      if (!bodyEntry) {
        return null;
      }
      bodyEntry.root.getWorldPosition(_labelWorldAnchor);
      return _labelWorldAnchor;
    };
    for (const entry of this.bodies.values()) {
      const label = entry.label;
      if (!label) {
        continue;
      }
      entry.root.getWorldPosition(_labelWorldCenter);
      let labelOpacity = 1;
      const bodyName = entry.root.name;
      if (bodyName !== "earth" && this.sceneSnapshot) {
        const orbitRadius = heavenlyBodyOrbitRadiusAu(
          bodyName,
          this.sceneSnapshot,
          bodyWorldPosition,
        );
        if (orbitRadius !== null && orbitRadius > 0) {
          const cameraDistance = this.camera.position.distanceTo(_labelWorldCenter);
          labelOpacity = skyOpacityForCameraDistance(
            cameraDistance,
            orbitRadius,
            this.skyboxFadeOrbitMultiple,
            this.skyboxFadeSpanOrbitMultiple,
          );
        }
      }
      _labelWorldAnchor
        .copy(_labelWorldCenter)
        .addScaledVector(_cameraScreenDown, entry.radius * LABEL_OFFSET_BODY_RADII);
      entry.root.worldToLocal(_labelWorldAnchor);
      label.position.copy(_labelWorldAnchor);
      hideWhenBehind(label, labelOpacity);
    }
    for (const label of this.constellationSky.labels) {
      hideWhenBehind(label, this.skyOpacity);
    }
  }

  private onResize = (): void => {
    const width = this.canvasHost.clientWidth;
    const height = this.canvasHost.clientHeight;
    if (width <= 0 || height <= 0) {
      return;
    }
    this.camera.aspect = width / Math.max(height, 1);
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
    this.outlinePipeline?.setSize(
      Math.max(1, Math.floor(width * this.renderer.getPixelRatio())),
      Math.max(1, Math.floor(height * this.renderer.getPixelRatio())),
    );
    this.labelRenderer.setSize(width, height);

    if (this.camera.position.lengthSq() < 1e-12) {
      this.camera.position.set(0, 0, this.defaultCameraDistance || 1);
      this.controls.update();
    }
  };

  private renderIntervalMs(now: number): number {
    if (now < this.cameraMotionUntil || now < this.cameraOrbitTweenUntil) {
      return VIEWER_FRAME_MS;
    }
    return this.geometryPollIntervalMs;
  }

  private animate = (): void => {
    this.animationId = requestAnimationFrame(this.animate);
    updateCameraTargetTweens(performance.now());
    if (!cameraOrbitTweenActive()) {
      this.controls.update();
    }
    this.updateLabels();
    this.labelRenderer.render(this.scene, this.camera);
    const now = performance.now();
    if (now - this.lastRenderTime < this.renderIntervalMs(now)) {
      return;
    }
    this.lastRenderTime = now;
    if (!this.renderReady || this.outlinePipeline === null) {
      return;
    }
    this.syncFloatingOrigin();
    this.layoutArrows();
    this.syncClipPlanes();
    this.outlinePipeline.prepareFrame();
    this.outlinePipeline.renderPipeline.render();
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
    const distance = this.defaultCameraDistance || 1;
    this.defaultCameraOffset.set(0, 0, distance);
    this.controls.target.set(0, 0, 0);
    this.camera.position.copy(this.defaultCameraOffset);
    this.controls.update();
  }

  private bodyWorldPosition(name: string): THREE.Vector3 | null {
    const entry = this.bodies.get(name);
    if (!entry) {
      return null;
    }
    return new THREE.Vector3().setFromMatrixPosition(entry.root.matrix);
  }

  private animateCameraForPointingTarget(snapshot: SceneSnapshot): void {
    this.cameraOrbitTween?.stop();
    this.cameraOrbitTween = null;

    const currentOffset = this.camera.position.clone();
    const currentOffsetDirection =
      currentOffset.lengthSq() > 1e-20
        ? currentOffset.clone().normalize()
        : this.defaultCameraOffset.clone().normalize();
    const startPose = cameraPoseFromState(this.camera, _orbitPivot);
    const endPose = desiredOrbitTargetCameraPose(
      this.camera,
      snapshot,
      this.observerPosition,
      (name) => this.bodyWorldPosition(name),
      currentOffsetDirection,
      this.controls.maxDistance,
    );
    this.cameraOrbitTweenUntil = performance.now() + 1200;
    this.cameraOrbitTween = tweenCameraToOrbitPose(
      this.camera,
      this.controls,
      _orbitPivot,
      startPose,
      endPose,
      () => {
        this.cameraMotionUntil = performance.now() + 250;
        this.clampCameraDistance();
        this.syncClipPlanes();
        this.layoutArrows();
      },
    );
  }
}
