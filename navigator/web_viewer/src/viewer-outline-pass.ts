import { Color, Mesh, Object3D, type Camera, type Group, type Scene } from "three";
import type OutlineNode from "three/addons/tsl/display/OutlineNode.js";
import { RenderPipeline, WebGPURenderer } from "three/webgpu";
import { float, pass, uniform } from "three/tsl";
import { outline } from "three/addons/tsl/display/OutlineNode.js";

const ARROW_OUTLINE_EDGE_STRENGTH = 1;
const ARROW_OUTLINE_EDGE_THICKNESS = 0.4;

const BODY_OUTLINE_EDGE_STRENGTH = 1;
const BODY_OUTLINE_EDGE_THICKNESS = 0.4;

export const BODY_OUTLINE_NAMES = [
  "earth",
  "iss",
  "moon",
  "sun",
  "galactic_center",
] as const;

export type ViewerOutlinePipeline = {
  renderPipeline: RenderPipeline;
  syncArrowOutline: (arrowGroups: Iterable<Group>) => void;
  syncBodyOutline: (bodies: ReadonlyMap<string, { root: Object3D }>) => void;
  setSize: (width: number, height: number) => void;
  dispose: () => void;
};

function createOutlinePass(
  scene: Scene,
  camera: Camera,
  edgeThickness: number,
): OutlineNode {
  return outline(scene, camera, {
    selectedObjects: [],
    edgeThickness: float(edgeThickness),
    edgeGlow: float(0),
    downSampleRatio: 1,
  });
}

function outlineColorForPass(outlinePass: OutlineNode, edgeStrength: number) {
  const visibleEdgeColor = uniform(new Color(0xffffff));
  const strength = uniform(edgeStrength);
  const edgeMask = outlinePass.visibleEdge.add(outlinePass.hiddenEdge);
  return edgeMask.mul(visibleEdgeColor).mul(strength);
}

export function createViewerOutlinePipeline(
  renderer: WebGPURenderer,
  scene: Scene,
  camera: Camera,
): ViewerOutlinePipeline {
  const arrowOutlinePass = createOutlinePass(scene, camera, ARROW_OUTLINE_EDGE_THICKNESS);
  const bodyOutlinePass = createOutlinePass(scene, camera, BODY_OUTLINE_EDGE_THICKNESS);

  const scenePass = pass(scene, camera);
  const renderPipeline = new RenderPipeline(renderer);
  renderPipeline.outputNode = outlineColorForPass(arrowOutlinePass, ARROW_OUTLINE_EDGE_STRENGTH)
    .add(outlineColorForPass(bodyOutlinePass, BODY_OUTLINE_EDGE_STRENGTH))
    .add(scenePass);
  renderPipeline.needsUpdate = true;

  const syncArrowOutline = (arrowGroups: Iterable<Group>): void => {
    const next: Object3D[] = [];
    for (const group of arrowGroups) {
      for (const child of group.children) {
        if (child instanceof Mesh && child.userData.arrowOutline === true) {
          next.push(child);
        }
      }
    }
    arrowOutlinePass.selectedObjects = next;
  };

  const syncBodyOutline = (bodies: ReadonlyMap<string, { root: Object3D }>): void => {
    const next: Object3D[] = [];
    for (const name of BODY_OUTLINE_NAMES) {
      const entry = bodies.get(name);
      if (!entry) {
        continue;
      }
      entry.root.traverse((child) => {
        if (child instanceof Mesh) {
          next.push(child);
        }
      });
    }
    bodyOutlinePass.selectedObjects = next;
  };

  return {
    renderPipeline,
    syncArrowOutline,
    syncBodyOutline,
    setSize: (width: number, height: number) => {
      arrowOutlinePass.setSize(width, height);
      bodyOutlinePass.setSize(width, height);
    },
    dispose: () => {
      arrowOutlinePass.dispose();
      bodyOutlinePass.dispose();
      renderPipeline.dispose();
    },
  };
}
