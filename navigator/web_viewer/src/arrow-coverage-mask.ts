/**
 * Screen-space mask of arrow geometry for body-outline suppression.
 * See {@link ./viewer-outline-pipeline.ts} for why this exists and how it is composited.
 */
// @ts-nocheck
import { Vector2 } from "three";
import {
  Mesh,
  NodeMaterial,
  Object3D,
  RenderTarget,
  Sprite,
  SpriteNodeMaterial,
  RendererUtils,
} from "three/webgpu";
import { color, screenUV, texture, saturate } from "three/tsl";

const _size = /* @__PURE__ */ new Vector2();

let _rendererState;

export type ArrowCoverageMask = {
  coverageNode: unknown;
  setSize: (width: number, height: number) => void;
  render: (
    renderer: import("three/webgpu").WebGPURenderer,
    scene: import("three").Scene,
    camera: import("three").Camera,
    arrowMeshes: Object3D[],
  ) => void;
  dispose: () => void;
};

export function createArrowCoverageMask(): ArrowCoverageMask {
  const renderTargetMask = new RenderTarget(undefined, undefined, { depthBuffer: false });

  const arrowMaskMaterial = new NodeMaterial();
  arrowMaskMaterial.colorNode = color(1, 1, 1);
  arrowMaskMaterial.depthTest = false;
  arrowMaskMaterial.depthWrite = false;

  const arrowMaskSpriteMaterial = new SpriteNodeMaterial();
  arrowMaskSpriteMaterial.colorNode = color(1, 1, 1);
  arrowMaskSpriteMaterial.depthTest = false;
  arrowMaskSpriteMaterial.depthWrite = false;

  const maskTextureUniform = texture(renderTargetMask.texture);
  const coverageNode = saturate(maskTextureUniform.sample(screenUV).r);
  const selectionCache = new Set();

  const setSize = (width, height) => {
    renderTargetMask.setSize(width, height);
  };

  const collectArrowMeshes = (arrowMeshes) => {
    selectionCache.clear();
    for (const selectedObject of arrowMeshes) {
      selectedObject.traverse((object) => {
        if (object instanceof Mesh || object instanceof Sprite) {
          selectionCache.add(object);
        }
      });
    }
  };

  const render = (renderer, scene, camera, arrowMeshes) => {
    collectArrowMeshes(arrowMeshes);
    if (selectionCache.size === 0) {
      _rendererState = RendererUtils.resetRendererState(renderer, _rendererState);
      renderer.setRenderTarget(renderTargetMask);
      renderer.setClearColor(0x000000, 0);
      renderer.clear();
      RendererUtils.restoreRendererState(renderer, _rendererState);
      return;
    }

    _rendererState = RendererUtils.resetRendererAndSceneState(renderer, scene, _rendererState);

    renderer.getDrawingBufferSize(_size);
    setSize(_size.width, _size.height);

    renderer.setRenderTarget(renderTargetMask);
    renderer.setClearColor(0x000000, 1);
    renderer.clear();

    renderer.setRenderObjectFunction((object, renderScene, renderCamera, geometry, _material, group, lightsNode, clippingContext) => {
      if (!selectionCache.has(object)) {
        return;
      }
      const overrideMaterial = object instanceof Sprite ? arrowMaskSpriteMaterial : arrowMaskMaterial;
      renderer.renderObject(
        object,
        renderScene,
        renderCamera,
        geometry,
        overrideMaterial,
        group,
        lightsNode,
        clippingContext,
      );
    });
    renderer.render(scene, camera);

    renderer.setRenderObjectFunction(_rendererState.renderObjectFunction);

    selectionCache.clear();
    RendererUtils.restoreRendererAndSceneState(renderer, scene, _rendererState);
  };

  return {
    coverageNode,
    setSize,
    render,
    dispose: () => {
      renderTargetMask.dispose();
      arrowMaskMaterial.dispose();
      arrowMaskSpriteMaterial.dispose();
    },
  };
}
