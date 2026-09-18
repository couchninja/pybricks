import type { SceneArrow, SceneSnapshot } from "./scene-types";

export function pointingArrowName(pointingTarget: string): string {
  return `pointing_arrow_${pointingTarget}`;
}

export function isPointingArrowName(name: string): boolean {
  return name.startsWith("pointing_arrow_");
}

export function pointingArrowForTarget(snapshot: SceneSnapshot, pointingTarget: string): SceneArrow | undefined {
  return snapshot.arrows.find((entry) => entry.name === pointingArrowName(pointingTarget));
}
