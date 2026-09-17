import { EarthSunViewer } from "./earth-sun-viewer";
import type { SceneSnapshot } from "./scene-types";

const SCENE_POLL_MS = 1000 / 30;

function mountStyles(): void {
  const style = document.createElement("style");
  style.textContent = `
    .earth-sun-viewer {
      position: relative;
      width: 100%;
      min-height: 42vh;
      border-radius: 12px;
      overflow: hidden;
      background: #0a0e12;
      margin-bottom: 1.25rem;
    }
    .earth-sun-viewer-canvas {
      position: relative;
      width: 100%;
      height: min(52vh, 520px);
    }
    .earth-sun-viewer-canvas canvas {
      display: block;
      width: 100% !important;
      height: 100% !important;
    }
    .earth-sun-viewer-caption {
      position: absolute;
      top: 8px;
      left: 50%;
      transform: translateX(-50%);
      padding: 4px 10px;
      border-radius: 6px;
      background: rgba(15, 20, 25, 0.72);
      color: #e8eef4;
      font: 600 13px system-ui, sans-serif;
      pointer-events: none;
      z-index: 2;
    }
    .earth-sun-viewer-fps {
      position: absolute;
      top: 8px;
      left: 8px;
      color: rgba(255, 255, 255, 0.78);
      font: 600 13px system-ui, sans-serif;
      pointer-events: none;
      z-index: 2;
    }
    .earth-sun-viewer-pointing-target {
      position: absolute;
      right: 8px;
      bottom: 8px;
      min-width: 180px;
      padding: 8px 12px;
      border-radius: 8px;
      border: 1px solid rgba(80, 220, 255, 0.9);
      background: rgba(40, 44, 52, 0.88);
      color: rgb(80, 220, 255);
      font: 600 14px system-ui, sans-serif;
      text-align: center;
      pointer-events: none;
      z-index: 2;
    }
  `;
  document.head.appendChild(style);
}

async function fetchScene(): Promise<SceneSnapshot> {
  const response = await fetch("/api/scene");
  if (!response.ok) {
    throw new Error(`scene fetch failed: ${response.status}`);
  }
  return response.json() as Promise<SceneSnapshot>;
}

export function startEarthSunViewer(mount: HTMLElement): EarthSunViewer {
  mountStyles();
  const viewer = new EarthSunViewer(mount);

  let inFlight = false;
  const poll = async (): Promise<void> => {
    if (inFlight) {
      return;
    }
    inFlight = true;
    try {
      const snapshot = await fetchScene();
      viewer.applySnapshot(snapshot);
    } catch {
      // Keep the last good frame; the status panel still reports navigator state.
    } finally {
      inFlight = false;
    }
  };

  void poll();
  window.setInterval(() => {
    void poll();
  }, SCENE_POLL_MS);

  return viewer;
}

const mount = document.getElementById("viewer-root");
if (mount) {
  startEarthSunViewer(mount);
}
