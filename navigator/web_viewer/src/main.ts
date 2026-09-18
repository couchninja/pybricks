import { EarthSunViewer } from "./earth-sun-viewer";
import type { SceneSnapshot } from "./scene-types";

const SCENE_POLL_REALTIME_MS = 10_000;
const SCENE_POLL_FAST_MS = 1000 / 30;

type TimeScaleStatus = {
  preset: string | null;
  time_iso?: string;
};

type NavigatorWebStatus = {
  target: string;
  time_scale: TimeScaleStatus;
};

function scenePollIntervalMs(preset: string | null): number {
  return preset === "realtime" ? SCENE_POLL_REALTIME_MS : SCENE_POLL_FAST_MS;
}

function mountStyles(): void {
  const style = document.createElement("style");
  style.textContent = `
    .earth-sun-viewer {
      position: relative;
      width: 100%;
      flex: 1 1 auto;
      min-height: 0;
      height: 100%;
      border-radius: 12px;
      overflow: hidden;
      background: #0a0e12;
    }
    .earth-sun-viewer-canvas {
      position: relative;
      width: 100%;
      height: 100%;
      min-height: 240px;
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

function navigatorStatusFromDetail(detail: unknown): NavigatorWebStatus | null {
  if (detail === null || typeof detail !== "object") {
    return null;
  }
  const record = detail as Record<string, unknown>;
  const target = record.target;
  if (typeof target !== "string") {
    return null;
  }
  const timeScaleRaw = record.time_scale;
  if (timeScaleRaw === null || typeof timeScaleRaw !== "object" || !("preset" in timeScaleRaw)) {
    return null;
  }
  const timeScale = timeScaleRaw as TimeScaleStatus;
  const preset = timeScale.preset;
  if (preset !== null && typeof preset !== "string") {
    return null;
  }
  const timeIso = timeScale.time_iso;
  if (timeIso !== undefined && typeof timeIso !== "string") {
    return null;
  }
  return { target, time_scale: { preset, time_iso: timeIso } };
}

export function startEarthSunViewer(mount: HTMLElement): EarthSunViewer {
  mountStyles();
  const viewer = new EarthSunViewer(mount);
  viewer.setGeometryPollIntervalMs(SCENE_POLL_FAST_MS);

  let scenePollMs = SCENE_POLL_FAST_MS;
  let pollTimer: ReturnType<typeof setTimeout> | null = null;
  let inFlight = false;

  const applyScenePollMs = (nextMs: number): void => {
    if (nextMs === scenePollMs) {
      return;
    }
    scenePollMs = nextMs;
    viewer.setGeometryPollIntervalMs(nextMs);
    if (pollTimer !== null) {
      window.clearTimeout(pollTimer);
      pollTimer = null;
      scheduleNextScenePoll();
    }
  };

  const applyNavigatorStatus = (detail: unknown): void => {
    const status = navigatorStatusFromDetail(detail);
    if (status === null) {
      return;
    }
    applyScenePollMs(scenePollIntervalMs(status.time_scale.preset));
    viewer.applyNavigatorStatus({
      target: status.target,
      timeIso: status.time_scale.time_iso ?? null,
    });
  };

  const pollScene = async (): Promise<void> => {
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

  const scheduleNextScenePoll = (): void => {
    pollTimer = window.setTimeout(() => {
      pollTimer = null;
      void pollScene().finally(() => {
        scheduleNextScenePoll();
      });
    }, scenePollMs);
  };

  window.addEventListener("navigator-status", (event) => {
    applyNavigatorStatus((event as CustomEvent).detail);
  });
  applyNavigatorStatus((window as Window & { __navigatorLastStatus?: unknown }).__navigatorLastStatus);

  void pollScene().finally(() => {
    scheduleNextScenePoll();
  });

  return viewer;
}

const mount = document.getElementById("viewer-root");
if (mount) {
  startEarthSunViewer(mount);
}
