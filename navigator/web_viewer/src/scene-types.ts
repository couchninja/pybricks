export type Rgb = [number, number, number];

export type SceneBody = {
  name: string;
  radius: number;
  color: Rgb;
  matrix: number[];
};

export type ScenePath = {
  name: string;
  color: Rgb;
  matrix: number[];
  segments: number[][][];
};

export type ArrowDistanceAnchor = "earth_center" | "observer" | "galactic_center";

export type SceneArrow = {
  name: string;
  color: Rgb;
  base: [number, number, number];
  direction: [number, number, number];
  distance_anchor: ArrowDistanceAnchor;
};

export type SceneSnapshot = {
  /** Maps mean ecliptic directions into the viewer root frame (column-major 3×3). */
  inertial_to_root_rotation: number[];
  bodies: SceneBody[];
  paths: ScenePath[];
  arrows: SceneArrow[];
};
