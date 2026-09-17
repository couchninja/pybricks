export type Rgb = [number, number, number];

export type SceneBody = {
  name: string;
  label: string;
  radius: number;
  color: Rgb;
  matrix: number[];
};

export type ScenePath = {
  name: string;
  color: Rgb;
  segments: number[][][];
};

export type ArrowDistanceAnchor = "earth_center" | "galactic_center";

export type SceneArrow = {
  name: string;
  color: Rgb;
  base: [number, number, number];
  direction: [number, number, number];
  distance_anchor: ArrowDistanceAnchor;
};

export type SceneSnapshot = {
  time_iso: string;
  pointing_target_label: string;
  scene_scale: number;
  camera_distance_au: number;
  default_camera_distance_au: number;
  earth_radius_au: number;
  arrow_mesh_length_au: number;
  arrow_length_camera_distance_fraction: number;
  arrow_min_length_au: number;
  z_near: number;
  z_far: number;
  bodies: SceneBody[];
  paths: ScenePath[];
  arrows: SceneArrow[];
};
