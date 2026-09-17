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

export type ArrowDistanceAnchor = "earth_center" | "observer" | "galactic_center";

export type SceneArrow = {
  name: string;
  color: Rgb;
  base: [number, number, number];
  direction: [number, number, number];
  distance_anchor: ArrowDistanceAnchor;
};

export type SceneSnapshot = {
  time_iso: string;
  pointing_target: string;
  pointing_target_label: string;
  scene_scale: number;
  camera_distance_au: number;
  default_camera_distance_au: number;
  earth_radius_au: number;
  earth_orbit_radius_au: number;
  skybox_fade_camera_distance_orbit_multiple: number;
  skybox_fade_span_orbit_radius_multiple: number;
  arrow_mesh_length_au: number;
  arrow_length_camera_distance_fraction: number;
  z_near: number;
  z_far: number;
  /** Maps mean ecliptic directions into the viewer root frame (column-major 3×3). */
  inertial_to_root_rotation: number[];
  /** Galactic orbit diameter; upper bound for camera distance from the orbit pivot. */
  milky_way_diameter_au: number;
  bodies: SceneBody[];
  paths: ScenePath[];
  arrows: SceneArrow[];
};
