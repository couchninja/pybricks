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

export type KeplerianOrbitSnapshot = {
  kind: "keplerian";
  name: string;
  color: Rgb;
  matrix: number[];
  epoch_iso: string;
  period_days: number;
  a_au: number;
  e: number;
  i_rad: number;
  raan_rad: number;
  argp_rad: number;
  M0_rad: number;
  mu_au3_per_day2: number;
  origin_body: string | null;
  /** Earth heliocentric ecliptic AU; geocentric orbits only (solar-system path space). */
  origin_heliocentric_au?: [number, number, number];
};

export type TleOrbitSnapshot = {
  kind: "tle";
  name: string;
  color: Rgb;
  matrix: number[];
  line1: string;
  line2: string;
  epoch_iso: string;
  period_s: number;
  origin_body: string | null;
  eci_to_ecliptic: number[];
  origin_heliocentric_au?: [number, number, number];
};

export type ParametricOrbit = KeplerianOrbitSnapshot | TleOrbitSnapshot;

export type ArrowDistanceAnchor = "earth_center" | "observer" | "galactic_center";

export type SceneArrow = {
  name: string;
  color: Rgb;
  base: [number, number, number];
  direction: [number, number, number];
  distance_anchor: ArrowDistanceAnchor;
};

export type SceneSnapshot = {
  simulation_time_iso: string;
  /** Maps mean ecliptic directions into the viewer root frame (column-major 3×3). */
  inertial_to_root_rotation: number[];
  bodies: SceneBody[];
  paths: ScenePath[];
  parametric_orbits: ParametricOrbit[];
  arrows: SceneArrow[];
};
