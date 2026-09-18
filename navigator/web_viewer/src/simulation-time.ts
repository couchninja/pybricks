/** Parse Astropy-style UTC ISO timestamps from scene snapshots. */
export function simulationTimeMsFromIso(iso: string): number {
  const normalized = iso.includes("T") ? iso : iso.replace(" ", "T");
  const utcIso = normalized.endsWith("Z") ? normalized : `${normalized}Z`;
  return Date.parse(utcIso);
}
