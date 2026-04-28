/**
 * POI candidate filters for mapoi WebUI.
 *
 * landmark system tag (#85) は Nav2 navigation goal にしない reference 専用。
 * goal candidate / route waypoint candidate / initial pose candidate の dropdown
 * から landmark POI を除外する純関数群を集約する。
 *
 * Browser からは `MapoiPoiFilter.*` のグローバルとして使い、
 * Node (vitest) からは `module.exports` 経由で import する dual export 形式。
 * 依存ゼロの pure 関数で DOM / window 等を参照しない。
 */
(function () {
  'use strict';

  function hasLandmarkTag(poi) {
    const tags = (poi && poi.tags) || [];
    return tags.some((t) => typeof t === 'string' && t.toLowerCase() === 'landmark');
  }

  function filterGoalCandidates(pois) {
    return (pois || []).filter((p) => {
      const tags = (p && p.tags ? p.tags : []).map((t) => String(t).toLowerCase());
      return tags.includes('goal') && !tags.includes('landmark');
    });
  }

  function filterInitialPoseCandidates(pois) {
    return (pois || []).filter((p) => !hasLandmarkTag(p));
  }

  function filterRouteWaypointCandidates(pois) {
    return (pois || []).filter((p) => !hasLandmarkTag(p));
  }

  const api = {
    hasLandmarkTag,
    filterGoalCandidates,
    filterInitialPoseCandidates,
    filterRouteWaypointCandidates,
  };

  if (typeof window !== 'undefined') {
    window.MapoiPoiFilter = window.MapoiPoiFilter || {};
    Object.assign(window.MapoiPoiFilter, api);
  }
  if (typeof module !== 'undefined' && module.exports) {
    module.exports = api;
  }
}());
