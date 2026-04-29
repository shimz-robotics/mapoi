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

  // tolerance 最小値 (msg spec): xy = 1 mm、yaw = 約 0.057° (#138)。
  // 0 / 負値 は「無反応 POI」を許してしまうため明示的に禁止。
  const TOLERANCE_MIN = 0.001;

  function degToRad(deg) {
    const v = parseFloat(deg);
    return Number.isFinite(v) ? v * Math.PI / 180.0 : NaN;
  }

  function radToDeg(rad) {
    const v = parseFloat(rad);
    return Number.isFinite(v) ? v * 180.0 / Math.PI : NaN;
  }

  /**
   * POI form の tolerance round-trip を保つ純関数 (#87 / #138)。
   * `||` だと意図的な小さい値も default で上書きされるので Number.isFinite で判定。
   * 最終 xy / yaw は msg spec に従い `>= TOLERANCE_MIN` を保証する (UI HTML min を
   * bypass する経路に対する防御)。
   *
   * @param {string|number} rawXyValueM xy input 値 (m)
   * @param {string|number} rawYawValueRad yaw input から deg→rad 変換済の値
   * @returns {{xy: number, yaw: number}} 両方とも >= TOLERANCE_MIN
   */
  function parseTolerance(rawXyValueM, rawYawValueRad) {
    const xyParsed = parseFloat(rawXyValueM);
    const yawParsed = parseFloat(rawYawValueRad);
    const xy = Number.isFinite(xyParsed) && xyParsed >= TOLERANCE_MIN ? xyParsed : TOLERANCE_MIN;
    const yaw = Number.isFinite(yawParsed) && yawParsed >= TOLERANCE_MIN ? yawParsed : TOLERANCE_MIN;
    return { xy, yaw };
  }

  /**
   * POI tag combination 排他 check (#85)。
   * `goal+landmark` / `initial_pose+landmark` は仕様上排他。save 前に
   * 弾くことで mapoi_config.yaml に invalid な POI が永続化するのを防ぐ。
   *
   * @returns {{ ok: boolean, error?: string }}
   */
  function validatePoiTags(poi) {
    const tags = (poi && poi.tags ? poi.tags : []).map((t) => String(t).toLowerCase());
    const has = (n) => tags.includes(n);
    if (has('goal') && has('landmark')) {
      return {
        ok: false,
        error: '"goal" と "landmark" は併用できません (landmark は Nav2 navigation goal 不可な reference 専用)。',
      };
    }
    if (has('initial_pose') && has('landmark')) {
      return {
        ok: false,
        error: '"initial_pose" と "landmark" は併用できません (landmark は到達不可な reference 点)。',
      };
    }
    return { ok: true };
  }

  const api = {
    hasLandmarkTag,
    filterGoalCandidates,
    filterInitialPoseCandidates,
    filterRouteWaypointCandidates,
    validatePoiTags,
    parseTolerance,
    degToRad,
    radToDeg,
    TOLERANCE_MIN,
  };

  if (typeof window !== 'undefined') {
    window.MapoiPoiFilter = window.MapoiPoiFilter || {};
    Object.assign(window.MapoiPoiFilter, api);
  }
  if (typeof module !== 'undefined' && module.exports) {
    module.exports = api;
  }
}());
