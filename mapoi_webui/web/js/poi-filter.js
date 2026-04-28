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

  /**
   * POI form の tolerance round-trip を保つ純関数 (#87)。
   * `||` だと xy=0 / yaw=0 の意図的な値も default で上書きされるので
   * Number.isFinite ベースで判定する (Codex review #137 medium 対応)。
   *
   * @param {string|number} rawXyValue input value から読んだ生 (string for `<input>`)
   * @param {number|undefined} originalYaw fillForm 時の yaw 元値 (UI 入力欄が無いため保持専用)
   * @returns {{xy: number, yaw: number}}
   */
  function parseTolerance(rawXyValue, originalYaw) {
    const xyVal = parseFloat(rawXyValue);
    const xy = Number.isFinite(xyVal) ? xyVal : 0.5;
    const yaw = Number.isFinite(originalYaw) ? originalYaw : 0.0;
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
  };

  if (typeof window !== 'undefined') {
    window.MapoiPoiFilter = window.MapoiPoiFilter || {};
    Object.assign(window.MapoiPoiFilter, api);
  }
  if (typeof module !== 'undefined' && module.exports) {
    module.exports = api;
  }
}());
