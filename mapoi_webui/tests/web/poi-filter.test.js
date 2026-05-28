// vitest unit tests for the POI candidate filters in
// mapoi_webui/web/js/poi-filter.js (#85 / #142).
//
// 対象: MapoiPoiFilter.{hasLandmarkTag, filterWaypointCandidates,
//   filterInitialPoseCandidates, filterRouteWaypointCandidates}
//
// テスト方針:
// - landmark system tag を持つ POI が dropdown 候補から確実に除外されること
//   を検証する (#85: landmark = Nav2 navigation 不可な reference 専用)。
// - waypoint system tag (#142 で goal からリネーム) が Nav2 navigation candidate を表す。
// - DOM / window 依存ゼロの pure JS。

import { describe, it, expect } from 'vitest';
import * as filter from '../../web/js/poi-filter.js';

const {
  hasLandmarkTag,
  filterWaypointCandidates,
  filterInitialPoseCandidates,
  filterRouteWaypointCandidates,
  validatePoiTags,
  parseTolerance,
  degToRad,
  radToDeg,
  formatToleranceYawDeg,
  roundTo,
  TOLERANCE_MIN,
  POSE_XY_DIGITS,
  POSE_YAW_DIGITS,
  TOLERANCE_XY_DIGITS,
  TOLERANCE_YAW_DIGITS,
} = filter;

const poi = (name, tags) => ({ name, tags });

describe('hasLandmarkTag', () => {
  it('returns true when tags include landmark', () => {
    expect(hasLandmarkTag(poi('a', ['landmark']))).toBe(true);
    expect(hasLandmarkTag(poi('b', ['event', 'landmark', 'hazard']))).toBe(true);
  });

  it('returns false when landmark is absent', () => {
    expect(hasLandmarkTag(poi('a', ['waypoint']))).toBe(false);
    expect(hasLandmarkTag(poi('b', []))).toBe(false);
    expect(hasLandmarkTag(poi('c', ['waypoint', 'pause']))).toBe(false);
  });

  it('matches case-insensitively', () => {
    expect(hasLandmarkTag(poi('a', ['LandMark']))).toBe(true);
    expect(hasLandmarkTag(poi('b', ['LANDMARK']))).toBe(true);
  });

  it('handles missing / null input gracefully', () => {
    expect(hasLandmarkTag(null)).toBe(false);
    expect(hasLandmarkTag(undefined)).toBe(false);
    expect(hasLandmarkTag({})).toBe(false);
    expect(hasLandmarkTag({ tags: null })).toBe(false);
  });
});

describe('filterWaypointCandidates', () => {
  it('keeps waypoint POIs without landmark', () => {
    const pois = [
      poi('north_goal', ['waypoint']),
      poi('checkpoint', ['waypoint', 'checkpoint']),
    ];
    expect(filterWaypointCandidates(pois).map((p) => p.name)).toEqual(['north_goal', 'checkpoint']);
  });

  it('drops POIs that combine waypoint with landmark', () => {
    // waypoint+landmark は仕様上排他だが万一 config に混入しても候補に載せない。
    const pois = [
      poi('valid', ['waypoint']),
      poi('reference', ['waypoint', 'landmark']),
    ];
    expect(filterWaypointCandidates(pois).map((p) => p.name)).toEqual(['valid']);
  });

  it('drops landmark-only POIs (no waypoint tag)', () => {
    const pois = [
      poi('landmark_only', ['landmark']),
      poi('hazard_landmark', ['event', 'landmark', 'hazard']),
    ];
    expect(filterWaypointCandidates(pois)).toEqual([]);
  });

  it('returns [] for empty / null input', () => {
    expect(filterWaypointCandidates([])).toEqual([]);
    expect(filterWaypointCandidates(null)).toEqual([]);
    expect(filterWaypointCandidates(undefined)).toEqual([]);
  });
});

describe('filterInitialPoseCandidates', () => {
  it('drops landmark POIs from initial pose candidate list', () => {
    // landmark は到達不可なので initial pose には設定できない (#144 では initial_pose tag は
    // 廃止したが、landmark POI を initial pose として publish させない原則は維持)。
    const pois = [
      poi('start_zone', ['waypoint']),
      poi('landmark_x', ['landmark']),
      poi('hazard', ['event', 'landmark', 'hazard']),
      poi('north', ['waypoint']),
    ];
    expect(filterInitialPoseCandidates(pois).map((p) => p.name)).toEqual(['start_zone', 'north']);
  });

  it('keeps any non-landmark POI as initial pose candidate', () => {
    // initial pose dropdown は全 navigable POI を載せて user に選ばせる (manual override 用途)。
    const pois = [
      poi('a', ['waypoint']),
      poi('b', ['pause']),
    ];
    expect(filterInitialPoseCandidates(pois).map((p) => p.name)).toEqual(['a', 'b']);
  });
});

describe('filterRouteWaypointCandidates', () => {
  it('drops landmark POIs from route waypoint candidate list', () => {
    const pois = [
      poi('start_zone', ['waypoint']),
      poi('hazard', ['event', 'landmark', 'hazard']),
      poi('north_goal', ['waypoint']),
    ];
    expect(filterRouteWaypointCandidates(pois).map((p) => p.name)).toEqual(['start_zone', 'north_goal']);
  });

  it('returns input copy for null / undefined / empty', () => {
    expect(filterRouteWaypointCandidates(null)).toEqual([]);
    expect(filterRouteWaypointCandidates(undefined)).toEqual([]);
    expect(filterRouteWaypointCandidates([])).toEqual([]);
  });
});

describe('degToRad / radToDeg', () => {
  it('converts known angles', () => {
    expect(degToRad(0)).toBeCloseTo(0, 6);
    expect(degToRad(45)).toBeCloseTo(Math.PI / 4, 6);
    expect(degToRad(90)).toBeCloseTo(Math.PI / 2, 6);
    expect(degToRad(180)).toBeCloseTo(Math.PI, 6);
    expect(radToDeg(Math.PI / 4)).toBeCloseTo(45, 6);
    expect(radToDeg(Math.PI)).toBeCloseTo(180, 6);
  });

  it('parses string input alongside numeric', () => {
    expect(degToRad('45')).toBeCloseTo(Math.PI / 4, 6);
    expect(radToDeg('3.14159265')).toBeCloseTo(180, 4);
  });

  it('returns NaN for non-numeric input', () => {
    expect(Number.isNaN(degToRad(''))).toBe(true);
    expect(Number.isNaN(degToRad('abc'))).toBe(true);
    expect(Number.isNaN(radToDeg(NaN))).toBe(true);
  });

  it('round-trips deg → rad → deg (#138)', () => {
    [0.1, 1, 45, 90, 180, 359].forEach((deg) => {
      expect(radToDeg(degToRad(deg))).toBeCloseTo(deg, 6);
    });
  });
});

describe('formatToleranceYawDeg (#267)', () => {
  // editor は rad を保存し度数で表示する。保存値は roundTo(degToRad(deg), 4) なので、
  // 区切りの良い度数は綺麗に表示され編集なし save でも churn しないことを pin する。
  it('displays round degrees cleanly (integer, no trailing zeros)', () => {
    [30, 45, 90, 120, 180].forEach((deg) => {
      const storedRad = roundTo(degToRad(deg), TOLERANCE_YAW_DIGITS);
      expect(formatToleranceYawDeg(storedRad)).toBe(deg);
    });
  });

  it('treats yaw 不問 (π / 3.1416) as 180', () => {
    expect(formatToleranceYawDeg(3.1416)).toBe(180);
    expect(formatToleranceYawDeg(Math.PI)).toBe(180);
  });

  it('keeps two-decimal precision for non-round values', () => {
    // goal の strict yaw 0.1 rad ≈ 5.73°
    expect(formatToleranceYawDeg(0.1)).toBe(5.73);
  });

  it('round-trips round degrees without drift (deg → rad(4桁) → deg)', () => {
    [30, 45, 90, 120, 180].forEach((deg) => {
      const storedRad = roundTo(degToRad(deg), TOLERANCE_YAW_DIGITS);
      const shownDeg = formatToleranceYawDeg(storedRad);
      const resavedRad = roundTo(degToRad(shownDeg), TOLERANCE_YAW_DIGITS);
      expect(resavedRad).toBe(storedRad);
    });
  });

  it('returns NaN for non-numeric input', () => {
    expect(Number.isNaN(formatToleranceYawDeg('abc'))).toBe(true);
    expect(Number.isNaN(formatToleranceYawDeg(NaN))).toBe(true);
  });
});

describe('roundTo (#242)', () => {
  it('exposes save-path digit constants', () => {
    // backend yaml_handler._POSE_XY_DIGITS 等と必ず一致させる (両層 defense-in-depth)
    expect(POSE_XY_DIGITS).toBe(3);
    expect(POSE_YAW_DIGITS).toBe(4);
    expect(TOLERANCE_XY_DIGITS).toBe(3);
    expect(TOLERANCE_YAW_DIGITS).toBe(4);
  });

  it('rounds finite numbers to the requested digits', () => {
    expect(roundTo(0.10000038482226709, POSE_XY_DIGITS)).toBe(0.1);
    expect(roundTo(3.1399991679827224, POSE_YAW_DIGITS)).toBe(3.14);
    expect(roundTo(0.7850002283279937, TOLERANCE_YAW_DIGITS)).toBe(0.785);
    expect(roundTo(1.2345, 2)).toBe(1.23);
    expect(roundTo(1.2355, 2)).toBe(1.24);
  });

  it('accepts string input alongside numeric', () => {
    expect(roundTo('1.23456', 3)).toBe(1.235);
    // 数値文字列 → 丸めた結果が 0 (≒ -0 / +0 区別なく 0 扱い) になることだけを確認
    expect(roundTo('-0.0049', 2) === 0).toBe(true);
  });

  it('returns NaN for non-numeric / non-finite input', () => {
    expect(Number.isNaN(roundTo('', 3))).toBe(true);
    expect(Number.isNaN(roundTo('abc', 3))).toBe(true);
    expect(Number.isNaN(roundTo(NaN, 3))).toBe(true);
    expect(Number.isNaN(roundTo(Infinity, 3))).toBe(true);
    expect(Number.isNaN(roundTo(-Infinity, 3))).toBe(true);
  });
});

describe('parseTolerance', () => {
  it('exposes msg-spec min value (#138)', () => {
    expect(TOLERANCE_MIN).toBe(0.001);
  });

  it('clamps xy < 0.001 to TOLERANCE_MIN (#138 spec)', () => {
    // 0 / 負値 / 0 寄り は無反応 POI を作るため明示的に min に補正
    expect(parseTolerance('0', 0.785)).toEqual({ xy: 0.001, yaw: 0.785 });
    expect(parseTolerance('-0.5', 0.785)).toEqual({ xy: 0.001, yaw: 0.785 });
    expect(parseTolerance('0.0005', 0.785)).toEqual({ xy: 0.001, yaw: 0.785 });
  });

  it('clamps yaw < 0.001 to TOLERANCE_MIN (#138 spec)', () => {
    expect(parseTolerance('0.5', 0)).toEqual({ xy: 0.5, yaw: 0.001 });
    expect(parseTolerance('0.5', -0.1)).toEqual({ xy: 0.5, yaw: 0.001 });
    expect(parseTolerance('0.5', 0.0005)).toEqual({ xy: 0.5, yaw: 0.001 });
  });

  it('falls back to TOLERANCE_MIN when input is empty / non-numeric / NaN', () => {
    expect(parseTolerance('', undefined)).toEqual({ xy: 0.001, yaw: 0.001 });
    expect(parseTolerance('abc', null)).toEqual({ xy: 0.001, yaw: 0.001 });
    expect(parseTolerance(NaN, NaN)).toEqual({ xy: 0.001, yaw: 0.001 });
  });

  it('preserves valid values above TOLERANCE_MIN', () => {
    expect(parseTolerance('0.5', 0.785)).toEqual({ xy: 0.5, yaw: 0.785 });
    expect(parseTolerance(0.42, Math.PI / 6)).toEqual({ xy: 0.42, yaw: Math.PI / 6 });
  });

  it('handles deg-converted yaw values (typical UI flow)', () => {
    // UI で 45° 入力 → degToRad(45) → parseTolerance に渡す流れ
    const yawRad = degToRad(45);
    const result = parseTolerance('0.5', yawRad);
    expect(result.xy).toBe(0.5);
    expect(result.yaw).toBeCloseTo(Math.PI / 4, 6);
  });
});

describe('validatePoiTags', () => {
  it('accepts non-conflicting combinations', () => {
    expect(validatePoiTags(poi('a', ['waypoint'])).ok).toBe(true);
    expect(validatePoiTags(poi('b', ['landmark'])).ok).toBe(true);
    expect(validatePoiTags(poi('c', ['event', 'landmark', 'hazard'])).ok).toBe(true);
    expect(validatePoiTags(poi('d', ['waypoint', 'pause'])).ok).toBe(true);
    expect(validatePoiTags(poi('e', [])).ok).toBe(true);
  });

  it('rejects waypoint + landmark combination', () => {
    const result = validatePoiTags(poi('a', ['waypoint', 'landmark']));
    expect(result.ok).toBe(false);
    expect(result.error).toMatch(/waypoint.*landmark/);
  });

  it('rejects pause + landmark combination (#143)', () => {
    // landmark は到達不可な reference のため pause 動作が成立しない。
    const result = validatePoiTags(poi('a', ['pause', 'landmark']));
    expect(result.ok).toBe(false);
    expect(result.error).toMatch(/pause.*landmark/);
  });

  it('matches case-insensitively for all exclusion pairs', () => {
    expect(validatePoiTags(poi('a', ['WAYPOINT', 'Landmark'])).ok).toBe(false);
    expect(validatePoiTags(poi('b', ['Pause', 'Landmark'])).ok).toBe(false);  // #143
  });

  // (initial_pose × landmark 排他 test は #144 で initial_pose system tag を廃止したため削除。)

  it('handles missing / null poi gracefully', () => {
    expect(validatePoiTags(null).ok).toBe(true);
    expect(validatePoiTags(undefined).ok).toBe(true);
    expect(validatePoiTags({}).ok).toBe(true);
  });
});
