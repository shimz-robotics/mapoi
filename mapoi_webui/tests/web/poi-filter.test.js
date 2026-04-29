// vitest unit tests for the POI candidate filters in
// mapoi_webui/web/js/poi-filter.js (#85).
//
// 対象: MapoiPoiFilter.{hasLandmarkTag, filterGoalCandidates,
//   filterInitialPoseCandidates, filterRouteWaypointCandidates}
//
// テスト方針:
// - landmark system tag を持つ POI が dropdown 候補から確実に除外されること
//   を検証する (#85: landmark = Nav2 goal 不可な reference 専用)。
// - DOM / window 依存ゼロの pure JS。

import { describe, it, expect } from 'vitest';
import * as filter from '../../web/js/poi-filter.js';

const {
  hasLandmarkTag,
  filterGoalCandidates,
  filterInitialPoseCandidates,
  filterRouteWaypointCandidates,
  validatePoiTags,
  parseTolerance,
  degToRad,
  radToDeg,
  TOLERANCE_MIN,
} = filter;

const poi = (name, tags) => ({ name, tags });

describe('hasLandmarkTag', () => {
  it('returns true when tags include landmark', () => {
    expect(hasLandmarkTag(poi('a', ['landmark']))).toBe(true);
    expect(hasLandmarkTag(poi('b', ['event', 'landmark', 'hazard']))).toBe(true);
  });

  it('returns false when landmark is absent', () => {
    expect(hasLandmarkTag(poi('a', ['goal']))).toBe(false);
    expect(hasLandmarkTag(poi('b', []))).toBe(false);
    expect(hasLandmarkTag(poi('c', ['goal', 'pause']))).toBe(false);
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

describe('filterGoalCandidates', () => {
  it('keeps goal POIs without landmark', () => {
    const pois = [
      poi('north_goal', ['goal']),
      poi('checkpoint', ['goal', 'checkpoint']),
    ];
    expect(filterGoalCandidates(pois).map((p) => p.name)).toEqual(['north_goal', 'checkpoint']);
  });

  it('drops POIs that combine goal with landmark', () => {
    // goal+landmark は仕様上排他だが万一 config に混入しても候補に載せない。
    const pois = [
      poi('valid', ['goal']),
      poi('reference', ['goal', 'landmark']),
    ];
    expect(filterGoalCandidates(pois).map((p) => p.name)).toEqual(['valid']);
  });

  it('drops landmark-only POIs (no goal tag)', () => {
    const pois = [
      poi('landmark_only', ['landmark']),
      poi('hazard_landmark', ['event', 'landmark', 'hazard']),
    ];
    expect(filterGoalCandidates(pois)).toEqual([]);
  });

  it('returns [] for empty / null input', () => {
    expect(filterGoalCandidates([])).toEqual([]);
    expect(filterGoalCandidates(null)).toEqual([]);
    expect(filterGoalCandidates(undefined)).toEqual([]);
  });
});

describe('filterInitialPoseCandidates', () => {
  it('drops landmark POIs (排他: landmark + initial_pose)', () => {
    const pois = [
      poi('start_zone', ['goal', 'initial_pose']),
      poi('landmark_x', ['landmark']),
      poi('hazard', ['event', 'landmark', 'hazard']),
      poi('north', ['goal']),
    ];
    expect(filterInitialPoseCandidates(pois).map((p) => p.name)).toEqual(['start_zone', 'north']);
  });

  it('keeps non-landmark POIs even without initial_pose tag', () => {
    // initial pose dropdown は全 navigable POI を載せて user に選ばせる
    // (manual override 用途) ため、initial_pose タグの有無で絞らない。
    const pois = [
      poi('a', ['goal']),
      poi('b', ['pause']),
    ];
    expect(filterInitialPoseCandidates(pois).map((p) => p.name)).toEqual(['a', 'b']);
  });
});

describe('filterRouteWaypointCandidates', () => {
  it('drops landmark POIs from route waypoint candidate list', () => {
    const pois = [
      poi('start_zone', ['goal']),
      poi('hazard', ['event', 'landmark', 'hazard']),
      poi('north_goal', ['goal']),
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
    expect(validatePoiTags(poi('a', ['goal'])).ok).toBe(true);
    expect(validatePoiTags(poi('b', ['landmark'])).ok).toBe(true);
    expect(validatePoiTags(poi('c', ['event', 'landmark', 'hazard'])).ok).toBe(true);
    expect(validatePoiTags(poi('d', ['initial_pose', 'goal'])).ok).toBe(true);
    expect(validatePoiTags(poi('e', [])).ok).toBe(true);
  });

  it('rejects goal + landmark combination', () => {
    const result = validatePoiTags(poi('a', ['goal', 'landmark']));
    expect(result.ok).toBe(false);
    expect(result.error).toMatch(/goal.*landmark/);
  });

  it('rejects initial_pose + landmark combination', () => {
    const result = validatePoiTags(poi('a', ['initial_pose', 'landmark']));
    expect(result.ok).toBe(false);
    expect(result.error).toMatch(/initial_pose.*landmark/);
  });

  it('matches case-insensitively for both system tags', () => {
    expect(validatePoiTags(poi('a', ['GOAL', 'Landmark'])).ok).toBe(false);
    expect(validatePoiTags(poi('b', ['Initial_Pose', 'LANDMARK'])).ok).toBe(false);
  });

  it('handles missing / null poi gracefully', () => {
    expect(validatePoiTags(null).ok).toBe(true);
    expect(validatePoiTags(undefined).ok).toBe(true);
    expect(validatePoiTags({}).ok).toBe(true);
  });
});
