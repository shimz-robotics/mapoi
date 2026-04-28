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
    expect(hasLandmarkTag(poi('c', ['origin', 'pause']))).toBe(false);
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
      poi('b', ['origin']),
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

describe('parseTolerance', () => {
  it('preserves xy=0 as a finite value (not default 0.5)', () => {
    // Codex review #137 medium: `||` だと意図的な 0 が 0.5 に化けるため
    expect(parseTolerance('0', 0.1)).toEqual({ xy: 0, yaw: 0.1 });
  });

  it('falls back to xy=0.5 when input is empty / non-numeric', () => {
    expect(parseTolerance('', undefined)).toEqual({ xy: 0.5, yaw: 0.0 });
    expect(parseTolerance('abc', undefined)).toEqual({ xy: 0.5, yaw: 0.0 });
    expect(parseTolerance(NaN, undefined)).toEqual({ xy: 0.5, yaw: 0.0 });
  });

  it('preserves originalYaw including 0 when finite', () => {
    expect(parseTolerance('0.3', 0)).toEqual({ xy: 0.3, yaw: 0 });
    expect(parseTolerance('0.3', 0.15)).toEqual({ xy: 0.3, yaw: 0.15 });
  });

  it('defaults yaw to 0 when originalYaw is missing / null / NaN', () => {
    expect(parseTolerance('0.3', undefined)).toEqual({ xy: 0.3, yaw: 0 });
    expect(parseTolerance('0.3', null)).toEqual({ xy: 0.3, yaw: 0 });
    expect(parseTolerance('0.3', NaN)).toEqual({ xy: 0.3, yaw: 0 });
  });

  it('parses numeric input alongside string input', () => {
    expect(parseTolerance(0.42, 0)).toEqual({ xy: 0.42, yaw: 0 });
    expect(parseTolerance('0.42', 0)).toEqual({ xy: 0.42, yaw: 0 });
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
