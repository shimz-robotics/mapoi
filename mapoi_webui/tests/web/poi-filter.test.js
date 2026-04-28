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
