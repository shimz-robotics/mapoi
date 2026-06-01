// @vitest-environment jsdom
//
// nav-controls.js (#199) の unit test。
// - 純粋ヘルパ (resolveCapabilities / deriveNavControlsState / decideMapSwitchOutcome /
//   initialSectionOpenStates) は DOM 非依存だが、applyNavControlsState を同じ file で
//   検証するため jsdom 環境で回す。
// - backend 側契約は #279 (test_get_navigation_capabilities.py) が pin 済。ここはその
//   frontend 投影: backend_status 優先 / legacy subscriber fallback / localization 独立 3-state /
//   navigation availability 表示 / map switch 分岐 / compact 初期 section を固める。

import { describe, it, expect, beforeEach } from 'vitest';
import * as nav from '../../web/js/nav-controls.js';

const {
  resolveCapabilities,
  deriveNavControlsState,
  applyNavControlsState,
  decideMapSwitchOutcome,
  initialSectionOpenStates,
} = nav;

describe('resolveCapabilities (navigation payload の前回値保持)', () => {
  it('incoming があればそれを採用する', () => {
    const incoming = { navigation_available: true };
    expect(resolveCapabilities(incoming, { navigation_available: false })).toBe(incoming);
  });

  it('incoming が無ければ previous を維持する (poll で navigation 欠落しても UI を保つ)', () => {
    const previous = { navigation_available: true };
    expect(resolveCapabilities(null, previous)).toBe(previous);
    expect(resolveCapabilities(undefined, previous)).toBe(previous);
  });

  it('両方無ければ null', () => {
    expect(resolveCapabilities(null, null)).toBeNull();
    expect(resolveCapabilities(undefined, undefined)).toBeNull();
  });
});

describe('deriveNavControlsState — navigation 操作 gate', () => {
  it('backend_status があれば backend_ready をそのまま使う (subscriber 数より優先)', () => {
    const ready = deriveNavControlsState({
      backend_status: { backend_ready: true },
      command_available: false, // subscriber は 0 でも backend_ready 優先
    });
    expect(ready.navOperationsEnabled).toBe(true);

    const notReady = deriveNavControlsState({
      backend_status: { backend_ready: false },
      command_available: true, // subscriber がいても backend_ready=false が優先
    });
    expect(notReady.navOperationsEnabled).toBe(false);
  });

  it('backend_status 不在は command_available (subscriber 数) で代用する (後方互換)', () => {
    expect(deriveNavControlsState({ command_available: true }).navOperationsEnabled).toBe(true);
    expect(deriveNavControlsState({ command_available: false }).navOperationsEnabled).toBe(false);
  });
});

describe('deriveNavControlsState — navigation availability 表示', () => {
  it('navigation_available=true は connected 表示', () => {
    const s = deriveNavControlsState({ navigation_available: true });
    expect(s.navigationAvailable).toBe(true);
    expect(s.navClass).toContain('navigation-available');
    expect(s.navClass).not.toContain('navigation-unavailable');
    expect(s.navLabel).toBe('Navigation connected');
  });

  it('navigation_available=false は unavailable 表示', () => {
    const s = deriveNavControlsState({ navigation_available: false });
    expect(s.navigationAvailable).toBe(false);
    expect(s.navClass).toContain('navigation-unavailable');
    expect(s.navLabel).toBe('Navigation unavailable');
  });

  it('backend_status からツールチップを組む (backend_type / backend_ready / reason)', () => {
    const s = deriveNavControlsState({
      backend_status: { backend_type: 'nav2', backend_ready: false, reason: 'lifecycle inactive' },
    });
    expect(s.navTooltip).toBe('backend_type: nav2\nbackend_ready: false\nreason: lifecycle inactive');
  });

  it('reason が無ければツールチップに reason 行を出さない', () => {
    const s = deriveNavControlsState({ backend_status: { backend_type: 'nav2', backend_ready: true } });
    expect(s.navTooltip).toBe('backend_type: nav2\nbackend_ready: true');
  });

  it('backend_status 不在時は topics の subscriber 数からツールチップを組む', () => {
    const s = deriveNavControlsState({
      topics: {
        goal: { topic: 'mapoi/nav/goal', subscribers: 1 },
        cancel: { topic: 'mapoi/nav/cancel', subscribers: 0 },
      },
    });
    expect(s.navTooltip).toBe('mapoi/nav/goal: 1\nmapoi/nav/cancel: 0');
  });
});

describe('deriveNavControlsState — localization は navigation と独立 (#209 3-state)', () => {
  it('localization_status.backend_ready=true は connected', () => {
    const s = deriveNavControlsState({ localization_status: { backend_ready: true, backend_type: 'amcl' } });
    expect(s.localizationReady).toBe(true);
    expect(s.locClass).toContain('navigation-available');
    expect(s.locLabel).toBe('Localization connected');
    expect(s.locTooltip).toBe('backend_type: amcl\nbackend_ready: true');
  });

  it('localization_status.backend_ready=false は unavailable', () => {
    const s = deriveNavControlsState({ localization_status: { backend_ready: false } });
    expect(s.localizationReady).toBe(false);
    expect(s.locClass).toContain('navigation-unavailable');
    expect(s.locLabel).toBe('Localization unavailable');
  });

  it('localization_status 不在は unknown (安全側で disable / topic 未受信メッセージ)', () => {
    const s = deriveNavControlsState({ navigation_available: true });
    expect(s.localizationReady).toBe(false);
    expect(s.locClass).toContain('navigation-unknown');
    expect(s.locLabel).toBe('Localization unknown');
    expect(s.locTooltip).toBe('mapoi/localization/backend_status not received yet');
  });

  it('navigation ready でも localization は独立 (片方が他方を上書きしない)', () => {
    const s = deriveNavControlsState({ backend_status: { backend_ready: true } });
    expect(s.navOperationsEnabled).toBe(true);
    expect(s.localizationReady).toBe(false); // localization_status 不在
  });
});

describe('deriveNavControlsState — capabilities 不在', () => {
  it('null/undefined でも安全な default を返す', () => {
    for (const caps of [null, undefined, {}]) {
      const s = deriveNavControlsState(caps);
      expect(s.navOperationsEnabled).toBe(false);
      expect(s.navigationAvailable).toBe(false);
      expect(s.localizationReady).toBe(false);
      expect(s.navLabel).toBe('Navigation unavailable');
      expect(s.locLabel).toBe('Localization unknown');
    }
  });
});

describe('applyNavControlsState — DOM への適用 (jsdom)', () => {
  beforeEach(() => {
    document.body.innerHTML = `
      <div id="nav-body"></div>
      <div id="navigation-availability"><span id="navigation-availability-text"></span></div>
      <div id="localization-availability"><span id="localization-availability-text"></span></div>
      <button id="btn-nav-go"></button>
      <button id="btn-nav-run"></button>
      <button id="btn-nav-pause"></button>
      <button id="btn-nav-resume"></button>
      <button id="btn-nav-stop"></button>
      <button id="btn-nav-setpose"></button>
      <select id="nav-initialpose-select"></select>
      <select id="navigation-map-select"></select>
    `;
  });

  const operationButtonIds = [
    'btn-nav-go', 'btn-nav-run', 'btn-nav-pause', 'btn-nav-resume', 'btn-nav-stop',
  ];

  it('available + ready 状態は操作 UI を enable し connected 表示にする', () => {
    const state = deriveNavControlsState({
      backend_status: { backend_ready: true, backend_type: 'nav2' },
      navigation_available: true,
      localization_status: { backend_ready: true, backend_type: 'amcl' },
    });
    applyNavControlsState(state, document);

    operationButtonIds.forEach((id) => {
      expect(document.getElementById(id).disabled).toBe(false);
    });
    expect(document.getElementById('navigation-map-select').disabled).toBe(false);
    expect(document.getElementById('btn-nav-setpose').disabled).toBe(false);
    expect(document.getElementById('nav-initialpose-select').disabled).toBe(false);
    expect(document.getElementById('nav-body').classList.contains('navigation-unavailable-state')).toBe(false);
    expect(document.getElementById('navigation-availability').className).toContain('navigation-available');
    expect(document.getElementById('navigation-availability-text').textContent).toBe('Navigation connected');
    expect(document.getElementById('navigation-availability').title)
      .toBe('backend_type: nav2\nbackend_ready: true');
  });

  it('backend not ready は navigation 操作 UI を disable する', () => {
    const state = deriveNavControlsState({
      backend_status: { backend_ready: false },
      navigation_available: false,
    });
    applyNavControlsState(state, document);

    operationButtonIds.forEach((id) => {
      expect(document.getElementById(id).disabled).toBe(true);
    });
    expect(document.getElementById('navigation-map-select').disabled).toBe(true);
    expect(document.getElementById('nav-body').classList.contains('navigation-unavailable-state')).toBe(true);
    expect(document.getElementById('navigation-availability-text').textContent).toBe('Navigation unavailable');
  });

  it('legacy fallback (command_available) でも操作 UI を enable する', () => {
    const state = deriveNavControlsState({ command_available: true });
    applyNavControlsState(state, document);
    operationButtonIds.forEach((id) => {
      expect(document.getElementById(id).disabled).toBe(false);
    });
  });

  it('Set Initial Pose 系は localization 側で独立に gate する', () => {
    // navigation ready だが localization 不在 → 操作ボタンは enable、setpose 系は disable。
    const state = deriveNavControlsState({ backend_status: { backend_ready: true } });
    applyNavControlsState(state, document);
    expect(document.getElementById('btn-nav-go').disabled).toBe(false);
    expect(document.getElementById('btn-nav-setpose').disabled).toBe(true);
    expect(document.getElementById('nav-initialpose-select').disabled).toBe(true);
    expect(document.getElementById('localization-availability-text').textContent).toBe('Localization unknown');
    expect(document.getElementById('localization-availability').className).toContain('navigation-unknown');
  });

  it('一部 DOM が欠けていても例外を投げない', () => {
    document.body.innerHTML = '<div id="nav-body"></div>'; // 他要素なし
    const state = deriveNavControlsState({ navigation_available: true });
    expect(() => applyNavControlsState(state, document)).not.toThrow();
    expect(document.getElementById('nav-body').classList.contains('navigation-unavailable-state')).toBe(false);
  });
});

describe('decideMapSwitchOutcome — map switch 応答の後続アクション', () => {
  it('error は両 select を巻き戻し "Map switch failed:" を前置する', () => {
    expect(decideMapSwitchOutcome({ error: 'map not found' }))
      .toEqual({ kind: 'error', message: 'Map switch failed: map not found', revert: 'both' });
  });

  it('warning は navigation 側 select のみ巻き戻す', () => {
    expect(decideMapSwitchOutcome({ warning: 'no subscriber' }))
      .toEqual({ kind: 'warning', message: 'no subscriber', revert: 'nav' });
  });

  it('error は warning より優先する', () => {
    expect(decideMapSwitchOutcome({ error: 'boom', warning: 'w' }).kind).toBe('error');
  });

  it('error/warning が無ければ switching として受理し巻き戻さない', () => {
    expect(decideMapSwitchOutcome({})).toEqual({ kind: 'switching', message: '', revert: 'none' });
    expect(decideMapSwitchOutcome(null)).toEqual({ kind: 'switching', message: '', revert: 'none' });
  });
});

describe('initialSectionOpenStates — compact 初期表示', () => {
  it('desktop は Navigation/Routes/POIs を開き Tags を閉じる', () => {
    expect(initialSectionOpenStates(false)).toEqual({ nav: true, route: true, poi: true, tag: false });
  });

  it('compact は Navigation のみ開き Routes/POIs/Tags を閉じる', () => {
    expect(initialSectionOpenStates(true)).toEqual({ nav: true, route: false, poi: false, tag: false });
  });
});
