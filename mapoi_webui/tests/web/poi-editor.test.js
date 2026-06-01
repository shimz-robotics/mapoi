// @vitest-environment jsdom
//
// PoiEditor (poi-editor.js) の DOM / インスタンス依存ロジックの実体テスト (#273)。
//
// 対象 (pure-helper に切り出せず poi-interactions.test.js の対象外だった配線):
// - updateDraggedPosition : ドラッグ移動の working copy 反映 / 丸め / dirty 化 /
//                           編集フォーム X/Y 同期 / pose 欠損 early return (#239)
// - updateDraggedYaw      : 扇形ハンドル回転の yaw 反映 / 丸め / dirty / Yaw 同期 (#275)
// - showForm/hideForm/exitEditMode : 編集フォーム開閉と onEditFormVisibilityChange 発火 (#240)
//
// テスト方針:
// - PoiEditor は constructor で多数の getElementById + addEventListener を行うため、
//   最小だが完全な DOM harness を jsdom 上に組んで実体を生成する (nav-controls.test.js の
//   applyNavControlsState と同じ jsdom 路線)。
// - 丸め桁数 (POSE_XY_DIGITS=3 / POSE_YAW_DIGITS=4) は MapoiPoiFilter に委譲されるので、
//   poi-filter.js を読み込んで global に載せる (browser の script 読み込み順と同じ依存)。
// - Leaflet / app.js への配線 (section 折りたたみの実適用) は app.js 側の責務なので対象外。
//   ここは PoiEditor が onEditFormVisibilityChange を「いつ・どの値で」叩くかまでを固める。

import {
  describe, it, expect, beforeEach, vi,
} from 'vitest';
import * as poiFilterMod from '../../web/js/poi-filter.js';
import * as poiEditorMod from '../../web/js/poi-editor.js';

// poi-editor.js は内部メソッドで bare global `MapoiPoiFilter` を参照する (browser では
// poi-filter.js が window に載せる)。jsdom でも同じになるよう global に明示登録する。
globalThis.MapoiPoiFilter = poiFilterMod.default ?? poiFilterMod;
const PoiEditor = poiEditorMod.default ?? poiEditorMod;

// PoiEditor constructor が要求する DOM。addEventListener する btn-* 系は必須
// (欠けると null.addEventListener で throw する)。
function buildDom() {
  document.body.innerHTML = `
    <div id="poi-list"></div>
    <div id="poi-edit-form" class="hidden"></div>
    <span id="form-title"></span>
    <button id="btn-save"></button>
    <button id="btn-discard"></button>
    <button id="btn-add-poi"></button>
    <span id="dirty-indicator"></span>
    <input id="poi-name" />
    <input id="poi-x" />
    <input id="poi-y" />
    <input id="poi-yaw" />
    <input id="poi-tolerance-xy" />
    <input id="poi-tolerance-yaw" />
    <input id="poi-tags" />
    <textarea id="poi-description"></textarea>
    <button id="btn-form-ok"></button>
    <button id="btn-form-cancel"></button>
  `;
}

function makePoi(name, x, y, yaw) {
  return {
    name, pose: { x, y, yaw }, tolerance: { xy: 0.5, yaw: Math.PI / 4 }, tags: ['waypoint'],
  };
}

let editor;

beforeEach(() => {
  buildDom();
  editor = new PoiEditor();
});

describe('updateDraggedPosition — ドラッグ移動の working copy 反映 (#239)', () => {
  beforeEach(() => {
    editor.loadPois([makePoi('a', 0, 0, 0), makePoi('b', 1, 1, 0)], 'v1');
  });

  it('x/y を POSE_XY_DIGITS=3 桁に丸めて working copy に反映する', () => {
    editor.updateDraggedPosition(0, 1.23456, -2.98765);
    expect(editor.pois[0].pose.x).toBe(1.235);
    expect(editor.pois[0].pose.y).toBe(-2.988);
  });

  it('yaw / tolerance / tags は不変 (position のみ更新)', () => {
    editor.updateDraggedPosition(0, 5, 6);
    expect(editor.pois[0].pose.yaw).toBe(0);
    expect(editor.pois[0].tolerance).toEqual({ xy: 0.5, yaw: Math.PI / 4 });
    expect(editor.pois[0].tags).toEqual(['waypoint']);
  });

  it('dirty 化する (Save / Discard を enable)', () => {
    expect(editor.dirty).toBe(false); // loadPois 直後は clean
    editor.updateDraggedPosition(0, 3, 4);
    expect(editor.dirty).toBe(true);
    expect(editor.btnSave.disabled).toBe(false);
    expect(editor.btnDiscard.disabled).toBe(false);
  });

  it('当該 POI を編集中なら X/Y 入力欄も同期する', () => {
    editor.editingIndex = 0;
    editor.updateDraggedPosition(0, 3.14159, 2.71828);
    expect(editor.inputX.value).toBe('3.142');
    expect(editor.inputY.value).toBe('2.718');
  });

  it('別 POI 編集中なら入力欄を同期しない (pose と dirty は更新)', () => {
    editor.editingIndex = 1; // 0 をドラッグするが編集中は 1
    editor.inputX.value = 'sentinel';
    editor.inputY.value = 'sentinel';
    editor.updateDraggedPosition(0, 9, 9);
    expect(editor.inputX.value).toBe('sentinel');
    expect(editor.inputY.value).toBe('sentinel');
    expect(editor.pois[0].pose.x).toBe(9);
    expect(editor.dirty).toBe(true);
  });

  it('フォーム閉 (editingIndex=-1) でも入力欄を同期しない (yaw 側と対称)', () => {
    editor.editingIndex = -1;
    editor.inputX.value = 'sentinel';
    editor.inputY.value = 'sentinel';
    editor.updateDraggedPosition(0, 9, 9);
    expect(editor.inputX.value).toBe('sentinel');
    expect(editor.inputY.value).toBe('sentinel');
    expect(editor.pois[0].pose.x).toBe(9);
    expect(editor.dirty).toBe(true);
  });

  it('pose 欠損 POI は early return (dirty 化せず throw しない)', () => {
    editor.loadPois([makePoi('a', 0, 0, 0), { name: 'no-pose' }], 'v1');
    expect(() => editor.updateDraggedPosition(1, 5, 5)).not.toThrow();
    expect(editor.dirty).toBe(false);
    expect(editor.pois[1]).toEqual({ name: 'no-pose' });
  });

  it('範囲外 index は early return (throw しない)', () => {
    expect(() => editor.updateDraggedPosition(99, 1, 1)).not.toThrow();
    expect(editor.dirty).toBe(false);
  });

  it('renderList が走り更新後の座標が list card に反映される', () => {
    editor.updateDraggedPosition(0, 7, 8);
    const cards = editor.listEl.querySelectorAll('.poi-card');
    expect(cards.length).toBe(editor.pois.length);
    // card detail は `(x, y) yaw=...` を toFixed(2) で描く。更新後の 7/8 が反映されていること。
    expect(cards[0].textContent).toContain('(7.00, 8.00)');
  });
});

describe('updateDraggedYaw — 扇形ハンドル回転の yaw 反映 (#275)', () => {
  beforeEach(() => {
    editor.loadPois([makePoi('a', 1, 2, 0)], 'v1');
  });

  it('yaw を POSE_YAW_DIGITS=4 桁に丸めて反映する (position は不変)', () => {
    editor.updateDraggedYaw(0, 1.5707963);
    expect(editor.pois[0].pose.yaw).toBe(1.5708);
    expect(editor.pois[0].pose.x).toBe(1);
    expect(editor.pois[0].pose.y).toBe(2);
  });

  it('dirty 化する', () => {
    expect(editor.dirty).toBe(false);
    editor.updateDraggedYaw(0, 0.5);
    expect(editor.dirty).toBe(true);
  });

  it('当該 POI を編集中なら Yaw 入力欄も同期する', () => {
    editor.editingIndex = 0;
    editor.updateDraggedYaw(0, 1.5707963);
    expect(editor.inputYaw.value).toBe('1.5708');
  });

  it('非編集中は Yaw 入力欄を同期しない', () => {
    editor.editingIndex = -1;
    editor.inputYaw.value = 'sentinel';
    editor.updateDraggedYaw(0, 0.5);
    expect(editor.inputYaw.value).toBe('sentinel');
    expect(editor.pois[0].pose.yaw).toBe(0.5);
  });

  it('pose 欠損 POI は early return', () => {
    editor.loadPois([{ name: 'no-pose' }], 'v1');
    expect(() => editor.updateDraggedYaw(0, 1)).not.toThrow();
    expect(editor.dirty).toBe(false);
  });

  it('範囲外 index は early return (position 側と対称)', () => {
    expect(() => editor.updateDraggedYaw(99, 1)).not.toThrow();
    expect(editor.dirty).toBe(false);
  });
});

describe('showForm / hideForm / exitEditMode — 編集フォーム開閉と可視性通知 (#240)', () => {
  it('showForm は hidden を外し onEditFormVisibilityChange(true) を発火する', () => {
    const spy = vi.fn();
    editor.onEditFormVisibilityChange = spy;
    editor.showForm();
    expect(editor.formEl.classList.contains('hidden')).toBe(false);
    expect(spy).toHaveBeenCalledWith(true);
  });

  it('hideForm は hidden を付け onEditFormVisibilityChange(false) を発火する', () => {
    const spy = vi.fn();
    editor.onEditFormVisibilityChange = spy;
    editor.hideForm();
    expect(editor.formEl.classList.contains('hidden')).toBe(true);
    expect(spy).toHaveBeenCalledWith(false);
  });

  it('exitEditMode は編集中なら editingIndex を閉じ hideForm で false 通知 + フォームを隠す', () => {
    editor.editingIndex = 2;
    editor.formEl.classList.remove('hidden'); // 開いている状態から
    const spy = vi.fn();
    editor.onEditFormVisibilityChange = spy;
    editor.exitEditMode();
    expect(editor.editingIndex).toBe(-1);
    expect(spy).toHaveBeenCalledWith(false);
    expect(editor.formEl.classList.contains('hidden')).toBe(true);
  });

  it('exitEditMode は非編集中 (editingIndex=-1) なら no-op (通知しない)', () => {
    editor.editingIndex = -1;
    const spy = vi.fn();
    editor.onEditFormVisibilityChange = spy;
    editor.exitEditMode();
    expect(editor.editingIndex).toBe(-1);
    expect(spy).not.toHaveBeenCalled();
  });

  it('onEditFormVisibilityChange 未設定でも showForm / hideForm は throw しない', () => {
    editor.onEditFormVisibilityChange = null;
    expect(() => editor.showForm()).not.toThrow();
    expect(() => editor.hideForm()).not.toThrow();
  });
});
