// D. section 折りたたみの app.js 実適用 (#284, #273 で明示の重点)。
// POI 編集フォームを開くと PoiEditor.showForm → onEditFormVisibilityChange(true) →
// app.js collapseSectionsForEditing が nav/tag/route を畳み poi-list を隠す。閉じると
// 開く前の開閉状態へ復元する (全開ではなく「元の状態」)。IIFE の app.js は単体 export を
// 持たず jsdom では new MapViewer で落ちるため、この配線は実ブラウザでのみ通せる。
const { test, expect } = require('@playwright/test');
const { loadApp, poiCard, displayOf, setSectionOpen } = require('./helpers');

test.describe('D. section 折りたたみの app.js 実適用', () => {
  test('編集フォーム開で他 section と poi-list が畳まれ、閉じると元の開閉に戻る', async ({ page }) => {
    await loadApp(page);

    // 初期状態を既知に固定: nav 開・route 開・tag 閉 (= 1280px 既定だが防御的に設定)。
    // tag を「閉」にしておくことで、復元が全開ではなく「元の状態」であることを検証できる。
    await setSectionOpen(page, '#btn-nav-toggle', '#nav-body', true);
    await setSectionOpen(page, '#btn-route-toggle', '#route-body', true);
    await setSectionOpen(page, '#btn-tag-toggle', '#tag-body', false);

    const navBefore = await displayOf(page, '#nav-body');
    const routeBefore = await displayOf(page, '#route-body');
    expect(navBefore).not.toBe('none'); // 開いている前提 (collapse を意味あるものにする)
    expect(routeBefore).not.toBe('none');
    expect(await displayOf(page, '#tag-body')).toBe('none'); // 元から閉

    // POI の Edit を開く。
    await poiCard(page, 'poi_wedge').locator('.btn-edit').click();
    await expect(page.locator('#poi-edit-form')).not.toHaveClass(/(^|\s)hidden(\s|$)/);

    // 編集中は nav/tag/route と poi-list が全て非表示。
    expect(await displayOf(page, '#nav-body')).toBe('none');
    expect(await displayOf(page, '#tag-body')).toBe('none');
    expect(await displayOf(page, '#route-body')).toBe('none');
    expect(await displayOf(page, '#poi-list')).toBe('none');

    // Cancel で閉じる → 元の開閉状態へ復元。
    await page.locator('#btn-form-cancel').click();
    await expect(page.locator('#poi-edit-form')).toHaveClass(/(^|\s)hidden(\s|$)/);
    expect(await displayOf(page, '#nav-body')).toBe(navBefore);     // 元の開へ
    expect(await displayOf(page, '#route-body')).toBe(routeBefore); // 元の開へ
    expect(await displayOf(page, '#tag-body')).toBe('none');        // 元の閉のまま (全開ではない)
    expect(await displayOf(page, '#poi-list')).not.toBe('none');    // 一覧復活
  });
});
