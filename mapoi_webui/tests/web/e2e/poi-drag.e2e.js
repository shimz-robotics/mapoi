// B. POI ドラッグ enable/disable の実適用 (#284, 優先度 高)。
// pure helper shouldEnablePoiDrag の bool を実際に L.marker.dragging.enable()/disable() へ
// 流した結果を実ブラウザで検証する。Leaflet は draggable な marker の icon に
// 'leaflet-marker-draggable' class を付けるので、これで「掴める/掴めない」を観測する。
const { test, expect } = require('@playwright/test');
const { loadApp, selectPoi } = require('./helpers');

// 選択中 (highlighted) の POI arrow marker = draggable。yaw ハンドル (.poi-yaw-handle) も
// draggable だが class が異なるので、.poi-arrow-icon に限定すれば arrow だけ数えられる。
const DRAGGABLE_ARROW = '.poi-arrow-icon.leaflet-marker-draggable';

test.describe('B. POI ドラッグ enable/disable の実適用', () => {
  test('選択中 POI marker だけ draggable になり、ドラッグで dirty 化する', async ({ page }) => {
    await loadApp(page);
    // 初期は draggable な arrow marker 無し・未 dirty。
    await expect(page.locator(DRAGGABLE_ARROW)).toHaveCount(0);
    await expect(page.locator('#btn-save')).toBeDisabled();

    await selectPoi(page, 'poi_wedge');
    // 選択中 1 個だけ draggable。
    await expect(page.locator(DRAGGABLE_ARROW)).toHaveCount(1);

    // その marker を実ドラッグ → dragend → onPoiDragEnd → working copy 更新 + dirty。
    const marker = page.locator(DRAGGABLE_ARROW);
    const box = await marker.boundingBox();
    const cx = box.x + box.width / 2;
    const cy = box.y + box.height / 2;
    await page.mouse.move(cx, cy);
    await page.mouse.down();
    await page.mouse.move(cx + 45, cy + 30, { steps: 10 });
    await page.mouse.move(cx + 75, cy + 55, { steps: 10 });
    await page.mouse.up();

    await expect(page.locator('#btn-save')).toBeEnabled();
    await expect(page.locator('#dirty-indicator')).not.toBeEmpty();
  });

  test('route 編集中は選択中 POI でもドラッグ不可になり、Cancel で戻る', async ({ page }) => {
    await loadApp(page);
    await selectPoi(page, 'poi_wedge');
    await expect(page.locator(DRAGGABLE_ARROW)).toHaveCount(1);

    // route 編集開始 → routeEditor.onEditingChange → setPoiDraggingAllowed(false)。
    await page.locator('#btn-add-route').click();
    await expect(page.locator(DRAGGABLE_ARROW)).toHaveCount(0);

    // Cancel → setPoiDraggingAllowed(true) で再び draggable。
    await page.locator('#btn-route-form-cancel').click();
    await expect(page.locator(DRAGGABLE_ARROW)).toHaveCount(1);
  });
});
