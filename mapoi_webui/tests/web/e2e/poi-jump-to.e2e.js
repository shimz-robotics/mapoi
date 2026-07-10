// POI list 選択で map を該当 POI へパンする jump-to (#382) の browser smoke。
// Leaflet の viewport 判定 (getBounds/panTo) は jsdom で動かないため e2e 層で pin する。
//
// marker の特定方法: 選択中 marker は highlight icon (stroke #e67e22, map-icons.js) で
// 一意に locate できる。未選択 marker は DOM 順 = showPois の生成順 (= pois 配列順:
// wedge, wedge2, disc, pause, landmark) で nth 指定する (highlightPoi の setIcon が
// 要素を作り直すまでは生成順が保たれる)。
//
// map のパン操作は背景の mouse drag (Leaflet 標準)。背景 drag は POI 選択に影響しない。
// 慣性で drag 後も少し流れるため、位置を比較する検証は静定待ちを挟む。
const { test, expect } = require('@playwright/test');
const { loadApp, poiCard, setSectionOpen } = require('./helpers');

// icon の外枠 div で比較する (path は stroke 幅で bounding box が変わるため)
const HIGHLIGHTED = '.poi-arrow-icon:has(path[stroke="#e67e22"])';

// map を左へ 2 回 drag して view を右へ大きくずらす (POI 群は初期 fitBounds の中央
// 付近に固まっているので、計 ~800px で全て viewport 外へ出る)。
async function panMapAway(page) {
  const box = await page.locator('#map').boundingBox();
  const cx = box.x + box.width / 2;
  const cy = box.y + box.height / 2;
  for (let i = 0; i < 2; i += 1) {
    await page.mouse.move(cx + 200, cy);
    await page.mouse.down();
    await page.mouse.move(cx - 200, cy, { steps: 10 });
    await page.mouse.up();
  }
  // 慣性の静定を待つ
  await page.waitForTimeout(500);
}

test.describe('POI list jump-to pan (#382)', () => {
  test('selecting an off-screen POI pans it into view without changing zoom', async ({ page }) => {
    await loadApp(page);
    await setSectionOpen(page, '#btn-poi-toggle', '#poi-body', true);

    // poi_pause (生成順 4 番目 = nth(3)) を viewport 外へ追いやる
    const pauseMarker = page.locator('.poi-arrow-icon').nth(3);
    await expect(pauseMarker).toBeInViewport();
    await panMapAway(page);
    await expect(pauseMarker).not.toBeInViewport();
    const zoomBefore = await page.locator('#map').getAttribute('data-zoom');

    await poiCard(page, 'poi_pause').locator('.poi-card-name').click();

    // panTo (アニメーション) 完了後、選択 marker が viewport 内に戻る。zoom は不変
    await expect(page.locator(HIGHLIGHTED)).toBeInViewport();
    expect(await page.locator('#map').getAttribute('data-zoom')).toBe(zoomBefore);
  });

  test('selecting a visible POI does not move the map', async ({ page }) => {
    await loadApp(page);
    await setSectionOpen(page, '#btn-poi-toggle', '#poi-body', true);

    const pauseMarker = page.locator('.poi-arrow-icon').nth(3);
    const before = await pauseMarker.boundingBox();

    await poiCard(page, 'poi_pause').locator('.poi-card-name').click();

    // highlight で icon は作り直されるが、パンしないので位置は変わらない
    const highlighted = page.locator(HIGHLIGHTED);
    await expect(highlighted).toHaveCount(1);
    const after = await highlighted.boundingBox();
    expect(Math.abs(after.x - before.x)).toBeLessThanOrEqual(1);
    expect(Math.abs(after.y - before.y)).toBeLessThanOrEqual(1);
  });

  test('selecting a visibility-off POI (no marker) does not pan or throw', async ({ page }) => {
    await loadApp(page);
    await setSectionOpen(page, '#btn-poi-toggle', '#poi-body', true);

    // poi_pause の marker を消してから viewport 外相当の状況を作る
    await poiCard(page, 'poi_pause').locator('.poi-card-checkbox').uncheck();
    await expect(page.locator('.poi-arrow-icon')).toHaveCount(4);
    await panMapAway(page);

    const reference = page.locator('.poi-arrow-icon').first();
    const before = await reference.boundingBox();

    await poiCard(page, 'poi_pause').locator('.poi-card-name').click();
    await expect(poiCard(page, 'poi_pause')).toHaveClass(/(^|\s)selected(\s|$)/);
    await page.waitForTimeout(500);

    // marker が無い POI ではパンしない (基準 marker の位置が不変)
    const after = await page.locator('.poi-arrow-icon').first().boundingBox();
    expect(Math.abs(after.x - before.x)).toBeLessThanOrEqual(1);
    expect(Math.abs(after.y - before.y)).toBeLessThanOrEqual(1);
  });
});
