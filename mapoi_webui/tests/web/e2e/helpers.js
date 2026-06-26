// mapoi_webui e2e 共有ヘルパ (#284)。*.e2e.js ではないので Playwright のテスト対象にならない。
const { expect } = require('@playwright/test');

// app 起動を明示待ちする。SSE (/api/events) が開きっぱなしで 'networkidle' は来ないため、
// 代わりに「全 POI marker が描画される」= loadMaps→switchMap→loadPois→showPois 完了 を待つ。
async function loadApp(page) {
  const resp = await page.goto('/');
  expect(resp.status()).toBe(200);
  await expect(page.locator('.poi-arrow-icon')).toHaveCount(5);
}

// POI 名の完全一致で card を返す。'poi_wedge' が 'poi_wedge2' に部分一致しないよう anchored regex。
// 名前に regex メタ文字が来ても壊れないようエスケープする。
function poiCard(page, name) {
  const escaped = name.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  return page.locator('.poi-card').filter({
    has: page.locator('.poi-card-name', { hasText: new RegExp(`^${escaped}$`) }),
  });
}

// card 本体クリックで selectPoi → onSelectionChange → highlightPoi (marker 強調 + draggable + yaw ハンドル)。
// Edit/Copy/Del/checkbox は stopPropagation するので、name 要素をクリックして card の click に委ねる。
async function selectPoi(page, name) {
  await poiCard(page, name).locator('.poi-card-name').click();
  await expect(poiCard(page, name)).toHaveClass(/(^|\s)selected(\s|$)/);
}

// computed display を返す ('' ではなく実値 block/flex/none)。
function displayOf(page, selector) {
  return page.locator(selector).evaluate((el) => getComputedStyle(el).display);
}

// 指定 section を望む開閉状態にする (toggle ボタンを必要時だけ押す)。
async function setSectionOpen(page, toggleId, bodyId, wantOpen) {
  const isClosed = (await displayOf(page, bodyId)) === 'none';
  if (wantOpen === isClosed) {
    await page.locator(toggleId).click();
  }
}

module.exports = { loadApp, poiCard, selectPoi, displayOf, setSectionOpen };
