const { test, expect } = require('@playwright/test');
const { loadApp, poiCard, setSectionOpen } = require('./helpers');

function routeItem(page, name) {
  const escaped = name.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
  return page.locator('.route-item').filter({
    has: page.locator('.route-item-name', { hasText: new RegExp(`^${escaped}$`) }),
  });
}

async function openRouteEdit(page) {
  await loadApp(page);
  await setSectionOpen(page, '#btn-route-toggle', '#route-body', true);
  await routeItem(page, 'test_route').locator('.btn-edit').click();
  await expect(page.locator('#route-edit-form')).not.toHaveClass(/(^|\s)hidden(\s|$)/);
}

test.describe('Route edit POI candidate sync', () => {
  test('POI list selection updates the matching route edit select', async ({ page }) => {
    await openRouteEdit(page);
    await setSectionOpen(page, '#btn-poi-toggle', '#poi-body', true);

    await poiCard(page, 'poi_wedge').locator('.poi-card-name').click();
    await expect(page.locator('#route-wp-select')).toHaveValue('poi_wedge');
    await expect(page.locator('#route-lm-select')).toHaveValue('');

    await poiCard(page, 'poi_landmark').locator('.poi-card-name').click();
    await expect(page.locator('#route-wp-select')).toHaveValue('');
    await expect(page.locator('#route-lm-select')).toHaveValue('poi_landmark');
  });

  test('route edit map clicks sync selects while preserving add semantics', async ({ page }) => {
    await openRouteEdit(page);

    await page.locator('.poi-arrow-icon').nth(4).click();
    await expect(page.locator('#route-lm-select')).toHaveValue('poi_landmark');
    await expect(page.locator('#route-wp-select')).toHaveValue('');
    await expect(page.locator('#route-waypoint-list .wp-item')).toHaveCount(3);

    await page.locator('.poi-arrow-icon').nth(1).click();
    await expect(page.locator('#route-wp-select')).toHaveValue('poi_wedge2');
    await expect(page.locator('#route-lm-select')).toHaveValue('');
    await expect(page.locator('#route-waypoint-list .wp-item')).toHaveCount(4);
  });
});
