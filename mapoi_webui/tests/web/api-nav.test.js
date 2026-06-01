// MapoiApi の navigation 系 fetch 契約 (#199)。
// navSwitchMap / mode が叩く endpoint・method・body・戻り値を、global fetch を
// stub して固定する。DOM 非依存なので default (node) 環境で回す。
// backend 側の validation は test_api_nav_endpoints.py (#279) が pin 済で、ここは
// client が「正しい URL に正しい payload を投げ、JSON をそのまま返す」ことだけを見る。

import { describe, it, expect, vi, afterEach } from 'vitest';
import * as MapoiApi from '../../web/js/api.js';

afterEach(() => {
  vi.restoreAllMocks();
  delete globalThis.fetch;
});

function mockFetch(payload) {
  const fetchMock = vi.fn().mockResolvedValue({
    json: vi.fn().mockResolvedValue(payload),
  });
  globalThis.fetch = fetchMock;
  return fetchMock;
}

describe('MapoiApi.navSwitchMap', () => {
  it('POST /api/nav/switch-map に map_name を JSON body で送る', async () => {
    const fetchMock = mockFetch({ ok: true });
    const res = await MapoiApi.navSwitchMap('floor1');

    expect(fetchMock).toHaveBeenCalledTimes(1);
    const [url, opts] = fetchMock.mock.calls[0];
    expect(url).toBe('/api/nav/switch-map');
    expect(opts.method).toBe('POST');
    expect(opts.headers).toMatchObject({ 'Content-Type': 'application/json' });
    expect(JSON.parse(opts.body)).toEqual({ map_name: 'floor1' });
    expect(res).toEqual({ ok: true });
  });

  it('backend の warning/error 応答をそのまま透過する', async () => {
    mockFetch({ warning: 'no subscriber' });
    expect(await MapoiApi.navSwitchMap('floor1')).toEqual({ warning: 'no subscriber' });
  });
});

describe('MapoiApi.mode', () => {
  it('GET /api/mode を叩き JSON をそのまま返す', async () => {
    const fetchMock = mockFetch({ mode: 'navigation' });
    const res = await MapoiApi.mode();

    expect(fetchMock).toHaveBeenCalledTimes(1);
    const [url, opts] = fetchMock.mock.calls[0];
    expect(url).toBe('/api/mode');
    expect(opts).toBeUndefined(); // GET は options なし
    expect(res).toEqual({ mode: 'navigation' });
  });
});
