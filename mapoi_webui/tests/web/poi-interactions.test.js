// vitest unit tests for the pure POI 直接操作 helpers in
// mapoi_webui/web/js/poi-interactions.js (#273)。
//
// 対象 (map-viewer.js から切り出した純粋ロジック・幾何):
// - classifyPoiClick   : シングル/ダブルクリック判別コア (#240)
// - wedgeVertexCount    : 扇形 polygon の頂点数 (#275)
// - wedgePointsWorld    : 扇形 polygon の頂点列 (world) (#275)
// - worldToLatLng / latLngToWorld : 等方スケール座標変換 (#273)
// - yawFromDelta        : yaw 回転ハンドルの角度算出 + 等方性不変条件 (#275 / PR #276)
// - shouldEnablePoiDrag : POI marker ドラッグ可否の gate (#239)
// - classifyYawTolerance: tolerance.yaw → wedge / disc / none の分類 (#267 / #275)
// - shouldShowYawHandle : yaw 回転ハンドルの出し分け gate (#275)
//
// テスト方針:
// - map-viewer.js の _handlePoiClick / _wedgePoints / _addYawHandle / worldToLatLng /
//   _applyPoiDraggable / _refreshYawHandle / _drawSectorForPoi がこれらに委譲するので、
//   ここで純粋ロジックを固めれば DOM/Leaflet 配線を起こさずに回帰検知できる
//   (各 PR の Cursor review でテスト不足 medium を繰り返し指摘された箇所)。
// - Leaflet / DOM / window への依存はゼロで pure JS のみ。

import { describe, it, expect } from 'vitest';
import * as poi from '../../web/js/poi-interactions.js';

const {
  classifyPoiClick,
  wedgeVertexCount,
  wedgePointsWorld,
  worldToLatLng,
  latLngToWorld,
  yawFromDelta,
  shouldEnablePoiDrag,
  classifyYawTolerance,
  shouldShowYawHandle,
} = poi;

function nearly(a, b, tol = 1e-9) {
  return Math.abs(a - b) < tol;
}

describe('classifyPoiClick (#240)', () => {
  const TH = 300; // map-viewer の _poiDblClickMs

  it('単発 (prev 無し) はシングルで今回の (index, now) を保持する', () => {
    const r = classifyPoiClick(-1, 0, 2, 1000, TH);
    expect(r.isDouble).toBe(false);
    expect(r.nextIndex).toBe(2);
    expect(r.nextTime).toBe(1000);
  });

  it('同一 index への閾値内 2 連クリックはダブルで追跡 state を reset する', () => {
    const r = classifyPoiClick(2, 1000, 2, 1100, TH);
    expect(r.isDouble).toBe(true);
    expect(r.nextIndex).toBe(-1);
    expect(r.nextTime).toBe(0);
  });

  it('別 index は閾値内でもシングル', () => {
    const r = classifyPoiClick(2, 1000, 3, 1100, TH);
    expect(r.isDouble).toBe(false);
    expect(r.nextIndex).toBe(3);
    expect(r.nextTime).toBe(1100);
  });

  it('閾値境界: 299ms=ダブル / 300ms=シングル / 301ms=シングル (strict <)', () => {
    expect(classifyPoiClick(2, 1000, 2, 1299, TH).isDouble).toBe(true);
    expect(classifyPoiClick(2, 1000, 2, 1300, TH).isDouble).toBe(false);
    expect(classifyPoiClick(2, 1000, 2, 1301, TH).isDouble).toBe(false);
  });

  it('3 連クリック: ダブル後の reset state (-1,0) からの再クリックはシングル', () => {
    const dbl = classifyPoiClick(2, 1000, 2, 1100, TH);
    expect(dbl.isDouble).toBe(true);
    // dbl 後に保持される (-1, 0) を prev として渡す
    const third = classifyPoiClick(dbl.nextIndex, dbl.nextTime, 2, 1150, TH);
    expect(third.isDouble).toBe(false);
    expect(third.nextIndex).toBe(2);
    expect(third.nextTime).toBe(1150);
  });

  it('判定は渡された prev だけで決まる (capture-before-callback の保証)', () => {
    // map-viewer は prev をコールバック前に capture して渡す。コールバック (highlightPoi) が
    // 追跡 state を reset しても、capture 済みの prev で判定するのでダブルは保たれる。
    // 一方、reset 後の prev (-1,0) を渡すと別物としてシングルになる — 渡し方の重要性を固定する。
    expect(classifyPoiClick(2, 1000, 2, 1100, TH).isDouble).toBe(true);
    expect(classifyPoiClick(-1, 0, 2, 1100, TH).isDouble).toBe(false);
  });
});

describe('wedgeVertexCount (#275)', () => {
  it('狭い扇形でも最低 8 分割を保証する', () => {
    expect(wedgeVertexCount(0.001)).toBe(8);
    expect(wedgeVertexCount(0.05)).toBe(8); // 2*0.05/0.1 = 1 → max(8,1)
    expect(wedgeVertexCount(0.4)).toBe(8); // 2*0.4/0.1 = 8 → max(8,8)
  });

  it('0.1 rad 刻みで ceil した分割数を返す', () => {
    expect(wedgeVertexCount(Math.PI / 4)).toBe(16); // 2*0.7854/0.1 = 15.7 → 16
    expect(wedgeVertexCount(Math.PI / 2)).toBe(32); // 2*1.5708/0.1 = 31.4 → 32
  });

  it('yawTol に対し単調非減少', () => {
    let prev = 0;
    for (let t = 0.05; t < Math.PI; t += 0.05) {
      const n = wedgeVertexCount(t);
      expect(n).toBeGreaterThanOrEqual(prev);
      prev = n;
    }
  });
});

describe('wedgePointsWorld (#275)', () => {
  const cx = 5;
  const cy = -3;
  const r = 2;
  const yawCenter = 0.7;
  const yawTol = Math.PI / 6; // 30°

  it('先頭は中心点で、長さは N+2 (中心 + 弧 N+1 点)', () => {
    const pts = wedgePointsWorld(cx, cy, r, yawCenter, yawTol);
    const N = wedgeVertexCount(yawTol);
    expect(pts.length).toBe(N + 2);
    expect(pts[0]).toEqual({ x: cx, y: cy });
  });

  it('弧の始端 (i=0) は yawCenter - yawTol、終端 (i=N) は yawCenter + yawTol', () => {
    const pts = wedgePointsWorld(cx, cy, r, yawCenter, yawTol);
    const start = pts[1];
    const end = pts[pts.length - 1];
    const aStart = yawCenter - yawTol;
    const aEnd = yawCenter + yawTol;
    expect(nearly(start.x, cx + r * Math.cos(aStart))).toBe(true);
    expect(nearly(start.y, cy + r * Math.sin(aStart))).toBe(true);
    expect(nearly(end.x, cx + r * Math.cos(aEnd))).toBe(true);
    expect(nearly(end.y, cy + r * Math.sin(aEnd))).toBe(true);
  });

  it('弧上の全頂点は中心から半径 r', () => {
    const pts = wedgePointsWorld(cx, cy, r, yawCenter, yawTol);
    pts.slice(1).forEach((p) => {
      const d = Math.hypot(p.x - cx, p.y - cy);
      expect(nearly(d, r, 1e-9)).toBe(true);
    });
  });

  it('弧は yawCenter について対称に張られる', () => {
    const pts = wedgePointsWorld(cx, cy, r, yawCenter, yawTol);
    const start = pts[1];
    const end = pts[pts.length - 1];
    const angStart = Math.atan2(start.y - cy, start.x - cx);
    const angEnd = Math.atan2(end.y - cy, end.x - cx);
    // start/end の角度が yawCenter から ±yawTol
    expect(nearly(angEnd - yawCenter, yawTol, 1e-9)).toBe(true);
    expect(nearly(yawCenter - angStart, yawTol, 1e-9)).toBe(true);
  });
});

describe('worldToLatLng / latLngToWorld (#273)', () => {
  const origin = [-10, 20];
  const resolution = 0.05;

  it('world → latlng → world で round-trip する', () => {
    const cases = [[0, 0], [3.2, -7.5], [-10, 20], [100, -100]];
    cases.forEach(([x, y]) => {
      const ll = worldToLatLng(x, y, origin, resolution);
      const back = latLngToWorld(ll.lat, ll.lng, origin, resolution);
      expect(nearly(back.x, x, 1e-9)).toBe(true);
      expect(nearly(back.y, y, 1e-9)).toBe(true);
    });
  });

  it('x,y を同一 resolution で割る等方スケール (同距離の world 変位は同距離の latlng 変位)', () => {
    const base = worldToLatLng(0, 0, origin, resolution);
    const dx = worldToLatLng(1, 0, origin, resolution);
    const dy = worldToLatLng(0, 1, origin, resolution);
    const lngStep = dx.lng - base.lng;
    const latStep = dy.lat - base.lat;
    expect(nearly(lngStep, latStep, 1e-12)).toBe(true);
    expect(nearly(lngStep, 1 / resolution, 1e-9)).toBe(true);
  });
});

describe('yawFromDelta + 等方性不変条件 (#275 / PR #276 false-positive 根拠)', () => {
  it('latlng 差分から atan2 で yaw を返す', () => {
    expect(nearly(yawFromDelta(0, 1), 0)).toBe(true); // +lng → 0
    expect(nearly(yawFromDelta(1, 0), Math.PI / 2)).toBe(true); // +lat → π/2
    expect(nearly(yawFromDelta(0, -1), Math.PI)).toBe(true); // -lng → π
  });

  it('worldToLatLng が等方スケールなので、handle の latlng yaw は world yaw と厳密一致する', () => {
    // wedgePointsWorld → worldToLatLng → yawFromDelta の round-trip を、複数の
    // origin / resolution / world yaw で確認する。resolution が約分されて yaw が保たれる
    // (= PR #276 review の「yaw 座標系」high が false-positive である根拠)。
    const center = { cx: 4, cy: -6 };
    const r = 1.5;
    const configs = [
      { origin: [0, 0], resolution: 0.05 },
      { origin: [-12.3, 7.8], resolution: 0.1 },
      { origin: [100, -50], resolution: 0.25 },
    ];
    const yaws = [0, 0.3, 1.0, Math.PI / 2, 2.5, -0.7, -2.9];
    configs.forEach(({ origin, resolution }) => {
      const cLL = worldToLatLng(center.cx, center.cy, origin, resolution);
      yaws.forEach((theta) => {
        const tip = {
          x: center.cx + r * Math.cos(theta),
          y: center.cy + r * Math.sin(theta),
        };
        const tipLL = worldToLatLng(tip.x, tip.y, origin, resolution);
        const yaw = yawFromDelta(tipLL.lat - cLL.lat, tipLL.lng - cLL.lng);
        expect(nearly(yaw, theta, 1e-9)).toBe(true);
      });
    });
  });
});

describe('shouldEnablePoiDrag — ドラッグ可否 gate (#239)', () => {
  it('選択中 && 全体許可中 のみドラッグ可', () => {
    expect(shouldEnablePoiDrag(true, true)).toBe(true);
  });

  it('未選択 marker はドラッグ不可 (全体許可中でも)', () => {
    expect(shouldEnablePoiDrag(false, true)).toBe(false);
  });

  it('route 編集中 (全体不許可) は選択中でもドラッグ不可', () => {
    expect(shouldEnablePoiDrag(true, false)).toBe(false);
  });

  it('両方 false は不可', () => {
    expect(shouldEnablePoiDrag(false, false)).toBe(false);
  });

  it('falsy/truthy を厳密 boolean に正規化する', () => {
    expect(shouldEnablePoiDrag(undefined, true)).toBe(false);
    expect(shouldEnablePoiDrag(true, null)).toBe(false);
    expect(shouldEnablePoiDrag(1, 'yes')).toBe(true);
  });
});

describe('classifyYawTolerance — yaw 制約の種別 (#267 / #275)', () => {
  it('0 < yawTol < π は wedge (扇形 + 回転ハンドル対象)', () => {
    expect(classifyYawTolerance(0.001)).toBe('wedge');
    expect(classifyYawTolerance(Math.PI / 4)).toBe('wedge');
    expect(classifyYawTolerance(Math.PI - 1e-9)).toBe('wedge');
  });

  it('yawTol >= π は disc (yaw 不問 = 全方位)', () => {
    expect(classifyYawTolerance(Math.PI)).toBe('disc');
    expect(classifyYawTolerance(Math.PI + 0.5)).toBe('disc');
    expect(classifyYawTolerance(2 * Math.PI)).toBe('disc');
  });

  it('yawTol <= 0 は none (扇形なし、xy 円 outline のみ)', () => {
    expect(classifyYawTolerance(0)).toBe('none');
    expect(classifyYawTolerance(-0.5)).toBe('none');
  });

  it('NaN (tolerance.yaw 欠損で 0 化されない異常値) は none に倒す', () => {
    expect(classifyYawTolerance(NaN)).toBe('none');
  });

  it('境界 π の前後で wedge → disc に切り替わる', () => {
    expect(classifyYawTolerance(Math.PI - 1e-6)).toBe('wedge');
    expect(classifyYawTolerance(Math.PI)).toBe('disc');
  });
});

describe('shouldShowYawHandle — 回転ハンドルの出し分け (#275)', () => {
  it('選択中 && 許可中 && wedge POI の 3 条件で表示', () => {
    expect(shouldShowYawHandle(2, true, true)).toBe(true);
  });

  it('未選択 (index < 0) は出さない', () => {
    expect(shouldShowYawHandle(-1, true, true)).toBe(false);
  });

  it('route 編集中 (許可なし) は出さない', () => {
    expect(shouldShowYawHandle(2, false, true)).toBe(false);
  });

  it('wedge でない POI (disc / 扇形なし = hasWedge false) は出さない', () => {
    expect(shouldShowYawHandle(2, true, false)).toBe(false);
  });

  it('index 0 (先頭 POI) も選択中として扱う', () => {
    expect(shouldShowYawHandle(0, true, true)).toBe(true);
  });

  it('classifyYawTolerance との連携: disc/none は hasWedge=false 経由で非表示', () => {
    const hasWedge = (yawTol) => classifyYawTolerance(yawTol) === 'wedge';
    expect(shouldShowYawHandle(2, true, hasWedge(Math.PI / 3))).toBe(true); // wedge
    expect(shouldShowYawHandle(2, true, hasWedge(Math.PI))).toBe(false); // disc
    expect(shouldShowYawHandle(2, true, hasWedge(0))).toBe(false); // none
  });
});
