/**
 * Leaflet map viewer for mapoi.
 * Displays the occupancy grid map and POI markers.
 */
class MapViewer {
  constructor(containerId) {
    this.map = L.map(containerId, {
      crs: L.CRS.Simple,
      minZoom: -3,
      maxZoom: 5,
      zoomSnap: 0.25,
      attributionControl: false,
    });

    this.imageOverlay = null;
    this.poiMarkers = [];  // { marker, poi, index }
    this.sectorLayers = []; // POI tolerance を扇形描画した Leaflet layer (#136)
    this.routeLayers = []; // Leaflet layers for route lines and labels
    this.metadata = null;
    this.tagColors = {};  // tag_name -> color, built from definitions
    this.robotMarker = null;     // Leaflet marker for robot position
    this.onMapClick = null;      // callback(worldX, worldY)
    this.onPoiClick = null;      // callback(index)
    this.onRouteClick = null;    // callback(routeIndex)
    this._poseTool = null;       // pose tool state
    this._routePolylines = [];   // { line, hitLine, arrowMarkers, labelMarkers, routeIdx, color, latlngs } for click & highlight
    this._activeRouteIdx = -1;   // 現在 active な route index (highlightRoute で更新)
    this._lastRobotPose = null;  // 最新 pose (updateRobotMarker で更新)
    this._robotConnectorLayers = []; // 現在のロボット位置 → active route 先頭 POI への connector
    this._reachedRouteSignatures = new Set(); // 到達済み route の "name|firstWaypoint" signature 集合 (page 内 sticky)

    // ロボットの実寸 (m)。connector 到達閾値とロボットマーカーサイズの両方に
    // 使う (#116)。default は TurtleBot3 burger (~0.105m) に余裕を持たせた値。
    // backend (`/api/nav/status` payload の `robot_radius`) が起動時 launch
    // param 由来の値を返すので、app.js が `setRobotRadius` で上書きする (#117)。
    // backend が古い (key 不在) 場合や上書き前の初回描画はこの default を使う。
    this.robotRadiusM = 0.15;

    this.map.on('click', (e) => {
      if (this._poseTool) {
        this._handlePoseToolClick(e.latlng);
        return;
      }
      if (this.onMapClick && this.metadata) {
        const world = this.latLngToWorld(e.latlng);
        this.onMapClick(world.x, world.y);
      }
    });

    this.map.on('mousemove', (e) => {
      if (this._poseTool && this._poseTool.phase === 'yaw') {
        this._updatePoseToolArrow(e.latlng);
      }
    });

    // zoom が変わると 1m あたりの pixel が変わるので、robot marker のピクセル
    // サイズを再計算するために再描画する (#116)。pose 未受信時は updateRobotMarker
    // が early return するので safe。
    this.map.on('zoomend', () => {
      if (this._lastRobotPose) this.updateRobotMarker(this._lastRobotPose);
    });

    // route polyline は parallel offset を pixel 単位で計算するため、zoom 変化で
    // 各 vertex の世界座標が同じでも画面表示用 displayLatlngs の位置が変わる
    // (#69 PR-2)。最後の showRoutes 引数を保持して再描画する。再描画後は
    // _activeRouteIdx に基づいて highlight も再適用する (clearRoutes は active
    // index を保持するので、新規 _routePolylines に対して再 paint すれば良い)。
    //
    // perf 最適化: 直近の showRoutes で実際に描画された route 数 (= drawable 数) が
    // 1 本以下なら offsetPx は必ず 0 で displayLatlngs === latlngs (世界座標)。
    // Leaflet の zoom 変換に任せれば足りるので redraw skip して DOM/SVG layer
    // の全削除→全作成コストを避ける (#69 round 2 low 対応)。
    // visibleNames で filter された結果でも、POI 未解決で drawable=0/1 になる
    // ケースに正しく対応する。
    this.map.on('zoomend', () => {
      if (!this._cachedRoutesArgs) return;
      if (this._routePolylines.length <= 1) return;
      const { routes, pois, visibleNames } = this._cachedRoutesArgs;
      const prevActive = this._activeRouteIdx;
      this.showRoutes(routes, pois, visibleNames);
      if (prevActive >= 0) {
        this.highlightRoute(prevActive);
      }
    });
  }

  /**
   * Convert world coordinates to Leaflet LatLng.
   * In CRS.Simple, lat=y, lng=x (in pixel space).
   * World coords: x, y  ->  pixel: (x - originX) / res, (y - originY) / res
   * But image y-axis is flipped: pixel_y measured from top, world y increases upward.
   * We use: lat = pixel_y_from_bottom = (y - originY) / res
   *         lng = (x - originX) / res
   */
  worldToLatLng(x, y) {
    const m = this.metadata;
    const lng = (x - m.origin[0]) / m.resolution;
    const lat = (y - m.origin[1]) / m.resolution;
    return L.latLng(lat, lng);
  }

  /**
   * Convert Leaflet LatLng back to world coordinates.
   */
  latLngToWorld(latlng) {
    const m = this.metadata;
    return {
      x: latlng.lng * m.resolution + m.origin[0],
      y: latlng.lat * m.resolution + m.origin[1],
    };
  }

  /**
   * Load and display a map image.
   */
  async loadMap(mapName) {
    // Fetch metadata
    this.metadata = await MapoiApi.getMapMetadata(mapName);
    if (this.metadata.error) {
      document.getElementById('map-status').textContent = 'Map not found';
      return;
    }

    const m = this.metadata;

    // Map 切替時は robot pose / connector / 到達履歴 / active 選択を全 reset。
    // 直後の fitBounds() で zoomend hook が走るため、旧 map の latlng 基準で
    // updateRobotMarker / _updateRobotConnector が実行されると、新 map 側で
    // 誤座標描画 + 旧 signature の sticky 引継ぎが起こる (Codex PR #118 round 1
    // medium)。同様に _cachedRoutesArgs に旧 map の pois が残っていると、
    // fitBounds → zoomend → showRoutes (旧 pois) で stale 描画する (#69 PR-2)。
    this._lastRobotPose = null;
    this._reachedRouteSignatures.clear();
    this._activeRouteIdx = -1;
    if (this.robotMarker) {
      this.map.removeLayer(this.robotMarker);
      this.robotMarker = null;
    }
    this._clearRobotConnector();
    // 旧 map の route layer (line / hitLine / arrow / label) も新 overlay に
    // 重なるため明示的に物理削除。app.js の loadRoutes が遅い・失敗する・
    // map 切替が連続するケースで stale polyline が見えないように同期 reset
    // (#69 PR-2 round 1 medium 対応)。clearRoutes 内で _cachedRoutesArgs = null
    // も実施されるため、再度の null 代入は冗長だが副作用は無い。
    this.clearRoutes();

    // Remove old overlay
    if (this.imageOverlay) {
      this.map.removeLayer(this.imageOverlay);
    }

    // Image bounds in CRS.Simple coordinates
    // Bottom-left: (0, 0), Top-right: (width, height) in pixel coords
    const bounds = [
      [0, 0],               // bottom-left (lat=0, lng=0)
      [m.height, m.width],  // top-right
    ];

    const imageUrl = MapoiApi.getMapImageUrl(mapName);
    this.imageOverlay = L.imageOverlay(imageUrl, bounds).addTo(this.map);
    this.map.fitBounds(bounds);

    document.getElementById('map-status').textContent =
      `${mapName} (${m.width}x${m.height}, res=${m.resolution})`;
  }

  /**
   * Set tag definitions and build color map.
   */
  setTagDefinitions(tags) {
    const palette = ['#e74c3c', '#3498db', '#27ae60', '#9b59b6', '#e67e22', '#1abc9c', '#f39c12', '#8e44ad'];
    this.tagColors = {};
    (tags || []).forEach((td, i) => {
      this.tagColors[td.name] = palette[i % palette.length];
    });
  }

  /**
   * Tag-based color for POI markers.
   */
  getPoiColor(poi) {
    const tags = poi.tags || [];
    for (const tag of tags) {
      if (this.tagColors[tag]) return this.tagColors[tag];
    }
    return '#3498db';
  }

  /**
   * Create an SVG arrow icon for a POI marker.
   * The arrow points in the yaw direction (ROS: 0=+x, pi/2=+y).
   */
  createArrowIcon(color, yaw, highlight) {
    const strokeColor = highlight ? '#e67e22' : '#fff';
    const strokeWidth = highlight ? 2.5 : 1.5;
    // SVG arrow pointing UP; CSS rotation converts yaw to visual direction.
    // yaw=0 → right(+x) → rotate 90° CW from up
    const rotDeg = 90 - (yaw * 180 / Math.PI);
    const svg = `<svg width="32" height="32" viewBox="0 0 32 32" style="transform: rotate(${rotDeg}deg);">` +
      `<path d="M16 2 L27 24 L16 18 L5 24 Z" fill="${color}" fill-opacity="0.85" stroke="${strokeColor}" stroke-width="${strokeWidth}" stroke-linejoin="round"/>` +
      `</svg>`;
    return L.divIcon({
      className: 'poi-arrow-icon',
      html: svg,
      iconSize: [32, 32],
      iconAnchor: [16, 18],  // anchor at the notch (POI position)
    });
  }

  /**
   * Display POI markers as directional arrows on the map.
   * @param {Array} pois - POI array
   * @param {Set|null} visibleSet - if provided, only show POIs whose index is in this set
   */
  showPois(pois, visibleSet) {
    this.clearPois();
    pois.forEach((poi, index) => {
      if (!poi.pose) return;
      if (visibleSet && !visibleSet.has(index)) return;

      // 扇形 (sector) を arrow icon の背景に描画 (#136)
      this._drawSectorForPoi(poi);

      const latlng = this.worldToLatLng(poi.pose.x, poi.pose.y);
      const color = this.getPoiColor(poi);
      const yaw = poi.pose.yaw || 0;

      const icon = this.createArrowIcon(color, yaw, false);
      const marker = L.marker(latlng, { icon }).addTo(this.map);

      marker.bindTooltip(poi.name || `POI ${index}`, {
        permanent: false,
        direction: 'top',
        offset: [0, -14],
      });

      marker.on('click', (e) => {
        L.DomEvent.stopPropagation(e);
        if (this.onPoiClick) {
          this.onPoiClick(index);
        }
      });

      this.poiMarkers.push({ marker, poi, index, color, yaw });
    });
  }

  /**
   * Highlight a specific POI marker.
   */
  highlightPoi(index) {
    this.poiMarkers.forEach((item) => {
      const isHighlighted = item.index === index;
      const icon = this.createArrowIcon(item.color, item.yaw, isHighlighted);
      item.marker.setIcon(icon);
      if (isHighlighted) {
        item.marker.setZIndexOffset(1000);
      } else {
        item.marker.setZIndexOffset(0);
      }
    });
  }

  /**
   * Enable two-click pose tool (RViz-style).
   * Phase 'position': click sets X/Y, then enters 'yaw' phase.
   * Phase 'yaw': mousemove shows arrow preview, click sets yaw, then enters 'position' phase.
   * @param {object} callbacks - { onPositionSet(x,y), onYawSet(yaw) }
   * @param {L.LatLng|null} initialLatLng - if provided, start in 'yaw' phase with this position
   */
  enablePoseTool(callbacks, initialLatLng) {
    this.disablePoseTool();
    this._poseTool = {
      phase: initialLatLng ? 'yaw' : 'position',
      startLatLng: initialLatLng || null,
      circle: null,
      line: null,
      arrowHead: null,
      callbacks,
    };
    if (initialLatLng) {
      this._poseTool.circle = L.circleMarker(initialLatLng, {
        radius: 6, color: '#e67e22', fillColor: '#e67e22', fillOpacity: 0.8, weight: 2,
      }).addTo(this.map);
    }
    this.map.getContainer().style.cursor = 'crosshair';
  }

  /**
   * Disable the pose tool and clean up visuals.
   */
  disablePoseTool() {
    if (!this._poseTool) return;
    this._clearPoseToolVisuals();
    this._poseTool = null;
    this.map.getContainer().style.cursor = '';
  }

  /** @private */
  _handlePoseToolClick(latlng) {
    const pt = this._poseTool;
    if (pt.phase === 'position') {
      this._clearPoseToolVisuals();
      pt.startLatLng = latlng;
      const world = this.latLngToWorld(latlng);

      pt.circle = L.circleMarker(latlng, {
        radius: 6, color: '#e67e22', fillColor: '#e67e22', fillOpacity: 0.8, weight: 2,
      }).addTo(this.map);

      pt.phase = 'yaw';
      if (pt.callbacks.onPositionSet) {
        pt.callbacks.onPositionSet(world.x, world.y);
      }
    } else if (pt.phase === 'yaw') {
      const start = pt.startLatLng;
      const yaw = Math.atan2(latlng.lat - start.lat, latlng.lng - start.lng);

      this._clearPoseToolVisuals();
      pt.phase = 'position';

      if (pt.callbacks.onYawSet) {
        pt.callbacks.onYawSet(yaw);
      }
    }
  }

  /** @private */
  _updatePoseToolArrow(latlng) {
    const pt = this._poseTool;
    if (!pt || !pt.startLatLng) return;
    const start = pt.startLatLng;

    // Dashed line from position to cursor
    if (pt.line) {
      pt.line.setLatLngs([start, latlng]);
    } else {
      pt.line = L.polyline([start, latlng], {
        color: '#e67e22', weight: 3, dashArray: '6, 4',
      }).addTo(this.map);
    }

    // Arrowhead at tip
    const yaw = Math.atan2(latlng.lat - start.lat, latlng.lng - start.lng);
    const rotDeg = 90 - (yaw * 180 / Math.PI);
    const svg = `<svg width="24" height="24" viewBox="0 0 24 24" style="transform: rotate(${rotDeg}deg);">` +
      `<path d="M12 2 L20 18 L12 14 L4 18 Z" fill="#e67e22" fill-opacity="0.9" stroke="#fff" stroke-width="1.5" stroke-linejoin="round"/>` +
      `</svg>`;
    const icon = L.divIcon({
      className: 'pose-tool-arrow',
      html: svg,
      iconSize: [24, 24],
      iconAnchor: [12, 14],
    });

    if (pt.arrowHead) {
      pt.arrowHead.setLatLng(latlng);
      pt.arrowHead.setIcon(icon);
    } else {
      pt.arrowHead = L.marker(latlng, { icon, interactive: false }).addTo(this.map);
    }
  }

  /** @private */
  _clearPoseToolVisuals() {
    const pt = this._poseTool;
    if (!pt) return;
    if (pt.circle) { this.map.removeLayer(pt.circle); pt.circle = null; }
    if (pt.line) { this.map.removeLayer(pt.line); pt.line = null; }
    if (pt.arrowHead) { this.map.removeLayer(pt.arrowHead); pt.arrowHead = null; }
  }

  /**
   * Update a POI marker's position on the map.
   */
  updatePoiMarkerPosition(index, x, y) {
    const item = this.poiMarkers.find((m) => m.index === index);
    if (!item) return;
    item.marker.setLatLng(this.worldToLatLng(x, y));
  }

  /**
   * Update a POI marker's arrow rotation to reflect a new yaw.
   */
  updatePoiMarkerYaw(index, yaw) {
    const item = this.poiMarkers.find((m) => m.index === index);
    if (!item) return;
    item.yaw = yaw;
    const icon = this.createArrowIcon(item.color, yaw, true);
    item.marker.setIcon(icon);
  }

  /**
   * Remove all POI markers from the map.
   */
  clearPois() {
    this.poiMarkers.forEach((item) => {
      this.map.removeLayer(item.marker);
    });
    this.poiMarkers = [];
    // POI 扇形 layer も同時に clear (#136)
    if (this.sectorLayers) {
      this.sectorLayers.forEach((l) => this.map.removeLayer(l));
    }
    this.sectorLayers = [];
  }

  /**
   * POI の tolerance を 円 (xy 判定領域) + 扇形 (yaw 制約) の重ね描きで描画する (#136 / #179)。
   *
   * - 円 outline: 半径 = `tolerance.xy` (xy 進入判定の境界、yaw 不問)、
   *   `L.polyline` で 36 角形を構成して常時描画 (細実線・薄め)
   * - 扇形: `tolerance.yaw` が `0 < yaw < π` の時のみ重ね描き (`L.polygon`)
   *   - waypoint: 塗り扇形 (`fillOpacity` 高)
   *   - landmark: 中抜き扇形 (`fillOpacity = 0`、stroke のみ)
   *   - その他 (旧 event 等): 描画しない (#70 で別途整理)
   * - pause tag があれば xy 円沿いに dot pattern overlay (#179: 旧 dash → dot 形式)
   *
   * `tolerance.xy <= 0` または対象 tag 無しなら no-op。
   *
   * `L.circle` ではなく polyline で実装するのは、`L.circle` の radius が pixel/meter 換算に
   * `crs` 依存の挙動を持つ一方、扇形は world 座標を `worldToLatLng` で変換して描画するため。
   * 同じ計算式で円と扇形の弧を生成すれば視覚的整合 (両者の弧が完全一致) が保証される。
   */
  _drawSectorForPoi(poi) {
    if (!poi.pose) return;
    if (!poi.tolerance || typeof poi.tolerance.xy !== 'number' || poi.tolerance.xy <= 0) return;

    const tags = poi.tags || [];
    const isWaypoint = tags.includes('waypoint');
    const isLandmark = tags.includes('landmark');
    const isPause = tags.includes('pause');
    if (!isWaypoint && !isLandmark) return;

    const r = poi.tolerance.xy;
    const yawCenter = poi.pose.yaw || 0;
    const yawTol = (typeof poi.tolerance.yaw === 'number') ? poi.tolerance.yaw : 0;
    const hasYawConstraint = yawTol > 0 && yawTol < Math.PI;

    const color = this.getPoiColor(poi);

    // 円の頂点列 (36 角形 = 10 度刻み)。i = N_CIRCLE で起点に戻り polyline が自然に閉じる。
    const N_CIRCLE = 36;
    const circlePoints = [];
    for (let i = 0; i <= N_CIRCLE; i += 1) {
      const a = (2 * Math.PI * i) / N_CIRCLE;
      const wx = poi.pose.x + r * Math.cos(a);
      const wy = poi.pose.y + r * Math.sin(a);
      circlePoints.push(this.worldToLatLng(wx, wy));
    }

    // xy 判定領域 (円 outline): 控えめな細実線で常時表示 (#179)
    const circle = L.polyline(circlePoints, {
      color,
      weight: 1,
      opacity: 0.4,
      interactive: false,
    }).addTo(this.map);
    circle.bringToBack();
    this.sectorLayers.push(circle);

    // 扇形 (yaw 制約): 0 < yaw < π の時のみ重ね描き (#179)。
    // 完全円 (yaw 不問) なら扇形 = 円で情報冗長になるため省略。
    if (hasYawConstraint) {
      const totalAngle = 2 * yawTol;
      const N = Math.max(8, Math.ceil(totalAngle / 0.1));
      const startAngle = yawCenter - yawTol;
      const centerLatLng = this.worldToLatLng(poi.pose.x, poi.pose.y);
      const sectorPoints = [centerLatLng];  // 中心点 → 弧 → 中心 (polygon で自動閉じる)
      for (let i = 0; i <= N; i += 1) {
        const a = startAngle + (totalAngle * i) / N;
        const wx = poi.pose.x + r * Math.cos(a);
        const wy = poi.pose.y + r * Math.sin(a);
        sectorPoints.push(this.worldToLatLng(wx, wy));
      }
      const sector = L.polygon(sectorPoints, {
        color,
        weight: 1,
        opacity: 0.7,
        fillColor: color,
        fillOpacity: isWaypoint ? 0.25 : 0,  // waypoint=塗り、landmark=中抜き
        interactive: false,
      }).addTo(this.map);
      sector.bringToBack();
      this.sectorLayers.push(sector);
    }

    // pause overlay: xy 円沿いに dot pattern を重ね描き (#179)。
    // 旧 dashArray '6, 4' (cycle 比 60% on, 1.5:1 dash) は dot として識別しづらく潰れて見える feedback
    // (#178 PR コメント) を受け、'2, 6' (cycle 比 25% on, 1:3 on:off) で短い dot + 大きい gap に変更。
    // RViz 側 (dot 0.02m / step 0.10m, 20% on / 1:4) と厳密比率は僅差だが共に sparse dot で整合。
    if (isPause) {
      const pauseOutline = L.polyline(circlePoints, {
        color,
        weight: 4,
        opacity: 0.9,
        dashArray: '2, 6',
        lineCap: 'round',  // dot を丸く描画して角ばった dash 感を消す
        interactive: false,
      }).addTo(this.map);
      pauseOutline.bringToBack();
      this.sectorLayers.push(pauseOutline);
    }
  }

  /**
   * Get the color for a route by its index.
   */
  getRouteColor(routeIdx) {
    const palette = ['#2980b9', '#8e44ad', '#16a085', '#d35400', '#c0392b', '#7f8c8d'];
    if (routeIdx < palette.length) return palette[routeIdx];
    // HSL fallback: distribute hues avoiding palette hues
    const usedHues = [207, 282, 168, 24, 4, 210]; // approx hues of palette colors
    const n = routeIdx - palette.length;
    const hue = (n * 47 + 30) % 360; // step by golden-angle-ish offset, start at 30
    return `hsl(${hue}, 55%, 45%)`;
  }

  /**
   * Create an SVG chevron (>) direction marker.
   * Default orientation: tip points UP (^). Rotated by rotDeg.
   * Open V-shape with no fill — lightweight and unambiguous.
   */
  createRouteDirectionSvg(color, rotDeg, size) {
    const s = size || 18;
    return `<svg width="${s}" height="${s}" viewBox="0 0 16 16" style="transform: rotate(${rotDeg}deg);">` +
      `<path d="M4 12 L8 4 L12 12" fill="none" stroke="${color}" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/>` +
      `<path d="M4 12 L8 4 L12 12" fill="none" stroke="#fff" stroke-width="5" stroke-linecap="round" stroke-linejoin="round" opacity="0.4"/>` +
      `<path d="M4 12 L8 4 L12 12" fill="none" stroke="${color}" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/>` +
      `</svg>`;
  }

  /**
   * Compute CSS rotation (degrees) for a route direction marker (teardrop pointing UP by default).
   * from/to are Leaflet LatLng objects.
   */
  routeDirectionDeg(from, to) {
    const screenAngle = Math.atan2(to.lat - from.lat, to.lng - from.lng) * (180 / Math.PI);
    return 90 - screenAngle;
  }

  /**
   * Offset a polyline by `offsetPx` pixels perpendicular to each segment.
   *
   * Leaflet `latLngToContainerPoint` で pixel 空間に変換し、純粋幾何関数
   * `MapoiGeometry.offsetPointsPx` (mapoi_webui/web/js/geometry.js) に委譲、
   * 結果を `containerPointToLatLng` で latlng に戻す。pure helper 側で
   * canonical normal / bisector / cap / 退化 fallback 等の幾何ロジックを担当
   * (#129 で単体テストの対象として切り出し)。
   *
   * pixel 計算なので zoom 変化で再計算が必要 → showRoutes は zoomend で cached
   * args を使って呼び直す (#69 PR-2)。
   *
   * @param {Array<L.LatLng>} latlngs - 入力経路 (raw)
   * @param {number} offsetPx - 正負で canonical 法線に対する offset 方向、0 なら no-op
   * @returns {Array<L.LatLng>}
   */
  _offsetLatLngs(latlngs, offsetPx) {
    if (!latlngs || latlngs.length < 2 || offsetPx === 0) return latlngs;
    const points = latlngs.map((ll) => this.map.latLngToContainerPoint(ll));
    const offsetPoints = MapoiGeometry.offsetPointsPx(points, offsetPx);
    return offsetPoints.map((pt) => this.map.containerPointToLatLng(L.point(pt.x, pt.y)));
  }

  /**
   * Display routes on the map as polylines with waypoint order labels.
   * @param {Array} routes - [{name, waypoints: [poiName, ...]}, ...]
   * @param {Array} pois - POI array to resolve waypoint names to coordinates
   * @param {Set|null} visibleNames - if provided, only show routes whose name is in this set
   */
  showRoutes(routes, pois, visibleNames) {
    this.clearRoutes();
    if (!this.metadata || !routes || !pois) return;

    // zoomend で再描画するため最後の引数を保持。
    // - visibleNames: 軽量なので `new Set` で snapshot し、呼び出し側の Set 変更が
    //   redraw に漏れない (#69 round 1 low 対応)。
    // - routes / pois: 配列が大きくなり得る (POI 多数) ため deep copy しない。代わりに
    //   呼び出し側 (app.js) は showRoutes 後に in-place mutation しない前提を取る。
    //   in-place 更新ではなく新配列で再 showRoutes する運用 (現状そう)。仕様変更で
    //   in-place mutation が出てきた場合は、ここを snapshot に切り替える。
    this._cachedRoutesArgs = {
      routes,
      pois,
      visibleNames: visibleNames ? new Set(visibleNames) : null,
    };

    // Build name -> poi lookup
    const poiByName = {};
    pois.forEach((poi) => {
      if (poi.name) poiByName[poi.name] = poi;
    });

    // 共通区間で polyline が完全重複する視認性問題への (b) parallel offset 対策 (#69 PR-2)。
    //
    // 1) drawable な route (rawLatlngs.length >= 2、つまり POI 2 点以上に解決でき
    //    実際に描画される route) を先に列挙する。
    //    visible だが waypoint 不足 / POI 未解決で描画されない route を offset lane
    //    数に含めると、画面上の route 数 1 でも offsetPx > 0 で route が中心から
    //    ずれる + zoomend skip 条件が崩れる (Codex round 2 low 対応)。
    // 2) drawable 数で中心揃え offset index を付与し、各 route に offsetPx を割り当てる。
    //    表示中 3 route → offset index は -1, 0, +1 → offsetPx は -STEP, 0, +STEP。
    //    route 数 1 のみだと offset 0 で _offsetLatLngs は no-op、原点描画と完全一致。
    const drawableEntries = [];
    routes.forEach((route, routeIdx) => {
      if (visibleNames && !visibleNames.has(route.name)) return;
      const rawLatlngs = [];
      // orders は rawLatlngs と並行配列。各 vertex に「元 waypoint 配列での
      // 1-based index」を保持する。missing POI が途中にある場合 (例: 2 番目が
      // 未解決で skip) でも、地図上のラベルが route editor 側の waypoint 順番
      // (1, 3, 4 等) を保つ (#69 round 4 low 対応、Codex review)。
      const orders = [];
      let firstWaypointName = '';
      (route.waypoints || []).forEach((wpName, i) => {
        const poi = poiByName[wpName];
        if (poi && poi.pose) {
          rawLatlngs.push(this.worldToLatLng(poi.pose.x, poi.pose.y));
          orders.push(i + 1);
          if (!firstWaypointName) firstWaypointName = wpName;
        }
      });
      if (rawLatlngs.length < 2) return;
      drawableEntries.push({ route, routeIdx, rawLatlngs, orders, firstWaypointName });
    });
    const totalDrawable = drawableEntries.length;
    const OFFSET_STEP_PX = 6;

    drawableEntries.forEach(({ route, routeIdx, rawLatlngs, orders, firstWaypointName }, visibleIdx) => {
      const offsetPx = (visibleIdx - (totalDrawable - 1) / 2) * OFFSET_STEP_PX;
      const color = this.getRouteColor(routeIdx);

      // 表示用に offset を適用 (offsetPx === 0 なら同じ配列参照が返る)
      const displayLatlngs = this._offsetLatLngs(rawLatlngs, offsetPx);

      // Draw polyline (dashed)
      const line = L.polyline(displayLatlngs, {
        color: color,
        weight: 3,
        opacity: 0.7,
        dashArray: '8, 6',
      }).addTo(this.map);

      line.bindTooltip(route.name || `Route ${routeIdx + 1}`, {
        sticky: true,
        direction: 'top',
        offset: [0, -8],
      });

      line.bringToBack();
      this.routeLayers.push(line);

      // Invisible wider hit area for easier clicking
      const hitLine = L.polyline(displayLatlngs, {
        color: '#000',
        weight: 16,
        opacity: 0,
        interactive: true,
      }).addTo(this.map);

      hitLine.on('click', (e) => {
        L.DomEvent.stopPropagation(e);
        if (this.onRouteClick) this.onRouteClick(routeIdx);
      });

      hitLine.bringToBack();
      this.routeLayers.push(hitLine);

      const arrowMarkers = [];
      const labelMarkers = [];

      // Draw teardrop direction markers along segments (offset 後の座標で配置)
      for (let i = 0; i < displayLatlngs.length - 1; i++) {
        const from = displayLatlngs[i];
        const to = displayLatlngs[i + 1];
        const midLat = (from.lat + to.lat) / 2;
        const midLng = (from.lng + to.lng) / 2;
        const rotDeg = this.routeDirectionDeg(from, to);

        const arrowIcon = L.divIcon({
          className: 'route-arrow',
          html: this.createRouteDirectionSvg(color, rotDeg, 16),
          iconSize: [16, 16],
          iconAnchor: [8, 8],
        });
        const arrowMarker = L.marker([midLat, midLng], {
          icon: arrowIcon,
          interactive: false,
        }).addTo(this.map);
        this.routeLayers.push(arrowMarker);
        arrowMarkers.push(arrowMarker);
      }

      // Draw order labels at each waypoint (offset 後位置でラベル重なりも軽減)。
      // 番号は元 waypoint 配列の 1-based index (orders[i]) を使い、missing POI
      // が skip されても route editor 側の順番と整合する (#69 round 4 low 対応)。
      displayLatlngs.forEach((latlng, i) => {
        const order = orders[i];
        const icon = L.divIcon({
          className: 'route-order-label',
          html: `<span style="background: ${color};">${order}</span>`,
          iconSize: [22, 22],
          iconAnchor: [-6, 28],
        });
        const labelMarker = L.marker(latlng, {
          icon: icon,
          interactive: false,
        }).addTo(this.map);
        this.routeLayers.push(labelMarker);
        labelMarkers.push(labelMarker);
      });

      this._routePolylines.push({
        line, hitLine, arrowMarkers, labelMarkers,
        routeIdx, routeName: route.name || '', firstWaypointName,
        color,
        // latlngs: raw (POI 物理座標)。robot connector の到達判定など物理距離を
        //   評価する用途で使う
        // displayLatlngs: offset 後 (画面表示用)。connector の視覚端点や zoom
        //   再描画時の再計算で使う
        latlngs: rawLatlngs, displayLatlngs,
      });
    });
  }

  /**
   * Remove all route layers from the map.
   */
  clearRoutes() {
    this.routeLayers.forEach((layer) => {
      this.map.removeLayer(layer);
    });
    this.routeLayers = [];
    this._routePolylines = [];
    // cached args は次回 showRoutes で上書きされるので残しても害は無いが、
    // route データが本当に空 (cleared) のとき zoom した瞬間に古い args で
    // 再描画されないよう、clearRoutes が「外から明示的に空にされた」ケースで
    // 初期化することにした。showRoutes 内部から clearRoutes を呼ぶ場合は直後に
    // _cachedRoutesArgs が再設定されるので race にはならない。
    this._cachedRoutesArgs = null;
    this.clearEditingRoutePreview();
    this._clearRobotConnector();
    // 到達履歴 (_reachedRouteSignatures) は clearRoutes では reset しない。
    // clearRoutes は visibility toggle / 編集 preview 出入り等の表示再描画でも
    // 呼ばれるため、ここで reset すると sticky 性が壊れる (Codex round 2 medium)。
    // signature は "routeName|firstWaypoint" 複合 ID で、先頭 POI 変更や
    // map 切替で別 route が同名で再生成された場合は signature が変わる
    // ため、自動的に fresh 評価される (Codex round 3 medium 対応)。
  }

  /**
   * Highlight a specific route on the map.
   *
   * - routeIdx >= 0: active route を太線・実線・不透明にし、他 route は dim (薄い + 細線 + dashed)
   *   にして焦点を作る。矢印・順序ラベルにも opacity を適用。
   * - routeIdx === -1: 全 route を default 表示 (中程度の太さ + dashed + 通常 opacity) に戻す。
   *
   * 複数 route が同じ POI を経由して polyline が重なる視認性問題への (d) active highlight 対策。
   */
  highlightRoute(routeIdx) {
    // 描画済みの _routePolylines に対象 routeIdx が実在する時のみ active state に入る。
    // 非表示 / 未描画 / waypoint 不足等で対象 entry がない場合に全 route を dim にしない (Codex review medium #105)。
    const activeExists = routeIdx >= 0
      && this._routePolylines.some((item) => item.routeIdx === routeIdx);
    this._routePolylines.forEach((item) => {
      const isActive = item.routeIdx === routeIdx;
      if (activeExists && isActive) {
        item.line.setStyle({ weight: 5, opacity: 1.0, dashArray: null });
        this._setMarkersOpacity(item.arrowMarkers, 1.0);
        this._setMarkersOpacity(item.labelMarkers, 1.0);
      } else if (activeExists && !isActive) {
        item.line.setStyle({ weight: 2, opacity: 0.25, dashArray: '8, 6' });
        this._setMarkersOpacity(item.arrowMarkers, 0.25);
        this._setMarkersOpacity(item.labelMarkers, 0.25);
      } else {
        item.line.setStyle({ weight: 3, opacity: 0.7, dashArray: '8, 6' });
        this._setMarkersOpacity(item.arrowMarkers, 1.0);
        this._setMarkersOpacity(item.labelMarkers, 1.0);
      }
    });
    this._activeRouteIdx = routeIdx;
    this._updateRobotConnector();
    // route 切替で marker 色も追従させる (pose 未受信時は updateRobotMarker が
    // early return するので safe)。
    if (this._lastRobotPose) {
      this.updateRobotMarker(this._lastRobotPose);
    }
  }

  _setMarkersOpacity(markers, opacity) {
    if (!markers) return;
    markers.forEach((m) => {
      if (m && typeof m.setOpacity === 'function') {
        m.setOpacity(opacity);
      }
    });
  }

  /**
   * Show a live preview of the route being edited.
   * @param {Array} waypoints - waypoint name array (editingWaypoints)
   * @param {Array} pois - POI array to resolve names
   * @param {string} color - route color
   */
  showEditingRoutePreview(waypoints, pois, color) {
    this.clearEditingRoutePreview();
    if (!this.metadata || !waypoints || !pois) return;

    const poiByName = {};
    pois.forEach((poi) => { if (poi.name) poiByName[poi.name] = poi; });

    const latlngs = [];
    const resolved = [];
    waypoints.forEach((wpName, i) => {
      const poi = poiByName[wpName];
      if (poi && poi.pose) {
        const ll = this.worldToLatLng(poi.pose.x, poi.pose.y);
        latlngs.push(ll);
        resolved.push({ latlng: ll, order: i + 1 });
      }
    });

    this._editingPreviewLayers = [];

    // Draw solid polyline (editing preview is solid, not dashed)
    if (latlngs.length >= 2) {
      const line = L.polyline(latlngs, {
        color: color,
        weight: 5,
        opacity: 0.9,
      }).addTo(this.map);
      line.bringToBack();
      this._editingPreviewLayers.push(line);

      // Directional teardrop markers
      for (let i = 0; i < latlngs.length - 1; i++) {
        const from = latlngs[i];
        const to = latlngs[i + 1];
        const midLat = (from.lat + to.lat) / 2;
        const midLng = (from.lng + to.lng) / 2;
        const rotDeg = this.routeDirectionDeg(from, to);
        const arrowIcon = L.divIcon({
          className: 'route-arrow',
          html: this.createRouteDirectionSvg(color, rotDeg, 20),
          iconSize: [20, 20],
          iconAnchor: [10, 10],
        });
        const arrowMarker = L.marker([midLat, midLng], {
          icon: arrowIcon,
          interactive: false,
        }).addTo(this.map);
        this._editingPreviewLayers.push(arrowMarker);
      }
    }

    // Order labels
    resolved.forEach(({ latlng, order }) => {
      const icon = L.divIcon({
        className: 'route-order-label',
        html: `<span style="background: ${color}; box-shadow: 0 0 6px ${color};">${order}</span>`,
        iconSize: [22, 22],
        iconAnchor: [-6, 28],
      });
      const labelMarker = L.marker(latlng, {
        icon: icon,
        interactive: false,
      }).addTo(this.map);
      this._editingPreviewLayers.push(labelMarker);
    });
  }

  /**
   * Remove editing route preview layers.
   */
  clearEditingRoutePreview() {
    if (this._editingPreviewLayers) {
      this._editingPreviewLayers.forEach((layer) => {
        this.map.removeLayer(layer);
      });
    }
    this._editingPreviewLayers = [];
  }

  /**
   * Create an SVG icon for the robot marker (colored circle with direction arrow).
   * @param {number} yaw - heading in radians
   * @param {string} color - circle fill color (default cyan); 内側の方向矢印は白固定
   * @param {number} sizePx - icon の outer width/height (px)。viewBox は固定 36 のままで
   *   内部 path / circle 座標は変えず、outer サイズだけリスケールする
   */
  createRobotIcon(yaw, color = '#00bcd4', sizePx = 36) {
    const rotDeg = 90 - (yaw * 180 / Math.PI);
    const half = sizePx / 2;
    const svg = `<svg width="${sizePx}" height="${sizePx}" viewBox="0 0 36 36" style="transform: rotate(${rotDeg}deg);">` +
      `<circle cx="18" cy="18" r="12" fill="${color}" fill-opacity="0.7" stroke="#fff" stroke-width="2"/>` +
      `<path d="M18 8 L24 24 L18 19 L12 24 Z" fill="#fff" fill-opacity="0.9" stroke="none"/>` +
      `</svg>`;
    return L.divIcon({
      className: 'robot-icon',
      html: svg,
      iconSize: [sizePx, sizePx],
      iconAnchor: [half, half],
    });
  }

  /**
   * Update `robotRadiusM` from backend (`/api/nav/status` の `robot_radius`)。
   *
   * 値は launch param `robot_radius` 由来で起動後は変わらない想定だが、
   * polling 経路で来るので idempotent に書く。次の `updateRobotMarker` で
   * marker サイズと connector 閾値が新値で再計算される。
   *
   * 不正値 (NaN / 非有限 / 0以下) は ignore して default / 前回値を維持する。
   * connector 到達判定で `< robotRadiusM` を使うため 0 以下を許すと到達認識が
   * 永久に立たない。
   */
  setRobotRadius(meters) {
    if (typeof meters !== 'number' || !Number.isFinite(meters) || meters <= 0) {
      return;
    }
    if (this.robotRadiusM === meters) return;
    this.robotRadiusM = meters;
  }

  /**
   * Compute the marker icon size (pixels) so it reflects the robot's real
   * world radius at the current map zoom level (#116).
   *
   * `worldToLatLng` + `latLngToContainerPoint` で 1m が今の zoom で何 pixel
   * になるかを実測する。CRS / resolution に依存せず robust。低 zoom で
   * marker が点になるのを防ぐため下限 16px を入れる。
   *
   * Note: 戻り値は icon の **outer width/height (= bounding box)** で、
   * 内部の見える circle は viewBox の `r=12 / 36 ≒ 2/3` の比率になる
   * (矢印 path 含む余白のため)。outer = robot diameter として扱うので
   * 見える円自体は実 robot より少し小さく描かれるが、heading 矢印を含めた
   * 総占有面積として robot 実寸に揃う設計 (Codex PR #118 round 1 low の
   * 意図明示)。見える円を robot 実寸に厳密に合わせる場合は別 PR で sizePx
   * を 1.5 倍するか、SVG 内部を r=18 に書き換える。
   *
   * pose / metadata 未受信時は default 36px (constructor 時 marker のため)。
   */
  _resolveRobotMarkerSizePx() {
    const MIN_PX = 16;
    const DEFAULT_PX = 36;
    if (!this.metadata || !this._lastRobotPose) return DEFAULT_PX;
    const center = this.worldToLatLng(this._lastRobotPose.x, this._lastRobotPose.y);
    const edge = this.worldToLatLng(
      this._lastRobotPose.x + this.robotRadiusM, this._lastRobotPose.y,
    );
    const pxRadius = this.map.latLngToContainerPoint(center)
      .distanceTo(this.map.latLngToContainerPoint(edge));
    const diameterPx = pxRadius * 2;
    return Math.max(MIN_PX, diameterPx);
  }

  /**
   * Resolve the marker color to use based on the current active route selection.
   * - active route が選択されていてかつ entry が `_routePolylines` に実在する場合は
   *   その色 (route 色と marker 色を揃える)
   * - 未選択 / entry 不在 (waypoint <2 / 非表示等) は default cyan
   *   PR #105 の `activeExists` ガードと同じ趣旨
   */
  _resolveRobotMarkerColor() {
    if (this._activeRouteIdx < 0) return '#00bcd4';
    const item = this._routePolylines.find(
      (it) => it.routeIdx === this._activeRouteIdx,
    );
    return item ? item.color : '#00bcd4';
  }

  /**
   * Update or hide the robot marker on the map.
   * @param {Object|null} pose - {x, y, yaw} or null to hide
   */
  updateRobotMarker(pose) {
    if (!pose || !this.metadata) {
      if (this.robotMarker) {
        this.map.removeLayer(this.robotMarker);
        this.robotMarker = null;
      }
      this._lastRobotPose = null;
      this._updateRobotConnector();
      return;
    }
    // _resolveRobotMarkerSizePx は this._lastRobotPose を読むため、
    // metadata / pose を使う前に最新値を保存しておく。
    this._lastRobotPose = pose;
    const latlng = this.worldToLatLng(pose.x, pose.y);
    const icon = this.createRobotIcon(
      pose.yaw || 0,
      this._resolveRobotMarkerColor(),
      this._resolveRobotMarkerSizePx(),
    );
    if (this.robotMarker) {
      this.robotMarker.setLatLng(latlng);
      this.robotMarker.setIcon(icon);
    } else {
      this.robotMarker = L.marker(latlng, {
        icon,
        interactive: false,
        zIndexOffset: 2000,
      }).addTo(this.map);
    }
    this._updateRobotConnector();
  }

  /**
   * Draw or refresh a directional connector from the current robot position
   * to the first waypoint of the active route.
   *
   * Guards (no-op の条件):
   * - active route が選択されていない (`_activeRouteIdx === -1`)
   * - robot pose 未取得 (`_lastRobotPose === null`)
   * - active route の polyline entry が `_routePolylines` に無い
   *   (waypoint 不足 / 非表示等。PR #105 の activeExists と同じガード)
   * - 当 route の signature (`routeName|firstWaypointName`) が
   *   `_reachedRouteSignatures` に登録済み。複合 ID にすることで
   *   - 表示再描画 (clearRoutes / visibility toggle / 編集 cancel 等) では
   *     sticky 性を維持
   *   - 同名のまま先頭 waypoint を変更した、map 切替で別 route が同名で
   *     再生成された等のケースは signature が変わるため fresh 評価される
   *     (Codex round 3 medium 対応)
   *
   * 距離が `this.robotRadiusM` (#116) 未満になった瞬間に signature を Set に
   * 登録し、以降同 signature の間は描画しない (page 内 sticky)。
   *
   * polyline は active route と同色 + dashed (本線 polyline と差別化) で、
   * 中点に既存の route 矢印 SVG を再利用して方向を示す。
   */
  _updateRobotConnector() {
    this._clearRobotConnector();
    if (this._activeRouteIdx < 0) return;
    if (!this._lastRobotPose || !this.metadata) return;
    const item = this._routePolylines.find(
      (it) => it.routeIdx === this._activeRouteIdx,
    );
    if (!item || !item.latlngs || item.latlngs.length === 0) return;
    const signature = this._routeSignature(item);
    if (signature && this._reachedRouteSignatures.has(signature)) return;

    const robotLatLng = this.worldToLatLng(
      this._lastRobotPose.x, this._lastRobotPose.y,
    );
    // 視覚端点は offset 後 (displayLatlngs) を使い、parallel offset で動いた
    // polyline 始点に矢印が刺さるようにする (#69 PR-2)。displayLatlngs が無い
    // 古い entry / no-offset 時は latlngs にフォールバック。
    const firstLatLngVisual = (item.displayLatlngs && item.displayLatlngs[0])
      || item.latlngs[0];

    // 先頭 POI に到達済みなら Set に登録して以降描かない。
    // 到達判定は offset で動かない物理座標 (latlngs[0] = POI 実位置) で評価する
    // (offset 後位置で判定すると同じ POI に到達しているのに route ごとに到達タイ
    //  ミングがずれる)。
    const firstWorld = this.latLngToWorld(item.latlngs[0]);
    const dx = this._lastRobotPose.x - firstWorld.x;
    const dy = this._lastRobotPose.y - firstWorld.y;
    if (Math.hypot(dx, dy) < this.robotRadiusM) {
      if (signature) {
        this._reachedRouteSignatures.add(signature);
      }
      return;
    }

    const line = L.polyline([robotLatLng, firstLatLngVisual], {
      color: item.color,
      weight: 3,
      opacity: 0.8,
      dashArray: '4, 6',
      interactive: false,
    }).addTo(this.map);
    line.bringToBack();
    this._robotConnectorLayers.push(line);

    const midLat = (robotLatLng.lat + firstLatLngVisual.lat) / 2;
    const midLng = (robotLatLng.lng + firstLatLngVisual.lng) / 2;
    const rotDeg = this.routeDirectionDeg(robotLatLng, firstLatLngVisual);
    const arrowIcon = L.divIcon({
      className: 'route-arrow',
      html: this.createRouteDirectionSvg(item.color, rotDeg, 18),
      iconSize: [18, 18],
      iconAnchor: [9, 9],
    });
    const arrowMarker = L.marker([midLat, midLng], {
      icon: arrowIcon,
      interactive: false,
    }).addTo(this.map);
    this._robotConnectorLayers.push(arrowMarker);
  }

  _clearRobotConnector() {
    if (!this._robotConnectorLayers) return;
    this._robotConnectorLayers.forEach((layer) => {
      this.map.removeLayer(layer);
    });
    this._robotConnectorLayers = [];
  }

  /**
   * route の到達履歴 sticky 用の identity signature。
   * routeName 単独だと「同名のまま先頭 waypoint 差替え / map 切替で別 route 再生成」
   * で stale entry が残る (Codex round 3 medium)。先頭 waypoint 名を加えた複合 ID
   * にすることで、これらの identity 変化で signature が変わり fresh 評価される。
   * routeName / firstWaypointName のいずれかが空なら identity 確定不可で空返し。
   */
  _routeSignature(item) {
    if (!item || !item.routeName || !item.firstWaypointName) return '';
    return `${item.routeName}|${item.firstWaypointName}`;
  }
}
