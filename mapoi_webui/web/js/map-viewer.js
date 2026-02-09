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
    this.routeLayers = []; // Leaflet layers for route lines and labels
    this.metadata = null;
    this.tagColors = {};  // tag_name -> color, built from definitions
    this.robotMarker = null;     // Leaflet marker for robot position
    this.onMapClick = null;      // callback(worldX, worldY)
    this.onPoiClick = null;      // callback(index)
    this._poseTool = null;       // pose tool state

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
    return '#f39c12';
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
  }

  /**
   * Get the color for a route by its index.
   */
  getRouteColor(routeIdx) {
    const palette = ['#8e44ad', '#2980b9', '#16a085', '#c0392b', '#d35400', '#7f8c8d'];
    return palette[routeIdx % palette.length];
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

    // Build name -> poi lookup
    const poiByName = {};
    pois.forEach((poi) => {
      if (poi.name) poiByName[poi.name] = poi;
    });

    routes.forEach((route, routeIdx) => {
      // Filter by visible set
      if (visibleNames && !visibleNames.has(route.name)) return;

      const color = this.getRouteColor(routeIdx);
      const waypoints = route.waypoints || [];

      // Resolve waypoint names to LatLng positions
      const latlngs = [];
      const resolved = []; // { latlng, order }
      waypoints.forEach((wpName, i) => {
        const poi = poiByName[wpName];
        if (poi && poi.pose) {
          const ll = this.worldToLatLng(poi.pose.x, poi.pose.y);
          latlngs.push(ll);
          resolved.push({ latlng: ll, order: i + 1 });
        }
      });

      if (latlngs.length < 2) return;

      // Draw polyline (dashed)
      const line = L.polyline(latlngs, {
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

      // Draw arrowhead markers along segments
      for (let i = 0; i < latlngs.length - 1; i++) {
        const from = latlngs[i];
        const to = latlngs[i + 1];
        // Place arrow at midpoint of segment
        const midLat = (from.lat + to.lat) / 2;
        const midLng = (from.lng + to.lng) / 2;
        const angle = Math.atan2(to.lng - from.lng, to.lat - from.lat) * (180 / Math.PI);

        const arrowIcon = L.divIcon({
          className: 'route-arrow',
          html: `<div style="transform: rotate(${90 - angle}deg); color: ${color}; font-size: 16px; font-weight: bold; line-height: 1;">&#9654;</div>`,
          iconSize: [16, 16],
          iconAnchor: [8, 8],
        });
        const arrowMarker = L.marker([midLat, midLng], {
          icon: arrowIcon,
          interactive: false,
        }).addTo(this.map);
        this.routeLayers.push(arrowMarker);
      }

      // Draw order labels at each waypoint
      resolved.forEach(({ latlng, order }) => {
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
  }

  /**
   * Create an SVG icon for the robot marker (red circle with direction arrow).
   */
  createRobotIcon(yaw) {
    const rotDeg = 90 - (yaw * 180 / Math.PI);
    const svg = `<svg width="36" height="36" viewBox="0 0 36 36" style="transform: rotate(${rotDeg}deg);">` +
      `<circle cx="18" cy="18" r="12" fill="#00bcd4" fill-opacity="0.7" stroke="#fff" stroke-width="2"/>` +
      `<path d="M18 8 L24 24 L18 19 L12 24 Z" fill="#fff" fill-opacity="0.9" stroke="none"/>` +
      `</svg>`;
    return L.divIcon({
      className: 'robot-icon',
      html: svg,
      iconSize: [36, 36],
      iconAnchor: [18, 18],
    });
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
      return;
    }
    const latlng = this.worldToLatLng(pose.x, pose.y);
    const icon = this.createRobotIcon(pose.yaw || 0);
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
  }
}
