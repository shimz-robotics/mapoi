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
    this.onMapClick = null;      // callback(worldX, worldY)
    this.onPoiClick = null;      // callback(index)

    this.map.on('click', (e) => {
      if (this.onMapClick && this.metadata) {
        const world = this.latLngToWorld(e.latlng);
        this.onMapClick(world.x, world.y);
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
}
