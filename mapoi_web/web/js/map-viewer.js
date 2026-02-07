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
    });

    this.imageOverlay = null;
    this.poiMarkers = [];  // { marker, poi, index }
    this.metadata = null;
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
   * Tag-based color for POI markers.
   */
  getPoiColor(poi) {
    const tags = poi.tags || [];
    if (tags.includes('initial_pose')) return '#27ae60';
    if (tags.includes('waypoint')) return '#3498db';
    if (tags.includes('goal')) return '#e74c3c';
    if (tags.includes('audio_info')) return '#9b59b6';
    return '#f39c12';
  }

  /**
   * Display POI markers on the map.
   */
  showPois(pois) {
    this.clearPois();
    pois.forEach((poi, index) => {
      if (!poi.pose) return;
      const latlng = this.worldToLatLng(poi.pose.x, poi.pose.y);
      const color = this.getPoiColor(poi);
      const marker = L.circleMarker(latlng, {
        radius: 8,
        fillColor: color,
        color: '#fff',
        weight: 2,
        fillOpacity: 0.8,
      }).addTo(this.map);

      marker.bindTooltip(poi.name || `POI ${index}`, {
        permanent: false,
        direction: 'top',
        offset: [0, -10],
      });

      marker.on('click', (e) => {
        L.DomEvent.stopPropagation(e);
        if (this.onPoiClick) {
          this.onPoiClick(index);
        }
      });

      this.poiMarkers.push({ marker, poi, index });
    });
  }

  /**
   * Highlight a specific POI marker.
   */
  highlightPoi(index) {
    this.poiMarkers.forEach((item) => {
      if (item.index === index) {
        item.marker.setStyle({ weight: 4, color: '#e67e22' });
        item.marker.bringToFront();
      } else {
        item.marker.setStyle({ weight: 2, color: '#fff' });
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
}
