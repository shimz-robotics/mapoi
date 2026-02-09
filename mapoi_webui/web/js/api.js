/**
 * REST API client for mapoi_webui backend.
 */
const MapoiApi = {
  async getMaps() {
    const res = await fetch('/api/maps');
    return res.json();
  },

  async getMapMetadata(mapName) {
    const res = await fetch(`/api/maps/${encodeURIComponent(mapName)}/metadata`);
    return res.json();
  },

  getMapImageUrl(mapName) {
    return `/api/maps/${encodeURIComponent(mapName)}/image`;
  },

  async getPois() {
    const res = await fetch('/api/pois');
    return res.json();
  },

  async savePois(pois) {
    const res = await fetch('/api/pois', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ pois }),
    });
    return res.json();
  },

  async getTagDefinitions() {
    const res = await fetch('/api/tag_definitions');
    return res.json();
  },

  async getRoutes() {
    const res = await fetch('/api/routes');
    return res.json();
  },

  async navGoal(poiName) {
    const res = await fetch('/api/nav/goal', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ poi_name: poiName }),
    });
    return res.json();
  },

  async navRoute(routeName) {
    const res = await fetch('/api/nav/route', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ route_name: routeName }),
    });
    return res.json();
  },

  async navCancel() {
    const res = await fetch('/api/nav/cancel', { method: 'POST' });
    return res.json();
  },

  async navInitialPose(poiName) {
    const res = await fetch('/api/nav/initialpose', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ poi_name: poiName }),
    });
    return res.json();
  },

  async navStatus() {
    const res = await fetch('/api/nav/status');
    return res.json();
  },
};
