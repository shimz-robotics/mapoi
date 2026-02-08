/**
 * Main application controller.
 * Wires together MapViewer, PoiEditor, and API calls.
 */
(async function() {
  const mapViewer = new MapViewer('map');
  const poiEditor = new PoiEditor();

  const mapSelect = document.getElementById('map-select');
  let currentMap = '';

  // --- Load maps list ---
  async function loadMaps() {
    const data = await MapoiApi.getMaps();
    mapSelect.innerHTML = '';
    data.maps.forEach((name) => {
      const opt = document.createElement('option');
      opt.value = name;
      opt.textContent = name;
      if (name === data.current_map) opt.selected = true;
      mapSelect.appendChild(opt);
    });
    currentMap = data.current_map;
    await switchMap(currentMap);
  }

  // --- Switch map ---
  async function switchMap(mapName) {
    currentMap = mapName;
    await mapViewer.loadMap(mapName);
    await loadPois();
  }

  // --- Load POIs ---
  async function loadPois() {
    const data = await MapoiApi.getPois();
    poiEditor.loadPois(data.pois || []);
    mapViewer.showPois(poiEditor.pois);
  }

  // --- Map selector change ---
  mapSelect.addEventListener('change', () => {
    if (poiEditor.dirty) {
      if (!confirm('Unsaved changes will be lost. Switch map?')) {
        mapSelect.value = currentMap;
        return;
      }
    }
    switchMap(mapSelect.value);
  });

  // --- Wire callbacks ---

  // Map click
  mapViewer.onMapClick = (x, y) => {
    if (poiEditor.placingMode) {
      poiEditor.placeNewPoi(x, y);
    } else if (poiEditor.editingIndex !== -1) {
      poiEditor.updateFormPosition(x, y);
    }
  };

  // POI marker click on map
  mapViewer.onPoiClick = (index) => {
    poiEditor.selectPoi(index);
    mapViewer.highlightPoi(index);
  };

  // POI selection in list
  poiEditor.onSelectionChange = (index) => {
    mapViewer.highlightPoi(index);
    // Refresh markers when dirty (POIs changed)
    if (poiEditor.dirty) {
      mapViewer.showPois(poiEditor.pois);
      if (index >= 0) mapViewer.highlightPoi(index);
    }
  };

  // Dirty state change — refresh map markers after save/discard
  poiEditor.onDirtyChange = (isDirty) => {
    if (!isDirty) {
      mapViewer.showPois(poiEditor.pois);
    }
  };

  // --- Initialize ---
  await loadMaps();
})();
