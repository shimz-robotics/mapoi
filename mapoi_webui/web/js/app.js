/**
 * Main application controller.
 * Wires together MapViewer, PoiEditor, and API calls.
 */
(async function() {
  const mapViewer = new MapViewer('map');
  const poiEditor = new PoiEditor();

  const mapSelect = document.getElementById('map-select');
  const routeListEl = document.getElementById('route-list');
  const poiListEl = document.getElementById('poi-list');
  const navGoalSelect = document.getElementById('nav-goal-select');
  const navRouteSelect = document.getElementById('nav-route-select');
  const navInitialPoseSelect = document.getElementById('nav-initialpose-select');
  const navStatusDot = document.getElementById('nav-status-dot');
  const navStatusText = document.getElementById('nav-status-text');
  let currentMap = '';
  let currentRoutes = [];       // raw route data from API
  let visibleRoutes = new Set(); // route names with checkbox checked

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
    await loadTagDefinitions();
    await loadPois();
    await loadRoutes();
  }

  // --- Load tag definitions ---
  async function loadTagDefinitions() {
    const data = await MapoiApi.getTagDefinitions();
    const tags = data.tags || [];
    poiEditor.setTagDefinitions(tags);
    mapViewer.setTagDefinitions(tags);
  }

  // --- Load POIs ---
  async function loadPois() {
    const data = await MapoiApi.getPois();
    poiEditor.loadPois(data.pois || []);
    mapViewer.showPois(poiEditor.pois, poiEditor.visiblePois);
    populateNavGoalSelect(data.pois || []);
    populateInitialPoseSelect(data.pois || []);
  }

  // --- Load routes ---
  async function loadRoutes() {
    const data = await MapoiApi.getRoutes();
    currentRoutes = data.routes || [];
    visibleRoutes = new Set(currentRoutes.map((r) => r.name));
    renderRouteList();
    redrawRoutes();
    populateNavRouteSelect(currentRoutes);
  }

  function redrawRoutes() {
    mapViewer.showRoutes(currentRoutes, poiEditor.pois, visibleRoutes);
  }

  function renderRouteList() {
    routeListEl.innerHTML = '';
    currentRoutes.forEach((route, idx) => {
      const color = mapViewer.getRouteColor(idx);
      const item = document.createElement('label');
      item.className = 'route-item';

      const cb = document.createElement('input');
      cb.type = 'checkbox';
      cb.checked = visibleRoutes.has(route.name);
      cb.addEventListener('change', () => {
        if (cb.checked) {
          visibleRoutes.add(route.name);
        } else {
          visibleRoutes.delete(route.name);
        }
        redrawRoutes();
      });

      const swatch = document.createElement('span');
      swatch.className = 'route-color-swatch';
      swatch.style.background = color;

      const nameSpan = document.createElement('span');
      nameSpan.className = 'route-item-name';
      nameSpan.textContent = route.name;

      const wpSpan = document.createElement('span');
      wpSpan.className = 'route-item-detail';
      wpSpan.textContent = (route.waypoints || []).join(' → ');

      const textWrap = document.createElement('span');
      textWrap.className = 'route-item-text';
      textWrap.appendChild(nameSpan);
      textWrap.appendChild(wpSpan);

      item.appendChild(cb);
      item.appendChild(swatch);
      item.appendChild(textWrap);
      routeListEl.appendChild(item);
    });
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
      mapViewer.showPois(poiEditor.pois, poiEditor.visiblePois);
      if (index >= 0) mapViewer.highlightPoi(index);
    }
  };

  // POI visibility toggle
  poiEditor.onVisibilityChange = () => {
    mapViewer.showPois(poiEditor.pois, poiEditor.visiblePois);
    if (poiEditor.selectedIndex >= 0) {
      mapViewer.highlightPoi(poiEditor.selectedIndex);
    }
  };

  // Dirty state change — refresh map markers and routes after save/discard
  poiEditor.onDirtyChange = (isDirty) => {
    if (!isDirty) {
      mapViewer.showPois(poiEditor.pois, poiEditor.visiblePois);
      loadRoutes();
    }
  };

  // --- Section toggles ---
  function setupSectionToggle(toggleBtnId, bodyEl) {
    const btn = document.getElementById(toggleBtnId);
    let open = true;
    btn.addEventListener('click', () => {
      open = !open;
      bodyEl.style.display = open ? '' : 'none';
      btn.innerHTML = open ? '&#9660;' : '&#9654;';
    });
  }
  setupSectionToggle('btn-route-toggle', routeListEl);
  setupSectionToggle('btn-poi-toggle', poiListEl);

  // --- All / None buttons ---
  // Routes
  document.getElementById('btn-route-all').addEventListener('click', () => {
    visibleRoutes = new Set(currentRoutes.map((r) => r.name));
    renderRouteList();
    redrawRoutes();
  });
  document.getElementById('btn-route-none').addEventListener('click', () => {
    visibleRoutes = new Set();
    renderRouteList();
    redrawRoutes();
  });
  // POIs
  document.getElementById('btn-poi-all').addEventListener('click', () => {
    poiEditor.setAllVisible();
  });
  document.getElementById('btn-poi-none').addEventListener('click', () => {
    poiEditor.setAllHidden();
  });

  // --- Navigation controls ---
  function populateNavGoalSelect(pois) {
    navGoalSelect.innerHTML = '<option value="">-- Select POI --</option>';
    pois
      .filter((p) => {
        const tags = (p.tags || []).map((t) => t.toLowerCase());
        return tags.includes('goal') || tags.includes('waypoint');
      })
      .forEach((p) => {
        const opt = document.createElement('option');
        opt.value = p.name;
        opt.textContent = p.name;
        navGoalSelect.appendChild(opt);
      });
  }

  function populateNavRouteSelect(routes) {
    navRouteSelect.innerHTML = '<option value="">-- Select Route --</option>';
    routes.forEach((r) => {
      const opt = document.createElement('option');
      opt.value = r.name;
      opt.textContent = r.name;
      navRouteSelect.appendChild(opt);
    });
  }

  function populateInitialPoseSelect(pois) {
    navInitialPoseSelect.innerHTML = '<option value="">-- Select POI --</option>';
    pois.forEach((p) => {
      const opt = document.createElement('option');
      opt.value = p.name;
      opt.textContent = p.name;
      navInitialPoseSelect.appendChild(opt);
    });
  }

  const statusLabels = {
    idle: 'Idle',
    navigating: 'Navigating',
    succeeded: 'Succeeded',
    aborted: 'Aborted',
    canceled: 'Canceled',
  };

  function updateNavStatus(status, target) {
    const s = status || 'idle';
    navStatusDot.className = 'nav-status-dot nav-status-' + s;
    let label = statusLabels[s] || s;
    if (target && s === 'navigating') {
      label += ': ' + target;
    }
    navStatusText.textContent = label;
  }

  let navPollingTimer = null;
  function startNavStatusPolling() {
    if (navPollingTimer) return;
    navPollingTimer = setInterval(async () => {
      try {
        const data = await MapoiApi.navStatus();
        updateNavStatus(data.status, data.target);
        mapViewer.updateRobotMarker(data.robot_pose);
      } catch (e) {
        // ignore fetch errors
      }
    }, 1000);
  }

  document.getElementById('btn-nav-go').addEventListener('click', async () => {
    const name = navGoalSelect.value;
    if (!name) return;
    await MapoiApi.navGoal(name);
    updateNavStatus('navigating', name);
  });

  document.getElementById('btn-nav-run').addEventListener('click', async () => {
    const name = navRouteSelect.value;
    if (!name) return;
    await MapoiApi.navRoute(name);
    updateNavStatus('navigating', name);
  });

  document.getElementById('btn-nav-setpose').addEventListener('click', async () => {
    const name = navInitialPoseSelect.value;
    if (!name) return;
    await MapoiApi.navInitialPose(name);
  });

  document.getElementById('btn-nav-stop').addEventListener('click', async () => {
    await MapoiApi.navCancel();
    updateNavStatus('canceled', '');
  });

  setupSectionToggle('btn-nav-toggle', document.getElementById('nav-body'));
  startNavStatusPolling();

  // --- Initialize ---
  await loadMaps();
})();
