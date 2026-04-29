/**
 * Main application controller.
 * Wires together MapViewer, PoiEditor, RouteEditor, and API calls.
 */
(async function() {
  const mapViewer = new MapViewer('map');
  const poiEditor = new PoiEditor();
  const routeEditor = new RouteEditor();

  const mapSelect = document.getElementById('map-select');
  const navGoalSelect = document.getElementById('nav-goal-select');
  const navRouteSelect = document.getElementById('nav-route-select');
  const navInitialPoseSelect = document.getElementById('nav-initialpose-select');
  const navStatusDot = document.getElementById('nav-status-dot');
  const navStatusText = document.getElementById('nav-status-text');
  let currentMap = '';

  // --- Tag editor state ---
  let tagDirty = false;
  let editingTags = []; // working copy of tags (both system + custom)
  const tagList = document.getElementById('tag-list');
  const tagAddName = document.getElementById('tag-add-name');
  const tagAddDesc = document.getElementById('tag-add-desc');
  const btnTagAdd = document.getElementById('btn-tag-add');
  const btnSaveTags = document.getElementById('btn-save-tags');
  const btnDiscardTags = document.getElementById('btn-discard-tags');
  const tagDirtyIndicator = document.getElementById('tag-dirty-indicator');

  function renderTagList() {
    tagList.innerHTML = '';
    editingTags.forEach((tag, i) => {
      const div = document.createElement('div');
      div.className = 'tag-item ' + (tag.is_system ? 'tag-item-system' : 'tag-item-custom');
      const icon = tag.is_system ? '<span class="tag-lock">&#128274;</span>' : '';
      const typeLabel = tag.is_system ? '<span class="tag-type-label tag-type-system">system</span>' : '<span class="tag-type-label tag-type-custom">custom</span>';
      const deleteBtn = tag.is_system
        ? '<button class="tag-delete-btn" disabled title="System tags cannot be deleted">&#10005;</button>'
        : `<button class="tag-delete-btn" data-index="${i}" title="Delete tag">&#10005;</button>`;
      div.innerHTML = `${icon}<span class="tag-item-name">${tag.name}</span><span class="tag-item-desc">${tag.description || ''}</span>${typeLabel}${deleteBtn}`;
      tagList.appendChild(div);
    });
    // Attach delete handlers
    tagList.querySelectorAll('.tag-delete-btn:not(:disabled)').forEach((btn) => {
      btn.addEventListener('click', (e) => {
        const idx = parseInt(e.currentTarget.dataset.index);
        editingTags.splice(idx, 1);
        setTagDirty(true);
        renderTagList();
      });
    });
  }

  function setTagDirty(dirty) {
    tagDirty = dirty;
    btnSaveTags.disabled = !dirty;
    btnDiscardTags.disabled = !dirty;
    tagDirtyIndicator.textContent = dirty ? 'unsaved changes' : '';
  }

  btnTagAdd.addEventListener('click', () => {
    const name = tagAddName.value.trim();
    const desc = tagAddDesc.value.trim();
    if (!name) { tagAddName.focus(); return; }
    // Check duplicate (case-insensitive)
    if (editingTags.some((t) => t.name.toLowerCase() === name.toLowerCase())) {
      alert('Tag "' + name + '" already exists.');
      return;
    }
    editingTags.push({ name, description: desc, is_system: false });
    tagAddName.value = '';
    tagAddDesc.value = '';
    setTagDirty(true);
    renderTagList();
  });

  btnSaveTags.addEventListener('click', async () => {
    const customTags = editingTags
      .filter((t) => !t.is_system)
      .map((t) => ({ name: t.name, description: t.description }));
    const result = await MapoiApi.saveCustomTags(customTags);
    if (result.error) {
      alert('Failed to save tags: ' + result.error);
      return;
    }
    setTagDirty(false);
    await loadTagDefinitions();
  });

  btnDiscardTags.addEventListener('click', async () => {
    if (tagDirty && !confirm('Discard unsaved tag changes?')) return;
    setTagDirty(false);
    await loadTagDefinitions();
  });

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
    // Update tag editor working copy
    editingTags = tags.map((t) => ({ ...t }));
    renderTagList();
  }

  // --- Load POIs ---
  async function loadPois() {
    const data = await MapoiApi.getPois();
    poiEditor.loadPois(data.pois || []);
    mapViewer.showPois(poiEditor.pois, poiEditor.visiblePois);
    populateNavGoalSelect(data.pois || []);
    populateInitialPoseSelect(data.pois || []);
    updateRouteEditorPoiNames();
  }

  function updateRouteEditorPoiNames() {
    // landmark POI は Nav2 navigation goal にできない reference 専用 (#85)。
    // route waypoint candidate からは除外、route.landmarks candidate に振り分ける (#143)。
    const navigablePois = MapoiPoiFilter.filterRouteWaypointCandidates(poiEditor.pois);
    routeEditor.setPoiNames(navigablePois.map((p) => p.name));
    const landmarkPois = (poiEditor.pois || []).filter((p) => MapoiPoiFilter.hasLandmarkTag(p));
    routeEditor.setLandmarkNames(landmarkPois.map((p) => p.name));
  }

  // --- Load routes ---
  async function loadRoutes() {
    const data = await MapoiApi.getRoutes();
    routeEditor.loadRoutes(data.routes || []);
    redrawRoutes();
    populateNavRouteSelect(routeEditor.routes);
  }

  function redrawRoutes() {
    if (routeEditor.editingIndex !== -1) {
      // Hide all other routes while editing; show only the editing preview
      mapViewer.clearRoutes();
      const color = routeEditor.getEditingRouteColor() || '#2980b9';
      mapViewer.showEditingRoutePreview(
        routeEditor.editingWaypoints, poiEditor.pois, color
      );
    } else {
      mapViewer.showRoutes(routeEditor.routes, poiEditor.pois, routeEditor.visibleRoutes);
    }
  }

  // --- Map selector change ---
  mapSelect.addEventListener('change', () => {
    if (poiEditor.dirty || routeEditor.dirty || tagDirty) {
      if (!confirm('Unsaved changes will be lost. Switch map?')) {
        mapSelect.value = currentMap;
        return;
      }
    }
    switchMap(mapSelect.value);
  });

  // --- Wire callbacks ---

  // Helper: enable pose tool for the currently editing POI
  function enablePoseToolForEditing() {
    const callbacks = {
      onPositionSet: (x, y) => {
        poiEditor.updateFormPosition(x, y);
        if (poiEditor.editingIndex >= 0) {
          mapViewer.updatePoiMarkerPosition(poiEditor.editingIndex, x, y);
        }
      },
      onYawSet: (yaw) => {
        poiEditor.updateFormYaw(yaw);
        if (poiEditor.editingIndex >= 0) {
          mapViewer.updatePoiMarkerYaw(poiEditor.editingIndex, yaw);
        }
      },
    };

    if (poiEditor.editingIndex === -2) {
      // New POI: placing click already set position → start in yaw phase
      const formX = parseFloat(poiEditor.inputX.value) || 0;
      const formY = parseFloat(poiEditor.inputY.value) || 0;
      mapViewer.enablePoseTool(callbacks, mapViewer.worldToLatLng(formX, formY));
    } else {
      // Existing POI: start in position phase (click 1 = position, click 2 = yaw)
      mapViewer.enablePoseTool(callbacks);
    }
  }

  // Map click — only used for placing new POI (pose tool handles editing clicks)
  mapViewer.onMapClick = (x, y) => {
    if (poiEditor.placingMode) {
      poiEditor.placeNewPoi(x, y);
      enablePoseToolForEditing();
    }
  };

  // POI marker click on map
  mapViewer.onPoiClick = (index) => {
    // If route editing is active, add the clicked POI as a waypoint
    if (routeEditor.editingIndex !== -1) {
      const poi = poiEditor.pois[index];
      if (poi && poi.name) {
        routeEditor.addWaypointByName(poi.name);
      }
      return;
    }
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
    // Enable pose tool when editing, disable when form closes
    if (poiEditor.editingIndex >= 0) {
      enablePoseToolForEditing();
    } else {
      mapViewer.disablePoseTool();
    }
    syncNavGoalSelect();
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
    updateRouteEditorPoiNames();
    if (!isDirty) {
      mapViewer.showPois(poiEditor.pois, poiEditor.visiblePois);
      loadRoutes();
    }
  };

  // Route click on map
  mapViewer.onRouteClick = (routeIdx) => {
    // Ignore during editing
    if (routeEditor.editingIndex !== -1) return;
    routeEditor.selectRoute(routeIdx);
  };

  // Route selection change — highlight on map
  routeEditor.onSelectionChange = (index) => {
    mapViewer.highlightRoute(index);
    syncNavRouteSelect();
  };

  // Route editing state change — redraw routes with editing preview
  routeEditor.onEditingChange = () => {
    redrawRoutes();
    // editRoute / copyRoute は selectedIndex を直接変えるが onSelectionChange を
    // 発火しないため、ここで highlightRoute を再適用して MapViewer 側の
    // _activeRouteIdx と同期する。これがないと「route A 選択 → route B Edit →
    // Cancel」で sidebar は B selected でも _activeRouteIdx は A のままになり、
    // robot connector が古い route の先頭 POI を指し続ける (PR #107 Codex round
    // 2/3 medium 残課題)。
    if (routeEditor.selectedIndex >= 0) {
      mapViewer.highlightRoute(routeEditor.selectedIndex);
    }
    syncNavRouteSelect();
  };

  // Route visibility toggle
  routeEditor.onVisibilityChange = () => {
    redrawRoutes();
    if (routeEditor.selectedIndex >= 0) {
      mapViewer.highlightRoute(routeEditor.selectedIndex);
    }
  };

  // Route dirty state change — refresh routes on map and nav select after save/discard
  routeEditor.onDirtyChange = (isDirty) => {
    redrawRoutes();
    if (routeEditor.selectedIndex >= 0) {
      mapViewer.highlightRoute(routeEditor.selectedIndex);
    }
    if (!isDirty) {
      populateNavRouteSelect(routeEditor.routes);
      // populate で options が rebuild されるが、選択中なら value を再同期して
      // ユーザーの選択 state を保持する。
      syncNavRouteSelect();
    }
  };

  // --- Section toggles ---
  function setupSectionToggle(toggleBtnId, bodyEl, initialOpen = true) {
    const btn = document.getElementById(toggleBtnId);
    let open = initialOpen;
    if (!open) {
      bodyEl.style.display = 'none';
      btn.innerHTML = '&#9654;';
    }
    btn.addEventListener('click', () => {
      open = !open;
      bodyEl.style.display = open ? '' : 'none';
      btn.innerHTML = open ? '&#9660;' : '&#9654;';
    });
  }
  setupSectionToggle('btn-route-toggle', document.getElementById('route-body'));
  setupSectionToggle('btn-poi-toggle', document.getElementById('poi-body'));
  setupSectionToggle('btn-tag-toggle', document.getElementById('tag-body'), false);

  // --- All / None buttons ---
  // Routes
  document.getElementById('btn-route-all').addEventListener('click', () => {
    routeEditor.setAllVisible();
  });
  document.getElementById('btn-route-none').addEventListener('click', () => {
    routeEditor.setAllHidden();
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
    // landmark POI は reference 専用、Nav2 navigation candidate から除外 (#85)。
    MapoiPoiFilter.filterWaypointCandidates(pois).forEach((p) => {
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

  // Nav dropdowns auto-fill from POI/route selection (#111)。
  // 選択中の route 名 / POI 名を Navigation グループの dropdown に反映し、
  // 「選択 → 再選択 → Run/Go」の重複操作を 1 ステップ減らす。
  // 選択解除 (-1) では dropdown を維持する (再 nav 時の再利用性のため)。
  //
  // 重要: 該当 name が dropdown options に無い場合 (例: goal タグ無し POI、
  // 未保存の Copy 直後の route 等)、`select.value = missingName` は silent
  // no-op ではなく value を空 ("") に落とし前値を消してしまう。前値維持を
  // 守るため options 存在チェックしてから代入する (PR #114 Codex round 1
  // medium 対応)。
  function _hasOption(selectEl, name) {
    return Array.from(selectEl.options).some((o) => o.value === name);
  }

  function syncNavRouteSelect() {
    const idx = routeEditor.selectedIndex;
    if (idx < 0) return;
    const name = routeEditor.routes[idx]?.name;
    if (!name) return;
    if (_hasOption(navRouteSelect, name)) navRouteSelect.value = name;
  }

  function syncNavGoalSelect() {
    const idx = poiEditor.selectedIndex;
    if (idx < 0) return;
    const name = poiEditor.pois[idx]?.name;
    if (!name) return;
    if (_hasOption(navGoalSelect, name)) navGoalSelect.value = name;
  }

  function populateInitialPoseSelect(pois) {
    navInitialPoseSelect.innerHTML = '<option value="">-- Select POI --</option>';
    // landmark + initial_pose は排他、initial pose candidate から除外 (#85)。
    MapoiPoiFilter.filterInitialPoseCandidates(pois).forEach((p) => {
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
    paused: 'Paused',
  };

  function updateNavStatus(status, target) {
    const s = status || 'idle';
    navStatusDot.className = 'nav-status-dot nav-status-' + s;
    let label = statusLabels[s] || s;
    if (target && s !== 'idle') {
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
        // robot_radius は launch param 由来 (#117)。古い backend では key
        // 不在になるが setRobotRadius が型 check で no-op にする。値が来る
        // 前に updateRobotMarker が走っても map-viewer の default 0.15 で
        // 描画されるので safe。
        mapViewer.setRobotRadius(data.robot_radius);
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

  document.getElementById('btn-nav-pause').addEventListener('click', async () => {
    await MapoiApi.navPause();
    updateNavStatus('paused', '');
  });

  document.getElementById('btn-nav-resume').addEventListener('click', async () => {
    await MapoiApi.navResume();
    updateNavStatus('navigating', '');
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
