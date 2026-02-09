/**
 * Route editor: list display, edit form, dirty state tracking.
 * Mirrors PoiEditor structure for routes.
 */
class RouteEditor {
  constructor() {
    this.routes = [];
    this.originalRoutes = [];
    this.dirty = false;
    this.editingIndex = -1; // -1 = closed, >=0 = editing, -2 = new route
    this.selectedIndex = -1; // -1 = none selected
    this.visibleRoutes = new Set();
    this.poiNames = [];
    this.editingWaypoints = []; // working waypoint list for form

    this.onDirtyChange = null;
    this.onVisibilityChange = null;
    this.onEditingChange = null; // callback(isEditing) - fired when editing starts/stops
    this.onSelectionChange = null; // callback(index) - fired when selection changes

    // DOM references
    this.listEl = document.getElementById('route-list');
    this.formEl = document.getElementById('route-edit-form');
    this.formTitle = document.getElementById('route-form-title');
    this.btnSaveRoutes = document.getElementById('btn-save-routes');
    this.btnDiscardRoutes = document.getElementById('btn-discard-routes');
    this.btnAddRoute = document.getElementById('btn-add-route');
    this.dirtyIndicator = document.getElementById('route-dirty-indicator');

    // Form inputs
    this.inputRouteName = document.getElementById('route-name');
    this.waypointListEl = document.getElementById('route-waypoint-list');
    this.waypointSelect = document.getElementById('route-wp-select');
    this.btnAddWaypoint = document.getElementById('btn-add-waypoint');

    // Bind button events
    document.getElementById('btn-route-form-ok').addEventListener('click', () => this.formOk());
    document.getElementById('btn-route-form-cancel').addEventListener('click', () => this.formCancel());
    this.btnAddRoute.addEventListener('click', () => this.startAddRoute());
    this.btnSaveRoutes.addEventListener('click', () => this.save());
    this.btnDiscardRoutes.addEventListener('click', () => this.discard());
    this.btnAddWaypoint.addEventListener('click', () => this.addWaypointFromSelect());
  }

  /**
   * Load routes from server response and reset state.
   */
  loadRoutes(routes) {
    this.routes = JSON.parse(JSON.stringify(routes));
    this.originalRoutes = JSON.parse(JSON.stringify(routes));
    this.editingIndex = -1;
    this.selectedIndex = -1;
    this.visibleRoutes = new Set(routes.map((r) => r.name));
    this.setDirty(false);
    this.hideForm();
    this.renderList();
  }

  /**
   * Set available POI names for waypoint dropdown.
   */
  setPoiNames(names) {
    this.poiNames = names || [];
  }

  /**
   * Render the route list.
   */
  renderList() {
    this.listEl.innerHTML = '';
    this.routes.forEach((route, idx) => {
      const color = this.getRouteColor(idx);
      const item = document.createElement('div');
      let cls = 'route-item';
      if (idx === this.editingIndex) cls += ' editing';
      if (idx === this.selectedIndex) cls += ' selected';
      item.className = cls;

      item.addEventListener('click', () => {
        this.selectRoute(idx);
      });

      const cb = document.createElement('input');
      cb.type = 'checkbox';
      cb.checked = this.visibleRoutes.has(route.name);
      cb.addEventListener('click', (e) => e.stopPropagation());
      cb.addEventListener('change', () => {
        if (cb.checked) {
          this.visibleRoutes.add(route.name);
        } else {
          this.visibleRoutes.delete(route.name);
        }
        if (this.onVisibilityChange) this.onVisibilityChange();
      });

      const swatch = document.createElement('span');
      swatch.className = 'route-color-swatch';
      swatch.style.background = color;

      const textWrap = document.createElement('span');
      textWrap.className = 'route-item-text';

      const nameSpan = document.createElement('span');
      nameSpan.className = 'route-item-name';
      nameSpan.textContent = route.name;

      const wpSpan = document.createElement('span');
      wpSpan.className = 'route-item-detail';
      wpSpan.textContent = (route.waypoints || []).join(' \u2192 ');

      textWrap.appendChild(nameSpan);
      textWrap.appendChild(wpSpan);

      const actions = document.createElement('span');
      actions.className = 'route-item-actions';

      const editBtn = document.createElement('button');
      editBtn.className = 'btn-edit';
      editBtn.textContent = 'Edit';
      editBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        this.editRoute(idx);
      });
      actions.appendChild(editBtn);

      const copyBtn = document.createElement('button');
      copyBtn.className = 'btn-copy';
      copyBtn.textContent = 'Copy';
      copyBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        this.copyRoute(idx);
      });
      actions.appendChild(copyBtn);

      const delBtn = document.createElement('button');
      delBtn.className = 'btn-delete';
      delBtn.textContent = 'Del';
      delBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        this.deleteRoute(idx);
      });
      actions.appendChild(delBtn);

      item.appendChild(cb);
      item.appendChild(swatch);
      item.appendChild(textWrap);
      item.appendChild(actions);
      this.listEl.appendChild(item);
    });
  }

  getRouteColor(idx) {
    const palette = ['#2980b9', '#8e44ad', '#16a085', '#d35400', '#c0392b', '#7f8c8d'];
    if (idx < palette.length) return palette[idx];
    const n = idx - palette.length;
    const hue = (n * 47 + 30) % 360;
    return `hsl(${hue}, 55%, 45%)`;
  }

  /**
   * Open edit form for a route.
   */
  editRoute(index) {
    this.editingIndex = index;
    this.selectedIndex = index;
    const route = this.routes[index];
    this.formTitle.textContent = 'Edit Route';
    this.fillForm(route);
    this.showForm();
    this.renderList();
    this._fireEditingChange();
  }

  /**
   * Delete a route.
   */
  deleteRoute(index) {
    const name = this.routes[index].name;
    this.routes.splice(index, 1);
    this.visibleRoutes.delete(name);
    this.selectedIndex = -1;
    this.setDirty(true);
    this.hideForm();
    this.renderList();
    if (this.onSelectionChange) this.onSelectionChange(-1);
    if (this.onVisibilityChange) this.onVisibilityChange();
  }

  /**
   * Copy a route (deep clone with "_copy" suffix, open in edit form).
   */
  copyRoute(index) {
    const original = this.routes[index];
    const copy = JSON.parse(JSON.stringify(original));
    copy.name = original.name + '_copy';
    this.routes.push(copy);
    const newIdx = this.routes.length - 1;
    this.visibleRoutes.add(copy.name);
    this.editingIndex = newIdx;
    this.selectedIndex = newIdx;
    this.formTitle.textContent = 'Edit Route (Copy)';
    this.fillForm(copy);
    this.showForm();
    this.setDirty(true);
    this.renderList();
    this._fireEditingChange();
  }

  /**
   * Start adding a new route.
   */
  startAddRoute() {
    this.editingIndex = -2;
    this.formTitle.textContent = 'New Route';
    this.fillForm({ name: '', waypoints: [] });
    this.showForm();
    this.renderList();
    this._fireEditingChange();
  }

  /**
   * Fill the edit form with route data.
   */
  fillForm(route) {
    this.inputRouteName.value = route.name || '';
    this.editingWaypoints = [...(route.waypoints || [])];
    this.renderWaypointList();
    this.populateWaypointSelect();
  }

  /**
   * Populate the waypoint POI dropdown.
   */
  populateWaypointSelect() {
    this.waypointSelect.innerHTML = '<option value="">-- Select POI --</option>';
    this.poiNames.forEach((name) => {
      const opt = document.createElement('option');
      opt.value = name;
      opt.textContent = name;
      this.waypointSelect.appendChild(opt);
    });
  }

  /**
   * Render the ordered waypoint list in the form.
   */
  renderWaypointList() {
    this.waypointListEl.innerHTML = '';
    if (this.editingWaypoints.length === 0) {
      const empty = document.createElement('div');
      empty.className = 'wp-empty';
      empty.textContent = 'No waypoints added';
      this.waypointListEl.appendChild(empty);
      return;
    }
    this.editingWaypoints.forEach((wp, i) => {
      const row = document.createElement('div');
      row.className = 'wp-item';

      const num = document.createElement('span');
      num.className = 'wp-num';
      num.textContent = (i + 1) + '.';

      const name = document.createElement('span');
      name.className = 'wp-name';
      name.textContent = wp;
      // Warn if POI doesn't exist
      if (this.poiNames.length > 0 && !this.poiNames.includes(wp)) {
        name.classList.add('wp-missing');
        name.title = 'POI not found';
      }

      const btns = document.createElement('span');
      btns.className = 'wp-btns';

      const upBtn = document.createElement('button');
      upBtn.type = 'button';
      upBtn.textContent = '\u25B2';
      upBtn.title = 'Move up';
      upBtn.disabled = i === 0;
      upBtn.addEventListener('click', () => this.moveWaypoint(i, -1));

      const downBtn = document.createElement('button');
      downBtn.type = 'button';
      downBtn.textContent = '\u25BC';
      downBtn.title = 'Move down';
      downBtn.disabled = i === this.editingWaypoints.length - 1;
      downBtn.addEventListener('click', () => this.moveWaypoint(i, 1));

      const removeBtn = document.createElement('button');
      removeBtn.type = 'button';
      removeBtn.textContent = '\u00D7';
      removeBtn.title = 'Remove';
      removeBtn.className = 'wp-remove';
      removeBtn.addEventListener('click', () => this.removeWaypoint(i));

      btns.appendChild(upBtn);
      btns.appendChild(downBtn);
      btns.appendChild(removeBtn);

      row.appendChild(num);
      row.appendChild(name);
      row.appendChild(btns);
      this.waypointListEl.appendChild(row);
    });
  }

  addWaypointFromSelect() {
    const val = this.waypointSelect.value;
    if (!val) return;
    this.editingWaypoints.push(val);
    this.waypointSelect.value = '';
    this.renderWaypointList();
    this._fireEditingChange();
  }

  /**
   * Add a waypoint by POI name (e.g. from map click).
   */
  addWaypointByName(name) {
    if (this.editingIndex === -1) return;
    this.editingWaypoints.push(name);
    this.renderWaypointList();
    this._fireEditingChange();
  }

  moveWaypoint(index, dir) {
    const newIndex = index + dir;
    if (newIndex < 0 || newIndex >= this.editingWaypoints.length) return;
    const tmp = this.editingWaypoints[index];
    this.editingWaypoints[index] = this.editingWaypoints[newIndex];
    this.editingWaypoints[newIndex] = tmp;
    this.renderWaypointList();
    this._fireEditingChange();
  }

  removeWaypoint(index) {
    this.editingWaypoints.splice(index, 1);
    this.renderWaypointList();
    this._fireEditingChange();
  }

  /**
   * Confirm form edits.
   */
  formOk() {
    const name = this.inputRouteName.value.trim();
    if (!name) {
      alert('Route name is required.');
      return;
    }
    // Check uniqueness
    const dupIndex = this.routes.findIndex((r, i) => r.name === name && i !== this.editingIndex);
    if (dupIndex >= 0) {
      alert('Route name "' + name + '" already exists.');
      return;
    }
    if (this.editingWaypoints.length === 0) {
      alert('At least one waypoint is required.');
      return;
    }
    // Warn about missing POIs (non-blocking)
    if (this.poiNames.length > 0) {
      const missing = this.editingWaypoints.filter((wp) => !this.poiNames.includes(wp));
      if (missing.length > 0) {
        const proceed = confirm(
          'Warning: The following waypoints reference nonexistent POIs:\n' +
          missing.join(', ') + '\n\nContinue anyway?');
        if (!proceed) return;
      }
    }

    const route = { name: name, waypoints: [...this.editingWaypoints] };

    if (this.editingIndex === -2) {
      // New route
      this.routes.push(route);
      this.visibleRoutes.add(name);
    } else if (this.editingIndex >= 0) {
      // Update existing - handle name change for visibility
      const oldName = this.routes[this.editingIndex].name;
      if (this.visibleRoutes.has(oldName)) {
        this.visibleRoutes.delete(oldName);
        this.visibleRoutes.add(name);
      }
      this.routes[this.editingIndex] = route;
    }
    this.editingIndex = -1;
    this.setDirty(true);
    this.hideForm();
    this.renderList();
    this._fireEditingChange();
    if (this.onVisibilityChange) this.onVisibilityChange();
  }

  /**
   * Cancel form edits.
   */
  formCancel() {
    this.editingIndex = -1;
    this.hideForm();
    this.renderList();
    this._fireEditingChange();
  }

  showForm() {
    this.formEl.classList.remove('hidden');
  }

  hideForm() {
    this.formEl.classList.add('hidden');
  }

  setDirty(isDirty) {
    this.dirty = isDirty;
    this.btnSaveRoutes.disabled = !isDirty;
    this.btnDiscardRoutes.disabled = !isDirty;
    this.dirtyIndicator.textContent = isDirty ? 'Unsaved changes' : '';
    if (this.onDirtyChange) this.onDirtyChange(isDirty);
  }

  /**
   * Save routes to server.
   */
  async save() {
    if (!this.dirty) return;
    this.btnSaveRoutes.textContent = 'Saving...';
    try {
      const result = await MapoiApi.saveRoutes(this.routes);
      if (result.success) {
        this.originalRoutes = JSON.parse(JSON.stringify(this.routes));
        this.setDirty(false);
        this.btnSaveRoutes.textContent = 'Saved!';
        setTimeout(() => { this.btnSaveRoutes.textContent = 'Save'; }, 1500);
      } else {
        alert('Save failed: ' + (result.error || 'unknown error'));
        this.btnSaveRoutes.textContent = 'Save';
      }
    } catch (e) {
      alert('Save error: ' + e.message);
      this.btnSaveRoutes.textContent = 'Save';
    }
  }

  /**
   * Discard all changes, reload from original.
   */
  discard() {
    if (!this.dirty) return;
    this.routes = JSON.parse(JSON.stringify(this.originalRoutes));
    this.editingIndex = -1;
    this.visibleRoutes = new Set(this.routes.map((r) => r.name));
    this.setDirty(false);
    this.hideForm();
    this.renderList();
    if (this.onVisibilityChange) this.onVisibilityChange();
  }

  setAllVisible() {
    this.visibleRoutes = new Set(this.routes.map((r) => r.name));
    this.renderList();
    if (this.onVisibilityChange) this.onVisibilityChange();
  }

  setAllHidden() {
    this.visibleRoutes = new Set();
    this.renderList();
    if (this.onVisibilityChange) this.onVisibilityChange();
  }

  /**
   * Select a route by index (toggle: re-click deselects).
   */
  selectRoute(index) {
    this.selectedIndex = this.selectedIndex === index ? -1 : index;
    this.renderList();
    if (this.onSelectionChange) this.onSelectionChange(this.selectedIndex);
  }

  /**
   * Returns the editing route color, or null if not editing an existing route.
   */
  getEditingRouteColor() {
    if (this.editingIndex >= 0) return this.getRouteColor(this.editingIndex);
    if (this.editingIndex === -2) return this.getRouteColor(this.routes.length);
    return null;
  }

  /** @private */
  _fireEditingChange() {
    if (this.onEditingChange) this.onEditingChange(this.editingIndex !== -1);
  }
}
