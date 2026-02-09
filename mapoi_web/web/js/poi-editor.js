/**
 * POI editor: list display, edit form, dirty state tracking.
 */
class PoiEditor {
  constructor() {
    this.pois = [];          // current POI list (working copy)
    this.originalPois = [];  // snapshot for dirty detection
    this.dirty = false;
    this.selectedIndex = -1;
    this.editingIndex = -1;  // -1 = closed, >=0 = editing existing, -2 = new POI
    this.placingMode = false;
    this.visiblePois = new Set(); // indices of visible POIs

    this.tagDefinitions = [];  // [{name, description, is_system}, ...]
    this.onDirtyChange = null;     // callback(isDirty)
    this.onSelectionChange = null; // callback(index)
    this.onPlacingModeChange = null; // callback(isPlacing)
    this.onVisibilityChange = null;  // callback(visibleSet)

    // DOM references
    this.listEl = document.getElementById('poi-list');
    this.formEl = document.getElementById('poi-edit-form');
    this.formTitle = document.getElementById('form-title');
    this.btnSave = document.getElementById('btn-save');
    this.btnDiscard = document.getElementById('btn-discard');
    this.btnAddPoi = document.getElementById('btn-add-poi');
    this.dirtyIndicator = document.getElementById('dirty-indicator');

    // Form inputs
    this.inputName = document.getElementById('poi-name');
    this.inputX = document.getElementById('poi-x');
    this.inputY = document.getElementById('poi-y');
    this.inputYaw = document.getElementById('poi-yaw');
    this.inputRadius = document.getElementById('poi-radius');
    this.inputTags = document.getElementById('poi-tags');
    this.inputDescription = document.getElementById('poi-description');

    // Bind button events
    document.getElementById('btn-form-ok').addEventListener('click', () => this.formOk());
    document.getElementById('btn-form-cancel').addEventListener('click', () => this.formCancel());
    this.btnAddPoi.addEventListener('click', () => this.startAddPoi());
    this.btnSave.addEventListener('click', () => this.save());
    this.btnDiscard.addEventListener('click', () => this.discard());
  }

  /**
   * Load POIs from server response and reset state.
   */
  loadPois(pois) {
    this.pois = JSON.parse(JSON.stringify(pois));
    this.originalPois = JSON.parse(JSON.stringify(pois));
    this.selectedIndex = -1;
    this.editingIndex = -1;
    this.visiblePois = new Set(pois.map((_, i) => i));
    this.setDirty(false);
    this.hideForm();
    this.renderList();
  }

  /**
   * Render the POI card list.
   */
  renderList() {
    this.listEl.innerHTML = '';
    this.pois.forEach((poi, i) => {
      const card = document.createElement('div');
      card.className = 'poi-card' + (i === this.selectedIndex ? ' selected' : '');
      card.dataset.index = i;

      const cb = document.createElement('input');
      cb.type = 'checkbox';
      cb.className = 'poi-card-checkbox';
      cb.checked = this.visiblePois.has(i);
      cb.addEventListener('click', (e) => e.stopPropagation());
      cb.addEventListener('change', () => {
        if (cb.checked) {
          this.visiblePois.add(i);
        } else {
          this.visiblePois.delete(i);
        }
        if (this.onVisibilityChange) {
          this.onVisibilityChange(this.visiblePois);
        }
      });
      card.appendChild(cb);

      const info = document.createElement('div');
      info.className = 'poi-card-info';

      const name = document.createElement('div');
      name.className = 'poi-card-name';
      name.textContent = poi.name || '(unnamed)';
      info.appendChild(name);

      const detail = document.createElement('div');
      detail.className = 'poi-card-detail';
      const pose = poi.pose || {};
      const tags = (poi.tags || []).join(', ');
      const desc = poi.description || '';
      detail.textContent = `(${(pose.x||0).toFixed(2)}, ${(pose.y||0).toFixed(2)}) yaw=${(pose.yaw||0).toFixed(2)} ${tags} ${desc}`;
      info.appendChild(detail);

      card.appendChild(info);

      const actions = document.createElement('div');
      actions.className = 'poi-card-actions';

      const editBtn = document.createElement('button');
      editBtn.className = 'btn-edit';
      editBtn.textContent = 'Edit';
      editBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        this.editPoi(i);
      });
      actions.appendChild(editBtn);

      const delBtn = document.createElement('button');
      delBtn.className = 'btn-delete';
      delBtn.textContent = 'Del';
      delBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        this.deletePoi(i);
      });
      actions.appendChild(delBtn);

      card.appendChild(actions);

      card.addEventListener('click', () => this.selectPoi(i));
      this.listEl.appendChild(card);
    });
  }

  /**
   * Select a POI by index (highlight in list and on map).
   */
  selectPoi(index) {
    this.selectedIndex = index;
    this.renderList();
    if (this.onSelectionChange) {
      this.onSelectionChange(index);
    }
  }

  /**
   * Open edit form for a POI.
   */
  editPoi(index) {
    this.editingIndex = index;
    const poi = this.pois[index];
    this.formTitle.textContent = 'Edit POI';
    this.fillForm(poi);
    this.showForm();
    this.selectPoi(index);
  }

  /**
   * Delete a POI.
   */
  deletePoi(index) {
    this.pois.splice(index, 1);
    if (this.selectedIndex === index) this.selectedIndex = -1;
    if (this.selectedIndex > index) this.selectedIndex--;
    // Rebuild visiblePois with shifted indices
    const newVisible = new Set();
    this.visiblePois.forEach((vi) => {
      if (vi < index) newVisible.add(vi);
      else if (vi > index) newVisible.add(vi - 1);
    });
    this.visiblePois = newVisible;
    this.setDirty(true);
    this.hideForm();
    this.renderList();
    if (this.onSelectionChange) {
      this.onSelectionChange(this.selectedIndex);
    }
  }

  /**
   * Start adding a new POI — enter placing mode.
   */
  startAddPoi() {
    if (this.placingMode) {
      this.cancelPlacing();
      return;
    }
    this.placingMode = true;
    this.btnAddPoi.textContent = 'Click map...';
    this.btnAddPoi.classList.add('placing');
    if (this.onPlacingModeChange) {
      this.onPlacingModeChange(true);
    }
  }

  cancelPlacing() {
    this.placingMode = false;
    this.btnAddPoi.textContent = '+ Add POI';
    this.btnAddPoi.classList.remove('placing');
    if (this.onPlacingModeChange) {
      this.onPlacingModeChange(false);
    }
  }

  /**
   * Called when map is clicked in placing mode.
   */
  placeNewPoi(x, y) {
    if (!this.placingMode) return;
    this.cancelPlacing();

    const newPoi = {
      name: `poi_${this.pois.length}`,
      pose: { x: Math.round(x * 100) / 100, y: Math.round(y * 100) / 100, yaw: 0.0 },
      radius: 0.5,
      tags: ['goal'],
      description: '',
    };

    this.editingIndex = -2; // new
    this.formTitle.textContent = 'New POI';
    this.fillForm(newPoi);
    this.showForm();
  }

  /**
   * Set available tag definitions for tag helper UI.
   */
  setTagDefinitions(tags) {
    this.tagDefinitions = tags || [];
  }

  /**
   * Render tag chips below the tags input.
   */
  renderTagChips() {
    let container = document.getElementById('tag-chips');
    if (!container) {
      container = document.createElement('div');
      container.id = 'tag-chips';
      container.className = 'tag-chips';
      this.inputTags.parentElement.appendChild(container);
    }
    container.innerHTML = '';

    if (this.tagDefinitions.length === 0) return;

    const currentTags = this.inputTags.value.split(',').map(t => t.trim()).filter(t => t);

    this.tagDefinitions.forEach((td) => {
      const chip = document.createElement('span');
      const isActive = currentTags.includes(td.name);
      chip.className = 'tag-chip' +
        (isActive ? ' active' : '') +
        (td.is_system ? ' system' : ' user');
      chip.textContent = td.name;
      chip.title = td.description + (td.is_system ? ' (system)' : '');
      chip.addEventListener('click', () => {
        this.toggleTag(td.name);
      });
      container.appendChild(chip);
    });
  }

  /**
   * Toggle a tag in the tags input field.
   */
  toggleTag(tagName) {
    let tags = this.inputTags.value.split(',').map(t => t.trim()).filter(t => t);
    const idx = tags.indexOf(tagName);
    if (idx >= 0) {
      tags.splice(idx, 1);
    } else {
      tags.push(tagName);
    }
    this.inputTags.value = tags.join(', ');
    this.renderTagChips();
  }

  /**
   * Fill the edit form with POI data.
   */
  fillForm(poi) {
    this.inputName.value = poi.name || '';
    const pose = poi.pose || {};
    this.inputX.value = pose.x || 0;
    this.inputY.value = pose.y || 0;
    this.inputYaw.value = pose.yaw || 0;
    this.inputRadius.value = poi.radius || 0.5;
    this.inputTags.value = (poi.tags || []).join(', ');
    this.inputDescription.value = poi.description || '';
    this.renderTagChips();
  }

  /**
   * Read POI data from the form.
   */
  readForm() {
    return {
      name: this.inputName.value.trim(),
      pose: {
        x: parseFloat(this.inputX.value) || 0,
        y: parseFloat(this.inputY.value) || 0,
        yaw: parseFloat(this.inputYaw.value) || 0,
      },
      radius: parseFloat(this.inputRadius.value) || 0.5,
      tags: this.inputTags.value.split(',').map(t => t.trim()).filter(t => t),
      description: this.inputDescription.value.trim(),
    };
  }

  /**
   * Confirm form edits.
   */
  formOk() {
    const poi = this.readForm();
    if (this.editingIndex === -2) {
      // New POI
      this.pois.push(poi);
      this.selectedIndex = this.pois.length - 1;
      this.visiblePois.add(this.selectedIndex);
    } else if (this.editingIndex >= 0) {
      // Merge: keep unknown fields from original
      this.pois[this.editingIndex] = { ...this.pois[this.editingIndex], ...poi };
    }
    this.editingIndex = -1;
    this.setDirty(true);
    this.hideForm();
    this.renderList();
    if (this.onSelectionChange) {
      this.onSelectionChange(this.selectedIndex);
    }
  }

  /**
   * Cancel form edits.
   */
  formCancel() {
    this.editingIndex = -1;
    this.hideForm();
  }

  showForm() {
    this.formEl.classList.remove('hidden');
  }

  hideForm() {
    this.formEl.classList.add('hidden');
  }

  /**
   * Update form X/Y from map click (when editing).
   */
  updateFormPosition(x, y) {
    if (this.editingIndex === -1) return;
    this.inputX.value = Math.round(x * 100) / 100;
    this.inputY.value = Math.round(y * 100) / 100;
  }

  setDirty(isDirty) {
    this.dirty = isDirty;
    this.btnSave.disabled = !isDirty;
    this.btnDiscard.disabled = !isDirty;
    this.dirtyIndicator.textContent = isDirty ? 'Unsaved changes' : '';
    if (this.onDirtyChange) {
      this.onDirtyChange(isDirty);
    }
  }

  /**
   * Save POIs to server.
   */
  async save() {
    if (!this.dirty) return;
    this.btnSave.textContent = 'Saving...';
    try {
      const result = await MapoiApi.savePois(this.pois);
      if (result.success) {
        this.originalPois = JSON.parse(JSON.stringify(this.pois));
        this.setDirty(false);
        this.btnSave.textContent = 'Saved!';
        setTimeout(() => { this.btnSave.textContent = 'Save'; }, 1500);
      } else {
        alert('Save failed: ' + (result.error || 'unknown error'));
        this.btnSave.textContent = 'Save';
      }
    } catch (e) {
      alert('Save error: ' + e.message);
      this.btnSave.textContent = 'Save';
    }
  }

  /**
   * Discard all changes, reload from original.
   */
  discard() {
    if (!this.dirty) return;
    this.pois = JSON.parse(JSON.stringify(this.originalPois));
    this.selectedIndex = -1;
    this.editingIndex = -1;
    this.setDirty(false);
    this.hideForm();
    this.renderList();
    if (this.onSelectionChange) {
      this.onSelectionChange(-1);
    }
  }

  /**
   * Show all POIs on the map.
   */
  setAllVisible() {
    this.visiblePois = new Set(this.pois.map((_, i) => i));
    this.renderList();
    if (this.onVisibilityChange) this.onVisibilityChange(this.visiblePois);
  }

  /**
   * Hide all POIs from the map.
   */
  setAllHidden() {
    this.visiblePois = new Set();
    this.renderList();
    if (this.onVisibilityChange) this.onVisibilityChange(this.visiblePois);
  }
}
