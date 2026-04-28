/**
 * Sidebar drag-resize (#122 PR-2)。
 *
 * #sidebar-resizer (6px のスプリッタ) を mousedown でつかんで左右に動かすと
 * #poi-panel の幅が変わる。リリース時に localStorage に永続化、次回ロード時
 * に復元する。
 *
 * 制約:
 * - 幅は MIN_WIDTH / MAX_WIDTH で clamp (panel 中身が崩れない範囲)
 * - mobile (max-width: 768px) では handle が CSS で非表示なので何もしない
 * - drag 中は document 全体で text 選択を抑止し ew-resize cursor を強制
 *
 * リサイズ後に Leaflet マップが新サイズを認識できるよう `window` の
 * `resize` event を dispatch する (Leaflet が listen して invalidateSize 相当
 * を内部で行う)。
 */
(function () {
  const STORAGE_KEY = 'mapoi_sidebar_width_px';
  const MIN_WIDTH = 280;
  const MAX_WIDTH = 800;

  document.addEventListener('DOMContentLoaded', () => {
    const resizer = document.getElementById('sidebar-resizer');
    const panel = document.getElementById('poi-panel');
    if (!resizer || !panel) return;

    // 初期化: 保存幅を復元 (範囲外 / NaN は無視して default 350px のまま)
    const savedRaw = localStorage.getItem(STORAGE_KEY);
    if (savedRaw !== null) {
      const saved = parseInt(savedRaw, 10);
      if (!Number.isNaN(saved) && saved >= MIN_WIDTH && saved <= MAX_WIDTH) {
        panel.style.width = saved + 'px';
      }
    }

    let dragging = false;
    let startX = 0;
    let startWidth = 0;

    resizer.addEventListener('mousedown', (e) => {
      e.preventDefault();
      dragging = true;
      startX = e.clientX;
      startWidth = panel.getBoundingClientRect().width;
      resizer.classList.add('dragging');
      document.body.style.cursor = 'ew-resize';
      document.body.style.userSelect = 'none';
    });

    document.addEventListener('mousemove', (e) => {
      if (!dragging) return;
      // resizer の右側に panel があるので、左へドラッグ (clientX 減少) すると
      // panel 幅が増える。
      const delta = startX - e.clientX;
      let newWidth = startWidth + delta;
      newWidth = Math.max(MIN_WIDTH, Math.min(MAX_WIDTH, newWidth));
      panel.style.width = newWidth + 'px';
    });

    document.addEventListener('mouseup', () => {
      if (!dragging) return;
      dragging = false;
      resizer.classList.remove('dragging');
      document.body.style.cursor = '';
      document.body.style.userSelect = '';
      const finalWidth = Math.round(panel.getBoundingClientRect().width);
      try {
        localStorage.setItem(STORAGE_KEY, String(finalWidth));
      } catch (_) {
        // localStorage が使えない環境 (private mode 等) では永続化を skip
      }
      // Leaflet マップに新サイズを通知 (`resize` event を listen している)。
      window.dispatchEvent(new Event('resize'));
    });
  });
})();
