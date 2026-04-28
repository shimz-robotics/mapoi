/**
 * Sidebar drag-resize (#122 PR-2)。
 *
 * #sidebar-resizer (6px のスプリッタ) を mousedown でつかんで左右に動かすと
 * #poi-panel の幅が変わる。リリース時に localStorage に永続化、次回ロード時
 * に復元する。
 *
 * 制約:
 * - 幅は MIN_WIDTH / 動的 maxAllowedWidth() で clamp
 *   maxAllowedWidth は MAX_WIDTH と「viewport から map 用の最低幅 (MAP_RESERVE_PX)
 *   を確保した値」のうち小さい方。viewport 縮小で 800px 上限が viewport を
 *   超えないようにする (Codex PR #124 round 1 medium)。
 * - mobile (max-width: 768px) では handle が CSS で非表示なので何もしない
 * - drag 中は document 全体で text 選択を抑止し ew-resize cursor を強制
 * - window blur / mouseleave で drag を確実に cancel し stuck を防ぐ
 *
 * リサイズ後に Leaflet マップが新サイズを認識できるよう `window` の
 * `resize` event を dispatch する (Leaflet が listen して invalidateSize 相当
 * を内部で行う)。復元直後にも 1 回 dispatch して初期化タイミングずれを防ぐ。
 *
 * localStorage 読み書きは private mode 等で SecurityError が出るため try/catch
 * で囲み、失敗時は default 350px のまま drag は引き続き使えるようにする。
 */
(function () {
  const STORAGE_KEY = 'mapoi_sidebar_width_px';
  const MIN_WIDTH = 280;
  const MAX_WIDTH = 800;
  const MAP_RESERVE_PX = 200; // viewport 縮小時に map 側に確保する最低幅

  function maxAllowedWidth() {
    return Math.max(MIN_WIDTH, Math.min(MAX_WIDTH, window.innerWidth - MAP_RESERVE_PX));
  }

  function clamp(width) {
    return Math.max(MIN_WIDTH, Math.min(maxAllowedWidth(), width));
  }

  document.addEventListener('DOMContentLoaded', () => {
    const resizer = document.getElementById('sidebar-resizer');
    const panel = document.getElementById('poi-panel');
    if (!resizer || !panel) return;

    // 初期化: 保存幅を復元 (範囲外 / NaN / storage 例外は無視して default のまま)
    let savedRaw = null;
    try {
      savedRaw = localStorage.getItem(STORAGE_KEY);
    } catch (_) {
      // private mode / cookies blocked など。default 350px で続行
    }
    if (savedRaw !== null) {
      const saved = parseInt(savedRaw, 10);
      if (!Number.isNaN(saved)) {
        panel.style.width = clamp(saved) + 'px';
        // 復元直後に Leaflet にサイズ通知 (map 初期化タイミング次第のずれを防ぐ)
        window.dispatchEvent(new Event('resize'));
      }
    }

    let dragging = false;
    let startX = 0;
    let startWidth = 0;

    function cancelDrag() {
      if (!dragging) return;
      dragging = false;
      resizer.classList.remove('dragging');
      document.body.style.cursor = '';
      document.body.style.userSelect = '';
    }

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
      panel.style.width = clamp(startWidth + delta) + 'px';
    });

    document.addEventListener('mouseup', () => {
      if (!dragging) return;
      cancelDrag();
      const finalWidth = Math.round(panel.getBoundingClientRect().width);
      try {
        localStorage.setItem(STORAGE_KEY, String(finalWidth));
      } catch (_) {
        // localStorage が使えない環境では永続化を skip
      }
      // Leaflet マップに新サイズを通知 (`resize` event を listen している)。
      window.dispatchEvent(new Event('resize'));
    });

    // ブラウザ外 mouseup や window が blur した時に drag が stuck するのを防ぐ。
    window.addEventListener('blur', cancelDrag);

    // viewport 縮小で現在幅が maxAllowedWidth() を超えた場合は再 clamp。
    // drag 中は startWidth 基準で計算しているので無視 (drag 終了後に補正される)。
    window.addEventListener('resize', () => {
      if (dragging) return;
      const current = Math.round(panel.getBoundingClientRect().width);
      const clamped = clamp(current);
      if (clamped !== current) {
        panel.style.width = clamped + 'px';
      }
    });
  });
})();
