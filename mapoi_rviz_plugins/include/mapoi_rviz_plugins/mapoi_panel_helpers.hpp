#pragma once

#include <string>

namespace mapoi_rviz_plugins::detail
{

// バックエンド接続状態バッジの表示文字列を組み立てる純関数 (#400、PR #419 review で
// UpdateNavButtonsEnabled から切り出し)。表示規則 (Nav / Loc 共通):
//   - received == false → 空文字 (contract 未実装 publisher / 旧構成では UI 不変の後方互換)
//   - received && !alive → "<prefix>: 切断 (bridge 停止)" — liveliness lost。保持中の reason は
//     死亡直前の stale 値のため表示しない (issue #400 の要件)
//   - alive && ready → "<prefix>: 接続"
//   - alive && !ready → "<prefix>: 未準備 (<reason>)" (reason 空なら括弧ごと省略)
// Qt / ROS 非依存 (呼び出し側が QString へ変換して QLabel に載せる)。gtest で分岐表を pin する。
inline std::string build_backend_badge_text(
  const std::string & prefix, bool received, bool alive, bool ready,
  const std::string & reason)
{
  if (!received) {
    return "";
  }
  if (!alive) {
    return prefix + ": 切断 (bridge 停止)";
  }
  if (ready) {
    return prefix + ": 接続";
  }
  if (!reason.empty()) {
    return prefix + ": 未準備 (" + reason + ")";
  }
  return prefix + ": 未準備";
}

}  // namespace mapoi_rviz_plugins::detail
