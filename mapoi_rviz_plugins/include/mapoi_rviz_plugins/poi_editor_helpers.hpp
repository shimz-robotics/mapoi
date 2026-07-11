// Pure helper functions for PoiEditorPanel (#158 で header 化、unit test 容易化のため)。
// Qt 依存なし、stdlib のみ。
#pragma once

#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include "mapoi_rviz_plugins/config_path_update_policy.hpp"

namespace mapoi_rviz_plugins::detail
{

// tolerance.xy / tolerance.yaw の最小値 (msg spec、poi-filter.js の TOLERANCE_MIN と同じ値、#138)。
constexpr double kToleranceMin = 0.001;

// PoiTable column index 定数 (#158 round 1 medium): magic number を排除して
// 「name → pose → tolerance → tags → description」順序を 1 箇所に集約。
// #346 の TU 分割で poi_editor.cpp / poi_editor_validation.cpp の両方から参照される
// ため header へ移動 (PR #371 review medium: TU ごとの重複定義は列並び替え時に
// 片方だけ更新されて validation が黙って壊れるリスクがあった)。
constexpr int kColName = 0;
constexpr int kColPose = 1;
constexpr int kColTolerance = 2;
constexpr int kColTags = 3;
constexpr int kColDescription = 4;
constexpr int kColCount = 5;

// 文字列を strict に double に parse し、有限性を確認する純関数 (Codex review #139 medium 対応)。
// std::stod は "1abc" を 1 として返し NaN/Inf も throw しないため、validation と save で
// 同じ parser を使うために helper にする。min check は呼び出し側の責任。
//
// success: parsed value を out に書き、true を返す。
// failure: out 不変、false を返す (parse failure / 末尾 garbage / NaN / +-Inf いずれも reject)。
inline bool try_parse_finite_double(const std::string & str, double & out)
{
  if (str.empty()) return false;
  try {
    size_t pos = 0;
    const double v = std::stod(str, &pos);
    while (pos < str.size() && std::isspace(static_cast<unsigned char>(str[pos]))) ++pos;
    if (pos != str.size()) return false;
    if (!std::isfinite(v)) return false;
    out = v;
    return true;
  } catch (...) {
    return false;
  }
}

// `delimiter` で区切って trim ( leading/trailing whitespace + 改行を除去) した結果を返す。
// **中間の空要素は保持** (`"a,,b"` → `["a", "", "b"]`) するが、**末尾の空要素は std::getline
// 仕様で省略される** (`"0.5,"` → `["0.5"]`)。tolerance validation は「2 要素必須」check で
// 末尾区切りを拒否できる。先頭空要素は保持される (`",0.5"` → `["", "0.5"]`)。
// `0.5,45.0` `0.5, 45.0` `0.5 , 45.0` 等の表記揺れと改行混入 (`copy & paste 由来`) を許容する。
inline std::vector<std::string> split_and_trim(const std::string & input, char delimiter)
{
  std::vector<std::string> result;
  std::stringstream ss(input);
  std::string item;
  while (std::getline(ss, item, delimiter)) {
    const auto begin = item.find_first_not_of(" \t\r\n");
    const auto end = item.find_last_not_of(" \t\r\n");
    if (begin == std::string::npos) {
      result.push_back("");
    } else {
      result.push_back(item.substr(begin, end - begin + 1));
    }
  }
  return result;
}

// PoiEditorPanel::ValidatePois の pose セル ("x, y, yaw") 判定 (#346)。判定ロジックのみを
// 純関数化し、QMessageBox 向けの文言組み立ては呼び出し側 (poi_editor_validation.cpp) が
// 元の文字列そのまま (raw / invalid_value) を使って行う — Qt の `.arg(double)` 書式化に
// 依存する数値以外は、ここでは文字列のまま持ち回って表示文言を一切変えない。
enum class PoseCellStatus
{
  kOk,
  kWrongFieldCount,  // "x, y, yaw" の 3 要素に split できなかった
  kInvalidValue,     // いずれかの要素が有限 double として parse できなかった
};

struct PoseCellValidation
{
  PoseCellStatus status = PoseCellStatus::kWrongFieldCount;
  std::string raw;            // 元の pose_str (kWrongFieldCount のメッセージに使う)
  std::string invalid_value;  // parse 失敗した要素 (kInvalidValue のメッセージに使う)
};

inline PoseCellValidation validate_pose_cell(const std::string & pose_str)
{
  PoseCellValidation result;
  result.raw = pose_str;
  const auto parts = split_and_trim(pose_str, ',');
  if (parts.size() != 3) {
    result.status = PoseCellStatus::kWrongFieldCount;
    return result;
  }
  for (const auto & part : parts) {
    double v = 0.0;
    if (!try_parse_finite_double(part, v)) {
      result.status = PoseCellStatus::kInvalidValue;
      result.invalid_value = part;
      return result;
    }
  }
  result.status = PoseCellStatus::kOk;
  return result;
}

// PoiEditorPanel::ValidatePois の tolerance セル ("xy_m, yaw_rad") 判定 (#346)。
// xy / yaw は独立に評価する (両方エラーがあれば呼び出し側は両方の警告を積める)。
enum class ToleranceFieldStatus
{
  kOk,
  kParseError,     // 有限 double として parse できない
  kBelowMinimum,   // 0 以上だが kToleranceMin 未満
  kYawExceeds2Pi,  // yaw のみ: 2π 超過 (#159、deg 入力の取り違え防御)
};

struct ToleranceCellValidation
{
  bool format_ok = false;  // "xy, yaw" の 2 要素に split できたか
  std::string xy_raw;      // parts[0] (kParseError のメッセージに使う)
  std::string yaw_raw;     // parts[1] (kParseError のメッセージに使う)
  ToleranceFieldStatus xy_status = ToleranceFieldStatus::kOk;
  double xy_value = 0.0;   // xy_status==kOk|kBelowMinimum 時のみ意味を持つ
  ToleranceFieldStatus yaw_status = ToleranceFieldStatus::kOk;
  double yaw_value = 0.0;  // yaw_status==kOk|kBelowMinimum|kYawExceeds2Pi 時のみ意味を持つ
};

inline ToleranceCellValidation validate_tolerance_cell(const std::string & tolerance_str)
{
  ToleranceCellValidation result;
  const auto parts = split_and_trim(tolerance_str, ',');
  if (parts.size() != 2) {
    result.format_ok = false;
    return result;
  }
  result.format_ok = true;
  result.xy_raw = parts[0];
  result.yaw_raw = parts[1];

  double xy_val = 0.0;
  if (!try_parse_finite_double(parts[0], xy_val)) {
    result.xy_status = ToleranceFieldStatus::kParseError;
  } else {
    result.xy_value = xy_val;
    if (xy_val < kToleranceMin) {
      result.xy_status = ToleranceFieldStatus::kBelowMinimum;
    }
  }

  double yaw_val = 0.0;
  if (!try_parse_finite_double(parts[1], yaw_val)) {
    result.yaw_status = ToleranceFieldStatus::kParseError;
  } else {
    result.yaw_value = yaw_val;
    if (yaw_val < kToleranceMin) {
      result.yaw_status = ToleranceFieldStatus::kBelowMinimum;
    } else if (yaw_val > 2.0 * M_PI) {
      result.yaw_status = ToleranceFieldStatus::kYawExceeds2Pi;
    }
  }
  return result;
}

// PoiEditorPanel::ValidatePois のタグ排他判定 (#85 / #143、#346 で純関数化)。
// waypoint+landmark (landmark は Nav2 navigation 不可) と pause+landmark (landmark は
// 到達不可なので pause が成立しない) の 2 組を独立に判定する。
struct TagExclusivityResult
{
  bool waypoint_landmark_conflict = false;
  bool pause_landmark_conflict = false;
};

inline TagExclusivityResult check_tag_exclusivity(const std::vector<std::string> & tags)
{
  bool has_waypoint = false;
  bool has_landmark = false;
  bool has_pause = false;
  for (const auto & t : tags) {
    if (t == "waypoint") has_waypoint = true;
    else if (t == "landmark") has_landmark = true;
    else if (t == "pause") has_pause = true;
  }
  TagExclusivityResult result;
  result.waypoint_landmark_conflict = has_waypoint && has_landmark;
  result.pause_landmark_conflict = has_pause && has_landmark;
  return result;
}

// PoiEditorPanel::ConfigPathCallback の未保存編集ガード判定 (#399)。
// 外部で mapoi/config_path が再 publish された時に、テーブルを無条件で全再構築すると
// 未保存のセル編集が黙って消える。suppression 適用後の action・dirty フラグ・dialog 表示中
// フラグの 3 つから、UI が「そのまま再構築 / 確認ダイアログ / イベント破棄」のいずれを
// 取るべきかを判定する純関数。Qt 非依存 (callback 側が結果を QMessageBox 表示へ写像する)。
enum class ConfigReloadGuardDecision
{
  kProceed,   // 従来通り再構築 (Noop なら何もしない、それ以外は InitConfigs/UpdatePoiTable)
  kAskUser,   // 未保存編集ありのため確認ダイアログを出す
  kDrop,      // dialog 表示中に届いたイベントは破棄 (再入・dialog 積み重ね防止)
};

// action    : suppression 適用後の ConfigPathUpdateAction (Noop = suppress 中 or 更新不要)
// table_dirty: UI スレッド上のユーザー編集有無 (未保存編集ガードの主軸)
// dialog_open: 既に確認ダイアログを表示中か (QMessageBox のネストイベントループ中に
//              後続 queued lambda が走って dialog が積み重なるのを防ぐ)
inline ConfigReloadGuardDecision decide_config_reload_guard(
  ConfigPathUpdateAction action,
  bool table_dirty,
  bool dialog_open)
{
  // 再構築対象が無い (Noop = suppress 中 or 同一 map で更新不要) なら dirty でも何もしない。
  // Noop は table を触らないので未保存編集は消えない。
  if (action == ConfigPathUpdateAction::Noop) {
    return ConfigReloadGuardDecision::kProceed;
  }
  // dialog 表示中に届いたイベントは破棄する。ユーザーが「再読込」を選べばその時点の
  // 最新状態が fetch されるため drop で欠損しない (再入防止)。
  if (dialog_open) {
    return ConfigReloadGuardDecision::kDrop;
  }
  // 未保存編集がある状態での外部変更は、破棄可否をユーザーに確認する。
  if (table_dirty) {
    return ConfigReloadGuardDecision::kAskUser;
  }
  // dirty でなければ従来通り再構築 (サーバ状態に追従)。
  return ConfigReloadGuardDecision::kProceed;
}

// PoiEditorPanel::SaveButton の外部変更検出判定 (#399)。保存先ファイルが baseline (最後に
// テーブルへ読み込んだ / 保存した時点の内容) と食い違う場合に上書き確認を出すべきかを返す。
// WebUI の expected_version 楽観ロック相当のクライアント側実装で、config_version の厳密共有は
// 不要で内容比較で目的を達せられる。
//
// PR #413 review (low: itemText(0)/currentText 非対称) を踏まえ、比較は **保存先 path**
// (currentText 由来) が baseline_path と一致する時のみ行う。FileComboBox「the other」で
// 別ファイルを保存先に指定した場合は基準を持たないため比較しない (誤った基準ファイルとの
// 比較を避ける)。baseline_content が空 (読めなかった等) の場合もガード無効化 = 従来挙動。
inline bool should_confirm_overwrite(
  const std::string & save_path,
  const std::string & baseline_path,
  const std::string & baseline_content,
  const std::string & current_content)
{
  if (save_path != baseline_path) {
    return false;  // 保存先が baseline と別 path: 比較不能 (基準なし)
  }
  if (baseline_content.empty()) {
    return false;  // baseline を読めなかった: ガード無効 (従来挙動)
  }
  return baseline_content != current_content;  // path 一致 && baseline 非空 && 内容不一致
}

}  // namespace mapoi_rviz_plugins::detail
