// Pure helper functions for PoiEditorPanel (#151 で header 化、unit test 容易化のため)。
// rclcpp / Qt 依存なし。stdlib のみ。
#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <QString>

namespace mapoi_rviz_plugins::detail
{

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

// `delimiter` で区切って trim ( leading/trailing whitespace を除去) した結果を返す。
// 空要素も保持する (`a,,b` → `["a", "", "b"]`)。`SplitSentence(", ")` の硬い分割と異なり、
// `0.5,45.0` `0.5, 45.0` `0.5 , 45.0` 等の表記揺れを許容する (#151 round 1 medium)。
inline std::vector<std::string> split_and_trim(const std::string & input, char delimiter)
{
  std::vector<std::string> result;
  std::stringstream ss(input);
  std::string item;
  while (std::getline(ss, item, delimiter)) {
    const auto begin = item.find_first_not_of(" \t");
    const auto end = item.find_last_not_of(" \t");
    if (begin == std::string::npos) {
      result.push_back("");
    } else {
      result.push_back(item.substr(begin, end - begin + 1));
    }
  }
  return result;
}

// rad → 表示用 deg 文字列。yaml 値の rad → deg 変換で「45° 狙い 0.7853981 rad」が
// 44.9999... と表示されるのを避けるための四捨五入 helper (#151)。
//
// 戦略: **整数度** との差が <= 0.05° なら「整数 deg を 1 桁表示」、
//       誤差が大きい (= 細かい角度を意図的に入れたケース) はそのまま 4 桁表示で精度維持
//       (round 1 ヘビー medium 対応で「0.1° 単位」→「整数 deg 単位」に修正)。
inline QString format_yaw_deg(double rad)
{
  const double deg = rad * 180.0 / M_PI;
  const double rounded_to_int = std::round(deg);
  if (std::abs(deg - rounded_to_int) <= 0.05) {
    return QString::number(rounded_to_int, 'f', 1);
  }
  return QString::number(deg, 'f', 4);
}

}  // namespace mapoi_rviz_plugins::detail
