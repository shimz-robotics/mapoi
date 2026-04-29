// Pure helper functions for PoiEditorPanel (#151 で header 化、unit test 容易化のため)。
// 依存: stdlib + QString (format_yaw_deg のみ Qt5::Core が必要、それ以外は stdlib のみ)。
#pragma once

#include <cctype>
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
// **中間の空要素は保持** (`"a,,b"` → `["a", "", "b"]`) するが、**末尾の空要素は std::getline
// 仕様で省略される** (`"0.5,"` → `["0.5"]`)。tolerance validation は「2 要素必須」check で
// 末尾区切りを拒否できるので致命的ではない。先頭空要素は保持される (`",0.5"` → `["", "0.5"]`)。
// `SplitSentence(", ")` の硬い分割と異なり `0.5,45.0` `0.5, 45.0` `0.5 , 45.0` 等の表記揺れを
// 許容する (#151 round 1 medium、round 3 軽微 medium で挙動明示)。
inline std::vector<std::string> split_and_trim(const std::string & input, char delimiter)
{
  std::vector<std::string> result;
  std::stringstream ss(input);
  std::string item;
  while (std::getline(ss, item, delimiter)) {
    // copy & paste で改行が混入する可能性に対応 (#151 round 4 medium)。
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

// rad → 表示用 deg 文字列。yaml 値の rad → deg 変換で「45° 狙い 0.7853981 rad」が
// 44.9999... と表示されるのを避けるための四捨五入 helper (#151)。
//
// 戦略: **「整数度を rad 化した値」と元 rad がほぼ一致** (= 1e-9 未満) するなら「整数度を
//       1 桁表示」(例: `0.7853981... rad` → `45.0`)。それ以外は 4 桁表示で元値の精度維持。
//       (round 3 ヘビー medium 対応で「整数度との deg 差判定」→「rad 差判定」に修正:
//       前者は `45.04°` のような端数も `45.0` に丸めて round-trip で値喪失する regression
//       を生んでいたため。本ロジックは「元の user 入力が整数度だったかどうか」を逆算する。)
inline QString format_yaw_deg(double rad)
{
  const double deg = rad * 180.0 / M_PI;
  const double rounded_to_int = std::round(deg);
  const double rad_of_int = rounded_to_int * M_PI / 180.0;
  if (std::abs(rad - rad_of_int) < 1e-9) {
    return QString::number(rounded_to_int, 'f', 1);
  }
  return QString::number(deg, 'f', 4);
}

}  // namespace mapoi_rviz_plugins::detail
