// Pure helper functions for PoiEditorPanel (#158 で header 化、unit test 容易化のため)。
// Qt 依存なし、stdlib のみ。
#pragma once

#include <cctype>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

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

}  // namespace mapoi_rviz_plugins::detail
