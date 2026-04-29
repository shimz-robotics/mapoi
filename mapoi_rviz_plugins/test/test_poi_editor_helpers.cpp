// Pure helper functions のための gtest (#151 round 1 low 対応)。
// QString を返す helper があるので Qt5::Core link が必要 (CMakeLists.txt 参照)。
#include <gtest/gtest.h>
#include <cmath>

#include "mapoi_rviz_plugins/poi_editor_helpers.hpp"

using mapoi_rviz_plugins::detail::format_yaw_deg;
using mapoi_rviz_plugins::detail::split_and_trim;
using mapoi_rviz_plugins::detail::try_parse_finite_double;

// --- format_yaw_deg ---

TEST(FormatYawDeg, ExactQuarterPi)
{
  // 0.7853981... rad は「整数度 45 を rad 化したもの」とほぼ一致 → 1 桁表示 "45.0"
  EXPECT_EQ(format_yaw_deg(M_PI / 4).toStdString(), "45.0");
}

TEST(FormatYawDeg, Zero)
{
  EXPECT_EQ(format_yaw_deg(0.0).toStdString(), "0.0");
}

TEST(FormatYawDeg, Pi)
{
  EXPECT_EQ(format_yaw_deg(M_PI).toStdString(), "180.0");
}

TEST(FormatYawDeg, NegativeHalfPi)
{
  EXPECT_EQ(format_yaw_deg(-M_PI / 2).toStdString(), "-90.0");
}

TEST(FormatYawDeg, FineAnglePreserved)
{
  // 12.3456° の rad は「整数度を rad 化したもの」と一致しない → 4 桁表示で精度維持
  const double rad = 12.3456 * M_PI / 180.0;
  EXPECT_EQ(format_yaw_deg(rad).toStdString(), "12.3456");
}

TEST(FormatYawDeg, NearIntKeeps4DigitsToPreserveRoundTrip)
{
  // **round 3 ヘビー medium 回帰防止**: `45.04°` を rad 化した値は「整数度 45 を rad 化した
  // 値」と一致しないので 4 桁表示。これにより read-save round-trip で値が壊れない。
  // (旧実装では「整数 45 との deg 差 < 0.05° で 1 桁化」→ `45.0` 表示 → save で `45.0°`
  // として書き戻る regression があった)
  const double rad = 45.04 * M_PI / 180.0;
  EXPECT_EQ(format_yaw_deg(rad).toStdString(), "45.0400");
}

TEST(FormatYawDeg, IntegerDegreeShownAsOneDigit)
{
  // 12° ぴったり (= rad で integer*pi/180) → 1 桁表示
  const double rad = 12.0 * M_PI / 180.0;
  EXPECT_EQ(format_yaw_deg(rad).toStdString(), "12.0");
}

// --- split_and_trim ---

TEST(SplitAndTrim, BasicTwoParts)
{
  const auto result = split_and_trim("0.5, 45.0", ',');
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0], "0.5");
  EXPECT_EQ(result[1], "45.0");
}

TEST(SplitAndTrim, NoSpaceAfterComma)
{
  const auto result = split_and_trim("0.5,45.0", ',');
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0], "0.5");
  EXPECT_EQ(result[1], "45.0");
}

TEST(SplitAndTrim, SpaceBeforeComma)
{
  const auto result = split_and_trim("0.5 , 45.0", ',');
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0], "0.5");
  EXPECT_EQ(result[1], "45.0");
}

TEST(SplitAndTrim, MultipleParts)
{
  const auto result = split_and_trim("a, b , c", ',');
  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result[0], "a");
  EXPECT_EQ(result[1], "b");
  EXPECT_EQ(result[2], "c");
}

TEST(SplitAndTrim, EmptyMiddleElement)
{
  const auto result = split_and_trim("a,,b", ',');
  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result[0], "a");
  EXPECT_EQ(result[1], "");
  EXPECT_EQ(result[2], "b");
}

TEST(SplitAndTrim, OnlyWhitespaceElement)
{
  const auto result = split_and_trim("a,   ,b", ',');
  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result[0], "a");
  EXPECT_EQ(result[1], "");
  EXPECT_EQ(result[2], "b");
}

TEST(SplitAndTrim, TrailingDelimiter)
{
  // "0.5," → 末尾区切りの空要素は std::getline では生成されない (= 1 要素のみ)。
  // tolerance validation 側で「2 要素必須」として reject されるので致命的ではないが、
  // helper の契約を test で固定する。
  const auto result = split_and_trim("0.5,", ',');
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0], "0.5");
}

TEST(SplitAndTrim, LeadingDelimiter)
{
  const auto result = split_and_trim(",0.5", ',');
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0], "");
  EXPECT_EQ(result[1], "0.5");
}

// --- try_parse_finite_double ---

TEST(TryParseFiniteDouble, ValidNumber)
{
  double out = 0.0;
  EXPECT_TRUE(try_parse_finite_double("3.14", out));
  EXPECT_DOUBLE_EQ(out, 3.14);
}

TEST(TryParseFiniteDouble, RejectsTrailingGarbage)
{
  double out = 0.0;
  EXPECT_FALSE(try_parse_finite_double("1abc", out));
}

TEST(TryParseFiniteDouble, RejectsEmpty)
{
  double out = 0.0;
  EXPECT_FALSE(try_parse_finite_double("", out));
}

TEST(TryParseFiniteDouble, RejectsInf)
{
  double out = 0.0;
  EXPECT_FALSE(try_parse_finite_double("inf", out));
}

TEST(TryParseFiniteDouble, RejectsNan)
{
  double out = 0.0;
  EXPECT_FALSE(try_parse_finite_double("nan", out));
}

TEST(TryParseFiniteDouble, AllowsTrailingWhitespace)
{
  double out = 0.0;
  EXPECT_TRUE(try_parse_finite_double("3.14 ", out));
  EXPECT_DOUBLE_EQ(out, 3.14);
}
