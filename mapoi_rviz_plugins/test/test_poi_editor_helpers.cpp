// Pure helper functions のための gtest (#158 round 1)。Qt 不要 (header は stdlib のみ)。
#include <gtest/gtest.h>

#include "mapoi_rviz_plugins/poi_editor_helpers.hpp"

using mapoi_rviz_plugins::detail::split_and_trim;
using mapoi_rviz_plugins::detail::try_parse_finite_double;

// --- split_and_trim ---

TEST(SplitAndTrim, BasicTwoParts)
{
  const auto result = split_and_trim("0.5, 0.7854", ',');
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0], "0.5");
  EXPECT_EQ(result[1], "0.7854");
}

TEST(SplitAndTrim, NoSpaceAfterComma)
{
  const auto result = split_and_trim("0.5,0.7854", ',');
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0], "0.5");
  EXPECT_EQ(result[1], "0.7854");
}

TEST(SplitAndTrim, SpaceBeforeComma)
{
  const auto result = split_and_trim("0.5 , 0.7854", ',');
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0], "0.5");
  EXPECT_EQ(result[1], "0.7854");
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
  // "0.5," → 末尾区切りの空要素は std::getline 仕様で省略される (= 1 要素のみ)。
  // tolerance validation は「2 要素必須」check で reject する。
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

TEST(SplitAndTrim, TrimsNewlines)
{
  // copy & paste で改行が混入してもパースが落ちないこと
  const auto result = split_and_trim("0.5,\n0.7854", ',');
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0], "0.5");
  EXPECT_EQ(result[1], "0.7854");
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
