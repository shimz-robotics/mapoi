// Pure helper functions のための gtest (#158 round 1)。Qt 不要。
// header 依存: stdlib + geometry_msgs / tf2 (calc_yaw のみ、#397 step 8 で追加)。
// テスト target は ament_auto_add_gtest で ament_cmake_auto が package.xml の
// geometry_msgs / tf2 depend から link を解決する。
#include <gtest/gtest.h>

#include <cmath>

#include "mapoi_rviz_plugins/poi_editor_helpers.hpp"

using mapoi_rviz_plugins::detail::split_and_trim;
using mapoi_rviz_plugins::detail::try_parse_finite_double;

// 未保存編集ガード判定 (#399)
using mapoi_rviz_plugins::detail::ConfigPathUpdateAction;
using mapoi_rviz_plugins::detail::ConfigReloadGuardDecision;
using mapoi_rviz_plugins::detail::decide_config_reload_guard;
using mapoi_rviz_plugins::detail::should_confirm_overwrite;

// 自由関数化した helpers (#397 step 8)
using mapoi_rviz_plugins::detail::calc_yaw;
using mapoi_rviz_plugins::detail::join;
using mapoi_rviz_plugins::detail::split_sentence;

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

// --- decide_config_reload_guard (#399) ---
//
// ConfigPathCallback は外部 config_path 再 publish 時に、この判定を QMessageBox 表示 /
// 再構築 / drop へ写像するだけ。dirty×action×dialog_open の分岐表をここで pin し、
// 未保存編集を黙って捨てる回帰 (dirty 判定の抜け・drop 条件の反転等) を安く捕える。

TEST(DecideConfigReloadGuard, NotDirtyProceedsForRefresh)
{
  // 未保存編集なし: 従来通り再構築 (サーバ状態に追従)。
  EXPECT_EQ(
    decide_config_reload_guard(
      ConfigPathUpdateAction::RefreshCurrentMap, false, false),
    ConfigReloadGuardDecision::kProceed);
}

TEST(DecideConfigReloadGuard, NotDirtyProceedsForReinitialize)
{
  EXPECT_EQ(
    decide_config_reload_guard(
      ConfigPathUpdateAction::ReinitializeMap, false, false),
    ConfigReloadGuardDecision::kProceed);
}

TEST(DecideConfigReloadGuard, DirtyRefreshAsksUser)
{
  // 未保存編集あり + 再構築対象あり: 破棄可否を確認する。
  EXPECT_EQ(
    decide_config_reload_guard(
      ConfigPathUpdateAction::RefreshCurrentMap, true, false),
    ConfigReloadGuardDecision::kAskUser);
}

TEST(DecideConfigReloadGuard, DirtyReinitializeAsksUser)
{
  // map 切替でも未保存編集があれば確認する (Reinitialize も table を捨てるため)。
  EXPECT_EQ(
    decide_config_reload_guard(
      ConfigPathUpdateAction::ReinitializeMap, true, false),
    ConfigReloadGuardDecision::kAskUser);
}

TEST(DecideConfigReloadGuard, NoopProceedsEvenWhenDirty)
{
  // Noop (suppress 中 or 更新不要) は table を触らないため dirty でも確認不要。
  EXPECT_EQ(
    decide_config_reload_guard(
      ConfigPathUpdateAction::Noop, true, false),
    ConfigReloadGuardDecision::kProceed);
}

TEST(DecideConfigReloadGuard, DialogOpenDropsEvent)
{
  // dialog 表示中に届いたイベントは破棄する (dialog 積み重ね防止)。dirty でも drop。
  EXPECT_EQ(
    decide_config_reload_guard(
      ConfigPathUpdateAction::RefreshCurrentMap, true, true),
    ConfigReloadGuardDecision::kDrop);
}

TEST(DecideConfigReloadGuard, DialogOpenDropsEvenWhenNotDirty)
{
  // dirty でなくても dialog 表示中の再構築 (Refresh/Reinit) は drop する
  // (ネストイベントループ中の再入を一切許さない)。
  EXPECT_EQ(
    decide_config_reload_guard(
      ConfigPathUpdateAction::ReinitializeMap, false, true),
    ConfigReloadGuardDecision::kDrop);
}

TEST(DecideConfigReloadGuard, NoopTakesPrecedenceOverDialogOpen)
{
  // Noop は最優先で Proceed (table を触らないので dialog 中でも drop 不要 = 無害)。
  EXPECT_EQ(
    decide_config_reload_guard(
      ConfigPathUpdateAction::Noop, true, true),
    ConfigReloadGuardDecision::kProceed);
}

// --- should_confirm_overwrite (#399) ---

TEST(ShouldConfirmOverwrite, ContentDiffersConfirms)
{
  // path 一致 && baseline 非空 && 内容不一致: 上書き確認が必要。
  EXPECT_TRUE(should_confirm_overwrite(
    "/maps/mapA/mapoi_config.yaml", "/maps/mapA/mapoi_config.yaml",
    "poi: [old]", "poi: [changed_externally]"));
}

TEST(ShouldConfirmOverwrite, ContentMatchesNoConfirm)
{
  // 内容一致: 外部変更なし = 確認不要。
  EXPECT_FALSE(should_confirm_overwrite(
    "/maps/mapA/mapoi_config.yaml", "/maps/mapA/mapoi_config.yaml",
    "poi: [same]", "poi: [same]"));
}

TEST(ShouldConfirmOverwrite, DifferentPathDoesNotCompare)
{
  // 保存先が baseline と別 path (FileComboBox「the other」で別ファイル指定): 比較しない。
  // 内容が違っても基準を持たないファイルとの比較は無意味なので false。
  EXPECT_FALSE(should_confirm_overwrite(
    "/maps/mapB/other.yaml", "/maps/mapA/mapoi_config.yaml",
    "poi: [baseline_of_A]", "poi: [content_of_B]"));
}

TEST(ShouldConfirmOverwrite, EmptyBaselineDisablesGuard)
{
  // baseline を読めなかった (空): ガード無効 = 従来挙動 (無条件上書き)。
  EXPECT_FALSE(should_confirm_overwrite(
    "/maps/mapA/mapoi_config.yaml", "/maps/mapA/mapoi_config.yaml",
    "", "poi: [anything_on_disk]"));
}

TEST(ShouldConfirmOverwrite, EmptyBaselinePathDoesNotMatchNonEmptySave)
{
  // baseline 未取得 (path も空) の状態で保存先を指定 → path 不一致で比較しない。
  EXPECT_FALSE(should_confirm_overwrite(
    "/maps/mapA/mapoi_config.yaml", "", "", "poi: [x]"));
}

// --- calc_yaw (#397 step 8) ---
//
// PoiEditorPanel::calcYaw を detail::calc_yaw として自由関数化した純関数のテスト。
// quaternion → yaw 変換の契約を pin する。

// ヘルパ: roll=0, pitch=0, yaw=theta の quaternion を作る (tf2 直接使用)。
namespace
{
geometry_msgs::msg::Pose make_pose_yaw(double yaw_rad)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_rad);
  geometry_msgs::msg::Pose pose;
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}
}  // namespace

TEST(CalcYaw, IdentityQuaternionIsZero)
{
  // 恒等 quaternion (w=1, xyz=0) → yaw = 0
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  EXPECT_NEAR(calc_yaw(pose), 0.0, 1e-9);
}

TEST(CalcYaw, KnownYawFortyFiveDeg)
{
  // yaw = π/4 (45°) の quaternion → π/4 が返る
  const double expected = M_PI / 4.0;
  EXPECT_NEAR(calc_yaw(make_pose_yaw(expected)), expected, 1e-9);
}

TEST(CalcYaw, KnownYawNegativeNinetyDeg)
{
  // yaw = -π/2 (-90°) の quaternion → -π/2 が返る
  const double expected = -M_PI / 2.0;
  EXPECT_NEAR(calc_yaw(make_pose_yaw(expected)), expected, 1e-9);
}

TEST(CalcYaw, NearPiBoundary)
{
  // yaw ≈ π (180°) 付近。getRPY の返値範囲は (-π, π] で、±π のどちらで返るかは実装依存。
  // 「常に 0 を返す」ような退行も検知できるよう |result| ≈ π まで断言する (PR #425 review)。
  const double yaw = M_PI;
  const double result = calc_yaw(make_pose_yaw(yaw));
  EXPECT_NEAR(std::abs(result), M_PI, 1e-6);
}

// --- join (#397 step 8) ---

TEST(Join, EmptyVector)
{
  // 空ベクタ → 空文字列
  EXPECT_EQ(join({}, ", "), "");
}

TEST(Join, SingleElement)
{
  EXPECT_EQ(join({"foo"}, ", "), "foo");
}

TEST(Join, MultipleElements)
{
  EXPECT_EQ(join({"waypoint", "pause", "landmark"}, ", "), "waypoint, pause, landmark");
}

TEST(Join, NullDelim)
{
  // delim が nullptr の場合は単純連結 (区切りなし)
  EXPECT_EQ(join({"a", "b", "c"}, nullptr), "abc");
}

// --- split_sentence (#397 step 8) ---

TEST(SplitSentence, BasicTwoParts)
{
  // 区切り ", " (カンマ+スペース) での基本分割。スペースは区切りの一部として消費される。
  const auto result = split_sentence("waypoint, pause", ", ");
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0], "waypoint");
  EXPECT_EQ(result[1], "pause");
}

TEST(SplitSentence, CommaOnlyDelimiterDoesNotTrim)
{
  // 区切り "," (スペース無し) だと後続トークンの先頭スペースは残る = trim しない契約
  // (trim が要る場合は split_and_trim を使う)。PR #425 review のコメント誤読対策として分離。
  const auto result = split_sentence("waypoint, pause", ",");
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0], "waypoint");
  EXPECT_EQ(result[1], " pause");
}

TEST(SplitSentence, NoDelimiter)
{
  // 区切り無し → 1 要素のベクタ
  const auto result = split_sentence("single", ", ");
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0], "single");
}

TEST(SplitSentence, TrailingDelimiter)
{
  // "a, b, " → ["a", "b", ""] (末尾区切り後に空文字列が入る)
  const auto result = split_sentence("a, b, ", ", ");
  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result[0], "a");
  EXPECT_EQ(result[1], "b");
  EXPECT_EQ(result[2], "");
}

TEST(SplitSentence, EmptyString)
{
  // 空文字列 → 1 要素 (空文字列) のベクタ (while loop が回らず末尾トークンだけ push)
  const auto result = split_sentence("", ", ");
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0], "");
}

TEST(SplitSentence, EmptyDelimiterReturnsWholeSentence)
{
  // 空 delimiter は無限ループガードにより分割せず全体を 1 要素で返す (PR #425 review)。
  const auto result = split_sentence("abc", "");
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0], "abc");
}

TEST(SplitSentence, RoundTripWithJoin)
{
  // UI 表示は join(", ")・保存/検証は split_sentence(", ") の対で使われるため、
  // 往復の同一性を pin する (片方だけ変更された時のタグ列回帰を検知、PR #425 review)。
  const std::vector<std::string> tags = {"waypoint", "pause", "landmark"};
  EXPECT_EQ(split_sentence(join(tags, ", "), ", "), tags);
  const std::string joined = "a, b, c";
  EXPECT_EQ(join(split_sentence(joined, ", "), ", "), joined);
}
