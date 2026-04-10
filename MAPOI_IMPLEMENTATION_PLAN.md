# mapoi_packages 実装計画書

作成日: 2026-04-03
作成元: Claude Code セッション (feature/poi-pause-tag ブランチ)

---

## 1. 概要

mapoi_packages に対して以下の2つの作業を行う:

1. **仕様変更**: 全POIで PoiEvent を発行 + pause タグ付きPOIでは追加で自動pause
2. **テスト基盤構築**: gtest 単体テスト + launch_testing 統合テスト

### 背景: 実現したいフロー

```
① custom_tagが設定されたeventPOIに進入
② 自律移動の一時停止 + eventトピックの発行
③ ハンド+アーム側でeventトピックを受け取り、POI名ごとに設定された動作を行う
④ 動作終了後にハンド+アーム側からmapoi_resumeトピックを発行
⑤ mapoi側がトピックを受け取り次第再開、次のPOIへ移動
```

---

## 2. 現状の仕様と問題点

### 現在の動作 (mapoi_nav_server.cpp:610-649)

| タグ構成 | POI進入時の動作 | PoiEvent発行 | 自動pause |
|---------|---------------|-------------|----------|
| custom_tagのみ (例: audio_info) | PoiEvent発行のみ | する | **しない** |
| pauseタグのみ | 自動pauseのみ | **しない** | する |
| pause + custom_tag | pauseのみ (pause優先) | **しない** | する |

**問題**: PoiEvent発行とpauseが排他的。フロー②の「一時停止 + eventトピック発行」を同時に行えない。

### 推奨する仕様

| タグ構成 | POI進入時の動作 | PoiEvent発行 | 自動pause |
|---------|---------------|-------------|----------|
| custom_tagのみ | PoiEvent発行のみ (**変更なし**) | する | しない |
| pauseタグのみ | 自動pauseのみ (**変更なし**) | しない | する |
| **全POI** | **PoiEvent発行** | **する** | - |
| **pauseタグ付きPOI** | **PoiEvent発行 → 自動pause** | **する** | **する** |

全POIでPoiEventを発行し、pauseはpauseタグの有無で制御する。

---

## 3. 仕様変更の実装詳細

### 変更ファイルと内容

#### 3-1. `mapoi_server/src/mapoi_nav_server.cpp`

**変更箇所A: `rebuild_event_pois()` (L545-567)**

タグによるフィルタリングを削除し、全POIを監視対象にする。

```cpp
// 変更前
void MapoiNavServer::rebuild_event_pois()
{
  if (!system_tags_loaded_) {
    return;
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  event_pois_.clear();
  for (const auto & poi : pois_list_) {
    bool has_user_tag = false;
    bool has_pause_tag = false;
    for (const auto & tag : poi.tags) {
      if (tag == "pause") {
        has_pause_tag = true;
      } else if (system_tags_.find(tag) == system_tags_.end()) {
        has_user_tag = true;
      }
    }
    if (has_user_tag || has_pause_tag) {
      event_pois_.push_back(poi);
    }
  }
  RCLCPP_INFO(this->get_logger(), "Monitoring %zu POIs with user/pause tags for radius events.", event_pois_.size());
}

// 変更後
void MapoiNavServer::rebuild_event_pois()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  event_pois_ = pois_list_;
  RCLCPP_INFO(this->get_logger(), "Monitoring %zu POIs for radius events.", event_pois_.size());
}
```

**変更箇所B: `radius_check_callback()` のENTER分岐 (L610-624)**

全POIでPoiEventを発行。pauseタグPOIでは追加でpause対象として名前を保存。

```cpp
// 変更前
if (!was_inside && dist <= poi.radius) {
  poi_inside_state_[poi.name] = true;
  if (is_pause_poi) {
    pause_triggered_poi = poi.name;
    RCLCPP_INFO(..., "POI ENTER (pause tag): %s ...", ...);
  } else {
    // PoiEvent publish
    poi_event_pub_->publish(event);
    RCLCPP_INFO(..., "POI ENTER: %s ...", ...);
  }
}

// 変更後
if (!was_inside && dist <= poi.radius) {
  poi_inside_state_[poi.name] = true;
  // 全POIでPoiEvent発行
  mapoi_interfaces::msg::PoiEvent event;
  event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_ENTER;
  event.poi = poi;
  event.stamp = this->now();
  poi_event_pub_->publish(event);
  RCLCPP_INFO(..., "POI ENTER: %s (dist=%.2f, radius=%.2f)", ...);

  // pauseタグがあれば自動pause対象
  if (is_pause_poi) {
    pause_triggered_poi = poi.name;
  }
}
```

**変更箇所C: `radius_check_callback()` のEXIT分岐 (L625-638)**

全POIでEXIT eventを発行。

```cpp
// 変更前
} else if (was_inside && dist > poi.radius * hysteresis) {
  poi_inside_state_[poi.name] = false;
  if (!is_pause_poi) {
    // PoiEvent publish
    poi_event_pub_->publish(event);
    RCLCPP_INFO(..., "POI EXIT: %s ...", ...);
  } else {
    RCLCPP_INFO(..., "POI EXIT (pause tag): %s ...", ...);
  }
}

// 変更後
} else if (was_inside && dist > poi.radius * hysteresis) {
  poi_inside_state_[poi.name] = false;
  // 全POIでEXIT event発行
  mapoi_interfaces::msg::PoiEvent event;
  event.event_type = mapoi_interfaces::msg::PoiEvent::EVENT_EXIT;
  event.poi = poi;
  event.stamp = this->now();
  poi_event_pub_->publish(event);
  RCLCPP_INFO(..., "POI EXIT: %s (dist=%.2f, radius=%.2f)", ...);
}
```

**変更箇所D: `radius_check_callback()` のpauseタグ判定 (L604-608)**

is_pause_poi の判定ロジック自体は変更不要。タグ文字列比較のままでOK。

---

## 4. テスト基盤の実装詳細

### 4-1. テスト用設定ファイル

**新規作成: `mapoi_server/test/test_mapoi_config.yaml`**

```yaml
poi:
  - name: poi_goal_only
    pose: {x: 1.0, y: 0.0, yaw: 0.0}
    radius: 0.5
    tags: [goal]
    description: "goalタグのみ"
  - name: poi_with_pause
    pose: {x: 0.0, y: 2.0, yaw: 0.0}
    radius: 0.5
    tags: [goal, pause]
    description: "pauseタグ付き"
  - name: poi_with_custom
    pose: {x: 3.0, y: 0.0, yaw: 0.0}
    radius: 0.5
    tags: [goal, audio_info]
    description: "custom_tag付き"
  - name: poi_pause_and_custom
    pose: {x: 0.0, y: 4.0, yaw: 0.0}
    radius: 0.5
    tags: [goal, pause, audio_info]
    description: "pause + custom_tag"

custom_tags:
  - {name: audio_info, description: "Audio guide trigger"}
```

### 4-2. package.xml への依存追加

既存の `<test_depend>` に以下を追加:

```xml
<test_depend>ament_cmake_gtest</test_depend>
<test_depend>launch_testing</test_depend>
<test_depend>launch_testing_ament_cmake</test_depend>
<test_depend>launch_testing_ros2</test_depend>
```

### 4-3. CMakeLists.txt への追加

`ament_auto_package()` の前に追加:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Level 1: gtest unit tests
  ament_auto_add_gtest(test_nav_server_unit
    test/test_nav_server_unit.cpp
  )
  target_compile_definitions(test_nav_server_unit PRIVATE UNIT_TEST)

  # Level 2: launch_testing integration tests
  add_launch_test(
    test/test_poi_event_integration.py
  )

  # Install test config
  install(
    FILES test/test_mapoi_config.yaml
    DESTINATION share/${PROJECT_NAME}/test
  )
endif()
```

### 4-4. ヘッダー修正 (`mapoi_nav_server.hpp`)

`class MapoiNavServer` のprivateセクション末尾に追加:

```cpp
#ifdef UNIT_TEST
  friend class NavServerTestFixture;
  FRIEND_TEST(NavServerTestFixture, DistanceCalculation);
  FRIEND_TEST(NavServerTestFixture, DistanceCalculationZero);
  FRIEND_TEST(NavServerTestFixture, RebuildEventPoisIncludesAllPois);
  FRIEND_TEST(NavServerTestFixture, RebuildEventPoisEmpty);
  FRIEND_TEST(NavServerTestFixture, PauseTagDetection);
#endif
```

`#include <gtest/gtest.h>` も `#ifdef UNIT_TEST` で囲んでヘッダー先頭に追加。

### 4-5. gtest単体テスト

**新規作成: `mapoi_server/test/test_nav_server_unit.cpp`**

```cpp
#define UNIT_TEST
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "mapoi_server/mapoi_nav_server.hpp"

class NavServerTestFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<MapoiNavServer>();
  }
  void TearDown() override
  {
    node_.reset();
  }

  // ヘルパー: テスト用POIを作成
  mapoi_interfaces::msg::PointOfInterest make_poi(
    const std::string & name, double x, double y,
    double radius, const std::vector<std::string> & tags)
  {
    mapoi_interfaces::msg::PointOfInterest poi;
    poi.name = name;
    poi.pose.position.x = x;
    poi.pose.position.y = y;
    poi.radius = radius;
    poi.tags = tags;
    return poi;
  }

  std::shared_ptr<MapoiNavServer> node_;
};

// distance_2d が正しい2D距離を返す
TEST_F(NavServerTestFixture, DistanceCalculation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 3.0;
  pose.position.y = 4.0;
  double dist = node_->distance_2d(pose, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(dist, 5.0);
}

// 同一座標で距離0
TEST_F(NavServerTestFixture, DistanceCalculationZero)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 2.0;
  double dist = node_->distance_2d(pose, 1.0, 2.0);
  EXPECT_DOUBLE_EQ(dist, 0.0);
}

// rebuild_event_pois() が全POIを含む (仕様変更後)
TEST_F(NavServerTestFixture, RebuildEventPoisIncludesAllPois)
{
  {
    std::lock_guard<std::mutex> lock(node_->data_mutex_);
    node_->pois_list_.push_back(make_poi("goal_only", 1.0, 0.0, 0.5, {"goal"}));
    node_->pois_list_.push_back(make_poi("with_pause", 0.0, 2.0, 0.5, {"goal", "pause"}));
    node_->pois_list_.push_back(make_poi("with_custom", 3.0, 0.0, 0.5, {"goal", "audio_info"}));
    node_->pois_list_.push_back(make_poi("origin_only", 0.0, 0.0, 0.5, {"origin"}));
  }
  node_->rebuild_event_pois();
  EXPECT_EQ(node_->event_pois_.size(), 4u);  // 全POI
}

// 空のpois_list_の場合
TEST_F(NavServerTestFixture, RebuildEventPoisEmpty)
{
  node_->rebuild_event_pois();
  EXPECT_EQ(node_->event_pois_.size(), 0u);
}

// pauseタグの判定ロジック確認
TEST_F(NavServerTestFixture, PauseTagDetection)
{
  auto poi_pause = make_poi("pause_poi", 0.0, 0.0, 0.5, {"goal", "pause"});
  auto poi_no_pause = make_poi("normal_poi", 0.0, 0.0, 0.5, {"goal", "audio_info"});

  auto has_pause = [](const mapoi_interfaces::msg::PointOfInterest & p) {
    for (const auto & tag : p.tags) {
      if (tag == "pause") return true;
    }
    return false;
  };

  EXPECT_TRUE(has_pause(poi_pause));
  EXPECT_FALSE(has_pause(poi_no_pause));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

### 4-6. launch_testing統合テスト

**新規作成: `mapoi_server/test/test_poi_event_integration.py`**

```python
import os
import unittest
import time

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from mapoi_interfaces.msg import PoiEvent
from ament_index_python.packages import get_package_share_directory


@launch_testing.markers.keep_alive
def generate_test_description():
    pkg_share = get_package_share_directory('mapoi_server')
    test_config_dir = os.path.join(pkg_share, 'test')

    mapoi_server_node = launch_ros.actions.Node(
        package='mapoi_server',
        executable='mapoi_server',
        name='mapoi_server',
        parameters=[{
            'maps_path': test_config_dir,
            'map_name': '.',
            'config_file': 'test_mapoi_config.yaml',
        }],
    )

    mapoi_nav_server_node = launch_ros.actions.Node(
        package='mapoi_server',
        executable='mapoi_nav_server',
        name='mapoi_nav_server',
        parameters=[{
            'radius_check_hz': 10.0,
            'map_frame': 'map',
            'base_frame': 'base_link',
        }],
    )

    return launch.LaunchDescription([
        mapoi_server_node,
        mapoi_nav_server_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'mapoi_server': mapoi_server_node, 'mapoi_nav_server': mapoi_nav_server_node}


class TestPoiEventIntegration(unittest.TestCase):
    """mapoi_poi_events トピックの発行を検証する統合テスト"""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_poi_event_node')
        cls.received_events = []
        cls.sub = cls.node.create_subscription(
            PoiEvent, 'mapoi_poi_events',
            lambda msg: cls.received_events.append(msg), 10)
        cls.tf_broadcaster = StaticTransformBroadcaster(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.received_events.clear()

    def _publish_tf(self, x, y):
        """map -> base_link のTFを発行"""
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _spin_and_wait(self, timeout_sec=3.0):
        """指定秒数spinしてイベントを待つ"""
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_enter_event_goal_only_poi(self):
        """goalタグのみのPOI (1.0, 0.0) 付近 → ENTER イベント発行"""
        self._publish_tf(1.0, 0.0)
        self._spin_and_wait(3.0)
        enter_events = [e for e in self.received_events
                        if e.event_type == PoiEvent.EVENT_ENTER
                        and e.poi.name == 'poi_goal_only']
        self.assertGreater(len(enter_events), 0,
                           "goal-only POI の ENTER イベントが発行されるべき")

    def test_enter_event_pause_poi(self):
        """pauseタグ付きPOI (0.0, 2.0) 付近 → ENTER イベント発行"""
        self._publish_tf(0.0, 2.0)
        self._spin_and_wait(3.0)
        enter_events = [e for e in self.received_events
                        if e.event_type == PoiEvent.EVENT_ENTER
                        and e.poi.name == 'poi_with_pause']
        self.assertGreater(len(enter_events), 0,
                           "pause POI の ENTER イベントが発行されるべき")

    def test_enter_event_custom_poi(self):
        """custom_tag付きPOI (3.0, 0.0) 付近 → ENTER イベント発行"""
        self._publish_tf(3.0, 0.0)
        self._spin_and_wait(3.0)
        enter_events = [e for e in self.received_events
                        if e.event_type == PoiEvent.EVENT_ENTER
                        and e.poi.name == 'poi_with_custom']
        self.assertGreater(len(enter_events), 0,
                           "custom_tag POI の ENTER イベントが発行されるべき")

    def test_exit_event(self):
        """POI内 → radius外 に移動 → EXIT イベント発行"""
        # まず POI 内に入る
        self._publish_tf(1.0, 0.0)
        self._spin_and_wait(2.0)
        self.received_events.clear()
        # POI 外に出る (radius=0.5, hysteresis=1.15 → 0.575m以上離れる)
        self._publish_tf(5.0, 5.0)
        self._spin_and_wait(2.0)
        exit_events = [e for e in self.received_events
                       if e.event_type == PoiEvent.EVENT_EXIT]
        self.assertGreater(len(exit_events), 0,
                           "EXIT イベントが発行されるべき")
```

---

## 5. 検証コマンド

```bash
# UIDの問題を先に解決する (プロジェクトディレクトリの所有権)
sudo chown -R $(id -u):$(id -g) /home/kimura/work/ros2_ws/src/mapoi_packages/

# ビルド
cd /home/kimura/work/ros2_ws
colcon build --packages-select mapoi_server

# 全テスト実行
colcon test --packages-select mapoi_server --event-handlers console_direct+

# gtest のみ
colcon test --packages-select mapoi_server --ctest-args -R test_nav_server_unit

# launch_testing のみ
colcon test --packages-select mapoi_server --ctest-args -R test_poi_event_integration

# 結果確認
colcon test-result --verbose
```

---

## 6. 実装手順チェックリスト

- [ ] `sudo chown -R` でプロジェクトディレクトリの所有権を修正
- [ ] `mapoi_server/test/` ディレクトリ作成
- [ ] `mapoi_server/test/test_mapoi_config.yaml` 作成
- [ ] `mapoi_server/package.xml` にテスト依存追加
- [ ] `mapoi_server/CMakeLists.txt` にテストターゲット追加
- [ ] `mapoi_server/include/mapoi_server/mapoi_nav_server.hpp` にfriend宣言追加
- [ ] `mapoi_server/test/test_nav_server_unit.cpp` 作成
- [ ] `mapoi_server/test/test_poi_event_integration.py` 作成
- [ ] ビルド確認: `colcon build --packages-select mapoi_server`
- [ ] gtest実行確認 (現仕様では RebuildEventPoisIncludesAllPois が失敗するはず)
- [ ] 仕様変更を実装 (上記セクション3の変更箇所A/B/Cを適用)
- [ ] 再ビルド・全テスト実行で全件パスを確認

---

## 7. 関連ファイル一覧

| ファイル | 役割 |
|---------|------|
| `mapoi_server/src/mapoi_nav_server.cpp` | 仕様変更のメイン対象 |
| `mapoi_server/include/mapoi_server/mapoi_nav_server.hpp` | friend宣言追加 |
| `mapoi_server/src/mapoi_server.cpp` | POI/タグ管理 (変更なし) |
| `mapoi_interfaces/msg/PoiEvent.msg` | EVENT_ENTER=1, EVENT_EXIT=2 (変更なし) |
| `mapoi_interfaces/msg/PointOfInterest.msg` | POIデータ構造 (変更なし) |
| `mapoi_server/maps/tag_definitions.yaml` | システムタグ定義: goal, origin, pause |

## 8. トピック一覧

| トピック名 | メッセージ型 | 用途 |
|-----------|------------|------|
| `mapoi_poi_events` | `mapoi_interfaces/PoiEvent` | POI進入/退出イベント |
| `mapoi_nav_status` | `std_msgs/String` | ナビ状態 (navigating/paused/succeeded等) |
| `mapoi_pause` | `std_msgs/String` | 一時停止指示 |
| `mapoi_resume` | `std_msgs/String` | 再開指示 |
| `mapoi_config_path` | `std_msgs/String` | 設定パス (transient_local) |
