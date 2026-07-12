#pragma once

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <filesystem>
#include <set>

#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mapoi_interfaces/msg/point_of_interest.hpp>
#include <mapoi_interfaces/srv/select_map.hpp>
#include <mapoi_interfaces/srv/get_pois_info.hpp>
#include <mapoi_interfaces/srv/get_maps_info.hpp>
#include <mapoi_interfaces/srv/get_tag_definitions.hpp>

#include <yaml-cpp/yaml.h>
#endif

#include <math.h>

#include <QMessageBox>
#include <QLabel>
#include <QRadioButton>
#include <QCheckBox>

namespace Ui {
class PoiEditorUi;
}


namespace mapoi_rviz_plugins
{

class PoiEditorPanel: public rviz_common::Panel
{
  Q_OBJECT
public:
  PoiEditorPanel(QWidget* parent = nullptr);
    ~PoiEditorPanel() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();

private Q_SLOTS:
  void MapComboBox();
  void ResetButton();
  void TableChanged(int row, int column);
  void RowMoved(int logicalIndex, int oldVisualIndex, int newVisualIndex);
  void NewButton();
  void CopyButton();
  void DeleteButton();
  void FileComboBox();
  void SaveButton();
  void TagFilterChanged(int index);
  void TagHelperSelected(int index);

protected:
  Ui::PoiEditorUi* ui_;
  std::string current_map_;
  std::string config_path_;
  std::vector<std::string> map_name_list_;
  bool is_table_color_;

  // Tag filter: store all POIs to restore when filter is cleared
  std::vector<mapoi_interfaces::msg::PointOfInterest> all_pois_;

  // Tag definitions from server
  std::vector<std::string> known_tag_names_;
  std::vector<bool> known_tag_is_system_;

  // ROS
  rclcpp::Node::SharedPtr node_;

  // Shared service node and persistent clients
  rclcpp::Node::SharedPtr service_node_;
  rclcpp::Client<mapoi_interfaces::srv::SelectMap>::SharedPtr select_map_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetPoisInfo>::SharedPtr get_pois_info_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetMapsInfo>::SharedPtr get_maps_info_client_;
  rclcpp::Client<mapoi_interfaces::srv::GetTagDefinitions>::SharedPtr get_tag_defs_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reload_map_info_client_;

  // Parameter client for /mapoi_rviz2_publisher (Display Settings UI が制御する)
  rclcpp::AsyncParametersClient::SharedPtr rviz2_pub_param_client_;

  // Display Settings widgets (programmatically created in constructor)
  QRadioButton * route_radio_all_ = nullptr;
  QRadioButton * route_radio_selected_ = nullptr;
  QRadioButton * route_radio_none_ = nullptr;
  QRadioButton * label_radio_index_ = nullptr;
  QRadioButton * label_radio_name_ = nullptr;
  QRadioButton * label_radio_both_ = nullptr;
  QRadioButton * label_radio_none_ = nullptr;
  QCheckBox * tolerance_sector_check_ = nullptr;

  // 最後に publisher と一致が確認できた値の cache。SetParameters 失敗時に UI を revert する元になる。
  // 初期値は publisher の declare_parameter default と一致 (sync 成功すればその値で更新される)。
  std::string cached_route_mode_ = "selected";
  std::string cached_label_fmt_ = "index";
  bool cached_tolerance_sector_ = true;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poi_pose_sub_;
  void PoiPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_path_sub_;
  void ConfigPathCallback(std_msgs::msg::String::SharedPtr msg);

  // Save 直後の 1.5 秒間は config_path callback 経由の UpdatePoiTable を抑制し、
  // SAVED! の green feedback を維持する (#135 副作用対応)。1.5 秒後の QTimer 末尾で
  // rebuild + flag クリアされる。
  bool suppress_config_callback_update_ = false;

  // 未保存編集ガード (#399)。以下 3 メンバは全て UI (Qt メイン) スレッド上でのみ触るため
  // ロック不要 (TableChanged/New/Copy/Delete/RowMoved/SaveButton/ConfigPathCallback の
  // queued lambda はいずれも UI スレッドで実行される)。
  //
  // ユーザーがセルを編集 / 行を追加削除移動したことを示す dirty フラグ。UpdatePoiTable
  // (全再構築 = サーバ状態と一致) 末尾と SaveButton 成功直後に clear する。外部で
  // config_path が再 publish された時、これが true なら再構築前に確認ダイアログを出す。
  bool table_dirty_ = false;

  // 外部変更検出 (SaveButton) のための baseline スナップショット。UpdatePoiTable 末尾で
  // テーブルの元データに対応する config ファイル (FileComboBox->itemText(0)) を全読みして
  // {path, content} を保存する。SaveButton 成功書き込み直後は書き込んだ内容と保存先 path で
  // 更新する。読めない場合は両方 clear = 比較不能 (ガード無効 = 従来挙動)。
  std::string baseline_path_;
  std::string baseline_content_;

  // ConfigPathCallback の確認ダイアログ表示中フラグ (#399)。QMessageBox のネストイベント
  // ループ中に後続の queued lambda が走って dialog が積み重なるのを防ぐ (dialog 中に届いた
  // config_path イベントは drop する。再読込を選べば最新状態が fetch されるので欠損しない)。
  bool config_dialog_open_ = false;

  // 内容 diff ガード (#403)。直前に処理した config_path イベントの path と mtime を保持し、
  // path も mtime も変わっていない再 publish (= 内容変化なし) を Noop に落とす。
  // UI (Qt メイン) スレッド上でのみ読み書きする (#399 の table_dirty_ 等と同じ規約)。
  std::string last_seen_config_path_;
  std::filesystem::file_time_type last_seen_config_mtime_{};

  // Functions
  void InitConfigs(std::string map_name);
  void UpdatePoiTable();
  void UpdatePoiCount();
  void PopulateTagFilter();
  void LoadTagDefinitions();
  bool ValidatePois();
  double calcYaw(geometry_msgs::msg::Pose pose);

  // Display Settings 副パネル (QGroupBox + Radio/Check) を構築し connect まで行う。
  // onInitialize から rviz2_pub_param_client_ 生成後・初期 Sync 前に一度だけ呼ぶ。
  void SetupDisplaySettingsUi();

  // Display Settings UI を mapoi_rviz2_publisher の現在 parameter 値に同期する。
  // onInitialize 末尾の初期同期と、SetParameters 失敗時の UI rollback の両方で使う。
  // 成功時は cached_* を更新する。
  void SyncDisplaySettingsFromPublisher();

  // POI 名フィルタ (#405): NameFilterEdit の現在テキストに基づいて全行の表示/非表示を設定する。
  // setRowHidden (行削除なし) を使うため rowCount() は不変。UpdatePoiTable / TagFilterChanged の
  // 再構築後に呼び出すことで、再構築後もフィルタ状態を維持する。textChanged からも直接呼ばれる。
  // 末尾で UpdatePoiCount を呼び、可視行数と件数表示を同期する (PR #420 review)。
  void ApplyNameFilter();
  // 単一行だけフィルタを再評価する (#405)。名前セルの編集確定時 (TableChanged) に使用。
  void ApplyNameFilterToRow(int row);

  // service down 等で sync が失敗した場合、cached_* に基づいて UI を revert する。
  // SyncDisplaySettingsFromPublisher() で sync を試みた後 (cache 未更新) のフォールバック用。
  void RevertDisplaySettingsUiFromCache();

  std::string join(const std::vector<std::string>& v, const char* delim);
  std::vector<std::string> SplitSentence(std::string sentence,
                                         std::string delimiter);
};
} // end namespace mapoi_rviz_plugins
