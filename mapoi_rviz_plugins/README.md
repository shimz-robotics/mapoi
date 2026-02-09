# mapoi_rviz_plugins

mapoi 用の RViz2 プラグインパッケージです。GUI から地図切替・POI 選択・自律走行の操作、および POI の編集ができます。

## プラグイン

### MapoiPanel（パネルプラグイン）

RViz2 のパネルとして追加できるナビゲーション操作パネルです。

- 地図の切替をドロップダウンから選択
- 地図に応じた目的地（`goal` タグの POI）をリストから選択
- ルートをリストから選択して走行開始
- 選択した POI の位置に自己位置を修正（Initial Pose の設定）
- 選択した POI への自律走行を開始
- 自律走行の停止
- ナビゲーション状態の表示（`navigating`, `succeeded`, `aborted`, `canceled`）
- 選択中の POI/ルートを RViz2 上でハイライト表示

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 初期位置の配信 |
| `goal_pose` | `geometry_msgs/PoseStamped` | ゴール位置の配信 |
| `mapoi_cancel` | `std_msgs/String` | ナビゲーションのキャンセル |
| `mapoi_route` | `std_msgs/String` | ルート走行の開始 |
| `mapoi_highlight_goal` | `std_msgs/String` | ゴールマーカーのハイライト |
| `mapoi_highlight_route` | `std_msgs/String` | ルートマーカーのハイライト |

#### サブスクライバー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_config_path` | `std_msgs/String` | 設定ファイルパスの変更検知 |
| `mapoi_nav_status` | `std_msgs/String` | ナビゲーション状態の表示 |

### PoiEditor（パネルプラグイン）

POI の情報を表形式で表示・編集・保存できるパネルです。

- 現在の設定ファイルから POI 情報を読み込み
- 表形式での POI 情報の確認・編集（name, description, pose, radius, tags）
- POI の追加・コピー・削除
- タグによるフィルタリング表示
- TagHelperComboBox によるタグ入力補助（システムタグは `[S]` 表記）
- バリデーション付き保存（名前重複・座標形式・半径チェック、未定義タグの警告）
- MapoiPoseTool と連携した位置入力
- 保存後に mapoi_server の設定を自動リロード

#### サブスクライバー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_rviz_pose` | `geometry_msgs/PoseStamped` | MapoiPoseTool からの位置入力 |
| `mapoi_config_path` | `std_msgs/String` | 設定ファイルパスの変更検知 |

### MapoiPoseTool（ツールプラグイン）

RViz2 のツールバーに追加できるポーズ指定ツールです。ショートカットキー: `i`

- RViz2 上でクリック＆ドラッグして位置・姿勢を指定
- PoiEditor の選択行に位置を反映
- 使用後は自動的にデフォルトツールに戻る

#### パブリッシャー

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `mapoi_rviz_pose` | `geometry_msgs/PoseStamped` | 指定した位置・姿勢の配信 |

## RViz2 への追加方法

1. RViz2 を起動
2. Panels > Add New Panel から `MapoiPanel` または `PoiEditor` を追加
3. ツールバーの「+」ボタンから `MapoiPoseTool` を追加

## 依存パッケージ

- `rviz_common`
- `rviz_default_plugins`
- `std_srvs`
- `mapoi_interfaces`
