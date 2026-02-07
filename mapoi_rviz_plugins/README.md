# mapoi_rviz_plugins

mapoi 用の RViz2 プラグインパッケージです。GUI から地図切替・POI 選択・自律走行の操作ができます。

## プラグイン

### MapoiPanel（パネルプラグイン）

RViz2 のパネルとして追加できる操作パネルです。

- 地図の切替をドロップダウンから選択
- 地図に応じた目的地（POI）をリストから選択
- 選択した POI の位置に自己位置を修正（Initial Pose の設定）
- 選択した POI への自律走行を開始
- 自律走行の停止

### PoiEditor（パネルプラグイン）

POI の情報を表形式で表示・編集できるパネルです。

- 現在の設定ファイルから POI 情報を読み込み
- 表形式での POI 情報の確認・編集

### MapoiPoseTool（ツールプラグイン）

RViz2 のツールバーに追加できるポーズ指定ツールです。

- RViz2 上でクリック＆ドラッグして位置・姿勢を指定
- mapoi 向けのポーズをパブリッシュ
- 使用後は自動的にデフォルトツールに戻る

## RViz2 への追加方法

1. RViz2 を起動
2. Panels > Add New Panel から `MapoiPanel` または `PoiEditor` を追加
3. ツールバーの「+」ボタンから `MapoiPoseTool` を追加

## 依存パッケージ

- `rviz_common`
- `rviz_default_plugins`
- `std_srvs`
- `mapoi_interfaces`

## 開発方法

1. Qt Creator で `.ui` ファイルを作成・編集（`src/ui/` 配下）
2. 対応するヘッダファイル（`include/mapoi_rviz_plugins/`）とソースファイル（`src/`）を作成
3. `CMakeLists.txt` にファイルを追加してビルド

参考: [RViz2プラグインの作成方法](https://qiita.com/RyodoTanaka/items/4ca117672ad171472578)
