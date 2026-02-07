# mapoi_web 実装メモ

## 実装状況 (2026-02-07)
- パッケージスケルトン: 完了
- バックエンド (Python): 完了
- フロントエンド (HTML/CSS/JS): 完了
- ラウンチファイル: 完了
- colcon build: 成功
- 単体テスト (YAML round-trip, PGM→PNG): 成功
- **動作テスト: 未実施**

## 次のステップ (動作確認)

### 1. 基本起動テスト
```bash
# まず mapoi_server を起動（シミュレータなしでもOK）
ros2 run mapoi_server mapoi_server --ros-args -p maps_path:=<maps_path> -p map_name:=turtlebot3_world

# 別ターミナルで mapoi_web を起動
ros2 run mapoi_web mapoi_web_node.py --ros-args -p maps_path:=<maps_path> -p map_name:=turtlebot3_world

# ブラウザで http://localhost:8080 にアクセス
```

### 2. 確認項目
- [ ] Flask起動ログが表示されること
- [ ] ブラウザで地図画像が表示されること
- [ ] POI一覧が表示されること（マーカー + カードリスト）
- [ ] POIの追加（マップクリック）→ フォーム入力 → OK
- [ ] POIの編集（Editボタン）→ フォーム修正 → OK
- [ ] POIの削除（Delボタン）
- [ ] 保存（Saveボタン）→ YAML書き込み → reload_map_info呼び出し
- [ ] 保存後にYAMLファイルの内容が正しいこと
- [ ] 変更破棄（Discardボタン）
- [ ] マップ切り替え（セレクトボックス）
- [ ] スマホブラウザからのアクセスとレスポンシブ表示

### 3. 既知の注意点
- `maps_path` パラメータは必須。ソースディレクトリのパスを直接指定推奨
- PyYAML はコメントを保持しない（mapoi_config.yaml のコメントは消える）
- RViz PoiEditor との同時編集は後勝ち（Phase 1 制限）
- Leaflet.js は CDN (unpkg.com) から読み込み。オフライン環境では別途対応が必要

### 4. 起こりうる問題
- Flask の `send_from_directory` パスが install 先と一致しない場合
  → `ament_index_python` で share ディレクトリを解決するロジックあり
- `web_port` パラメータの型が int だが launch YAML から文字列で渡される場合
  → launch YAML で `"8080"` としているが ROS2 パラメータ型変換に注意

## Phase 2 予定
- ナビゲーション操作（目的地送信、ルート実行）
- ルート編集
- POIマーカーのドラッグ&ドロップ
- 自動リフレッシュ（ポーリング/WebSocket）
