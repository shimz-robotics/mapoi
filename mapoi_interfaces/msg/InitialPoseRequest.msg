# operator map switch や reload による initial pose 候補通知 (#149 round 8 で字面付き型化)。
# mapoi_nav_server が Nav2 LoadMap 成功後に publish し、bridge 群も subscribe する。
# - map_name : 通知対象の map (= MapoiServer::map_name_)。情報用フィールド (世代識別の素材)。
#              現状の subscriber 実装では検証に使っていない (#155 で pending 戦略を別途検討中)。
#              空文字は publisher 側では許容しない (= mapoi_server / mapoi_webui は必ず現在 map を入れる)。
# - poi_name : 採用する POI 名。空文字は「採用候補なし」を意味し、subscriber は無視する
#              (default の POI list 先頭採用は select_map response で
#              既に解決済。外部 publisher が空文字を送っても何も起こらない)。
# QoS: transient_local (depth=1) で後起動 subscriber も latched 値を受信できる。
# stale message 排除は publisher 上書き (depth=1) に依存。世代不一致時の pending / 再評価は
# scope が大きく、本 PR では実装しない (#155)。
string map_name
string poi_name
