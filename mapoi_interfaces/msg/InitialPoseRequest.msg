# SwitchMap や reload による initial pose 候補通知 (#149 round 8 high 対応)。
# mapoi_server が新 map ロード時に publish し、mapoi_nav_server / bridge 群が subscribe する。
# - map_name : 通知対象の map (= MapoiServer::map_name_)。subscriber 側で「自分が知っている map と
#              一致するか」を check して、世代不一致 (stale) なら無視 / 待機する。空文字は publisher 側
#              では許容しない (= mapoi_server / mapoi_webui は必ず現在 map を入れる)。
# - poi_name : 採用する POI 名。空文字は「採用候補なし」を意味し、subscriber は無視する
#              (default の POI list 先頭採用は publisher 側 = mapoi_server::compute_initial_poi_name で
#              既に解決済。外部 publisher が空文字を送っても何も起こらない)。
# QoS: transient_local (depth=1) で後起動 subscriber も latched 値を受信できる。
string map_name
string poi_name
