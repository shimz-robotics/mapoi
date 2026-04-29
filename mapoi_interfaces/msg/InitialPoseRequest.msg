# SwitchMap や reload による initial pose 候補通知 (#149 round 8 high 対応)。
# mapoi_server が新 map ロード時に publish し、mapoi_nav_server / bridge 群が subscribe する。
# - map_name : 通知対象の map (= MapoiServer::map_name_)。subscriber 側で「自分が知っている map と
#              一致するか」を check して、世代不一致 (stale) なら無視 / 待機する。空文字は許容しない。
# - poi_name : 採用する POI 名 (空 = default = POI list 先頭)。
# QoS: transient_local (depth=1) で後起動 subscriber も latched 値を受信できる。
string map_name
string poi_name
