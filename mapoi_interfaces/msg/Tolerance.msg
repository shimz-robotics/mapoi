# Spatial tolerance (Nav2 SimpleGoalChecker と同 semantics)
# xy / yaw 共に 0 / 負値は禁止 (実用上「無反応 POI」を作れてしまうため)。
# 最小値 0.001 (= xy: 1 mm / yaw: 約 0.057°)。「制約を実質緩めたい」場合は
# 大きい値を明示的に指定する (例: yaw=3.14 ≒ 360° tolerance で yaw 不問)。
# yaw の挙動上の用途: RViz の扇形描画に加え、waypoint_arrival_mode=mapoi での単発 Go
# (GOAL) 到達判定にも使う (#261)。robot 姿勢が tolerance.yaw 以内なら radius 進入で即完了、
# 超えていれば Nav2 SUCCEEDED (最終姿勢合わせ) を待つ。route waypoint 進行は xy のみで判定。
float64 xy        # Euclidean tolerance (m)、>= 0.001
float64 yaw       # Angular tolerance (rad)、>= 0.001
