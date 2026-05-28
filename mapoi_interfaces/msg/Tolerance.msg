# Spatial tolerance (Nav2 SimpleGoalChecker と同 semantics)
# xy / yaw 共に 0 / 負値は禁止 (実用上「無反応 POI」を作れてしまうため)。
# 最小値 0.001 (= xy: 1 mm / yaw: 約 0.057°)。「制約を実質緩めたい」場合は
# 大きい値を明示的に指定する (例: yaw=3.14 ≒ 360° tolerance で yaw 不問)。
# yaw の挙動上の用途: RViz の扇形描画に加え、waypoint_arrival_mode=mapoi の到達判定にも使う
# (#261/#265)。route 中間 waypoint / 最終 goal / 単発 Go すべて統一で、到達 =
# (tolerance.xy ∧ tolerance.yaw) ∨ Nav2 SUCCEEDED。robot 姿勢が tolerance.yaw 以内なら radius
# 進入で即到達/前進、超えていれば Nav2 SUCCEEDED (姿勢合わせ) を待つ。通過点で yaw を問わない
# なら tolerance.yaw を大きく (例 3.14 ≒ 360°) する。waypoint_arrival_mode=nav2 では yaw は
# Nav2 goal_checker 任せで本フィールドは挙動非依存 (RViz 描画のみ)。
float64 xy        # Euclidean tolerance (m)、>= 0.001
float64 yaw       # Angular tolerance (rad)、>= 0.001
