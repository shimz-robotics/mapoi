# Spatial tolerance (Nav2 SimpleGoalChecker と同 semantics)
float64 xy        # Euclidean tolerance (m)、Nav2 xy_goal_tolerance と同等
float64 yaw       # angular tolerance (rad)、Nav2 yaw_goal_tolerance と同等
                  # 0 = 未指定として扱い Nav2 default にフォールバック
