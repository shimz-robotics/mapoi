# Navigation backend presence and readiness summary published by mapoi navigation
# bridges (e.g. mapoi_nav_server). Consumers (mapoi_webui, mapoi_rviz_plugins)
# use this to gate navigation UI when no backend is ready.

# Backend identifier (e.g. "nav2", "custom", "unknown")
string backend_type

# True if a navigation bridge node is running.
bool backend_present

# True if the bridge can actually execute navigation commands right now
# (i.e. underlying action / service servers are available).
bool backend_ready

# Per-capability readiness flags. backend_ready is the AND of the capabilities
# that a given backend declares as required.
bool goal_ready
bool route_ready
bool switch_map_ready
bool initialpose_ready

# Optional human-readable reason populated when backend_ready is false
# (e.g. "navigate_to_pose action not available").
string reason
