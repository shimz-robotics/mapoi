# Minimal navigation backend readiness, published 1 Hz on
# `mapoi/nav/backend_status` (transient_local) by navigation bridges
# such as `mapoi_nav_server` (Nav2 bridge) or any custom bridge.
#
# Custom bridges only need to populate these three fields. UI consumers
# (mapoi_webui, mapoi_rviz_plugins) gate navigation operations on
# `backend_ready` alone. Per-capability detail belongs to `reason` or to
# bridge-specific topics, not this message.
#
# Localization readiness (initial pose / AMCL etc.) is intentionally NOT
# part of this contract; it will be exposed by a separate
# `LocalizationBackendStatus` topic — see issue #209.

# Bridge identifier shown in tooltips (e.g. "nav2", "custom_lidar_planner").
# Free-form, only used to help operators distinguish multiple bridges.
string backend_type

# True when the bridge can accept and execute navigation commands
# (goal / route / pause / resume / cancel / map switch) right now.
bool backend_ready

# Optional human-readable reason populated when backend_ready is false.
string reason
