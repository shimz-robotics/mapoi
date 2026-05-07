# Minimal navigation backend readiness, published 1 Hz on
# `mapoi/nav/backend_status` by navigation bridges such as
# `mapoi_nav_server` (Nav2 bridge) or any custom bridge.
#
# Custom bridges only need to populate these three fields. UI consumers
# (mapoi_webui, mapoi_rviz_plugins) gate navigation operations on
# `backend_ready` alone. Per-capability detail belongs to `reason` or to
# bridge-specific topics, not this message.
#
# Localization readiness (initial pose / AMCL etc.) is intentionally NOT
# part of this contract; it will be exposed by a separate
# `LocalizationBackendStatus` topic — see issue #209.
#
# QoS contract (issue #208):
#   durability               : TRANSIENT_LOCAL  (late-joiner UI receives the latest status)
#   reliability              : RELIABLE
#   liveliness (publisher)   : MANUAL_BY_TOPIC  (each publish() asserts liveliness on this topic)
#   liveliness (subscriber)  : AUTOMATIC        (compatibility with legacy publishers; see
#                                                ROS 2 QoS compatibility table)
#   liveliness_lease_duration: 5 s              (publishers MUST publish at least every 5 s;
#                                                subscribers MUST set lease_duration >= 5 s)
# Subscribers detect bridge death by handling the Liveliness Changed event
# (alive_count transitions to 0). UI gating MUST treat a lost-liveliness
# state as `backend_ready=false` regardless of the latched payload.
#
# NOTE (issue #213): the lease must exceed any blocking call inside the publisher
# node's executor. mapoi_nav_server's `select_map` flow can block the
# SingleThreadedExecutor for up to ~11 s per Nav2 map node during `send_load_map_request`,
# which can trigger false-positive liveliness lost events even at 5 s lease.
# Issue #213 tracks splitting backend_status_timer into a Reentrant callback group
# + MultiThreadedExecutor so the timer keeps publishing during such blocking calls.

# Bridge identifier shown in tooltips (e.g. "nav2", "custom_lidar_planner").
# Free-form, only used to help operators distinguish multiple bridges.
string backend_type

# True when the bridge can accept and execute navigation commands
# (goal / route / pause / resume / cancel / map switch) right now.
bool backend_ready

# Optional human-readable reason populated when backend_ready is false.
string reason
