# mapoi_rviz_plugins

> Japanese version: [README.ja.md](./README.ja.md)

RViz2 plugin package for mapoi. Provides a GUI for map switching, POI selection, autonomous navigation operation, and POI editing.

## Plugins

### MapoiPanel (panel plugin)

A navigation operation panel that can be added as an RViz2 panel.

- Select a map to switch to from a dropdown
- Select a destination (a POI tagged `goal`) from a list, scoped to the current map
- Select a route from a list and start navigation along it
- Correct the robot's estimated pose to a selected POI's position (setting the Initial Pose)
- Start autonomous navigation to a selected POI
- Pause / resume / cancel autonomous navigation
- Display navigation status (`navigating`, `succeeded`, `aborted`, `paused`, `canceled`)
- Highlight the selected POI/route in RViz2

#### Publishers

| Topic | Type | Description |
| --- | --- | --- |
| `mapoi/nav/goal_pose_poi` | `std_msgs/String` | Publishes the goal POI name. The actual Nav2 action call is handled by the navigation bridge (`mapoi_nav2_bridge` etc.) |
| `mapoi/nav/pause` | `std_msgs/String` | Pauses navigation |
| `mapoi/nav/resume` | `std_msgs/String` | Resumes navigation |
| `mapoi/nav/cancel` | `std_msgs/String` | Cancels navigation |
| `mapoi/nav/route` | `std_msgs/String` | Starts route navigation |
| `mapoi/highlight/goal` | `std_msgs/String` | Highlights the goal marker |
| `mapoi/highlight/route` | `std_msgs/String` | Highlights the route marker |

#### Service clients

| Service | Type | Description |
| --- | --- | --- |
| `mapoi/request_initial_pose` | `RequestInitialPose` | Requests `mapoi_server` to publish when setting the Initial Pose (#211). The panel does not publish `mapoi/initialpose_poi` directly |

#### Subscribers

| Topic | Type | Description |
| --- | --- | --- |
| `mapoi/config_path` | `std_msgs/String` | Detects changes to the config file path |
| `mapoi/nav/status` | `std_msgs/String` | Displays navigation status (`"status"` / `"status:target"` format, transient_local QoS). If a target is included, a late-joining panel can restore messages like "Navigating: target" / "Arrived: target" |

### PoiEditor (panel plugin)

A panel that displays, edits, and saves POI information in tabular form.

- Loads POI information from the current config file
- View and edit POI information in a table (5-column layout: name, pose, tolerance, tags, description, #158)
  - The **pose** column uses `x, y, yaw` format (e.g. `1.00, 2.00, 0.79`; x/y in m, yaw in rad, displayed with 2 decimal places)
  - The **tolerance** column uses a single-column combined `xy, yaw` format (e.g. `0.50, 0.79`; xy in m, yaw in rad, displayed with 2 decimal places)
    - Validation constraints: `xy >= 0.001 m` / `0.001 rad <= yaw <= 2π rad`
    - The `2π rad` (= 360°) maximum is a guard that rejects old deg-based inputs (e.g. `45`) mistakenly entered as rad after the deg → rad unit change (#158)
  - **description** is the last column (long text is fine, avoids squeezing the other columns)
- Add / copy / delete POIs
- Filter the displayed list by tag
- Tag input assistance via a TagHelperComboBox (system tags shown with an `[S]` marker)
- Save with validation (duplicate-name check, coordinate format, `tolerance.xy` check, warnings for undefined tags)
- Position input in conjunction with MapoiPoseTool
- Automatically reloads `mapoi_server`'s config after saving

#### Subscribers

| Topic | Type | Description |
| --- | --- | --- |
| `mapoi_rviz_pose` | `geometry_msgs/PoseStamped` | Position input from MapoiPoseTool |
| `mapoi/config_path` | `std_msgs/String` | Detects changes to the config file path |

### MapoiPoseTool (tool plugin)

A pose-specification tool that can be added to the RViz2 toolbar. Shortcut key: `i`

- Click and drag on RViz2 to specify a position and orientation
- Reflects the position onto the selected row in PoiEditor
- Automatically switches back to the default tool after use

#### Publishers

| Topic | Type | Description |
| --- | --- | --- |
| `mapoi_rviz_pose` | `geometry_msgs/PoseStamped` | Publishes the specified position and orientation |

## Adding to RViz2

1. Launch RViz2
2. Add `MapoiPanel` or `PoiEditor` via Panels > Add New Panel
3. Add `MapoiPoseTool` via the "+" button on the toolbar

## Dependencies

- `rviz_common`
- `rviz_default_plugins`
- `std_srvs`
- `mapoi_interfaces`
