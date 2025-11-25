# mapoi packages

`mapoi` is a meta package to manage `map` and `poi` for Navigation2.

## Build and execute example

```sh
# cd path/to/your_ws
git clone git@github.com:shimz-robotics/mapoi.git src/mapoi_packages
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch mapoi_turtlebot3_example turtlebot3_navigation_launch.yaml
```

Open another terminal

```sh
ros2 run mapoi_server mapoi_nav_server --ros-args -p route_name:=route_1
```

Edit `src/mapoi_packages/mapoi_turtlebot3_example/maps/turtlebot3_world/map_info.yaml`

## Make your original map

make maps directory in `your_package/maps/site`.
Add `mapoi_poi_server` with `map_info_file` in `your_robot_launch.yaml`.

```your_robot_launch.yaml
# --- Mapoi Server ---
- node:
    pkg: mapoi_server
    exec: mapoi_poi_server
    param:
      - {name: map_info_file, value: "$(find-pkg-share mapoi_server)/maps/turtlebot3_world/map_info.yaml"}
```

To test, build your package, source `install/setup.bash`, launch `your_robot_launch.yaml` and command in another terminal.

```sh
ros2 run mapoi_server mapoi_nav_server --ros-args -p route_name:=route_1
```

## contents

### mapoi_server

**This is a main package of `mapoi`.**
`mapoi_server` provides map and poi information to the other packages.
See the [README.md](/mapoi_server/README.md).

### mapoi_interfaces

`mapoi_interfaces` consolidates `msg` and `srv` files used by mapoi packages.
See the [README.md](/mapoi_interfaces/README.md).

### mapoi_rviz_plugins

`mapoi_rviz_plugins` provides GUI-based operation using RViz.
See the [README.md](/mapoi_rviz_plugins/README.md).

