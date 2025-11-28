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

Edit `src/mapoi_packages/mapoi_turtlebot3_example/maps/turtlebot3_world/mapoi_config.yaml`

## Make your original map

make maps directory in `your_package/maps/site`.

```
# mapoi_turtlebot3_example
maps
├── turtlebot3_dqn_stage1
│   ├── mapoi_config.yaml
│   ├── turtlebot3.pgm
│   └── turtlebot3.yaml
└── turtlebot3_world
    ├── mapoi_config.yaml
    ├── turtlebot3.pgm
    └── turtlebot3.yaml
```

Add `mapoi_server` with `mapoi_config_file` in `your_robot_launch.yaml`.

```your_robot_launch.yaml
# Mapoi Server
- node:
    pkg: mapoi_server
    exec: mapoi_server
    param:
      - {name: maps_path, value: "$(find-pkg-share mapoi_turtlebot3_example)/maps"}
      - {name: map_name, value: "turtlebot3_world"}
      - {name: config_file, value: "mapoi_config.yaml"}
      - {name: pub_interval_ms, value: 500}

- node:
    pkg: mapoi_server
    exec: mapoi_nav_server

- node:
    pkg: mapoi_server
    exec: mapoi_rviz2_publisher
```

Configure parameters to automatically set the initial position and enable map switching when you use `amcl`.

```
amcl:
  ros__parameters:
    ...
    ...
    ...
    # Added parameters for initial pose
    set_initial_pose: True
    initial_pose: {x: -2.0, y: -0.5, yaw: 0.0}
    # Added parameter to switch maps
    first_map_only_: False
```

To test, build your package, source `install/setup.bash`, launch `your_robot_launch.yaml` and command in another terminal.

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
