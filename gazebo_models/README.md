# Vendored Gazebo default models

This directory contains the minimal subset of Gazebo default models
required by `turtlebot3_*.world` files when running the Docker demo.

## Why vendor them

The Docker image sets `GAZEBO_MODEL_DATABASE_URI=` (empty) to prevent
Gazebo from blocking on `gazebosim.org` at startup. With the URI
disabled, Gazebo cannot fetch these models online, so we ship them in
the repo and `COPY` them into `~/.gazebo/models/` at image build time.

## Source

Copied verbatim from the upstream [osrf/gazebo_models][upstream]
repository (Apache-2.0 licensed).

- `ground_plane/` — flat infinite ground plane with collision
- `sun/` — directional light source

To refresh from upstream:

```bash
for model in ground_plane sun; do
  for file in model.config model.sdf; do
    curl -fsSL -o gazebo_models/${model}/${file} \
      https://raw.githubusercontent.com/osrf/gazebo_models/master/${model}/${file}
  done
done
```

[upstream]: https://github.com/osrf/gazebo_models
