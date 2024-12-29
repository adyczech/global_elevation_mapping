# global_elevation_mapping

Global elevation mapping package relies on the [grid_map](https://github.com/anybotics/grid_map) package for creating global elevation map.

It subscribes to local elevation map (`input_grid_map_topic`) in `robot_frame` and publishes the assembled global elevation map (`output_grid_map_topic`) in designated `global_frame`.

Configuration of the package is done in a yaml file.
