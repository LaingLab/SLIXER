# The SO-101 model

`so101.urdf` is `Simulation/SO101/so101_new_calib.urdf` from
[TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), unchanged apart from its name
(upstream commit `385e8d7c68e2`). The meshes Slixer shows, in `slixer/web/models/`, were made from that
model's STL files by `slixer/bake_meshes.py`: each link's meshes merged into one and simplified.

Both are the work of TheRobotStudio, used under the Apache License 2.0 (`LICENSE`, beside this file).

The original STLs aren't included: they're 16 MB, and only needed to re-bake the meshes. To do that, put
upstream's `Simulation/SO101/assets/` folder here as `assets/` and run `python slixer/bake_meshes.py`.
