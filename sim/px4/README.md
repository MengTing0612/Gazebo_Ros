# PX4 Source Cache

`scripts/bootstrap_px4_ros2.sh` downloads PX4-Autopilot tag `v1.17.0` into
this directory as `PX4-Autopilot/`.

The source is intentionally not committed here because PX4 and its recursive
submodules are large.  The tag is pinned to match both the MicoAir743v2 board
firmware target and `ros2_ws/src/px4_msgs` branch `release/1.17`.
