# Attributed cloud windows

`rolling_bonxai_cloud` provides bounded crops of an existing point map and
checksummed lossless zstd exchange. It sits alongside Rolling-Bonxai's
occupancy mapper and preserves map points without another voxelization.

`CloudWindow` accepts XYZ/intensity/reflectivity records in one named map
frame, selects a half-open axis-aligned box, and enforces point/payload
budgets. Oversized windows fail explicitly; they are never silently thinned.
Encoding `xyz_intensity_reflectivity_f32le_planes_zstd_v1` uses four byte
planes (least to most significant). Each plane contains the x, y, z, intensity,
and reflectivity arrays, each of `point_count` bytes, inside a zstd frame with
content size and checksum. This rearrangement preserves every IEEE-754 float32
bit and improves compression without moving points. Unknown radiometry is NaN.

`rolling_bonxai_msgs/msg/SurfaceMapWindow` carries the sender/map identities,
frame, snapshot and graph revisions, selected pose and source time, bounds,
resolution and payload. Packets replace a complete window. They are not deltas.

Build with:

```bash
colcon build --packages-up-to rolling_bonxai_cloud
source install/setup.bash
ros2 run rolling_bonxai_cloud surface_window_receiver --ros-args \
  -r map_window/packet:=/sender/map_window/packet
```

The receiver publishes `map_window/received_cloud` in the sender's frame and
validates metadata, budgets, bounds, frame size and checksum. It has its own
`max_points` (1,000,000) and `max_payload_bytes` (8 MiB) limits. Empty snapshots
publish empty replacement clouds. This receiver needs neither DELIRIOM, PCL
nor the occupancy engine. Inter-platform alignment/fusion is separate; preserve
the sender's frame and establish its transform before combining maps.

The ROS-free `window_core` library uses C++17 and zstd. `window_ros` adds ROS
message validation and conversion. See the companion DELIRIOM
`doc/rederive/MAP_WINDOWS.md` for the integrated publisher and RViz workflow.
