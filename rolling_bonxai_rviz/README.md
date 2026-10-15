# RViz map-window panel

Build `rolling_bonxai_rviz` with your ROS 2 RViz installation; the build matches
RViz's exported Qt major version (locally tested on Lyrical / Qt 6). Add
`rolling_bonxai_rviz/MapWindowPanel` using RViz's Panels menu, and set the
mapping node's namespace. The panel listens to `map_window/state` and sends
zero-based pose selections on `map_window/select`. `-1` follows the latest
pose. The slider and Previous/Next controls are debounced, and Follow live
returns to the latest available pose. Namespace settings are saved in RViz.

Use PointCloud2 (`map_window/cloud`), Marker (`map_window/bounds`) and Pose
(`map_window/center`) displays to inspect the selected map window alongside
the global map. Windows show the current optimized geometry around a selected
pose, including observations acquired later. They are not historical replay.

ROS callbacks only update a bounded mailbox; widget updates run on Qt's GUI
thread. Headless tests exercise pose selection, returning to live following,
and changing namespaces without accepting stale state.
