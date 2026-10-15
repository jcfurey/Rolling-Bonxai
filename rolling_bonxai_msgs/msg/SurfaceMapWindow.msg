# Points describe the CURRENT optimized map cropped around center_keyframe.
# This is a full replacement snapshot, not a delta or a historical observation.
# All XYZ and center_pose values are in header.frame_id (the sender's map).
# extent is axis-aligned in that frame. A receiver needs a separately estimated
# transform between platforms before fusing their maps.
std_msgs/Header header
uint32 schema_version
string<=256 source_id
string<=64 map_id
uint64 snapshot_revision
uint64 graph_revision
int64 center_keyframe
uint64 keyframe_count
builtin_interfaces/Time source_stamp
builtin_interfaces/Time center_stamp
geometry_msgs/Pose center_pose
geometry_msgs/Vector3 extent
float64 resolution
bool cleared
uint32 point_count
uint32 uncompressed_bytes
# xyz_intensity_reflectivity_f32le_planes_zstd_v1: four low-to-high byte
# planes, each with x/y/z/intensity/reflectivity arrays of point_count bytes.
# Intensity/reflectivity NaN means unknown. Zstd includes size and checksum.
string<=64 encoding
uint8[<=33554432] data
