# Sensor stamp and first bridge receipts, not Python receipt or publication time.
builtin_interfaces/Time stamp
builtin_interfaces/Time received_ros
float64 received_steady
string<=128 frame_id
# Lowercase hex encoding of the actual rclcpp MessageInfo publisher GID.
string<=128 raw_gid
# Lowercase hex encoding of the typed bridge publisher's own get_gid().
string<=128 bridge_gid
string<=64 source_epoch
# Shared by all four sources; increments whenever any source changes or fails.
string<=64 bridge_boot_id
uint64 capture_generation
uint64 sequence
bool valid
string<=64 reason
