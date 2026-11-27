# Sensor stamp and first bridge receipts, not Python receipt or publication time.
builtin_interfaces/Time stamp
builtin_interfaces/Time received_ros
uint64 received_steady_ns
uint64 sample_steady_ns
uint64 clock_generation
string frame_id
# Lowercase hex encoding of the actual rclcpp MessageInfo publisher GID.
string raw_gid
# Lowercase hex encoding of the typed bridge publisher's own get_gid().
string bridge_gid
string source_epoch
# Receiver process identity. Generation changes only on stream/clock reinitialization.
string bridge_boot_id
uint64 capture_generation
uint64 sequence
bool valid
string<=64 reason
