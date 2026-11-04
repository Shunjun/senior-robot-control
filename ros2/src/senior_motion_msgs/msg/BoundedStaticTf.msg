# Native TF receipt, not a sensor capture or a freshness renewal.
string<=64 bridge_boot_id
string<=128 bridge_gid
string<=128 raw_gid
uint64 generation
uint64 sequence
# Original first callback within this lifetime; independent of current sequence.
uint64 first_sequence
builtin_interfaces/Time first_received_ros
float64 first_received_steady
builtin_interfaces/Time received_ros
float64 received_steady
# Admission is separate from the original receipt and may follow clock startup.
bool admitted
builtin_interfaces/Time admitted_from_ros
float64 admitted_from_steady
bool valid
string<=64 reason
senior_motion_msgs/StaticTransformEdge[<=32] edges
