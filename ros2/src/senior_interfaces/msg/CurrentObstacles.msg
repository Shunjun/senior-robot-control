# Same-host absolute deadline; receiving/caching this message never renews it.
string<=36 backend_boot
string<=36 host_boot_id
string<=128 map_id
uint64 reference_revision
string<=128 map_generation
senior_interfaces/RawCaptureHeader capture
bool valid
string<=64 reason
uint64 valid_until_steady_ns
string frame_id
geometry_msgs/Point32[] points
