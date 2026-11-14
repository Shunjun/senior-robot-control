# All timestamps use the host monotonic clock shared by the local ROS processes.
string<=128 robot_id
string<=128 owner_boot_id
string<=128 session_id
uint64 generation
uint64 connection_epoch
uint64 sequence
float64 steady_deadline
float64 vx
float64 wz
string<=128 request_id
uint64 gesture
# v2 input target identity. Local output generation remains the field above.
string<=16 source_kind
string<=128 source_id
uint64 source_epoch
float64 received_steady
# prepared|intent|release|expiry|connection_retired|estop|fault|task_revoke
# qualification_lost|shutdown|estop_reset|fault_recover
string<=32 close_reason
# Explicit user ESTOP: non-navigation source, request_id preserved, gesture=0.
# Recovery controls: idle source, request_id preserved, gesture=expected T5 revision.
