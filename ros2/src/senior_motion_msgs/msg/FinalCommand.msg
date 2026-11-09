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
