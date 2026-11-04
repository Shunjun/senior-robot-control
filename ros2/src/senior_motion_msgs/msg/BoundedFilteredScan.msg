# Self-filtered beams retain the original bounded raw receipt and beam order.
senior_motion_msgs/RawCaptureHeader raw_capture
string<=64 source_epoch
uint64 sequence
bool valid
string<=64 reason
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[<=2048] ranges
