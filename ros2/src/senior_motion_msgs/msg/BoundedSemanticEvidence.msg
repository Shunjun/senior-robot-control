senior_motion_msgs/EgressHeader depth_owner
senior_motion_msgs/RawCaptureHeader rgb_capture
senior_motion_msgs/RawCaptureHeader rgb_camera_info_capture
# Original capture and first receipts; no publication-time replacement.
builtin_interfaces/Time rgb_stamp
builtin_interfaces/Time rgb_received_ros
builtin_interfaces/Time rgb_camera_info_stamp
builtin_interfaces/Time rgb_camera_info_received_ros
float64 rgb_received_steady
float64 rgb_camera_info_received_steady
string<=128 rgb_frame_id
string<=128 rgb_source
string<=128 rgb_camera_info_source
string<=64 source_epoch
string<=64 registration_id
uint64 sequence
bool valid
string<=64 reason
# Registered color calibration actually used for this capture.
uint16 rgb_width
uint16 rgb_height
string<=32 distortion_model
float64[<=16] d
float64[9] k
float64[9] r
float64[12] p
uint16 binning_x
uint16 binning_y
uint32[4] roi
bool roi_do_rectify
# Complete bounded original depth evidence, not just the matched proposals.
senior_motion_msgs/BoundedDepthEvidence depth
senior_motion_msgs/SemanticObject[<=32] objects
senior_motion_msgs/SemanticAssociation[<=8192] associations
