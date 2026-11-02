# Original bounded image metadata; this message deliberately carries no pixel body.
senior_motion_msgs/RawCaptureHeader capture
uint32 width
uint32 height
uint32 step
string<=32 encoding
bool is_bigendian
uint32 byte_length
# SHA-256 of the exact original data bytes, including row padding.
uint8[32] sha256
