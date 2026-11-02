# Original TF edge; stamp zero is valid static calibration metadata.
string<=128 parent_frame
string<=128 child_frame
builtin_interfaces/Time stamp
float64[3] translation
float64[4] rotation_xyzw
