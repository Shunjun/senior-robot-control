# Captured by the producer/driver before DDS or processing queues, never restamped.
# driver_read is driver receipt, not a claim about unknown pre-driver buffering.
string<=36 clock_domain
string<=64 source_epoch
uint64 sequence
uint64 capture_start_steady_ns
uint64 capture_end_steady_ns
string<=16 time_basis
# A physical world/reset coordinate, distinct from producer source_epoch.
# Only sim_step uses these fields; its tick 0 is valid. Other bases ignore them.
string<=64 measurement_epoch
uint64 measurement_tick_ns
