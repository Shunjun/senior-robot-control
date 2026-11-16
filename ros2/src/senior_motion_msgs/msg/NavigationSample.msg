# Same-host CLOCK_MONOTONIC, captured before computing this final velocity.
# clock_domain is Linux boot_id; run_id is the private run topic nonce.
string<=36 clock_domain
string<=32 run_id
uint64 sequence
uint64 produced_steady_ns
float64 vx
float64 wz
