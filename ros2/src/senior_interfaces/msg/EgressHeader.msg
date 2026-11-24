# Native relay and actual DDS owner identity; not an observation receipt or TTL.
string<=128 relay_gid
string<=64 relay_boot
uint64 relay_sequence
string<=128 owner_gid
string<=64 owner_epoch
uint64 owner_generation
uint64 owner_sequence
bool valid
bool owner_lost
bool upstream_invalid
string<=64 reason
