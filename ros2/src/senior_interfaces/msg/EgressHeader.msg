# Native relay and actual DDS owner identity; not an observation receipt or TTL.
string relay_gid
string relay_boot
uint64 relay_sequence
string owner_gid
string owner_epoch
uint64 owner_generation
uint64 owner_sequence
bool valid
bool owner_lost
bool upstream_invalid
string<=64 reason
