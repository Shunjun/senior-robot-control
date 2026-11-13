#pragma once
// Private same-build seqpacket v2. The actual guard and native roundtrip tests
// share this codec; no old-frame/default-field compatibility is accepted.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace final_command_frame {
constexpr size_t size = 799;

inline void integer(std::vector<uint8_t>& out, uint64_t n, unsigned bytes) {
  for (int i = static_cast<int>(bytes) - 1; i >= 0; --i) {
    out.push_back(static_cast<uint8_t>(n >> (8 * i)));
  }
}
inline void number(std::vector<uint8_t>& out, double n) {
  if (!std::isfinite(n)) throw std::invalid_argument("nonfinite frame number");
  uint64_t bits;
  std::memcpy(&bits, &n, sizeof(bits));
  integer(out, bits, 8);
}
inline void text(std::vector<uint8_t>& out, const std::string& s, size_t bound) {
  if (s.size() > bound || s.find('\0') != std::string::npos) {
    throw std::invalid_argument("frame string bound");
  }
  out.insert(out.end(), s.begin(), s.end());
  out.insert(out.end(), bound + 1 - s.size(), 0);
}

template<class Command, class Gid>
std::vector<uint8_t> encode(uint8_t kind, const Gid& gid, const Command& m) {
  if (kind > 2 || gid.size() != 24) throw std::invalid_argument("frame kind/gid");
  std::vector<uint8_t> out{'S', 'C', 'F', '2', 2, kind};
  integer(out, size, 2);
  out.insert(out.end(), gid.begin(), gid.end());
  text(out, m.robot_id, 128);
  text(out, m.owner_boot_id, 128);
  text(out, m.session_id, 128);
  text(out, m.request_id, 128);
  integer(out, m.generation, 8);
  integer(out, m.connection_epoch, 8);
  integer(out, m.sequence, 8);
  integer(out, m.gesture, 8);
  number(out, m.steady_deadline);
  number(out, m.vx);
  number(out, m.wz);
  text(out, m.source_kind, 16);
  text(out, m.source_id, 128);
  integer(out, m.source_epoch, 8);
  number(out, m.received_steady);
  text(out, m.close_reason, 32);
  if (out.size() != size) throw std::logic_error("frame size mismatch");
  return out;
}

template<class C> bool same_payload(const C& a, const C& b) {
  return a.robot_id == b.robot_id && a.owner_boot_id == b.owner_boot_id &&
    a.session_id == b.session_id && a.request_id == b.request_id &&
    a.generation == b.generation && a.connection_epoch == b.connection_epoch &&
    a.sequence == b.sequence && a.gesture == b.gesture &&
    a.steady_deadline == b.steady_deadline && a.vx == b.vx && a.wz == b.wz &&
    a.source_kind == b.source_kind && a.source_id == b.source_id &&
    a.source_epoch == b.source_epoch && a.received_steady == b.received_steady &&
    a.close_reason == b.close_reason;
}

template<class C> bool barrier(const C& m) {
  if (m.vx != 0 || m.wz != 0 || m.sequence != 0 || m.steady_deadline != 0) return false;
  if (m.close_reason == "estop_reset" || m.close_reason == "fault_recover") {
    return !m.request_id.empty() && m.source_kind == "idle" &&
      m.source_id.empty() && m.source_epoch == 0;
  }
  if (m.gesture != 0) return false;
  return m.close_reason == "prepared" || m.close_reason == "release" ||
    m.close_reason == "expiry" || m.close_reason == "connection_retired" ||
    m.close_reason == "estop" || m.close_reason == "fault" ||
    m.close_reason == "task_revoke" || m.close_reason == "qualification_lost" ||
    m.close_reason == "shutdown";
}
}  // namespace final_command_frame
