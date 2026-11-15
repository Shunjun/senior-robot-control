#pragma once
// Private same-build seqpacket v3. The actual guard and native roundtrip tests
// share this codec; no old-frame/default-field compatibility is accepted.
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <vector>

namespace final_command_frame {
constexpr size_t size = 823;

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
  if (kind > 4 || gid.size() != 24) throw std::invalid_argument("frame kind/gid");
  std::vector<uint8_t> out{'S', 'C', 'F', '3', 3, kind};
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
  number(out, m.manual_linear);
  number(out, m.manual_linear_limit);
  number(out, m.degraded_linear_limit);
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
    a.close_reason == b.close_reason && a.manual_linear == b.manual_linear &&
    a.manual_linear_limit == b.manual_linear_limit &&
    a.degraded_linear_limit == b.degraded_linear_limit;
}

template<class C> bool manual_metadata(const C& m) {
  if (!std::isfinite(m.manual_linear) || !std::isfinite(m.manual_linear_limit) ||
      !std::isfinite(m.degraded_linear_limit)) return false;
  if (m.source_kind != "manual" || m.close_reason != "intent") {
    return m.manual_linear == 0 && m.manual_linear_limit == 0 && m.degraded_linear_limit == 0;
  }
  return std::abs(m.manual_linear) <= 1 && m.manual_linear_limit >= 0 &&
    m.degraded_linear_limit >= 0 &&
    m.vx == m.manual_linear * m.manual_linear_limit;
}

// Identity notifications cannot enter the actuator generation/sequence frontier.
template<class C> bool terminal_identity(const C& m) {
  return manual_metadata(m) && m.source_kind == "navigation" && !m.source_id.empty() && m.source_id == m.session_id &&
    !m.owner_boot_id.empty() && !m.request_id.empty() && m.source_epoch > 0 &&
    (m.close_reason == "task_terminal" || (m.close_reason == "task_settled" &&
      m.generation == 0 && m.connection_epoch == m.source_epoch)) && m.sequence == 0 && m.steady_deadline == 0 &&
    m.vx == 0 && m.wz == 0 && m.gesture == 0;
}

template<class C> bool terminal_identity_authorized(
    const C& m, const std::string& robot, const std::string& boot, bool invalid_boot, double now) {
  if (!terminal_identity(m) || invalid_boot || boot.empty() || boot != m.owner_boot_id ||
      m.robot_id != robot || !std::isfinite(now) || !std::isfinite(m.received_steady) ||
      m.received_steady < 0 || m.received_steady > now) return false;
  for (const auto* text : {&m.robot_id, &m.owner_boot_id, &m.session_id, &m.request_id, &m.source_id})
    if (text->size() > 128 || text->find('\0') != std::string::npos) return false;
  return true;
}

template<class C> bool barrier(const C& m) {
  if (!manual_metadata(m)) return false;
  if (m.vx != 0 || m.wz != 0 || m.sequence != 0 || m.steady_deadline != 0) return false;
  if (m.close_reason == "estop_reset" || m.close_reason == "fault_recover") {
    return !m.request_id.empty() && m.source_kind == "idle" &&
      m.source_id.empty() && m.source_epoch == 0;
  }
  if (m.gesture != 0) return false;
  if (m.close_reason == "task_done") return m.source_kind == "navigation";
  return m.close_reason == "prepared" || m.close_reason == "release" ||
    m.close_reason == "expiry" || m.close_reason == "connection_retired" ||
    m.close_reason == "estop" || m.close_reason == "fault" ||
    m.close_reason == "task_revoke" || m.close_reason == "qualification_lost" ||
    m.close_reason == "shutdown";
}
}  // namespace final_command_frame
