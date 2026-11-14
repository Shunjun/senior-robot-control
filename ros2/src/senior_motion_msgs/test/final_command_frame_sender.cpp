#include "final_command_frame.hpp"
#include <sys/socket.h>
#include <cstdlib>
#include <string>

struct Command {
  std::string robot_id = "route-sim", owner_boot_id = "boot";
  std::string session_id = "session", request_id = "token";
  uint64_t generation = 5, connection_epoch = 7, sequence = 9, gesture = 1, source_epoch = 42;
  double steady_deadline = 10.55, vx = .4, wz = .2, received_steady = 10.25;
  std::string source_kind = "manual", source_id = "authority", close_reason = "intent";
};

void zero_barrier(Command& m, const std::string& reason) {
  m.vx = m.wz = 0;
  m.sequence = m.gesture = 0;
  m.steady_deadline = 0;
  m.close_reason = reason;
}

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  Command m;
  std::string mode = argv[2];
  if (mode == "manual_release" || mode == "manual_intent") {
    m.generation = 6;
    mode = mode == "manual_release" ? "release" : "intent";
  }
  if (mode == "release" || mode == "navrelease") {
    zero_barrier(m, "release");
    if (mode == "navrelease") {
      m.source_kind = "navigation";
      m.source_id = m.session_id = "execution";
      m.close_reason = "task_revoke";
    }
    if (!final_command_frame::barrier(m)) return 10;
  }
  if (mode == "press1" || mode == "press2" || mode == "echo") {
    zero_barrier(m, "estop");
    m.generation = mode == "press1" ? 7 : mode == "press2" ? 8 : 9;
    m.request_id = mode == "echo" ? "" : mode;
    if (!final_command_frame::barrier(m)) return 18;
  }
  if (mode == "reset" || mode == "qualification_lost") {
    zero_barrier(m, mode == "reset" ? "estop_reset" : "qualification_lost");
    if (mode == "reset") {
      m.source_kind = "idle";
      m.source_id = "";
      m.source_epoch = 0;
    }
    if (!final_command_frame::barrier(m)) return 16;
    if (mode == "reset") {
      Command wrong = m;
      wrong.source_kind = "manual";
      if (final_command_frame::barrier(wrong)) return 17;
    }
  }
  if (mode == "boundary") m.source_id = std::string(128, 'x');
  if (mode == "oversize") m.source_id = std::string(129, 'x');

  Command changed = m;
  changed.source_epoch++;
  if (final_command_frame::same_payload(m, changed)) return 11;
  changed = m;
  changed.received_steady += .01;
  if (final_command_frame::same_payload(m, changed)) return 12;
  changed = m;
  changed.source_kind = m.source_kind == "navigation" ? "manual" : "navigation";
  if (final_command_frame::same_payload(m, changed)) return 13;
  changed = m;
  changed.source_id = "other";
  if (final_command_frame::same_payload(m, changed)) return 14;
  changed = m;
  changed.close_reason = "shutdown";
  if (final_command_frame::same_payload(m, changed)) return 15;

  std::array<uint8_t, 24> gid{};
  gid[0] = 17;
  try {
    const auto frame = final_command_frame::encode(1, gid, m);
    const auto sent = ::send(std::atoi(argv[1]), frame.data(), frame.size(), 0);
    return sent == static_cast<ssize_t>(frame.size()) ? 0 : 3;
  } catch (const std::invalid_argument&) {
    return 42;
  }
}
