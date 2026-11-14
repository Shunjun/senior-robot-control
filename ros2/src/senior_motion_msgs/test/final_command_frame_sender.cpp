#include "final_command_frame.hpp"
#include <sys/socket.h>
#include <cstdlib>
#include <string>

struct Command {
  std::string robot_id = "route-sim", owner_boot_id = "boot";
  std::string session_id = "session", request_id = "token";
  uint64_t generation = 5, connection_epoch = 7, sequence = 9, gesture = 1, source_epoch = 42;
  double steady_deadline = 10.55, vx = .4, wz = .2, received_steady = 10.25;
  double manual_linear = .5, manual_linear_limit = .8, degraded_linear_limit = .3;
  std::string source_kind = "manual", source_id = "authority", close_reason = "intent";
};

void zero_barrier(Command& m, const std::string& reason) {
  m.vx = m.wz = 0;
  m.manual_linear = m.manual_linear_limit = m.degraded_linear_limit = 0;
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
  if (mode == "release" || mode == "navrelease" || mode == "navdone") {
    zero_barrier(m, "release");
    if (mode == "navrelease" || mode == "navdone") {
      m.source_kind = "navigation";
      m.source_id = m.session_id = "execution";
      m.close_reason = mode == "navdone" ? "task_done" : "task_revoke";
    }
    if (!final_command_frame::barrier(m)) return 10;
    if (mode == "navdone") {
      for (int mutation = 0; mutation < 5; ++mutation) {
        Command invalid = m;
        if (mutation == 0) invalid.vx = .01;
        if (mutation == 1) invalid.steady_deadline = 10.55;
        if (mutation == 2) invalid.sequence = 1;
        if (mutation == 3) invalid.gesture = 1;
        if (mutation == 4) invalid.source_kind = "manual";
        if (final_command_frame::barrier(invalid)) return 19;
      }
    }
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
  if (mode == "terminal" || mode == "collected") {
    zero_barrier(m, "task_terminal");
    m.source_kind = "navigation";
    m.source_id = m.session_id = "execution";
    if (mode == "collected") {
      m.close_reason = "task_settled"; m.generation = 0; m.connection_epoch = m.source_epoch;
    }
    if (final_command_frame::barrier(m)) return 20;
    if (!final_command_frame::terminal_identity_authorized(m, m.robot_id, m.owner_boot_id, false, 10.3)) return 21;
    for (int mutation = 0; mutation < 10; ++mutation) {
      Command wrong = m;
      if (mutation == 0) wrong.vx = .1;
      if (mutation == 1) wrong.steady_deadline = 10.55;
      if (mutation == 2) wrong.sequence = 1;
      if (mutation == 3) wrong.gesture = 1;
      if (mutation == 4) wrong.source_epoch = 0;
      if (mutation == 5) wrong.source_kind = "manual";
      if (mutation == 6) wrong.session_id = "different";
      if (mutation == 7) wrong.received_steady = 10.4;
      if (mutation == 8) wrong.request_id = "";
      if (mutation == 9) wrong.source_id = std::string(129, 'x');
      if (final_command_frame::terminal_identity_authorized(wrong, m.robot_id, m.owner_boot_id, false, 10.3)) return 22;
    }
    if (final_command_frame::terminal_identity_authorized(m, m.robot_id, "", false, 10.3) ||
        final_command_frame::terminal_identity_authorized(m, m.robot_id, "other", false, 10.3) ||
        final_command_frame::terminal_identity_authorized(m, m.robot_id, m.owner_boot_id, true, 10.3)) return 23;
  }
  if (mode == "boundary") m.source_id = std::string(128, 'x');
  if (mode == "oversize") m.source_id = std::string(129, 'x');

  // Metadata participates in duplicate identity just like velocity and time.
  for (int field = 0; field < 3; ++field) {
    Command changed = m;
    if (field == 0) changed.manual_linear += .01;
    if (field == 1) changed.manual_linear_limit += .01;
    if (field == 2) changed.degraded_linear_limit += .01;
    if (final_command_frame::same_payload(m, changed)) return 24;
  }
  Command valid;
  if (!final_command_frame::manual_metadata(valid)) return 25;
  for (int field = 0; field < 5; ++field) {
    Command invalid = valid;
    if (field == 0) invalid.manual_linear = 1.1;
    if (field == 1) invalid.manual_linear_limit = -.1;
    if (field == 2) invalid.degraded_linear_limit = -.1;
    if (field == 3) invalid.vx += .01;
    if (field == 4) invalid.degraded_linear_limit = std::nan("");
    if (final_command_frame::manual_metadata(invalid)) return 26;
  }

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
    const auto frame = final_command_frame::encode(mode == "collected" ? 4 : mode == "terminal" ? 3 : 1, gid, m);
    const auto sent = ::send(std::atoi(argv[1]), frame.data(), frame.size(), 0);
    return sent == static_cast<ssize_t>(frame.size()) ? 0 : 3;
  } catch (const std::invalid_argument&) {
    return 42;
  }
}
