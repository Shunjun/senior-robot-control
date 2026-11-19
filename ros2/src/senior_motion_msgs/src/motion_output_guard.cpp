// Sole actuator transport for the in-memory control owner. No estop state lives here.
#include <algorithm>
#include <array>
#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <ctime>
#include <fcntl.h>
#include <sys/stat.h>
#include <signal.h>
#include <pthread.h>
#include <memory>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "senior_motion_msgs/sample_time.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MotionOutput : public rclcpp::Node {
 public:
  MotionOutput(int fd, const std::string &topic) : Node("motion_output"), fd_(fd) {
    init_trace();
    publisher_ = create_publisher<geometry_msgs::msg::Twist>(topic, 1);
    timer_ = create_wall_timer(5ms, [this]() { poll(); });
  }
  ~MotionOutput() override {
    if (rclcpp::ok()) stop("destructor");
    flush_trace();
    finish_trace("closed");
    ::close(fd_);
  }
 private:
  // Private same-host packet: vx, wz, original deadline, calculation entry, sequence.
  static constexpr size_t packet_size = 4 * sizeof(double) + sizeof(uint64_t);
  int fd_;
  uint64_t sequence_ = 0, last_produced_ = 0;
  double deadline_ = 0;
  bool moving_ = false, disconnected_ = false;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  static uint64_t steady_ns() { return senior_motion_msgs::time::monotonic_ns(); }
  // Finite diagnostic data only. Neither this buffer nor its fd carries actuator packets.
  struct TraceRow {
    const char *event, *reason;
    uint64_t sequence, produced, deadline, checked, before, after, observed, thread, process;
    double vx, wz;
  };
  int trace_fd_ = -1;
  uint64_t trace_start_ = 0, trace_end_ = 0, trace_lost_ = 0, trace_rows_ = 0;
  size_t trace_used_ = 0, trace_size_ = 0;
  std::string trace_id_;
  std::array<TraceRow, 192> trace_buffer_{};
  uint64_t active_produced_ = 0, active_sequence_ = 0;
  static uint64_t cpu_ns(clockid_t clock) {
    timespec value{};
    if (::clock_gettime(clock, &value) != 0) return 0;
    return static_cast<uint64_t>(value.tv_sec) * 1000000000ULL + value.tv_nsec;
  }
  void init_trace() {
    const char *id = std::getenv("SENIOR_MOTION_TRACE_ID");
    const char *fd = std::getenv("SENIOR_MOTION_TRACE_FD");
    const char *start = std::getenv("SENIOR_MOTION_TRACE_START_NS");
    const char *end = std::getenv("SENIOR_MOTION_TRACE_END_NS");
    if (!id || !fd || !start || !end || !*id || std::strlen(id) > 64) return;
    for (const char *c = id; *c; ++c)
      if (!( (*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') ||
             (*c >= '0' && *c <= '9') || *c == '-' || *c == '_')) return;
    char *tail = nullptr;
    const auto descriptor = std::strtol(fd, &tail, 10);
    if (!tail || *tail || descriptor < 0 || descriptor > 1000000) return;
    trace_start_ = std::strtoull(start, &tail, 10);
    if (!tail || *tail) return;
    trace_end_ = std::strtoull(end, &tail, 10);
    if (!tail || *tail || trace_end_ <= trace_start_ ||
        trace_end_ - trace_start_ > 30000000000ULL) return;
    struct stat info{};
    const int flags = ::fcntl(static_cast<int>(descriptor), F_GETFL);
    if (::fstat(static_cast<int>(descriptor), &info) != 0 || !S_ISFIFO(info.st_mode) ||
        flags < 0 || !(flags & O_NONBLOCK)) return;
    const long size = ::fpathconf(static_cast<int>(descriptor), _PC_PIPE_BUF);
    if (size <= 0) return;
    trace_fd_ = static_cast<int>(descriptor);
    trace_id_ = id;
    trace_size_ = std::min(static_cast<size_t>(size), size_t{4096});
  }
  bool tracing() const {
    if (trace_fd_ < 0) return false;
    const auto now = steady_ns();
    return now >= trace_start_ && now < trace_end_;
  }
  void trace(const char *event, const char *reason, uint64_t sequence, uint64_t produced,
             uint64_t deadline, uint64_t checked, double vx, double wz, uint64_t before) {
    if (!tracing()) return;
    if (trace_used_ == trace_buffer_.size()) { ++trace_lost_; return; }
    trace_buffer_[trace_used_++] = TraceRow{event, reason, sequence, produced, deadline,
      checked, before, last_produced_, steady_ns(), cpu_ns(CLOCK_THREAD_CPUTIME_ID),
      cpu_ns(CLOCK_PROCESS_CPUTIME_ID), vx, wz};
  }
  ssize_t write_trace(const void *bytes, size_t size) {
    // Popen restores SIGPIPE to default. A closed diagnostic reader must not kill
    // the actuator owner; block/consume only our newly generated thread signal.
    sigset_t blocked{}, previous{}, pending{};
    sigemptyset(&blocked); sigaddset(&blocked, SIGPIPE);
    if (::pthread_sigmask(SIG_BLOCK, &blocked, &previous) != 0) return -1;
    if (::sigpending(&pending) != 0) {
      ::pthread_sigmask(SIG_SETMASK, &previous, nullptr);
      return -1;
    }
    const bool already_pending = sigismember(&pending, SIGPIPE) == 1;
    const auto written = ::write(trace_fd_, bytes, size);
    const auto saved_errno = errno;
    if (written < 0 && saved_errno == EPIPE && !already_pending &&
        ::sigpending(&pending) == 0 && sigismember(&pending, SIGPIPE) == 1) {
      int caught = 0;
      ::sigwait(&blocked, &caught);  // Already pending on this thread; never waits for a reader.
    }
    ::pthread_sigmask(SIG_SETMASK, &previous, nullptr);
    errno = saved_errno;
    return written;
  }
  void finish_trace(const char *reason) {
    if (trace_fd_ < 0) return;
    std::array<char, 256> bytes{};
    const int size = std::snprintf(bytes.data(), bytes.size(),
      "{\"trace_id\":\"%s\",\"pid\":%d,\"event\":\"trace_end\",\"reason\":\"%s\","
      "\"rows\":%llu,\"lost\":%llu}\n", trace_id_.c_str(), static_cast<int>(::getpid()), reason,
      static_cast<unsigned long long>(trace_rows_), static_cast<unsigned long long>(trace_lost_));
    if (size > 0 && static_cast<size_t>(size) < bytes.size() &&
        static_cast<size_t>(size) <= trace_size_)
      write_trace(bytes.data(), static_cast<size_t>(size));
    ::close(trace_fd_); trace_fd_ = -1;
  }
  void flush_trace() {
    if (trace_fd_ < 0) return;
    for (size_t index = 0; index < trace_used_; ++index) {
      if (trace_rows_ >= 4095) { ++trace_lost_; continue; }
      const auto &r = trace_buffer_[index];
      std::array<char, 4096> bytes{};
      const int size = std::snprintf(bytes.data(), bytes.size(),
        "{\"trace_id\":\"%s\",\"pid\":%d,\"event\":\"%s\",\"reason\":\"%s\","
        "\"sequence\":%llu,\"computed_ns\":%llu,\"deadline_ns\":%llu,\"checked_ns\":%llu,"
        "\"last_produced_before\":%llu,\"last_produced_after\":%llu,\"observed_ns\":%llu,"
        "\"thread_ns\":%llu,\"process_ns\":%llu,\"vx\":%.17g,\"wz\":%.17g,"
        "\"since_computed_ms\":%.9g,\"remaining_ms\":%.9g,\"lost\":%llu}\n",
        trace_id_.c_str(), static_cast<int>(::getpid()), r.event, r.reason,
        static_cast<unsigned long long>(r.sequence), static_cast<unsigned long long>(r.produced),
        static_cast<unsigned long long>(r.deadline), static_cast<unsigned long long>(r.checked),
        static_cast<unsigned long long>(r.before), static_cast<unsigned long long>(r.after),
        static_cast<unsigned long long>(r.observed), static_cast<unsigned long long>(r.thread),
        static_cast<unsigned long long>(r.process), r.vx, r.wz,
        (static_cast<double>(r.checked) - static_cast<double>(r.produced)) / 1e6,
        (static_cast<double>(r.deadline) - static_cast<double>(r.checked)) / 1e6,
        static_cast<unsigned long long>(trace_lost_));
      if (size <= 0 || static_cast<size_t>(size) > trace_size_ ||
          static_cast<size_t>(size) >= bytes.size()) { ++trace_lost_; continue; }
      // Nonblocking atomic FIFO write; never fall back to a blocking ROS logger.
      if (write_trace(bytes.data(), static_cast<size_t>(size)) != size) ++trace_lost_;
      ++trace_rows_;
    }
    trace_used_ = 0;
    if (steady_ns() >= trace_end_) finish_trace("window_ended");
  }
  void publish(double vx, double wz, uint64_t produced, uint64_t deadline,
               uint64_t packet_sequence = 0, const char *reason = "frame") {
    const auto before = last_produced_;
    if (produced < last_produced_) {
      trace("drop", "stale_publish", packet_sequence, produced, deadline, steady_ns(), vx, wz, before);
      return;
    }
    TraceRow entry{};
    const bool measured = tracing();
    if (measured) {
      entry = TraceRow{"publish_entry", reason, packet_sequence, produced, deadline,
        steady_ns(), before, before, steady_ns(), cpu_ns(CLOCK_THREAD_CPUTIME_ID),
        cpu_ns(CLOCK_PROCESS_CPUTIME_ID), vx, wz};
    }
    geometry_msgs::msg::Twist value;
    value.linear.x = vx;
    value.angular.z = wz;
    // Preserve the original final admission clock read and stopping barrier.
    uint64_t checked = 0;
    if (vx != 0 || wz != 0) {
      checked = steady_ns();
      if (checked >= deadline) {
        trace("drop", "expired_publish", packet_sequence, produced, deadline, checked, vx, wz, before);
        stop("expired_publish", produced, deadline, packet_sequence);
        return;
      }
    }
    publisher_->publish(value);
    last_produced_ = produced;
    moving_ = vx != 0 || wz != 0;
    if (measured) {
      if (trace_used_ < trace_buffer_.size()) trace_buffer_[trace_used_++] = entry;
      else ++trace_lost_;
    }
    if (packet_sequence) { active_produced_ = produced; active_sequence_ = packet_sequence; }
    trace("publish", reason, packet_sequence, produced, deadline,
          checked ? checked : steady_ns(), vx, wz, before);
  }
  void stop(const char *reason = "stop", uint64_t produced = 0,
            uint64_t deadline = 0, uint64_t packet_sequence = 0) {
    const auto before = last_produced_;
    const auto stopped = steady_ns();
    publish(0, 0, stopped, 0, 0, reason);
    deadline_ = 0;
    trace("stop", reason, packet_sequence, produced, deadline, stopped, 0, 0, before);
  }
  void disconnect(const char *reason) {
    disconnected_ = true;
    stop(reason, active_produced_, static_cast<uint64_t>(std::llround(deadline_ * 1e9)), active_sequence_);
    const char stopped = 1;
    ::send(fd_, &stopped, 1, MSG_DONTWAIT | MSG_NOSIGNAL);
  }
  void poll() {
    poll_input();
    flush_trace();  // All serialization/write work follows the actuator decision.
  }
  void poll_input() {
    if (disconnected_) { rclcpp::shutdown(); return; }
    bool received = false;
    double vx = 0, wz = 0, candidate_deadline = 0;
    uint64_t produced_ns = 0, deadline_ns = 0, candidate_sequence = 0;
    for (size_t count = 0; count < 64; ++count) {
      std::array<unsigned char, packet_size + 1> bytes{};
      const auto size = ::recv(fd_, bytes.data(), bytes.size(), MSG_DONTWAIT);
      if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      if (size != static_cast<ssize_t>(packet_size)) {
        disconnect(size == 0 ? "eof" : "invalid_size"); return;
      }
      double next_vx, next_wz, deadline, produced;
      uint64_t sequence;
      std::memcpy(&next_vx, bytes.data(), sizeof(double));
      std::memcpy(&next_wz, bytes.data() + sizeof(double), sizeof(double));
      std::memcpy(&deadline, bytes.data() + 2 * sizeof(double), sizeof(double));
      std::memcpy(&produced, bytes.data() + 3 * sizeof(double), sizeof(double));
      std::memcpy(&sequence, bytes.data() + 4 * sizeof(double), sizeof(uint64_t));
      if (!std::isfinite(next_vx) || !std::isfinite(next_wz) || !std::isfinite(deadline) ||
          !std::isfinite(produced) || produced <= 0 || produced > 9e9 || deadline < 0 ||
          deadline > 9e9 || sequence <= sequence_) { disconnect("invalid_frame"); return; }
      sequence_ = sequence;
      const auto original_ns = static_cast<uint64_t>(std::llround(produced * 1e9));
      const auto expires_ns = static_cast<uint64_t>(std::llround(deadline * 1e9));
      const auto now = steady_ns();
      if (original_ns > now) { disconnect("future_produced"); return; }
      if (count == 63) { disconnect("drain_limit"); return; }
      const char *drop = original_ns < last_produced_ ? "stale_produced" :
        (received && original_ns < produced_ns) ? "stale_drain_candidate" :
        ((next_vx != 0 || next_wz != 0) && now >= expires_ns) ? "expired_recv" : nullptr;
      if (drop) {
        trace("drop", drop, sequence, original_ns, expires_ns, now, next_vx, next_wz, last_produced_);
        continue;
      }
      if (received)
        trace("drop", "superseded_in_drain", candidate_sequence, produced_ns, deadline_ns,
              now, vx, wz, last_produced_);
      trace("recv", "candidate", sequence, original_ns, expires_ns, now, next_vx, next_wz, last_produced_);
      candidate_sequence = sequence;
      vx = next_vx; wz = next_wz; produced_ns = original_ns; deadline_ns = expires_ns;
      candidate_deadline = deadline;
      received = true;
    }
    if (received) {
      // Recheck after draining the private queue without renewing production time.
      if (vx != 0 || wz != 0) {
        const auto checked = steady_ns();
        if (checked >= deadline_ns) {
          trace("drop", "expired_drain", candidate_sequence, produced_ns, deadline_ns,
                checked, vx, wz, last_produced_);
          if (moving_ && static_cast<double>(steady_ns()) / 1e9 >= deadline_)
            stop("expired_drain", produced_ns, deadline_ns, candidate_sequence);
          return;
        }
      }
      deadline_ = candidate_deadline;
      publish(vx, wz, produced_ns, deadline_ns, candidate_sequence,
              (vx == 0 && wz == 0) ? "explicit_zero" : "frame");
    } else if (moving_) {
      const auto checked = steady_ns();
      if (static_cast<double>(checked) / 1e9 >= deadline_)
        stop("watchdog_timeout", active_produced_,
             static_cast<uint64_t>(std::llround(deadline_ * 1e9)), active_sequence_);
    }
  }
};

int main(int argc, char **argv) {
  if (argc != 3 || !argv[2][0]) return 2;
  const int fd = std::atoi(argv[1]);
  if (fd < 0) return 2;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionOutput>(fd, argv[2]);
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
