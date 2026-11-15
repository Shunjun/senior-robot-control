#pragma once
// Values are supplied by the shared installed sensor-policy JSON through launch.
// This header owns no separate deployment-specific age constants.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <limits>
#include <optional>

namespace sensor_timing {
struct Policy {
  double soft, hard, period;
  int64_t soft_ns, hard_ns;
  Policy(double s, double h, double p) : soft(s), hard(h), period(p),
    soft_ns(0), hard_ns(0) {
    if (!std::isfinite(s) || !std::isfinite(h) || !std::isfinite(p) || s<0 || h<=s || h>1e6 || p<=0)
      throw std::invalid_argument("shared sensor policy is required");
    soft_ns=static_cast<int64_t>(std::llround(s*1e9));
    hard_ns=static_cast<int64_t>(std::llround(h*1e9));
  }
  std::optional<int64_t> try_age_ns(int64_t stamp, int64_t received_ros,
      double received_steady, int64_t now_ros, double now_steady) const noexcept {
    if (stamp<0 || received_ros<stamp || now_ros<received_ros ||
        !std::isfinite(received_steady) || !std::isfinite(now_steady) || now_steady<received_steady)
      return std::nullopt;
    const double elapsed_seconds=now_steady-received_steady;
    if (!std::isfinite(elapsed_seconds) || elapsed_seconds>9e9)
      return std::nullopt;
    const auto elapsed = static_cast<int64_t>(std::llround(elapsed_seconds*1e9));
    const auto initial=received_ros-stamp;
    const auto combined=elapsed>std::numeric_limits<int64_t>::max()-initial ?
      std::numeric_limits<int64_t>::max() : initial+elapsed;
    return std::max(now_ros-stamp,combined);
  }
  //Compatible: explicit programmer callers retain the throwing API. Runtime
  // input boundaries use try_age_ns; unrelated program failures are not caught.
  int64_t age_ns(int64_t stamp, int64_t received_ros, double received_steady,
      int64_t now_ros, double now_steady) const {
    const auto age = try_age_ns(stamp, received_ros, received_steady, now_ros, now_steady);
    if (!age) throw std::invalid_argument("invalid or out-of-range sensor clock");
    return *age;
  }
};
struct AgeSummary {
  uint64_t count=0, over_soft=0, over_hard=0, invalid_clock=0;
  double sum=0, maximum=0;
  void reject_invalid_clock() noexcept {
    if (invalid_clock < std::numeric_limits<uint64_t>::max()) ++invalid_clock;
  }
  void observe(int64_t ns, const Policy &p) {
    ++count; const double age=static_cast<double>(ns)/1e9;
    sum+=age; maximum=std::max(maximum,age);
    over_soft+=ns>=p.soft_ns; over_hard+=ns>=p.hard_ns;
  }
};
}  // namespace sensor_timing

namespace sensor_timing {
struct QueuePolicy {
  int count;
  double ratio, repeat, recovery, wait_ms;
  QueuePolicy(int c, double r, double p, double h, double w)
  : count(c), ratio(r), repeat(p), recovery(h), wait_ms(w) {
    if (c<1 || !std::isfinite(r) || r<=0 || r>1 || !std::isfinite(p) || p<=0 ||
        !std::isfinite(h) || h<0 || h>=1 || !std::isfinite(w) || w<=0)
      throw std::invalid_argument("invalid shared queue warning policy");
  }
  size_t threshold(size_t capacity) const {
    return std::max<size_t>(1,std::min<size_t>(count,std::floor(ratio*capacity)));
  }
};
struct QueueMonitor {
  bool over=false, announced=false, crossing=false;
  size_t peak=0, warning_pending=0;
  double warning_wait=0., last=-1e30;
  void note(size_t pending, size_t capacity, double wait_ms, double, const QueuePolicy &p) {
    const auto threshold=p.threshold(capacity);
    if (pending>threshold || wait_ms>p.wait_ms) {
      if (!over) crossing=true;
      over=true; peak=std::max(peak,pending);
      warning_pending=pending; warning_wait=wait_ms;
    } else if (pending<=std::floor(threshold*p.recovery)) over=false;
  }
  // 1 warning, -1 recovery, 0 no log. This never grants/revokes control.
  int observe(size_t pending, size_t capacity, double wait_ms, double now, const QueuePolicy &p) {
    note(pending,capacity,wait_ms,now,p);
    if (over) {warning_pending=pending; warning_wait=wait_ms;}
    if (now-last<p.repeat) return 0;
    if (over || crossing) {last=now; announced=true; crossing=false; return 1;}
    if (announced) {last=now; announced=false; peak=0; return -1;}
    return 0;
  }
};
}  // namespace sensor_timing
