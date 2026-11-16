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
  Policy(double soft_ms, double hard_ms, double p) : soft(soft_ms / 1000), hard(hard_ms / 1000), period(p),
    soft_ns(0), hard_ns(0) {
    if (!std::isfinite(soft_ms) || !std::isfinite(hard_ms) || !std::isfinite(p) || soft_ms<0 || hard_ms<=soft_ms || hard_ms>1e9 || p<=0)
      throw std::invalid_argument("shared sensor policy is required");
    soft_ns=static_cast<int64_t>(std::llround(soft_ms*1e6));
    hard_ns=static_cast<int64_t>(std::llround(hard_ms*1e6));
  }
  std::optional<int64_t> try_age_ns(uint64_t measured_ns,
      double received_steady, double now_steady) const noexcept {
    if (!std::isfinite(received_steady) || !std::isfinite(now_steady) ||
        received_steady < 0 || now_steady < received_steady || now_steady > 9e9)
      return std::nullopt;
    const auto received = static_cast<uint64_t>(std::llround(received_steady * 1e9));
    const auto now = static_cast<uint64_t>(std::llround(now_steady * 1e9));
    if (measured_ns > received) return std::nullopt;
    return static_cast<int64_t>(now - measured_ns);
  }
  int64_t age_ns(uint64_t measured_ns, double received_steady, double now_steady) const {
    const auto age = try_age_ns(measured_ns, received_steady, now_steady);
    if (!age) throw std::invalid_argument("invalid monotonic sensor clock");
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
