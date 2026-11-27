#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace senior_sensor_ingress {

// One serial consumer owns this state. Lifecycle code alone confirms a new boot.
class SourceOrder {
public:
  void confirm_boot(const std::string & boot)
  {
    if (boot.empty()) {throw std::invalid_argument("source boot must be nonempty");}
    if (boot != boot_) {
      boot_ = boot;
      generation_ = 0;
      latest_stamp_.reset();
    }
  }

  bool observe_generation(const std::string & boot, std::uint64_t generation)
  {
    if (boot.empty() || generation == 0) {
      reason_ = "invalid_source_metadata";
      return false;
    }
    if (boot_.empty()) {confirm_boot(boot);}
    if (boot != boot_) {
      reason_ = "unconfirmed_source_boot";
      return false;
    }
    if (generation < generation_) {
      reason_ = "old_source_generation";
      return false;
    }
    if (generation > generation_) {
      generation_ = generation;
      latest_stamp_.reset();
    }
    reason_ = "";
    return true;
  }

  bool accept(const std::string & boot, std::uint64_t generation, std::int64_t stamp)
  {
    if (stamp < 0) {
      reason_ = "invalid_source_metadata";
      return false;
    }
    if (!observe_generation(boot, generation)) {return false;}
    if (latest_stamp_ && stamp <= *latest_stamp_) {
      reason_ = "non_increasing_capture_stamp";
      return false;
    }
    latest_stamp_ = stamp;
    reason_ = "";
    return true;
  }

  const char * reason() const {return reason_;}
  const std::string & boot_id() const {return boot_;}
  std::uint64_t generation() const {return generation_;}

  // Receiving entry only: the configured single local bridge supplies this
  // immutable first-receipt coordinate. No age limit or clock resampling.
  bool accept_capture(const std::string & boot, std::uint64_t generation,
    std::int64_t stamp, std::uint64_t received)
  {
    if (boot.empty() || generation == 0 || stamp < 0) {
      reason_ = "invalid_source_metadata";
      return false;
    }
    if (!boot_.empty() && boot != boot_) {
      if (last_receipt_ && received <= *last_receipt_) {
        reason_ = "old_source_boot";
        return false;
      }
      confirm_boot(boot);
    }
    if (!accept(boot, generation, stamp)) {return false;}
    if (!last_receipt_ || received > *last_receipt_) {last_receipt_ = received;}
    return true;
  }

private:
  std::string boot_;
  std::uint64_t generation_{0};
  std::optional<std::int64_t> latest_stamp_;
  std::optional<std::uint64_t> last_receipt_;
  const char * reason_{""};
};
}  // namespace senior_sensor_ingress
