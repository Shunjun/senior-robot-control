#include "senior_sensor_ingress/rgbd_candidate_ledger.hpp"

#include <algorithm>
#include <openssl/sha.h>

namespace senior_sensor_ingress {
namespace {
template<typename T>
void append_bounded(std::deque<T> & values, const T & value)
{
  if (values.size() == 4) values.pop_front();
  values.push_back(value);
}
}  // namespace

bool RgbdCandidateLedger::remember_image(size_t stream, const Image & image)
{
  if (poison_exhausted_ || stream >= images_.size() || !image.capture.valid || image.width == 0 || image.height == 0 ||
      image.data.empty() || image.data.size() > 16777216u ||
      uint64_t(image.step) * image.height != image.data.size()) return false;
  if (std::find(poisoned_images_[stream].begin(), poisoned_images_[stream].end(), image.capture) !=
      poisoned_images_[stream].end()) return false;
  Layout layout;
  layout.capture = image.capture;
  layout.width = image.width;
  layout.height = image.height;
  layout.step = image.step;
  layout.encoding = image.encoding;
  layout.is_bigendian = image.is_bigendian;
  layout.byte_length = image.data.size();
  // //Compatible: Humble BoundedVector<uint8_t> does not expose data().
  SHA256(&image.data[0], image.data.size(), layout.sha256.data());
  for (const auto & prior : images_[stream]) {
    if (prior.capture == layout.capture) {
      const bool same = prior == layout;
      if (!same) {
        if (poisoned_images_[stream].size() >= 64) poison_exhausted_ = true;
        else poisoned_images_[stream].push_back(image.capture);
        images_[stream].clear();
      }
      return same;
    }
  }
  append_bounded(images_[stream], layout);
  return true;
}

bool RgbdCandidateLedger::remember_info(size_t stream, const Info & info)
{
  if (poison_exhausted_ || stream >= infos_.size() || !info.capture.valid || info.width == 0 || info.height == 0 ||
      info.d.size() > 16) return false;
  if (std::find(poisoned_infos_[stream].begin(), poisoned_infos_[stream].end(), info.capture) !=
      poisoned_infos_[stream].end()) return false;
  for (const auto & prior : infos_[stream]) {
    if (prior.capture == info.capture) {
      const bool same = prior == info;
      if (!same) {
        if (poisoned_infos_[stream].size() >= 64) poison_exhausted_ = true;
        else poisoned_infos_[stream].push_back(info.capture);
        infos_[stream].clear();
      }
      return same;
    }
  }
  append_bounded(infos_[stream], info);
  return true;
}

bool RgbdCandidateLedger::match_image(size_t stream, const Layout & layout) const
{
  if (stream >= images_.size()) return false;
  return std::find(images_[stream].begin(), images_[stream].end(), layout) != images_[stream].end();
}

bool RgbdCandidateLedger::match_info(size_t stream, const Info & info) const
{
  if (stream >= infos_.size()) return false;
  return std::find(infos_[stream].begin(), infos_[stream].end(), info) != infos_[stream].end();
}

void RgbdCandidateLedger::clear()
{
  for (auto & values : images_) values.clear();
  for (auto & values : infos_) values.clear();
}
}  // namespace senior_sensor_ingress
