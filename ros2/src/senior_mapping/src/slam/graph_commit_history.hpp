#ifndef SENIOR_SLAM_BRIDGE__GRAPH_COMMIT_HISTORY_HPP_
#define SENIOR_SLAM_BRIDGE__GRAPH_COMMIT_HISTORY_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace senior_slam_bridge
{
enum class CommitLookup {found, pending, gap, invalid};

// Single-owner bounded history. A failed snapshot consumes its commit sequence.
template<typename Commit, std::size_t Capacity>
class GraphCommitHistory
{
public:
  static_assert(Capacity > 0);

  void append(std::uint64_t sequence, std::optional<Commit> commit)
  {
    if (latest_ == std::numeric_limits<std::uint64_t>::max() || sequence != latest_ + 1) {
      throw std::logic_error("Graph commit history is not contiguous");
    }
    entries_[index(sequence)] = std::move(commit);
    latest_ = sequence;
  }

  std::pair<CommitLookup, const Commit *> lookup(std::uint64_t sequence) const
  {
    if (sequence == 0) {
      return {CommitLookup::invalid, nullptr};
    }
    if (sequence > latest_) {
      return {CommitLookup::pending, nullptr};
    }
    if (latest_ - sequence >= Capacity || !entries_[index(sequence)]) {
      return {CommitLookup::gap, nullptr};
    }
    return {CommitLookup::found, &*entries_[index(sequence)]};
  }

private:
  static std::size_t index(std::uint64_t sequence) {return (sequence - 1) % Capacity;}

  std::array<std::optional<Commit>, Capacity> entries_;
  std::uint64_t latest_{0};
};
}  // namespace senior_slam_bridge

#endif  // SENIOR_SLAM_BRIDGE__GRAPH_COMMIT_HISTORY_HPP_
