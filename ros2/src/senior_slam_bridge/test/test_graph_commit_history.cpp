#include "graph_commit_history.hpp"

#include <stdexcept>

namespace
{
using senior_slam_bridge::CommitLookup;
using senior_slam_bridge::GraphCommitHistory;

void require(bool condition)
{
  if (!condition) {
    throw std::runtime_error("Unexpected graph commit history result");
  }
}
}  // namespace

int main()
{
  GraphCommitHistory<int, 2> history;
  require(history.lookup(0).first == CommitLookup::invalid);
  require(history.lookup(1).first == CommitLookup::pending);
  history.append(1, 11);
  require(history.lookup(1).first == CommitLookup::found && *history.lookup(1).second == 11);
  history.append(2, std::nullopt);
  require(history.lookup(2).first == CommitLookup::gap);
  history.append(3, 33);
  require(history.lookup(1).first == CommitLookup::gap);
  require(history.lookup(2).first == CommitLookup::gap);
  require(history.lookup(3).first == CommitLookup::found && *history.lookup(3).second == 33);
  require(history.lookup(4).first == CommitLookup::pending);
  try {
    history.append(5, 55);
    throw std::runtime_error("Unexpectedly accepted a skipped graph commit");
  } catch (const std::logic_error &) {
  }
  require(history.lookup(4).first == CommitLookup::pending);
  history.append(4, 44);
  require(history.lookup(3).first == CommitLookup::found && *history.lookup(3).second == 33);
  require(history.lookup(4).first == CommitLookup::found && *history.lookup(4).second == 44);
}
