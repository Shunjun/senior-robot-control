#ifndef SENIOR_SLAM_BRIDGE__STAGED_GRAPH_HPP_
#define SENIOR_SLAM_BRIDGE__STAGED_GRAPH_HPP_

#include <filesystem>
#include <string>

namespace senior_slam_bridge
{
class StagedGraph
{
public:
  StagedGraph(const std::filesystem::path & input, const std::string & expected_digest);
  ~StagedGraph();
  StagedGraph(const StagedGraph &) = delete;
  StagedGraph & operator=(const StagedGraph &) = delete;

  std::filesystem::path prefix() const {return directory_ / "graph";}
  const std::string & digest() const {return digest_;}

private:
  std::filesystem::path directory_;
  std::string digest_;
};
}  // namespace senior_slam_bridge

#endif  // SENIOR_SLAM_BRIDGE__STAGED_GRAPH_HPP_
