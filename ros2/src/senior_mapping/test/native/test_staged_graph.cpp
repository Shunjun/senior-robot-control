#include "staged_graph.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace fs = std::filesystem;

int main(int argc, char ** argv)
{
  if (argc != 2) {
    return 1;
  }
  const fs::path input = fs::canonical(argv[1]) / "fixture";
  const std::string graph = input.string() + ".posegraph";
  const std::string data = input.string() + ".data";
  std::ofstream(graph) << "graph";
  std::ofstream(data) << "dataset";
  const std::string digest = "16fce5d6611d61b39ddc8834a112271930b3cbdbf1313bc9da35e125ff0bbfb5";
  senior_slam_bridge::StagedGraph staged(input, digest);
  if (staged.digest() != digest || staged.prefix() == input) {
    throw std::runtime_error("Expected a private, verified copy");
  }
  std::ofstream(graph) << "changed";
  std::ifstream copy(staged.prefix().string() + ".posegraph");
  std::string contents;
  copy >> contents;
  if (contents != "graph") {
    throw std::runtime_error("The parsed copy must retain the verified bytes");
  }

  const auto reject = [&](const std::string & expected) {
      try {
        senior_slam_bridge::StagedGraph invalid(input, expected);
      } catch (const std::exception &) {
        return;
      }
      throw std::runtime_error("Unexpectedly accepted an invalid pair");
    };
  reject(digest);
  std::ofstream(graph) << "graph";
  fs::remove(data);
  reject(digest);
  fs::create_symlink(graph, data);
  reject(digest);
  fs::remove(data);
  std::ofstream(data) << "dataset";
  fs::resize_file(data, 64 * 1024 * 1024 + 1);
  reject(digest);
  fs::remove(data);
  fs::remove(graph);
}
