#include "native_capture.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <unistd.h>

namespace fs = std::filesystem;
using senior_slam_bridge::CaptureDirectory;
using senior_slam_bridge::CappedArchive;

int main(int argc, char ** argv)
{
  if (argc != 2) {
    return 1;
  }
#ifdef __linux__
  const auto save = [](const std::string & path) {
      std::ofstream output(path, std::ios::binary);
      output.exceptions(std::ios::badbit | std::ios::failbit);
      output << "native";
      output.close();
    };
  if (senior_slam_bridge::capture_native_archive(save, 6) !=
    std::vector<char>{'n', 'a', 't', 'i', 'v', 'e'})
  {
    throw std::runtime_error("Unexpected native stream bytes");
  }
  try {
    senior_slam_bridge::capture_native_archive(save, 5);
    throw std::runtime_error("Unexpectedly accepted oversized native stream");
  } catch (const std::ios_base::failure &) {
  }
#endif
  CappedArchive graph(5);
  graph.stream() << "graph";
  CappedArchive data(7);
  data.stream() << "dataset";
  try {
    graph.stream() << "extra";
    throw std::runtime_error("Unexpectedly wrote past the cap");
  } catch (const std::ios_base::failure &) {
  }
  auto temp = (fs::canonical(argv[1]) / "capture-test-XXXXXX").string();
  if (!mkdtemp(temp.data())) {
    throw std::runtime_error("Could not create a capture test root");
  }
  const fs::path root(temp);
  fs::permissions(root, fs::perms::owner_all | fs::perms::group_read);
  try {
    CaptureDirectory invalid(root);
    throw std::runtime_error("Unexpectedly accepted a nonprivate root");
  } catch (const std::invalid_argument &) {
  }
  fs::permissions(root, fs::perms::owner_all);
  CaptureDirectory directory(root);
  directory.require_free_slot();
  bool publication_checked = false;
  try {
    directory.write_pair(graph.bytes(), data.bytes(), [&]() {
      publication_checked = fs::is_regular_file(root / "capture/graph.posegraph") &&
        fs::is_regular_file(root / "capture/graph.data");
      return false;
    });
    throw std::logic_error("Unexpectedly published an unqualified capture");
  } catch (const std::runtime_error &) {
  }
  if (!publication_checked || !fs::is_empty(root)) {
    throw std::runtime_error("Publication rejection did not remove the staged capture pair");
  }
  directory.require_free_slot();
  const auto output = directory.write_pair(graph.bytes(), data.bytes());
  const auto prefix = root / output.relative_prefix;
  std::ifstream graph_file(prefix.string() + ".posegraph");
  std::string contents;
  graph_file >> contents;
  if (contents != "graph" || output.pair_digest != "16fce5d6611d61b39ddc8834a112271930b3cbdbf1313bc9da35e125ff0bbfb5") {
    throw std::runtime_error("Unexpected bytes or digest");
  }
  try {
    directory.require_free_slot();
    throw std::runtime_error("Occupied capture slot was not detected before sampling");
  } catch (const std::invalid_argument &) {
  }
  try {
    directory.write_pair(graph.bytes(), data.bytes());
    throw std::runtime_error("Unexpectedly accepted a second outstanding capture");
  } catch (const std::invalid_argument &) {
  }
  fs::remove_all(prefix.parent_path());
  directory.require_free_slot();
  fs::remove_all(root);
}
