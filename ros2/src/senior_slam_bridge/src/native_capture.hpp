#ifndef SENIOR_SLAM_BRIDGE__NATIVE_CAPTURE_HPP_
#define SENIOR_SLAM_BRIDGE__NATIVE_CAPTURE_HPP_

#include <filesystem>
#include <functional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace senior_slam_bridge
{
inline constexpr std::size_t max_graph_file_bytes = 64 * 1024 * 1024;

class CappedArchive : private std::streambuf
{
public:
  explicit CappedArchive(std::size_t limit = max_graph_file_bytes);
  std::ostream & stream() {return stream_;}
  const std::vector<char> & bytes() const {return bytes_;}
  std::vector<char> take_bytes() {return std::move(bytes_);}

private:
  std::streamsize xsputn(const char * data, std::streamsize size) override;
  int_type overflow(int_type character) override;

  const std::size_t limit_;
  std::vector<char> bytes_;
  std::ostream stream_{this};
};

std::vector<char> capture_native_archive(
  const std::function<void(const std::string &)> & save,
  std::size_t limit = max_graph_file_bytes);

struct CaptureArtifact
{
  std::string relative_prefix;
  std::string pair_digest;
};

class CaptureDirectory
{
public:
  explicit CaptureDirectory(const std::filesystem::path & root);
  ~CaptureDirectory();
  CaptureDirectory(const CaptureDirectory &) = delete;
  CaptureDirectory & operator=(const CaptureDirectory &) = delete;

  void require_free_slot() const;
  // Recheck publication authority after the files are synced, while rollback is still possible.
  CaptureArtifact write_pair(
    const std::vector<char> & graph, const std::vector<char> & data,
    const std::function<bool()> & publication_allowed = {});

private:
  int directory_;
};
}  // namespace senior_slam_bridge

#endif  // SENIOR_SLAM_BRIDGE__NATIVE_CAPTURE_HPP_
