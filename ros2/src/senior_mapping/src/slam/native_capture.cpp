#include "native_capture.hpp"

#include <array>
#include <cerrno>
#include <exception>
#include <future>
#include <stdexcept>
#include <system_error>

#include <fcntl.h>
#include <openssl/evp.h>
#include <sys/stat.h>
#include <unistd.h>

namespace senior_slam_bridge
{
namespace
{
namespace fs = std::filesystem;

class FileDescriptor
{
public:
  explicit FileDescriptor(int value) : value_(value)
  {
    if (value < 0) {
      throw std::system_error(errno, std::generic_category(), "Open capture artifact");
    }
  }

  ~FileDescriptor()
  {
    if (value_ >= 0) {
      close(value_);
    }
  }

  FileDescriptor(const FileDescriptor &) = delete;
  FileDescriptor & operator=(const FileDescriptor &) = delete;
  int get() const {return value_;}

  int release()
  {
    const int value = value_;
    value_ = -1;
    return value;
  }

private:
  int value_;
};

void require_private(int descriptor)
{
  struct stat status {};
  if (fstat(descriptor, &status) != 0 || !S_ISDIR(status.st_mode) ||
    status.st_uid != geteuid() || (status.st_mode & 07777) != 0700)
  {
    throw std::invalid_argument("Capture root must be owned by this user with mode 0700");
  }
}

void write_file(int directory, const char * name, const std::vector<char> & data)
{
  FileDescriptor file(openat(directory, name, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600));
  std::size_t offset = 0;
  while (offset < data.size()) {
    const auto size = write(file.get(), data.data() + offset, data.size() - offset);
    if (size < 0 && errno == EINTR) {
      continue;
    }
    if (size <= 0) {
      throw std::runtime_error("Could not write complete capture artifact");
    }
    offset += static_cast<std::size_t>(size);
  }
  if (fsync(file.get()) != 0 || close(file.release()) != 0) {
    throw std::runtime_error("Could not finalize capture artifact");
  }
}

std::string digest_pair(const std::vector<char> & graph, const std::vector<char> & data)
{
  std::array<unsigned char, 64> pair{};
  std::array<unsigned char, 32> digest{};
  unsigned size = 0;
  if (EVP_Digest(graph.data(), graph.size(), pair.data(), &size, EVP_sha256(), nullptr) != 1 ||
    size != 32 ||
    EVP_Digest(data.data(), data.size(), pair.data() + 32, &size, EVP_sha256(), nullptr) != 1 ||
    size != 32 ||
    EVP_Digest(pair.data(), pair.size(), digest.data(), &size, EVP_sha256(), nullptr) != 1 ||
    size != digest.size())
  {
    throw std::runtime_error("Could not hash captured graph pair");
  }
  constexpr char hex[] = "0123456789abcdef";
  std::string result;
  for (const auto byte : digest) {
    result += hex[byte >> 4];
    result += hex[byte & 0x0f];
  }
  return result;
}
}  // namespace

CappedArchive::CappedArchive(std::size_t limit) : limit_(limit)
{
  stream_.exceptions(std::ios::badbit | std::ios::failbit);
}

std::streamsize CappedArchive::xsputn(const char * data, std::streamsize size)
{
  if (size < 0 || static_cast<std::size_t>(size) > limit_ - bytes_.size()) {
    return 0;
  }
  bytes_.insert(bytes_.end(), data, data + size);
  return size;
}

CappedArchive::int_type CappedArchive::overflow(int_type character)
{
  if (traits_type::eq_int_type(character, traits_type::eof())) {
    return traits_type::not_eof(character);
  }
  const char byte = traits_type::to_char_type(character);
  return xsputn(&byte, 1) == 1 ? character : traits_type::eof();
}

std::vector<char> capture_native_archive(
  const std::function<void(const std::string &)> & save, std::size_t limit)
{
  int endpoints[2];
  if (pipe(endpoints) != 0) {
    throw std::system_error(errno, std::generic_category(), "Create native capture pipe");
  }
  FileDescriptor read_end(endpoints[0]);
  FileDescriptor write_end(endpoints[1]);
  if (fcntl(read_end.get(), F_SETFD, FD_CLOEXEC) != 0 ||
    fcntl(write_end.get(), F_SETFD, FD_CLOEXEC) != 0)
  {
    throw std::system_error(errno, std::generic_category(), "Set capture pipe close-on-exec");
  }
  auto collected = std::async(std::launch::async, [&]() {
    CappedArchive archive(limit);
    std::array<char, 64 * 1024> block;
    std::exception_ptr overflow;
    while (true) {
      const auto size = read(read_end.get(), block.data(), block.size());
      if (size < 0 && errno == EINTR) {
        continue;
      }
      if (size < 0) {
        throw std::system_error(errno, std::generic_category(), "Read native capture pipe");
      }
      if (size == 0) {
        break;
      }
      // Drain after reaching the cap so the vendor writer can finish without SIGPIPE.
      if (!overflow) {
        try {
          archive.stream().write(block.data(), size);
        } catch (...) {
          overflow = std::current_exception();
        }
      }
    }
    if (overflow) {
      std::rethrow_exception(overflow);
    }
    return archive.take_bytes();
  });

  std::exception_ptr save_error;
  try {
    //Compatible: the pinned vendor APIs accept a Linux procfs path, not an ostream.
    save("/proc/self/fd/" + std::to_string(write_end.get()));
  } catch (...) {
    save_error = std::current_exception();
  }
  close(write_end.release());
  auto result = collected.get();
  if (save_error) {
    std::rethrow_exception(save_error);
  }
  return result;
}

CaptureDirectory::CaptureDirectory(const fs::path & root)
{
  if (!root.is_absolute() || root.native().size() > 1024 || fs::canonical(root) != root) {
    throw std::invalid_argument("Capture root must be an absolute canonical directory");
  }
  FileDescriptor directory(open(root.c_str(), O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC));
  require_private(directory.get());
  if (!fs::is_empty(root)) {
    throw std::invalid_argument("Capture root must initially be empty");
  }
  directory_ = directory.release();
}

CaptureDirectory::~CaptureDirectory()
{
  close(directory_);
}

void CaptureDirectory::require_free_slot() const
{
  require_private(directory_);
  struct stat status {};
  if (fstatat(directory_, "capture", &status, AT_SYMLINK_NOFOLLOW) == 0) {
    throw std::invalid_argument("Capture slot is unavailable");
  }
  if (errno != ENOENT) {
    throw std::system_error(errno, std::generic_category(), "Check capture slot");
  }
}

CaptureArtifact CaptureDirectory::write_pair(
  const std::vector<char> & graph, const std::vector<char> & data,
  const std::function<bool()> & publication_allowed)
{
  if (graph.empty() || data.empty() ||
    graph.size() > max_graph_file_bytes || data.size() > max_graph_file_bytes)
  {
    throw std::invalid_argument("Capture pair must contain two bounded files");
  }
  auto digest = digest_pair(graph, data);
  require_private(directory_);
  if (mkdirat(directory_, "capture", 0700) != 0) {
    throw std::invalid_argument("Capture slot is unavailable");
  }
  try {
    FileDescriptor slot(openat(directory_, "capture", O_RDONLY | O_DIRECTORY | O_NOFOLLOW | O_CLOEXEC));
    try {
      write_file(slot.get(), "graph.posegraph", graph);
      write_file(slot.get(), "graph.data", data);
      if (publication_allowed && !publication_allowed()) {
        throw std::runtime_error("Graph capture owner retired before publication");
      }
    } catch (...) {
      unlinkat(slot.get(), "graph.posegraph", 0);
      unlinkat(slot.get(), "graph.data", 0);
      throw;
    }
  } catch (...) {
    unlinkat(directory_, "capture", AT_REMOVEDIR);
    throw;
  }
  return {"capture/graph", std::move(digest)};
}
}  // namespace senior_slam_bridge
