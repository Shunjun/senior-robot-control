#include "staged_graph.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
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

constexpr std::size_t max_file_bytes = 64 * 1024 * 1024;
using Digest = std::array<unsigned char, 32>;

Digest copy_and_hash(const fs::path & input, const fs::path & output)
{
  const int descriptor = open(input.c_str(), O_RDONLY | O_NOFOLLOW | O_NONBLOCK | O_CLOEXEC);
  if (descriptor < 0) {
    throw std::runtime_error("Could not open staged graph file");
  }
  std::unique_ptr<FILE, decltype(&fclose)> source(fdopen(descriptor, "rb"), &fclose);
  if (!source) {
    close(descriptor);
    throw std::runtime_error("Could not read staged graph file");
  }
  struct stat info {};
  if (fstat(descriptor, &info) != 0 || !S_ISREG(info.st_mode) || info.st_size <= 0 ||
    static_cast<std::uintmax_t>(info.st_size) > max_file_bytes)
  {
    throw std::runtime_error("Staged graph file must be a nonempty bounded regular file");
  }

  std::ofstream copy(output, std::ios::binary);
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> hash(EVP_MD_CTX_new(), &EVP_MD_CTX_free);
  if (!copy || !hash || EVP_DigestInit_ex(hash.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("Could not start private graph copy");
  }

  std::array<char, 64 * 1024> buffer;
  std::size_t total = 0;
  while (const auto size = fread(buffer.data(), 1, buffer.size(), source.get())) {
    total += size;
    if (total > max_file_bytes || EVP_DigestUpdate(hash.get(), buffer.data(), size) != 1 ||
      !copy.write(buffer.data(), static_cast<std::streamsize>(size)))
    {
      throw std::runtime_error("Could not copy staged graph bytes");
    }
  }
  copy.close();
  Digest digest{};
  unsigned size = 0;
  if (ferror(source.get()) || total != static_cast<std::size_t>(info.st_size) || !copy ||
    EVP_DigestFinal_ex(hash.get(), digest.data(), &size) != 1 || size != digest.size())
  {
    throw std::runtime_error("Could not finish private graph copy");
  }
  return digest;
}

std::string pair_digest(const Digest & graph, const Digest & data)
{
  std::array<unsigned char, 64> pair{};
  std::copy(graph.begin(), graph.end(), pair.begin());
  std::copy(data.begin(), data.end(), pair.begin() + graph.size());
  Digest digest{};
  unsigned size = 0;
  if (EVP_Digest(pair.data(), pair.size(), digest.data(), &size, EVP_sha256(), nullptr) != 1 ||
    size != digest.size())
  {
    throw std::runtime_error("Could not hash graph pair");
  }
  constexpr char hex[] = "0123456789abcdef";
  std::string result;
  for (const auto byte : digest) {
    result += hex[byte >> 4];
    result += hex[byte & 0x0f];
  }
  return result;
}

void remove_private_copy(const fs::path & directory) noexcept
{
  std::error_code error;
  fs::remove_all(directory, error);
}
}  // namespace

StagedGraph::StagedGraph(const fs::path & input, const std::string & expected_digest)
{
  if (expected_digest.size() != 64 ||
    expected_digest.find_first_not_of("0123456789abcdef") != std::string::npos)
  {
    throw std::runtime_error("Expected a lowercase SHA-256 pair digest");
  }
  if (!input.is_absolute() || input.native().size() > 1024 || input.filename().empty() ||
    input.filename() == "." || input.filename() == ".." ||
    fs::canonical(input.parent_path()) != input.parent_path())
  {
    throw std::runtime_error("Expected an absolute staged prefix without symlinks");
  }

  auto temp = (input.parent_path() / ".senior-graph-XXXXXX").string();
  if (!mkdtemp(temp.data())) {
    throw std::runtime_error("Could not create private graph directory");
  }
  directory_ = temp;
  try {
    const auto graph = copy_and_hash(input.string() + ".posegraph", prefix().string() + ".posegraph");
    const auto data = copy_and_hash(input.string() + ".data", prefix().string() + ".data");
    digest_ = pair_digest(graph, data);
    if (digest_ != expected_digest) {
      throw std::runtime_error("Staged graph pair digest mismatch");
    }
  } catch (...) {
    remove_private_copy(directory_);
    throw;
  }
}

StagedGraph::~StagedGraph()
{
  remove_private_copy(directory_);
}
}  // namespace senior_slam_bridge
