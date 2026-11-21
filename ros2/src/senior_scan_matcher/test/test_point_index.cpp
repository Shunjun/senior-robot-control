// Dependency-free equivalence tests; checks remain active in Release builds.
#include "point_index.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

using senior_scan_matcher::detail::PointIndex;
using Point = PointIndex::Point;

namespace {
void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

double brute_force(const std::vector<Point> & points, const Point & query)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & p : points)
    best = std::min(best, std::hypot(query.first - p.first, query.second - p.second));
  return best;
}

void radar_cloud_matches_brute_force()
{
  std::mt19937 generator(20261121);
  std::uniform_real_distribution<double> radius(0.08, 8.0);
  std::vector<Point> points, queries;
  constexpr double pi = 3.14159265358979323846;
  for (unsigned i = 0; i < 2048; ++i) {
    const double angle = -pi + i * 2 * pi / 2048;
    const double distance = radius(generator);
    const Point point{distance * std::cos(angle), distance * std::sin(angle)};
    points.push_back(point);
    queries.emplace_back(0.06 + 0.998 * point.first - 0.06 * point.second,
      -0.03 + 0.06 * point.first + 0.998 * point.second);
  }
  // Three queries straddling the existing 2 * max_residual cutoff. Their
  // far-right location makes the isolated endpoint the closest reference.
  constexpr double cutoff = 0.18;
  points.emplace_back(10.0, 0.0);
  const double edge = 10.0 + cutoff;
  queries.emplace_back(std::nextafter(edge, 10.0), 0.0);
  queries.emplace_back(edge, 0.0);
  queries.emplace_back(std::nextafter(edge, 11.0), 0.0);
  PointIndex index;
  index.reset(points);
  double old_squared = 0, new_squared = 0;
  unsigned old_inliers = 0, new_inliers = 0;
  for (const auto & query : queries) {
    const double expected = brute_force(points, query);
    const double actual = index.nearest(query.first, query.second);
    require(actual == expected, "nearest distance changed");
    if (expected <= cutoff) {old_squared += expected * expected; ++old_inliers;}
    if (actual <= cutoff) {new_squared += actual * actual; ++new_inliers;}
  }
  require(old_inliers == new_inliers && old_squared == new_squared,
    "residual or inlier qualification changed");
}

void collinear_duplicates_match_brute_force()
{
  std::vector<Point> points;
  for (int i = -30; i <= 30; ++i) {
    points.emplace_back(0.0, i * 0.1);
    points.emplace_back(0.0, i * 0.1);
  }
  PointIndex index;
  index.reset(points);
  // Equal split coordinates and equal-distance alternatives must not prune
  // away a nearer point on the other side of a median.
  for (int i = -40; i <= 40; ++i) {
    const Point query{0.18, i * 0.075};
    require(index.nearest(query.first, query.second) == brute_force(points, query),
      "collinear/duplicate nearest distance changed");
  }
}

void reset_replaces_reference()
{
  PointIndex index;
  index.reset({{0.0, 0.0}, {0.1, 0.0}});
  require(index.nearest(0, 0) == 0, "initial index missing");
  index.reset({{3.0, 4.0}});
  require(index.nearest(0, 0) == 5.0, "old anchor survived replacement");
  index.clear();
  require(std::isinf(index.nearest(0, 0)), "old anchor survived clear");
  index.reset({});
  require(std::isinf(index.nearest(0, 0)), "empty reference has a nearest point");
}
}  // namespace

int main()
{
  try {
    radar_cloud_matches_brute_force();
    collinear_duplicates_match_brute_force();
    reset_replaces_reference();
    std::cout << "PASS: 3 point-index equivalence scenarios\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
