#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace senior_scan_matcher::detail {

// A balanced KD-tree stored in median order. The index owns its points so a
// later anchor replacement cannot leave dangling references. Distances use
// hypot exactly as the brute-force residual check; there is no approximation.
class PointIndex {
public:
  using Point = std::pair<double, double>;

  void reset(const std::vector<Point> & points)
  {
    points_.clear();
    points_.reserve(points.size());
    for (const auto & point : points) {
      // NaN distances were ignored by the old std::min loop too. Excluding
      // nonfinite endpoints also gives nth_element a strict ordering.
      if (std::isfinite(point.first) && std::isfinite(point.second)) points_.push_back(point);
    }
    build(0, points_.size(), false);
  }

  void clear() { points_.clear(); }

  double nearest(double x, double y) const
  {
    double best = std::numeric_limits<double>::infinity();
    if (std::isfinite(x) && std::isfinite(y)) search(0, points_.size(), false, x, y, best);
    return best;
  }

private:
  static double coordinate(const Point & point, bool y_axis)
  {
    return y_axis ? point.second : point.first;
  }

  void build(std::size_t begin, std::size_t end, bool y_axis)
  {
    if (begin == end) return;
    const auto middle = begin + (end - begin) / 2;
    std::nth_element(points_.begin() + begin, points_.begin() + middle, points_.begin() + end,
      [y_axis](const Point & a, const Point & b) {
        return coordinate(a, y_axis) < coordinate(b, y_axis);
      });
    build(begin, middle, !y_axis);
    build(middle + 1, end, !y_axis);
  }

  void search(std::size_t begin, std::size_t end, bool y_axis,
    double x, double y, double & best) const
  {
    if (begin == end) return;
    const auto middle = begin + (end - begin) / 2;
    const auto & point = points_[middle];
    best = std::min(best, std::hypot(x - point.first, y - point.second));
    const double split_distance = (y_axis ? y : x) - coordinate(point, y_axis);
    if (split_distance < 0) {
      search(begin, middle, !y_axis, x, y, best);
      if (std::abs(split_distance) <= best) search(middle + 1, end, !y_axis, x, y, best);
    } else {
      search(middle + 1, end, !y_axis, x, y, best);
      if (std::abs(split_distance) <= best) search(begin, middle, !y_axis, x, y, best);
    }
  }

  std::vector<Point> points_;
};

}  // namespace senior_scan_matcher::detail
