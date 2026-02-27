#include "boink/simulators/track/line.h"

#include <algorithm>
#include <cassert>
#include <unordered_set>

#include "boink/exception.h"

namespace boink {

Line::Line(const std::vector<btVector3>& points) : points_dist_(points.size()) {
  if (points.size() < 2)
    throw Exception(Exception::Type::InvalidArgumentError,
                    "The vector of given points is lesser than 2");

  assert(points_dist_.size() == points.size());
  std::transform(points.cbegin(), points.cend(), points_dist_.begin(),
                 [](const btVector3& v) {
                   return std::pair<btVector3, btScalar>(v, 0.0f);
                 });

  // Order the points
  // We assume that first given point is the
  // starting point. fuck for now direciton
  std::vector<size_t> sorted_indices;
  std::vector<size_t> unused_indices(points_dist_.size());

  std::generate(unused_indices.begin(), unused_indices.end(),
                [n = 0]() mutable { return n++; });

  // Assume first element of points is the first in order
  sorted_indices.push_back(0);
  unused_indices.erase(
      std::find(unused_indices.begin(), unused_indices.end(), 0));

  // Very inefficient
  while (unused_indices.size() > 0) {
    size_t prev_i = sorted_indices[sorted_indices.size() - 1];
    size_t curr_i = this->getIthClosestIndex(this->getPoint(prev_i), 2).first;
    size_t ith = 3;
    while (std::find(unused_indices.begin(), unused_indices.end(), curr_i) ==
           unused_indices.end()) {
      curr_i = this->getIthClosestIndex(this->getPoint(prev_i), ith).first;
      ith++;
    }

    sorted_indices.push_back(curr_i);
    unused_indices.erase(
        std::find(unused_indices.begin(), unused_indices.end(), curr_i));
  }

  assert(sorted_indices.size() == points_dist_.size());
  assert(sorted_indices.size() == points.size());

  std::unordered_set<size_t> seen;
  btScalar length = 0.0;
  size_t prev_index = sorted_indices[0];
  for (size_t i = 0; i < sorted_indices.size(); i++) {
    size_t index = sorted_indices[i];
    if (seen.insert(index).second) {
      length += (points[index] - points[prev_index]).length();

      assert(length > 1e-6 || i == 0);
      points_dist_[i] = {points[index], length};
      prev_index = index;
    }
  }

  // Delete duplicated entries in point_dist_
  // they are sorted so we look for entries that are next to each other
  for (auto it = points_dist_.begin(); it + 1 != points_dist_.end();) {
    auto it2 = it + 1;
    if (btFabs(it->second - it2->second) < 1e-8)
      it = points_dist_.erase(it);
    else
      it++;
  }
}

btScalar Line::getLength() const {
  return points_dist_[points_dist_.size() - 1].second;
}

btScalar Line::getCoverage(const btVector3& point) const {
  size_t closest_i = this->getClosestIndex(point).first;
  size_t sec_closest_i = this->getIthClosestIndex(point, 2).first;

  size_t first_i;
  size_t second_i;
  if (points_dist_[closest_i].second < points_dist_[sec_closest_i].second) {
    first_i = closest_i;
    second_i = sec_closest_i;
  } else {
    second_i = closest_i;
    first_i = sec_closest_i;
  }

  // if are the first and last points
  if (first_i == 0 && second_i == this->getPointsSize() - 1) {
    first_i = second_i;
    second_i = 0;
  }
  assert(first_i != second_i);

  btVector3 dir = points_dist_[second_i].first - points_dist_[first_i].first;
  btScalar len = dir.length();
  if (len < 0.0001)
    dir = {0, 0, 0};
  else
    dir.normalize();
  btVector3 our_vec = point - points_dist_[first_i].first;

  // assert(our_vec.dot(dir)>0);
  return our_vec.dot(dir) + points_dist_[first_i].second;
}

std::pair<size_t, btScalar> Line::getClosestIndex(
    const btVector3& point) const {
  size_t closest_i = 0;
  btScalar closest_dist2 = (points_dist_[closest_i].first - point).length2();
  for (size_t i = 1; i < points_dist_.size(); i++) {
    btScalar curr_len2 = (points_dist_[i].first - point).length2();
    if (curr_len2 < closest_dist2) {
      closest_dist2 = curr_len2;
      closest_i = i;
    }
  }

  return {closest_i, btSqrt(closest_dist2)};
}

std::pair<size_t, btScalar> Line::getIthClosestIndex(const btVector3& point,
                                                     size_t ith) const {
  assert(points_dist_.size() > ith);

  std::vector<size_t> closest_is;
  btScalar ith_closest_dist2;
  size_t ith_closest_i;
  for (size_t i = 0; i < ith; i++) {
    ith_closest_i = 0;
    while (std::find(closest_is.begin(), closest_is.end(), ith_closest_i) !=
           closest_is.end()) {
      ith_closest_i++;
      assert(ith_closest_i < points_dist_.size());
    }
    ith_closest_dist2 = (this->getPoint(ith_closest_i) - point).length2();

    for (size_t j = 0; j < points_dist_.size(); j++) {
      if (std::find(closest_is.begin(), closest_is.end(), j) !=
          closest_is.end()) {
        continue;
      }

      btScalar curr_len2 = (this->getPoint(j) - point).length2();
      if (curr_len2 < ith_closest_dist2) {
        ith_closest_dist2 = curr_len2;
        ith_closest_i = j;
      }
    }
    closest_is.push_back(ith_closest_i);
  }

  return {ith_closest_i, btSqrt(ith_closest_dist2)};
}

btVector3& Line::getPoint(size_t index) { return points_dist_[index].first; }

const btVector3& Line::getPoint(size_t index) const {
  return points_dist_.at(index).first;
}

}  // namespace boink
