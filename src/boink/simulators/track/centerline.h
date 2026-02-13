#pragma once

#include <LinearMath/btScalar.h>
#include <LinearMath/btVector3.h>

#include <vector>

namespace boink
{
  class Centerline
  {
  public:
    Centerline()=default;
    Centerline(
        const std::vector<btVector3>& points);

    btScalar getLength() const;
    btScalar getCoverage(const btVector3& point) const;
    const auto& getPointsAndDist() const{return points_dist_;}
  private:
    std::pair<size_t,btScalar> getClosestIndex(const btVector3& point) const;
    std::pair<size_t,btScalar> getSecondClosestIndex(const btVector3& point) const;
    std::pair<size_t,btScalar> getIthClosestIndex(
        const btVector3& point, size_t ith) const;

    btVector3& getPoint(size_t index);
    const btVector3& getPoint(size_t index) const;
    size_t getPointsSize() const { return points_dist_.size();}
  private:
    // Point and distance from first point on curve
    std::vector<std::pair<btVector3,btScalar>> points_dist_;
  };
}
