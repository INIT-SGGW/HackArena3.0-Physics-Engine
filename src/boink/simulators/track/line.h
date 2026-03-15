#pragma once

#include "boink/gltf_extractor.h"
#include <LinearMath/btScalar.h>
#include <LinearMath/btVector3.h>

#include <optional>
#include <utility>
#include <vector>

namespace boink
{
  class Line
  {
  public:
    Line()=default;

    btScalar getLength() const {return line_length_;}
    bool isClosed() const {return is_line_closed_;}
    const auto& getPointsAndDist() const{return points_dist_;}
    btScalar getCoverage(const btVector3& point) const;

    void reverse();

    std::vector<std::pair<btVector3,btScalar>>::const_iterator 
      getClosest(const btVector3& point) const;

    auto begin() const
    {
      return points_dist_.begin();
    }
    auto end() const
    {
      return points_dist_.end();
    }

    /**
     * @brief Return ith next closest point.
     *
     * @param point to which search for the closest entry
     * @param ith values form 1 to N
     *
     * @return Iterotor to pair which holds the closest point or
     * end iterator if ith is invalid
     */
    std::vector<std::pair<btVector3,btScalar>>::const_iterator 
      getIthClosest(const btVector3& point, size_t ith) const;

    /**
     * @brief 
     *
     * @param point 
     *
     * @return Returns interpolated point and distance cumulated from
     * first_point to interpolated point or if line is open and projected
     * point does not lie on the line returns the closest of ends and its
     * distance.
     */
    std::pair<btVector3,btScalar> getClosestPointInterpolated(
        const btVector3& point) const;

    std::pair<btVector3,btScalar>& getPointAndDist(size_t index)
    {return points_dist_[index];}

    const std::pair<btVector3,btScalar>& getPointAndDist(size_t index) const
    {return points_dist_[index];}

    btVector3& getPoint(size_t index);
    const btVector3& getPoint(size_t index) const;
    size_t getPointsSize() const { return points_dist_.size();}
  private:
    Line(
        std::vector<btVector3> points,
        bool is_line_closed);
  public:
    /**
     * @brief Creates line strip starting with point closest to first_point.
     *
     * @param extractor
     * @param first_point
     * @param name
     *
     * @return 
     */
    static Line createLine(
        const GltfExtractor& extractor,
        std::string_view name,
        std::optional<btVector3> first_point=std::nullopt);

    /**
     * @brief 
     *
     * a and b must be diffrent values
     *
     * @param a 
     * @param b
     * @param point
     *
     * @return Interpolated point on line defined by a and b
     *
     */
    static btVector3 getPointInterpolated(
        const btVector3& a,
        const btVector3& b,
        const btVector3& point);
  private:
    static constexpr btScalar kDesiredDistance=10.f;
  private:
    // Point and distance from first point on curve
    std::vector<std::pair<btVector3,btScalar>> points_dist_;
    btScalar line_length_=0.f;
    bool is_line_closed_=false;
  };
}
