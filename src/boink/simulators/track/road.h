#pragma once

#include "boink/simulators/track/line.h"

#include <vector>

namespace boink
{
  class Road
  {
  public:
    struct Metrics
    {
      btVector3 tangent;
      btVector3 normal;
      btVector3 right;

      btScalar left_width;
      btScalar right_width;

      btScalar curvature;
      // minus downhill postivie uphill -90 to +90
      btScalar grade;
      btScalar bank;
    };

    enum class Side
    {
      Center,
      Right,
      Left
    };
  public:
    Road()=default;
    Road(Line centerline, Line rightline, Line leftline);

    const std::vector<Metrics>& getRoadData() const {return road_data_;}
    std::vector<Metrics>& getRoadData() {return road_data_;}

    const btVector3 getPoint(size_t index,Side side=Side::Center) const;
    const btVector3 getInterpolatedPoint1(const btVector3& point,Side side=Side::Center) const;
    btVector3 getRandomPosition(Side side=Side::Center) const;

    size_t getSize() const {return road_data_.size();}
    btScalar getLength(Side side=Side::Center) const
    {return getLine(side).getLength();}

    const Metrics& getMetrics(size_t index) const;
    const Metrics& getClosestMetrics(const btVector3& point) const;

    btScalar getCoverage(const btVector3& point,Side side=Side::Center) const
    {return getLine(side).getCoverage(point);}

    bool isClosed() const {return is_road_closed_;}
    const Line& getLine(Side side) const;
  private:
    void createRoadData();
    Metrics generateMetrics(size_t i) const;
    btScalar calculateCurvature(size_t i) const;

    Line& getLine(Side side);
  private:
    Line centerline_;
    Line rightline_;
    Line leftline_;

    bool is_road_closed_;
    std::vector<Metrics> road_data_;
  };
}
