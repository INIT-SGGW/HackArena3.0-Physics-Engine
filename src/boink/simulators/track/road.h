#pragma once

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include "boink/simulators/track/ground.h"
#include "boink/simulators/track/line.h"
#include "boink/bounding_box.h"

#include <vector>
#include <memory>

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
      btScalar left_max_width;
      std::vector<std::pair<btScalar,Ground::Type>> left_grounds;

      btScalar right_width;
      btScalar right_max_width;
      std::vector<std::pair<btScalar,Ground::Type>> right_grounds;

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
    Road(
        Line centerline, 
        Line rightline, 
        Line leftline, 
        std::shared_ptr<btDynamicsWorld> world,
        std::string file);

    const std::vector<Metrics>& getRoadData() const {return road_data_;}
    std::vector<Metrics>& getRoadData() {return road_data_;}

    const btVector3 getPoint(size_t index,Side side=Side::Center) const;
    const btVector3 getInterpolatedPoint1(const btVector3& point,Side side=Side::Center) const;
    btVector3 getRandomPosition(Side side=Side::Center) const;

    size_t getSize(Side side) const {return getLine(side).getPointsSize();}
    btScalar getLength(Side side=Side::Center) const
    {return getLine(side).getLength();}

    const Metrics& getMetrics(size_t index) const;
    const Metrics& getClosestMetrics(const btVector3& point) const;

    btScalar getCoverage(const btVector3& point,Side side=Side::Center) const
    {return getLine(side).getCoverage(point);}

    bool isClosed() const {return is_road_closed_;}
    const Line& getLine(Side side) const;

    // returns number of corners which are on the road
    int isObjectOnRoad(
        const btVector3& position,
        const btQuaternion& orientation,
        const btVector3& offset,
        const BoundingBox& box,
        bool max_lines=false) const;
  private:
    void loadMetrics();
    void createRoadData();
    Metrics generateMetrics(size_t i) const;
    void saveMetrics();

    btScalar calculateCurvature(size_t i) const;
    btScalar calculateFirstHitDist(
        const btVector3& from,const btVector3& dir, btScalar max_search=50.f) const;
    std::vector<std::pair<btScalar,Ground::Type>>calculateGroundWidths(
        const btVector3& from,
        const btVector3& to,
        btVector3 down,
        btScalar step=0.1f) const;

    Line& getLine(Side side);
  private:
    Line centerline_;
    Line rightline_;
    Line leftline_;

    std::shared_ptr<btDynamicsWorld> world_;
    std::string file_;

    bool is_road_closed_;
    std::vector<Metrics> road_data_;
  };
}
