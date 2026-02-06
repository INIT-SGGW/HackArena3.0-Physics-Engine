#pragma once

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include <memory>
#include <string_view>
#include <vector>
#include <utility>

namespace boink
{
  class Track
  {
  public:
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
  public:
    Track(std::string_view filename,
      std::shared_ptr<btDiscreteDynamicsWorld> world);
    Track(const Track&)=delete;
    Track(Track&&)=default;

    Track& operator=(const Track&)=delete;
    Track& operator=(Track&&)=default;

    ~Track() noexcept;

    const btVector3& getPosition() const;
    void setPosition(const btVector3& position);

    const Centerline& getCenterline() const {return centerline_;}
  private:
    static constexpr std::string_view TRACK_NAME="Sideroad";
    static constexpr std::string_view CENTERLINE_NAME="Centerline";
  private:
    std::unique_ptr<btTriangleMesh> mesh_;
    std::unique_ptr<btBvhTriangleMeshShape> collision_shape_;
    std::unique_ptr<btDefaultMotionState> motion_state_;
    std::unique_ptr<btRigidBody> rigidbody_;

    std::shared_ptr<btDiscreteDynamicsWorld> world_;

    Centerline centerline_;
  };
}
