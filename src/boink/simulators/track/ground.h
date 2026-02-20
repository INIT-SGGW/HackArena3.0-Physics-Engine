#pragma once

#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <LinearMath/btDefaultMotionState.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>

#include <memory>
#include <vector>
#include <cassert>

namespace boink
{
  class Ground
  {
  public:
    enum class Type
    {
      Asphalt,
      Grass,
      Sand,
      Gravel
    };

    struct SurfaceInfo
    {
      btScalar resistive_coef;
      btScalar rolling_resistance;
      Type type;
    };
  public:
    Ground(
        const std::vector<btVector3>& vertices, 
        const std::vector<unsigned int>& indices,
        const btTransform& transform,
        const SurfaceInfo* p_surface_info,
        std::shared_ptr<btDiscreteDynamicsWorld> world);

    Ground(const Ground&)=delete;
    Ground(Ground&&)=default;

    Ground& operator=(const Ground&)=delete;
    Ground& operator=(Ground&&)=delete;

    ~Ground() noexcept;

    void setWorldTransform(const btTransform& transform);
    btTransform getWorldTransform() const;

    void setSurfaceInfo(const SurfaceInfo* p_surface_info);
    const SurfaceInfo* getSurfaceInfo() const {return p_surface_info_;}

    const btTransform& getModelTransform() const {return model_transform_;}
  private:
    std::unique_ptr<btTriangleMesh> mesh_;
    std::unique_ptr<btBvhTriangleMeshShape> collision_shape_;
    std::unique_ptr<btDefaultMotionState> motion_state_;
    std::unique_ptr<btRigidBody> rigidbody_;

    std::shared_ptr<btDiscreteDynamicsWorld> world_;

    const SurfaceInfo* p_surface_info_;
    const btTransform model_transform_;
  public:
    static inline const char* toString(Type type)
    {
      switch(type)
      {
        case Type::Grass:
          return "grass";
        case Type::Asphalt:
          return "asphalt";
        case Type::Sand:
          return "sand";
        case Type::Gravel:
          return "sand";
      }

      assert(false && "Invalid Ground Type");
      return "Unknown";
    }
  };
}
