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
      Tarmac,
      Grass,
      Sand
    };

    struct SurfaceInfo
    {
      btScalar friction;
      btScalar resistive_coef;
      btScalar rolling_resistance;
      Type type;
    };
  public:
    Ground(
        const std::vector<btVector3>& vertices, 
        const std::vector<unsigned int> indices,
        const btTransform& transform,
        Type type,
        std::shared_ptr<btDiscreteDynamicsWorld> world);

    Ground(const Ground&)=delete;
    Ground(Ground&&)=default;

    Ground& operator=(const Ground&)=delete;
    Ground& operator=(Ground&&)=delete;

    ~Ground() noexcept;

    void setWorldTransform(const btTransform& transform);
    btTransform getWorldTransform() const;

    const btTransform& getModelTransform() const {return model_transform_;}
  private:
    std::unique_ptr<btTriangleMesh> mesh_;
    std::unique_ptr<btBvhTriangleMeshShape> collision_shape_;
    std::unique_ptr<btDefaultMotionState> motion_state_;
    std::unique_ptr<btRigidBody> rigidbody_;

    std::shared_ptr<btDiscreteDynamicsWorld> world_;

    Type surface_type_;
    const btTransform model_transform_;
  private:
    static SurfaceInfo s_kTarmacSuraface_;
    static SurfaceInfo s_kGrassSuraface_;
    static SurfaceInfo s_kSandSuraface_;
  public:
    static SurfaceInfo& getSurfaceInfo(Type type);
    static inline const char* toString(Type type)
    {
      switch(type)
      {
        case Type::Grass:
          return "grass";
        case Type::Tarmac:
          return "tarmac";
        case Type::Sand:
          return "sand";
      }

      assert(false && "Invalid Ground Type");
      return "Unknown";
    }
  };
}
