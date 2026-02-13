#include "boink/simulators/track/ground.h"
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <memory>

namespace boink
{
  Ground::Ground(
      const std::vector<btVector3>& vertices, 
      const std::vector<unsigned int> indices,
      const btTransform& transform,
      Type type,
      std::shared_ptr<btDiscreteDynamicsWorld> world)
    :
      mesh_(new btTriangleMesh()),
      world_(world),
      surface_type_(type),
      model_transform_(transform)
  {
    motion_state_=std::unique_ptr<btDefaultMotionState>(
        new btDefaultMotionState(transform));
    assert(indices.size()%3==0);

    for(size_t i=0;i<indices.size();i+=3)
    {
      mesh_->addTriangle(
          vertices[indices[i]],
          vertices[indices[i+1]],
          vertices[indices[i+2]]);
    }

    collision_shape_=std::unique_ptr<btBvhTriangleMeshShape>(
        new btBvhTriangleMeshShape(mesh_.get(),true));
                                           
		btScalar mass(0.);
		btVector3 local_inertia(0, 0, 0);

		btRigidBody::btRigidBodyConstructionInfo rb_info
      (mass, motion_state_.get(), collision_shape_.get(), local_inertia);

    rigidbody_=std::unique_ptr<btRigidBody>(new btRigidBody(rb_info));

    rigidbody_->setUserPointer(&this->getSurfaceInfo(surface_type_));

		world_->addRigidBody(rigidbody_.get());
  }

  Ground::~Ground() noexcept
  {
    // Must check in order for move semantics to work.
    if(world_)
      world_->removeRigidBody(rigidbody_.get());
  }

  void Ground::setWorldTransform(const btTransform& transform)
  {
    rigidbody_->setWorldTransform(transform);
  }

  btTransform Ground::getWorldTransform() const
  {
    return rigidbody_->getWorldTransform();
  }

  Ground::SurfaceInfo Ground::s_kGrassSuraface_= {1.5f,0.5f,0.3f,Type::Grass};
  Ground::SurfaceInfo Ground::s_kSandSuraface_= {3.5f,5.f,0.4f,Type::Sand};
  Ground::SurfaceInfo Ground::s_kTarmacSuraface_= {3.5f,0.0f,0.1f,Type::Tarmac};

  Ground::SurfaceInfo& Ground::getSurfaceInfo(Type type)
  {
    switch(type)
    {
      case Type::Grass:
        return s_kGrassSuraface_;
      case Type::Tarmac:
        return s_kTarmacSuraface_;
      case Type::Sand:
        return s_kSandSuraface_;
    }

    assert(false && "Invalid Ground Type");
    return s_kTarmacSuraface_;
  }

}
