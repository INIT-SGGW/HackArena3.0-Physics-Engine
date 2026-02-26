#include "boink/simulators/track/ground.h"

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

#include "boink/exception.h"

#include <memory>

namespace boink
{
  Ground::Ground(
      const std::vector<btVector3>& vertices, 
      const std::vector<unsigned int>& indices,
      const btTransform& transform,
      const SurfaceInfo* p_surface_info,
      std::shared_ptr<btDiscreteDynamicsWorld> world)
    :
      mesh_(new btTriangleMesh()),
      world_(world),
      p_surface_info_(p_surface_info),
      model_transform_(transform)
  {
    motion_state_=std::unique_ptr<btDefaultMotionState>(
        new btDefaultMotionState(transform));

    if(indices.size()%3!=0)
      throw Exception(
          Exception::Type::UnsupportedFormatError,
          "Expected indices size to be multiply of 3");

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

    rb_info.m_friction=1.0f;
    rigidbody_=std::unique_ptr<btRigidBody>(new btRigidBody(rb_info));

    rigidbody_->setUserPointer((void*)p_surface_info_);

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

  void Ground::setSurfaceInfo(const SurfaceInfo* p_surface_info)
  {
    p_surface_info_=p_surface_info;
  }



}
