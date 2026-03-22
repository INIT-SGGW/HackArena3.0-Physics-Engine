#include "boink/simulators/track/ground.h"

#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>

#include "boink/exception.h"
#include "boink/collision_group.h"

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
      model_transform_(transform),
      user_data_(p_surface_info_)
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

    rigidbody_->setUserPointer((void*)&user_data_);

		world_->addRigidBody(
        rigidbody_.get(),
        Collision::Group::Static,
        Collision::Group::Vehicle);
  }

  Ground::Ground(Ground&& other) noexcept
      : mesh_(std::move(other.mesh_)),
        collision_shape_(std::move(other.collision_shape_)),
        motion_state_(std::move(other.motion_state_)),
        rigidbody_(std::move(other.rigidbody_)),
        world_(std::move(other.world_)),
        p_surface_info_(other.p_surface_info_),
        model_transform_(other.model_transform_),
        user_data_(std::move(other.user_data_))
  {
    rigidbody_->setUserPointer((void*)&user_data_);
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
