#include "boink/simulation/track.h"


#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <LinearMath/btDefaultMotionState.h>

#include "boink/gltf_extractor.h"

#include <memory>
#include <vector>

namespace boink
{
  Track::Track(std::string_view filename,
      std::shared_ptr<btDiscreteDynamicsWorld> world)
    :
      mesh_(new btTriangleMesh()),
      world_(world)
  {
    GltfExtractor extractor(filename);
    const auto& node=extractor.getNode(TRACK_NAME);
    std::vector<btVector3> vertices=node.vertices;
    std::vector<unsigned int> indices=node.indices;

    motion_state_=std::unique_ptr<btDefaultMotionState>(
        new btDefaultMotionState(node.transform));

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

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool is_dynamic = (mass != 0.f);

		btVector3 local_inertia(0, 0, 0);
		if (is_dynamic)
			collision_shape_->calculateLocalInertia(mass, local_inertia);

		btRigidBody::btRigidBodyConstructionInfo rb_info
      (mass, motion_state_.get(), collision_shape_.get(), local_inertia);

    rigidbody_=std::unique_ptr<btRigidBody>(new btRigidBody(rb_info));

		world_->addRigidBody(rigidbody_.get());
  }

  Track::~Track() noexcept
  {
    if(world_)
      world_->removeRigidBody(rigidbody_.get());
  }

  const btVector3& Track::getPosition() const
  {
     return rigidbody_->getWorldTransform().getOrigin();
  }

  void Track::setPosition(const btVector3& position)
  {
    rigidbody_->getWorldTransform().setOrigin(position);
  }

}
