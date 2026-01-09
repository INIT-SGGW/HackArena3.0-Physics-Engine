#include "boink/simulation/track.h"


#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <LinearMath/btDefaultMotionState.h>
#include "boink/utils/utility.h"
#include <memory>
#include <piksel/model.hh>
#include <vector>

namespace boink
{
  Track::Track(std::string_view filename,
      std::shared_ptr<btDiscreteDynamicsWorld> world)
    :
      mesh_(new btTriangleMesh()),
      world_(world)
  {
    // This is not optimal but it is run once on the start.
    piksel::Model model(filename,1.f);
    std::vector<btVector3> vertices;
    std::vector<unsigned int> indices;
    for(const auto& mesh:model.getMeshes())
    {
      auto [scale,transform]=glm2bt(mesh.getTransform());
      for(const auto& vertex:mesh.getVertices())
      {
        btVector3 bt_vertex=glm2bt(vertex.pos);
        bt_vertex.setX(bt_vertex.getX()*scale.getX());
        bt_vertex.setY(bt_vertex.getY()*scale.getY());
        bt_vertex.setZ(bt_vertex.getZ()*scale.getZ());
        bt_vertex=transform*bt_vertex;
        vertices.push_back(bt_vertex);
      }
      for(const auto& index:mesh.getIndices())
      {
        indices.push_back(index);
      }
    }

    motion_state_=std::unique_ptr<btDefaultMotionState>(
        new btDefaultMotionState(glm2bt(model.getTransform()).second));

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
}
