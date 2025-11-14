#pragma once

#include "Eigen/Core"
#include "boink/components/component_manager.h"

#include "boink/components/transform.h"
#include "boink/components/kinematics.h"
#include "boink/components/bolid_model.h"

namespace boink
{
  class BolidSpawnSystem
  {
  public:
    /**
      * @brief Sets up all entities.
      *
      * @tparam Components_ The component types to that belong to ComponentManager.
      *
      * @param component_manager Reference to the ComponentManager storing components.
      * @param dt Delta time.
      */
    template<typename... Components_>
    void Setup(ComponentManager<Components_...>& component_manager,
        double dt)
    {
      auto view=component_manager.
        template getComponentView<BolidModel,Transform, Kinematics>();
      view.forEach(
        [&,dt](BolidModel& bolid_model, Transform& trans,Kinematics& kin)
        {
          using namespace Eigen;
          // TODO consider maybe const static component
          // type that is the same for all bolids like BolidModel

          // Assumes +X is right +Y is DOWN and +Z is forward

          // TODO here we setup initial values of a bolid
          // they must be fetched from client.
          bolid_model.front_left_wheel=Vector3d(-1.0,0.0,2.0);
          bolid_model.front_right_wheel=Vector3d(1.0,0.0,2.0);
          bolid_model.rear_left_wheel=Vector3d(-1.0,0.0,-2.0);
          bolid_model.rear_right_wheel=Vector3d(1.0,0.0,-2.0);
          trans.position=Vector3d::Zero();
          trans.rotation=Matrix3d::Identity();
          kin.velocity=Vector3d::Zero();
          kin.acceleration=Vector3d::Zero();

          // 
          bolid_model.front=
            0.5*(bolid_model.front_left_wheel-bolid_model.front_right_wheel)+
            bolid_model.front_right_wheel;

          Vector3d temp_rear=
            0.5*(bolid_model.rear_left_wheel-bolid_model.rear_right_wheel)+
            bolid_model.rear_right_wheel;
          bolid_model.direction=bolid_model.front-temp_rear;
          bolid_model.direction.normalize();
        }
      );
    }
  };
}
