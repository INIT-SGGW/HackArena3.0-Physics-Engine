#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "boink/components/component_manager.h"

#include "boink/components/transform.h"
#include "boink/components/kinematics.h"
#include "boink/components/car_model.h"

namespace boink
{
  class CarSpawnSystem
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
    template <typename TupleStaticComponents_, typename TupleComponents_>
    void Setup(
        ComponentManager<TupleStaticComponents_,TupleComponents_>& component_manager,
        double dt)
    {
      auto view=component_manager.
        template getComponentView<Transform, Kinematics>();
      auto static_view=component_manager.
        template getStaticComponentView<CarModel>();

      auto& car_model=std::get<CarModel&>(static_view);

      view.forEach(
        [&,dt](Transform& trans,Kinematics& kin)
        {
          using namespace Eigen;

          // TODO those values should be somehow fetched on creation of a car
          // or updated somewhere somehow
          trans.position=Vector3d::Zero();
          trans.rotation=Matrix3d::Identity();
          kin.velocity=Vector3d::Zero();
          kin.acceleration=0.0;

          // 
          car_model.front=
            0.5*(car_model.front_left_wheel-car_model.front_right_wheel)+
            car_model.front_right_wheel;

          Vector3d temp_rear=
            0.5*(car_model.rear_left_wheel-car_model.rear_right_wheel)+
            car_model.rear_right_wheel;
          car_model.direction=car_model.front-temp_rear;
          car_model.direction.normalize();

          Vector3d temp=car_model.rear_left_wheel -car_model.rear_right_wheel;
          
          // It should always point to the top of a car
          car_model.normal=car_model.direction.cross(temp);
          car_model.normal.normalize();
        }
      );
    }
  };
}
