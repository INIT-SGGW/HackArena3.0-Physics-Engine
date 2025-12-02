#pragma once

#include "Eigen/Core"
#include "boink/components/component_manager.h"

#include "boink/components/transform.h"
#include "boink/components/car_inputs.h"
#include "boink/components/car_model.h"
#include "boink/components/kinematics.h"

#include "boink/utils/math.h"

#include <cmath>

namespace boink
{
  /**
   * @brief Simulates movement of a vehicle
   *
   */
  class MovementSystem
  {
  public:
    /**
      * @brief Updates all entities with Transform and Velocity components.
      *
      * @tparam Components_ The component types to that belong to ComponentManager.
      *
      * @param component_manager Reference to the ComponentManager storing components.
      * @param dt Delta time.
      */
    template <typename TupleStaticComponents_, typename TupleComponents_>
    void Update(
      ComponentManager<TupleStaticComponents_,TupleComponents_>& component_manager,
      double dt)
    {
      auto view=component_manager
        .template getComponentView<CarInput,Transform,Kinematics>();

      auto static_comps=component_manager.
        template getStaticComponentView<CarModel>();
      const auto& model = std::get<CarModel&>(static_comps);

      view.forEach(
        [&, dt](
          const CarInput& input,
          Transform& trans, 
          Kinematics& kin
        )
        {
          using namespace Eigen;

          // Assumes that Y axis points to graviational force.
          // Assumes the right hand coordinates system.
          // Assumes that during delta time car moves with constant acceleration.
          // Assumes that postive angle means roaton counterclockwise.
          // this means that angle_x is postive when rotation is from Y to Z, for
          // angle_y is from Z to X and for angle_z from X to Y.
          
          // Assumes that initial car postion is at 0,0,0 if not displacement
          // vector must be provided
          //
          //Assumes breaking is just accelerting in oppostie dir

          // Steer_angle >0 -> left; <0 -> right

          // TODO Simulate engine output to the wheels
          // must be in other system
          kin.acceleration=input.throttle-input.brake;

          double turn_angle=
            math::deg2rad(input.steer_angle * model.max_steer_angle_deg);

          double sin_turn=std::sin(turn_angle);
          double cos_turn=std::cos(turn_angle);
          Matrix3d turn_rotation=math::getRodriguesRotationMatrix(sin_turn,cos_turn,model.normal);
          Vector3d wheel_direction=
            turn_rotation*model.direction;

          // Simulate car moement
          if(kin.acceleration*dt + kin.velocity.norm()<0)
          {
            kin.acceleration=0;
            kin.velocity=0*wheel_direction;
          }
          else
            kin.velocity=(kin.velocity.norm()+kin.acceleration*dt)*wheel_direction;

          Vector3d front_displacement=
            kin.velocity*dt+
            (0.5*kin.acceleration*dt*dt)*wheel_direction;

          Vector3d new_model_direction=(model.direction+front_displacement);
          new_model_direction.normalize();
          Matrix3d delta_rotation=
            math::getRotationMatrix(
                model.direction,new_model_direction);

          Vector3d new_front=model.front+front_displacement;
          Vector3d rot_front=delta_rotation*model.front;
          Vector3d front_new_disp_after_d_rot=new_front-rot_front;

          // Its rotation matrix so traspose equals to inverse.
          trans.position=
            trans.rotation.transpose()*front_new_disp_after_d_rot+
            trans.position;

          // Order matters
          trans.rotation=trans.rotation*delta_rotation;
        }
      );
    }
  };
}
