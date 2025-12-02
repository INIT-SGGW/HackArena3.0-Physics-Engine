#include "boink/world.h"

using namespace boink;
using namespace Eigen;

void test()
{
  CarModel car_model;
  car_model.front_left_wheel=Vector3d(-1.0,0.0,2.0);
  car_model.front_right_wheel=Vector3d(1.0,0.0,2.0);
  car_model.rear_left_wheel=Vector3d(-1.0,0.0,-2.0);
  car_model.rear_right_wheel=Vector3d(1.0,0.0,-2.0);
  car_model.max_steer_angle_deg=30;

  World world(car_model);
  Entity::ID id=world.car_manager.addCar();

  world.start(0.0);

  CarInput input;
  input.brake=1.0;
  input.steer_angle=0.0;
  input.throttle=0.0;
  world.car_manager.updateCar(id,input);

  for(int i=0;i<3;i++)
  {
    world.update(0.5);

    auto [input,trans,kins]=world.car_manager.
      getCarComponents<boink::CarInput,boink::Transform,boink::Kinematics>(id);

    auto model=world.
      car_manager.getCarStaticComponents<boink::CarModel>();

    double max_angle=std::get<boink::CarModel&>(model).max_steer_angle_deg;

    //out_state->brake_applied=input.brake;
    //out_state->throttle_applied=input.throttle;
    double angle=boink::math::deg2rad(input.steer_angle/2.0*max_angle);

    //out_state->wheel_angles[0]=angle;
    //out_state->wheel_angles[1]=angle;
    //out_state->speed=kins.velocity.norm();

    Eigen::Vector3d rpy=trans.rotation.canonicalEulerAngles(2,1,0);
    //out_state->orientation.roll=rpy(0);
    //out_state->orientation.pitch=rpy(1);
    //out_state->orientation.yaw=rpy(2);

    //out_state->position={trans.position.x(),trans.position.y(),trans.position.z()};
  }
}

int main()
{
  test();
  return 0;
}
