#include <tuple>

#include "boink/components/car_inputs.h"
#include "boink/components/transform.h"
#include "boink/components/kinematics.h"

#include "boink/car_manager.h"
#include "boink/world.h"

using namespace boink;

void test_tuple();
void test_component_manager();
void test_component_manager2();
void test_component_manager3();
void test_component_manager4();
int main()
{
  std::cout<<"----------------------------------"<<std::endl;
  std::cout<<"Tuple Test"<<std::endl;
  test_tuple();
  std::cout<<"----------------------------------"<<std::endl;
  std::cout<<"Component Manager Test 1"<<std::endl;
  test_component_manager();
  std::cout<<"----------------------------------"<<std::endl;
  std::cout<<"Component Manager Test 2"<<std::endl;
  test_component_manager2();
  std::cout<<"----------------------------------"<<std::endl;
  std::cout<<"Component Manager Test 3"<<std::endl;
  test_component_manager3();
  std::cout<<"----------------------------------"<<std::endl;
  std::cout<<"Component Manager Test 4"<<std::endl;
  test_component_manager4();
  std::cout<<"----------------------------------"<<std::endl;

  return 0;
}

void test_tuple()
{
  Transform trans;
  trans.position={1.0,2.0,3.0};
  trans.rotation<<
    1.0,0.0,0.0,
    0.0,1.0,0.0,
    0.0,0.0,1.0;

  std::tuple<Transform> tuple_trans{trans};
  std::tuple<Transform> tuple_trans2{std::move(trans)};
}

void test_component_manager()
{

  CarManager<std::tuple<Transform,Kinematics>,std::tuple<>> car_manager({});
  Entity::ID car_id=car_manager.addCar();

}

void test_component_manager2()
{
  Transform trans;
  trans.position={1.0,2.0,3.0};
  trans.rotation<<
    1.0,0.0,0.0,
    0.0,1.0,0.0,
    0.0,0.0,1.0;
  Kinematics kins;
  kins.acceleration=10.0;
  kins.velocity={7.0,9.0,1.0};

  CarManager<std::tuple<Transform,Kinematics>,std::tuple<>> car_manager({});
  Entity::ID car_id=car_manager.addCar(trans,kins);
}

void test_component_manager3()
{
  Transform trans;
  trans.position={1.0,2.0,3.0};
  trans.rotation<<
    1.0,0.0,0.0,
    0.0,1.0,0.0,
    0.0,0.0,1.0;

  CarManager<std::tuple<Transform,Kinematics>,std::tuple<>> car_manager({});
  Entity::ID car_id=car_manager.addCar(trans,Kinematics{});
}

void test_component_manager4()
{

  Transform trans;
  trans.position={1.0,2.0,3.0};
  trans.rotation<<
    1.0,0.0,0.0,
    0.0,1.0,0.0,
    0.0,0.0,1.0;

  // STATIC COMPONENT some strange behavior
  //CarModel car_model;
  //car_model.front_left_wheel={1.0,1.0,1.0};
  World world({});
  auto tuple=world.car_manager.getCarStaticComponents<CarModel>();
  std::cout<<std::get<0>(tuple).front_left_wheel<<std::endl;
  std::get<0>(tuple).front_left_wheel={2.0,2.0,2.0};

  tuple=world.car_manager.getCarStaticComponents<CarModel>();
  std::cout<<std::get<0>(tuple).front_left_wheel<<std::endl;
  
  Entity::ID car_id=world.car_manager.addCar(CarInput{},trans,Kinematics{});
  world.start(0.0);
  auto comps=world.car_manager.
    getCarComponents<boink::Transform,boink::Kinematics>(car_id);
  std::get<Transform&>(comps).position={1.0,1.0,1.0};
  world.update(0.5);

}

