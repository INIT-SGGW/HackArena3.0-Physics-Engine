#pragma once

#include "Eigen/Core"
#include "boink/components/component_manager.h"

#include "boink/components/transform.h"
#include "boink/components/kinematics.h"
#include "boink/components/car_model.h"

#include <iostream>

namespace boink
{
    class DebugSystem
    {
    public:
        template <typename TupleStaticComponents_, typename TupleComponents_>
        void Setup(
            ComponentManager<TupleStaticComponents_,TupleComponents_>& component_manager,
            double dt)
        {
            size_t index = 0;
            auto view = component_manager.
              template getComponentView<Transform, Kinematics>();
            auto static_comps=component_manager.
              template getStaticComponentView<CarModel>();
            const auto& model = std::get<CarModel&>(static_comps);
            view.forEach(
                [&](Transform& trans, Kinematics& kin)
                {
                    Log(dt,model,trans,kin,component_manager.getIDByIndex(index));
                    index++;
                }
            );
        }

        template <typename TupleStaticComponents_, typename TupleComponents_>
        void Update(
            ComponentManager<TupleStaticComponents_,TupleComponents_>& component_manager,
            double dt)
        {
            size_t index = 0;
            auto view = component_manager.
              template getComponentView<Transform, Kinematics>();
            auto static_comps=component_manager.
              template getStaticComponentView<CarModel>();
            const auto& model = std::get<CarModel&>(static_comps);
            view.forEach(
                [&](Transform& trans, Kinematics& kin)
                {
                    Log(dt,model,trans,kin,component_manager.getIDByIndex(index));
                    index++;
                }
            );
        }

    private:
        void PrintVector(const Eigen::Vector3d& vec)
        {
            std::cout << "X: " << vec.x() << std::endl;
            std::cout << "Y: " << vec.y() << std::endl;
            std::cout << "Z: " << vec.z() << std::endl;
        }
        void Log(
            double dt,const CarModel& model,Transform& trans, Kinematics& kin, size_t id)
        {
            std::cout << "Delta time: "<<dt<<std::endl;
            std::cout << "Data for ";
            std::cout << id<<" object:" << std::endl;
            
            std::cout<<"Position"<<std::endl;
            PrintVector(trans.position);
            std::cout << std::endl;

            std::cout<<"Rotation"<<std::endl;
            std::cout <<trans.rotation<< std::endl;

            std::cout<<"Model direction"<<std::endl;
            PrintVector(model.direction);
            std::cout << std::endl;

            std::cout<<"Velocity"<<std::endl;
            PrintVector(kin.velocity);
            std::cout << std::endl;
        }
    };
}
