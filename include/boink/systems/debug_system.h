#pragma once

#include "Eigen/Core"
#include "boink/components/component_manager.h"

#include "boink/components/transform.h"
#include "boink/components/kinematics.h"
#include "boink/components/bolid_model.h"

#include <iostream>

namespace boink
{
    class DebugSystem
    {
    public:
        template<typename... Components_>
        void Setup(ComponentManager<Components_...>& component_manager,
            double)
        {
            size_t index = 0;
            auto view = component_manager.
              template getComponentView<BolidModel,Transform, Kinematics>();
            view.forEach(
                [&](BolidModel& model,Transform& trans, Kinematics& kin)
                {
                    Log(model,trans,kin,component_manager.getIDByIndex(index));
                    index++;
                }
            );
        }

        template<typename... Components_>
        void Update(ComponentManager<Components_...>& component_manager,
            double)
        {
            size_t index = 0;
            auto view = component_manager.
              template getComponentView<BolidModel,Transform, Kinematics>();
            view.forEach(
                [&](BolidModel& model,Transform& trans, Kinematics& kin)
                {
                    Log(model,trans,kin,component_manager.getIDByIndex(index));
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
        void Log(BolidModel& model,Transform& trans, Kinematics& kin, size_t id)
        {
            std::cout << "Data for ";
            std::cout << id<<" object:" << std::endl;
            
            std::cout<<"Position"<<std::endl;
            PrintVector(trans.position);
            std::cout << std::endl;

            std::cout<<"Model front"<<std::endl;
            PrintVector(model.front);
            std::cout << std::endl;

            std::cout<<"Model direction"<<std::endl;
            PrintVector(model.direction);
            std::cout << std::endl;

            std::cout<<"Velocity"<<std::endl;
            PrintVector(kin.velocity);
            std::cout << std::endl;
        }
    };
}
