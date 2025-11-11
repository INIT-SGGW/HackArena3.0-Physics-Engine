#pragma once

#include "boink/components/component_manager.h"

#include "boink/components/transform.h"
#include "boink/components/velocity.h"

#include <iostream>

namespace boink
{
    template<typename... Components_>
    class DebugSystem
    {
    public:
        void Update(ComponentManager<Components_...>& component_manager,
            double)
        {
            size_t index = 0;
            auto view = component_manager.template getComponentView<Transform, Velocity>();
            view.forEach(
                [&index](Transform& trans, Velocity& vel)
            {
                std::cout << "Data for " << index << " object:" << std::endl;
                std::cout << "X: " << trans.position.x() << std::endl;
                std::cout << "Y: " << trans.position.y() << std::endl;
                std::cout << "Z: " << trans.position.z() << std::endl;
                std::cout << std::endl;
                std::cout << "RotX: " << trans.rotation.x() << std::endl;
                std::cout << "RotY: " << trans.rotation.y() << std::endl;
                std::cout << "RotZ: " << trans.rotation.z() << std::endl;
                std::cout << std::endl;
                std::cout << "VelX: " << vel.velocity.x() << std::endl;
                std::cout << "VelY: " << vel.velocity.y() << std::endl;
                std::cout << "VelZ: " << vel.velocity.z() << std::endl;
                std::cout << std::endl;
            }
            );
        }
    };
}
