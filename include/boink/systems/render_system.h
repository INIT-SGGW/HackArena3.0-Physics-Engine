#pragma once

#include "boink/components/component_manager.h"

#include "boink/components/transform.h"
#include "boink/components/car_model.h"

namespace boink
{
  class RenderSystem
  {
  public:
    template <typename TupleStaticComponents_, typename TupleComponents_>
    void Setup(
        ComponentManager<TupleStaticComponents_,TupleComponents_>& component_manager,
        double dt)
    {
        auto view = component_manager.
          template getComponentView<Transform>();
        auto static_comps=component_manager.
          template getStaticComponentView<CarModel>();
        const auto& model = std::get<CarModel&>(static_comps);
        view.forEach(
            [&](Transform& trans)
            {
            }
        );
    }
    template <typename TupleStaticComponents_, typename TupleComponents_>
    void Update(
      ComponentManager<TupleStaticComponents_,TupleComponents_>& component_manager,
      double dt)
    {
      auto view=component_manager
        .template getComponentView<Transform>();

      auto static_comps=component_manager.
        template getStaticComponentView<CarModel>();
      const auto& model = std::get<CarModel&>(static_comps);

      view.forEach(
        [&, dt](
          Transform& trans
        )
        {
        }
      );
    }

    template <typename TupleStaticComponents_, typename TupleComponents_>
    void UpdateWhenPossible(
      ComponentManager<TupleStaticComponents_,TupleComponents_>& component_manager,
      double dt)
    {

    }
  private:
  
  };
}
