#pragma once

#include "boink/components/component_manager.h"

#include "boink/components/transform.h"
#include "boink/components/car_model.h"

#include "piksel/window.hh"
#include "piksel/graphics.hh"
#include "piksel/camera.hh"

#include "piksel/config.hh"

#include "glm/gtc/matrix_transform.hpp"

namespace boink
{
  class RenderSystem
  {
  public:
    RenderSystem()
      :wnd("Debug",800,600),cam({10.f,0.f,0.f},{0.f,0.f,0.f}),
      gfx(wnd,cam),cube(5.5f,1.5f,1.f,ASSET_PATH"/container.jpg",0)
    {}

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
          cube.translate=glm::translate(glm::mat4(1.f),{0.f,0.00f,0.0f});
          cube.rotate=glm::rotate(cube.rotate,glm::radians(360.f*(float)dt/2),
              {1.f,0.f,1.f});
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
    piksel::Window wnd;
    piksel::Camera cam;
    piksel::Graphics gfx;
    piksel::Cube cube;
  };
}
