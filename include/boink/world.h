#pragma once

#include <type_traits>

#if !defined(BOINK_API)
#if defined(_WIN32) || defined(__CYGWIN__)
#if defined(BOINK_BUILD_DLL)
#define BOINK_API __declspec(dllexport)
#elif defined(BOINK_USE_DLL)
#define BOINK_API __declspec(dllimport)
#else
#define BOINK_API
#endif
#else
#define BOINK_API __attribute__((visibility("default")))
#endif
#endif

#include "boink/components/transform.h"
#include "boink/components/kinematics.h"
#include "boink/components/car_inputs.h"
#include "boink/components/car_model.h"
#include "boink/components/rigidbody.h"

#include "boink/car_manager.h"
#include "boink/simulation.h"
#include "boink/debug_render.h"
#include "boink/utils/tuple_unpack.h"

namespace boink
{
  /**
   * @brief Represents the world simulation containing all entities and systems.
   *
   */
  class BOINK_API World
  {
  public:
    using CarComponents=std::tuple<CarInput,Rigidbody,Transform,Kinematics>;
    using CarSystems=std::tuple<>;
  public:
    World(CarModel&& car_model);

    /**
     * @brief Start simulation.
     *
     */
    void start(double dt);

    /**
     * @brief Update the world for a simulation step.
     *
     * @param dt Delta time in seconds.
     */
    void update(double dt);

    /**
     * @brief Binds visual debuger with simulation.
     *
     * @param dbg Pointer to the DebugRender object.
     */
    void setDebuger(DebugRender* dbg);

    CarManager<CarComponents,CarSystems>& getCarManager();

    template<typename... Ts_>
    Entity::ID addCar(Ts_&&... components)
    {
      // Checks whether Ts_ types are equal to Components_ order matters.
      static_assert(
        std::is_same_v<
          std::tuple<std::remove_cvref_t<Ts_>...>,
          CarComponents 
        >,
        "Provided number of components is not the same or"
        " order is not matched"
      );

      auto&& rb=getComponent<Rigidbody>(components...);

      // Ensure that rigidbody can be modified.
      static_assert(
          !std::is_const_v<std::remove_reference_t<decltype(rb)>>,
          "The Rigidbody componet cannot be const.");

      const CarModel& car_model=
        std::get<0>(car_manager_.getCarStaticComponents<CarModel>());

      rb=simulation.createCarRigidbody(car_model);

      return car_manager_.addCar(std::forward<Ts_>(components)...);
    }

    void addGround(const btVector3& dims, const btVector3& pos);
    void addSphere(btScalar radius, const btVector3& pos);
  private:
    // The order of simulation and car_manger objects
    // must be preserved because custom deleter of Rigidbody object
    // needs dynamic_world to exists.
    Simulation simulation;
    CarManager<CarComponents,CarSystems> car_manager_;
    double time_passed_=0.0;
  };
}

