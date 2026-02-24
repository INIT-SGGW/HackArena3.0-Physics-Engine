#include "boink/simulators/vehicle/tyre.h"

#include "boink/exception.h"

namespace boink
{
  Tyre::Tyre(Type type,btScalar wear_rate)
    :type_(type),wear_rate_(wear_rate)
  {}

  void Tyre::update(btScalar dt)
  {
    health_-=wear_rate_*dt;
    if(health_<0.f)
      health_=0.f;
  }

  void Tyre::setOnTyrePressure(btScalar tyre_pressure)
  {
    (void)tyre_pressure;
    //TODO
  }

  const char* Tyre::toString(Type type)
  {
    switch(type)
    {
      case Type::Hard:
        return "hard";
      case Type::Soft:
        return "soft";
      case Type::Wet:
        return "wet";
      default:
        throw Exception(
            Exception::Type::InternalError,
            "Called toString with type Count or unspecified");
    }
  }
}
