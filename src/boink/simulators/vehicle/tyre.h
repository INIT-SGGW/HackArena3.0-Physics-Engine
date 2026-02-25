#pragma once

#include <LinearMath/btScalar.h>

namespace boink
{
  class Tyre
  {
  public:
    enum class Type
    {
      Soft,
      Hard,
      Wet,
      Count
    };
  public:
    Tyre(Type type, btScalar wear_rate);

    void update(btScalar dt);
    Type getType() const { return type_;}
    btScalar getHealth() const { return health_;}
    void setOnTyrePressure(btScalar tyre_pressure);
  public:
    static const char* toString(Type type);
  private:
    Type type_;
    btScalar health_=1.f;
    btScalar wear_rate_;
  };
}
