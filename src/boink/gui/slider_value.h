#pragma once

namespace boink
{
  class SliderValue
  {
  public:
    SliderValue()=default;

    SliderValue(float current)
      :current_(current),prev_(current)
    {}

    float& get() {return current_;}

    bool hasChanged() const
    {
      float epsilon=1e-5;
      float diff=prev_-current_;

      if(diff<0.f)
        diff*=-1.f;

      if(diff<epsilon)
        return false;

      prev_=current_;
      return true;
    }
  private:
    float current_;
    mutable float prev_;
  };
}
