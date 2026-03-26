#pragma once

#include <cmath>
#include <vector>

namespace boink
{
class Curve
// TODO: change vector for array to define Curve as constexpr
{
 public:
  Curve(std::vector<btScalar>&& values, btScalar step, btScalar min_arg)
      : values_(std::move(values)),
        inv_step_(1.0f / step),
        min_arg_(min_arg),
        max_index_(static_cast<int>(values_.size()) - 1)
  {
  }

  btScalar GetValue(btScalar x) const
  {
    auto normalized_x = (x - min_arg_) * inv_step_;

    if (normalized_x <= 0.0f) return values_[0];
    if (normalized_x >= max_index_) return values_[max_index_];

    int x0 = static_cast<int>(normalized_x);
    int x1 = x0 + 1;

    return values_[x0] + (values_[x1] - values_[x0]) * (normalized_x - x0);
  }

 private:
  const std::vector<btScalar> values_;
  const btScalar inv_step_;
  const btScalar min_arg_;
  const int max_index_;
};
}  // namespace boink