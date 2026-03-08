#pragma once

namespace boink
{
  enum CollisionGroup : int
  {
    None=0,
    Static=1<<0,
    Vehicle=1<<1,

    All=-1
  };
}
