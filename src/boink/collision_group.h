#pragma once

namespace boink::Collision
{
  enum Group : int
  {
    None=0,
    Static=1<<0,
    Vehicle=1<<1,

    All=-1
  };
}
