#pragma once

#include "boink/id_dispatcher.h"

#include <unordered_map>
#include <vector>
#include <algorithm>

namespace boink
{
  template<typename ID_, typename Value_>
  class IDRegistry
  {
  public:
    ID_ add(Value_ value)
    {
      ID_ new_id=id_dispatcher_.getNewID();
      values_.emplace(new_id,std::move(value));
      creation_order_.push_back(new_id);

      return new_id;
    }

    bool remove(ID_ id)
    {
      auto it=values_.find(id);

      if(it==values_.end())
        return false;

      values_.erase(it);
      id_dispatcher_.removeID(id);
      creation_order_.erase(
          std::remove(creation_order_.begin(),creation_order_.end(),id),
          creation_order_.end());

      return true;
    }

    const Value_& at(ID_ id) const {return values_.at(id);}
    Value_& at(ID_ id) {return values_.at(id);}

    auto begin() { return creation_order_.begin(); }
    auto end() { return creation_order_.end(); }

    auto cbegin() const { return creation_order_.cbegin(); }
    auto cend() const { return creation_order_.cend(); }
  private:
    IDDispatcher<ID_> id_dispatcher_;
    std::unordered_map<ID_,Value_> values_;
    std::vector<ID_> creation_order_;
  };
}
