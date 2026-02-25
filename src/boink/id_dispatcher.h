#pragma once

namespace boink
{
  template<typename ID_>
  class IDDispatcher
  {
  public:
    ID_ getNewID() const
    {
      return available_id_++;
    }
    void removeID(ID_ id)
    {
      (void)id;
    }
  private:
    mutable ID_ available_id_=0;
  };
}
