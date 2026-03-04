#pragma once

namespace boink
{
  struct BulletUserData
  {
    enum class Type
    {
      Ground,
      Vehicle,
      Default
    };

    BulletUserData()
      :type_(Type::Default)
    {}

    Type getType() const {return type_;}
  protected:
    explicit BulletUserData(Type type)
      :type_(type)
    {}
  private:
    Type type_;
  };
}
