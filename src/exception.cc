#include "boink/exception.h"

#include <sstream>

namespace boink
{
  Exception::Exception(
        Type type,
        const std::string& msg, 
        const std::source_location location)
    :
      type_(type),msg_(msg),location_(location)
  {
  }

  const char* Exception::what() const noexcept 
  {
    return msg_.c_str();
  }

  Exception::Type Exception::getType() const noexcept
  {
    return type_;
  }

  std::string_view Exception::getFile()const noexcept
  {
    return location_.file_name();
  }

  unsigned int Exception::getLine()const noexcept
  {
    return location_.line();
  }

  std::string Exception::getFormattedMessage() const
  {
    std::stringstream ss;
    ss<<"Type: "<<getTypeString(getType())<<std::endl;
    ss<<"File: "<<getFile()<<std::endl;
    ss<<"Line: "<<getLine()<<std::endl;
    ss<<"Message: "<<what()<<std::endl;

    return ss.str();
  }

  std::string_view Exception::getTypeString(Exception::Type type)
  {
    switch(type)
    {
      case Type::InvalidArgumentError:
        return "InvalidArgumentError";
      case Type::UnsupportedFormatError:
        return "UnsupportedFormatError";
      case Type::IOError:
        return "IOError";
    }

    return "";
  }
}
