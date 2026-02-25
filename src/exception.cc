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
    buffer_=getFormattedMessage();
    return buffer_.data();
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
    ss<<"Boink exception"<<std::endl;
    ss<<"Type: "<<getTypeString(getType())<<std::endl;
    ss<<"File: "<<getFile()<<std::endl;
    ss<<"Line: "<<getLine()<<std::endl;
    ss<<"Message: "<<msg_;

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
      case Type::NotFoundError:
        return "NotFoundError";
      case Type::InternalError:
        return "InternalError";
    }

    return "";
  }
}
