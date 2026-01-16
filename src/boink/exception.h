#pragma once

#include <exception>
#include <string>
#include <source_location>

namespace boink{
  class Exception : public std::exception
  {
  public:
    enum class Type
    {
      IOError,
      InvalidArgumentError,
      UnsupportedFormatError,
      NotFoundError,
    };
  public:
    Exception(
        Type type,
        const std::string& msg, 
        const std::source_location location=std::source_location::current()
    );

    ~Exception() override=default;

    virtual const char* what() const noexcept override;

    Type getType() const noexcept;
    std::string_view getFile() const noexcept;
    unsigned int getLine() const noexcept;
  private:
    std::string getFormattedMessage() const;
    static std::string_view getTypeString(Type type);
  private:
    Type type_;
    std::string msg_;
    const std::source_location location_;
    mutable std::string buffer_;
  };
}
