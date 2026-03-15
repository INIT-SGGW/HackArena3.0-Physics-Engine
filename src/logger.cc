#include "boink/logger.h"
#include <spdlog/common.h>
#include <spdlog/spdlog.h>

namespace boink
{
  void Logger::init(const std::string& log_file)
  {
      file_sink_ = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
          log_file, true);
      file_sink_->set_level(spdlog::level::trace);
      file_sink_->set_pattern("[%Y-%m-%d %H:%M:%S] [BOINK] [%^%l%$] [%s:%#] %v");

      console_sink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
#ifndef NDEBUG
      console_sink_->set_level(spdlog::level::debug);
#else
      console_sink_->set_level(spdlog::level::info);
#endif 
      console_sink_->set_pattern("[%H:%M:%S] [BOINK] [%^%l%$] [%s:%#] %v");

      std::vector<spdlog::sink_ptr> sinks {console_sink_, file_sink_};
      logger_ = std::make_shared<spdlog::logger>(
          "multi_sink", sinks.begin(), sinks.end());
      logger_->set_level(spdlog::level::trace);
  }

  std::shared_ptr<spdlog::logger> Logger::logger_ = nullptr;
  std::shared_ptr<spdlog::sinks::basic_file_sink_mt> Logger::file_sink_=
    nullptr;
  std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> Logger::console_sink_=
    nullptr;

}
