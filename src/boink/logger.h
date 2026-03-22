#pragma once

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/logger.h"
#include "spdlog/spdlog.h"
#include "boink/fmt_btvector3.h"

#include <memory>

namespace boink
{
  class Logger {
  public:
    static void init(const std::string& log_file="logs/boink.log");

    static std::shared_ptr<spdlog::sinks::basic_file_sink_mt> file_sink() 
    { return file_sink_; }
    static std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> console_sink() 
    { return console_sink_; }

    static std::shared_ptr<spdlog::logger>& get_logger() { return logger_; }

private:
    static std::shared_ptr<spdlog::logger> logger_;
    static std::shared_ptr<spdlog::sinks::basic_file_sink_mt> file_sink_;
    static std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> console_sink_;
  };
}

#define BOINK_TRACE(...)    SPDLOG_LOGGER_CALL(boink::Logger::get_logger(), spdlog::level::trace, __VA_ARGS__)
#define BOINK_DEBUG(...)    SPDLOG_LOGGER_CALL(boink::Logger::get_logger(), spdlog::level::debug, __VA_ARGS__)
#define BOINK_INFO(...)     SPDLOG_LOGGER_CALL(boink::Logger::get_logger(), spdlog::level::info, __VA_ARGS__)
#define BOINK_WARN(...)     SPDLOG_LOGGER_CALL(boink::Logger::get_logger(), spdlog::level::warn, __VA_ARGS__)
#define BOINK_ERROR(...)    SPDLOG_LOGGER_CALL(boink::Logger::get_logger(), spdlog::level::err, __VA_ARGS__)
#define BOINK_CRITICAL(...) SPDLOG_LOGGER_CALL(boink::Logger::get_logger(), spdlog::level::critical, __VA_ARGS__)
