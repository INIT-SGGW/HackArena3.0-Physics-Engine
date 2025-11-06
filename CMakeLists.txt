cmake_minimum_required(VERSION 3.10)

project(BoinkEngine VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_compile_definitions(
  $<$<CONFIG:Release>:NDEBUG>
)

# Enable PIC for spdlog before adding it
set(SPDLOG_BUILD_PIC ON CACHE BOOL "Build spdlog with position independent code" FORCE)
add_subdirectory(third_party/spdlog)

add_subdirectory(third_party/eigen)

add_library(boink SHARED)

target_compile_options(boink PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -pedantic>
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
)

target_sources(boink
  PRIVATE
    src/test.cpp
)

target_include_directories(boink
  PUBLIC include
  PRIVATE src
)

target_link_libraries(boink
  PRIVATE eigen
  PUBLIC spdlog::spdlog
)

include(CTest)

set(TEST_SOURCES
  eigen_test
  spdlog_test
)

foreach(name IN LISTS TEST_SOURCES)
  add_executable(${name} tests/${name}.cpp)
  target_link_libraries(${name} PRIVATE boink)
  add_test(${name} COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${name})
endforeach()
