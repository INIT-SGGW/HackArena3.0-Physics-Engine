cmake_minimum_required(VERSION 3.21)

project(BoinkEngine VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 20)
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

find_package(Threads REQUIRED)

include(GenerateExportHeader)
add_library(boink SHARED)
generate_export_header(boink)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    if (MSVC)
        message(STATUS "Enabling Address Sanitizer for MSVC Debug build.")
        
        target_compile_options(boink PRIVATE
          /fsanitize=address
          /W4
          /WX
        )
        target_link_options(boink PRIVATE /fsanitize=address)
    else()
        message(STATUS "Enabling Address/Undefined Sanitizer for GCC/Clang Debug build.")
        
        target_compile_options( boink PRIVATE
            -fsanitize=address,undefined
            -fno-omit-frame-pointer
            -Wall
            -Werror
            -Wextra
            -pedantic
        )
        target_link_options(boink PRIVATE -fsanitize=address,undefined)
    endif()
endif()

target_sources(boink
  PRIVATE
    src/boink.cc
    src/boink_capi.cc
  PUBLIC
    src/math.cc
    src/world.cc
)

target_include_directories(boink
  PUBLIC include
  PRIVATE src
  PRIVATE ${CMAKE_BINARY_DIR} # for boink_export.h
)

target_link_libraries(boink
  PUBLIC Eigen3::Eigen
  PUBLIC spdlog::spdlog
)

include(CTest)

set(TEST_SOURCES
  #eigen_test
  #spdlog_test
  world_test
)

foreach(name IN LISTS TEST_SOURCES)
  add_executable(${name} tests/${name}.cc)
  target_link_libraries(${name}
    PRIVATE 
      boink
      Threads::Threads # Required to resolve pthreads warning/errors on MSVC
  )
  add_test(${name} COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${name})
endforeach()
