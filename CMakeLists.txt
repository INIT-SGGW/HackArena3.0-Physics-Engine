cmake_minimum_required(VERSION 3.21)

project(BoinkEngine VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(LSAN_SUPPRESSIONS_FILE "${CMAKE_SOURCE_DIR}/lsan_suppress.supp")
set(LSAN_ENV_VAR "LSAN_OPTIONS=suppressions=${LSAN_SUPPRESSIONS_FILE}")

option(USE_ASAN_WIN "Enable asan on windows. If enabled SPDLOG will crash (TODO) and memory_leak wont work" OFF)

add_compile_definitions(
  $<$<CONFIG:Release>:NDEBUG>
)
add_compile_definitions($<$<CONFIG:Debug>:_DEBUG>)

# Enable PIC for spdlog before adding it
set(SPDLOG_BUILD_PIC ON CACHE BOOL "Build spdlog with position independent code" FORCE)
add_subdirectory(third_party/spdlog)

add_subdirectory(third_party/eigen)

add_subdirectory(third_party/piksel)

find_package(Threads REQUIRED)

#include(GenerateExportHeader) 
add_library(boink SHARED)
#generate_export_header(boink)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  if (MSVC)
    message(STATUS "Enabling Address Sanitizer for MSVC Debug build.")
    if(USE_ASAN_WIN)
    target_compile_options(boink PUBLIC /fsanitize=address)
    endif()
    target_compile_options(boink 
      PRIVATE
      /W4
      /WX
      /EHsc
    )
    if(USE_ASAN_WIN)
    target_link_options(boink PUBLIC /fsanitize=address)
    endif() 
  else()
    message(STATUS "Enabling Address/Undefined Sanitizer for GCC/Clang Debug build.")
    
    target_compile_options( boink PRIVATE
      -fsanitize=address,undefined,leak
      -fno-omit-frame-pointer
      PRIVATE
      -Wall
      -Werror
      -Wextra
      -pedantic
    )
    target_link_options(boink PUBLIC -fsanitize=address,undefined,leak)
  endif()
endif()

if(WIN32)
  # This files on Windows must be PUBLIC
  #(i dunno why and for now i dont care)
  set(BOINK_SOURCES_VISIBILITY PUBLIC)
  target_compile_definitions(boink PRIVATE BOINK_BUILD_DLL)
else()
  set(BOINK_SOURCES_VISIBILITY PRIVATE)
endif()


target_sources(boink
  PRIVATE
    src/boink_capi.cc
  ${BOINK_SOURCES_VISIBILITY}
    src/math.cc
    src/world.cc
)

target_include_directories(boink
  PUBLIC include
  PRIVATE src
  #PRIVATE ${CMAKE_BINARY_DIR} # for boink_export.h
)

target_link_libraries(boink
  PUBLIC Eigen3::Eigen
  PUBLIC spdlog::spdlog
  PUBLIC piksel
)

include(CTest)

set(TEST_SOURCES
  eigen_test
  spdlog_test
  world_test
  capi_test2
  mem_component_test
  type_diff_test
  asan_test
  memory_leak_test
  utils_math
  piksel_test
  present_test
)

foreach(name IN LISTS TEST_SOURCES)
  add_executable(${name} tests/${name}.cc)
  target_compile_definitions(${name} PRIVATE BOINK_USE_DLL)
  target_link_libraries(${name}
    PRIVATE 
      boink
      Threads::Threads # Required to resolve pthreads warning/errors on MSVC
  )
  add_test(NAME ${name} COMMAND ${name})
  set_tests_properties(${name} PROPERTIES
    ENVIRONMENT "${LSAN_ENV_VAR}")
endforeach()

set(TEST_SOURCES_C
  capi_test
  rotate_test
)
foreach(name IN LISTS TEST_SOURCES_C)
  add_executable(${name} tests/${name}.c)
  target_compile_definitions(${name} PRIVATE BOINK_USE_DLL)
  target_link_libraries(${name}
    PRIVATE 
      boink
      Threads::Threads # Required to resolve pthreads warning/errors on MSVC
  )
  add_test(NAME ${name} COMMAND ${name})
  set_tests_properties(${name} PROPERTIES
    ENVIRONMENT "${LSAN_ENV_VAR}")
endforeach()
