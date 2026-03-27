cmake_minimum_required(VERSION 3.21)
if(WIN32)
    set(VCPKG_TARGET_TRIPLET "x64-windows-static-md" CACHE STRING "" FORCE)
endif()

project(BoinkEngine VERSION 0.17.8)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/boink/version.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/boink/version.h
)

set(LSAN_SUPPRESSIONS_FILE "${CMAKE_CURRENT_SOURCE_DIR}/lsan_suppress.supp")
set(LSAN_ENV_VAR "LSAN_OPTIONS=suppressions=${LSAN_SUPPRESSIONS_FILE}")

option(USE_ASAN_WIN 
  "Enable asan on windows. 
  If enabled SPDLOG will crash (TODO) and memory_leak wont work" OFF)

add_compile_definitions(
  $<$<CONFIG:Release>:NDEBUG>
)
add_compile_definitions($<$<CONFIG:Debug>:_DEBUG>)

add_library(boink SHARED)

set_target_properties(boink PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)

find_package(Threads REQUIRED)

find_package(Bullet CONFIG REQUIRED)
target_link_libraries(boink PUBLIC ${BULLET_LIBRARIES})

find_package(piksel CONFIG REQUIRED)                                                                                                                                          
target_link_libraries(boink PUBLIC piksel::piksel)
#target_compile_definitions(boink PRIVATE GLFW_STATIC)

find_package(spdlog CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)

target_link_libraries(boink PRIVATE 
    spdlog::spdlog_header_only
    fmt::fmt-header-only
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  if (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    if(USE_ASAN_WIN)
    message(STATUS "Enabling Address Sanitizer for MSVC Debug build.")
    target_compile_options(boink PUBLIC /fsanitize=address)
    endif()
    target_compile_options(boink 
      PRIVATE
      /W4
      #/WX
      /EHsc
    )
    if(USE_ASAN_WIN)
    target_link_options(boink PUBLIC /fsanitize=address)
    endif() 
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
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
  set(BOINK_SOURCES_VISIBILITY PRIVATE)
  target_compile_definitions(boink PRIVATE BOINK_BUILD_DLL)
else()
  set(BOINK_SOURCES_VISIBILITY PRIVATE)
endif()

target_sources(boink
  PRIVATE
    src/boink_c_api.cc
  ${BOINK_SOURCES_VISIBILITY}
    src/simulation.cc
    src/utility.cc
    src/track.cc
    src/tinygltf_impl.cc
    src/vehicle.cc
    src/vehicle_mesh.cc
    src/gltf_extractor.cc
    src/exception.cc
    src/ground.cc
    src/line.cc
    src/race.cc
    src/debugger.cc
    src/renderer.cc
    src/simulation_gui.cc
    src/debugger_gui.cc
    src/race_gui.cc
    src/track_gui.cc
    src/vehicle_gui.cc
    src/camera_controller.cc
    src/vehicle_contoller.cc
    src/weather.cc
    src/weather_gui.cc
    src/smoothed_value.cc
    src/surface_info_gui.cc
    src/vehicle_raycaster.cc
    src/raycast_vehicle.cc
    src/wheel_info.cc
    src/ghost_mode.cc
    src/ghost_gui.cc
    src/constants.cc
    src/logger.cc
    src/pitstop.cc
    src/road.cc
)

target_include_directories(boink
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

#if (CMAKE_BUILD_TYPE STREQUAL "Debug")
include(CTest)

set(TEST_SOURCES_C
  capi_test
  capi_test2
  capi_without_debugger_test
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
#endif()

# Include the module that defines standard installation directory variables
include(GNUInstallDirs)

# 1. Install the 'boink' shared library
# LIBRARY: for the shared library file (.so, .dylib)
# ARCHIVE: for the import library file on Windows/macOS (.lib, .a)
# RUNTIME: for the DLL file on Windows (.dll)
install(TARGETS boink
    EXPORT BoinkEngineTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# 2. Install the public headers
# We assume your public headers are under 'include/boink' and need to be installed to 'include/boink'.
# The $<BUILD_INTERFACE:...> in target_include_directories is a good practice for interface setup.
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/boink
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
)

# 3. Install the generated version header
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/generated/boink/version.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/boink
)

# 4. Export the targets for use by other CMake projects
# This generates a file (e.g., BoinkEngineTargets.cmake) that other projects can use with find_package().
install(EXPORT BoinkEngineTargets
    FILE BoinkEngineTargets.cmake
    NAMESPACE BoinkEngine::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/BoinkEngine
)

# 5. (Optional) Install a CMake configuration file
# This is a common practice to make the package discoverable with find_package(BoinkEngine).

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/BoinkEngineConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/BoinkEngineConfig.cmake
    @ONLY
)

# Install the configuration files
# This is where the magic happens: the files are installed to the standard location
# where find_package() will look (e.g., lib/cmake/BoinkEngine).
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/BoinkEngineConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/BoinkEngine
)
