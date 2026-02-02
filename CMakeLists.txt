cmake_minimum_required(VERSION 3.20)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
endif()

project(VehicleAutowareAgent VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_INTEGRATION_TESTS "Build integration tests" OFF)
option(ENABLE_COVERAGE "Enable code coverage" OFF)
option(ENABLE_SANITIZERS "Enable address/thread sanitizers" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CompilerWarnings.cmake")
    include(CompilerWarnings)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/SubmoduleHelpers.cmake")
    include(SubmoduleHelpers)
endif()

if(ENABLE_SANITIZERS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Sanitizers.cmake")
    include(Sanitizers)
    enable_sanitizers()
endif()

message(STATUS "Finding system dependencies...")

# Threading
find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GRPC REQUIRED grpc++ gpr)

if(NOT TARGET gRPC::grpc)
    add_library(gRPC::grpc INTERFACE IMPORTED)
    target_link_libraries(gRPC::grpc INTERFACE ${GRPC_LIBRARIES})
    target_include_directories(gRPC::grpc INTERFACE ${GRPC_INCLUDE_DIRS})
endif()

find_program(gRPC_CPP_PLUGIN_PATH NAMES grpc_cpp_plugin)

if(gRPC_CPP_PLUGIN_PATH AND NOT TARGET gRPC::grpc_cpp_plugin)
    add_executable(gRPC::grpc_cpp_plugin IMPORTED)
    set_target_properties(gRPC::grpc_cpp_plugin PROPERTIES
            IMPORTED_LOCATION "${gRPC_CPP_PLUGIN_PATH}")
endif()

find_package(Protobuf REQUIRED)
if(NOT TARGET protobuf::libprotobuf)
    add_library(protobuf::libprotobuf ALIAS Protobuf::libprotobuf)
endif()

#add_subdirectory(src/gateway_bridge)

add_subdirectory(src/map_routes)

add_executable(autoware_agent src/main.cc)
target_link_libraries(autoware_agent PRIVATE autoware_lib)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_custom_command(TARGET autoware_agent POST_BUILD
            COMMAND ${CMAKE_STRIP} $<TARGET_FILE:autoware_agent>
            COMMENT "Stripping autoware_agent binary"
    )
endif()

# Root CMakeLists.txt
configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/Config.h.in"
        "${CMAKE_CURRENT_BINARY_DIR}/generated/Config.h"
        @ONLY
)

include_directories("${CMAKE_CURRENT_BINARY_DIR}/generated")

# Tests
if(BUILD_TESTS)
    find_package(GTest REQUIRED)
endif()
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

#install(TARGETS Autoware_Agent
#        RUNTIME DESTINATION bin)

# Install systemd service file only on Linux.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/systemd/autoware-agent.service")
    install(FILES systemd/autoware-agent.service
            DESTINATION /lib/systemd/system)
endif()

message(STATUS "==========================================")
message(STATUS "Vehicle Autoware Agent")
message(STATUS "==========================================")
message(STATUS "Version:           ${PROJECT_VERSION}")
message(STATUS "Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard:      ${CMAKE_CXX_STANDARD}")
message(STATUS "")
if(BUILD_TESTS)
    message(STATUS "  GoogleTest")
endif()
message(STATUS "")
message(STATUS "Build options:")
message(STATUS "  Tests:           ${BUILD_TESTS}")
message(STATUS "  Sanitizers:      ${ENABLE_SANITIZERS}")
message(STATUS "==========================================")