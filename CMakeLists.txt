cmake_minimum_required(VERSION 3.20)

if (POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
endif ()

if (POLICY CMP0144)
    cmake_policy(SET CMP0144 OLD)
endif ()

project(vehicle_autoware_agent VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_INTEGRATION_TESTS "Build integration tests" OFF)
option(ENABLE_COVERAGE "Enable code coverage" OFF)
option(ENABLE_SANITIZERS "Enable sanitizers" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CompilerWarnings.cmake")
    include(CompilerWarnings)
endif ()
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/SubmoduleHelpers.cmake")
    include(SubmoduleHelpers)
endif ()
if (ENABLE_SANITIZERS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Sanitizers.cmake")
    include(Sanitizers)
    enable_sanitizers()
endif ()

find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fastcdr REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost 1.74 REQUIRED COMPONENTS system)
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenMP REQUIRED)
# find_package(protobuf CONFIG REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)

set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})
configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/src/Config.h.in"
        "${PROTO_GEN_DIR}/Config.h"
        @ONLY
)
add_library(config_header INTERFACE)
target_include_directories(config_header INTERFACE ${PROTO_GEN_DIR})

add_subdirectory(src/map_routes)
add_subdirectory(src/gateway_bridge)
add_subdirectory(src/cluster_bridge)
add_subdirectory(src/planning_bridge)
add_subdirectory(src/perception_bridge)
add_subdirectory(src/location_bridge)
add_subdirectory(src/trip_bridge)
add_subdirectory(proto)

add_executable(autoware_agent
        src/main.cc
        src/AutowareApp.cpp
        src/AutowareApp.h
        src/zenoh_publisher.h
)

target_link_libraries(autoware_agent
        autoware_controller
        map_routes_lib
        cluster_bridge
        planning_bridge
        perception_bridge
        trip_bridge
        location_bridge
        proto_lib
        zenohc::lib
        zenohcxx::zenohc
        Threads::Threads
)

ament_target_dependencies(autoware_agent rclcpp fastcdr yaml-cpp)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    add_custom_command(TARGET autoware_agent POST_BUILD
            COMMAND ${CMAKE_STRIP} $<TARGET_FILE:autoware_agent>
            COMMENT "Stripping autoware_agent"
    )
endif ()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(GTest REQUIRED)
    enable_testing()
    add_subdirectory(tests)
endif ()

if (CMAKE_SYSTEM_NAME STREQUAL "Linux"
        AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/systemd/autoware-agent.service")
    install(FILES systemd/autoware-agent.service
            DESTINATION /lib/systemd/system)
endif ()

install(TARGETS autoware_agent RUNTIME DESTINATION bin)
install(TARGETS autoware_agent DESTINATION lib/${PROJECT_NAME})

message(STATUS "==========================================")
message(STATUS "Vehicle Autoware Agent")
message(STATUS "==========================================")
message(STATUS "Version:           ${PROJECT_VERSION}")
message(STATUS "Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard:      ${CMAKE_CXX_STANDARD}")
message(STATUS "")
message(STATUS "Libraries:")
message(STATUS "  map_routes_lib      (RouteConfig)")
message(STATUS "  autoware_controller (AutowareController - TripController)")
message(STATUS "  autoware_controller (Cluster Bridge)")
message(STATUS "  autoware_controller (Trip Bridge)")
message(STATUS "  autoware_controller (Planning Bridge)")
message(STATUS "  autoware_controller (Perception Bridge)")
message(STATUS "  proto_lib           (Protobuf & gRPC)")
message(STATUS "")
if (BUILD_TESTS)
    message(STATUS "  Tests: GoogleTest")
endif ()
message(STATUS "")
message(STATUS "Build options:")
message(STATUS "  Tests:           ${BUILD_TESTS}")
message(STATUS "  Sanitizers:      ${ENABLE_SANITIZERS}")
message(STATUS "==========================================")

ament_package()