cmake_minimum_required(VERSION 3.20)

if (POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
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
find_package(Boost REQUIRED COMPONENTS system)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GRPC REQUIRED grpc++ gpr)
pkg_check_modules(Protobuf REQUIRED protobuf)

if (NOT TARGET gRPC::grpc++)
    add_library(gRPC::grpc++ INTERFACE IMPORTED)
    target_link_libraries(gRPC::grpc++ INTERFACE ${GRPC_LIBRARIES})
    target_include_directories(gRPC::grpc++ INTERFACE ${GRPC_INCLUDE_DIRS})
    target_link_directories(gRPC::grpc++ INTERFACE ${GRPC_LIBRARY_DIRS})
endif ()

if (NOT TARGET protobuf::libprotobuf)
    add_library(protobuf::libprotobuf INTERFACE IMPORTED)
    target_link_libraries(protobuf::libprotobuf INTERFACE ${Protobuf_LIBRARIES})
    target_include_directories(protobuf::libprotobuf INTERFACE ${Protobuf_INCLUDE_DIRS})
    target_link_directories(protobuf::libprotobuf INTERFACE ${Protobuf_LIBRARY_DIRS})
endif ()

find_program(PROTOC NAMES protoc REQUIRED)
find_program(GRPC_CPP_PLUGIN NAMES grpc_cpp_plugin REQUIRED)

set(PROTO_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/proto)
set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

set(PROTO_FILES
        ${PROTO_SRC_DIR}/vehicle_gateway.proto
        ${PROTO_SRC_DIR}/vehicle_frame.proto
)

set(PROTO_SRCS ${PROTO_GEN_DIR}/vehicle_gateway.pb.cc)
set(PROTO_HDRS ${PROTO_GEN_DIR}/vehicle_gateway.pb.h)
set(GRPC_SRCS ${PROTO_GEN_DIR}/vehicle_gateway.grpc.pb.cc)
set(GRPC_HDRS ${PROTO_GEN_DIR}/vehicle_gateway.grpc.pb.h)

set(PROTO_SRCS ${PROTO_GEN_DIR}/vehicle_frame.pb.cc)
set(PROTO_HDRS ${PROTO_GEN_DIR}/vehicle_frame.pb.h)
set(GRPC_SRCS ${PROTO_GEN_DIR}/vehicle_frame.grpc.pb.cc)
set(GRPC_HDRS ${PROTO_GEN_DIR}/vehicle_frame.grpc.pb.h)


add_custom_command(
        OUTPUT ${PROTO_SRCS} ${PROTO_HDRS} ${GRPC_SRCS} ${GRPC_HDRS}
        COMMAND ${PROTOC}
        ARGS
        --grpc_out=${PROTO_GEN_DIR}
        --cpp_out=${PROTO_GEN_DIR}
        --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
        --experimental_allow_proto3_optional
        -I${PROTO_SRC_DIR}
        ${PROTO_FILES}
        DEPENDS ${PROTO_FILES}
        COMMENT "Generating gRPC + Protobuf stubs"
        VERBATIM
)
add_custom_target(generate_proto_code DEPENDS ${PROTO_SRCS} ${GRPC_SRCS})

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
#add planning
add_subdirectory(src/planning_bridge)
add_subdirectory(src/perception_bridge)

add_executable(autoware_agent
        src/main.cc
        ${PROTO_SRCS}
        ${GRPC_SRCS}
)

add_dependencies(autoware_agent generate_proto_code)

target_link_libraries(autoware_agent PRIVATE
        autoware_controller
        map_routes_lib
        cluster_bridge
        planning_bridge
        perception_bridge
        ${GRPC_LIBRARIES}
        ${Protobuf_LIBRARIES}
        Threads::Threads
)

target_include_directories(autoware_agent PRIVATE
        ${PROTO_GEN_DIR}
        ${GRPC_INCLUDE_DIRS}
        ${Protobuf_INCLUDE_DIRS}
)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    add_custom_command(TARGET autoware_agent POST_BUILD
            COMMAND ${CMAKE_STRIP} $<TARGET_FILE:autoware_agent>
            COMMENT "Stripping autoware_agent"
    )
endif ()

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(GTest REQUIRED)
    enable_testing()
    add_subdirectory(tests)
endif()

if (CMAKE_SYSTEM_NAME STREQUAL "Linux"
        AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/systemd/autoware-agent.service")
    install(FILES systemd/autoware-agent.service
            DESTINATION /lib/systemd/system)
endif ()

install(TARGETS autoware_agent
        RUNTIME DESTINATION bin)

message(STATUS "==========================================")
message(STATUS "Vehicle Autoware Agent")
message(STATUS "==========================================")
message(STATUS "Version:           ${PROJECT_VERSION}")
message(STATUS "Build type:        ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard:      ${CMAKE_CXX_STANDARD}")
message(STATUS "")
message(STATUS "Libraries:")
message(STATUS "  map_routes_lib      (RouteConfig)")
message(STATUS "  autoware_controller (AutowareController + TripController)")
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