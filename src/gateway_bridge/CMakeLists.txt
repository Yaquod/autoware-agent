cmake_minimum_required(VERSION 3.10)
project(gateway_bridge)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GRPC REQUIRED grpc++)
pkg_check_modules(Protobuf REQUIRED protobuf)

find_program(PROTOC protoc REQUIRED)
find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin REQUIRED)

set(PROTO_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../proto)
set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

set(PROTO_FILE  ${PROTO_SRC_DIR}/vehicle_gateway.proto)
set(PROTO_SRCS  ${PROTO_GEN_DIR}/vehicle_gateway.pb.cc)
set(PROTO_HDRS  ${PROTO_GEN_DIR}/vehicle_gateway.pb.h)
set(GRPC_SRCS   ${PROTO_GEN_DIR}/vehicle_gateway.grpc.pb.cc)
set(GRPC_HDRS   ${PROTO_GEN_DIR}/vehicle_gateway.grpc.pb.h)

add_custom_command(
        OUTPUT ${PROTO_SRCS} ${PROTO_HDRS} ${GRPC_SRCS} ${GRPC_HDRS}
        COMMAND ${PROTOC}
        ARGS
        --grpc_out=${PROTO_GEN_DIR}
        --cpp_out=${PROTO_GEN_DIR}
        --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
        --experimental_allow_proto3_optional
        -I${PROTO_SRC_DIR}
        ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}
        COMMENT "Generating gRPC and Protobuf code"
        VERBATIM
)

add_custom_target(generate_proto_code DEPENDS ${PROTO_SRCS} ${GRPC_SRCS})

add_library(autoware_controller
        src/AutowareController.cc
)

ament_target_dependencies(autoware_controller
        rclcpp
        geometry_msgs
        autoware_auto_vehicle_msgs
        autoware_adapi_v1_msgs
)

target_include_directories(autoware_controller PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(gateway_bridge_node
        ../main.cc
        ${PROTO_SRCS}
        ${GRPC_SRCS}
)

target_link_libraries(gateway_bridge_node PRIVATE
        autoware_controller
        rclcpp::rclcpp
        ${sensor_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${autoware_auto_vehicle_msgs_TARGETS}
        ${GRPC_LIBRARIES}
        ${Protobuf_LIBRARIES}
        pthread
)

target_include_directories(gateway_bridge_node PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../map_routes
        ${PROTO_GEN_DIR}
        ${GRPC_INCLUDE_DIRS}
        ${Protobuf_INCLUDE_DIRS}
)

add_dependencies(gateway_bridge_node generate_proto_code)

install(TARGETS gateway_bridge_node autoware_controller
        DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
        rclcpp
        autoware_auto_vehicle_msgs
        autoware_adapi_v1_msgs
)

ament_package()