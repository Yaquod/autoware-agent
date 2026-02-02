cmake_minimum_required(VERSION 3.8)
project(gateway_bridge)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GRPC REQUIRED grpc++)
pkg_check_modules(Protobuf REQUIRED protobuf)

find_program(PROTOC protoc REQUIRED)
find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin REQUIRED)

set(PROTO_SRC_DIR ../../proto)
set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

set(PROTO_FILE ${PROTO_SRC_DIR}/vehicle_gateway.proto)
set(PROTO_SRCS ${PROTO_GEN_DIR}/vehicle_gateway.pb.cc)
set(PROTO_HDRS ${PROTO_GEN_DIR}/vehicle_gateway.pb.h)
set(GRPC_SRCS ${PROTO_GEN_DIR}/vehicle_gateway.grpc.pb.cc
        ../main.cc)
set(GRPC_HDRS ${PROTO_GEN_DIR}/vehicle_gateway.grpc.pb.h)

add_custom_command(
        OUTPUT ${PROTO_SRCS} ${PROTO_HDRS} ${GRPC_SRCS} ${GRPC_HDRS}
        COMMAND ${PROTOC}
        ARGS
        --grpc_out=${PROTO_GEN_DIR}
        --cpp_out=${PROTO_GEN_DIR}
        --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
        --experimental_allow_proto3_optional
        -I${PROTO_SRC_DIR}
        -I/usr/include
        ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}
        COMMENT "Generating gRPC and Protobuf code from vehicle_gateway.proto"
        VERBATIM
)

add_custom_target(generate_proto_code
        DEPENDS ${PROTO_SRCS} ${PROTO_HDRS} ${GRPC_SRCS} ${GRPC_HDRS}
)

add_executable(gateway_bridge_node
        src/gnss_gateway_bridge.cpp
        src/sensor_service.cpp
        src/vehicle_service.cpp
        src/trip_service.cpp
        ${PROTO_SRCS}
        ${GRPC_SRCS}
)

# Ensure proto generation happens before compilation
add_dependencies(gateway_bridge_node generate_proto_code)

target_include_directories(gateway_bridge_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${PROTO_GEN_DIR}
        ${GRPC_INCLUDE_DIRS}
        ${Protobuf_INCLUDE_DIRS}
)

target_link_libraries(gateway_bridge_node PRIVATE
        rclcpp::rclcpp
        ${sensor_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${GRPC_LIBRARIES}
        ${Protobuf_LIBRARIES}
        pthread
)

target_compile_definitions(gateway_bridge_node PRIVATE
        ${rclcpp_DEFINITIONS}
        ${sensor_msgs_DEFINITIONS}
        ${geometry_msgs_DEFINITIONS}
        ${nav_msgs_DEFINITIONS}
)

install(
        TARGETS gateway_bridge_node
        DESTINATION lib/${PROJECT_NAME}
)

install(
        DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        OPTIONAL
)

install(
        DIRECTORY include
        DESTINATION include
        OPTIONAL
)

ament_export_include_directories(include)
ament_export_dependencies(
        rclcpp
        sensor_msgs
        geometry_msgs
        nav_msgs
        std_msgs
)

ament_package()