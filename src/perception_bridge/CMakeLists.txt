find_package(rclcpp REQUIRED)
find_package(autoware_internal_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


add_library(perception_bridge
        src/PerceptionBridge.cc
        include/FrameStates.h
)

target_link_libraries(perception_bridge
        PUBLIC rclcpp::rclcpp
        PUBLIC ${autoware_internal_msgs_TARGETS}
        PUBLIC ${autoware_perception_msgs_TARGETS}
        PUBLIC ${sensor_msgs_TARGETS}
        PUBLIC ${nav_msgs_TARGETS}
        PUBLIC ${autoware_adapi_v1_msgs_TARGETS}
        PUBLIC ${PCL_LIBRARIES}
        PUBLIC ${pcl_conversions_LIBRARIES}


        PUBLIC map_routes_lib
        PUBLIC config_header
        PUBLIC fmt::fmt
        PUBLIC yaml-cpp
        PUBLIC gRPC::grpc++
        PUBLIC protobuf::libprotobuf
        PRIVATE Boost::system
)

add_dependencies(perception_bridge generate_proto_code)

target_include_directories(perception_bridge PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../map_routes
        ${CMAKE_BINARY_DIR}/generated

        ${PCL_INCLUDE_DIRS}
        ${pcl_conversions_INCLUDE_DIRS}
)

install(TARGETS perception_bridge
        DESTINATION lib/${PROJECT_NAME}
)