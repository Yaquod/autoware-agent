find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(tier4_external_api_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(autoware_internal_debug_msgs REQUIRED)
find_package(autoware_internal_msgs REQUIRED)
find_package(autoware_internal_planning_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


add_library(cluster_bridge
        src/ClusterBridge.cc
        include/FrameStates.h
)

target_link_libraries(cluster_bridge
        PUBLIC rclcpp::rclcpp
        PUBLIC ${geometry_msgs_TARGETS}
        PUBLIC ${autoware_vehicle_msgs_TARGETS}
        PUBLIC ${autoware_adapi_v1_msgs_TARGETS}
        PUBLIC ${autoware_planning_msgs_TARGETS}
        PUBLIC ${tier4_external_api_msgs_TARGETS}
        PUBLIC ${autoware_perception_msgs_TARGETS}
        PUBLIC ${autoware_internal_debug_msgs_TARGETS}
        PUBLIC ${autoware_internal_planning_msgs_TARGETS}
        PUBLIC ${autoware_internal_msgs_TARGETS}
        PUBLIC ${autoware_auto_planning_msgs_TARGETS}
        PUBLIC ${nav_msgs_TARGETS}
        PUBLIC ${tier4_vehicle_msgs_TARGETS}
        PUBLIC map_routes_lib
        PUBLIC config_header
        PUBLIC fmt::fmt
        PUBLIC yaml-cpp
        PUBLIC gRPC::grpc++
        PUBLIC protobuf::libprotobuf
        PRIVATE Boost::system
)

target_include_directories(cluster_bridge PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../map_routes
        ${CMAKE_BINARY_DIR}/generated
)

install(TARGETS cluster_bridge
        DESTINATION lib/${PROJECT_NAME}
)