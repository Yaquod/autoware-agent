find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(tier4_external_api_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_library(autoware_controller
        src/AutowareController.cc
        src/TripController.cc
        src/VehicleGatewayClient.cc
        include/VehicleGatewayClient.h
        src/VehicleGatewayService.cc
        include/VehicleGatewayService.h
        include/Providers.h
        include/ClusterBridgeProvider.h
        include/AutowareControllerProvider.h
)

target_link_libraries(autoware_controller
        PUBLIC rclcpp::rclcpp
        PUBLIC tf2_ros::tf2_ros
        PUBLIC ${geometry_msgs_TARGETS}
        PUBLIC ${autoware_vehicle_msgs_TARGETS}
        PUBLIC ${autoware_adapi_v1_msgs_TARGETS}
        PUBLIC ${autoware_planning_msgs_TARGETS}
        PUBLIC ${tier4_external_api_msgs_TARGETS}
        PUBLIC map_routes_lib
        PUBLIC proto_lib
        PUBLIC config_header
        PUBLIC fmt::fmt
        PUBLIC yaml-cpp
        PRIVATE Boost::system
)

target_include_directories(autoware_controller PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../map_routes
)

install(TARGETS autoware_controller
        DESTINATION lib/${PROJECT_NAME}
)