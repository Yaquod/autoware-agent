find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(autoware_vehicle_msgs REQUIRED)
# find_package(tier4_external_api_msgs REQUIRED)
# find_package(autoware_perception_msgs REQUIRED)
# find_package(autoware_internal_debug_msgs REQUIRED)
# find_package(autoware_internal_msgs REQUIRED)
# find_package(autoware_internal_planning_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(tier4_vehicle_msgs REQUIRED)
# find_package(autoware_adapi_v1_msgs REQUIRED)
# find_package(autoware_planning_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


add_library(location_bridge
        src/LocationBridge.cc
        include/FrameStates.h
)

target_link_libraries(location_bridge
        PUBLIC rclcpp::rclcpp
    
        PUBLIC ${nav_msgs_TARGETS}
        PUBLIC map_routes_lib
        PUBLIC config_header
        PUBLIC fmt::fmt
        PUBLIC yaml-cpp
        PUBLIC zenohc::lib
        PUBLIC zenohcxx::zenohc
        PUBLIC proto_lib
        PRIVATE Boost::system
)

target_include_directories(location_bridge PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../map_routes
        ${CMAKE_BINARY_DIR}/generated
)

install(TARGETS location_bridge
        DESTINATION lib/${PROJECT_NAME}
)