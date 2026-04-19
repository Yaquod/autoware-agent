find_package(rclcpp REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(tier4_system_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(tier4_external_api_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


add_library(trip_bridge
        src/TripBridge.cc
        include/FrameStates.h
)

target_link_libraries(trip_bridge
        PUBLIC rclcpp::rclcpp
        PUBLIC ${autoware_adapi_v1_msgs_TARGETS}
        PUBLIC ${tier4_system_msgs_TARGETS}

        PUBLIC ${autoware_planning_msgs_TARGETS}
        PUBLIC ${tier4_external_api_msgs_TARGETS}
        PUBLIC map_routes_lib
        PUBLIC config_header
        PUBLIC fmt::fmt
        PUBLIC yaml-cpp
        PUBLIC zenohc::lib
        PUBLIC zenohcxx::zenohc
        PUBLIC proto_lib
        PRIVATE Boost::system
)

target_include_directories(trip_bridge PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../map_routes
        ${CMAKE_BINARY_DIR}/generated
        ${CMAKE_CURRENT_SOURCE_DIR}/../gateway_bridge/include
)

install(TARGETS trip_bridge
        DESTINATION lib/${PROJECT_NAME}
)