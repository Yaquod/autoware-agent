find_package(rclcpp REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_internal_debug_msgs REQUIRED)
find_package(autoware_internal_planning_msgs REQUIRED)
find_package(autoware_internal_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


add_library(planning_bridge
        src/PlanningBridge.cc
        include/FrameStates.h
)

target_link_libraries(planning_bridge
        PUBLIC rclcpp::rclcpp
        PUBLIC ${autoware_adapi_v1_msgs_TARGETS}
        PUBLIC ${autoware_planning_msgs_TARGETS}
        PUBLIC ${autoware_internal_debug_msgs_TARGETS}
        PUBLIC ${autoware_internal_planning_msgs_TARGETS}
        PUBLIC ${autoware_internal_msgs_TARGETS}
        PUBLIC map_routes_lib
        PUBLIC config_header
        PUBLIC fmt::fmt
        PUBLIC yaml-cpp
        PUBLIC proto_lib
        PRIVATE Boost::system
)

target_include_directories(planning_bridge PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../map_routes
        ${CMAKE_BINARY_DIR}/generated
)

install(TARGETS planning_bridge
        DESTINATION lib/${PROJECT_NAME}
)