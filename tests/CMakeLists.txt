set(UNIT_TEST_SOURCES
        unit/routing_tests.cc
)

set(INTEGRATION_TEST_SOURCES
        integration/autoware_agent/autoware_agent_test.cc
        integration/autoware_agent/full_trip_test.cc
)

find_package(rclcpp REQUIRED)
find_package(GTest REQUIRED)
find_package(tier4_external_api_msgs QUIET)
find_package(fmt QUIET)

# Unit Tests
add_executable(autoware_tests
        test_main.cpp
        ${UNIT_TEST_SOURCES}
)

target_include_directories(autoware_tests PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/tests/mocks
        ${CMAKE_SOURCE_DIR}/src/gateway_bridge/include
        ${CMAKE_SOURCE_DIR}/src/map_routes
)

target_link_libraries(autoware_tests PRIVATE
        map_routes_lib
        rclcpp::rclcpp
        GTest::gtest
        GTest::gmock
        GTest::gtest_main
        yaml-cpp
)

if (fmt_FOUND)
    target_link_libraries(autoware_tests PRIVATE fmt::fmt)
endif ()

if (tier4_external_api_msgs_FOUND)
    target_link_libraries(autoware_tests PRIVATE
            ${tier4_external_api_msgs_TARGETS}
    )
endif ()

add_test(NAME autoware_tests COMMAND autoware_tests)

# Integration Tests  (live test against running Autoware instance)
if (BUILD_INTEGRATION_TESTS)
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
    find_package(Boost REQUIRED COMPONENTS system)

    add_executable(autoware_integration_tests
            test_main.cpp
            ${INTEGRATION_TEST_SOURCES}
            ${PROTO_SRCS}
            ${GRPC_SRCS}
    )

    target_include_directories(autoware_integration_tests PRIVATE
            ${CMAKE_SOURCE_DIR}/src/gateway_bridge/include
            ${CMAKE_SOURCE_DIR}/src/map_routes
            ${CMAKE_SOURCE_DIR}/src/cluster_bridge/include
            ${CMAKE_BINARY_DIR}/generated
            ${CMAKE_SOURCE_DIR}/src
            ${GRPC_INCLUDE_DIRS}
            ${Protobuf_INCLUDE_DIRS}
    )

    target_link_libraries(autoware_integration_tests PRIVATE
            autoware_controller
            map_routes_lib
            cluster_bridge
            rclcpp::rclcpp
            gRPC::grpc++
            protobuf::libprotobuf
            ${GRPC_LIBRARIES}
            ${Protobuf_LIBRARIES}
            ${geometry_msgs_TARGETS}
            ${autoware_vehicle_msgs_TARGETS}
            ${autoware_adapi_v1_msgs_TARGETS}
            ${autoware_planning_msgs_TARGETS}
            ${tier4_external_api_msgs_TARGETS}
            ${autoware_perception_msgs_TARGETS}
            ${autoware_internal_debug_msgs_TARGETS}
            ${autoware_internal_planning_msgs_TARGETS}
            ${autoware_internal_msgs_TARGETS}
            ${autoware_auto_planning_msgs_TARGETS}
            ${nav_msgs_TARGETS}
            ${tier4_vehicle_msgs_TARGETS}
            fmt::fmt
            yaml-cpp
            Boost::system
            GTest::gtest
            GTest::gmock
            GTest::gtest_main
    )

    add_dependencies(autoware_integration_tests generate_proto_code)

    set_target_properties(autoware_integration_tests PROPERTIES
            BUILD_RPATH "/opt/ros/humble/lib;$ENV{HOME}/autoware/install/lib"
            INSTALL_RPATH "/opt/ros/humble/lib;$ENV{HOME}/autoware/install/lib"
    )
    add_test(NAME autoware_integration_tests COMMAND autoware_integration_tests)
endif ()