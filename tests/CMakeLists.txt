set(UNIT_TEST_SOURCES
        unit/routing_tests.cc
)

set(INTEGRATION_TEST_SOURCES
        integration/autoware_agent/autoware_agent_test.cc
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
    find_package(geometry_msgs REQUIRED)
    find_package(autoware_vehicle_msgs REQUIRED)
    find_package(autoware_adapi_v1_msgs REQUIRED)
    find_package(tier4_external_api_msgs REQUIRED)
    find_package(fmt REQUIRED)
    find_package(Boost REQUIRED COMPONENTS system)

    add_executable(autoware_integration_tests
            test_main.cpp
            ${INTEGRATION_TEST_SOURCES}
    )

    target_include_directories(autoware_integration_tests PRIVATE
            ${CMAKE_SOURCE_DIR}/src/gateway_bridge/include
            ${CMAKE_SOURCE_DIR}/src/map_routes
            ${CMAKE_BINARY_DIR}/generated
            ${CMAKE_SOURCE_DIR}/src
    )

    target_link_libraries(autoware_integration_tests PRIVATE
            autoware_controller
            map_routes_lib
            rclcpp::rclcpp
            ${geometry_msgs_TARGETS}
            ${autoware_vehicle_msgs_TARGETS}
            ${autoware_adapi_v1_msgs_TARGETS}
            ${tier4_external_api_msgs_TARGETS}
            fmt::fmt
            yaml-cpp
            Boost::system
            GTest::gtest
            GTest::gmock
            GTest::gtest_main
    )
    set_target_properties(autoware_integration_tests PROPERTIES
            BUILD_RPATH "/opt/ros/humble/lib;$ENV{HOME}/autoware/install/lib"
            INSTALL_RPATH "/opt/ros/humble/lib;$ENV{HOME}/autoware/install/lib"
    )
    add_test(NAME autoware_integration_tests COMMAND autoware_integration_tests)
endif ()