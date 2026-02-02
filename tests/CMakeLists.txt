file(GLOB_RECURSE UNIT_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/unit/*_test.cpp")

add_executable(autoware_tests
        test_main.cpp
        ${UNIT_TEST_SOURCES}
        unit/routing_tests.cc
)

target_include_directories(autoware_tests
        PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/tests/mocks
)

target_link_libraries(autoware_tests
        PRIVATE
        map_routes_lib
        yaml-cpp
        GTest::gtest
        GTest::gmock
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(autoware_tests
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        # Apply the environment variable to all discovered tests here
        PROPERTIES
        ENVIRONMENT "VEHICLE_AGENT_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/unit"
        TIMEOUT 60
)

add_test(NAME routing_unit_test COMMAND autoware_tests)

set_tests_properties(routing_unit_test PROPERTIES ENVIRONMENT
        "VEHICLE_AGENT_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/unit")

if (BUILD_INTEGRATION_TESTS)
    file(GLOB_RECURSE INTEGRATION_TEST_SOURCES "integration/**/*_test.cpp")

    add_executable(autoware_integration_tests
            test_main.cpp
            ${INTEGRATION_TEST_SOURCES}
    )

    target_include_directories(autoware_integration_tests
            PRIVATE
            ${CMAKE_SOURCE_DIR}/src
    )

    target_link_libraries(autoware_integration_tests
            PRIVATE
            map_routes_lib
            GTest::gtest
            GTest::gmock
            GTest::gtest_main
    )

    gtest_discover_tests(autoware_integration_tests
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
endif ()