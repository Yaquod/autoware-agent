find_package(yaml-cpp REQUIRED)

add_library(map_routes_lib OBJECT
        RouteConfig.cc
        RouteConfig.h
)

install(FILES nishishinjuku_routes.yaml
        DESTINATION share/autoware_agent/maps)

target_compile_definitions(map_routes_lib PRIVATE
        DEFAULT_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}")

target_link_libraries(map_routes_lib
        PUBLIC yaml-cpp
        PUBLIC config_header
)

target_include_directories(map_routes_lib
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
        ${CMAKE_BINARY_DIR}/generated
        ${CMAKE_SOURCE_DIR}/src
)

target_compile_options(map_routes_lib PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
)