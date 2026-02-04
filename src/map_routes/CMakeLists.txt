find_package(yaml-cpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

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
        PUBLIC ${GEOGRAPHICLIB_LIBRARIES}
)

target_include_directories(map_routes_lib
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
        ${CMAKE_BINARY_DIR}/generated
        ${CMAKE_SOURCE_DIR}/src
        ${GEOGRAPHICLIB_INCLUDE_DIRS}
)

target_compile_options(map_routes_lib PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
)