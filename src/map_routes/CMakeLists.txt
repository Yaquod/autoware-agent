find_package(lanelet2_core       REQUIRED)
find_package(lanelet2_io         REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(GeographicLib       REQUIRED)
find_package(yaml-cpp            REQUIRED)
find_package(autoware_lanelet2_extension REQUIRED) 
#added
find_package(lanelet2_routing REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED geographiclib)

add_library(map_routes_lib OBJECT
        LaneletMap.cc
        LaneletMap.h
        MapProjectorInfo.h
        MapProjectorInfo.cc
)

install(FILES lanelet2_map.osm map_config.yaml
        DESTINATION share/autoware_agent/maps)

target_compile_definitions(map_routes_lib PRIVATE
        DEFAULT_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}")

target_link_libraries(map_routes_lib
        PUBLIC yaml-cpp
        PUBLIC config_header
        PUBLIC ${GEOGRAPHICLIB_LIBRARIES}
        lanelet2_core::lanelet2_core
        lanelet2_io::lanelet2_io
        lanelet2_projection::lanelet2_projection
        #addded
        lanelet2_routing::lanelet2_routing 
        ${autoware_lanelet2_extension_LIBRARIES} 
        PUBLIC ${TARGETS}
)

target_include_directories(map_routes_lib
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
        ${CMAKE_BINARY_DIR}/generated
        ${CMAKE_SOURCE_DIR}/src
        ${GEOGRAPHICLIB_INCLUDE_DIRS}
         ${autoware_lanelet2_extension_INCLUDE_DIRS}
)

target_compile_options(map_routes_lib PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra>
)