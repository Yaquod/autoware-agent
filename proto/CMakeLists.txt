file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")

if(NOT PROTO_FILES)
    message(WARNING "No .proto files found in ${CMAKE_CURRENT_SOURCE_DIR}")
    add_library(proto_lib INTERFACE)
    target_link_libraries(proto_lib INTERFACE
            gRPC::grpc++
            protobuf::libprotobuf
    )
    return()
endif()

message(STATUS "Found ${CMAKE_CURRENT_LIST_LENGTH} .proto files")

add_library(proto_lib STATIC)

set(GENERATED_SOURCES)

foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
    get_filename_component(PROTO_ABS ${PROTO_FILE} ABSOLUTE)

    # Output files
    set(PROTO_SRC "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.cc")
    set(PROTO_HDR "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.pb.h")
    set(GRPC_SRC "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.grpc.pb.cc")
    set(GRPC_HDR "${CMAKE_CURRENT_BINARY_DIR}/${PROTO_NAME}.grpc.pb.h")

    # Generate protobuf and gRPC code
    add_custom_command(
            OUTPUT ${PROTO_SRC} ${PROTO_HDR} ${GRPC_SRC} ${GRPC_HDR}
            COMMAND protobuf::protoc
            ARGS
            --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
            --grpc_out=${CMAKE_CURRENT_BINARY_DIR}
            --plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
            -I${CMAKE_CURRENT_SOURCE_DIR}
            ${PROTO_ABS}
            DEPENDS ${PROTO_ABS}
            COMMENT "Generating C++ code from ${PROTO_NAME}.proto"
            VERBATIM
    )

    list(APPEND GENERATED_SOURCES ${PROTO_SRC} ${GRPC_SRC})
endforeach()

target_sources(proto_lib PRIVATE ${GENERATED_SOURCES})

target_include_directories(proto_lib
        PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(proto_lib
        PUBLIC
        gRPC::grpc++
        gRPC::grpc++_reflection
        protobuf::libprotobuf
)

set_target_properties(proto_lib PROPERTIES
        POSITION_INDEPENDENT_CODE ON
)