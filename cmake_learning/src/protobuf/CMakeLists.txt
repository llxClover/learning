cmake_minimum_required(VERSION 3.0)

project(protobuf_test)

find_package(Protobuf REQUIRED)

set(CMAKE_CXX_STANDARD 11)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_library(${PROJECT_NAME} SHARED src/proto_test.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include 
                    ${PROJECT_SOURCE_DIR}/proto
                    ${PROJECT_SOURCE_DIR}
                    ${PROJECT_SOURCE_DIR}/../print_hello_world/include
)

add_executable(proto_test src/proto_test.cpp proto/student.pb.cc)

target_link_libraries(proto_test 
                      ${PROJECT_NAME} 
                      protobuf::libprotobuf
)
