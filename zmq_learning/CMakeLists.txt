cmake_minimum_required(VERSION 3.0.2)

project(zmq_test)

add_compile_options(-std=c++11)

include_directories(
    include
    ${CMAKE_CURRENT_SOURCE_DIR}
)


add_executable(publisher src/publisher.cpp)

add_executable(subscriber src/subscriber.cpp)


target_link_libraries(publisher zmq)

target_link_libraries(subscriber zmq)

