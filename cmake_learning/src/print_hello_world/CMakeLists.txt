cmake_minimum_required(VERSION 3.0)

project(print_hello_world)

set(CMAKE_CXX_STANDARD 11)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# -----编译方案一------
# 如果源文件依赖的库文件没有，需要同时生成，则需要打开add_library()注释，
# 与add_executable() target_link_libraries()同时运行，注意target_link_libraries()链接的是
# 上面同时生成的库文件

# -----编译方案二------
# 先打开add_library(), 关闭add_executable() target_link_libraries()，
# cmake .. $$ make
# 再关闭add_library()，打开add_executable() target_link_libraries()。
# 原则就是：有库文件的话，直接target_link_libraries()，没有的话要先生成库文件，后target_link_libraries()
# 

# 如果源文件依赖的库文件没有，需要同时生成，则需要打开add_library()注释，
# 与add_executable() target_link_libraries()同时运行，注意target_link_libraries()链接的是上面同时生成的库文件

# # 生成动态库
add_library(${PROJECT_NAME} SHARED src/print_hello_world.cpp)
# # 生成静态库
add_library(${PROJECT_NAME}_static STATIC src/print_hello_world.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(hello_world src/print_hello_world.cpp)

target_link_libraries(hello_world ${PROJECT_SOURCE_DIR}/lib/lib${PROJECT_NAME}.so)

# 写法1
# 添加动态库目录
# LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)
# target_link_libraries(hello_world lib${PROJECT_NAME}.so)

#写法2（动态库）
# target_link_libraries(hello_world ${PROJECT_SOURCE_DIR}/lib/lib${PROJECT_NAME}.so)

# 写法3 （静态库）
# target_link_libraries(hello_world ${PROJECT_SOURCE_DIR}/lib/lib${PROJECT_NAME}_static.a)



# 教程1：生成静动态链接库
# https://blog.csdn.net/hp_cpp/article/details/110401496?spm=1001.2014.3001.5502
# 教程2：使用静动态链接库
# https://blog.csdn.net/hp_cpp/article/details/110404651?spm=1001.2014.3001.5502
