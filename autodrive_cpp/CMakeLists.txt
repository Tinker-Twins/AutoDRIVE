project(autodrive)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)
set(OpenCV_DIR "/usr/local/include/opencv4")
include_directories(${OpenCV_INCLUDE_DIRS})

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/autodrive.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")


add_executable(autodrive ${sources})


target_link_libraries(autodrive ${OpenCV_LIBS} z ssl uv uWS)
