cmake_minimum_required (VERSION 3.5)

project(autodrive)

add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)
set(OpenCV_DIR "/usr/local/include/opencv4")
include_directories(${OpenCV_INCLUDE_DIRS})

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(SOURCE_FILES src/example.cpp)
set(LIBRARY_FILES src/json.hpp src/autodrive.hpp src/autodrive.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(example ${SOURCE_FILES})
add_library(autodrive STATIC ${LIBRARY_FILES})

target_link_libraries(example PUBLIC autodrive z ssl uv uWS ${OpenCV_LIBS})
