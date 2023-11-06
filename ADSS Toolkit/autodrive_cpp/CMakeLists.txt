cmake_minimum_required (VERSION 3.5)

project(autodrive)

add_definitions(-std=c++11)

find_package(OpenCV REQUIRED)
set(OpenCV_DIR "/usr/local/include/opencv4")
include_directories(${OpenCV_INCLUDE_DIRS})

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(LIBRARY_FILES src/json.hpp src/autodrive.hpp src/autodrive.cpp)
set(SOURCE_FILES_NIGEL src/example_nigel.cpp)
set(SOURCE_FILES_F1TENTH src/example_f1tenth.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_library(autodrive STATIC ${LIBRARY_FILES})

add_executable(example_nigel ${SOURCE_FILES_NIGEL})
add_executable(example_f1tenth ${SOURCE_FILES_F1TENTH})

target_link_libraries(example_nigel PUBLIC autodrive z ssl uv uWS ${OpenCV_LIBS})
target_link_libraries(example_f1tenth PUBLIC autodrive z ssl uv uWS ${OpenCV_LIBS})
