# CMake configuration
cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0148 OLD)

# Project name
project(autodrive_nigel)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  tf
  cmake_modules
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

# Install Python modules
catkin_python_setup()

# Catkin configuration
catkin_package(
 INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs nav_msgs tf
 DEPENDS # Eigen3
)

# Add CPP headers
include_directories(include)
include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Add CPP libraries
add_library(${PROJECT_NAME} src/lidar_odometry.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Add CPP executables
add_executable(adss src/adss.cpp)
add_executable(scss src/scss.cpp)
add_executable(lidar_odometer src/lidar_odometer.cpp)

# Link CPP targets
target_link_libraries(adss ${catkin_LIBRARIES})
target_link_libraries(scss ${catkin_LIBRARIES})
target_link_libraries(lidar_odometer ${PROJECT_NAME} ${catkin_LIBRARIES})

# Install Python executables
catkin_install_python(PROGRAMS
  scripts/server.py
  scripts/teleop.py
  scripts/nav_ctrl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Add other directories
install(DIRECTORY config launch maps rviz 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)