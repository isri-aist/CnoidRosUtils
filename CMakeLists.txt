cmake_minimum_required(VERSION 3.1)
project(cnoid_ros_utils)

# If CMAKE_CXX_STANDARD is not specified, the features specified in target_compile_features or INTERFACE_COMPILE_FEATURES
# may cause -std=gnu++11 to be added to the compile options and overwrite the C++ version specified in add_compile_options.
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  eigen_conversions
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  sensor_msgs
)

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# choreonoid
find_package(PkgConfig)
pkg_check_modules(CNOID REQUIRED choreonoid)
link_directories(${CNOID_LIBRARY_DIRS})
execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=plugindir choreonoid
  OUTPUT_VARIABLE CNOID_PLUGIN_SUBDIR
  OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=sharedir choreonoid
  OUTPUT_VARIABLE CNOID_SHARE_SUBDIR
  OUTPUT_STRIP_TRAILING_WHITESPACE)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  tf2_ros
  eigen_conversions
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  sensor_msgs
  DEPENDS EIGEN3
  )

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(plugin)
