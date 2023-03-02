# 3.6 required for using IMPORTED_TARGET with pkg-config module
cmake_minimum_required(VERSION 3.6)
project(cnoid_ros_utils VERSION 0.1.0)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# If CMAKE_CXX_STANDARD is not specified, the features specified in target_compile_features or INTERFACE_COMPILE_FEATURES
# may cause -std=gnu++11 to be added to the compile options and overwrite the C++ version specified in add_compile_options.
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)

if(DEFINED CATKIN_DEVEL_PREFIX)
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
endif()

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# choreonoid
find_package(PkgConfig)
pkg_check_modules(CNOID REQUIRED choreonoid IMPORTED_TARGET)
execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=plugindir choreonoid
  OUTPUT_VARIABLE CNOID_PLUGIN_SUBDIR
  OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=sharedir choreonoid
  OUTPUT_VARIABLE CNOID_SHARE_SUBDIR
  OUTPUT_STRIP_TRAILING_WHITESPACE)

if(DEFINED CATKIN_DEVEL_PREFIX)
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
  link_directories(${catkin_LIBRARY_DIRS})
  add_library(ROSDependencies INTERFACE)
  target_include_directories(ROSDependencies INTERFACE ${catkin_INCLUDE_DIRS})
  target_link_libraries(ROSDependencies INTERFACE ${catkin_LIBRARIES})
else()
  add_library(ROSDependencies INTERFACE)
  function(add_ros_dependency NAME)
    pkg_check_modules(${NAME} REQUIRED ${NAME} IMPORTED_TARGET)
    target_link_libraries(ROSDependencies INTERFACE PkgConfig::${NAME})
  endfunction()
  function(add_ros_dependencies)
    foreach(NAME ${ARGN})
      add_ros_dependency(${NAME})
    endforeach()
  endfunction()
  add_ros_dependencies(
    roscpp
    tf2_ros
    eigen_conversions
    std_msgs
    std_srvs
    geometry_msgs
    visualization_msgs
    sensor_msgs
  )
endif()

add_subdirectory(plugin)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)
configure_package_config_file(cmake/CnoidRosUtilsConfig.in.cmake ${CMAKE_CURRENT_BINARY_DIR}/cmake/CnoidRosUtilsConfig.cmake INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CnoidRosUtils/)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/cmake/CnoidRosUtilsConfigVersion.cmake VERSION ${PROJECT_VERSION} COMPATIBILITY AnyNewerVersion ARCH_INDEPENDENT)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/CnoidRosUtilsConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/cmake/CnoidRosUtilsConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CnoidRosUtils
)

OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)
if(INSTALL_DOCUMENTATION)
  add_subdirectory(doc)
endif()
