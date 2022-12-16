add_library(CnoidRosUtilsPlugin
  SHARED
  Plugin.cpp
  ClockPublisherItem.cpp
  PosePublisherItem.cpp
  )
target_include_directories(CnoidRosUtilsPlugin PUBLIC
  ${CNOID_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(CnoidRosUtilsPlugin PUBLIC
  CnoidBodyPlugin
  ${CNOID_LIBRARIES}
  ${catkin_LIBRARIES}
  )
set(CNOID_PLUGIN_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/plugin/lib)
set_target_properties(CnoidRosUtilsPlugin PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CNOID_PLUGIN_OUTPUT_DIR}
  )
catkin_add_env_hooks(99.cnoid_ros_utils
  SHELLS sh
  DIRECTORY ${PROJECT_SOURCE_DIR}/env-hooks
  )
