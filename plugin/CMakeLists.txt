add_library(CnoidRosUtilsPlugin
  SHARED
  Plugin.cpp
  ClockPublisherItem.cpp
  PosePublisherItem.cpp
  )
target_include_directories(CnoidRosUtilsPlugin PUBLIC
  ${CNOID_INCLUDE_DIRS}
  )
target_link_libraries(CnoidRosUtilsPlugin PUBLIC
  CnoidBodyPlugin
  ${CNOID_LIBRARIES}
  ${catkin_LIBRARIES}
  )
set_target_properties(CnoidRosUtilsPlugin PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CNOID_PLUGIN_SUBDIR}
  )
