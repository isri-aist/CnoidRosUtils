link_directories(${CNOID_LIBRARY_DIRS})

add_library(CnoidRosUtilsPlugin
  SHARED
  Plugin.cpp
  ClockPublisherItem.cpp
  PosePublisherItem.cpp
  )
target_link_libraries(CnoidRosUtilsPlugin PUBLIC ROSDependencies PkgConfig::CNOID CnoidBodyPlugin)

if(DEFINED CATKIN_DEVEL_PREFIX)
  set(CNOID_PLUGIN_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/plugin/lib)
  set_target_properties(CnoidRosUtilsPlugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CNOID_PLUGIN_OUTPUT_DIR}
    )
  catkin_add_env_hooks(99.cnoid_ros_utils
    SHELLS sh
    DIRECTORY ${PROJECT_SOURCE_DIR}/env-hooks
    )
endif()

install(TARGETS CnoidRosUtilsPlugin DESTINATION ${CNOID_PLUGIN_SUBDIR})
