add_library(CnoidRosUtilsPlugin SHARED Plugin.cpp ClockPublisherItem.cpp
                                       PosePublisherItem.cpp)

target_link_libraries(CnoidRosUtilsPlugin PUBLIC ROSDependencies
                                                 Choreonoid::CnoidBodyPlugin)

install(TARGETS CnoidRosUtilsPlugin DESTINATION ${CNOID_PLUGIN_SUBDIR})

configure_file("${PROJECT_SOURCE_DIR}/env-hooks/99.cnoid_ros_utils.sh.in"
               ${PROJECT_BINARY_DIR}/env-hooks/99.cnoid_ros_utils.sh @ONLY)
ament_environment_hooks("${PROJECT_BINARY_DIR}/env-hooks/99.cnoid_ros_utils.sh")
