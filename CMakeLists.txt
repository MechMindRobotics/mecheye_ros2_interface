cmake_minimum_required(VERSION 3.5)
project (mecheye_ros_interface)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARDREQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/usr/include/jsoncpp")
set(CMAKE_VERBOSE_MAKEFILE on)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rosidl_default_generators REQUIRED)

## Find ament and any ament packages
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Set2DExposureTime.srv"
  "srv/Set2DROI.srv"
  "srv/Set2DSharpenFactor.srv"
  "srv/Set2DToneMappingEnable.srv"
  "srv/Set3DExposure.srv"
  "srv/Set3DGain.srv"
  "srv/Set3DROI.srv"
  "srv/SetCloudOutlierFilterMode.srv"
  "srv/SetCloudSmoothMode.srv"
  "srv/SetCurrentUserSet.srv"
  "srv/SetDepthRange.srv"
  "srv/SetFringeContrastThreshold.srv"
  "srv/SetFringeMinThreshold.srv"
  "srv/SetLaserSettings.srv"
  "srv/SaveAllSettingsToUserSets.srv"
  "srv/Set2DExpectedGrayValue.srv"
  "srv/Set2DExposureMode.srv"
  "srv/Set2DExposureSequence.srv"
  "srv/SetUhpSettings.srv"
  "srv/SetUhpCaptureMode.srv"
  "srv/SetUhpFringeCodingMode.srv"
  "srv/AddUserSet.srv"
  "srv/CaptureColorMap.srv"
  "srv/CaptureColorPointCloud.srv"
  "srv/CaptureDepthMap.srv"
  "srv/CapturePointCloud.srv"
  "srv/DeleteUserSet.srv"
  "srv/DeviceInfo.srv"
  "srv/Get2DExpectedGrayValue.srv"
  "srv/Get2DExposureMode.srv"
  "srv/Get2DExposureSequence.srv"
  "srv/Get2DExposureTime.srv"
  "srv/Get2DROI.srv"
  "srv/Get2DSharpenFactor.srv"
  "srv/Get2DToneMappingEnable.srv"
  "srv/Get3DExposure.srv"
  "srv/Get3DGain.srv"
  "srv/Get3DROI.srv"
  "srv/GetAllUserSets.srv"
  "srv/GetCloudOutlierFilterMode.srv"
  "srv/GetCloudSmoothMode.srv"
  "srv/GetCurrentUserSet.srv"
  "srv/GetDepthRange.srv"
  "srv/GetFringeContrastThreshold.srv"
  "srv/GetFringeMinThreshold.srv"
  "srv/GetLaserSettings.srv"
  "srv/GetUhpSettings.srv"
  "srv/GetUhpCaptureMode.srv"
  "srv/GetUhpFringeCodingMode.srv"

  DEPENDENCIES
  std_msgs
  sensor_msgs
)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${BOOST_INCLUDE_DIRS}
    /opt/mech-mind/mech-eye-sdk/include
)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

link_directories(/opt/mech-mind/mech-eye-sdk/lib)

add_executable(start src/main.cpp)
ament_target_dependencies(start
    cv_bridge
    image_transport
    rclcpp
    std_msgs
    sensor_msgs
    tf2
    rosidl_default_generators
)

# aux_source_directory(src SRC_LIST)
# add_executable(start ${SRC_LIST})
target_link_libraries(start
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${ZMQ_LIBRARIES}
    ${EIGEN_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    MechEyeApi
)

rosidl_target_interfaces(start ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS start
    DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(
    cv_bridge
    image_transport
    rclcpp
    std_msgs
    sensor_msgs
    tf2
    rosidl_default_generators
)
ament_package()
