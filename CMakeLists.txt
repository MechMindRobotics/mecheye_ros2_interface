cmake_minimum_required(VERSION 3.5)
project (mecheye_ros_interface)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARDREQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE on)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pcl_conversions REQUIRED)

## Find ament and any ament packages
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetCurrentUserSet.srv"
  "srv/SaveAllSettingsToUserSets.srv"
  "srv/AddUserSet.srv"

  "srv/CaptureColorImage.srv"
  "srv/CaptureStereoColorImages.srv"
  "srv/CaptureTexturedPointCloud.srv"
  "srv/CaptureDepthMap.srv"
  "srv/CapturePointCloud.srv"

  "srv/DeleteUserSet.srv"
  "srv/DeviceInfo.srv"
  "srv/GetAllUserSets.srv"
  "srv/GetCurrentUserSet.srv"

  "srv/SetIntParameter.srv"
  "srv/GetIntParameter.srv"
  "srv/SetRangeParameter.srv"
  "srv/GetRangeParameter.srv"
  "srv/SetBoolParameter.srv"
  "srv/GetBoolParameter.srv"
  "srv/SetEnumParameter.srv"
  "srv/GetEnumParameter.srv"
  "srv/SetFloatParameter.srv"
  "srv/GetFloatParameter.srv"
  "srv/SetFloatArrayParameter.srv"
  "srv/GetFloatArrayParameter.srv"
  "srv/SetROIParameter.srv"
  "srv/GetROIParameter.srv"

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

aux_source_directory(src SRC_LIST)
add_executable(start ${SRC_LIST})
ament_target_dependencies(start
    cv_bridge
    image_transport
    rclcpp
    std_msgs
    sensor_msgs
    tf2
    rosidl_default_generators
    pcl_conversions
)

# aux_source_directory(src SRC_LIST)
# add_executable(start ${SRC_LIST})
target_link_libraries(start
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${EIGEN_LIBRARIES}
    ${BOOST_LIBRARIES}
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
