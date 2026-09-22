cmake_minimum_required(VERSION 3.8...3.28)

project(mecheye_ros_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE on)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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
  "srv/CaptureAll.srv"

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

set(mecheye_sdk_include_dir /opt/mech-mind/mech-eye-sdk/include)
set(mecheye_sdk_library_dir /opt/mech-mind/mech-eye-sdk/lib)

add_executable(start
  src/MechMindCamera.cpp
  src/main.cpp
)
target_include_directories(start
  PRIVATE
    include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${mecheye_sdk_include_dir}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
target_link_directories(start PRIVATE ${mecheye_sdk_library_dir})
ament_target_dependencies(start
  cv_bridge
  image_transport
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)

target_link_libraries(start
  Eigen3::Eigen
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  MechEyeApi
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(start ${cpp_typesupport_target})

install(TARGETS start
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(FILES
  launch/start_camera.py
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  cv_bridge
  image_transport
  pcl_conversions
  rclcpp
  rosidl_default_runtime
  sensor_msgs
  std_msgs
  tf2
)
ament_package()
