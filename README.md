# ROS 2 Interface for Mech-Eye Industrial 3D Camera

This documentation provides instructions on using the ROS 2 interface for Mech-Eye Industrial 3D Camera.

If you have any questions or have anything to share, feel free to post on the [Mech-Mind Online Community](https://community.mech-mind.com/). The community also contains a [specific category for development with Mech-Eye SDK](https://community.mech-mind.com/c/mech-eye-sdk-development/19).

## Prerequisites

In order to use the ROS 1 interface, the following prerequisites must be satisfied:

* Ubuntu version: 22.04 recommended
* ROS version: [ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) has been tested to work with this interface.

  >Note: If you have multiple versions of ROS installed, make sure to set the `ROS_DISTRO` environment variable to the version you are about to use.
  >
  >* Execute the following command to check ROS's environment variables:
  >
  >  ```bash
  >  printenv | grep ROS
  >  ```
  >
  >* Execute the following command to change the value of the `ROS_DISTRO` environment variable. Replace `distro` with the short name of the ROS version that you use.
  >
  >  ```bash
  >  export ROS_DISTRO=distro
  >  ```

* Dependencies:

  |   Package    |    Version    |
  | :----------: | :-----------: |
  |    OpenCV    |     >= 3      |
  |     PCL      |     >= 1.8    |
  | Mech-Eye SDK |     Latest    |

* [Mech-Eye SDK (latest version)](https://downloads.mech-mind.com/?tab=tab-sdk)

### Install Dependencies

If you have installed "ros-noetic-desktop" successfully, execute the following commands to install the dependencies.

```bash
sudo apt install libopencv-dev
sudo apt install ros-noetic-cv-bridge
sudo apt install libpcl-dev
sudo apt install ros-noetic-pcl-conversions
sudo apt install python3-colcon-common-extensions
```

### Install Mech-Eye SDK

>Note: If you have installed Mech-Eye SDK before, please uninstall it first with the following command:
>
>```bash
>sudo dpkg -P MechEyeApi
>```

* If the system architecture is AMD64, execute the following command:

  ```bash
  sudo dpkg -i 'Mech-Eye_API_x.x.x_amd64.deb'
  ```

* If the system architecture is ARM64, execute the following command:

  ```bash
  sudo dpkg -i 'Mech-Eye_API_x.x.x_arm64.deb'
  ```

## Clone and Compile the Interface

Execute the following commands to clone and compile the interface:

```bash
mkdir -p ~/colcon_ws/src && cd ~/colcon_ws/src
git clone https://github.com/MechMindRobotics/mecheye_ros2_interface.git
cd ~/colcon_ws
colcon build
```

## Use the Interface

1. (Optional) Change the configurations in `~/colcon_ws/src/mecheye_ros2_interface/src/MechMindCamera.cpp` according to your needs:

   * `save_file`: Set this argument to `true` to allow file saving to the `/tmp/` directory. If you set this argument to `false`, the obtained data are not saved locally automatically.
   * `camera_ip`: If you want to connect to a specific camera by its IP address, change the value of this argument to the IP address of your camera. You also need to comment and uncomment the corresponding lines in `~/catkin_ws/src/mecheye_ros_interface/src/MechMindCamera.cpp`.
   * `fx`, `fy`, `u`, and `v`: If you need to convert the camera reference frame to another frame, change these arguments to the transformation parameters. Use quaternions for rotation.

   > Note: Remember to run `colcon build` again after making changes to `MechMindCamera.h` and `*.cpp`.

2. Open a terminal and execute the following command to start up the interface:

    ```bash
    source ~/colcon_ws/install/setup.bash
    ros2 run mecheye_ros_interface start
    ```

    or

    ```bash
    ros2 launch ~/colcon_ws/src/mecheye_ros2_interface/launch/start_camera.py
    ```

3. Enter the index number of the camera to which you want to connect, and press the Enter key.
4. Open a new terminal, and execute the following command to invoke a service. Replace `service_name` with the actual name of the service, `ServiceName` with the actual name of the service file, `parameter_name` with the actual name of the parameter, and `parameter_value` with the actual value of the parameter.

   >Note: For example commands of each service, refer to the [Services](#services) section.

    ```bash
    source ~/colcon_ws/install/setup.bash
    ros2 service call [/service_name] [mecheye_ros_interface/srv/ServiceName] "{parameter_name: parameter_value}"
    ```

## Topics

The following topics are provided:

* /mechmind/camera_info: Camera intrinsic parameters.
* /mechmind/color_image: 2D image encoded as "bgr8".
* /mechmind/stereo_color_image_left: The left stereo 2D image encoded as "bgr8".
* /mechmind/stereo_color_image_right: The right stereo 2D image encoded as "bgr8".
* /mechmind/depth_map: Depth map encoded as a single-channel image, each channel containing a 32-bit float number.
* /mechmind/point_cloud: Point cloud data.
* /mechmind/textured_point_cloud: Textured point cloud data.

  > Note: For the DEEP and LSR series, Mech-Eye API 2.3.4, the point cloud is not textured correctly when the point cloud unit is set to m. This issue will be fixed in Mech-Eye API 2.4.0. As a workaround, you can comment out line 127 in `MechMindCamera.cpp`.

## Services

### Data Acquisition

#### [capture_color_image](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/CaptureColorMap.srv)

Invoke this service to obtain a 2D image.

Example:

  ```bash
  ros2 service call /capture_color_image mecheye_ros_interface/srv/CaptureColorMap
  ```

#### [capture_depth_map](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/CaptureDepthMap.srv)

Invoke this service to obtain a depth map.

Example:

  ```bash
  ros2 service call /capture_depth_map mecheye_ros_interface/srv/CaptureDepthMap
  ```

#### [capture_point_cloud](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/CapturePointCloud.srv)

Invoke this service to obtain an untextured point cloud.

Example:

  ```bash
  ros2 service call /capture_point_cloud mecheye_ros_interface/srv/CapturePointCloud
  ```

#### [capture_textured_point_cloud](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/CaptureColorPointCloud.srv)

Invoke this service to obtain a textured point cloud.

Example:

  ```bash
  ros2 service call /capture_textured_point_cloud mecheye_ros_interface/srv/CaptureColorPointCloud
  ```

#### [capture_stereo_color_images](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/CaptureStereoColorMap.srv)

Invoke this service to obtain a stereo 2D image.

Example:

  ```bash
  ros2 service call /capture_stereo_color_images mecheye_ros_interface/srv/CaptureStereoColorMap
  ```

### Manage Parameter Groups

#### [get_all_user_sets](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/GetAllUserSets.srv)

Invoke this service to obtain the names of all available parameter groups.

Example:

  ```bash
  ros2 service call /get_all_user_sets mecheye_ros_interface/srv/GetAllUserSets
  ```

### [get_current_user_set](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/GetCurrentUserSet.srv)

Invoke this service to obtain the name of the currently selected parameter group.

Example:

  ```bash
  ros2 service call /get_current_user_set mecheye_ros_interface/srv/GetCurrentUserSet
  ```

#### [set_current_user_set](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/SetCurrentUserSet.srv)

Invoke this service to select the parameter group to use.

This service has one parameter:

* `value` (string): the name of the parameter group to be selected.

Example: Select the "123" parameter group.

  ```bash
  ros2 service call /set_current_user_set mecheye_ros_interface/srv/SetCurrentUserSet "{value: '123'}" 
  ```

>Note: Parameter group names that consist of numbers only must be surrounded by single quotation marks.

#### [add_user_set](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/AddUserSet.srv)

Invoke this service to add a parameter group. The newly added parameter group is automatically selected as the current parameter group.

This service has one parameter:

* `value` (string): the name of the parameter group to be added.

Example: Add a parameter group named "123".

  ```bash
  ros2 service call /add_user_set mecheye_ros_interface/srv/AddUserSet "{value: '123'}"
  ```

>Note: Parameter group names that consist of numbers only must be surrounded by single quotation marks.

#### [delete_user_set](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/DeleteUserSet.srv)

Invoke this service to delete the specified parameter group.

This service has one parameter:

* `value` (string): the name of the parameter group to be deleted.

Example: Delete the parameter group named "123".

  ```bash
  ros2 service call /delete_user_set mecheye_ros_interface/srv/DeleteUserSet "{value: '123'}"
  ```

>Note: Parameter group names that consist of numbers only must be surrounded by single quotation marks.

### Obtain Camera Information

#### [device_info](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/DeviceInfo.srv)

Invoke this service to print the following information of the currently connected camera:

* Model
* Serial number
* Hardware version
* Firmware version
* IP address
* Subnet mask
* IP address assigment method
* Port

Example:

  ```bash
  ros2 service call /device_info mecheye_ros_interface/srv/DeviceInfo
  ```

### Adjust Camera Parameters

> Note: The functions of obtaining and adjusting camera parameter values have been restructured. Mech-Eye SDK 2.3.4 and above provide services according to the data types of the camera parameters. To obtain or adjust the value of a camera parameter, call the service corresponding to the data type of the camera parameter and enter the name of the camera parameter as the service's parameter. The data types and names of the camera parameters can be found in the header files in the installation path of Mech-Eye SDK: `/opt/mech-mind/mech-eye-sdk/include/area_scan_3d_camera/parameters/`.

The following data types of camera parameters are distinguished:

* _Int
* _Float
* _Bool
* _Enum
* _Roi
* _Range
* _FloatArray

#### [get_int_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/GetIntParameter.srv)

Invoke this service to obtain the value of the specified _Int-type camera parameter.

This service has one parameter:

* `name` (string): the name of the camera parameter.

Example: Obtain the value of the `Scan2DExpectedGrayValue` parameter.

  ```bash
  ros2 service call /get_int_parameter mecheye_ros_interface/srv/GetIntParameter "{name: Scan2DExpectedGrayValue}"
  ```

#### [set_int_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/SetIntParameter.srv)

Invoke this service to set the value of the specified _Int-type camera parameter.

This service has two parameters:

* `name` (string): the name of the camera parameter.
* `value` (int): the new value of the camera parameter.

Example: Set the value of the `Scan2DExpectedGrayValue` parameter to 101.

  ```bash
  ros2 service call /set_int_parameter mecheye_ros_interface/srv/SetIntParameter "{name: Scan2DExpectedGrayValue, value: 101}"
  ```

#### [get_float_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/GetFloatParameter.srv)

Invoke this service to obtain the value of the specified _Float-type camera parameter.

This service has one parameter:

* `name` (string): the name of the camera parameter.

Example: Obtain the value of the `Scan2DExposureTime` parameter.

  ```bash
  ros2 service call /get_float_parameter mecheye_ros_interface/srv/GetFloatParameter "{name: Scan2DExposureTime}"
  ```

#### [set_float_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/SetFloatParameter.srv)

Invoke this service to set the value of the specified _Float-type camera parameter.

This service has two parameters:

* `name` (string): the name of the camera parameter.
* `value` (float): the new value of the camera parameter.

Example: Set the value of the `Scan2DExposureTime` parameter to 40.1.

  ```bash
  ros2 service call /set_float_parameter mecheye_ros_interface/srv/SetFloatParameter "{name: Scan2DExposureTime, value: 40.1}"
  ```

#### [get_bool_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/GetBoolParameter.srv)

Invoke this service to obtain the value of the specified _Bool-type camera parameter.

This service has one parameter:

* `name` (string): the name of the camera parameter.

Example: Obtain the value of the `Scan2DToneMappingEnable` parameter.

  ```bash
  ros2 service call /get_bool_parameter mecheye_ros_interface/srv/GetBoolParameter"{name: Scan2DToneMappingEnable}"
  ```

#### [set_bool_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/SetBoolParameter.srv)

Invoke this service to set the value of the specified _Bool-type camera parameter.

This service has two parameters:

* `name` (string): the name of the camera parameter.
* `value` (bool): the new value of the camera parameter.

Example: Set the value of the `Scan2DToneMappingEnable` parameter to `True`.

  ```bash
  ros2 service call /set_bool_parameter mecheye_ros_interface/srv/SetBoolParameter "{name: Scan2DToneMappingEnable, value: True}"
  ```

#### [get_enum_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/GetEnumParameter.srv)

Invoke this service to obtain the value of the specified _Enum-type camera parameter.

This service has one parameter:

* `name` (string): the name of the camera parameter.

Example: Obtain the value of the `Scan2DExposureMode` parameter.

  ```bash
  ros2 service call /get_enum_parameter mecheye_ros_interface/srv/GetEnumParameter "{name: Scan2DExposureMode}"
  ```

#### [set_enum_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/SetEnumParameter.srv)

Invoke this service to set the value of the specified _Enum-type camera parameter.

This service has two parameters:

* `name` (string): the name of the camera parameter.
* `value` (string): the new value of the camera parameter.

Example: Set the value of the `Scan2DExposureMode` parameter to `Timed`.

  ```bash
  ros2 service call /set_enum_parameter mecheye_ros_interface/srv/SetEnumParameter "{name: Scan2DExposureMode, value: Timed}"
  ```

#### [get_roi_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/GetROIParameter.srv)

Invoke this service to obtain the value of the specified _Roi-type camera parameter.

This service has one parameter:

* `name` (string): the name of the camera parameter.

Example: Obtain the value of the `Scan2DROI` parameter.

  ```bash
  ros2 service call /get_roi_parameter mecheye_ros_interface/srv/GetROIParameter "{name: Scan2DROI}"
  ```

#### [set_roi_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/SetROIParameter.srv)

Invoke this service to set the value of the specified _Roi-type camera parameter.

This service has five parameters:

* `name` (string): the name of the camera parameter.
* `x` (uint32): the new x-coordinate of the upper-left corner of the ROI.
* `y` (uint32): the new y-coordinate of the upper-left corner of the ROI.
* `width` (uint32): the new width of the ROI.
* `height` (uint32): the new height of the ROI.

Example: Set the value of the `Scan2DROI` parameter to [20, 20, 600, 800] (which is an ROI that is 600 px wide, 800 px high and has its upper-left corner at the (20,20) pixel).

  ```bash
  ros2 service call /set_roi_parameter mecheye_ros_interface/srv/SetROIParameter "{name: Scan2DROI, x: 20, y: 20, width: 600, height: 800}"
  ```

#### [get_range_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/GetRangeParameter.srv)

Invoke this service to obtain the value of the specified _Range-type camera parameter.

This service has one parameter:

* `name` (string): the name of the camera parameter.

Example: Obtain the value of the `DepthRange` parameter.

  ```bash
  ros2 service call /get_range_parameter mecheye_ros_interface/srv/GetRangeParameter "{name: DepthRange}"
  ```

#### [set_range_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/SetRangeParameter.srv)

Invoke this service to set the value of the specified _Range-type camera parameter.

This service has three parameters:

* `name` (string): the name of the camera parameter.
* `lower` (int32): the new minimum value of the camera parameter's value range.
* `upper` (int32): the new maximum value of the camera parameter's value range.

Example: Set the value of the `DepthRange` parameter to 200–1000.

  ```bash
  ros2 service call /set_range_parameter mecheye_ros_interface/srv/SetRangeParameter "{name: DepthRange, lower: 200, upper: 1000}"
  ```

#### [get_float_array_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/GetFloatArrayParameter.srv)

Invoke this service to obtain the value of the specified _FloatArray-type camera parameter.

This service has one parameter:

* `name` (string): the name of the camera parameter.

Example: Obtain the value of the `Scan2DHDRExposureSequence` parameter.

  ```bash
  ros2 service call /get_float_array_parameter mecheye_ros_interface/srv/GetFloatArrayParameter "{name: Scan2DHDRExposureSequence}"
  ```

#### [set_float_array_parameter](https://github.com/MechMindRobotics/mecheye_ros2_interface/blob/master/srv/SetFloatArrayParameter.srv)

Invoke this service to set the value of the specified _FloatArray-type camera parameter.

This service has two parameters:

* `name` (string): the name of the camera parameter.
* `array` (float64[]): the new value of the camera parameter.

  >Note: The possible number of elements in the sequence and the possible value range of each element can be found in the header files.

Example: Set the value of the `Scan2DHDRExposureSequence` parameter to [30.0, 35.5, 40.0].

  ```bash
  ros2 service call /set_float_array_parameter mecheye_ros_interface/srv/SetFloatArrayParameter "{name: Scan2DHDRExposureSequence, array: [30.0,35.5,40.0]}"
  ```
