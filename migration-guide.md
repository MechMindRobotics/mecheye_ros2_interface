# Migration Guide from Version 2.0.2 to Version 2.3.4

As Mech-Eye API has been restructured in version 2.2.0, the ROS 2 interface for Mech-Eye Industrial 3D Camera has also been restructured accordingly. This document lists the topics and services that have changed from the previous release of the ROS 2 interface to the latest release. You can modify your program according to this document.

## Compatibility

* Previous release of the ROS 2 interface: compatible with Mech-Eye SDK 2.0.1 and 2.0.2
* Latest release of the ROS 2 interface: compatible with Mech-Eye SDK 2.3.4

If you would like to use the latest ROS 2 interface for the camera, please install the latest version of Mech-Eye SDK or upgrade Mech-Eye SDK to the latest version.

If you are using Mech-Eye SDK 2.0.2 and below, please install the previous release of the ROS 2 interface (can be found from the [Releases page](https://github.com/MechMindRobotics/mecheye_ros2_interface/releases) on GitHub.)

>Note: No compatible release of ROS 2 interface was made for Mech-Eye SDK 2.1.0 to 2.3.3. If you are using these versions of Mech-Eye SDK and would like to use the ROS 2 interface, please upgrade Mech-Eye SDK to the latest version.

## Topics

### Depth Map

* Version 2.0.2: /mechmind/depth_image
* Version 2.3.4: /mechmind/depth_map

### Textured Point Cloud

* Version 2.0.2: /mechmind/color_point_cloud
* Version 2.3.4: /mechmind/textured_point_cloud

## Services

### Data Acquisition

#### Capture 2D Image

* Version 2.0.2:

  ```bash
  ros2 service call /capture_color_map mecheye_ros_interface/srv/CaptureColorMap
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /capture_color_image mecheye_ros_interface/srv/CaptureColorImage
  ```

#### Capture Textured Point Cloud

* Version 2.0.2:

  ```bash
  ros2 service call /capture_color_point_cloud mecheye_ros_interface/srv/CaptureColorPointCloud
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /capture_textured_point_cloud mecheye_ros_interface/srv/CaptureTexturedPointCloud
  ```

### Adjust Camera Parameters

#### Get Scan2DExpectedGrayValue

* Version 2.0.2:

  ```bash
  ros2 service call /get_2d_expected_gray_value mecheye_ros_interface/srv/Get2DExpectedGrayValue
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_int_parameter mecheye_ros_interface/srv/GetIntParameter "{name: Scan2DExpectedGrayValue}"
  ```

#### Set Scan2DExpectedGrayValue

* Version 2.0.2:

  ```bash
  ros2 service call /set_2d_expected_gray_value mecheye_ros_interface/srv/Set2DExpectedGrayValue "{value: 20}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_int_parameter mecheye_ros_interface/srv/SetIntParameter "{name: Scan2DExpectedGrayValue, value:20}"
  ```

#### Get Scan2DExposureMode

* Version 2.0.2:

  ```bash
  ros2 service call /get_2d_exposure_mode mecheye_ros_interface/srv/Get2DExposureMode
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_enum_parameter mecheye_ros_interface/srv/GetEnumParameter "{name: Scan2DExposureMode}"
  ```

#### Set Scan2DExposureMode

* Version 2.0.2:

  ```bash
  ros2 service call /set_2d_exposure_mode mecheye_ros_interface/srv/Set2DExposureMode "{value: HDR}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_enum_parameter mecheye_ros_interface/srv/GetEnumParameter "{name: Scan2DExposureMode, value: HDR}"
  ```

#### Get Scan2DHDRExposureSequence

* Version 2.0.2:

  ```bash
  ros2 service call /get_2d_exposure_sequence mecheye_ros_interface/srv/Get2DExposureSequence
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_float_array_parameter mecheye_ros_interface/srv/GetFloatArrayParameter "{name: Scan2DHDRExposureSequence}"
  ```

#### Set Scan2DHDRExposureSequence

* Version 2.0.2:

  ```bash
  ros2 service call /set_2d_exposure_sequence mecheye_ros_interface/srv/Set2DExposureSequence "{sequence: [30.0,35.5,40.0]}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_float_array_parameter mecheye_ros_interface/srv/SetFloatArrayParameter "{name: Scan2DHDRExposureSequence, array: [30.0, 35.5, 40.0]}"
  ```

#### Get Scan2DExposureTime

* Version 2.0.2:

  ```bash
  ros2 service call /get_2d_exposure_time mecheye_ros_interface/srv/Get2DExposureTime
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_float_parameter mecheye_ros_interface/srv/GetFloatParameter "{name: Scan2DExposureTime}"
  ```

#### Set Scan2DExposureTime

* Version 2.0.2:

  ```bash
  ros2 service call /set_2d_exposure_time mecheye_ros_interface/srv/Set2DExposureTime "{value: 35.5}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_float_parameter mecheye_ros_interface/srv/SetFloatParameter "{name: Scan2DExposureTime, value: 35.5}"
  ```

#### Get Scan2DROI

* Version 2.0.2:

  ```bash
  ros2 service call /get_2d_roi mecheye_ros_interface/srv/Get2DROI
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_roi_parameter mecheye_ros_interface/srv/GetROIParameter "{name: Scan2DROI}"
  ```

#### Set Scan2DROI

* Version 2.0.2:

  ```bash
  ros2 service call /set_2d_roi mecheye_ros_interface/srv/Set2DROI "{x: 20, y: 20, width: 600, height: 800}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_roi_parameter mecheye_ros_interface/srv/SetROIParameter "{name: Scan2DROI, x: 20, y: 20, width: 600, height: 800}"
  ```

#### Get Scan2DSharpenFactor

* Version 2.0.2:

  ```bash
  ros2 service call /get_2d_sharpen_factor mecheye_ros_interface/srv/Get2DSharpenFactor
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_float_parameter mecheye_ros_interface/srv/GetFloatParameter "{name: Scan2DSharpenFactor}"
  ```

#### Set Scan2DSharpenFactor

* Version 2.0.2:

  ```bash
  ros2 service call /set_2d_sharpen_factor mecheye_ros_interface/srv/Set2DSharpenFactor "{value: 0.5}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_float_parameter mecheye_ros_interface/srv/SetFloatParameter "{name: Scan2DSharpenFactor, value: 0.5}"
  ```

#### Get Scan2DToneMappingEnable

* Version 2.0.2:

  ```bash
  ros2 service call /get_2d_tone_mapping mecheye_ros_interface/srv/Get2DToneMappingEnable
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_bool_parameter mecheye_ros_interface/srv/GetBoolParameter "{name: Scan2DToneMappingEnable}"
  ```

#### Set Scan2DToneMappingEnable

* Version 2.0.2:

  ```bash
  ros2 service call /set_2d_tone_mapping mecheye_ros_interface/srv/Set2DToneMappingEnable "{value: True}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_bool_parameter mecheye_ros_interface/srv/SetBoolParameter "{name: Scan2DToneMappingEnable, value: True}"
  ```

#### Get Scan3DExposureSequence

* Version 2.0.2:

  ```bash
  ros2 service call /get_3d_exposure mecheye_ros_interface/srv/Get3DExposure
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_float_array_parameter mecheye_ros_interface/srv/GetFloatArrayParameter "{name: Scan3DExposureSequence}"
  ```

#### Set Scan3DExposureSequence

* Version 2.0.2:

  ```bash
  ros2 service call /set_3d_exposure mecheye_ros_interface/srv/Set3DExposure "{sequence: [30.0,35.5,40.0]}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_float_array_parameter mecheye_ros_interface/srv/SetFloatArrayParameter "{name: Scan3DExposureSequence, array: [30.0, 35.5, 40.0]}"
  ```

#### Get Scan3DGain

* Version 2.0.2:

  ```bash
  ros2 service call /get_3d_gain mecheye_ros_interface/srv/Get3DGain
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_float_parameter mecheye_ros_interface/srv/GetFloatParameter "{name: Scan3DGain}"
  ```

#### Set Scan3DGain

* Version 2.0.2:

  ```bash
  ros2 service call /set_3d_gain mecheye_ros_interface/srv/Set3DGain "{value: 2.5}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_float_parameter mecheye_ros_interface/srv/SetFloatParameter "{name: Scan3DGain, value: 2.5}"
  ```

#### Get Scan3DROI

* Version 2.0.2:

  ```bash
  ros2 service call /get_3d_roi mecheye_ros_interface/srv/Get3DROI
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_roi_parameter mecheye_ros_interface/srv/GetROIParameter "{name: Scan3DROI}"
  ```

#### Set Scan3DROI

* Version 2.0.2:

  ```bash
  ros2 service call /set_3d_roi mecheye_ros_interface/srv/Set3DROI "{x: 20, y: 20, width: 600, height: 800}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_roi_parameter mecheye_ros_interface/srv/SetROIParameter "{name: Scan3DROI, x: 20, y: 20, width: 600, height: 800}"
  ```

#### Get PointCloudOutlierRemoval

* Version 2.0.2:

  ```bash
  ros2 service call /get_cloud_outlier_filter_mode mecheye_ros_interface/srv/GetCloudOutlierFilterMode
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_enum_parameter mecheye_ros_interface/srv/GetEnumParameter "{name: PointCloudOutlierRemoval}"
  ```

#### Set PointCloudOutlierRemoval

* Version 2.0.2:

  ```bash
  ros2 service call /set_cloud_outlier_filter_mode mecheye_ros_interface/srv/SetCloudOutlierFilterMode "{value: 'Off'}"
  ```

  >Note: The `Off` option must be surrounded by single quotation marks.

* Version 2.3.4:

  ```bash
  ros2 service call /set_enum_parameter mecheye_ros_interface/srv/SetEnumParameter "{name: PointCloudOutlierRemoval, value: 'Off'}"
  ```

  >Note: The `Off` option must be surrounded by single quotation marks.

#### Get PointCloudSurfaceSmoothing

* Version 2.0.2:

  ```bash
  ros2 service call /get_cloud_smooth_mode mecheye_ros_interface/srv/GetCloudSmoothMode
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_enum_parameter mecheye_ros_interface/srv/GetEnumParameter "{name: PointCloudSurfaceSmoothing}"
  ```

#### Set PointCloudSurfaceSmoothing

* Version 2.0.2:

  ```bash
  ros2 service call /set_cloud_smooth_mode mecheye_ros_interface/srv/SetCloudSmoothMode "{value: 'Off'}"
  ```

  >Note: The `Off` option must be surrounded by single quotation marks.

* Version 2.3.4:

  ```bash
  ros2 service call /set_enum_parameter mecheye_ros_interface/srv/SetEnumParameter "{name: PointCloudSurfaceSmoothing, value: 'Off'}"
  ```

  >Note: The `Off` option must be surrounded by single quotation marks.

#### Get DepthRange

* Version 2.0.2:

  ```bash
  ros2 service call /get_depth_range mecheye_ros_interface/srv/GetDepthRange
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_range_parameter mecheye_ros_interface/srv/GetRangeParameter "{name: DepthRange}"
  ```

#### Set DepthRange

* Version 2.0.2:

  ```bash
  ros2 service call /set_depth_range mecheye_ros_interface/srv/SetDepthRange "{lower: 300, upper: 1000}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_range_parameter mecheye_ros_interface/srv/SetRangeParameter "{name: DepthRange, lower: 300, upper: 1000}"
  ```

#### Get FringeContrastThreshold

* Version 2.0.2:

  ```bash
  ros2 service call /get_fringe_contrast_threshold mecheye_ros_interface/srv/GetFringeContrastThreshold
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_int_parameter mecheye_ros_interface/srv/GetIntParameter "{name: FringeContrastThreshold}"
  ```

#### Set FringeContrastThreshold

* Version 2.0.2:

  ```bash
  ros2 service call /set_fringe_contrast_threshold mecheye_ros_interface/srv/SetFringeContrastThreshold "{value: 3}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_int_parameter mecheye_ros_interface/srv/SetIntParameter "{name: FringeContrastThreshold, value: 3}"
  ```

#### Get FringeMinThreshold

* Version 2.0.2:

  ```bash
  ros2 service call /get_fringe_min_threshold mecheye_ros_interface/srv/GetFringeMinThreshold
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_int_parameter mecheye_ros_interface/srv/GetIntParameter "{name: FringeMinThreshold}"
  ```

#### Set FringeMinThreshold

* Version 2.0.2:

  ```bash
  ros2 service call /set_fringe_min_threshold mecheye_ros_interface/srv/SetFringeMinThreshold "{value: 3}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_int_parameter mecheye_ros_interface/srv/SetIntParameter "{name: FringeMinThreshold, value: 3}"
  ```

#### Get LaserPowerLevel

* Version 2.0.2: This service obtains the values of all the parameters in the "Laser" category.

  ```bash
  ros2 service call /get_laser_settings mecheye_ros_interface/srv/GetLaserSettings
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_int_parameter mecheye_ros_interface/srv/GetIntParameter "{name: LaserPowerLevel}"
  ```

#### Set LaserPowerLevel

* Version 2.0.2: This service sets the values of all the parameters in the "Laser" category.

  ```bash
  ros2 service call /set_laser_settings mecheye_ros_interface/srv/SetLaserSettings "{fringe_coding_mode: Fast, frame_range_start: 20, frame_range_end: 80, frame_partition_count: 2, power_level: 60}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_int_parameter mecheye_ros_interface/srv/SetIntParameter "{name: LaserPowerLevel, value: 60}"
  ```

#### Get LaserFringeCodingMode

* Version 2.0.2: This service obtains the values of all the parameters in the "Laser" category.

  ```bash
  ros2 service call /get_laser_settings mecheye_ros_interface/srv/GetLaserSettings
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_enum_parameter mecheye_ros_interface/srv/GetEnumParameter "{name: LaserFringeCodingMode}"
  ```

#### Set LaserFringeCodingMode

* Version 2.0.2: This service sets the values of all the parameters in the "Laser" category.

  ```bash
  ros2 service call /set_laser_settings mecheye_ros_interface/srv/SetLaserSettings "{fringe_coding_mode: Fast, frame_range_start: 20, frame_range_end: 80, frame_partition_count: 2, power_level: 60}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_enum_parameter mecheye_ros_interface/srv/SetEnumParameter "{name: LaserFringeCodingMode, value: Fast}"
  ```

#### Get LaserFrameRange

* Version 2.0.2: This service obtains the values of all the parameters in the "Laser" category.

  ```bash
  ros2 service call /get_laser_settings mecheye_ros_interface/srv/GetLaserSettings
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_range_parameter mecheye_ros_interface/srv/GetRangeParameter "{name: LaserFrameRange}"
  ```

#### Set LaserFrameRange

* Version 2.0.2: This service sets the values of all the parameters in the "Laser" category.

  ```bash
  ros2 service call /set_laser_settings mecheye_ros_interface/srv/SetLaserSettings "{fringe_coding_mode: Fast, frame_range_start: 20, frame_range_end: 80, frame_partition_count: 2, power_level: 60}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_range_parameter mecheye_ros_interface/srv/SetRangeParameter "{name: LaserFrameRange, lower: 20, upper: 80}"
  ```

#### Get LaserFramePartitionCount

* Version 2.0.2: This service obtains the values of all the parameters in the "Laser" category.

  ```bash
  ros2 service call /get_laser_settings mecheye_ros_interface/srv/GetLaserSettings
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_int_parameter mecheye_ros_interface/srv/GetIntParameter "{name: LaserFramePartitionCount}"
  ```

#### Set LaserFramePartitionCount

* Version 2.0.2: This service sets the values of all the parameters in the "Laser" category.

  ```bash
  ros2 service call /set_laser_settings mecheye_ros_interface/srv/SetLaserSettings "{fringe_coding_mode: Fast, frame_range_start: 20, frame_range_end: 80, frame_partition_count: 2, power_level: 60}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_int_parameter mecheye_ros_interface/srv/SetIntParameter "{name: LaserFramePartitionCount, value: 2}"
  ```

#### Get AntiFlickerMode

* Version 2.0.2:

  ```bash
  ros2 service call /get_projector_anti_flicker_mode mecheye_ros_interface/srv/GetProjectorAntiFlickerMode
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_enum_parameter mecheye_ros_interface/srv/GetEnumParameter "{name: AntiFlickerMode}"
  ```

#### Set AntiFlickerMode

* Version 2.0.2:

  ```bash
  ros2 service call /set_projector_anti_flicker_mode mecheye_ros_interface/srv/SetProjectorAntiFlickerMode "{value: 'Off'}"
  ```

  >Note: The `Off` option must be surrounded by single quotation marks.

* Version 2.3.4:

  ```bash
  ros2 service call /set_enum_parameter mecheye_ros_interface/srv/SetEnumParameter "{name: AntiFlickerMode, value: 'Off'}"
  ```

  >Note: The `Off` option must be surrounded by single quotation marks.

#### Get ProjectorFringeCodingMode

* Version 2.0.2:

  * Models other than UHP:

    ```bash
    ros2 service call /get_projector_fringe_coding_mode mecheye_ros_interface/srv/GetProjectorFringeCodingMode
    ```

  * UHP:

    ```bash
    ros2 service call /get_uhp_fringe_coding_mode mecheye_ros_interface/srv/GetUhpFringeCodingMode
    ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_enum_parameter mecheye_ros_interface/srv/GetEnumParameter "{name: ProjectorFringeCodingMode}"
  ```

#### Set ProjectorFringeCodingMode

* Version 2.0.2:

  * Models other than UHP:

    ```bash
    ros2 service call /set_projector_fringe_coding_mode mecheye_ros_interface/srv/SetProjectorFringeCodingMode "{value: Accurate}"
    ```

  * UHP:

    ```bash
    ros2 service call /set_uhp_fringe_coding_mode mecheye_ros_interface/srv/SetUhpFringeCodingMode "{value: Accurate}"
    ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_enum_parameter mecheye_ros_interface/srv/SetEnumParameter "{name: ProjectorFringeCodingMode, value: Accurate}"
  ```

#### Get ProjectorPowerLevel

* Version 2.0.2:

  ```bash
  ros2 service call /get_projector_power_level mecheye_ros_interface/srv/GetProjectorPowerLevel
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_enum_parameter mecheye_ros_interface/srv/GetEnumParameter "{name: ProjectorPowerLevel}"
  ```

#### Set ProjectorPowerLevel

* Version 2.0.2:

  ```bash
  ros2 service call /set_projector_power_level mecheye_ros_interface/srv/SetProjectorPowerLevel "{value: High}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_enum_parameter mecheye_ros_interface/srv/SetEnumParameter "{name: ProjectorPowerLevel, value: High}"
  ```

#### Get UhpCaptureMode

* Version 2.0.2:

  ```bash
  ros2 service call /get_uhp_capture_mode mecheye_ros_interface/srv/GetUhpCaptureMode
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /get_enum_parameter mecheye_ros_interface/srv/GetEnumParameter "{name: UhpCaptureMode}"
  ```

#### Set UhpCaptureMode

* Version 2.0.2:

  ```bash
  ros2 service call /set_uhp_capture_mode mecheye_ros_interface/srv/SetUhpCaptureMode "{capture_mode: Merge}"
  ```

* Version 2.3.4:

  ```bash
  ros2 service call /set_enum_parameter mecheye_ros_interface/srv/SetEnumParameter "{name: UhpCaptureMode, value: Merge}"
  ```
