#pragma once
#include <MechEyeApi.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mecheye_ros_interface/srv/add_user_set.hpp>
#include <mecheye_ros_interface/srv/capture_color_map.hpp>
#include <mecheye_ros_interface/srv/capture_color_point_cloud.hpp>
#include <mecheye_ros_interface/srv/capture_depth_map.hpp>
#include <mecheye_ros_interface/srv/capture_point_cloud.hpp>
#include <mecheye_ros_interface/srv/delete_user_set.hpp>
#include <mecheye_ros_interface/srv/device_info.hpp>
#include <mecheye_ros_interface/srv/get2_d_expected_gray_value.hpp>
#include <mecheye_ros_interface/srv/get2_d_exposure_mode.hpp>
#include <mecheye_ros_interface/srv/get2_d_exposure_sequence.hpp>
#include <mecheye_ros_interface/srv/get2_d_exposure_time.hpp>
#include <mecheye_ros_interface/srv/get2_droi.hpp>
#include <mecheye_ros_interface/srv/get2_d_sharpen_factor.hpp>
#include <mecheye_ros_interface/srv/get2_d_tone_mapping_enable.hpp>
#include <mecheye_ros_interface/srv/get3_d_exposure.hpp>
#include <mecheye_ros_interface/srv/get3_d_gain.hpp>
#include <mecheye_ros_interface/srv/get3_droi.hpp>
#include <mecheye_ros_interface/srv/get_all_user_sets.hpp>
#include <mecheye_ros_interface/srv/get_cloud_outlier_filter_mode.hpp>
#include <mecheye_ros_interface/srv/get_cloud_smooth_mode.hpp>
#include <mecheye_ros_interface/srv/get_current_user_set.hpp>
#include <mecheye_ros_interface/srv/get_depth_range.hpp>
#include <mecheye_ros_interface/srv/get_fringe_contrast_threshold.hpp>
#include <mecheye_ros_interface/srv/get_fringe_min_threshold.hpp>
#include <mecheye_ros_interface/srv/get_laser_settings.hpp>
#include <mecheye_ros_interface/srv/get_uhp_settings.hpp>
#include <mecheye_ros_interface/srv/get_uhp_capture_mode.hpp>
#include <mecheye_ros_interface/srv/get_uhp_fringe_coding_mode.hpp>
#include <mecheye_ros_interface/srv/get_projector_fringe_coding_mode.hpp>
#include <mecheye_ros_interface/srv/get_projector_power_level.hpp>
#include <mecheye_ros_interface/srv/get_projector_anti_flicker_mode.hpp>

#include <mecheye_ros_interface/srv/save_all_settings_to_user_sets.hpp>
#include <mecheye_ros_interface/srv/set2_d_expected_gray_value.hpp>
#include <mecheye_ros_interface/srv/set2_d_exposure_mode.hpp>
#include <mecheye_ros_interface/srv/set2_d_exposure_sequence.hpp>
#include <mecheye_ros_interface/srv/set2_d_exposure_time.hpp>
#include <mecheye_ros_interface/srv/set2_droi.hpp>
#include <mecheye_ros_interface/srv/set2_d_sharpen_factor.hpp>
#include <mecheye_ros_interface/srv/set2_d_tone_mapping_enable.hpp>
#include <mecheye_ros_interface/srv/set3_d_exposure.hpp>
#include <mecheye_ros_interface/srv/set3_d_gain.hpp>
#include <mecheye_ros_interface/srv/set3_droi.hpp>
#include <mecheye_ros_interface/srv/set_cloud_outlier_filter_mode.hpp>
#include <mecheye_ros_interface/srv/set_cloud_smooth_mode.hpp>
#include <mecheye_ros_interface/srv/set_current_user_set.hpp>
#include <mecheye_ros_interface/srv/set_depth_range.hpp>
#include <mecheye_ros_interface/srv/set_fringe_contrast_threshold.hpp>
#include <mecheye_ros_interface/srv/set_fringe_min_threshold.hpp>
#include <mecheye_ros_interface/srv/set_laser_settings.hpp>
#include <mecheye_ros_interface/srv/set_uhp_settings.hpp>
#include <mecheye_ros_interface/srv/set_uhp_capture_mode.hpp>
#include <mecheye_ros_interface/srv/set_uhp_fringe_coding_mode.hpp>

#include <mecheye_ros_interface/srv/set_projector_fringe_coding_mode.hpp>
#include <mecheye_ros_interface/srv/set_projector_power_level.hpp>
#include <mecheye_ros_interface/srv/set_projector_anti_flicker_mode.hpp>

class MechMindCamera
{
public:
    MechMindCamera();
    rclcpp::Node::SharedPtr node;

private:
    mmind::api::MechEyeDevice device;
    mmind::api::DeviceIntri intri;

    std::string camera_ip;
    bool save_file = false;
    bool use_external_intri = false;
    double fx = 0;
    double fy = 0;
    double u = 0;
    double v = 0;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_color;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info;

    void publishColorMap(mmind::api::ColorMap &colorMap);
    void publishDepthMap(mmind::api::DepthMap &depthMap);
    void publishPointCloud(mmind::api::PointXYZMap &pointXYZMap);
    void publishColorPointCloud(mmind::api::PointXYZBGRMap &pointXYZBGRMap);
    void publishCameraInfo(const std_msgs::msg::Header &header, int width, int height);

    rclcpp::Service<mecheye_ros_interface::srv::AddUserSet>::SharedPtr add_user_set_service;
    rclcpp::Service<mecheye_ros_interface::srv::CaptureColorMap>::SharedPtr capture_color_map_service;
    rclcpp::Service<mecheye_ros_interface::srv::CaptureColorPointCloud>::SharedPtr capture_color_point_cloud_service;
    rclcpp::Service<mecheye_ros_interface::srv::CaptureDepthMap>::SharedPtr capture_depth_map_service;
    rclcpp::Service<mecheye_ros_interface::srv::CapturePointCloud>::SharedPtr capture_point_cloud_service;
    rclcpp::Service<mecheye_ros_interface::srv::DeleteUserSet>::SharedPtr delete_user_set_service;
    rclcpp::Service<mecheye_ros_interface::srv::DeviceInfo>::SharedPtr device_info_service;
    rclcpp::Service<mecheye_ros_interface::srv::Get2DExpectedGrayValue>::SharedPtr get_2d_expected_gray_value_service;
    rclcpp::Service<mecheye_ros_interface::srv::Get2DExposureMode>::SharedPtr get_2d_exposure_mode_service;
    rclcpp::Service<mecheye_ros_interface::srv::Get2DExposureSequence>::SharedPtr get_2d_exposure_sequence_service;
    rclcpp::Service<mecheye_ros_interface::srv::Get2DExposureTime>::SharedPtr get_2d_exposure_time_service;
    rclcpp::Service<mecheye_ros_interface::srv::Get2DROI>::SharedPtr get_2d_roi_service;
    rclcpp::Service<mecheye_ros_interface::srv::Get2DSharpenFactor>::SharedPtr get_2d_sharpen_factor_service;
    rclcpp::Service<mecheye_ros_interface::srv::Get2DToneMappingEnable>::SharedPtr get_2d_tone_mapping_service;
    rclcpp::Service<mecheye_ros_interface::srv::Get3DExposure>::SharedPtr get_3d_exposure_service;
    rclcpp::Service<mecheye_ros_interface::srv::Get3DGain>::SharedPtr get_3d_gain_service;
    rclcpp::Service<mecheye_ros_interface::srv::Get3DROI>::SharedPtr get_3d_roi_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetAllUserSets>::SharedPtr get_all_user_sets_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetCloudOutlierFilterMode>::SharedPtr get_cloud_outlier_filter_mode_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetCloudSmoothMode>::SharedPtr get_cloud_smooth_mode_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetCurrentUserSet>::SharedPtr get_current_user_set_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetDepthRange>::SharedPtr get_depth_range_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetFringeContrastThreshold>::SharedPtr get_fringe_contrast_threshold_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetFringeMinThreshold>::SharedPtr get_fringe_min_threshold_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetLaserSettings>::SharedPtr get_laser_settings_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetUhpSettings>::SharedPtr get_uhp_settings_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetUhpCaptureMode>::SharedPtr get_uhp_capture_mode_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetUhpFringeCodingMode>::SharedPtr get_uhp_fringe_coding_mode_service;
    
    rclcpp::Service<mecheye_ros_interface::srv::GetProjectorFringeCodingMode>::SharedPtr get_projector_fringe_coding_mode_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetProjectorPowerLevel>::SharedPtr get_projector_power_level_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetProjectorAntiFlickerMode>::SharedPtr get_projector_anti_flicker_mode_service;
    
    rclcpp::Service<mecheye_ros_interface::srv::SaveAllSettingsToUserSets>::SharedPtr save_all_settings_to_user_sets_service;
    rclcpp::Service<mecheye_ros_interface::srv::Set2DExpectedGrayValue>::SharedPtr set_2d_expected_gray_value_service;
    rclcpp::Service<mecheye_ros_interface::srv::Set2DExposureMode>::SharedPtr set_2d_exposure_mode_service;
    rclcpp::Service<mecheye_ros_interface::srv::Set2DExposureSequence>::SharedPtr set_2d_exposure_sequence_service;
    rclcpp::Service<mecheye_ros_interface::srv::Set2DExposureTime>::SharedPtr set_2d_exposure_time_service;
    rclcpp::Service<mecheye_ros_interface::srv::Set2DROI>::SharedPtr set_2d_roi_service;
    rclcpp::Service<mecheye_ros_interface::srv::Set2DSharpenFactor>::SharedPtr set_2d_sharpen_factor_service;
    rclcpp::Service<mecheye_ros_interface::srv::Set2DToneMappingEnable>::SharedPtr set_2d_tone_mapping_service;
    rclcpp::Service<mecheye_ros_interface::srv::Set3DExposure>::SharedPtr set_3d_exposure_service;
    rclcpp::Service<mecheye_ros_interface::srv::Set3DGain>::SharedPtr set_3d_gain_service;
    rclcpp::Service<mecheye_ros_interface::srv::Set3DROI>::SharedPtr set_3d_roi_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetCloudOutlierFilterMode>::SharedPtr set_cloud_outlier_filter_mode_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetCloudSmoothMode>::SharedPtr set_cloud_smooth_mode_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetCurrentUserSet>::SharedPtr set_current_user_set_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetDepthRange>::SharedPtr set_depth_range_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetFringeContrastThreshold>::SharedPtr set_fringe_contrast_threshold_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetFringeMinThreshold>::SharedPtr set_fringe_min_threshold_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetLaserSettings>::SharedPtr set_laser_settings_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetUhpSettings>::SharedPtr set_uhp_settings_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetUhpCaptureMode>::SharedPtr set_uhp_capture_mode_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetUhpFringeCodingMode>::SharedPtr set_uhp_fringe_coding_mode_service;
    
    rclcpp::Service<mecheye_ros_interface::srv::SetProjectorFringeCodingMode>::SharedPtr set_projector_fringe_coding_mode_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetProjectorPowerLevel>::SharedPtr set_projector_power_level_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetProjectorAntiFlickerMode>::SharedPtr set_projector_anti_flicker_mode_service;
    
    void add_user_set_callback(const std::shared_ptr<mecheye_ros_interface::srv::AddUserSet::Request> req, std::shared_ptr<mecheye_ros_interface::srv::AddUserSet::Response> res);
    void capture_color_map_callback(const std::shared_ptr<mecheye_ros_interface::srv::CaptureColorMap::Request> req, std::shared_ptr<mecheye_ros_interface::srv::CaptureColorMap::Response> res);
    void capture_color_point_cloud_callback(const std::shared_ptr<mecheye_ros_interface::srv::CaptureColorPointCloud::Request> req, std::shared_ptr<mecheye_ros_interface::srv::CaptureColorPointCloud::Response> res);
    void capture_depth_map_callback(const std::shared_ptr<mecheye_ros_interface::srv::CaptureDepthMap::Request> req, std::shared_ptr<mecheye_ros_interface::srv::CaptureDepthMap::Response> res);
    void capture_point_cloud_callback(const std::shared_ptr<mecheye_ros_interface::srv::CapturePointCloud::Request> req, std::shared_ptr<mecheye_ros_interface::srv::CapturePointCloud::Response> res);
    void delete_user_set_callback(const std::shared_ptr<mecheye_ros_interface::srv::DeleteUserSet::Request> req, std::shared_ptr<mecheye_ros_interface::srv::DeleteUserSet::Response> res);
    void device_info_callback(const std::shared_ptr<mecheye_ros_interface::srv::DeviceInfo::Request> req, std::shared_ptr<mecheye_ros_interface::srv::DeviceInfo::Response> res);
    void get_2d_expected_gray_value_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DExpectedGrayValue::Request> req,
                                             std::shared_ptr<mecheye_ros_interface::srv::Get2DExpectedGrayValue::Response> res);
    void get_2d_exposure_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DExposureMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get2DExposureMode::Response> res);
    void get_2d_exposure_sequence_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DExposureSequence::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get2DExposureSequence::Response> res);
    void get_2d_exposure_time_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DExposureTime::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get2DExposureTime::Response> res);
    void get_2d_roi_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DROI::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get2DROI::Response> res);
    void get_2d_sharpen_factor_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DSharpenFactor::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get2DSharpenFactor::Response> res);
    void get_2d_tone_mapping_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DToneMappingEnable::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get2DToneMappingEnable::Response> res);
    void get_3d_exposure_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get3DExposure::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get3DExposure::Response> res);
    void get_3d_gain_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get3DGain::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get3DGain::Response> res);
    void get_3d_roi_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get3DROI::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get3DROI::Response> res);
    void get_all_user_sets_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetAllUserSets::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetAllUserSets::Response> res);
    void get_cloud_outlier_filter_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetCloudOutlierFilterMode::Request> req,
                                                std::shared_ptr<mecheye_ros_interface::srv::GetCloudOutlierFilterMode ::Response> res);
    void get_cloud_smooth_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetCloudSmoothMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetCloudSmoothMode::Response> res);
    void get_current_user_set_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetCurrentUserSet::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetCurrentUserSet::Response> res);
    void get_depth_range_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetDepthRange::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetDepthRange::Response> res);
    void get_fringe_contrast_threshold_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetFringeContrastThreshold::Request> req,
                                                std::shared_ptr<mecheye_ros_interface::srv::GetFringeContrastThreshold::Response> res);
    void get_fringe_min_threshold_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetFringeMinThreshold::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetFringeMinThreshold::Response> res);
    void get_laser_settings_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetLaserSettings::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetLaserSettings::Response> res);
    void get_uhp_settings_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetUhpSettings::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetUhpSettings::Response> res);
    void get_uhp_capture_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetUhpCaptureMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetUhpCaptureMode::Response> res);
    void get_uhp_fringe_coding_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetUhpFringeCodingMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetUhpFringeCodingMode::Response> res);
    
    void get_projector_fringe_coding_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetProjectorFringeCodingMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetProjectorFringeCodingMode::Response> res);
    void get_projector_power_level_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetProjectorPowerLevel::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetProjectorPowerLevel::Response> res);
    void get_projector_anti_flicker_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetProjectorAntiFlickerMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetProjectorAntiFlickerMode::Response> res);


    void save_all_settings_to_user_sets_callback(const std::shared_ptr<mecheye_ros_interface::srv::SaveAllSettingsToUserSets::Request> req,
                                                 std::shared_ptr<mecheye_ros_interface::srv::SaveAllSettingsToUserSets::Response> res);
    void set_2d_expected_gray_value_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DExpectedGrayValue::Request> req,
                                             std::shared_ptr<mecheye_ros_interface::srv::Set2DExpectedGrayValue::Response> res);
    void set_2d_exposure_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DExposureMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set2DExposureMode::Response> res);
    void set_2d_exposure_sequence_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DExposureSequence::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set2DExposureSequence::Response> res);
    void set_2d_exposure_time_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DExposureTime::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set2DExposureTime::Response> res);
    void set_2d_roi_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DROI::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set2DROI::Response> res);
    void set_2d_sharpen_factor_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DSharpenFactor::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set2DSharpenFactor::Response> res);
    void set_2d_tone_mapping_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DToneMappingEnable::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set2DToneMappingEnable::Response> res);
    void set_3d_exposure_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set3DExposure::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set3DExposure::Response> res);
    void set_3d_gain_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set3DGain::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set3DGain::Response> res);
    void set_3d_roi_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set3DROI::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set3DROI::Response> res);
    void set_cloud_outlier_filter_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetCloudOutlierFilterMode::Request> req,
                                                std::shared_ptr<mecheye_ros_interface::srv::SetCloudOutlierFilterMode ::Response> res);
    void set_cloud_smooth_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetCloudSmoothMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetCloudSmoothMode::Response> res);
    void set_current_user_set_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetCurrentUserSet::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetCurrentUserSet::Response> res);
    void set_depth_range_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetDepthRange::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetDepthRange::Response> res);
    void set_fringe_contrast_threshold_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetFringeContrastThreshold::Request> req,
                                                std::shared_ptr<mecheye_ros_interface::srv::SetFringeContrastThreshold::Response> res);
    void set_fringe_min_threshold_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetFringeMinThreshold::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetFringeMinThreshold::Response> res);
    void set_laser_settings_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetLaserSettings::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetLaserSettings::Response> res);

    void set_uhp_settings_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetUhpSettings::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetUhpSettings::Response> res);
    void set_uhp_capture_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetUhpCaptureMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetUhpCaptureMode::Response> res);
    void set_uhp_fringe_coding_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetUhpFringeCodingMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetUhpFringeCodingMode::Response> res);
    
    void set_projector_fringe_coding_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetProjectorFringeCodingMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetProjectorFringeCodingMode::Response> res);
    void set_projector_power_level_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetProjectorPowerLevel::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetProjectorPowerLevel::Response> res);
    void set_projector_anti_flicker_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetProjectorAntiFlickerMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetProjectorAntiFlickerMode::Response> res);
};
