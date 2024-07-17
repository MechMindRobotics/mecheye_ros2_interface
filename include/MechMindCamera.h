#pragma once
#include <area_scan_3d_camera/Camera.h>
#include <area_scan_3d_camera/CameraProperties.h>
#include <area_scan_3d_camera/Frame3D.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mecheye_ros_interface/srv/capture_color_image.hpp>
#include <mecheye_ros_interface/srv/capture_stereo_color_images.hpp>
#include <mecheye_ros_interface/srv/capture_textured_point_cloud.hpp>
#include <mecheye_ros_interface/srv/capture_depth_map.hpp>
#include <mecheye_ros_interface/srv/capture_point_cloud.hpp>
#include <mecheye_ros_interface/srv/delete_user_set.hpp>
#include <mecheye_ros_interface/srv/device_info.hpp>
#include <mecheye_ros_interface/srv/add_user_set.hpp>
#include <mecheye_ros_interface/srv/get_all_user_sets.hpp>
#include <mecheye_ros_interface/srv/get_current_user_set.hpp>
#include <mecheye_ros_interface/srv/save_all_settings_to_user_sets.hpp>
#include <mecheye_ros_interface/srv/set_current_user_set.hpp>
#include <mecheye_ros_interface/srv/set_int_parameter.hpp>
#include <mecheye_ros_interface/srv/get_int_parameter.hpp>
#include <mecheye_ros_interface/srv/set_bool_parameter.hpp>
#include <mecheye_ros_interface/srv/get_bool_parameter.hpp>
#include <mecheye_ros_interface/srv/set_float_parameter.hpp>
#include <mecheye_ros_interface/srv/get_float_parameter.hpp>
#include <mecheye_ros_interface/srv/set_enum_parameter.hpp>
#include <mecheye_ros_interface/srv/get_enum_parameter.hpp>
#include <mecheye_ros_interface/srv/set_range_parameter.hpp>
#include <mecheye_ros_interface/srv/get_range_parameter.hpp>
#include <mecheye_ros_interface/srv/set_roi_parameter.hpp>
#include <mecheye_ros_interface/srv/get_roi_parameter.hpp>
#include <mecheye_ros_interface/srv/set_float_array_parameter.hpp>
#include <mecheye_ros_interface/srv/get_float_array_parameter.hpp>

class MechMindCamera
{
public:
    MechMindCamera();
    rclcpp::Node::SharedPtr node;

private:
    mmind::eye::Camera camera;
    mmind::eye::CameraIntrinsics intrinsics;

    std::string camera_ip;
    bool save_file = false;
    bool use_external_intri = false;
    double fx = 0;
    double fy = 0;
    double u = 0;
    double v = 0;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_left;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_right;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_color;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info;

    void publishColorMap(mmind::eye::Color2DImage& color2DImage);
    void publishStereoColorMap(mmind::eye::Color2DImage& leftColor2DImage,
                               mmind::eye::Color2DImage& rightColor2DImage);
    void publishDepthMap(mmind::eye::DepthMap& depthMap);
    void publishPointCloud(mmind::eye::PointCloud& pointCloud);
    void publishColorPointCloud(mmind::eye::TexturedPointCloud& texturedPointCloud);
    void publishColorCameraInfo(const std_msgs::msg::Header& header, int width, int height);
    void publishDepthCameraInfo(const std_msgs::msg::Header& header, int width, int height);

    rclcpp::Service<mecheye_ros_interface::srv::AddUserSet>::SharedPtr add_user_set_service;
    rclcpp::Service<mecheye_ros_interface::srv::CaptureColorImage>::SharedPtr
        capture_color_image_service;
    rclcpp::Service<mecheye_ros_interface::srv::CaptureStereoColorImages>::SharedPtr
        capture_stereo_color_images_service;
    rclcpp::Service<mecheye_ros_interface::srv::CaptureTexturedPointCloud>::SharedPtr
        capture_textured_point_cloud_service;
    rclcpp::Service<mecheye_ros_interface::srv::CaptureDepthMap>::SharedPtr
        capture_depth_map_service;
    rclcpp::Service<mecheye_ros_interface::srv::CapturePointCloud>::SharedPtr
        capture_point_cloud_service;

    rclcpp::Service<mecheye_ros_interface::srv::DeleteUserSet>::SharedPtr delete_user_set_service;
    rclcpp::Service<mecheye_ros_interface::srv::DeviceInfo>::SharedPtr device_info_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetAllUserSets>::SharedPtr
        get_all_user_sets_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetCurrentUserSet>::SharedPtr
        get_current_user_set_service;
    rclcpp::Service<mecheye_ros_interface::srv::SaveAllSettingsToUserSets>::SharedPtr
        save_all_settings_to_user_sets_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetCurrentUserSet>::SharedPtr
        set_current_user_set_service;

    rclcpp::Service<mecheye_ros_interface::srv::GetFloatArrayParameter>::SharedPtr
        get_float_array_parameter_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetFloatArrayParameter>::SharedPtr
        set_float_array_parameter_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetROIParameter>::SharedPtr
        get_roi_parameter_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetROIParameter>::SharedPtr
        set_roi_parameter_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetRangeParameter>::SharedPtr
        get_range_parameter_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetRangeParameter>::SharedPtr
        set_range_parameter_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetIntParameter>::SharedPtr
        get_int_parameter_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetIntParameter>::SharedPtr
        set_int_parameter_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetBoolParameter>::SharedPtr
        get_bool_parameter_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetBoolParameter>::SharedPtr
        set_bool_parameter_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetEnumParameter>::SharedPtr
        get_enum_parameter_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetEnumParameter>::SharedPtr
        set_enum_parameter_service;
    rclcpp::Service<mecheye_ros_interface::srv::GetFloatParameter>::SharedPtr
        get_float_parameter_service;
    rclcpp::Service<mecheye_ros_interface::srv::SetFloatParameter>::SharedPtr
        set_float_parameter_service;

    void capture_color_image_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::CaptureColorImage::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::CaptureColorImage::Response> res);

    void capture_textured_point_cloud_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::CaptureTexturedPointCloud::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::CaptureTexturedPointCloud::Response> res);
    void capture_depth_map_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::CaptureDepthMap::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::CaptureDepthMap::Response> res);
    void capture_point_cloud_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::CapturePointCloud::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::CapturePointCloud::Response> res);

    void capture_stereo_color_images_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::CaptureStereoColorImages::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::CaptureStereoColorImages::Response> res);

    void add_user_set_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::AddUserSet::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::AddUserSet::Response> res);

    void delete_user_set_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::DeleteUserSet::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::DeleteUserSet::Response> res);
    void device_info_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::DeviceInfo::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::DeviceInfo::Response> res);

    void get_all_user_sets_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::GetAllUserSets::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::GetAllUserSets::Response> res);
    void get_current_user_set_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::GetCurrentUserSet::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::GetCurrentUserSet::Response> res);

    void save_all_settings_to_user_sets_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::SaveAllSettingsToUserSets::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::SaveAllSettingsToUserSets::Response> res);

    void set_current_user_set_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::SetCurrentUserSet::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::SetCurrentUserSet::Response> res);

    void get_float_array_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::GetFloatArrayParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::GetFloatArrayParameter::Response> res);

    void set_float_array_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::SetFloatArrayParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::SetFloatArrayParameter::Response> res);

    void get_roi_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::GetROIParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::GetROIParameter::Response> res);

    void set_roi_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::SetROIParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::SetROIParameter::Response> res);

    void get_range_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::GetRangeParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::GetRangeParameter::Response> res);

    void set_range_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::SetRangeParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::SetRangeParameter::Response> res);

    void get_int_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::GetIntParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::GetIntParameter::Response> res);

    void set_int_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::SetIntParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::SetIntParameter::Response> res);

    void get_bool_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::GetBoolParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::GetBoolParameter::Response> res);

    void set_bool_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::SetBoolParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::SetBoolParameter::Response> res);

    void get_enum_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::GetEnumParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::GetEnumParameter::Response> res);

    void set_enum_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::SetEnumParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::SetEnumParameter::Response> res);

    void get_float_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::GetFloatParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::GetFloatParameter::Response> res);

    void set_float_parameter_callback(
        const std::shared_ptr<mecheye_ros_interface::srv::SetFloatParameter::Request> req,
        std::shared_ptr<mecheye_ros_interface::srv::SetFloatParameter::Response> res);
};
