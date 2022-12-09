#include <MechMindCamera.h>
#include <SampleUtil.h>
#include <OpenCVUtil.h>
#include <PclUtil.h>
#include <opencv2/imgcodecs.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

MechMindCamera::MechMindCamera()
{
    node = rclcpp::Node::make_shared("mechmind_camera_publisher_service");

    node->declare_parameter<std::string>("camera_ip", "");
    node->declare_parameter<bool>("save_file", false);
    node->declare_parameter<bool>("use_external_intri",false);
    node->declare_parameter<double>("fx", 0.0);
    node->declare_parameter<double>("fy", 0.0);
    node->declare_parameter<double>("u", 0.0);
    node->declare_parameter<double>("v", 0.0);

    node->get_parameter("camera_ip", camera_ip);
    node->get_parameter("save_file", save_file);
    node->get_parameter("use_external_intri", use_external_intri);
    node->get_parameter("fx", fx);
    node->get_parameter("fy", fy);
    node->get_parameter("u", u);
    node->get_parameter("v", v);

    pub_color = node->create_publisher<sensor_msgs::msg::Image>("/mechmind/color_image", 1);
    pub_depth = node->create_publisher<sensor_msgs::msg::Image>("/mechmind/depth_image", 1);
    pub_pcl = node->create_publisher<sensor_msgs::msg::PointCloud2>("/mechmind/point_cloud", 1);
    pub_pcl_color = node->create_publisher<sensor_msgs::msg::PointCloud2>("/mechmind/color_point_cloud", 1);
    pub_camera_info = node->create_publisher<sensor_msgs::msg::CameraInfo>("/mechmind/camera_info", 1);

    if (!findAndConnect(device))
        return;

    // Uncomment the following lines to connect a camera with ip inside .launch file

    // mmind::api::ErrorStatus status;
    // mmind::api::MechEyeDeviceInfo info;
    // info.firmwareVersion = "1.6.0";
    // info.ipAddress = camera_ip;
    // info.port = 5577;
    // status = device.connect(info);
    // if (!status.isOK())
    // {
    //     showError(status);
    //     return;
    // }
    // std::cout << "Connected to the Mech-Eye device successfully." << std::endl;

    mmind::api::MechEyeDeviceInfo deviceInfo;
    showError(device.getDeviceInfo(deviceInfo));
    printDeviceInfo(deviceInfo);

    if (use_external_intri)
    {
        intri.textureCameraIntri.cameraMatrix[0] = fx;
        intri.textureCameraIntri.cameraMatrix[1] = fy;
        intri.textureCameraIntri.cameraMatrix[2] = u;
        intri.textureCameraIntri.cameraMatrix[3] = v;
        intri.depthCameraIntri.cameraMatrix[0] = fx;
        intri.depthCameraIntri.cameraMatrix[1] = fy;
        intri.depthCameraIntri.cameraMatrix[2] = u;
        intri.depthCameraIntri.cameraMatrix[3] = v; 
    }
    else
    {
        showError(device.getDeviceIntri(intri));
    }

    add_user_set_service = node->create_service<mecheye_ros_interface::srv::AddUserSet>("add_user_set", std::bind(&MechMindCamera::add_user_set_callback, this, std::placeholders::_1, std::placeholders::_2));
    capture_color_map_service =
        node->create_service<mecheye_ros_interface::srv::CaptureColorMap>("capture_color_map", std::bind(&MechMindCamera::capture_color_map_callback, this, std::placeholders::_1, std::placeholders::_2));
    capture_color_point_cloud_service = node->create_service<mecheye_ros_interface::srv::CaptureColorPointCloud>("capture_color_point_cloud",
                                                                                                                 std::bind(&MechMindCamera::capture_color_point_cloud_callback, this, std::placeholders::_1, std::placeholders::_2));
    capture_depth_map_service =
        node->create_service<mecheye_ros_interface::srv::CaptureDepthMap>("capture_depth_map", std::bind(&MechMindCamera::capture_depth_map_callback, this, std::placeholders::_1, std::placeholders::_2));
    capture_point_cloud_service =
        node->create_service<mecheye_ros_interface::srv::CapturePointCloud>("capture_point_cloud", std::bind(&MechMindCamera::capture_point_cloud_callback, this, std::placeholders::_1, std::placeholders::_2));
    delete_user_set_service = node->create_service<mecheye_ros_interface::srv::DeleteUserSet>("delete_user_set", std::bind(&MechMindCamera::delete_user_set_callback, this, std::placeholders::_1, std::placeholders::_2));
    device_info_service = node->create_service<mecheye_ros_interface::srv::DeviceInfo>("device_info", std::bind(&MechMindCamera::device_info_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_2d_expected_gray_value_service =
        node->create_service<mecheye_ros_interface::srv::Get2DExpectedGrayValue>("get_2d_expected_gray_value", std::bind(&MechMindCamera::get_2d_expected_gray_value_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_2d_exposure_mode_service =
        node->create_service<mecheye_ros_interface::srv::Get2DExposureMode>("get_2d_exposure_mode", std::bind(&MechMindCamera::get_2d_exposure_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_2d_exposure_sequence_service =
        node->create_service<mecheye_ros_interface::srv::Get2DExposureSequence>("get_2d_exposure_sequence", std::bind(&MechMindCamera::get_2d_exposure_sequence_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_2d_exposure_time_service =
        node->create_service<mecheye_ros_interface::srv::Get2DExposureTime>("get_2d_exposure_time", std::bind(&MechMindCamera::get_2d_exposure_time_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_2d_roi_service = node->create_service<mecheye_ros_interface::srv::Get2DROI>("get_2d_roi", std::bind(&MechMindCamera::get_2d_roi_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_2d_sharpen_factor_service =
        node->create_service<mecheye_ros_interface::srv::Get2DSharpenFactor>("get_2d_sharpen_factor", std::bind(&MechMindCamera::get_2d_sharpen_factor_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_2d_tone_mapping_service =
        node->create_service<mecheye_ros_interface::srv::Get2DToneMappingEnable>("get_2d_tone_mapping", std::bind(&MechMindCamera::get_2d_tone_mapping_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_3d_exposure_service = node->create_service<mecheye_ros_interface::srv::Get3DExposure>("get_3d_exposure", std::bind(&MechMindCamera::get_3d_exposure_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_3d_gain_service = node->create_service<mecheye_ros_interface::srv::Get3DGain>("get_3d_gain", std::bind(&MechMindCamera::get_3d_gain_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_3d_roi_service = node->create_service<mecheye_ros_interface::srv::Get3DROI>("get_3d_roi", std::bind(&MechMindCamera::get_3d_roi_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_all_user_sets_service =
        node->create_service<mecheye_ros_interface::srv::GetAllUserSets>("get_all_user_sets", std::bind(&MechMindCamera::get_all_user_sets_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_cloud_outlier_filter_mode_service = node->create_service<mecheye_ros_interface::srv::GetCloudOutlierFilterMode>(
        "get_cloud_outlier_filter_mode", std::bind(&MechMindCamera::get_cloud_outlier_filter_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_cloud_smooth_mode_service =
        node->create_service<mecheye_ros_interface::srv::GetCloudSmoothMode>("get_cloud_smooth_mode", std::bind(&MechMindCamera::get_cloud_smooth_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_current_user_set_service =
        node->create_service<mecheye_ros_interface::srv::GetCurrentUserSet>("get_current_user_set", std::bind(&MechMindCamera::get_current_user_set_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_depth_range_service = node->create_service<mecheye_ros_interface::srv::GetDepthRange>("get_depth_range", std::bind(&MechMindCamera::get_depth_range_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_fringe_contrast_threshold_service = node->create_service<mecheye_ros_interface::srv::GetFringeContrastThreshold>(
        "get_fringe_contrast_threshold", std::bind(&MechMindCamera::get_fringe_contrast_threshold_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_fringe_min_threshold_service =
        node->create_service<mecheye_ros_interface::srv::GetFringeMinThreshold>("get_fringe_min_threshold", std::bind(&MechMindCamera::get_fringe_min_threshold_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_laser_settings_service =
        node->create_service<mecheye_ros_interface::srv::GetLaserSettings>("get_laser_settings", std::bind(&MechMindCamera::get_laser_settings_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_uhp_settings_service = 
        node->create_service<mecheye_ros_interface::srv::GetUhpSettings>("get_uhp_settings", std::bind(&MechMindCamera::get_uhp_settings_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_uhp_capture_mode_service = 
        node->create_service<mecheye_ros_interface::srv::GetUhpCaptureMode>("get_uhp_capture_mode", std::bind(&MechMindCamera::get_uhp_capture_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_uhp_fringe_coding_mode_service = 
        node->create_service<mecheye_ros_interface::srv::GetUhpFringeCodingMode>("get_uhp_fringe_coding_mode_service", std::bind(&MechMindCamera::get_uhp_fringe_coding_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
        
    get_projector_fringe_coding_mode_service =
        node->create_service<mecheye_ros_interface::srv::GetProjectorFringeCodingMode>("get_projector_fringe_coding_mode", std::bind(&MechMindCamera::get_projector_fringe_coding_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_projector_power_level_service =
        node->create_service<mecheye_ros_interface::srv::GetProjectorPowerLevel>("get_projector_power_level", std::bind(&MechMindCamera::get_projector_power_level_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_projector_anti_flicker_mode_service =
        node->create_service<mecheye_ros_interface::srv::GetProjectorAntiFlickerMode>("get_projector_anti_flicker_mode", std::bind(&MechMindCamera::get_projector_anti_flicker_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    
    save_all_settings_to_user_sets_service = node->create_service<mecheye_ros_interface::srv::SaveAllSettingsToUserSets>(
        "save_all_settings_to_user_sets", std::bind(&MechMindCamera::save_all_settings_to_user_sets_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_2d_expected_gray_value_service =
        node->create_service<mecheye_ros_interface::srv::Set2DExpectedGrayValue>("set_2d_expected_gray_value", std::bind(&MechMindCamera::set_2d_expected_gray_value_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_2d_exposure_mode_service =
        node->create_service<mecheye_ros_interface::srv::Set2DExposureMode>("set_2d_exposure_mode", std::bind(&MechMindCamera::set_2d_exposure_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_2d_exposure_sequence_service =
        node->create_service<mecheye_ros_interface::srv::Set2DExposureSequence>("set_2d_exposure_sequence", std::bind(&MechMindCamera::set_2d_exposure_sequence_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_2d_exposure_time_service =
        node->create_service<mecheye_ros_interface::srv::Set2DExposureTime>("set_2d_exposure_time", std::bind(&MechMindCamera::set_2d_exposure_time_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_2d_roi_service = node->create_service<mecheye_ros_interface::srv::Set2DROI>("set_2d_roi", std::bind(&MechMindCamera::set_2d_roi_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_2d_sharpen_factor_service =
        node->create_service<mecheye_ros_interface::srv::Set2DSharpenFactor>("set_2d_sharpen_factor", std::bind(&MechMindCamera::set_2d_sharpen_factor_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_2d_tone_mapping_service =
        node->create_service<mecheye_ros_interface::srv::Set2DToneMappingEnable>("set_2d_tone_mapping", std::bind(&MechMindCamera::set_2d_tone_mapping_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_3d_exposure_service = node->create_service<mecheye_ros_interface::srv::Set3DExposure>("set_3d_exposure", std::bind(&MechMindCamera::set_3d_exposure_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_3d_gain_service = node->create_service<mecheye_ros_interface::srv::Set3DGain>("set_3d_gain", std::bind(&MechMindCamera::set_3d_gain_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_3d_roi_service = node->create_service<mecheye_ros_interface::srv::Set3DROI>("set_3d_roi", std::bind(&MechMindCamera::set_3d_roi_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_cloud_outlier_filter_mode_service = node->create_service<mecheye_ros_interface::srv::SetCloudOutlierFilterMode>(
        "set_cloud_outlier_filter_mode", std::bind(&MechMindCamera::set_cloud_outlier_filter_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_cloud_smooth_mode_service =
        node->create_service<mecheye_ros_interface::srv::SetCloudSmoothMode>("set_cloud_smooth_mode", std::bind(&MechMindCamera::set_cloud_smooth_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_current_user_set_service =
        node->create_service<mecheye_ros_interface::srv::SetCurrentUserSet>("set_current_user_set", std::bind(&MechMindCamera::set_current_user_set_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_depth_range_service = node->create_service<mecheye_ros_interface::srv::SetDepthRange>("set_depth_range", std::bind(&MechMindCamera::set_depth_range_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_fringe_contrast_threshold_service = node->create_service<mecheye_ros_interface::srv::SetFringeContrastThreshold>(
        "set_fringe_contrast_threshold", std::bind(&MechMindCamera::set_fringe_contrast_threshold_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_fringe_min_threshold_service =
        node->create_service<mecheye_ros_interface::srv::SetFringeMinThreshold>("set_fringe_min_threshold", std::bind(&MechMindCamera::set_fringe_min_threshold_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_laser_settings_service =
        node->create_service<mecheye_ros_interface::srv::SetLaserSettings>("set_laser_settings", std::bind(&MechMindCamera::set_laser_settings_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_uhp_settings_service = 
        node->create_service<mecheye_ros_interface::srv::SetUhpSettings>("set_uhp_settings", std::bind(&MechMindCamera::set_uhp_settings_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_uhp_capture_mode_service = 
        node->create_service<mecheye_ros_interface::srv::SetUhpCaptureMode>("set_uhp_capture_mode", std::bind(&MechMindCamera::set_uhp_capture_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_uhp_fringe_coding_mode_service = 
        node->create_service<mecheye_ros_interface::srv::SetUhpFringeCodingMode>("set_uhp_fringe_coding_mode", std::bind(&MechMindCamera::set_uhp_fringe_coding_mode_callback, this, std::placeholders::_1, std::placeholders::_2));

    set_projector_fringe_coding_mode_service =
        node->create_service<mecheye_ros_interface::srv::SetProjectorFringeCodingMode>("set_projector_fringe_coding_mode", std::bind(&MechMindCamera::set_projector_fringe_coding_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_projector_power_level_service =
        node->create_service<mecheye_ros_interface::srv::SetProjectorPowerLevel>("set_projector_power_level", std::bind(&MechMindCamera::set_projector_power_level_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_projector_anti_flicker_mode_service =
        node->create_service<mecheye_ros_interface::srv::SetProjectorAntiFlickerMode>("set_projector_anti_flicker_mode", std::bind(&MechMindCamera::set_projector_anti_flicker_mode_callback, this, std::placeholders::_1, std::placeholders::_2));
    
}

void MechMindCamera::publishColorMap(mmind::api::ColorMap &colorMap)
{
    cv::Mat color = cv::Mat(colorMap.height(), colorMap.width(), CV_8UC3, colorMap.data());
    cv_bridge::CvImage cv_image;
    cv_image.image = color;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    sensor_msgs::msg::Image ros_image;
    cv_image.toImageMsg(ros_image);
    ros_image.header.frame_id = "mechmind_camera/color_map";
    ros_image.header.stamp = node->now();
    pub_color->publish(ros_image);
    publishCameraInfo(ros_image.header, colorMap.width(), colorMap.height());
    if (save_file)
        saveMap(colorMap, "/tmp/mechmind_color.png");
}

void MechMindCamera::publishDepthMap(mmind::api::DepthMap &depthMap)
{
    cv::Mat depth = cv::Mat(depthMap.height(), depthMap.width(), CV_32FC1, depthMap.data());
    cv_bridge::CvImage cv_depth;
    cv_depth.image = depth;
    cv_depth.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    sensor_msgs::msg::Image ros_depth;
    cv_depth.toImageMsg(ros_depth);
    ros_depth.header.frame_id = "mechmind_camera/depth_map";
    ros_depth.header.stamp = node->now();
    pub_depth->publish(ros_depth);
    publishCameraInfo(ros_depth.header, depthMap.width(), depthMap.height());
    if (save_file)
        saveMap(depthMap, "/tmp/mechmind_depth.tiff");
}

void MechMindCamera::publishPointCloud(mmind::api::PointXYZMap &pointXYZMap)
{
    pcl::PointCloud<pcl::PointXYZ> cloud(pointXYZMap.width(), pointXYZMap.height());
    toPCL(cloud, pointXYZMap);
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud, ros_cloud);
    ros_cloud.header.frame_id = "mechmind_camera/point_cloud";
    ros_cloud.header.stamp = node->now();
    pub_pcl->publish(ros_cloud);
    publishCameraInfo(ros_cloud.header, pointXYZMap.width(), pointXYZMap.height());
    if (save_file)
        savePLY(pointXYZMap, "/tmp/mechmind_cloud.ply");
}

void MechMindCamera::publishColorPointCloud(mmind::api::PointXYZBGRMap &pointXYZBGRMap)
{
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud(pointXYZBGRMap.width(), pointXYZBGRMap.height());
    toPCL(color_cloud, pointXYZBGRMap);
    sensor_msgs::msg::PointCloud2 ros_color_cloud;
    pcl::toROSMsg(color_cloud, ros_color_cloud);
    ros_color_cloud.header.frame_id = "mechmind_camera/color_point_cloud";
    ros_color_cloud.header.stamp = node->now();
    pub_pcl_color->publish(ros_color_cloud);
    publishCameraInfo(ros_color_cloud.header, pointXYZBGRMap.width(), pointXYZBGRMap.height());
    if (save_file)
        savePLY(pointXYZBGRMap, "/tmp/mechmind_color_cloud.ply");
}

void MechMindCamera::publishCameraInfo(const std_msgs::msg::Header &header, int width, int height)
{
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header = header;
    camera_info.height = height;
    camera_info.width = width;
    camera_info.distortion_model = "plumb_bob";

    camera_info.d = std::vector<double>(intri.textureCameraIntri.distortion, intri.textureCameraIntri.distortion + 5);

    std::vector<double> K{intri.textureCameraIntri.cameraMatrix[0],
                          0.0,
                          intri.textureCameraIntri.cameraMatrix[2],
                          0.0,
                          intri.textureCameraIntri.cameraMatrix[1],
                          intri.textureCameraIntri.cameraMatrix[3],
                          0.0,
                          0.0,
                          1.0};
    for (size_t i = 0; i < 9; ++i)
    {
        camera_info.k[i] = K[i];
    }

    std::vector<double> R{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    for (size_t i = 0; i < 9; ++i)
    {
        camera_info.r[i] = R[i];
    }

    std::vector<double> P{intri.textureCameraIntri.cameraMatrix[0],
                          0.0,
                          intri.textureCameraIntri.cameraMatrix[2],
                          0.0,
                          0.0,
                          intri.textureCameraIntri.cameraMatrix[1],
                          intri.textureCameraIntri.cameraMatrix[3],
                          0.0,
                          0.0,
                          0.0,
                          1.0,
                          0.0};
    for (size_t i = 0; i < 12; ++i)
    {
        camera_info.p[i] = P[i];
    }
    pub_camera_info->publish(camera_info);
}

void MechMindCamera::add_user_set_callback(const std::shared_ptr<mecheye_ros_interface::srv::AddUserSet::Request> req, std::shared_ptr<mecheye_ros_interface::srv::AddUserSet::Response> res)
{
    mmind::api::ErrorStatus status = device.addUserSet({req->value.c_str()});
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::capture_color_map_callback(const std::shared_ptr<mecheye_ros_interface::srv::CaptureColorMap::Request> req, std::shared_ptr<mecheye_ros_interface::srv::CaptureColorMap::Response> res)
{
    mmind::api::ColorMap colorMap;
    mmind::api::ErrorStatus status = device.captureColorMap(colorMap);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
    publishColorMap(colorMap);
}

void MechMindCamera::capture_color_point_cloud_callback(const std::shared_ptr<mecheye_ros_interface::srv::CaptureColorPointCloud::Request> req,
                                                        std::shared_ptr<mecheye_ros_interface::srv::CaptureColorPointCloud::Response> res)
{
    mmind::api::PointXYZBGRMap pointXYZBGRMap;
    mmind::api::ErrorStatus status = device.capturePointXYZBGRMap(pointXYZBGRMap);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
    publishColorPointCloud(pointXYZBGRMap);
}

void MechMindCamera::capture_depth_map_callback(const std::shared_ptr<mecheye_ros_interface::srv::CaptureDepthMap::Request> req, std::shared_ptr<mecheye_ros_interface::srv::CaptureDepthMap::Response> res)
{
    mmind::api::DepthMap depthMap;
    mmind::api::ErrorStatus status = device.captureDepthMap(depthMap);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
    publishDepthMap(depthMap);
}

void MechMindCamera::capture_point_cloud_callback(const std::shared_ptr<mecheye_ros_interface::srv::CapturePointCloud::Request> req, std::shared_ptr<mecheye_ros_interface::srv::CapturePointCloud::Response> res)
{
    mmind::api::PointXYZMap pointXYZMap;
    mmind::api::ErrorStatus status = device.capturePointXYZMap(pointXYZMap);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
    publishPointCloud(pointXYZMap);
}

void MechMindCamera::delete_user_set_callback(const std::shared_ptr<mecheye_ros_interface::srv::DeleteUserSet::Request> req, std::shared_ptr<mecheye_ros_interface::srv::DeleteUserSet::Response> res)
{
    mmind::api::ErrorStatus status = device.deleteUserSet({req->value.c_str()});
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::device_info_callback(const std::shared_ptr<mecheye_ros_interface::srv::DeviceInfo::Request> req, std::shared_ptr<mecheye_ros_interface::srv::DeviceInfo::Response> res)
{
    mmind::api::MechEyeDeviceInfo deviceInfo;
    mmind::api::ErrorStatus status = device.getDeviceInfo(deviceInfo);
    showError(status);
    res->model = deviceInfo.model.c_str();
    res->id = deviceInfo.id.c_str();
    res->hardware_version = deviceInfo.hardwareVersion.c_str();
    res->firmware_version = deviceInfo.firmwareVersion.c_str();
    res->ip_address = deviceInfo.ipAddress.c_str();
    res->port = deviceInfo.port;
}

void MechMindCamera::get_2d_expected_gray_value_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DExpectedGrayValue::Request> req,
                                                         std::shared_ptr<mecheye_ros_interface::srv::Get2DExpectedGrayValue::Response> res)
{
    int value;
    mmind::api::ErrorStatus status = device.getScan2DExpectedGrayValue(value);
    showError(status);
    res->value = value;
}

void MechMindCamera::get_2d_exposure_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DExposureMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get2DExposureMode::Response> res)
{
    mmind::api::Scanning2DSettings::Scan2DExposureMode mode;
    mmind::api::ErrorStatus status = device.getScan2DExposureMode(mode);
    showError(status);
    switch (mode)
    {
    case mmind::api::Scanning2DSettings::Scan2DExposureMode::Timed:
        res->value = "Timed";
        break;

    case mmind::api::Scanning2DSettings::Scan2DExposureMode::Auto:
        res->value = "Auto";
        break;

    case mmind::api::Scanning2DSettings::Scan2DExposureMode::HDR:
        res->value = "HDR";
        break;

    case mmind::api::Scanning2DSettings::Scan2DExposureMode::Flash:
        res->value = "Flash";
        break;

    default:
        res->value = "";
        break;
    }
}

void MechMindCamera::get_2d_exposure_sequence_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DExposureSequence::Request> req,
                                                       std::shared_ptr<mecheye_ros_interface::srv::Get2DExposureSequence::Response> res)
{
    std::vector<double> sequence;
    mmind::api::ErrorStatus status = device.getScan2DHDRExposureSequence(sequence);
    showError(status);
    res->sequence = sequence;
}

void MechMindCamera::get_2d_exposure_time_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DExposureTime::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get2DExposureTime::Response> res)
{
    double value = -1;
    mmind::api::ErrorStatus status = device.getScan2DExposureTime(value);
    showError(status);
    res->value = value;
}

void MechMindCamera::get_2d_roi_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DROI::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get2DROI::Response> res)
{
    mmind::api::ROI roi;
    mmind::api::ErrorStatus status = device.getScan2DROI(roi);
    showError(status);
    res->x = roi.x;
    res->y = roi.y;
    res->width = roi.width;
    res->height = roi.height;
}

void MechMindCamera::get_2d_sharpen_factor_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DSharpenFactor::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get2DSharpenFactor::Response> res)
{
    double value = -1;
    mmind::api::ErrorStatus status = device.getScan2DSharpenFactor(value);
    showError(status);
    res->value = value;
}

void MechMindCamera::get_2d_tone_mapping_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get2DToneMappingEnable::Request> req,
                                                  std::shared_ptr<mecheye_ros_interface::srv::Get2DToneMappingEnable::Response> res)
{
    bool value;
    mmind::api::ErrorStatus status = device.getScan2DToneMappingEnable(value);
    showError(status);
    res->value = value;
}

void MechMindCamera::get_3d_exposure_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get3DExposure::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get3DExposure::Response> res)
{
    std::vector<double> sequence;
    mmind::api::ErrorStatus status = device.getScan3DExposure(sequence);
    showError(status);
    res->sequence = sequence;
}

void MechMindCamera::get_3d_gain_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get3DGain::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get3DGain::Response> res)
{
    double value = -1;
    mmind::api::ErrorStatus status = device.getScan3DGain(value);
    showError(status);
    res->value = value;
}

void MechMindCamera::get_3d_roi_callback(const std::shared_ptr<mecheye_ros_interface::srv::Get3DROI::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Get3DROI::Response> res)
{
    mmind::api::ROI roi;
    mmind::api::ErrorStatus status = device.getScan3DROI(roi);
    showError(status);
    res->x = roi.x;
    res->y = roi.y;
    res->width = roi.width;
    res->height = roi.height;
}

void MechMindCamera::get_all_user_sets_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetAllUserSets::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetAllUserSets::Response> res)
{
    std::vector<std::string> sequence;
    mmind::api::ErrorStatus status = device.getAllUserSets(sequence);
    showError(status);
    res->sequence = sequence;
}

void MechMindCamera::get_cloud_outlier_filter_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetCloudOutlierFilterMode::Request> req,
                                                            std::shared_ptr<mecheye_ros_interface::srv::GetCloudOutlierFilterMode::Response> res)
{
    mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode mode;
    mmind::api::ErrorStatus status = device.getCloudOutlierFilterMode(mode);
    showError(status);
    switch (mode)
    {
    case mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Off:
        res->value = "Off";
        break;

    case mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Normal:
        res->value = "Normal";
        break;

    case mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Weak:
        res->value = "Weak";
        break;

    default:
        res->value = "";
        break;
    }
}

void MechMindCamera::get_cloud_smooth_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetCloudSmoothMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetCloudSmoothMode::Response> res)
{
    mmind::api::PointCloudProcessingSettings::CloudSmoothMode mode;
    mmind::api::ErrorStatus status = device.getCloudSmoothMode(mode);
    showError(status);
    switch (mode)
    {
    case mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Off:
        res->value = "Off";
        break;

    case mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Normal:
        res->value = "Normal";
        break;

    case mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Weak:
        res->value = "Weak";
        break;

    case mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Strong:
        res->value = "Strong";
        break;

    default:
        res->value = "";
        break;
    }
}

void MechMindCamera::get_current_user_set_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetCurrentUserSet::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetCurrentUserSet::Response> res)
{
    std::string value;
    mmind::api::ErrorStatus status = device.getCurrentUserSet(value);
    showError(status);
    res->value = value.c_str();
}

void MechMindCamera::get_depth_range_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetDepthRange::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetDepthRange::Response> res)
{
    mmind::api::DepthRange depthRange;
    mmind::api::ErrorStatus status = device.getDepthRange(depthRange);
    showError(status);
    res->lower = depthRange.lower;
    res->upper = depthRange.upper;
}

void MechMindCamera::get_fringe_contrast_threshold_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetFringeContrastThreshold::Request> req,
                                                            std::shared_ptr<mecheye_ros_interface::srv::GetFringeContrastThreshold::Response> res)
{
    int value;
    mmind::api::ErrorStatus status = device.getFringeContrastThreshold(value);
    showError(status);
    res->value = value;
}

void MechMindCamera::get_fringe_min_threshold_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetFringeMinThreshold::Request> req,
                                                       std::shared_ptr<mecheye_ros_interface::srv::GetFringeMinThreshold::Response> res)
{
    int value;
    mmind::api::ErrorStatus status = device.getFringeMinThreshold(value);
    showError(status);
    res->value = value;
}

void MechMindCamera::get_laser_settings_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetLaserSettings::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetLaserSettings::Response> res)
{
    mmind::api::LaserSettings laserSettings;
    mmind::api::ErrorStatus status = device.getLaserSettings(laserSettings);
    showError(status);
    switch (laserSettings.fringeCodingMode)
    {
    case mmind::api::LaserSettings::LaserFringeCodingMode::Fast:
        res->fringe_coding_mode = "Fast";
        break;

    case mmind::api::LaserSettings::LaserFringeCodingMode::Accurate:
        res->fringe_coding_mode = "Accurate";
        break;

    default:
        res->fringe_coding_mode = "";
        break;
    }
    res->frame_range_start = laserSettings.frameRangeStart;
    res->frame_range_end = laserSettings.frameRangeEnd;
    res->frame_partition_count = laserSettings.framePartitionCount;
    res->power_level = laserSettings.powerLevel;
}

void MechMindCamera::get_uhp_settings_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetUhpSettings::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetUhpSettings::Response> res)
{
    mmind::api::UhpSettings uhpSettings;
    mmind::api::ErrorStatus status = device.getUhpSettings(uhpSettings);
    showError(status);
    switch (uhpSettings.captureMode)
    {
    case mmind::api::UhpSettings::UhpCaptureMode::Camera1:
        res->capture_mode = "Camera1";
        break;

    case mmind::api::UhpSettings::UhpCaptureMode::Camera2:
        res->capture_mode = "Camera2";
        break;

    case mmind::api::UhpSettings::UhpCaptureMode::Merge:
        res->capture_mode = "Merge";
        break;
    
    default:
        res->capture_mode = "";
        break;
    }
    switch (uhpSettings.fringeCodingMode)
    {
    case mmind::api::UhpSettings::UhpFringeCodingMode::Fast:
        res->fringe_coding_mode = "Fast";
        break;

    case mmind::api::UhpSettings::UhpFringeCodingMode::Accurate:
        res->fringe_coding_mode = "Accurate";
        break;

    default:
        res->fringe_coding_mode = "";
        break;
    }
}

void MechMindCamera::get_uhp_capture_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetUhpCaptureMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetUhpCaptureMode::Response> res)
{
    mmind::api::UhpSettings::UhpCaptureMode mode;
    mmind::api::ErrorStatus status = device.getUhpCaptureMode(mode);
    showError(status);
    switch (mode)
    {
    case mmind::api::UhpSettings::UhpCaptureMode::Camera1:
        res->capture_mode = "Camera1";
        break;

    case mmind::api::UhpSettings::UhpCaptureMode::Camera2:
        res->capture_mode = "Camera2";
        break;

    case mmind::api::UhpSettings::UhpCaptureMode::Merge:
        res->capture_mode = "Merge";
        break;
    
    default:
        res->capture_mode = "";
        break;
    }
}

void MechMindCamera::get_uhp_fringe_coding_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetUhpFringeCodingMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetUhpFringeCodingMode::Response> res)
{
    mmind::api::UhpSettings::UhpFringeCodingMode mode;
    mmind::api::ErrorStatus status = device.getUhpFringeCodingMode(mode);
    showError(status);
    switch (mode)
    {
    case mmind::api::UhpSettings::UhpFringeCodingMode::Fast:
        res->fringe_coding_mode = "Fast";
        break;

    case mmind::api::UhpSettings::UhpFringeCodingMode::Accurate:
        res->fringe_coding_mode = "Accurate";
        break;
    
    default:
        res->fringe_coding_mode = "";
        break;
    }
}


void MechMindCamera::get_projector_fringe_coding_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetProjectorFringeCodingMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetProjectorFringeCodingMode::Response> res)
{
    mmind::api::ProjectorSettings::FringeCodingMode mode;
    mmind::api::ErrorStatus status = device.getProjectorFringeCodingMode(mode);
    showError(status);
    switch (mode)
    {
    case mmind::api::ProjectorSettings::FringeCodingMode::Fast:
        res->value = "Fast";
        break;

    case mmind::api::ProjectorSettings::FringeCodingMode::Accurate:
        res->value = "Accurate";
        break;

    default:
        res->value = "";
        break;
    }
}
void MechMindCamera::get_projector_power_level_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetProjectorPowerLevel::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetProjectorPowerLevel::Response> res)
{
    mmind::api::ProjectorSettings::PowerLevel mode;
    mmind::api::ErrorStatus status = device.getProjectorPowerLevel(mode);
    showError(status);
    switch (mode)
    {
    case mmind::api::ProjectorSettings::PowerLevel::High:
        res->value = "High";
        break;

    case mmind::api::ProjectorSettings::PowerLevel::Normal:
        res->value = "Normal";
        break;

    case mmind::api::ProjectorSettings::PowerLevel::Low:
        res->value = "Low";
        break;

    default:
        res->value = "";
        break;
    }
}
void MechMindCamera::get_projector_anti_flicker_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::GetProjectorAntiFlickerMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::GetProjectorAntiFlickerMode::Response> res)
{
    mmind::api::ProjectorSettings::AntiFlickerMode mode;
    mmind::api::ErrorStatus status = device.getProjectorAntiFlickerMode(mode);
    showError(status);
    switch (mode)
    {
    case mmind::api::ProjectorSettings::AntiFlickerMode::Off:
        res->value = "Off";
        break;

    case mmind::api::ProjectorSettings::AntiFlickerMode::AC50Hz:
        res->value = "AC50Hz";
        break;

    case mmind::api::ProjectorSettings::AntiFlickerMode::AC60Hz:
        res->value = "AC60Hz";
        break;

    default:
        res->value = "";
        break;
    }
}



void MechMindCamera::save_all_settings_to_user_sets_callback(const std::shared_ptr<mecheye_ros_interface::srv::SaveAllSettingsToUserSets::Request> req,
                                                             std::shared_ptr<mecheye_ros_interface::srv::SaveAllSettingsToUserSets::Response> res) {}

void MechMindCamera::set_2d_expected_gray_value_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DExpectedGrayValue::Request> req,
                                                         std::shared_ptr<mecheye_ros_interface::srv::Set2DExpectedGrayValue::Response> res)
{
    mmind::api::ErrorStatus status = device.setScan2DExpectedGrayValue(req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_2d_exposure_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DExposureMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set2DExposureMode::Response> res)
{
    mmind::api::Scanning2DSettings::Scan2DExposureMode mode;
    if (req->value == "Timed")
        mode = mmind::api::Scanning2DSettings::Scan2DExposureMode::Timed;
    else if (req->value == "Auto")
        mode = mmind::api::Scanning2DSettings::Scan2DExposureMode::Auto;
    else if (req->value == "HDR")
        mode = mmind::api::Scanning2DSettings::Scan2DExposureMode::HDR;
    else if (req->value == "Flash")
        mode = mmind::api::Scanning2DSettings::Scan2DExposureMode::Flash;
    else
    {
        res->error_code = -4;
        res->error_description = "Invalid parameter";
        return;
    }
    mmind::api::ErrorStatus status = device.setScan2DExposureMode(mode);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_2d_exposure_sequence_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DExposureSequence::Request> req,
                                                       std::shared_ptr<mecheye_ros_interface::srv::Set2DExposureSequence::Response> res)
{
    std::vector<double> sequence(begin(req->sequence), end(req->sequence));
    mmind::api::ErrorStatus status = device.setScan2DHDRExposureSequence(sequence);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_2d_exposure_time_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DExposureTime::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set2DExposureTime::Response> res)
{
    mmind::api::ErrorStatus status = device.setScan2DExposureTime(req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_2d_roi_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DROI::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set2DROI::Response> res)
{
    mmind::api::ROI roi{req->x, req->y, req->width, req->height};
    mmind::api::ErrorStatus status = device.setScan2DROI(roi);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_2d_sharpen_factor_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DSharpenFactor::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set2DSharpenFactor::Response> res)
{
    mmind::api::ErrorStatus status = device.setScan2DSharpenFactor(req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_2d_tone_mapping_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set2DToneMappingEnable::Request> req,
                                                  std::shared_ptr<mecheye_ros_interface::srv::Set2DToneMappingEnable::Response> res)
{
    mmind::api::ErrorStatus status = device.setScan2DToneMappingEnable(req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_3d_exposure_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set3DExposure::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set3DExposure::Response> res)
{
    std::vector<double> sequence(begin(req->sequence), end(req->sequence));
    mmind::api::ErrorStatus status = device.setScan3DExposure(sequence);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_3d_gain_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set3DGain::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set3DGain::Response> res)
{
    mmind::api::ErrorStatus status = device.setScan3DGain(req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_3d_roi_callback(const std::shared_ptr<mecheye_ros_interface::srv::Set3DROI::Request> req, std::shared_ptr<mecheye_ros_interface::srv::Set3DROI::Response> res)
{
    mmind::api::ROI roi{req->x, req->y, req->width, req->height};
    mmind::api::ErrorStatus status = device.setScan3DROI(roi);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_cloud_outlier_filter_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetCloudOutlierFilterMode::Request> req,
                                                            std::shared_ptr<mecheye_ros_interface::srv::SetCloudOutlierFilterMode::Response> res)
{
    mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode mode;
    if (req->value == "Off")
        mode = mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Off;
    else if (req->value == "Normal")
        mode = mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Normal;
    else if (req->value == "Weak")
        mode = mmind::api::PointCloudProcessingSettings::CloudOutlierFilterMode::Weak;
    else
    {
        res->error_code = -4;
        res->error_description = "Invalid parameter";
        return;
    }
    mmind::api::ErrorStatus status = device.setCloudOutlierFilterMode(mode);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_cloud_smooth_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetCloudSmoothMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetCloudSmoothMode::Response> res)
{
    mmind::api::PointCloudProcessingSettings::CloudSmoothMode mode;
    if (req->value == "Off")
        mode = mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Off;
    else if (req->value == "Normal")
        mode = mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Normal;
    else if (req->value == "Weak")
        mode = mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Weak;
    else if (req->value == "Strong")
        mode = mmind::api::PointCloudProcessingSettings::CloudSmoothMode::Strong;
    else
    {
        res->error_code = -4;
        res->error_description = "Invalid parameter";
        return;
    }
    mmind::api::ErrorStatus status = device.setCloudSmoothMode(mode);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_current_user_set_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetCurrentUserSet::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetCurrentUserSet::Response> res)
{
    mmind::api::ErrorStatus status = device.setCurrentUserSet({req->value.c_str()});
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_depth_range_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetDepthRange::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetDepthRange::Response> res)
{
    mmind::api::DepthRange depthRange{req->lower, req->upper};
    mmind::api::ErrorStatus status = device.setDepthRange(depthRange);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_fringe_contrast_threshold_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetFringeContrastThreshold::Request> req,
                                                            std::shared_ptr<mecheye_ros_interface::srv::SetFringeContrastThreshold::Response> res)
{
    mmind::api::ErrorStatus status = device.setFringeContrastThreshold(req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_fringe_min_threshold_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetFringeMinThreshold::Request> req,
                                                       std::shared_ptr<mecheye_ros_interface::srv::SetFringeMinThreshold::Response> res)
{
    mmind::api::ErrorStatus status = device.setFringeMinThreshold(req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_laser_settings_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetLaserSettings::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetLaserSettings::Response> res)
{
    mmind::api::LaserSettings laserSettings{
        req->fringe_coding_mode == "Fast" ? mmind::api::LaserSettings::LaserFringeCodingMode::Fast : mmind::api::LaserSettings::LaserFringeCodingMode::Accurate, req->frame_range_start,
                                            req->frame_range_end, req->frame_partition_count, req->power_level
                                            };
    mmind::api::ErrorStatus status = device.setLaserSettings(laserSettings);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_uhp_settings_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetUhpSettings::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetUhpSettings::Response> res)
{
    mmind::api::UhpSettings uhpSettings{
        req->capture_mode == "Camera1" ? mmind::api::UhpSettings::UhpCaptureMode::Camera1 : (req->capture_mode == "Camera2" ? mmind::api::UhpSettings::UhpCaptureMode::Camera2 : mmind::api::UhpSettings::UhpCaptureMode::Merge),
        req->fringe_coding_mode == "Fast" ? mmind::api::UhpSettings::UhpFringeCodingMode::Fast : mmind::api::UhpSettings::UhpFringeCodingMode::Accurate
        };
    mmind::api::ErrorStatus status = device.setUhpSettings(uhpSettings);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_uhp_capture_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetUhpCaptureMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetUhpCaptureMode::Response> res)
{
    mmind::api::UhpSettings::UhpCaptureMode mode;
    if (req->capture_mode == "Camera1")
        mode = mmind::api::UhpSettings::UhpCaptureMode::Camera1;
    else if (req->capture_mode == "Camera2")
        mode = mmind::api::UhpSettings::UhpCaptureMode::Camera2;
    else if (req->capture_mode == "Merge")
        mode = mmind::api::UhpSettings::UhpCaptureMode::Merge;
    else
    {
        res->error_code = -4;
        res->error_description = "Invalid parameter";
        return;
    }
    mmind::api::ErrorStatus status = device.setUhpCaptureMode(mode);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::set_uhp_fringe_coding_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetUhpFringeCodingMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetUhpFringeCodingMode::Response> res)
{
    mmind::api::UhpSettings::UhpFringeCodingMode mode;
    if (req->fringe_coding_mode == "Fast")
        mode = mmind::api::UhpSettings::UhpFringeCodingMode::Fast;
    else if (req->fringe_coding_mode == "Accurate")
        mode = mmind::api::UhpSettings::UhpFringeCodingMode::Accurate;
    else
    {
        res->error_code = -4;
        res->error_description = "Invalid parameter";
        return;
    }
    mmind::api::ErrorStatus status = device.setUhpFringeCodingMode(mode);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}



void MechMindCamera::set_projector_fringe_coding_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetProjectorFringeCodingMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetProjectorFringeCodingMode::Response> res)
{
    mmind::api::ProjectorSettings::FringeCodingMode mode;
    if (req->value == "Fast")
        mode = mmind::api::ProjectorSettings::FringeCodingMode::Fast;
    else if (req->value == "Accurate")
        mode = mmind::api::ProjectorSettings::FringeCodingMode::Accurate;
    else
    {
        res->error_code = -4;
        res->error_description = "Invalid parameter";
        return;
    }
    mmind::api::ErrorStatus status = device.setProjectorFringeCodingMode(mode);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}
void MechMindCamera::set_projector_power_level_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetProjectorPowerLevel::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetProjectorPowerLevel::Response> res)
{
    mmind::api::ProjectorSettings::PowerLevel mode;
    if (req->value == "High")
        mode = mmind::api::ProjectorSettings::PowerLevel::High;
    else if (req->value == "Normal")
        mode = mmind::api::ProjectorSettings::PowerLevel::Normal;
    else if (req->value == "Low")
        mode = mmind::api::ProjectorSettings::PowerLevel::Low;
    else
    {
        res->error_code = -4;
        res->error_description = "Invalid parameter";
        return;
    }
    mmind::api::ErrorStatus status = device.setProjectorPowerLevel(mode);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}
void MechMindCamera::set_projector_anti_flicker_mode_callback(const std::shared_ptr<mecheye_ros_interface::srv::SetProjectorAntiFlickerMode::Request> req, std::shared_ptr<mecheye_ros_interface::srv::SetProjectorAntiFlickerMode::Response> res)
{
    mmind::api::ProjectorSettings::AntiFlickerMode mode;
    if (req->value == "Off")
        mode = mmind::api::ProjectorSettings::AntiFlickerMode::Off;
    else if (req->value == "AC50Hz")
        mode = mmind::api::ProjectorSettings::AntiFlickerMode::AC50Hz;
    else if (req->value == "AC60Hz")
        mode = mmind::api::ProjectorSettings::AntiFlickerMode::AC60Hz;
    else
    {
        res->error_code = -4;
        res->error_description = "Invalid parameter";
        return;
    }
    mmind::api::ErrorStatus status = device.setProjectorAntiFlickerMode(mode);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}
