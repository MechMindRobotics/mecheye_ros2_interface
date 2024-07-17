#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/string.hpp>
#include <area_scan_3d_camera/api_util.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <MechMindCamera.h>

namespace {

sensor_msgs::msg::PointField createPointField(std::string name, uint32_t offset, uint8_t datatype,
                                              uint32_t count)
{
    sensor_msgs::msg::PointField point_field;
    point_field.name = name;
    point_field.offset = offset;
    point_field.datatype = datatype;
    point_field.count = count;
    return point_field;
}

void convertToROSMsg(const mmind::eye::TexturedPointCloud& texturedPointCloud,
                     sensor_msgs::msg::PointCloud2& cloud)
{
    cloud.height = texturedPointCloud.height();
    cloud.width = texturedPointCloud.width();
    cloud.is_dense = false;
    cloud.is_bigendian = BOOST_ENDIAN_BIG_BYTE;
    cloud.point_step = sizeof(mmind::eye::PointXYZBGR);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);

    cloud.fields.reserve(4);
    cloud.fields.push_back(createPointField("x", offsetof(mmind::eye::PointXYZBGR, x),
                                            sensor_msgs::msg::PointField::FLOAT32, 1));
    cloud.fields.push_back(createPointField("y", offsetof(mmind::eye::PointXYZBGR, y),
                                            sensor_msgs::msg::PointField::FLOAT32, 1));
    cloud.fields.push_back(createPointField("z", offsetof(mmind::eye::PointXYZBGR, z),
                                            sensor_msgs::msg::PointField::FLOAT32, 1));
    cloud.fields.push_back(createPointField("rgb", offsetof(mmind::eye::PointXYZBGR, rgb),
                                            sensor_msgs::msg::PointField::FLOAT32, 1));

    memcpy(
        cloud.data.data(),
        reinterpret_cast<uint8_t*>(const_cast<mmind::eye::PointXYZBGR*>(texturedPointCloud.data())),
        (cloud.row_step * cloud.height));
}

void convertToROSMsg(const mmind::eye::PointCloud& pointCloud, sensor_msgs::msg::PointCloud2& cloud)
{
    cloud.height = pointCloud.height();
    cloud.width = pointCloud.width();
    cloud.is_dense = false;
    cloud.is_bigendian = BOOST_ENDIAN_BIG_BYTE;
    cloud.point_step = sizeof(mmind::eye::PointXYZ);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);

    cloud.fields.reserve(3);
    cloud.fields.push_back(createPointField("x", offsetof(mmind::eye::PointXYZ, x),
                                            sensor_msgs::msg::PointField::FLOAT32, 1));
    cloud.fields.push_back(createPointField("y", offsetof(mmind::eye::PointXYZ, y),
                                            sensor_msgs::msg::PointField::FLOAT32, 1));
    cloud.fields.push_back(createPointField("z", offsetof(mmind::eye::PointXYZ, z),
                                            sensor_msgs::msg::PointField::FLOAT32, 1));
    memcpy(cloud.data.data(),
           reinterpret_cast<uint8_t*>(const_cast<mmind::eye::PointXYZ*>(pointCloud.data())),
           (cloud.row_step * cloud.height));
}

} // namespace

MechMindCamera::MechMindCamera()
{
    node = rclcpp::Node::make_shared("mechmind_camera_publisher_service");

    node->declare_parameter<std::string>("camera_ip", "");
    node->declare_parameter<bool>("save_file", false);
    node->declare_parameter<bool>("use_external_intri", false);
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
    pub_color_left =
        node->create_publisher<sensor_msgs::msg::Image>("/mechmind/stereo_color_image_left", 1);
    pub_color_right =
        node->create_publisher<sensor_msgs::msg::Image>("/mechmind/stereo_color_image_right", 1);
    pub_depth = node->create_publisher<sensor_msgs::msg::Image>("/mechmind/depth_map", 1);
    pub_pcl = node->create_publisher<sensor_msgs::msg::PointCloud2>("/mechmind/point_cloud", 1);
    pub_pcl_color =
        node->create_publisher<sensor_msgs::msg::PointCloud2>("/mechmind/textured_point_cloud", 1);
    pub_camera_info =
        node->create_publisher<sensor_msgs::msg::CameraInfo>("/mechmind/camera_info", 1);

    if (!findAndConnect(camera))
        throw mmind::eye::ErrorStatus{mmind::eye::ErrorStatus::MMIND_STATUS_INVALID_DEVICE,
                                      "Camera not found."};

    // Uncomment the following lines and comment the above if function to connect to a specific
    // camera by its IP address. The IP address is set in the "start_camera.launch" file as the
    // value of the "camera_ip" argument.

    // mmind::eye::ErrorStatus status;
    // mmind::eye::CameraInfo info;
    // info.firmwareVersion = mmind::eye::Version("2.3.4");
    // info.ipAddress = camera_ip;
    // info.port = 5577;
    // status = camera.connect(info);
    // if (!status.isOK())
    // {
    //     throw status;
    // }
    // std::cout << "Connected to the camera successfully." << std::endl;

    mmind::eye::CameraInfo cameraInfo;
    showError(camera.getCameraInfo(cameraInfo));
    printCameraInfo(cameraInfo);

    if (use_external_intri) {
        intrinsics.texture.cameraMatrix.fx = fx;
        intrinsics.texture.cameraMatrix.fy = fy;
        intrinsics.texture.cameraMatrix.cx = u;
        intrinsics.texture.cameraMatrix.cy = v;
        intrinsics.depth.cameraMatrix.fx = fx;
        intrinsics.depth.cameraMatrix.fy = fy;
        intrinsics.depth.cameraMatrix.cx = u;
        intrinsics.depth.cameraMatrix.cy = v;
    } else {
        showError(camera.getCameraIntrinsics(intrinsics));
    }

    camera.setPointCloudUnit(mmind::eye::CoordinateUnit::Meter);
    capture_color_image_service =
        node->create_service<mecheye_ros_interface::srv::CaptureColorImage>(
            "capture_color_image", std::bind(&MechMindCamera::capture_color_image_callback, this,
                                             std::placeholders::_1, std::placeholders::_2));
    capture_stereo_color_images_service =
        node->create_service<mecheye_ros_interface::srv::CaptureStereoColorImages>(
            "capture_stereo_color_images",
            std::bind(&MechMindCamera::capture_stereo_color_images_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
    capture_textured_point_cloud_service =
        node->create_service<mecheye_ros_interface::srv::CaptureTexturedPointCloud>(
            "capture_textured_point_cloud",
            std::bind(&MechMindCamera::capture_textured_point_cloud_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
    capture_depth_map_service = node->create_service<mecheye_ros_interface::srv::CaptureDepthMap>(
        "capture_depth_map", std::bind(&MechMindCamera::capture_depth_map_callback, this,
                                       std::placeholders::_1, std::placeholders::_2));
    capture_point_cloud_service =
        node->create_service<mecheye_ros_interface::srv::CapturePointCloud>(
            "capture_point_cloud", std::bind(&MechMindCamera::capture_point_cloud_callback, this,
                                             std::placeholders::_1, std::placeholders::_2));

    add_user_set_service = node->create_service<mecheye_ros_interface::srv::AddUserSet>(
        "add_user_set", std::bind(&MechMindCamera::add_user_set_callback, this,
                                  std::placeholders::_1, std::placeholders::_2));

    delete_user_set_service = node->create_service<mecheye_ros_interface::srv::DeleteUserSet>(
        "delete_user_set", std::bind(&MechMindCamera::delete_user_set_callback, this,
                                     std::placeholders::_1, std::placeholders::_2));
    device_info_service = node->create_service<mecheye_ros_interface::srv::DeviceInfo>(
        "device_info", std::bind(&MechMindCamera::device_info_callback, this, std::placeholders::_1,
                                 std::placeholders::_2));

    get_all_user_sets_service = node->create_service<mecheye_ros_interface::srv::GetAllUserSets>(
        "get_all_user_sets", std::bind(&MechMindCamera::get_all_user_sets_callback, this,
                                       std::placeholders::_1, std::placeholders::_2));

    get_current_user_set_service =
        node->create_service<mecheye_ros_interface::srv::GetCurrentUserSet>(
            "get_current_user_set", std::bind(&MechMindCamera::get_current_user_set_callback, this,
                                              std::placeholders::_1, std::placeholders::_2));

    save_all_settings_to_user_sets_service =
        node->create_service<mecheye_ros_interface::srv::SaveAllSettingsToUserSets>(
            "save_all_settings_to_user_sets",
            std::bind(&MechMindCamera::save_all_settings_to_user_sets_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

    set_current_user_set_service =
        node->create_service<mecheye_ros_interface::srv::SetCurrentUserSet>(
            "set_current_user_set", std::bind(&MechMindCamera::set_current_user_set_callback, this,
                                              std::placeholders::_1, std::placeholders::_2));

    set_int_parameter_service = node->create_service<mecheye_ros_interface::srv::SetIntParameter>(
        "set_int_parameter", std::bind(&MechMindCamera::set_int_parameter_callback, this,
                                       std::placeholders::_1, std::placeholders::_2));

    get_int_parameter_service = node->create_service<mecheye_ros_interface::srv::GetIntParameter>(
        "get_int_parameter", std::bind(&MechMindCamera::get_int_parameter_callback, this,
                                       std::placeholders::_1, std::placeholders::_2));

    set_float_parameter_service =
        node->create_service<mecheye_ros_interface::srv::SetFloatParameter>(
            "set_float_parameter", std::bind(&MechMindCamera::set_float_parameter_callback, this,
                                             std::placeholders::_1, std::placeholders::_2));

    get_float_parameter_service =
        node->create_service<mecheye_ros_interface::srv::GetFloatParameter>(
            "get_float_parameter", std::bind(&MechMindCamera::get_float_parameter_callback, this,
                                             std::placeholders::_1, std::placeholders::_2));

    set_bool_parameter_service = node->create_service<mecheye_ros_interface::srv::SetBoolParameter>(
        "set_bool_parameter", std::bind(&MechMindCamera::set_bool_parameter_callback, this,
                                        std::placeholders::_1, std::placeholders::_2));

    get_bool_parameter_service = node->create_service<mecheye_ros_interface::srv::GetBoolParameter>(
        "get_bool_parameter", std::bind(&MechMindCamera::get_bool_parameter_callback, this,
                                        std::placeholders::_1, std::placeholders::_2));

    set_enum_parameter_service = node->create_service<mecheye_ros_interface::srv::SetEnumParameter>(
        "set_enum_parameter", std::bind(&MechMindCamera::set_enum_parameter_callback, this,
                                        std::placeholders::_1, std::placeholders::_2));

    get_enum_parameter_service = node->create_service<mecheye_ros_interface::srv::GetEnumParameter>(
        "get_enum_parameter", std::bind(&MechMindCamera::get_enum_parameter_callback, this,
                                        std::placeholders::_1, std::placeholders::_2));

    set_range_parameter_service =
        node->create_service<mecheye_ros_interface::srv::SetRangeParameter>(
            "set_range_parameter", std::bind(&MechMindCamera::set_range_parameter_callback, this,
                                             std::placeholders::_1, std::placeholders::_2));

    get_range_parameter_service =
        node->create_service<mecheye_ros_interface::srv::GetRangeParameter>(
            "get_range_parameter", std::bind(&MechMindCamera::get_range_parameter_callback, this,
                                             std::placeholders::_1, std::placeholders::_2));

    set_roi_parameter_service = node->create_service<mecheye_ros_interface::srv::SetROIParameter>(
        "set_roi_parameter", std::bind(&MechMindCamera::set_roi_parameter_callback, this,
                                       std::placeholders::_1, std::placeholders::_2));

    get_roi_parameter_service = node->create_service<mecheye_ros_interface::srv::GetROIParameter>(
        "get_roi_parameter", std::bind(&MechMindCamera::get_roi_parameter_callback, this,
                                       std::placeholders::_1, std::placeholders::_2));

    set_float_array_parameter_service =
        node->create_service<mecheye_ros_interface::srv::SetFloatArrayParameter>(
            "set_float_array_parameter",
            std::bind(&MechMindCamera::set_float_array_parameter_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

    get_float_array_parameter_service =
        node->create_service<mecheye_ros_interface::srv::GetFloatArrayParameter>(
            "get_float_array_parameter",
            std::bind(&MechMindCamera::get_float_array_parameter_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
}

void MechMindCamera::publishColorCameraInfo(const std_msgs::msg::Header& header, int width,
                                            int height)
{
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header = header;
    camera_info.height = height;
    camera_info.width = width;
    camera_info.distortion_model = "plumb_bob";

    std::vector<double> distortionParams = {
        intrinsics.texture.cameraDistortion.k1, intrinsics.texture.cameraDistortion.k2,
        intrinsics.texture.cameraDistortion.p1, intrinsics.texture.cameraDistortion.p2,
        intrinsics.texture.cameraDistortion.k3};

    camera_info.d = distortionParams;

    std::vector<double> K{intrinsics.texture.cameraMatrix.fx,
                          0.0,
                          intrinsics.texture.cameraMatrix.cx,
                          0.0,
                          intrinsics.texture.cameraMatrix.fy,
                          intrinsics.texture.cameraMatrix.cy,
                          0.0,
                          0.0,
                          1.0};
    for (size_t i = 0; i < 9; ++i) {
        camera_info.k[i] = K[i];
    }

    std::vector<double> R{
        intrinsics.depthToTexture.rotation[0][0], intrinsics.depthToTexture.rotation[1][0],
        intrinsics.depthToTexture.rotation[2][0], intrinsics.depthToTexture.rotation[0][1],
        intrinsics.depthToTexture.rotation[1][1], intrinsics.depthToTexture.rotation[2][1],
        intrinsics.depthToTexture.rotation[0][2], intrinsics.depthToTexture.rotation[1][2],
        intrinsics.depthToTexture.rotation[2][2]};
    for (size_t i = 0; i < 9; ++i) {
        camera_info.r[i] = R[i];
    }

    std::vector<double> P = {intrinsics.texture.cameraMatrix.fx,
                             0.0,
                             intrinsics.texture.cameraMatrix.cx,
                             -intrinsics.depthToTexture.translation[0],
                             0.0,
                             intrinsics.texture.cameraMatrix.fy,
                             intrinsics.texture.cameraMatrix.cy,
                             -intrinsics.depthToTexture.translation[1],
                             0.0,
                             0.0,
                             1.0,
                             0.0};
    for (size_t i = 0; i < 12; ++i) {
        camera_info.p[i] = P[i];
    }
    pub_camera_info->publish(camera_info);
}

void MechMindCamera::publishDepthCameraInfo(const std_msgs::msg::Header& header, int width,
                                            int height)
{
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header = header;
    camera_info.height = height;
    camera_info.width = width;
    camera_info.distortion_model = "plumb_bob";

    std::vector<double> distortionParams = {
        intrinsics.depth.cameraDistortion.k1, intrinsics.depth.cameraDistortion.k2,
        intrinsics.depth.cameraDistortion.p1, intrinsics.depth.cameraDistortion.p2,
        intrinsics.depth.cameraDistortion.k3};

    camera_info.d = distortionParams;

    std::vector<double> K{intrinsics.depth.cameraMatrix.fx,
                          0.0,
                          intrinsics.depth.cameraMatrix.cx,
                          0.0,
                          intrinsics.depth.cameraMatrix.fy,
                          intrinsics.depth.cameraMatrix.cy,
                          0.0,
                          0.0,
                          1.0};
    for (size_t i = 0; i < 9; ++i) {
        camera_info.k[i] = K[i];
    }

    std::vector<double> R{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    for (size_t i = 0; i < 9; ++i) {
        camera_info.r[i] = R[i];
    }

    std::vector<double> P = {intrinsics.depth.cameraMatrix.fx,
                             0.0,
                             intrinsics.depth.cameraMatrix.cx,
                             0.0,
                             0.0,
                             intrinsics.depth.cameraMatrix.fy,
                             intrinsics.depth.cameraMatrix.cy,
                             0.0,
                             0.0,
                             0.0,
                             1.0,
                             0.0};
    for (size_t i = 0; i < 12; ++i) {
        camera_info.p[i] = P[i];
    }
    pub_camera_info->publish(camera_info);
}

void MechMindCamera::publishColorMap(mmind::eye::Color2DImage& color2DImage)
{
    cv::Mat color =
        cv::Mat(color2DImage.height(), color2DImage.width(), CV_8UC3, color2DImage.data());
    cv_bridge::CvImage cv_image;
    cv_image.image = color;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    sensor_msgs::msg::Image ros_image;
    cv_image.toImageMsg(ros_image);
    ros_image.header.frame_id = "mechmind_camera/color_map";
    ros_image.header.stamp = node->now();
    pub_color->publish(ros_image);
    publishColorCameraInfo(ros_image.header, color2DImage.width(), color2DImage.height());
    if (save_file) {
        const std::string path = "/tmp/image_2d.png";
        cv::imwrite(path, color);
        std::cout << "Capture and save the color image : " << path << std::endl;
    }
}

void MechMindCamera::publishStereoColorMap(mmind::eye::Color2DImage& leftColor2DImage,
                                           mmind::eye::Color2DImage& rightColor2DImage)
{
    cv::Mat colorLeft = cv::Mat(leftColor2DImage.height(), leftColor2DImage.width(), CV_8UC3,
                                leftColor2DImage.data());
    cv_bridge::CvImage cv_image_left;
    cv_image_left.image = colorLeft;
    cv_image_left.encoding = sensor_msgs::image_encodings::BGR8;
    sensor_msgs::msg::Image ros_image_left;
    cv_image_left.toImageMsg(ros_image_left);
    ros_image_left.header.frame_id = "mechmind_camera/left_color_map";
    ros_image_left.header.stamp = node->now();
    pub_color_left->publish(ros_image_left);
    publishDepthCameraInfo(ros_image_left.header, leftColor2DImage.width(),
                           leftColor2DImage.height());

    cv::Mat colorRight = cv::Mat(rightColor2DImage.height(), rightColor2DImage.width(), CV_8UC3,
                                 rightColor2DImage.data());
    cv_bridge::CvImage cv_image_right;
    cv_image_right.image = colorRight;
    cv_image_right.encoding = sensor_msgs::image_encodings::BGR8;
    sensor_msgs::msg::Image ros_image_right;
    cv_image_right.toImageMsg(ros_image_right);
    ros_image_right.header.frame_id = "mechmind_camera/right_color_map";
    ros_image_right.header.stamp = node->now();
    pub_color_right->publish(ros_image_right);
    publishColorCameraInfo(ros_image_right.header, rightColor2DImage.width(),
                           rightColor2DImage.height());

    if (save_file) {
        const std::string pathLeft = "/tmp/left_stereo.png";
        cv::imwrite(pathLeft, colorLeft);
        std::cout << "Capture and save the left stereo 2D image: " << pathLeft << std::endl;

        const std::string pathRight = "/tmp/right_stereo.png";
        cv::imwrite(pathRight, colorRight);
        std::cout << "Capture and save the right stereo 2D image: " << pathRight << std::endl;
    }
}

void MechMindCamera::publishDepthMap(mmind::eye::DepthMap& depthMap)
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
    publishDepthCameraInfo(ros_depth.header, depthMap.width(), depthMap.height());
    if (save_file) {
        bool success = cv::imwrite("/tmp/depth_map.tiff", depth);
        if (success) {
            std::cout << "The depth map is saved to /tmp." << std::endl;
        } else {
            std::cerr << "Failed to save depth map." << std::endl;
        }
    }
}

void MechMindCamera::publishPointCloud(mmind::eye::PointCloud& pointCloud)
{
    sensor_msgs::msg::PointCloud2 ros_cloud;
    ros_cloud.header.frame_id = "mechmind_camera/point_cloud";
    ros_cloud.header.stamp = node->now();
    convertToROSMsg(pointCloud, ros_cloud);
    pub_pcl->publish(ros_cloud);
    publishDepthCameraInfo(ros_cloud.header, pointCloud.width(), pointCloud.height());
}

void MechMindCamera::publishColorPointCloud(mmind::eye::TexturedPointCloud& texturedPointCloud)
{
    sensor_msgs::msg::PointCloud2 ros_color_cloud;
    ros_color_cloud.header.frame_id = "mechmind_camera/textured_point_cloud";
    ros_color_cloud.header.stamp = node->now();
    convertToROSMsg(texturedPointCloud, ros_color_cloud);
    pub_pcl_color->publish(ros_color_cloud);
    publishDepthCameraInfo(ros_color_cloud.header, texturedPointCloud.width(),
                           texturedPointCloud.height());
}

void MechMindCamera::capture_color_image_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::CaptureColorImage::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::CaptureColorImage::Response> res)
{
    mmind::eye::Frame2D frame;
    auto status = camera.capture2D(frame);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
    auto colorMap = frame.getColorImage();
    publishColorMap(colorMap);
}

void MechMindCamera::capture_stereo_color_images_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::CaptureStereoColorImages::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::CaptureStereoColorImages::Response> res)
{
    mmind::eye::Frame2D frameLeft;
    mmind::eye::Frame2D frameRight;
    auto status = camera.captureStereo2D(frameLeft, frameRight);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
    auto colorMapLeft = frameLeft.getColorImage();
    auto colorMapRight = frameRight.getColorImage();
    publishStereoColorMap(colorMapLeft, colorMapRight);
}

void MechMindCamera::capture_depth_map_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::CaptureDepthMap::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::CaptureDepthMap::Response> res)
{
    mmind::eye::Frame3D frame;
    auto status = camera.capture3D(frame);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
    auto depthMap = frame.getDepthMap();
    publishDepthMap(depthMap);
}

void MechMindCamera::capture_point_cloud_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::CapturePointCloud::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::CapturePointCloud::Response> res)
{
    mmind::eye::Frame3D frame;
    auto status = camera.capture3D(frame);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
    auto pointCloud = frame.getUntexturedPointCloud();
    publishPointCloud(pointCloud);
    if (save_file) {
        frame.saveUntexturedPointCloud(mmind::eye::FileFormat::PLY, "/tmp/point_cloud.ply");
        std::cout << "The point cloud is saved to /tmp." << std::endl;
    }
}

void MechMindCamera::capture_textured_point_cloud_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::CaptureTexturedPointCloud::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::CaptureTexturedPointCloud::Response> res)
{
    mmind::eye::Frame2DAnd3D frame2DAnd3D;
    mmind::eye::ErrorStatus status = camera.capture2DAnd3D(frame2DAnd3D);

    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
    mmind::eye::TexturedPointCloud texturedPointCloud = frame2DAnd3D.getTexturedPointCloud();
    publishColorPointCloud(texturedPointCloud);
    if (save_file) {
        frame2DAnd3D.saveTexturedPointCloud(mmind::eye::FileFormat::PLY,
                                            "/tmp/textured_point_cloud.ply");
        std::cout << "The textured point cloud is saved to /tmp." << std::endl;
    }
}

void MechMindCamera::device_info_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::DeviceInfo::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::DeviceInfo::Response> res)
{
    mmind::eye::CameraInfo cameraInfo;
    mmind::eye::ErrorStatus status = camera.getCameraInfo(cameraInfo);
    showError(status);
    res->model = cameraInfo.model.c_str();
    res->serial_number = cameraInfo.serialNumber.c_str();
    res->hardware_version = cameraInfo.hardwareVersion.toString();
    res->firmware_version = cameraInfo.firmwareVersion.toString();
    res->ip_address = cameraInfo.ipAddress.c_str();
    res->subnet_mask = cameraInfo.subnetMask.c_str();
    res->ip_assignment_method = ipAssignmentMethodToString(cameraInfo.ipAssignmentMethod);
    res->port = cameraInfo.port;
}

void MechMindCamera::get_all_user_sets_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::GetAllUserSets::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::GetAllUserSets::Response> res)
{
    std::vector<std::string> sequence;
    mmind::eye::UserSetManager userSetManager = camera.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.getAllUserSetNames(sequence);
    showError(status);
    res->sequence = sequence;
}

void MechMindCamera::get_current_user_set_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::GetCurrentUserSet::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::GetCurrentUserSet::Response> res)
{
    std::string value;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getName(value);
    showError(status);
    res->value = value.c_str();
}

void MechMindCamera::set_current_user_set_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::SetCurrentUserSet::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::SetCurrentUserSet::Response> res)
{
    mmind::eye::UserSetManager userSetManager = camera.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.selectUserSet(req->value.c_str());
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::add_user_set_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::AddUserSet::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::AddUserSet::Response> res)
{
    mmind::eye::UserSetManager userSetManager = camera.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.addUserSet({req->value.c_str()});
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::delete_user_set_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::DeleteUserSet::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::DeleteUserSet::Response> res)
{
    mmind::eye::UserSetManager userSetManager = camera.userSetManager();
    mmind::eye::ErrorStatus status = userSetManager.deleteUserSet({req->value.c_str()});
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::save_all_settings_to_user_sets_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::SaveAllSettingsToUserSets::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::SaveAllSettingsToUserSets::Response> res)
{
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.saveAllParametersToDevice();
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::get_float_array_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::GetFloatArrayParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::GetFloatArrayParameter::Response> res)
{
    std::vector<double> array;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getFloatArrayValue(req->name, array);
    showError(status);
    res->array = array;
}

void MechMindCamera::set_float_array_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::SetFloatArrayParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::SetFloatArrayParameter::Response> res)
{
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setFloatArrayValue(req->name, req->array);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::get_roi_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::GetROIParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::GetROIParameter::Response> res)
{
    mmind::eye::ROI roi;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getRoiValue(req->name, roi);
    showError(status);
    res->x = roi.upperLeftX;
    res->y = roi.upperLeftY;
    res->width = roi.width;
    res->height = roi.height;
}

void MechMindCamera::set_roi_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::SetROIParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::SetROIParameter::Response> res)
{
    mmind::eye::ROI roi{req->x, req->y, req->width, req->height};
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setRoiValue(req->name, roi);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::get_range_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::GetRangeParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::GetRangeParameter::Response> res)
{
    mmind::eye::Range range{0, 0};
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getRangeValue(req->name, range);
    showError(status);
    res->lower = range.min;
    res->upper = range.max;
}

void MechMindCamera::set_range_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::SetRangeParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::SetRangeParameter::Response> res)
{
    mmind::eye::Range range{req->lower, req->upper};
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setRangeValue(req->name, range);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::get_int_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::GetIntParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::GetIntParameter::Response> res)
{
    int value;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getIntValue(req->name, value);
    showError(status);
    res->value = value;
}

void MechMindCamera::set_int_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::SetIntParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::SetIntParameter::Response> res)
{
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setIntValue(req->name, req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::get_bool_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::GetBoolParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::GetBoolParameter::Response> res)
{
    bool value;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getBoolValue(req->name, value);
    showError(status);
    res->value = value;
}

void MechMindCamera::set_bool_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::SetBoolParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::SetBoolParameter::Response> res)
{
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setBoolValue(req->name, req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::get_enum_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::GetEnumParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::GetEnumParameter::Response> res)
{
    std::string value;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getEnumValue(req->name, value);
    showError(status);
    res->value = value;
}

void MechMindCamera::set_enum_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::SetEnumParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::SetEnumParameter::Response> res)
{
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setEnumValue(req->name, req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}

void MechMindCamera::get_float_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::GetFloatParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::GetFloatParameter::Response> res)
{
    double value = -1;
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.getFloatValue(req->name, value);
    showError(status);
    res->value = value;
}

void MechMindCamera::set_float_parameter_callback(
    const std::shared_ptr<mecheye_ros_interface::srv::SetFloatParameter::Request> req,
    std::shared_ptr<mecheye_ros_interface::srv::SetFloatParameter::Response> res)
{
    mmind::eye::UserSet userSet = camera.currentUserSet();
    mmind::eye::ErrorStatus status = userSet.setFloatValue(req->name, req->value);
    showError(status);
    res->error_code = status.errorCode;
    res->error_description = status.errorDescription.c_str();
}
