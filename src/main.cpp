#include <csignal>
#include <MechMindCamera.h>
#include "MechMindCamera.cpp"
#include <area_scan_3d_camera/api_util.h>

void signalHandler(int signum) { rclcpp::shutdown(); }

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    rclcpp::executors::MultiThreadedExecutor executor;
    try {
        MechMindCamera mm_camera;
        executor.add_node(mm_camera.node);
        executor.spin();
    } catch (mmind::eye::ErrorStatus error) {
        showError(error);
        return error.errorCode;
    }
    return 0;
}