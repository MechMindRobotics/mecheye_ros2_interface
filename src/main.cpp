#include <csignal>
#include <MechMindCamera.h>

void signalHandler(int signum) { rclcpp::shutdown(); }

int main(int argc, char** argv)
{
    rclcpp::InitOptions init_options;
    init_options.auto_initialize_logging(false);
    rclcpp::init(argc, argv, init_options);
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
