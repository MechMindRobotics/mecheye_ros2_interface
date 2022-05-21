#include <MechMindCamera.h>
#include "MechMindCamera.cpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    MechMindCamera mm_camera;
    executor.add_node(mm_camera.node);
    executor.spin();
    return 0;
}