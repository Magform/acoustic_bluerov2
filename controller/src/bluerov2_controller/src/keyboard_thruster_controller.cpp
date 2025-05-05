#include "thruster_controller.hpp"
#include <memory>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<ThrusterController>();
    rclcpp::Rate rate(50);  // 50Hz

    while(rclcpp::ok()) {
        controller->processInput();
        rclcpp::spin_some(controller);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}