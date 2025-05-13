#include "controller_reader.h"
#include "publisher_node.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::atomic<float> axisLeftVertical(0.0), axisLeftHorizontal(0.0), axisRightVertical(0.0), axisRightHorizontal(0.0);

    ControllerReader controller_reader(axisLeftVertical, axisLeftHorizontal, axisRightVertical, axisRightHorizontal);
    controller_reader.start();

    auto publisher_node = std::make_shared<PublisherNode>(
        axisLeftVertical, axisLeftHorizontal, axisRightVertical, axisRightHorizontal,
        "config/config.yaml");
    rclcpp::spin(publisher_node);
    
    controller_reader.stop();
    rclcpp::shutdown();
    return 0;
}
