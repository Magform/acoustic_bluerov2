#include "controller_reader.h"
#include "publisher_node.h"
#include "controller_axes.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>

#define controller_dead_zone 0.2f

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    ControllerAxes axes;

    ControllerReader controller_reader(axes, controller_dead_zone);
    controller_reader.start();

    auto publisher_node = std::make_shared<PublisherNode>(axes, "config/config.yaml");
    rclcpp::spin(publisher_node);

    controller_reader.stop();
    rclcpp::shutdown();
    return 0;
}
