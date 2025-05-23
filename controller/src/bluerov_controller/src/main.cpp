#include "controller_reader.h"
#include "publisher_node.h"
#include "controller_axes.h"
#include "gui_viewer.h"
#include "status_node.h"
#include "status.h"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <string>
#include <filesystem>

#define controller_dead_zone 0.2f

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    ControllerAxes axes;
    Status status;

    ControllerReader controller_reader(axes, controller_dead_zone);
    controller_reader.start();

    GuiViewer gui_viewer(axes, &status);
    gui_viewer.start();

    auto publisher_node = std::make_shared<PublisherNode>(axes, "config.yaml");
    auto status_node = std::make_shared<StatusNode>(status, "config.yaml");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(publisher_node);
    executor.add_node(status_node);
    executor.spin();

    controller_reader.stop();
    gui_viewer.stop();
    rclcpp::shutdown();
    return 0;
}
