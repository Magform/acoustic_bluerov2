#include "controller_reader.h"
#include "publisher_node.h"
#include "controller_axes.h"
#include "gui_viewer.h"

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>
#include <string>
#include <filesystem>

#define controller_dead_zone 0.2f

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    ControllerAxes axes;

    ControllerReader controller_reader(axes, controller_dead_zone);
    controller_reader.start();

    GuiViewer gui_viewer(axes);
    gui_viewer.start();

    // Pass the config file path to the PublisherNode
    auto publisher_node = std::make_shared<PublisherNode>(axes, "config.yaml");
    rclcpp::spin(publisher_node);

    controller_reader.stop();
    gui_viewer.stop();
    rclcpp::shutdown();
    return 0;
}
