#include "status_node.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>

StatusNode::StatusNode(Status& status, const std::string& config_path)
    : Node("status_node"), _status(status) {

    if (!std::filesystem::exists(config_path)) {
        RCLCPP_ERROR(get_logger(), "Config file not found: %s", config_path.c_str());
        return;
    }

    YAML::Node config = YAML::LoadFile(config_path);
    std::string pose_topic = config["pose"] ? config["pose"].as<std::string>() : "NONE";

    if (pose_topic == "NONE") {
        RCLCPP_INFO(get_logger(), "Pose topic set to NONE. StatusNode will be disabled.");
        return;
    }

    _subscription = this->create_subscription<geometry_msgs::msg::Pose>(
        pose_topic, 10,
        std::bind(&StatusNode::pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribed to pose topic: %s", pose_topic.c_str());
}

void StatusNode::pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    _status.x.store(static_cast<float>(msg->position.x));
    _status.y.store(static_cast<float>(msg->position.y));
    _status.z.store(static_cast<float>(msg->position.z));
}
