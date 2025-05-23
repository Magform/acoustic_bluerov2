#ifndef STATUS_NODE_H
#define STATUS_NODE_H

#include "status.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <memory>

class StatusNode : public rclcpp::Node {
public:
    StatusNode(Status& status, const std::string& config_path);

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

    Status& _status;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _subscription;
};

#endif // STATUS_NODE_H
