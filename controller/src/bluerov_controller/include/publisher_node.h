#ifndef PUBLISHER_NODE_H
#define PUBLISHER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <atomic>
#include <map>
#include <vector>
#include <string>

#include "controller_axes.h"

class PublisherNode : public rclcpp::Node {
public:
    PublisherNode(ControllerAxes& axes, const std::string& config_path);
private:
    void timer_callback();

    ControllerAxes& _axes;

    float _max_speed;
    float _sending_time;
    int _thruster_count;
    std::map<std::string, std::vector<float>> _keymap;

    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> _thruster_publishers;
    rclcpp::TimerBase::SharedPtr _timer;
};

#endif // PUBLISHER_NODE_H
