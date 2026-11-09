#ifndef PUBLISHER_NODE_H
#define PUBLISHER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <atomic>
#include <chrono>
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
    std::map<std::string, std::vector<float>> _keymap;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr _thruster_pub;

    float _max_force;
    float _sending_time;
    float _threshold;
    float _resend_time;

    int _thruster_count;
    std::vector<float> _previous_thruster_values;
    std::chrono::steady_clock::time_point _last_sent_time;

    rclcpp::TimerBase::SharedPtr _timer;
};

#endif // PUBLISHER_NODE_H
