#include "publisher_node.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <cmath>
#include <algorithm>

PublisherNode::PublisherNode(ControllerAxes& axes, const std::string& config_path)
    : Node("controller_publisher"), _axes(axes)
{
    YAML::Node config = YAML::LoadFile(config_path);
    _max_force   = config["max_force"].as<float>();
    _sending_time = config["sending_time"].as<float>();
    _threshold   = config["threshold"].as<float>();
    _resend_time = config["resend_time"] ? config["resend_time"].as<float>() : 0.0f;

    for (const auto& item : config["keymap"]) {
        std::string axis = item.first.as<std::string>();
        std::vector<float> values = item.second.as<std::vector<float>>();
        _keymap[axis] = values;
    }

    _thruster_count = config["thruster"].as<int>();

    // Create ONE publisher for all thrusters
    _thruster_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "/bluerov2/cmd_thrusters", 10);

    _previous_thruster_values.resize(_thruster_count, 0.0f);
    _last_sent_time = std::chrono::steady_clock::now();

    // Store timer to keep it alive
    _timer = this->create_wall_timer(std::chrono::duration<double>(_sending_time),
                                     std::bind(&PublisherNode::timer_callback, this));
}

void PublisherNode::timer_callback() {
    std::vector<float> axis_values = {
        _axes.leftVertical.load(),
        _axes.leftHorizontal.load(),
        _axes.rightVertical.load(),
        _axes.rightHorizontal.load(),
        _axes.leftTrigger.load(),
        _axes.rightTrigger.load()
    };

    std::vector<float> thruster_values(_thruster_count, 0.0f);

    // Calculate thruster values based on axis values and keymap
    for (int axis_idx = 0; axis_idx < 6; ++axis_idx) {
        std::string axis_key = "ax" + std::to_string(axis_idx);
        if (_keymap.find(axis_key) != _keymap.end()) {
            const auto& multipliers = _keymap[axis_key];
            for (size_t i = 0; i < multipliers.size(); ++i) {
                thruster_values[i] += axis_values[axis_idx] * multipliers[i];
            }
        }
    }

    // Check if any thruster value exceeds the threshold
    bool publish_needed = false;
    for (size_t i = 0; i < thruster_values.size(); ++i) {
        if (std::abs(thruster_values[i] - _previous_thruster_values[i]) > _threshold) {
            publish_needed = true;
            break;
        }
    }

    // Resend the last command periodically, so a lost packet doesn't leave the ROV stuck
    auto now = std::chrono::steady_clock::now();
    if (_resend_time > 0.0f &&
        std::chrono::duration<float>(now - _last_sent_time).count() >= _resend_time) {
        publish_needed = true;
    }

    if (publish_needed) {
        std_msgs::msg::Int32MultiArray msg;
        msg.data.resize(thruster_values.size());
        for (size_t i = 0; i < thruster_values.size(); ++i) {
            msg.data[i] = std::lround(std::clamp(thruster_values[i], -1.0f, 1.0f) * _max_force);
        }

        _thruster_pub->publish(msg);
        _previous_thruster_values = thruster_values;
        _last_sent_time = now;
    }
}
