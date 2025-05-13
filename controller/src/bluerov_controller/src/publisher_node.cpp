#include "publisher_node.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <chrono>

PublisherNode::PublisherNode(std::atomic<float>& axisLeftVertical, std::atomic<float>& axisLeftHorizontal,
                             std::atomic<float>& axisRightVertical, std::atomic<float>& axisRightHorizontal,
                             const std::string& config_path)
    : Node("controller_publisher"),
      _axisLeftVertical(axisLeftVertical),
      _axisLeftHorizontal(axisLeftHorizontal),
      _axisRightVertical(axisRightVertical),
      _axisRightHorizontal(axisRightHorizontal) {

    YAML::Node config = YAML::LoadFile(config_path);
    _max_speed = config["max_speed"].as<float>();
    _sending_time = config["sending_time"].as<float>();

    for (const auto& item : config["keymap"]) {
        std::string axis = item.first.as<std::string>();
        std::vector<float> values = item.second.as<std::vector<float>>();
        _keymap[axis] = values;
    }

    _thruster_count = config["thruster"].as<int>();
    for (int i = 1; i <= _thruster_count; ++i) {
        auto pub = this->create_publisher<std_msgs::msg::Float64>(
            "/bluerov2/cmd_thruster" + std::to_string(i), 10);
        _thruster_publishers.push_back(pub);
    }

    // Store timer to keep it alive
    _timer = this->create_wall_timer(std::chrono::duration<double>(_sending_time),
                                     std::bind(&PublisherNode::timer_callback, this));
}

void PublisherNode::timer_callback() {
    std::vector<float> axis_values = {
        _axisLeftVertical.load(),
        _axisLeftHorizontal.load(),
        _axisRightVertical.load(),
        _axisRightHorizontal.load()
    };

    std::vector<float> thruster_values(_thruster_count, 0.0f);

    for (int axis_idx = 0; axis_idx < 4; ++axis_idx) {
        std::string axis_key = "ax" + std::to_string(axis_idx + 1);
        if (_keymap.find(axis_key) != _keymap.end()) {
            const auto& multipliers = _keymap[axis_key];
            for (size_t i = 0; i < multipliers.size(); ++i) {
                thruster_values[i] += axis_values[axis_idx] * multipliers[i];
            }
        }
    }

    for (size_t i = 0; i < _thruster_publishers.size(); ++i) {
        std_msgs::msg:Float64 msg;
        msg.data = thruster_values[i] * _max_speed;
        _thruster_publishers[i]->publish(msg);
    }
}
