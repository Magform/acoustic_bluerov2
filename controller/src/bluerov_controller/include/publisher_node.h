#ifndef PUBLISHER_NODE_H
#define PUBLISHER_NODE_H

    /**
    Publisher node creator, load the variable and the config file containing:
    - MAX_SPEED -> multiply the controller readed value to this speed.
    - SENDING_TIME -> delay between different sended value
    - KEYMAP -> a map like:
        #axes motor_multiplyer
        ax1 1,1,1,1,0,0
        ax2 1,-1,-1,1,0,0
        ax3 0,0,0,0,0,1
        ax4 0,0,0,0,0,0
        That contain another number to multiplay the axis befor sending it to the required motor 
        ros2 topic pub /bluerov2/cmd_thruster1 std_msgs/msg/Float32 "{data: 10}" -> is for example for publishing 10 to motor 1
    */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <atomic>
#include <map>
#include <vector>
#include <string>

class PublisherNode : public rclcpp::Node {
public:
    PublisherNode(std::atomic<float>& axisLeftVertical, std::atomic<float>& axisLeftHorizontal,
                  std::atomic<float>& axisRightVertical, std::atomic<float>& axisRightHorizontal,
                  const std::string& config_path);

private:
    void timer_callback();

    std::atomic<float>& _axisLeftVertical;
    std::atomic<float>& _axisLeftHorizontal;
    std::atomic<float>& _axisRightVertical;
    std::atomic<float>& _axisRightHorizontal;

    float _max_speed;
    float _sending_time;
    int _thruster_count;
    std::map<std::string, std::vector<float>> _keymap;

    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> _thruster_publishers;
    rclcpp::TimerBase::SharedPtr _timer;
};

#endif // PUBLISHER_NODE_H
