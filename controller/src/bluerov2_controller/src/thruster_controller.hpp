#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <termios.h>
#include <unordered_map>
#include <vector>
#include <memory>
#include <string>

struct ThrusterCommand {
    std::vector<int> thrusters;
    std::vector<float> directions;
};

class ThrusterController : public rclcpp::Node {
public:
    // Configuration constants
    static constexpr float STEP = 1.0f;
    static constexpr float MAX_THRUST = 100.0f;

    ThrusterController();
    ~ThrusterController();

    void processInput();

private:
    std::unordered_map<char, ThrusterCommand> keymap_;
    std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers_;
    std::unordered_map<int, float> thruster_values_;
    struct termios original_termios_;

    void handleKey(char key);
    bool validateThrusterId(int id);
    void publishThrusterValue(int id, float value);
    void resetAllThrusters();
    bool configureTerminal();
    void restoreTerminal();
    bool readChar(char& c);
    void loadKeymap(const std::string& filename);
    std::vector<std::string> split(const std::string& s, char delimiter);
    std::string trim(const std::string& s);
};