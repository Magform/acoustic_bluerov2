#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <unordered_map>
#include <vector>
#include <memory>
#include <algorithm>

using namespace std::chrono_literals;

struct ThrusterCommand {
    std::vector<int> thrusters;
    std::vector<float> directions;
};

// Configuration constants
constexpr float STEP = 1.0f;
constexpr float MAX_THRUST = 10.0f;

// Keymap configuration
std::unordered_map<char, ThrusterCommand> keymap = {
    {'w', {{1, 2, 3, 4}, {1, 1, 1, 1}}},     // Forward
    {'s', {{4, 3, 2, 1}, {-1, -1, -1, -1}}}, // Backward
    {'a', {{1, 2, 3, 4}, {1, -1, -1, 1}}},   // Left
    {'d', {{4, 3, 2, 1}, {-1, 1, 1, -1}}},   // Right
    {'q', {{1, 2, 3, 4}, {1, 1, -1, -1}}},   // Rotate anticlockwise
    {'e', {{4, 3, 1, 1}, {-1, -1, 1, 1}}},   // Rotate clockwise
    {'k', {{}, {}}},                         // Reset all thrusters
};

class ThrusterController : public rclcpp::Node {
public:
    ThrusterController() : Node("thruster_controller") {
        // Create publishers for thrusters 1-6
        for(int i = 1; i <= 6; ++i) {
            publishers_[i] = this->create_publisher<std_msgs::msg::Float32>(
                "/bluerov2/cmd_thruster" + std::to_string(i), 10);
            thruster_values_[i] = 0.0f;
        }
        
        // Configure terminal input
        if(!configureTerminal()) {
            RCLCPP_FATAL(get_logger(), "Failed to configure terminal input");
            rclcpp::shutdown();
        }
    }

    ~ThrusterController() {
        restoreTerminal();
    }

    void processInput() {
        char input;
        while(readChar(input)) {
            handleKey(input);
        }
    }

private:
    std::unordered_map<int, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers_;
    std::unordered_map<int, float> thruster_values_;
    struct termios original_termios_;

    void handleKey(char key) {
        if(keymap.find(key) == keymap.end()) return;

        const auto& cmd = keymap[key];
        
        // Handle special reset case
        if(key == 'k') {
            resetAllThrusters();
            RCLCPP_DEBUG(get_logger(), "Reset all thrusters to zero");
            return;
        }

        // Apply command to thrusters
        bool command_executed = false;
        for(size_t i = 0; i < cmd.thrusters.size(); ++i) {
            const int thruster_id = cmd.thrusters[i];
            const float direction = cmd.directions[i];

            if(!validateThrusterId(thruster_id)) continue;

            thruster_values_[thruster_id] += STEP * direction;
            thruster_values_[thruster_id] = std::clamp(
                thruster_values_[thruster_id],
                -MAX_THRUST,
                MAX_THRUST
            );

            publishThrusterValue(thruster_id, thruster_values_[thruster_id]);
            command_executed = true;
        }

        if(command_executed) {
            RCLCPP_DEBUG(get_logger(), "Executed command '%c'", key);
        }
    }

    bool validateThrusterId(int id) {
        if(publishers_.find(id) == publishers_.end()) {
            RCLCPP_ERROR(get_logger(), "Invalid thruster ID: %d", id);
            return false;
        }
        return true;
    }

    void publishThrusterValue(int id, float value) {
        auto msg = std_msgs::msg::Float32();
        msg.data = value;
        publishers_[id]->publish(msg);
    }

    void resetAllThrusters() {
        for(auto& [id, value] : thruster_values_) {
            value = 0.0f;
            publishThrusterValue(id, value);
        }
    }

    // Terminal handling functions
    bool configureTerminal() {
        if(!isatty(STDIN_FILENO)) return false;
        
        if(tcgetattr(STDIN_FILENO, &original_termios_) == -1) {
            return false;
        }

        struct termios new_termios = original_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        new_termios.c_cc[VMIN] = 0;
        new_termios.c_cc[VTIME] = 0;

        if(tcsetattr(STDIN_FILENO, TCSAFLUSH, &new_termios) == -1) {
            return false;
        }

        return true;
    }

    void restoreTerminal() {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &original_termios_);
    }

    bool readChar(char& c) {
        struct pollfd pfd = {STDIN_FILENO, POLLIN, 0};
        if(poll(&pfd, 1, 0) > 0) {
            if(read(STDIN_FILENO, &c, 1) == 1) {
                return true;
            }
        }
        return false;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto controller = std::make_shared<ThrusterController>();
    rclcpp::Rate rate(50);  // 50Hz

    while(rclcpp::ok()) {
        controller->processInput();
        rclcpp::spin_some(controller);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}