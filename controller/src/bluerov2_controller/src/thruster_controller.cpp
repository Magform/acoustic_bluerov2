#include "thruster_controller.hpp"
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

using namespace std::chrono_literals;

ThrusterController::ThrusterController() : Node("thruster_controller") {
    this->declare_parameter<std::string>("keymap_file", "keymap.conf");
    loadKeymap(this->get_parameter("keymap_file").as_string());

    for(int i = 1; i <= 6; ++i) {
        publishers_[i] = this->create_publisher<std_msgs::msg::Float32>(
            "/bluerov2/cmd_thruster" + std::to_string(i), 10);
        thruster_values_[i] = 0.0f;
    }

    if(!configureTerminal()) {
        RCLCPP_FATAL(get_logger(), "Failed to configure terminal input");
        rclcpp::shutdown();
    }
}

ThrusterController::~ThrusterController() {
    restoreTerminal();
}

void ThrusterController::processInput() {
    char input;
    while(readChar(input)) {
        handleKey(input);
    }
}

void ThrusterController::handleKey(char key) {
    if(keymap_.find(key) == keymap_.end()) return;

    const auto& cmd = keymap_[key];
    
    if(key == 'k') {
        resetAllThrusters();
        RCLCPP_DEBUG(get_logger(), "Reset all thrusters to zero");
        return;
    }

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

bool ThrusterController::validateThrusterId(int id) {
    if(publishers_.find(id) == publishers_.end()) {
        RCLCPP_ERROR(get_logger(), "Invalid thruster ID: %d", id);
        return false;
    }
    return true;
}

void ThrusterController::publishThrusterValue(int id, float value) {
    auto msg = std_msgs::msg::Float32();
    msg.data = value;
    publishers_[id]->publish(msg);
}

void ThrusterController::resetAllThrusters() {
    for(auto& [id, value] : thruster_values_) {
        value = 0.0f;
        publishThrusterValue(id, value);
    }
}

bool ThrusterController::configureTerminal() {
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

void ThrusterController::restoreTerminal() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &original_termios_);
}

bool ThrusterController::readChar(char& c) {
    struct pollfd pfd = {STDIN_FILENO, POLLIN, 0};
    if(poll(&pfd, 1, 0) > 0) {
        if(read(STDIN_FILENO, &c, 1) == 1) {
            return true;
        }
    }
    return false;
}

std::vector<std::string> ThrusterController::split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        if (!token.empty()) {
            tokens.push_back(token);
        }
    }
    return tokens;
}

std::string ThrusterController::trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t");
    size_t end = s.find_last_not_of(" \t");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}

void ThrusterController::loadKeymap(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        RCLCPP_ERROR(get_logger(), "Failed to open keymap file: %s", filename.c_str());
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;

        auto tokens = split(line, ' ');
        if (tokens.size() != 1 && tokens.size() != 3) {
            RCLCPP_WARN(get_logger(), "Invalid line: %s", line.c_str());
            continue;
        }

        char key = tokens[0][0];
        ThrusterCommand cmd;

        if (tokens.size() == 3) {
            for (const auto& t : split(tokens[1], ',')) {
                try {
                    cmd.thrusters.push_back(std::stoi(t));
                } catch (...) {
                    cmd.thrusters.clear();
                    break;
                }
            }

            for (const auto& d : split(tokens[2], ',')) {
                try {
                    cmd.directions.push_back(std::stof(d));
                } catch (...) {
                    cmd.directions.clear();
                    break;
                }
            }

            if (cmd.thrusters.size() != cmd.directions.size()) {
                RCLCPP_WARN(get_logger(), "Mismatched command in line: %s", line.c_str());
                continue;
            }
        }

        keymap_[key] = cmd;
    }
}