#include "controller_reader.h"
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <fstream>
#define LOG_ENABLE

ControllerReader::ControllerReader(ControllerAxes& axes, const std::string& config_path)
    : _axes(axes), _stop_thread(false), _reader_thread(), _joystick(nullptr)
    {

    YAML::Node config = YAML::LoadFile(config_path);
    _dead_zone = config["dead_zone"].as<float>();

    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        std::cerr << "Failed to init SDL: " << SDL_GetError() << std::endl;
        throw std::runtime_error("SDL init failed");
    }

    if (SDL_NumJoysticks() < 1) {
        std::cerr << "No joystick connected!" << std::endl;
        throw std::runtime_error("No joystick found");
    }

    _joystick = SDL_JoystickOpen(0);
    if (_joystick == nullptr) {
        std::cerr << "Failed to open joystick: " << SDL_GetError() << std::endl;
        throw std::runtime_error("Failed to open joystick");
    }
}


ControllerReader::~ControllerReader(){
    stop(); // ensure thread is stopped before destruction

    if (_joystick) {
        SDL_JoystickClose(_joystick);
    }
    SDL_Quit();
}

void ControllerReader::start(){
    _stop_thread = false;
    _reader_thread = std::thread(&ControllerReader::read_loop, this);
}

void ControllerReader::stop(){
    _stop_thread = true;
    if (_reader_thread.joinable()) {
        _reader_thread.join();
    }
}

float ControllerReader::apply_dead_zone(float value){
    if (std::abs(value) < _dead_zone) {
        return 0.0f;
    } else {
        return (value > 0 ? 1 : -1) * ((std::abs(value) - _dead_zone) / (1.0f - _dead_zone));
    }
}

void ControllerReader::read_loop(){
    #ifdef LOG_ENABLE
    std::ofstream logFile("controller_log.csv");
    logFile << "epoch,leftH,leftV,leftT,rightH,rightV,rightT\n";
    #endif

    while (!_stop_thread) {
        SDL_JoystickUpdate();
        float leftH = apply_dead_zone(SDL_JoystickGetAxis(_joystick, 0) / 32767.0f);
        float leftV = apply_dead_zone(SDL_JoystickGetAxis(_joystick, 1) / 32767.0f);
        float leftT = SDL_JoystickGetAxis(_joystick, 2) / 32767.0f;
        float rightH = apply_dead_zone(SDL_JoystickGetAxis(_joystick, 3) / 32767.0f);
        float rightV = apply_dead_zone(SDL_JoystickGetAxis(_joystick, 4) / 32767.0f);
        float rightT = SDL_JoystickGetAxis(_joystick, 5) / 32767.0f;

        _axes.leftHorizontal.store(leftH);
        _axes.leftVertical.store(leftV);
        _axes.leftTrigger.store(leftT);
        _axes.rightHorizontal.store(rightH);
        _axes.rightVertical.store(rightV);
        _axes.rightTrigger.store(rightT);

        #ifdef LOG_ENABLE
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()
                ).count();

        logFile << ms << ","
                << leftH << ","
                << leftV << ","
                << leftT << ","
                << rightH << ","
                << rightV << ","
                << rightT << "\n";
        #endif

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    #ifdef LOG_ENABLE
    logFile.close();
    #endif
}