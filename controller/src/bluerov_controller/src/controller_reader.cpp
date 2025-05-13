#include "controller_reader.h"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>

ControllerReader::ControllerReader(ControllerAxes& axes, float dead_zone)
    : _axes(axes), _dead_zone(dead_zone), _stop_thread(false), _reader_thread(), _joystick(nullptr)
    {
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
    while (!_stop_thread) {
        SDL_JoystickUpdate();
        
        // Normalize axis values [-32768, 32767] -> [-1.0f, 1.0f]
        _axes.leftHorizontal.store(apply_dead_zone(SDL_JoystickGetAxis(_joystick, 0) / 32767.0f));
        _axes.leftVertical.store(apply_dead_zone(SDL_JoystickGetAxis(_joystick, 1) / 32767.0f));
        _axes.leftTrigger.store(SDL_JoystickGetAxis(_joystick, 2) / 32767.0f);
        _axes.rightHorizontal.store(apply_dead_zone(SDL_JoystickGetAxis(_joystick, 3) / 32767.0f));
        _axes.rightVertical.store(apply_dead_zone(SDL_JoystickGetAxis(_joystick, 4) / 32767.0f));
        _axes.rightTrigger.store(SDL_JoystickGetAxis(_joystick, 5) / 32767.0f);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}