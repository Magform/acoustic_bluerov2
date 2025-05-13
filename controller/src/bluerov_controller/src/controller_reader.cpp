#include "controller_reader.h"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>

ControllerReader::ControllerReader(std::atomic<float>& axisLeftVertical, std::atomic<float>& axisLeftHorizontal,
                                   std::atomic<float>& axisRightVertical, std::atomic<float>& axisRightHorizontal)
    : _axisLeftVertical(axisLeftVertical), _axisLeftHorizontal(axisLeftHorizontal),
      _axisRightVertical(axisRightVertical), _axisRightHorizontal(axisRightHorizontal),
      _stop_thread(false), _reader_thread(), _joystick(nullptr) {

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

ControllerReader::~ControllerReader() {
    stop(); // ensure thread is stopped before destruction

    if (_joystick) {
        SDL_JoystickClose(_joystick);
    }
    SDL_Quit();
}

void ControllerReader::start() {
    _stop_thread = false;
    _reader_thread = std::thread(&ControllerReader::read_loop, this);
}

void ControllerReader::stop() {
    _stop_thread = true;
    if (_reader_thread.joinable()) {
        _reader_thread.join();
    }
}

void ControllerReader::read_loop() {
    while (!_stop_thread) {
        SDL_JoystickUpdate();

        // Normalize value from -32768 to 32767
        _axisLeftHorizontal.store(SDL_JoystickGetAxis(_joystick, 0) / 32767.0f);
        _axisLeftVertical.store(SDL_JoystickGetAxis(_joystick, 1) / 32767.0f);
        _axisRightHorizontal.store(SDL_JoystickGetAxis(_joystick, 2) / 32767.0f);
        _axisRightVertical.store(SDL_JoystickGetAxis(_joystick, 3) / 32767.0f);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
