#ifndef CONTROLLER_READER_H
#define CONTROLLER_READER_H

#include <atomic>
#include <thread>
#include <SDL2/SDL.h>

class ControllerReader {
public:
    ControllerReader(std::atomic<float>& axisLeftVertical, std::atomic<float>& axisLeftHorizontal,
                     std::atomic<float>& axisRightVertical, std::atomic<float>& axisRightHorizontal);
    ~ControllerReader();
    
    void start();
    void stop();

private:
    void read_loop();

    std::atomic<float>& _axisLeftVertical;
    std::atomic<float>& _axisLeftHorizontal;
    std::atomic<float>& _axisRightVertical;
    std::atomic<float>& _axisRightHorizontal;

    std::atomic<bool> _stop_thread;
    std::thread _reader_thread;

    SDL_Joystick* _joystick;
};

#endif // CONTROLLER_READER_H
