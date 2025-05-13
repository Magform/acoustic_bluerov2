#ifndef CONTROLLER_READER_H
#define CONTROLLER_READER_H

#include <atomic>
#include <thread>
#include <SDL2/SDL.h>

#include "controller_axes.h"

class ControllerReader {
    public:
        ControllerReader(ControllerAxes& axes, float dead_zone);
        ~ControllerReader();
        
        void start();
        void stop();
    
    private:
        void read_loop();
        float apply_dead_zone(float value);
    
        ControllerAxes& _axes;
    
        float _dead_zone;
        std::atomic<bool> _stop_thread;
        std::thread _reader_thread;
    
        SDL_Joystick* _joystick;
    };
    

#endif // CONTROLLER_READER_H
