#ifndef GUI_VIEWER_H
#define GUI_VIEWER_H

#include "controller_axes.h"
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>

class GuiViewer {
public:
    explicit GuiViewer(ControllerAxes& axes);
    ~GuiViewer();

    void start();
    void stop();

private:
    void run();

    ControllerAxes& _axes;
    std::atomic<bool> _stop_thread;
    std::thread _gui_thread;
};

#endif // GUI_VIEWER_H
