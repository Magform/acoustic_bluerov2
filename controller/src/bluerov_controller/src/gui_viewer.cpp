#include "gui_viewer.h"
#include <iostream>
#include <iomanip>

GuiViewer::GuiViewer(ControllerAxes& axes, Status* status)
    : _axes(axes), _status(status), _stop_thread(false) {}

GuiViewer::~GuiViewer() {
    stop();
}

void GuiViewer::start() {
    if (_gui_thread.joinable()) return;
    _stop_thread = false;
    _gui_thread = std::thread(&GuiViewer::run, this);
}

void GuiViewer::stop() {
    _stop_thread = true;
    if (_gui_thread.joinable()) {
        _gui_thread.join();
    }
}

void GuiViewer::run() {
    while (!_stop_thread) {
        // Clear terminal screen (Unix ANSI escape sequence)
        std::cout << "\033[2J\033[H";

        std::cout << std::fixed << std::setprecision(2);

        // Print Controller Axes
        std::cout << "=== Controller Axes ===\n";
        std::cout << "Left Vertical:    " << _axes.leftVertical.load() << "\n";
        std::cout << "Left Horizontal:  " << _axes.leftHorizontal.load() << "\n";
        std::cout << "Right Vertical:   " << _axes.rightVertical.load() << "\n";
        std::cout << "Right Horizontal: " << _axes.rightHorizontal.load() << "\n";
        std::cout << "Left Trigger:     " << _axes.leftTrigger.load() << "\n";
        std::cout << "Right Trigger:    " << _axes.rightTrigger.load() << "\n";

        // Optionally print Status if provided
        if (_status) {
            std::cout << "\n=== Status ===\n";
            std::cout << "X: " << _status->x.load() << "\n";
            std::cout << "Y: " << _status->y.load() << "\n";
            std::cout << "Z: " << _status->z.load() << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
