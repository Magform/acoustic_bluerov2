// controller_axes.h
#ifndef CONTROLLER_AXES_H
#define CONTROLLER_AXES_H

#include <atomic>

struct ControllerAxes {
    std::atomic<float> leftVertical{0.0f};
    std::atomic<float> leftHorizontal{0.0f};
    std::atomic<float> rightVertical{0.0f};
    std::atomic<float> rightHorizontal{0.0f};
    std::atomic<float> leftTrigger{0.0f};
    std::atomic<float> rightTrigger{0.0f};
};

#endif // CONTROLLER_AXES_H
