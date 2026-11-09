// controller_axes.h
#ifndef STATUS_H
#define STATUS_H

#include <atomic>

struct Status {
    std::atomic<float> x{0.0f};
    std::atomic<float> y{0.0f};
    std::atomic<float> z{0.0f};
};

#endif // STATUS_H
