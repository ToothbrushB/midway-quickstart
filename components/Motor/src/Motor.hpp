#include <cstdint>
#pragma once

class Motor {
public:
    Motor();
    void set(bool dir, uint32_t speed);
    void brake();
    void stop();
};