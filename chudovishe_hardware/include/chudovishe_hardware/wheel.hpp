#pragma once

#include <string>
#include <cmath>

#define ENCODER_MIN_VALUE 0
#define ENCODER_MAX_VALUE 9000
#define ENCODER_LOW_WRAP_FACTOR 0.3
#define ENCODER_HIGH_WRAP_FACTOR 0.7

namespace chudovishe_hardware
{
    class MotorWheel {
    public:
        std::string name = "";
        double command = 0.0;
        double position = 0.0;
        double velocity = 0.0;

        MotorWheel() = default;
    };
}