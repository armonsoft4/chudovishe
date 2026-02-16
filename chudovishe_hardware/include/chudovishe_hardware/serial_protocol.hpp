#pragma once
#include <cstdint>

#define HEAD_FRAME 0xABCD

typedef struct {
    uint16_t   header;
    int16_t  leftMotorTicks;
    int16_t  rightMotorTicks;
    uint16_t checksum;
} MotorWheelFeedback;

typedef struct {
    uint16_t   header = HEAD_FRAME;
    int16_t  leftMotorPWM;
    int16_t  rightMotorPWM;
    uint16_t checksum;
} MotorWheelDriveControl;