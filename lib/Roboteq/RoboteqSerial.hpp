#pragma once

#include <Arduino.h>
#include <Stream.h>

class RoboteqSerial
{
public:
    explicit RoboteqSerial(Stream &stream);

public:
    int16_t readFaultFlags();
    int16_t readVoltage();
    int16_t readMotorAmps(uint8_t channel);
    int16_t readAnalogInput(uint8_t channel);
    uint16_t readRotorAngle(uint8_t channel);
    uint16_t readRawSinCosSensor(uint8_t channel);
    uint8_t readUserBooleanValue(uint8_t booleanVariable);
    int16_t readBatteryAmps (uint8_t channel);
    int32_t readBrushlessCountRelative(uint8_t channel);
    int16_t readBLMotorSpeedInRpm(uint8_t channel);
    int32_t readEncoderCounterAbsolut(uint8_t channel);
    uint16_t readRawCanFrame();

private:
    String concatenateMessage(const char * message, int value);
    int readQuery(const char *message);
    int sendQuery(const char *message);

private:
    Stream &_serial;
    uint8_t timeout = 200;
};