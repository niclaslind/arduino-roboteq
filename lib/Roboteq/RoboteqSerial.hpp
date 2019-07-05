#pragma once

#include <Arduino.h>
#include <Stream.h>

class RoboteqSerial
{
public:
    RoboteqSerial(Stream &stream);

public:
    int sendQuery(const char* message);
    int readQuery(const char* message);
    
    int queryFaultFlags();
    int queryVoltage();

private:
    Stream &_serial;
    uint8_t timeout = 200;
};