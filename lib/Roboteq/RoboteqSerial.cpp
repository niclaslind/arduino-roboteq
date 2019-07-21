#include "RoboteqSerial.hpp"
#include <string>
#include <iostream>


RoboteqSerial::RoboteqSerial(Stream &stream)
    : _serial(stream)
{
}

int16_t RoboteqSerial::readFaultFlags()
{
    this->sendQuery("?FF\r\n");

    int16_t faultflag = this->readQuery("FF=");
    return faultflag;
}

int16_t RoboteqSerial::readVoltage()
{
    this->sendQuery("?V 2\r\n");

    int16_t voltage = this->readQuery("V=");
    return voltage;
}

int16_t RoboteqSerial::readMotorAmps(uint8_t channel)
{
    String data = this->concatenateMessage("?A ", channel);
    this->sendQuery(data.c_str());

    int16_t motorAmps = this->readQuery("A=");
    return motorAmps;
}

int16_t RoboteqSerial::readAnalogInput(uint8_t channel)
{
    String data = this->concatenateMessage("?AIC ", channel);
    this->sendQuery(data.c_str());

    int16_t analogInput = this->readQuery("AIC=");
    return analogInput;
}

uint16_t RoboteqSerial::readRotorAngle(uint8_t channel)
{
    String data = this->concatenateMessage("?ANG ", channel);
    this->sendQuery(data.c_str());

    uint16_t rotorAngle = this->readQuery("ANG=");
    return rotorAngle;
}

uint16_t RoboteqSerial::readRawSinCosSensor(uint8_t channel)
{
    String data = this->concatenateMessage("?ASI ", channel);
    this->sendQuery(data.c_str());

    uint16_t rawSinCosSensor = this->readQuery("ASI=");
    return rawSinCosSensor;
}

uint8_t RoboteqSerial::readUserBooleanValue(uint8_t booleanVariable)
{
    String data = this->concatenateMessage("?B ", booleanVariable);
    this->sendQuery(data.c_str());

    uint8_t userBoolean = this->readQuery("B=");
    return userBoolean;
}

int16_t RoboteqSerial::readBatteryAmps(uint8_t channel)
{
    String data = this->concatenateMessage("?BA ", channel);
    this->sendQuery(data.c_str());

    int16_t batteryAmps = this->readQuery("BA=");
    return batteryAmps;
}

int32_t RoboteqSerial::readBrushlessCountRelative(uint8_t channel)
{
    String data = this->concatenateMessage("?BCR ", channel);
    this->sendQuery(data.c_str());

    int32_t brushlessCountRelative = this->readQuery("BCR=");
    return brushlessCountRelative;
}

int16_t RoboteqSerial::readBLMotorSpeedInRpm(uint8_t channel) {
    String data = this->concatenateMessage("?BS ", channel);
    this->sendQuery(data.c_str());

    int16_t motorSpeedInRpm = this->readQuery("BS=");
    return motorSpeedInRpm;
}

int32_t RoboteqSerial::readEncoderCounterAbsolut(uint8_t channel) {
    String data = this->concatenateMessage("?C", channel);
    this->sendQuery(data.c_str());

    int32_t encoderCounterAbsolute = this->readQuery("C=");
    return encoderCounterAbsolute;
}

uint16_t RoboteqSerial::readRawCanFrame() {
    String data = "?CAN\r\n";
    this->sendQuery(data.c_str());

    uint16_t rawCanFrame = this->readQuery("CAN=");
    return rawCanFrame;;
}

String RoboteqSerial::concatenateMessage(const char *message, int value)
{
    String data = message;
    data += value;
    data += "\r\n";

    return data;
}

int RoboteqSerial::sendQuery(const char *message)
{
    this->_serial.write(message, strlen(message));
    this->_serial.flush();

    return 0;
}

int RoboteqSerial::readQuery(const char *message)
{
    String inputString;
    unsigned long startTime = millis();
    while (millis() - startTime < timeout && _serial.available())
    {
        inputString = _serial.readStringUntil('\r');

        if (inputString.startsWith(message))
        {
            return inputString.substring(inputString.indexOf("=") + 1).toInt();
        }
    }
    return -1;
}
