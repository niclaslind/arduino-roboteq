#pragma once

#include <Arduino.h>
#include <Stream.h>
#include "RoboteqAPICommands.hpp"

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
    bool readUserBooleanValue(uint8_t booleanVariable);
    int16_t readBatteryAmps(uint8_t channel);
    int32_t readBrushlessCountRelative(uint8_t channel);
    int16_t readBLMotorSpeedInRpm(uint8_t channel);
    int32_t readEncoderCounterAbsolut(uint8_t channel);
    uint16_t readRawCanFrame();
    int32_t readAbsoluteBrushlessCounter(uint8_t channel);
    uint8_t readRawCanReceivedFramesCount();
    int32_t readConvertedAnalogCommand(uint8_t channel);
    int32_t readInternalPulseCommand(uint8_t channel);
    int32_t readInternalSerialCommand(uint8_t channel);
    uint32_t readRoboCanAliveNodesMap(uint8_t channel);
    int32_t readEncoderCountRelative(uint8_t channel);
    uint32_t readDigitalInputs();
    uint8_t readIndividualDigitalInputs(uint8_t digitalInputNumber);
    uint16_t readDigitalOutputStatus();
    uint8_t readDestinationReached(uint8_t channel);
    int32_t readClosedLoopError(uint8_t channel);
    int16_t readFeedback(uint8_t channel);
    int16_t readFocAngleAdjust(uint8_t channel);
    String readFirmwareID();
    int16_t readRuntimeStatusFlag(uint8_t channel);
    int16_t raedRuntimeStatusFlag(uint8_t channel);
    uint8_t readStatusFlag();
    uint8_t readHallSensorStates(uint8_t channel);
    uint8_t isRoboCanNodeAlive(uint8_t nodeID);
    uint16_t readSpektrumReceiver(uint8_t radioChannel);
    uint8_t readLockStatus();
    uint8_t readMotorCommandApplied(uint8_t channel);
    int16_t readFieldOrientedControlMotorAmps(uint8_t channel);
    uint8_t readMagsensorTrackDetect(uint8_t channel);
    uint8_t readMagsensorMarkers(uint8_t channel);
    int16_t readMagsensorStatus();
    int16_t readMagsensorTrackPosition(uint8_t channel);
    int16_t readMagsensorGyroscope(uint8_t channel);
    int16_t readMotorPowerOutputApplied(uint8_t channel);
    uint16_t readPulseInput(uint8_t channel);
    int16_t readPulseInputAfterConversion(uint8_t channel);
    int16_t readEncoderMotorSpeedInRpm(uint8_t channel);
    uint32_t readScriptChecksum();
    int16_t readEncoderSpeedRelative(uint8_t channel);
    int8_t readTemperature(uint8_t channel);
    uint32_t readTime(uint8_t dataElementInNewControllerModel);
    int32_t readPositionRelativeTracking(uint8_t channel);
    String readControlUnitTypeAndControllerModel();
    uint32_t readMcuID(uint8_t dataElement);
    uint16_t readVolts(uint8_t dataElement);
    uint16_t readVolts();
    int32_t readUserIntegerVariable(uint8_t variableNumber);
    int16_t readSlipFrequency(uint8_t channel);

private:
    void sendQuery(const char *message);
    
    String readQuery(const char *message);
    String handleQueryRequest(const char * queryMessage, uint8_t extraParameter, const char* respondMessage);
    String handleQueryRequest(const char * queryMessage, const char * respondMessage);

    int handleQueryRequestToInt(const char * queryMessage, uint8_t extraParameter, const char* respondMessage);
    int handleQueryRequestToInt(const char * queryMessage, const char * respondMessage);

private:
    Stream &_serial;
    uint8_t _timeout = 200;
};