#pragma once

#include <stdint.h>
#include <Stream.h>
#include "RoboteqAPICommands.hpp"

using namespace RoboteqApi;

class RoboteqSerial
{
public:
    explicit RoboteqSerial(Stream &stream);

    // Runtime-Queries
public:
    int16_t readMotorAmps();
    int16_t readMotorAmps(uint8_t channel);
    int16_t readAnalogInputAfterConversion(uint8_t channel);
    uint16_t readRotorAngle(uint8_t channel);
    uint16_t readRawSinCosSensor(RoboteqApi::ReadRawSinCosSensorValue value);
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
    uint32_t readRoboCanAliveNodesMap(uint8_t node);
    int32_t readEncoderCountRelative(uint8_t channel);
    uint32_t readDigitalInputs();
    uint8_t readIndividualDigitalInputs(uint8_t digitalInputNumber);
    uint16_t readDigitalOutputStatus();
    uint8_t readDestinationReached(uint8_t channel);
    int32_t readClosedLoopError(uint8_t channel);
    int16_t readFeedback(uint8_t channel);
    int16_t readFocAngleAdjust(uint8_t channel);
    String readFirmwareID();
    int16_t readFaultFlags();
    int16_t readRuntimeStatusFlag(uint8_t channel);
    uint8_t readStatusFlag();
    uint8_t readHallSensorStates(uint8_t channel);
    uint8_t isRoboCanNodeAlive(uint8_t nodeID);
    uint16_t readSpektrumReceiver(uint8_t radioChannel);
    uint8_t readLockStatus();
    uint8_t readMotorCommandApplied(uint8_t channel);
    int16_t readFieldOrientedControlMotorAmps(RoboteqApi::ReadFieldOrientedControlMotorAmpsValue value);
    uint8_t readMagsensorTrackDetect(uint8_t channel);
    uint8_t readMagsensorMarkers(RoboteqApi::ReadMagsensorMarkersValue value);
    int16_t readMagsensorStatus();
    int16_t readMagsensorTrackPosition(RoboteqApi::ReadMagsensorTrackPositionValue value);
    int16_t readMagsensorGyroscope(uint8_t channel);
    int16_t readMotorPowerOutputApplied(uint8_t channel);
    uint16_t readPulseInput(uint8_t channel);
    int16_t readPulseInputAfterConversion(uint8_t channel);
    int16_t readEncoderMotorSpeedInRpm(uint8_t channel);
    uint32_t readScriptChecksum();
    int16_t readEncoderSpeedRelative(uint8_t channel);
    int8_t readTemperature(RoboteqApi::ReadTemperatureValue value);
    uint32_t readTime(RoboteqApi::ReadTimeValue &value);
    int32_t readPositionRelativeTracking(uint8_t channel);
    String readControlUnitTypeAndControllerModel();
    uint32_t readMcuID(RoboteqApi::ReadMcuIdValue value);
    uint16_t readVolts(RoboteqApi::ReadVoltsValue value);
    String readVolts();
    int32_t readUserIntegerVariable(uint8_t variableNumber);
    int16_t readSlipFrequency(uint8_t channel);

    // Motor commands
public:
    void setAcceleration(uint8_t channel, int32_t value);
    void nextAcceleration(uint8_t channel, int32_t value);
    void setUserBooleanVariable(uint8_t varNbr, bool value);
    void spectrumBind(uint8_t channel);
    void setEncoderCounters(uint8_t channel, int32_t value);
    void setBrushlessCounter(uint8_t channel, int32_t value);
    void setMotorCommandViaCan(uint8_t channel, int32_t value);
    void canSend(uint8_t element, uint8_t value);
    void resetIndividualDigitalOutBits(uint8_t outputNbr);
    void setIndividualOutBits(uint8_t outputNbr);
    void setDeceleration(uint8_t channel, int32_t value);
    void setAllDigitalOutBits(uint8_t value);
    void nextDecceleration(uint8_t channel, int32_t value);
    void saveConfigurationInEeprom();
    void emergencyStop();
    void goToSpeedOrRelativePosition(uint8_t channel, int32_t value);
    void loadHomeCounter(uint8_t channel);
    void emergencyStopRelease();
    void stopInAllModes(uint8_t channel);
    void goToAbsoluteDesiredPosition(uint8_t channel, int32_t value);
    void goToRelativeDesiredPosition(uint8_t channel, int32_t value);
    void nextGoToRelativeDesiredPosition(uint8_t channel, int32_t value);
    void nextGoToAbsoluteDesiredPosition(uint8_t channel, int32_t value);
    void microBasicRun(uint8_t mode);
    void setPulseOut(uint8_t channel, int16_t value);
    void setMotorSpeed(uint8_t channel, int32_t value);
    void nextVelocity(uint8_t channel, int32_t value);
    void setUserVarable(uint8_t varNbr, int32_t value);

private:
    void sendMotorCommand(const char *commandMessage);
    void sendMotorCommand(const char *commandMessage, uint8_t argument);
    void sendMotorCommand(const char *commandMessage, uint8_t argument, int32_t value);

private:
    void sendQuery(const char *message);
    String readQuery(const char *message);

private:
    String handleQueryRequest(const char *queryMessage, uint8_t extraParameter, const char *respondMessage);
    String handleQueryRequest(const char *queryMessage, const char *respondMessage);
    int handleQueryRequestToInt(const char *queryMessage, uint8_t extraParameter, const char *respondMessage);
    int handleQueryRequestToInt(const char *queryMessage, const char *respondMessage);

private:
    Stream &_stream;
    uint8_t _timeout = 200;
};