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
    int16_t readMotorAmps(bool *serialTimedOut=NULL);
    int16_t readMotorAmps(uint8_t channel, bool *serialTimedOut=NULL);
    int16_t readAnalogInputAfterConversion(uint8_t channel, bool *serialTimedOut=NULL);
    uint16_t readRotorAngle(uint8_t channel, bool *serialTimedOut=NULL);
    uint16_t readRawSinCosSensor(RoboteqApi::ReadRawSinCosSensorValue value, bool *serialTimedOut=NULL);
    bool readUserBooleanValue(uint8_t booleanVariable, bool *serialTimedOut=NULL);
    int16_t readBatteryAmps(uint8_t channel, bool *serialTimedOut=NULL);
    int32_t readBrushlessCountRelative(uint8_t channel, bool *serialTimedOut=NULL);
    int32_t readBLMotorSpeedInRpm(uint8_t channel, bool *serialTimedOut=NULL);
    int32_t readEncoderCounterAbsolut(uint8_t channel, bool *serialTimedOut=NULL);
    uint16_t readRawCanFrame(bool *serialTimedOut=NULL);
    int32_t readAbsoluteBrushlessCounter(uint8_t channel, bool *serialTimedOut=NULL);
    uint8_t readRawCanReceivedFramesCount(bool *serialTimedOut=NULL);
    int32_t readConvertedAnalogCommand(uint8_t channel, bool *serialTimedOut=NULL);
    int32_t readInternalPulseCommand(uint8_t channel, bool *serialTimedOut=NULL);
    int32_t readInternalSerialCommand(uint8_t channel, bool *serialTimedOut=NULL);
    uint32_t readRoboCanAliveNodesMap(uint8_t node, bool *serialTimedOut=NULL);
    int32_t readEncoderCountRelative(uint8_t channel, bool *serialTimedOut=NULL);
    uint32_t readDigitalInputs(bool *serialTimedOut=NULL);
    uint8_t readIndividualDigitalInputs(uint8_t digitalInputNumber, bool *serialTimedOut=NULL);
    uint16_t readDigitalOutputStatus(bool *serialTimedOut=NULL);
    uint8_t readDestinationReached(uint8_t channel, bool *serialTimedOut=NULL);
    int32_t readClosedLoopError(uint8_t channel, bool *serialTimedOut=NULL);
    int32_t readFeedback(uint8_t channel, bool *serialTimedOut=NULL);
    int16_t readFocAngleAdjust(uint8_t channel, bool *serialTimedOut=NULL);
    String readFirmwareID(bool *serialTimedOut=NULL);
    uint16_t readFaultFlags(bool *serialTimedOut=NULL);
    uint16_t readRuntimeStatusFlag(uint8_t channel, bool *serialTimedOut=NULL);
    uint16_t readStatusFlag(bool *serialTimedOut=NULL);
    uint8_t readHallSensorStates(uint8_t channel, bool *serialTimedOut=NULL);
    uint8_t isRoboCanNodeAlive(uint8_t nodeID, bool *serialTimedOut=NULL);
    uint16_t readSpektrumReceiver(uint8_t radioChannel, bool *serialTimedOut=NULL);
    uint8_t readLockStatus(bool *serialTimedOut=NULL);
    int32_t readMotorCommandApplied(uint8_t channel, bool *serialTimedOut=NULL);
    int16_t readFieldOrientedControlMotorAmps(RoboteqApi::ReadFieldOrientedControlMotorAmpsValue value, bool *serialTimedOut=NULL);
    uint8_t readMagsensorTrackDetect(uint8_t channel, bool *serialTimedOut=NULL);
    uint8_t readMagsensorMarkers(RoboteqApi::ReadMagsensorMarkersValue value, bool *serialTimedOut=NULL);
    uint16_t readMagsensorStatus(bool *serialTimedOut=NULL);
    int16_t readMagsensorTrackPosition(RoboteqApi::ReadMagsensorTrackPositionValue value, bool *serialTimedOut=NULL);
    int16_t readMagsensorGyroscope(uint8_t channel, bool *serialTimedOut=NULL);
    int16_t readMotorPowerOutputApplied(uint8_t channel, bool *serialTimedOut=NULL);
    uint16_t readPulseInput(uint8_t channel, bool *serialTimedOut=NULL);
    int16_t readPulseInputAfterConversion(uint8_t channel, bool *serialTimedOut=NULL);
    int32_t readEncoderMotorSpeedInRpm(uint8_t channel, bool *serialTimedOut=NULL);
    uint32_t readScriptChecksum(bool *serialTimedOut=NULL);
    int16_t readEncoderSpeedRelative(uint8_t channel, bool *serialTimedOut=NULL);
    int8_t readTemperature(RoboteqApi::ReadTemperatureValue value, bool *serialTimedOut=NULL);
    uint32_t readTime(RoboteqApi::ReadTimeValue &value, bool *serialTimedOut=NULL);
    int32_t readPositionRelativeTracking(uint8_t channel, bool *serialTimedOut=NULL);
    String readControlUnitTypeAndControllerModel(bool *serialTimedOut=NULL);
    uint32_t readMcuID(RoboteqApi::ReadMcuIdValue value, bool *serialTimedOut=NULL);
    uint16_t readVolts(RoboteqApi::ReadVoltsValue value, bool *serialTimedOut=NULL);
    String readVolts(bool *serialTimedOut=NULL);
    int32_t readUserIntegerVariable(uint8_t variableNumber, bool *serialTimedOut=NULL);
    int16_t readSlipFrequency(uint8_t channel, bool *serialTimedOut=NULL);

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

public:
    void startDataStream(const char *prefix, const char *delimiter, const char *query, int32_t dataStreamPeriod_ms);
    int32_t getDataFromStream(const char *prefix, const char *delimiter, int64_t *buf, size_t bufLen);
    int32_t parseDataStream(String &dataStream, const char *prefix, const char *delimiter, int64_t *buf, size_t bufLen);

private:
    void sendMotorCommand(const char *commandMessage);
    void sendMotorCommand(const char *commandMessage, uint8_t argument);
    void sendMotorCommand(const char *commandMessage, uint8_t argument, int32_t value);

private:
    void sendQuery(const char *message);
    String readQuery(const char *message, bool *serialTimedOut=NULL);

private:
    String handleQueryRequest(const char *queryMessage, uint8_t extraParameter, const char *respondMessage, bool *serialTimedOut=NULL);
    String handleQueryRequest(const char *queryMessage, const char *respondMessage, bool *serialTimedOut=NULL);
    int32_t handleQueryRequestToInt(const char *queryMessage, uint8_t extraParameter, const char *respondMessage, bool *serialTimedOut=NULL);
    int32_t handleQueryRequestToInt(const char *queryMessage, const char *respondMessage, bool *serialTimedOut=NULL);

private:


private:
    Stream &_stream;
    uint8_t _timeout = 200;
};
