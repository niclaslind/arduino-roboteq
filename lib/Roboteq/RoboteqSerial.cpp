#include "RoboteqSerial.hpp"

RoboteqSerial::RoboteqSerial(Stream &stream)
    : _serial(stream)
{
}

/**
 * @return: fault flags as a byte, each bit is a faultflag
 */
int16_t RoboteqSerial::readFaultFlags()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readFaulFlagQuery, RoboteqCommands::readFaultFlagRespond);
}

/**
 * @return: voltage of all voltage in Roboteq
 */
int16_t RoboteqSerial::readVoltage()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readVoltsQuery, RoboteqCommands::readVoltsRespond);
}

/**
 * @params: uint8_t channel: Motor Channel
 * @return: current motor amp of channel
 */
int16_t RoboteqSerial::readMotorAmps(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMotorAmpsQuery, channel, RoboteqCommands::readMotorAmpsRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readAnalogInput(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readAnalogInputQuery, channel, RoboteqCommands::readAnalogInputRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint16_t RoboteqSerial::readRotorAngle(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRotorAngleQuery, channel, RoboteqCommands::readRotorAngleRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint16_t RoboteqSerial::readRawSinCosSensor(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRawSinConSensorQuery, channel, RoboteqCommands::readRotorAngleRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
bool RoboteqSerial::readUserBooleanValue(uint8_t booleanVariable)
{
    return (bool)this->handleQueryRequestToInt(RoboteqCommands::readUserBooleanValueQuery, booleanVariable, RoboteqCommands::readUserBooleanValueRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readBatteryAmps(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readBatteryAmpsQuery, channel, RoboteqCommands::readBatteryAmpsRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readBrushlessCountRelative(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readBatteryAmpsQuery, channel, RoboteqCommands::readBatteryAmpsRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readBLMotorSpeedInRpm(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readBlMotorSpeedInRpmQuery, channel, RoboteqCommands::readBlMotorSpeedInRpmRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readEncoderCounterAbsolut(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readEncoderCounterAbsolutQuery, channel, RoboteqCommands::readEncoderCounterAbsolutRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readAbsoluteBrushlessCounter(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readAbsolutBrushlessCounterQuery, channel, RoboteqCommands::readAbsolutBrushlessCounterRespond);
}

uint8_t RoboteqSerial::readRawCanReceivedFramesCount()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRawCanRecivedFramesCountQuery, RoboteqCommands::readRawCanRecivedFramesCountRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readConvertedAnalogCommand(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readConvertedAnalogCommandQuery, channel, RoboteqCommands::readConvertedAnalogCommandRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readInternalPulseCommand(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readInternalPulseCommandQuery, channel, RoboteqCommands::readInternalPulseCommandRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readInternalSerialCommand(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readInternalSerialCommandQuery, channel, RoboteqCommands::readInternalSerialCommandRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint32_t RoboteqSerial::readRoboCanAliveNodesMap(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRoboCanAliveNodesMapQuery, channel, RoboteqCommands::readRoboCanAliveNodesMapRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readEncoderCountRelative(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readEncoderCountRelativeQuery, channel, RoboteqCommands::readEncoderCountRelativeRespond);
}

uint32_t RoboteqSerial::readDigitalInputs()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readDigitalInputsQuery, RoboteqCommands::readDigitalInputsRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readIndividualDigitalInputs(uint8_t digitalInputNumber)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readIndividualDigitalInputsQuery, digitalInputNumber, RoboteqCommands::readIndividualDigitalInputsRespond);
}

uint16_t RoboteqSerial::readDigitalOutputStatus()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readDigitalOutputStatusQuery, RoboteqCommands::readDigitalOutputStatusRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readDestinationReached(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readDestinationReachedQuery, channel, RoboteqCommands::readDestinationReachedRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readClosedLoopError(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readClosedLoopErrorQuery, channel, RoboteqCommands::readClosedLoopErrorRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readFeedback(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readFeedbackQuery, channel, RoboteqCommands::readFeedbackRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readFocAngleAdjust(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readFocAngleAdjustQuery, channel, RoboteqCommands::readFocAngleAdjustRespond);
}

String RoboteqSerial::readFirmwareID()
{
    return this->handleQueryRequest(RoboteqCommands::readFirmwareIDQuery, RoboteqCommands::readFirmwareIDRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readRuntimeStatusFlag(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRuntimeStatusFlagQuery, channel, RoboteqCommands::readRuntimeStatusFlagRespond);
}

uint8_t RoboteqSerial::readStatusFlag()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readStatusFlagQuery, RoboteqCommands::readStatusFlagRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readHallSensorStates(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readHallSensorStatesQuery, channel, RoboteqCommands::readHallSensorStatesRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::isRoboCanNodeAlive(uint8_t nodeID)
{
    return this->handleQueryRequestToInt(RoboteqCommands::isRoboCanNodeAliveQuery, nodeID, RoboteqCommands::isRoboCanNodeAliveRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint16_t RoboteqSerial::readSpektrumReceiver(uint8_t radioChannel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readSpektrumReceiverQuery, radioChannel, RoboteqCommands::readSpektrumReceiverRespond);
}

uint8_t RoboteqSerial::readLockStatus()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readLockStatusQuery, RoboteqCommands::readLockStatusRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readMotorCommandApplied(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMotorCommandAppliedQuery, channel, RoboteqCommands::readMotorCommandAppliedRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readFieldOrientedControlMotorAmps(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readFieldOrientedControlMotorAmpsQuery, channel, RoboteqCommands::readFieldOrientedControlMotorAmpsRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readMagsensorTrackDetect(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorTrackDetectQuery, channel, RoboteqCommands::readMagsensorTrackDetectRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readMagsensorMarkers(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorMarkersQuery, channel, RoboteqCommands::readMagsensorMarkersRespond);
}

int16_t RoboteqSerial::readMagsensorStatus()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorStatusQuery, RoboteqCommands::readMagsensorStatusRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readMagsensorTrackPosition(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorTrackPositionQuery, channel, RoboteqCommands::readMagsensorTrackPositionRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readMagsensorGyroscope(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorGyroscopeQuery, channel, RoboteqCommands::readMagsensorGyroscopeRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readMotorPowerOutputApplied(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMotorPowerOutputAppliedQuery, channel, RoboteqCommands::readMotorPowerOutputAppliedRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint16_t RoboteqSerial::readPulseInput(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readPulseInputQuery, channel, RoboteqCommands::readPulseInputRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readPulseInputAfterConversion(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readPulseInputAfterConversionQuery, channel, RoboteqCommands::readPulseInputAfterConversionRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readEncoderMotorSpeedInRpm(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readEncoderMotorSpeedInRpmQuery, channel, RoboteqCommands::readEncoderMotorSpeedInRpmRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint32_t RoboteqSerial::readScriptChecksum()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readScriptChecksumQuery, RoboteqCommands::readScriptChecksumRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readEncoderSpeedRelative(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readEncoderSpeedRelativeQuery, channel, RoboteqCommands::readEncoderSpeedRelativeRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int8_t RoboteqSerial::readTemperature(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readTemperatureQuery, channel, RoboteqCommands::readTemperatureRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint32_t RoboteqSerial::readTime(uint8_t dataElementInNewControllerModel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readTimeQuery, RoboteqCommands::readTimeRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readPositionRelativeTracking(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readPositionRelativeTrackingQuery, channel, RoboteqCommands::readPositionRelativeTrackingRespond);
}

String RoboteqSerial::readControlUnitTypeAndControllerModel()
{
    return this->handleQueryRequest(RoboteqCommands::readControlUnitTypeAndControllerModelQuery, RoboteqCommands::readControlUnitTypeAndControllerModelRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint32_t RoboteqSerial::readMcuID(uint8_t dataElement)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMcuIDQuery, dataElement, RoboteqCommands::readMcuIDRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint16_t RoboteqSerial::readVolts(uint8_t dataElement)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readVoltsQuery, dataElement, RoboteqCommands::readVoltsRespond);
}

uint16_t RoboteqSerial::readVolts()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readVoltsQuery, RoboteqCommands::readVoltsRespond);
}

int32_t RoboteqSerial::readUserIntegerVariable(uint8_t variableNumber)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readUserIntegerVariableQuery, variableNumber, RoboteqCommands::readUserIntegerVariableRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readSlipFrequency(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readSlipFrequencyQuery, channel, RoboteqCommands::readSlipFrequencyRespond);
}


void RoboteqSerial::sendMotorCommand(const char *commandMessage)
{
    this->sendQuery(commandMessage);
}

void RoboteqSerial::sendMotorCommand(const char *commandMessage, uint8_t channel)
{
    String command = commandMessage;
    command += channel;
    command += "_";
    this->sendQuery(command.c_str());
}

void RoboteqSerial::sendMotorCommand(const char *commandMessage, uint8_t channel, int32_t value)
{

}

void RoboteqSerial::sendQuery(const char *message)
{
    this->_serial.write(message, strlen(message));
    this->_serial.flush();
}

String RoboteqSerial::readQuery(const char *message)
{
    String inputString;
    unsigned long startTime = millis();
    while (millis() - startTime < _timeout && _serial.available())
    {
        inputString = _serial.readStringUntil('\r');

        if (inputString.startsWith(message))
        {
            return inputString.substring(inputString.indexOf("=") + 1);
        }
    }
    return "-1";
}


String RoboteqSerial::handleQueryRequest(const char *queryMessage, uint8_t extraParameter, const char *respondMessage)
{
    String query = queryMessage;
    query += String(extraParameter);
    query += "_";

    this->sendQuery(query.c_str());
    return this->readQuery(respondMessage);
}

String RoboteqSerial::handleQueryRequest(const char *queryMessage, const char *respondMessage)
{
    this->sendQuery(queryMessage);
    return this->readQuery(respondMessage);
}

int RoboteqSerial::handleQueryRequestToInt(const char *queryMessage, const char *respondMessage)
{
    this->sendQuery(queryMessage);
    return this->readQuery(respondMessage).toInt();
}

int RoboteqSerial::handleQueryRequestToInt(const char *queryMessage, uint8_t extraParameter, const char *respondMessage)
{
    String query = queryMessage;
    query += String(extraParameter);
    query += "_";

    this->sendQuery(query.c_str());
    return this->readQuery(respondMessage).toInt();
}

