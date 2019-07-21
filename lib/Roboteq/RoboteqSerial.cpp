#include "RoboteqSerial.hpp"

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

int16_t RoboteqSerial::readBLMotorSpeedInRpm(uint8_t channel)
{
    String data = this->concatenateMessage("?BS ", channel);
    this->sendQuery(data.c_str());

    int16_t motorSpeedInRpm = this->readQuery("BS=");
    return motorSpeedInRpm;
}

int32_t RoboteqSerial::readEncoderCounterAbsolut(uint8_t channel)
{
    String data = this->concatenateMessage("?C", channel);
    this->sendQuery(data.c_str());

    int32_t encoderCounterAbsolute = this->readQuery("C=");
    return encoderCounterAbsolute;
}

// ?CB
int32_t RoboteqSerial::readAbsoluteBrushlessCounter(uint8_t channel)
{
}

// ?CF
uint8_t RoboteqSerial::readRawCanReceivedFramesCount()
{
}

// ?CIA
int32_t RoboteqSerial::readConvertedAnalogCommand(uint8_t channel)
{
}

// ?CIP
int32_t RoboteqSerial::readInternalPulseCommand(uint8_t channel)
{
}

// ?CIS
int32_t RoboteqSerial::readInternalSerialCommand(uint8_t channel)
{
}

// ?CL
uint32_t RoboteqSerial::readRoboCanAliveNodesMap(uint8_t channel)
{
}

// ?CR
int32_t RoboteqSerial::readEncoderCountRelative(uint8_t channel)
{
}

// ?D
uint32_t RoboteqSerial::readDigitalInputs()
{
}

// ?DI
uint8_t RoboteqSerial::readIndividualDigitalInputs(uint8_t digitalInputNumber)
{
}

// ?DO
uint16_t RoboteqSerial::readDigitalOutputStatus()
{
}

// ?DR
uint8_t RoboteqSerial::readDestinationReached(uint8_t channel)
{
}

// ?E
int32_t RoboteqSerial::readClosedLoopError(uint8_t channel)
{
}

// ?F
int16_t RoboteqSerial::readFeedback(uint8_t channel)
{
}

// ?FC
int16_t RoboteqSerial::readFocAngleAdjust(uint8_t channel)
{
}

// ?FID
String RoboteqSerial::readFirmwareID()
{
}

// ?FM
int16_t RoboteqSerial::readRuntimeStatusFlag(uint8_t channel)
{
}

// ?FS
uint8_t RoboteqSerial::readStatusFlag()
{
}

// ?HS
uint8_t RoboteqSerial::readHallSensorStates(uint8_t channel)
{
}

// ?ICL
uint8_t RoboteqSerial::isRoboCanNodeAlive(uint8_t nodeID)
{
}

// ?K
uint16_t RoboteqSerial::readSpektrumReceiver(uint8_t radioChannel)
{
}

// ?LK
uint8_t RoboteqSerial::readLockStatus()
{
}

// ?M
uint8_t RoboteqSerial::readMotorCommandApplied(uint8_t channel)
{
}

// ?MA
int16_t RoboteqSerial::readFieldOrientedControlMotorAmps(uint8_t channel)
{
}

// ?MGD
uint8_t RoboteqSerial::readMagsensorTrackDetect(uint8_t channel)
{
}

// ?MGM
uint8_t RoboteqSerial::readMagsensorMarkers(uint8_t channel)
{
}

// ?MGS
int16_t RoboteqSerial::readMagsensorStatus()
{
}

// ?MGT
int16_t RoboteqSerial::readMagsensorTrackPosition(uint8_t channel)
{
}

// ?MGY
int16_t RoboteqSerial::readMagsensorGyroscope(uint8_t channel)
{
}

// ?P
int16_t RoboteqSerial::readMotorPowerOutputApplied(uint8_t channel)
{
}

// ?PI
uint16_t RoboteqSerial::readPulseInput(uint8_t channel)
{
}

// ?PIC
int16_t RoboteqSerial::readPulseInputAfterConversion(uint8_t channel)
{
}

// ?S
int16_t RoboteqSerial::readEncoderMotorSpeedInRpm(uint8_t channel)
{
}

// ?SCC
uint32_t RoboteqSerial::readScriptChecksum()
{
}

// ?SR
int16_t RoboteqSerial::readEncoderSpeedRelative(uint8_t channel)
{
}

// ?T
int8_t RoboteqSerial::readTemperature(uint8_t channel)
{
}

// ?TM
uint32_t RoboteqSerial::readTime(uint8_t dataElementInNewControllerModel)
{
}

// ?TR
int32_t RoboteqSerial::readPositionRelativeTracking(uint8_t channel)
{
}

// ?TRN
String RoboteqSerial::readControlUnitTypeAndControllerModel()
{
}

// ?UID
uint32_t RoboteqSerial::readMcuID(uint8_t dataElement)
{
}

// ?V[ee]
uint16_t RoboteqSerial::readVolts(uint8_t dataElement)
{
}

// ?V
uint16_t RoboteqSerial::readVolts()
{
}

// ?VAR
int32_t RoboteqSerial::readUserIntegerVariable(uint8_t variableNumber)
{
}

// ?SL
int16_t RoboteqSerial::readSlipFrequency(uint8_t channel)
{
}

String RoboteqSerial::concatenateMessage(const char *message, int value)
{
    String data = message;
    data += value;
    data += "\r\n";

    return data;
}

uint16_t RoboteqSerial::readRawCanFrame()
{
    String data = "?CAN\r\n";
    this->sendQuery(data.c_str());

    uint16_t rawCanFrame = this->readQuery("CAN=");
    return rawCanFrame;
    ;
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
