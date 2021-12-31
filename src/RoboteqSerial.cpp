#if ARDUINO >= 100
#include "Arduino.h"
#else
extern "C" {
#include "WConstants.h"
}
#endif

#include "RoboteqSerial.hpp"

RoboteqSerial::RoboteqSerial(Stream &stream)
    : _stream(stream)
{
}

/**
 * @description: Measures and reports the motor Amps, in Amps*10, for all operating channels. 
 *              For brush-less controllers this query reports the RMS value. 
 *              Note that the current flowing through the motors is often higher than this flowing through the battery
 * 
 * @note: Single channel controllers will report a single value. 
 *      Some power board units measure the Mo-tor Amps and calculate the Battery Amps, while other models measure the Battery Amps and calculate the Motor Amps. 
 *      The measured Amps is always more precise than the calculated Amps. See controller datasheet to find which Amps is measured by your particular model.
 * 
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Amps *10 for each channel
 */
int16_t RoboteqSerial::readMotorAmps(bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMotorAmpsQuery, RoboteqCommands::readMotorAmpsRespond, serialTimedOut);
}

/**
 * @description: Measures and reports the motor Amps, in Amps*10, for all operating channels. 
 *              For brush-less controllers this query reports the RMS value. 
 *              Note that the current flowing through the motors is often higher than this flowing through the battery
 * 
 * @note: Single channel controllers will report a single value. 
 *      Some power board units measure the Mo-tor Amps and calculate the Battery Amps, while other models measure the Battery Amps and calculate the Motor Amps. 
 *      The measured Amps is always more precise than the calculated Amps. See controller datasheet to find which Amps is measured by your particular model.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Amps *10 for current channel
 */
int16_t RoboteqSerial::readMotorAmps(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMotorAmpsQuery, channel, RoboteqCommands::readMotorAmpsRespond, serialTimedOut);
}

/**
 * @description: Returns value of an Analog input after all the adjustments are performed to convert it to a command or feedback value (Min/Max/Center/Deadband/Linearity). 
 *              If an input is disabled, the query returns 0. The total number of Analog input channels varies from one controller model to another and can be found in the product datasheet.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Converted analog input value +/-1000 range
 */
int16_t RoboteqSerial::readAnalogInputAfterConversion(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readAnalogInputQuery, channel, RoboteqCommands::readAnalogInputRespond, serialTimedOut);
}

/**
 * @description: On brushless controller operating in sinusoidal mode, this query returns the real time val-ue of the rotor’s electrical angle of brushless motor. 
 *                 This query is useful for verifying trou-bleshooting sin/cos and SPI/SSI sensors. Angle are reported in 0-511 degrees
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Rotor electrical angle
 */
uint16_t RoboteqSerial::readRotorAngle(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRotorAngleQuery, channel, RoboteqCommands::readRotorAngleRespond, serialTimedOut);
}

/**
 * @description: Returns real time raw values of ADC connected to sin/cos sensors of each motor or the real time values of the raw data reported by the SSI sensor of the motor. 
 *              This query is useful for verifying troubleshooting sin/cos sensors and SSI sensors.
 * 
 * @params: ReadRawSinCosSensorValue value: Which input you want to read
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: ADC value
 */
uint16_t RoboteqSerial::readRawSinCosSensor(RoboteqApi::ReadRawSinCosSensorValue value, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRawSinConSensorQuery, uint8_t(value), RoboteqCommands::readRotorAngleRespond, serialTimedOut);
}

/**
 * @description: Read the value of boolean internal variables that can be read and written to/from within a user MicroBasic script. 
 *              It is used to pass boolean states between user scripts and a microcomputer connected to the controller. 
 *              The total number of user boolean variables varies from one controller model to another and can be found in the product datasheet
 * 
 * @params: uint8_t booleanVariable: Boolean variable number
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: 0 or 1 state of the variable
 */
bool RoboteqSerial::readUserBooleanValue(uint8_t booleanVariable, bool *serialTimedOut)
{
    return (bool)this->handleQueryRequestToInt(RoboteqCommands::readUserBooleanValueQuery, booleanVariable, RoboteqCommands::readUserBooleanValueRespond, serialTimedOut);
}

/**
 * @description: Measures and reports the Amps flowing from the battery in Amps * 10.
 *               Battery Amps are often lower than motor Amps
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Amps *10 for each channel
 */
int16_t RoboteqSerial::readBatteryAmps(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readBatteryAmpsQuery, channel, RoboteqCommands::readBatteryAmpsRespond, serialTimedOut);
}

/**
 * @description: Returns the amount of Internal sensor (Hall, SinCos, Resolver) counts that have been measured from the last time this query was made. 
 *              Relative counter read is sometimes easier to work with, compared to full counter reading, as smaller numbers are usually returned.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Value
 */
int32_t RoboteqSerial::readBrushlessCountRelative(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readBatteryAmpsQuery, channel, RoboteqCommands::readBatteryAmpsRespond, serialTimedOut);
}

/**
 * @description: On brushless motor controllers, reports the actual speed measured using the motor’s In-ternal sensors (Hall, SinCos, Resolver) as the actual RPM value. 
 *              To report RPM accurately, the correct number of motor poles must be loaded in the BLPOL configuration parameter
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Speed in RPM
 */
int32_t RoboteqSerial::readBLMotorSpeedInRpm(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readBlMotorSpeedInRpmQuery, channel, RoboteqCommands::readBlMotorSpeedInRpmRespond, serialTimedOut);
}

/**
 * @description: Returns the encoder value as an absolute number. 
 *              The counter is 32-bit with a range of +/- 2147483648 counts.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Absolute counter value
 */
int32_t RoboteqSerial::readEncoderCounterAbsolut(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readEncoderCounterAbsolutQuery, channel, RoboteqCommands::readEncoderCounterAbsolutRespond, serialTimedOut);
}

/**
 * @description: On brushless motor controllers, returns the running total of Internal sensor (Hall, SinCos, Resolver) transition value as an absolute number. 
 *              The counter is 32-bit with a range of +/- 2147483648 counts.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Absolute counter value
 */
int32_t RoboteqSerial::readAbsoluteBrushlessCounter(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readAbsolutBrushlessCounterQuery, channel, RoboteqCommands::readAbsolutBrushlessCounterRespond, serialTimedOut);
}

/**
 * @description: This query is used in CAN-enabled controllers to read the content of a received CAN frame in the RawCAN mode. 
 *              Data will be available for reading with this query only after a ?CF query is first used to check how many received frames are pending in the FIFO buffer. 
 *              When the query is sent without arguments, the controller replies by outputting all elements of the frame separated by colons
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Number of frames in receive queue
 */
uint8_t RoboteqSerial::readRawCanReceivedFramesCount(bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRawCanRecivedFramesCountQuery, RoboteqCommands::readRawCanRecivedFramesCountRespond, serialTimedOut);
}

/**
 * @description: Returns the motor command value that is computed from the Analog inputs whether or not the command  is  actually  applied  to  the  motor.  
 *              The Analog inputs must be con-figured as Motor Command. This  query  can  be  used,  for  example,  to  read  the com-mand  joystick  from within  a  
 *              MicroBasic  script  or  from  an  external  microcomputer,  even though  the  controller  may  be  currently  responding  to  Serial or Pulse  command  because  of  a higher priority setting. 
 *              The returned value is the raw Analog input value with all the adjustments performed to convert it to a command (Min/Max/Center/Deadband/Linearity).
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Command value in +/-1000 range
 */
int32_t RoboteqSerial::readConvertedAnalogCommand(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readConvertedAnalogCommandQuery, channel, RoboteqCommands::readConvertedAnalogCommandRespond, serialTimedOut);
}

/**
 * @description: Returns the motor command value that is computed from the Pulse inputs whether or not the command  is  actually  applied  to  the  motor. The Pulse input must be configured as Motor Command. 
 *              This  query  can  be  used,  for  example,  to  read  the command  joystick  from  within  a  MicroBasic  script  or  from  an  external  microcomputer,  
 *              even though the controller may be currently responding to Serial or Analog command because of a higher priority setting. 
 *              The returned value is the raw Pulse input value with all the adjust-ments performed to convert it to a command (Min/Max/Center/Deadband/Linearity)
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Command value in +/-1000 range 
 */
int32_t RoboteqSerial::readInternalPulseCommand(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readInternalPulseCommandQuery, channel, RoboteqCommands::readInternalPulseCommandRespond, serialTimedOut);
}

/**
 * @description: Returns the motor command value that is issued from the serial input or from a MicroBa-sic script whether  or  not  the  command  is  actually  applied  to  the  motor. 
 *              This  query  can  be  used,  for example,  to  read  from  an  external  microcomputer  the  command  generated  inside  MicroBasic script, 
 *              even though the controller may be currently respond-ing to a Pulse or Analog command because of a higher priority setting.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Command value in +/-1000 range 
 */
int32_t RoboteqSerial::readInternalSerialCommand(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readInternalSerialCommandQuery, channel, RoboteqCommands::readInternalSerialCommandRespond, serialTimedOut);
}

/**
 * @description: With CL it is possible to see which nodes in a RoboCAN are alive and what type of device is present at each node. A complete state of the network is represented in sixteen 32-bit numbers. 
 *              Within each 32-bit word are 8 groups of 4-bits. 
 *              The 4-bits contain the node information. E.g. bits 0-3 of first number is for node 0, bits 8-11 of first number is for node 2, bits 4-7 of second number is for node 5 and bits 12-15 of fourth number is for node 11, etc.
 * 
 * @params: uint8_t nodes: 1= Nodes 0-3 ... 16= Nodes 124-127
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: 4 words of 4 bits.
 */
uint32_t RoboteqSerial::readRoboCanAliveNodesMap(uint8_t nodes, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRoboCanAliveNodesMapQuery, nodes, RoboteqCommands::readRoboCanAliveNodesMapRespond, serialTimedOut);
}

/**
 * @description: Returns the amount of counts that have been measured from the last time this query was made. 
 *              Relative counter read is sometimes easier to work with, compared to full counter reading, as smaller numbers are usually returned.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Counts since last read using ?CR
 */
int32_t RoboteqSerial::readEncoderCountRelative(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readEncoderCountRelativeQuery, channel, RoboteqCommands::readEncoderCountRelativeRespond, serialTimedOut);
}

/**
 * @description: Reports the status of each of the available digital inputs. The query response is a single digital number which must be converted to binary and gives the status of each of the in-puts. 
 *              The total number of Digital input channels varies from one controller model to anoth-er and can be found in the product datasheet
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: b1 + b2*2 + b3*4 + ... +bn*2^n-1
 */
uint32_t RoboteqSerial::readDigitalInputs(bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readDigitalInputsQuery, RoboteqCommands::readDigitalInputsRespond, serialTimedOut);
}

/**
 * @description: Reports the status of an individual Digital Input. The query response is a boolean value (0 or 1). 
 *              The total number of Digital input channels varies from one controller model to anoth-er and can be found in the product datasheet.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: 0 or 1 state for each input
 */
uint8_t RoboteqSerial::readIndividualDigitalInputs(uint8_t digitalInputNumber, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readIndividualDigitalInputsQuery, digitalInputNumber, RoboteqCommands::readIndividualDigitalInputsRespond, serialTimedOut);
}

/**
 * @description: Reads the actual state of all digital outputs. The response to that query is a single number which must be converted into binary in order to read the status of the individual output bits.
 *               When querying an individual output, the reply is 0 or 1 depending on its status. The total number of Digital output channels varies from one controller model to another and can be found in the product datasheet.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: d1 + d2*2 + d3*4 + ... + dn * 2^n-1
 */
uint16_t RoboteqSerial::readDigitalOutputStatus(bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readDigitalOutputStatusQuery, RoboteqCommands::readDigitalOutputStatusRespond, serialTimedOut);
}

/**
 * @description: This query is used when chaining commands in Position Count mode, to detect that a destination has been reached and that the next destination values that were loaded in the buffer have become active. 
 *              The Destination Reached bit is latched and is cleared once it has been read.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: 0: Not yet reached, 1: Reached
 */
uint8_t RoboteqSerial::readDestinationReached(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readDestinationReachedQuery, channel, RoboteqCommands::readDestinationReachedRespond, serialTimedOut);
}

/**
 * @description: In closed-loop modes, returns the difference between the desired speed or position and the measured feedback. 
 *              This query can be used to detect when the motor has reached the desired speed or position. In open loop mode, this query returns 0
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Error value
 */
int32_t RoboteqSerial::readClosedLoopError(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readClosedLoopErrorQuery, channel, RoboteqCommands::readClosedLoopErrorRespond, serialTimedOut);
}

/**
 * @description: Reports the value of the feedback sensors that are associated to each of the channels in closed-loop modes. 
 *              The feedback source can be Encoder, Analog or Pulse. Selecting the feedback source is done using the encoder, pulse or analog configuration parameters.
 *               This query is useful for verifying that the correct feedback source is used by the channel in the closed-loop mode and that its value is in range with expectations
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Feedback values
 */
int32_t RoboteqSerial::readFeedback(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readFeedbackQuery, channel, RoboteqCommands::readFeedbackRespond, serialTimedOut);
}

/**
 * @description: Read in real time the angle correction that is currently applied by the Field Oriented algo-rithm in order achieve optimal performance
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Angle correction
 */
int16_t RoboteqSerial::readFocAngleAdjust(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readFocAngleAdjustQuery, channel, RoboteqCommands::readFocAngleAdjustRespond, serialTimedOut);
}


// TODO: Make an overload of this function and make the user choose which fault flag we should look for
/**
 * @description: Reports the status of the controller fault conditions that can occur during operation.
 *              The response to that query is a single number which must be converted into binary in order to evaluate each of the individual status bits that compose it.
 * 
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: f1 + f2*2 + f3*4 + ... + fn*2^n-1
 *          f1= Overheat 
 *          f2= Overvoltage 
 *          f3= Undervoltage 
 *          f4= Short circuit 
 *          f5= Emergency stop 
 *          f6= Motor/Sensor Setup fault 
 *          f7= MOSFET failure 
 *          f8= Default configuration loaded at startup
 */
uint16_t RoboteqSerial::readFaultFlags(bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readFaulFlagQuery, RoboteqCommands::readFaultFlagRespond, serialTimedOut);
}

/**
 * @description: This query will report a string with the date and identification of the firmware revision of the controller.
 * 
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Firmware ID string
 */
String RoboteqSerial::readFirmwareID(bool *serialTimedOut)
{
    return this->handleQueryRequest(RoboteqCommands::readFirmwareIDQuery, RoboteqCommands::readFirmwareIDRespond, serialTimedOut);
}

// TODO: Make an overload of this function and make the user choose which fault flag we should look for
/**
 * @description: Report the runtime status of each motor. The response to that query is a single number which must be converted into binary in order to evaluate each of the individual status bits that compose it.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return:  f1 + f2*2 + f3*4 + ... + fn*2^n-1
 *      f1 = Amps Limit currently active
 *      f2 = Motor stalled
 *      f3 = Loop Error detected
 *      f4 = Safety Stop active
 *      f5 = Forward Limit triggered
 *      f6 = Reverse Limit triggered
 *      f7 = Amps Trigger activated
 */
uint16_t RoboteqSerial::readRuntimeStatusFlag(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRuntimeStatusFlagQuery, channel, RoboteqCommands::readRuntimeStatusFlagRespond, serialTimedOut);
}

// TODO: Make an overload of this function and make the user choose which fault flag we should look for
/**
 *  @description: Report the state of status flags used by the controller to indicate a number of internal conditions during normal operation. 
 *              The response to this query is the single number for all status flags. The status of individual flags is read by converting this number to binary and look at various bits of that number
 * 
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return:  f1 + f2*2 + f3*4 + ... + fn*2^n-1
 *          f1 = Serial mode
 *          f2 = Pulse mode 
 *          f3 = Analog mode 
 *          f4 = Power stage off 
 *          f5 = Stall detected 
 *          f6 = At limit 
 *          f7 = Unused 
 *          f8 = MicroBasic script running
 *          f9 = Motor/Sensor Tuning mode
 */
uint16_t RoboteqSerial::readStatusFlag(bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readStatusFlagQuery, RoboteqCommands::readStatusFlagRespond, serialTimedOut);
}

// TODO: Make an overload of this function and make the user choose which fault flag we should look for
/**
 *  @description: Reports that status of the hall sensor inputs. This function is mostly useful for trouble-shooting. When no sensors are connected, all inputs are pulled high and the value 7 will be replied.
 *              For 60 degrees spaced Hall sensors, 0-1- 3-4- 6-7 are valid combinations, while 2 and 5 are invalid combinations. 
 *              For 120 degrees spaced sensors, 1-2- 3-4- 5-6 are valid combinations, while 0 and 7 are invalid combinations. In normal conditions, valid values should appear at one time or the other as the motor shaft is rotated
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: HS= ha + 2*hb + 4*hc
 *         ha = hall sensor A
 *         hb = hall sensor B
 *         hc = hall sensor C
 */
uint8_t RoboteqSerial::readHallSensorStates(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readHallSensorStatesQuery, channel, RoboteqCommands::readHallSensorStatesRespond, serialTimedOut);
}

/**
 * @description: This query is used to determine if specific RoboCAN node is alive on CAN bus.
 * 
 * @params: uint8_t nodeID: Node ID
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: 
 *          0 : Not present
 *          1 : Alive
 */
uint8_t RoboteqSerial::isRoboCanNodeAlive(uint8_t nodeID, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::isRoboCanNodeAliveQuery, nodeID, RoboteqCommands::isRoboCanNodeAliveRespond, serialTimedOut);
}

/**
 * @description: On controller models with Spektrum radio support, this query is used to read the raw values of each of up to 6 receive channels. When signal is received, this query returns the value 0.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Raw joystick value, or 0 if transmitter is off or out of range
 */
uint16_t RoboteqSerial::readSpektrumReceiver(uint8_t radioChannel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readSpektrumReceiverQuery, radioChannel, RoboteqCommands::readSpektrumReceiverRespond, serialTimedOut);
}

/**
 * @description: Returns the status of the lock flag. If the configuration is locked, then it will not be possi-ble to read any configuration parameters until the lock is removed or until the parameters are reset to factory default.
 *             This feature is useful to protect the controller configuration from being copied by unauthorized people.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: 
 *      0: unlocked 
 *      1: locked
 */
uint8_t RoboteqSerial::readLockStatus(bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readLockStatusQuery, RoboteqCommands::readLockStatusRespond, serialTimedOut);
}

/**
 * @description: Reports the command value that is being used by the controller. The number that is reported will be depending on which mode is selected at the time. 
 *              The choice of one command mode vs. another is based on the command priority mechanism.  
 *              In the Serial mode, the reported value will be the command that is entered in via the RS232, RS485, TCP or USB port and to which an optional exponential correction is applied.  
 *              In the Analog and Pulse modes, this query will report the Analog or Pulse input after it is being convert-ed using the min, max, center, deadband, and linearity corrections.  
 *              This query is useful for viewing which command is actually being used and the effect of the correction that is being applied to the raw input.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Command value used for each motor. 0 to +/-1000 range
 */
int32_t RoboteqSerial::readMotorCommandApplied(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMotorCommandAppliedQuery, channel, RoboteqCommands::readMotorCommandAppliedRespond, serialTimedOut);
}

/**
 * @description: On brushless motor controllers operating in sinusoidal mode, this query returns the Torque (also known as Quadrature or Iq) current, and the Flux (also known as Direct, or Id) current. 
 *              Current is reported in Amps x 10.
 * 
 * @params: uint8_t ReadFieldOrientedControlMotorAmpsValue: value
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Amps * 10
 */
int16_t RoboteqSerial::readFieldOrientedControlMotorAmps(RoboteqApi::ReadFieldOrientedControlMotorAmpsValue value, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readFieldOrientedControlMotorAmpsQuery, uint8_t(value), RoboteqCommands::readFieldOrientedControlMotorAmpsRespond, serialTimedOut);
}

/**
 * @description: When one or more MGS1600 Magnetic Guide Sensors are connected to the controller, this query reports whether a magnetic tape is within the detection range of the sensor. 
 *              If no tape is detected, the output will be 0. If only one sensor is connected to any pulse in-put, no argument is needed for this query. 
 *              If more than one sensor is connected to pulse inputs and these inputs are enabled and configured in Magsensor MultiPWM mode, then the argument following the query is used to select the sensor
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: 
 *         0: No track detected
 *         1: Track detected
 */
uint8_t RoboteqSerial::readMagsensorTrackDetect(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorTrackDetectQuery, channel, RoboteqCommands::readMagsensorTrackDetectRespond, serialTimedOut);
}

/**
 * @description: When one or more MGS1600 Magnetic Guide Sensors are connected to the controller, this query reports whether left or right markers are present under sensor. 
 *              If only one sen-sor is connected to any pulse input this query will report the data of that sensor, regard-less which pulse input it is connected to. 
 *              If more than one sensor is connected to pulse inputs and these inputs are enabled and configured in Magsensor MultiPWM mode, then the argument following the query is used to select the sensor
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: 0: No marker detected, 1: Marker detected
 */
uint8_t RoboteqSerial::readMagsensorMarkers(RoboteqApi::ReadMagsensorMarkersValue value, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorMarkersQuery, uint8_t(value), RoboteqCommands::readMagsensorMarkersRespond, serialTimedOut);
}

/**
 * @description: When one or more MGS1600 Magnetic Guide Sensors are connected to the controller, this query reports the state of the sensor. 
 *              If only one sensor is connected to any pulse in-put, no argument is needed for this query. 
 *              If more than one sensor is connected to pulse inputs and these inputs are enabled and configured in Magsensor MultiPWM mode, then the argument following the query is used to select the sensor
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: 1 + f2*2 + f3*4 + ... + fn*2n-1
 *          f1: Tape cross
 *          f2: Tape detect
 *          f3: Left marker present
 *          f4: Right marker present
 *          f9: Sensor active
 */
uint16_t RoboteqSerial::readMagsensorStatus(bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorStatusQuery, RoboteqCommands::readMagsensorStatusRespond, serialTimedOut);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Position in millimeters
 */
int16_t RoboteqSerial::readMagsensorTrackPosition(RoboteqApi::ReadMagsensorTrackPositionValue value, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorTrackPositionQuery, uint8_t(value), RoboteqCommands::readMagsensorTrackPositionRespond, serialTimedOut);
}

/**
 * @description: When one or more MGS1600 Magnetic Guide Sensors are connected to the controller, this query reports the position of the tracks detected under the sensor. 
 *              If only one sensor is connected to any pulse input, the argument following the query selects which track to read. 
 *              If more than one sensor is connected to pulse inputs and these inputs are enabled and configured in Magsensor MultiPWM mode, then the argument following the query is used to select the sensor. 
 *              The reported position of the magnetic track in millimeters, us-ing the center of the sensor as the 0 reference
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Gyroscope value
 */
int16_t RoboteqSerial::readMagsensorGyroscope(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorGyroscopeQuery, channel, RoboteqCommands::readMagsensorGyroscopeRespond, serialTimedOut);
}

/**
 * @description: Reports the actual PWM level that is being applied to the motor at the power output stage. 
 *              This value takes into account all the internal corrections and any limiting resulting from temperature or over current. 
 *              A value of 1000 equals 100% PWM. The equivalent voltage at the motor wire is the battery voltage * PWM level.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: 0 to +/-1000 power level
 */
int16_t RoboteqSerial::readMotorPowerOutputApplied(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMotorPowerOutputAppliedQuery, channel, RoboteqCommands::readMotorPowerOutputAppliedRespond, serialTimedOut);
}

/**
 * @description: Reports the value of each of the enabled pulse input captures. The value is the raw num-ber in microseconds when configured in Pulse Width mode. 
 *              In Frequency mode, the returned value is in Hertz. In Duty Cycle mode, the reported value ranges between 0 and 4095 when the pulse duty cycle is 0% and 100% respectively. 
 *              In Pulse Count mode, the reported value in the number of pulses as detected. This counter only increments. In order to reset that counter the pulse capture mode needs to be set back to disabled and then again to Pulse Count.
 * 
 * @params: uint8_t channel: Pulse capture input number
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Value of pulse input
 */
uint16_t RoboteqSerial::readPulseInput(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readPulseInputQuery, channel, RoboteqCommands::readPulseInputRespond, serialTimedOut);
}

/**
 * @description: Returns value of a Pulse input after all the adjustments were performed to convert it to a command or feedback value (Min/Max/Center/Deadband/Linearity). 
 *              If an input is disabled, the query returns 0.
 * 
 * @params: uint8_t channel: Pulse input number
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Converted input value to +/-1000 range
 */
int16_t RoboteqSerial::readPulseInputAfterConversion(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readPulseInputAfterConversionQuery, channel, RoboteqCommands::readPulseInputAfterConversionRespond, serialTimedOut);
}

/**
 * @description: Reports the actual speed measured by the encoders as the actual RPM value. To report RPM accurately, the correct Pulses per Revolution (PPR) must be stored in the encoder configuration
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Speed in RPM
 */
int32_t RoboteqSerial::readEncoderMotorSpeedInRpm(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readEncoderMotorSpeedInRpmQuery, channel, RoboteqCommands::readEncoderMotorSpeedInRpmRespond, serialTimedOut);
}

/**
 * @description: Scans the script storage memory and computes a checksum number that is unique to each script. If not script is loaded the query outputs the value 0xFFFFFFFF. 
 *              Since a stored script cannot be read out, this query is useful for determining if the correct version of a given script is loaded.
 * 
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Checksum number
 */
uint32_t RoboteqSerial::readScriptChecksum(bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readScriptChecksumQuery, RoboteqCommands::readScriptChecksumRespond, serialTimedOut);
}

/**
 * @description: Returns the measured motor speed as a ratio of the Max RPM (MXRPM) configuration parameter. 
 *              The result is a value of between 0 and +/1000. As an example, if the Max RPM is set at 3000 inside the encoder configuration parameter and the motor spins at 1500 RPM,
 *              then the returned value to this query will be 500, which is 50% of the 3000 max. 
 *              Note that if the motor spins faster than the Max RPM, the returned value will exceed 1000. However, a larger value is ignored by the controller for its internal operation.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Speed relative to max
 */
int16_t RoboteqSerial::readEncoderSpeedRelative(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readEncoderSpeedRelativeQuery, channel, RoboteqCommands::readEncoderSpeedRelativeRespond, serialTimedOut);
}

/**
 * @description: Reports the temperature at each of the Heatsink sides and on the internal MCU silicon chip. The reported value is in degrees C with a one degree resolution.
 * 
 * @note: On some controller models, additional temperature values may reported. These are mea-sured at different points and not documented. You may safely ignore this extra data. 
 *      Other controller models only have one heatsink temperature sensor and therefore only report one value in addition to the Internal IC temperature.
 * 
 * @params: ReadTemperatureValue value: Temp element you want to read
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Temperature in degrees
 */
int8_t RoboteqSerial::readTemperature(RoboteqApi::ReadTemperatureValue value, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readTemperatureQuery, uint8_t(value), RoboteqCommands::readTemperatureRespond, serialTimedOut);
}

/**
 * @description: Reports the value of the time counter in controller models equipped with Real-Time clocks with internal or external battery backup. 
 *              On older controller models, time is count-ed in a 32-bit counter that keeps track the total number of seconds, and that can be converted into a full day and time value using external calculation. 
 *              On newer models, the time is kept in multiple registers for seconds, minutes, hours (24h format), dayofmonth, month, year in full
 * 
 * @params: ReadTimeValue value: Date element in new controller model
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Value
 */
// TODO: check this fuction
uint32_t RoboteqSerial::readTime(RoboteqApi::ReadTimeValue &value, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readTimeQuery, RoboteqCommands::readTimeRespond, serialTimedOut);
}

/**
 * @description: Reads the real-time value of the expected motor position in the position tracking closed loop mode and in speed position
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Position
 */
int32_t RoboteqSerial::readPositionRelativeTracking(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readPositionRelativeTrackingQuery, channel, RoboteqCommands::readPositionRelativeTrackingRespond, serialTimedOut);
}

/**
 * @description: Reports two strings identifying the Control Unit type and the Controller Model type. This query is useful for adapting the user software application to the controller model that is attached to the computer.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Control Unit Id String:Controller Model Id String
 */
String RoboteqSerial::readControlUnitTypeAndControllerModel(bool *serialTimedOut)
{
    return this->handleQueryRequest(RoboteqCommands::readControlUnitTypeAndControllerModelQuery, RoboteqCommands::readControlUnitTypeAndControllerModelRespond, serialTimedOut);
}

/**
 * @description: Reports MCU specific information. This query is useful for determining the type of MCU: 100 = STM32F10X, 300 = STM32F30X. The query also produces a unique Id number that is stored on the MCU silicon.
 * 
 * @params: ReadMcuIdValue &value: Data element you want to read
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Value 
 */
uint32_t RoboteqSerial::readMcuID(RoboteqApi::ReadMcuIdValue value, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMcuIDQuery, uint8_t(value), RoboteqCommands::readMcuIDRespond, serialTimedOut);
}

/**
 * @description: Reports the voltages measured inside the controller at three locations: the main battery voltage, 
 *              the internal voltage at the motor driver stage, and the voltage that is available on the 5V output on the DSUB 15 or 25 front connector. 
 *              For safe operation, the driver stage voltage must be above 12V. 
 *              The 5V output will typically show the controller’s internal regulated 5V minus the drop of a diode that is used for protection and will be in the 4.7V range. 
 *              The battery voltage is monitored for detecting the undervoltage or overvoltage con-ditions.
 * 
 * @params: uint8_t channel: MotorChannel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: voltage value in millivolts
 */
uint16_t RoboteqSerial::readVolts(RoboteqApi::ReadVoltsValue value, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readVoltsQuery, uint8_t(value), RoboteqCommands::readVoltsRespond, serialTimedOut);
}

/**
 * @description: Reports the voltages measured inside the controller at three locations: the main battery voltage, 
 *              the internal voltage at the motor driver stage, and the voltage that is available on the 5V output on the DSUB 15 or 25 front connector. 
 *              For safe operation, the driver stage voltage must be above 12V. 
 *              The 5V output will typically show the controller’s internal regulated 5V minus the drop of a diode that is used for protection and will be in the 4.7V range. 
 *              The battery voltage is monitored for detecting the undervoltage or overvoltage con-ditions.
 * 
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Volts * 10 for internal and battery volts. Milivolts for 5V output
 */
String RoboteqSerial::readVolts(bool *serialTimedOut)
{
    return this->handleQueryRequest(RoboteqCommands::readVoltsQuery, RoboteqCommands::readVoltsRespond, serialTimedOut);
}

/**
 * @description: Read the value of dedicated 32-bit internal variables that can be read and written to/from within a user MicroBasic script. 
 *              It is used to pass 32-bit signed number between user scripts and a microcomputer connected to the controller. 
 *              The total number of user integer variables varies from one controller model to another and can be found in the product datasheet
 * 
 * @params: uint8_t variableNumber: Variable number
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Value
 */
int32_t RoboteqSerial::readUserIntegerVariable(uint8_t variableNumber, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readUserIntegerVariableQuery, variableNumber, RoboteqCommands::readUserIntegerVariableRespond, serialTimedOut);
}

/**
 * @description: This query is only used in AC Induction boards. 
 *              Read the value of the Slip Frequency be-tween the rotor and the stator of an AC Induction motor.
 * 
 * @params: uint8_t channel: Motor channel
 * @params: bool *serialTimedOut: Optional flag that is set to TRUE in case the Serial timed out before a valid response has been received.
 * @return: Slip Frequency in Hertz * 10
 */
int16_t RoboteqSerial::readSlipFrequency(uint8_t channel, bool *serialTimedOut)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readSlipFrequencyQuery, channel, RoboteqCommands::readSlipFrequencyRespond, serialTimedOut);
}

/**
 * @description: Set the rate of speed change during acceleration for a motor channel. This command is identical to the MACC configuration command but is provided so that it can be changed rapidly during motor operation. 
 *              Acceleration value is in 0.1 * RPM per second. When using controllers fitted with encoder, the speed and acceleration value are actual RPMs. 
 *              Brush-less motor controllers use the hall sensor for measuring actual speed and acceleration will also be in actual RPM/s. 
 *              When using the controller without speed sensor, the acceleration value is relative to the Max RPM configuration parameter, which itself is a user-provided number for the speed normally expected at full power. 
 *              Assuming that the Max RPM pa-rameter is set to 1000, and acceleration value of 10000 means that the motor will go from 0 to full speed in exactly 1 second, regardless of the actual motor speed. 
 *              In Closed Loop Torque mode acceleration value is in 0.1 * miliAmps per second. This command is not applicable if either of the acceleration (MAC) or deceleration (MDEC) configuration value is set to 0.
 * 
 * @params: uint8_t channel: Motor channel, int32_t value: Acceleration value in 0.1 * RPM/s
 */
void RoboteqSerial::setAcceleration(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setAccelerationCommand, channel, value);
}

/**
 * @description: This command is used for chaining commands in Position Count mode. It is similar to AC except that it stores an acceleration value in a buffer. 
 *              This value will become the next acceleration the controller will use and becomes active upon reaching a previous desired position. 
 *              If omitted, the command will be chained using the last used acceleration value. This command is not applicable if either of the acceleration (MAC) or deceleration (MDEC) configuration value is set to 0
 * 
 * @params: uint8_t channel: Motor channel, int32_t value: Acceleration value in 0.1 * RPM/s
 */
void RoboteqSerial::nextAcceleration(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::nextAccelerationCommand, channel, value);
}

/**
 * @description: Set the state of user boolean variables inside the controller. These variables can then be read from within a user MicroBasic script to perform specific actions.
 * 
 * @params: uint8_t varNbr: Variable number, bool value: true or false
 */
void RoboteqSerial::setUserBooleanVariable(uint8_t varNbr, bool value)
{
    this->sendMotorCommand(RoboteqCommands::setUserBooleanVariableCommand, varNbr, value);
}

/**
 * @description: When used on controllers with Spektrum RC radio interface the BND command is used to pair the receiver with its transmitter.
 * 
 * @params: uint8_t channel: Motor channel
 */
void RoboteqSerial::spectrumBind(uint8_t channel)
{
    this->sendMotorCommand(RoboteqCommands::multiPurposeBindCommand, channel);
}

/**
 * @description: This command loads the encoder counter for the selected motor channel with the value contained in the command argument. Beware that changing the controller value while op-erating in closed-loop mode can have adverse effects.
 *
 *  @params: uint8_t channel: Motor channel, int32_t value: Counter value
 */
void RoboteqSerial::setEncoderCounters(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setEncoderCountersCommand, channel, value);
}

/**
 * @description: This command loads the brushless counter with the value contained in the command argument. Beware that changing the controller value while operating in closed-loop mode can have adverse effects. 
 * 
 * @params: uint8_t channel: Motor channel, int32_t value: Counter value
 */
void RoboteqSerial::setBrushlessCounter(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setBrushlessCounterCommand, channel, value);
}

/**
 * @description: This command is identical to the G (GO) command except that it is meant to be used for sending motor commands via CANOpen. See the G command for details
 * 
 * @params: uint8_t channel: Motor channel, int32_t value: Command value
 */
void RoboteqSerial::setMotorCommandViaCan(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setMotorCommandViaCanCommand, channel, value);
}

/**
 * @description: This command is used in CAN-enabled controllers to build and send CAN frames in the RawCAN mode (See RawCAN section in manual). 
 *          It can be used to enter the header, bytecount, and data, one element at a time. The frame is sent immediately after the byte-count is entered, and so it should be entered last.
 * 
 * @params: uint8_t element: 1= Header, 2= Bytecount, 3...10= Data0-Data7, uint8_t value: value
 */
void RoboteqSerial::canSend(uint8_t element, uint8_t value)
{
    this->sendMotorCommand(RoboteqCommands::canSendCommand, element, value);
}

/**
 * @description: The D0 command will turn off the single digital output selected by the number that fol-lows. 
 * 
 * @params: uint8_t channel: outputNbr: Output number
 */
void RoboteqSerial::resetIndividualDigitalOutBits(uint8_t outputNbr)
{
    this->sendMotorCommand(RoboteqCommands::resetIndividualDigitalOutBitsCommand, outputNbr);
}

/**
 * @description: The D1 command will activate the single digital output that is selected by the parameter that follows
 * 
 * @params: uint8_t channel: outputNbr: Output number
 */
void RoboteqSerial::setIndividualOutBits(uint8_t outputNbr)
{
    this->sendMotorCommand(RoboteqCommands::setIndividualDigitalOutBitsCommand, outputNbr);
}

/**
 * @description: Set the rate of speed change during decceleration for a motor channel. 
 *              This command is identical to the MDEC configuration command but is provided so that it can be changed rapidly during motor operation. 
 *              Decceleration value is in 0.1 * RPM per second. When using controllers fitted with encoder, the speed and decceleration value are actual RPMs. 
 *              Brushless motor controllers use the hall sensor for measuring actual speed and deccel-eration will also be in actual RPM/s. 
 *              When using the controller without speed sensor, the decceleration value is relative to the Max RPM configuration parameter, 
 *              which itself is a user-provided number for the speed normally expected at full power. 
 *              Assuming that the Max RPM parameter is set to 1000, and decceleration value of 10000 means that the motor will go from full speed to 0 in exactly 1 second, regardless of the actual motor speed.
 *              In Closed Loop Torque mode deceleration value is in 0.1 * miliAmps per second. This command is not applicable if either of the acceleration (MAC) or deceleration (MDEC) configuration value is set to 0.
 * 
 * @params: uint8_t channel: Motor channel, int32_t value: Deceleration value in 0.1 * RPM/s
 */
void RoboteqSerial::setDeceleration(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setDecelerationCommand, channel, value);
}

/**
 * @description: The D command will turn ON or OFF one or many digital outputs at the same time. The number can be a value from 0 to 255 and binary representation of that number has 1bit affected to its respective output pin
 * 
 * @params: uint8_t value: Bit pattern to be applied to all output lines at once
 */
void RoboteqSerial::setAllDigitalOutBits(uint8_t value)
{
    this->sendMotorCommand(RoboteqCommands::setAllDigitalOutBitsCommand, value);
}

/**
 * @description: This command is used for chaining commands in Position Count mode. It is similar to DC except that it stores a decceleration value in a buffer. 
 *              This value will become the next decceleration the controller will use and becomes active upon reaching a previous desired position. If omitted, the command will be chained using the last used decceleration value. 
 *              This command is not applicable if either of the acceleration (MAC) or deceleration (MDEC) configuration values is set to 0 (bypass command ramp). 
 * 
 * @params: uint8_t channel: Motor channel, int32_t value: Acceleration value
 */
void RoboteqSerial::nextDecceleration(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::nextDeccelarationCommand, channel, value);
}

/**
 * @description: This  command  causes any changes to the controller’s configuration to be saved to Flash. Saved configurations are then loaded again next time the controller is powered on. 
 *              This command is  a  duplication  of  the  EESAV  maintenance  command. It is provided as a Real-Time command as well in order to make it possible to save configuration changes from within MicroBasic scripts. 
 */
void RoboteqSerial::saveConfigurationInEeprom()
{
    this->sendMotorCommand(RoboteqCommands::saveConfigurationInEeporomCommand);
}

/**
 * @description: The EX  command  will  cause  the  controller to  enter  an  emergency  stop  in the same way  as  if hardware  emergency  stop  was  detected  on  an  input  pin. 
 *              The  emergency  stop  condition  will remain until controller is reset or until the MG release command is received.
 */
void RoboteqSerial::emergencyStop()
{
    this->sendMotorCommand(RoboteqCommands::emergencyStopCommand);
}

/**
 * @description: G is the main command for activating the motors. 
 *              The command is a number ranging 1000 to +1000 so that the controller respond the same way as when commanded using Analog or Pulse, which are also -1000 to +1000 commands. 
 *              The effect of the command differs from one operating mode to another.
 * 
 * @params: uint8_t channel: Motor channel, int32_t value: Command value
 */
void RoboteqSerial::goToSpeedOrRelativePosition(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::goToSpeedOrToRelativePositionCommand, channel, value);
}

/**
 * @description: This command loads the Home count value into the Encoder, SSI Sensor, or Brushless Counters. 
 *              The Home count can be any user value and is set using the EHOME, SHOME and BHOME configuration parameters. 
 *              When SSI sensors are used as absolute encoders (Absolute Feedback) then this command loads to the Home count value the SSI sensor counter. 
 *              In this case the Home count value is used as offset to the SSI sensor Counter. Beware that loading the counter with the home value while the controller is operating in closed loop can have adverse effects
 * 
 * @params: uint8_t channel: Motor channel
 */
void RoboteqSerial::loadHomeCounter(uint8_t channel)
{
    this->sendMotorCommand(RoboteqCommands::loadHomeCounterCommand, channel);
}

/**
 * @description: The MG command will release the emergency stop condition and allow the controller to return to normal operation. 
 *              Always make sure that the fault condition has been cleared before sending this command
 */
void RoboteqSerial::emergencyStopRelease()
{
    this->sendMotorCommand(RoboteqCommands::emergencyStopReleaseCommand);
}

/**
 * @description: The MS command will stop the motor for the specified motor channel.
 * 
 * @params: uint8_t channel: Motor channel
 */
void RoboteqSerial::stopInAllModes(uint8_t channel)
{
    this->sendMotorCommand(RoboteqCommands::stopInAllModesCommand, channel);
}

/**
 * @description: This command is used in the Position Count mode to make the motor move to a specified feedback sensor count value.
 * 
 * @params: uint8_t channel: Motor channel, int32_t value: Absolute count destination
 */
void RoboteqSerial::goToAbsoluteDesiredPosition(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::goToAbsoluteDesiredPositionCommand, channel, value);
}

/**
 * @description: This command is used in the Position Count mode to make the motor move to a feedback sensor count position that is relative to its current desired position. 
 * 
 * @params: uint8_t channel: Motor channel, int32_t value: Relative count position
 */
void RoboteqSerial::goToRelativeDesiredPosition(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::goToRelativeDesiredPositionCommand, channel, value);
}

/**
 * @description: This command is similar to PR except that it stores a relative count value in a buffer. 
 *              This value becomes active upon reaching a previous desired position and will become the next destination the controller will go to. 
 *              See Position Command Chaining in manual.
 * 
 * @params: uint8_t channel: Motor channel, int32_t value: Relative count position
 */
void RoboteqSerial::nextGoToRelativeDesiredPosition(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::nextGoToRelativeDesiredPositionCommand, channel, value);
}

/**
 * @description: This command is similar to P except that it stores an absolute count value in a buffer. 
 *              This value will become the next destination the controller will go to and becomes active upon reaching a previous desired position.
 *              See Position Command Chaining in manual. 
 * 
 * @params: uint8_t channel: Motor channel, int32_t value: Absolute count position
 */
void RoboteqSerial::nextGoToAbsoluteDesiredPosition(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::nextGoToAbsoluteDesiredPositionCommand, channel, value);
}

/**
 * @description: This command is used to start, stop and restart a MicroBasic script if one is loaded in the controller. 
 * 
 * @params: uint8_t mode: 0= Stop script, 1= Start/resume script, 2: Reinitialize and restart script
 */
void RoboteqSerial::microBasicRun(uint8_t mode)
{
    this->sendMotorCommand(RoboteqCommands::microBasicRunCommand, mode);
}

/**
 * @description: Set the pulse width on products with pulse outputs. 
 *              Command ranges from -1000 to +1000, resulting in pulse width of 1.0ms to 1.5ms respectively. 
 * 
 * @params: uint8_t channel: Motor channel, int16_t value: Value
 */
void RoboteqSerial::setPulseOut(uint8_t channel, int16_t value)
{
    this->sendMotorCommand(RoboteqCommands::setPulseOutCommand, channel, value);
}

/**
 * @description: In the Closed-Loop Speed mode, this command will cause the motor to spin at the de-sired RPM speed. 
 *              In Closed-Loop Position modes, this commands determines the speed at which the motor will move from one position to the next. 
 *              It will not actually start the motion. 
 * 
 * @params: uint8_t channel: Motor channel. int32_t value: Speed value in RPM
 */
void RoboteqSerial::setMotorSpeed(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setMotorSpeedCommand, channel, value);
}

/**
 * @description: This command is used in Position Count mode. It is similar to S except that it stores a ve-locity value in a buffer. 
 *              This value will become the next velocity the controller will use and becomes active upon reaching a previous desired position.
 *              If omitted, the command will be chained using the last used velocity value. 
 *              See Position Command Chaining in manual. 
 * 
 * @params: uint8_t channel: Motor channel, int32_t value: Velocity value
 */
void RoboteqSerial::nextVelocity(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::nextVelocityCommand, channel, value);
}

/**
 * @description: This command is used to set the value of user variables inside the controller. 
 *              These vari-ables can be then read from within a user MicroBasic script to perform specific actions. 
 *              The total number of variables depends on the controller model and can be found in the product datasheet. 
 *              Variables are signed 32-bit integers. 
 * 
 * @params: uint8_t varNbr: Variable number, int32_t value: Value
 */
void RoboteqSerial::setUserVarable(uint8_t varNbr, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setUserVariableCommand, varNbr, value);
}

/**
 * @description: 
 * 
 * @params: const char * commandMessage: Message to be sent on serial
 */
void RoboteqSerial::sendMotorCommand(const char *commandMessage)
{
    String command = commandMessage;
    command += "_";
    this->sendQuery(command.c_str());
}

/**
 * @description:
 * 
 * @params: uint8_t channel: Motor channel
 * @return: 
 */
void RoboteqSerial::sendMotorCommand(const char *commandMessage, uint8_t argument)
{
    String command = commandMessage;
    command += " ";
    command += argument;
    command += "_";
    this->sendQuery(command.c_str());
}

/**
 * @description:
 * 
 * @params: uint8_t channel: Motor channel
 * @return: 
 */
void RoboteqSerial::sendMotorCommand(const char *commandMessage, uint8_t argument, int32_t value)
{
    String command = commandMessage;
    command += " ";
    command += argument;
    command += " ";
    command += value;
    command += "_";
    this->sendQuery(command.c_str());
}

/**
 * @description:
 * 
 * @params: uint8_t channel: Motor channel
 * @return: 
 */
void RoboteqSerial::sendQuery(const char *message)
{
    this->_stream.write(message, strlen(message));
    this->_stream.flush();
}

/**
 * @description:
 * 
 * @params: uint8_t channel: Motor channel
 * @return: 
 */
String RoboteqSerial::readQuery(const char *message, bool *serialTimedOut)
{
    if(serialTimedOut!=NULL){
        *serialTimedOut = false;
    }
    String inputString;
    unsigned long startTime = millis();
    while (millis() - startTime < _timeout)
    {
        if(_stream.available())
        {
            inputString = _stream.readStringUntil('\r');

            if (inputString.startsWith(message))
            {
                return inputString.substring(inputString.indexOf("=") + 1);
            }
        }
    }
    if(serialTimedOut!=NULL){
        *serialTimedOut = true;
    }
    return "-1";
}

/**
 * @description:
 * 
 * @params: uint8_t channel: Motor channel
 * @return: 
 */
String RoboteqSerial::handleQueryRequest(const char *queryMessage, uint8_t extraParameter, const char *respondMessage, bool *serialTimedOut)
{
    String query = queryMessage;
    query += " ";
    query += String(extraParameter);
    query += "_";

    this->sendQuery(query.c_str());
    return this->readQuery(respondMessage, serialTimedOut);
}

/**
 * @description:
 * 
 * @params: const char* queryMessage: Message you want to send to Roboteq
 *          const char* respondMessage: Message you excpect to get back from Roboteq
 * @return: 
 */
String RoboteqSerial::handleQueryRequest(const char *queryMessage, const char *respondMessage, bool *serialTimedOut)
{
    String query = queryMessage;
    query += "_";
    this->sendQuery(query.c_str());
    return this->readQuery(respondMessage, serialTimedOut);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: Motor channel
 * @return: 
 */
int32_t RoboteqSerial::handleQueryRequestToInt(const char *queryMessage, const char *respondMessage, bool *serialTimedOut)
{
    String query = queryMessage;
    query += "_";
    this->sendQuery(query.c_str());
    return this->readQuery(respondMessage, serialTimedOut).toInt();
}

/**
 * @description:
 * 
 * @params: uint8_t channel: Motor channel
 * @return: 
 */
int32_t RoboteqSerial::handleQueryRequestToInt(const char *queryMessage, uint8_t extraParameter, const char *respondMessage, bool *serialTimedOut)
{
    String query = queryMessage;
    query += " ";
    query += String(extraParameter);
    query += "_";

    this->sendQuery(query.c_str());
    return this->readQuery(respondMessage, serialTimedOut).toInt();
}

/**
 * @brief Start a data stream in from the ESC.
 * 
 * @param prefix The data stream prefix.
 * @param delimiter  The data stream delimiter.
 * @param query  The data stream query.
 * @param dataStreamPeriod_ms  The data stream period in milliseconds.
 */
void RoboteqSerial::startDataStream(const char *prefix, const char *delimiter, const char *query, int32_t dataStreamPeriod_ms){
    String streamCmd = "/\"";
    streamCmd += prefix;
    streamCmd += "=\",\"";
    streamCmd += delimiter;
    streamCmd += "\"";
    streamCmd += query;
    streamCmd += "# ";
    streamCmd += dataStreamPeriod_ms;
    streamCmd += "_";

    _stream.write("# C_"); // Stop any streams
    _stream.flush();

    _stream.write(streamCmd.c_str()); // Start the stream
    _stream.flush();
    
    while(_stream.available()){ // Clear Rx buffer
        _stream.read();
    }
}

/**
 * @brief Get data from ESC data stream.
 * 
 * @param prefix The data stream prefix.
 * @param delimiter The data stream delimiter.
 * @param buf The int64 data buffer to write the data to.
 * @param bufLen The length of the data buffer.
 * @return int32_t Number of elements found; -1 if error.
 */
int32_t RoboteqSerial::getDataFromStream(const char *prefix, const char *delimiter, int64_t *buf, size_t bufLen){
    String dataStream;
    while(_stream.available()){
        dataStream += _stream.read();
    }
    return parseDataStream(dataStream, prefix, delimiter, buf, bufLen);
}

/**
 * @brief Split data stream string into its elements and return each element as int64 data.
 * 
 * @param dataStream The data stream string.
 * @param prefix The data stream prefix.
 * @param delimiter The data stream delimiter.
 * @param buf The int64 data buffer to write the data to.
 * @param bufLen The length of the data buffer.
 * @return int32_t Number of elements found; -1 if error.
 */
int32_t RoboteqSerial::parseDataStream(String &dataStream, const char *prefix, const char *delimiter, int64_t *buf, size_t bufLen){
    String tmp;
    tmp = prefix;
    tmp += "=";

    int32_t idxStrtStream = dataStream.indexOf(tmp) + tmp.length()-1;
    if(idxStrtStream == -1){
        return -1;
    }
    int32_t idxEndStream = dataStream.indexOf('\r', idxStrtStream);
    if(idxEndStream == -1){
        return -1;
    }

    String inputString = dataStream.substring(idxStrtStream+1, idxEndStream);
    size_t numOfElementsFound = 0;
    for(int32_t i=0; i<bufLen; i++){
        String dataElement;
        int32_t idxDelimiter = inputString.indexOf(delimiter);
        if(idxDelimiter == -1){ // This is the last element
            dataElement = inputString.substring(0, inputString.length());
            buf[i] = dataElement.toInt();
            numOfElementsFound++;
            break;
        }
        else{
            dataElement = inputString.substring(0, idxDelimiter);
            inputString = inputString.substring(idxDelimiter+1);

            buf[i] = dataElement.toInt();
            numOfElementsFound++;
        }
    }

    return numOfElementsFound;
}
