#include "RoboteqSerial.hpp"

RoboteqSerial::RoboteqSerial(Stream &stream)
    : _serial(stream)
{
}

/**
 * @description:
 * 
 * @return: fault flags as a byte, each bit is a faultflag
 */
int16_t RoboteqSerial::readFaultFlags()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readFaulFlagQuery, RoboteqCommands::readFaultFlagRespond);
}

/**
 * @description:
 * 
 * @return: voltage of all voltage in Roboteq
 */
int16_t RoboteqSerial::readVoltage()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readVoltsQuery, RoboteqCommands::readVoltsRespond);
}

/**
 * @description: Measures and reports the motor Amps, in Amps*10, for all operating channels.  For brush-less controllers this query reports the RMS value. Note that the current flowing through the motors is often higher than this flowing through the battery
 * 
 * @params: uint8_t channel: Motor Channel
 * @return: current motor amp of channel
 */
int16_t RoboteqSerial::readMotorAmps(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMotorAmpsQuery, channel, RoboteqCommands::readMotorAmpsRespond);
}

/**
 * @description: Returns value of an Analog input after all the adjustments are performed to convert it to a command or feedback value (Min/Max/Center/Deadband/Linearity). If an input is disabled, the query returns 0.  The total number of Analog input channels varies from one controller model to another and can be found in the product datasheet.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readAnalogInput(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readAnalogInputQuery, channel, RoboteqCommands::readAnalogInputRespond);
}

/**
 * @description: On brushless controller operating in sinusoidal mode, this query returns the real time val-ue of the rotor’s electrical angle of brushless motor. This query is useful for verifying trou-bleshooting sin/cos and SPI/SSI sensors. Angle are reported in 0-511 degrees
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint16_t RoboteqSerial::readRotorAngle(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRotorAngleQuery, channel, RoboteqCommands::readRotorAngleRespond);
}

/**
 * @description: Returns real time raw values of ADC connected to sin/cos sensors of each motor or the real time values of the raw data reported by the SSI sensor of the motor. This query is useful for verifying troubleshooting sin/cos sensors and SSI sensors.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint16_t RoboteqSerial::readRawSinCosSensor(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRawSinConSensorQuery, channel, RoboteqCommands::readRotorAngleRespond);
}

/**
 * @description: Read the value of boolean internal variables that can be read and written to/from within a user MicroBasic script. It is used to pass boolean states between user scripts and a microcomputer connected to the controller. The total number of user boolean variables varies from one controller model to another and can be found in the product datasheet
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
bool RoboteqSerial::readUserBooleanValue(uint8_t booleanVariable)
{
    return (bool)this->handleQueryRequestToInt(RoboteqCommands::readUserBooleanValueQuery, booleanVariable, RoboteqCommands::readUserBooleanValueRespond);
}

/**
 * @description: Measures and reports the Amps flowing from the battery in Amps * 10. Battery Amps are often lower than motor Amps
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readBatteryAmps(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readBatteryAmpsQuery, channel, RoboteqCommands::readBatteryAmpsRespond);
}

/**
 * @description: Returns the amount of Internal sensor (Hall, SinCos, Resolver)  counts that have been measured from the last time this query was made. Relative counter read is sometimes easier to work with, compared to full counter reading, as smaller numbers are usually returned.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readBrushlessCountRelative(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readBatteryAmpsQuery, channel, RoboteqCommands::readBatteryAmpsRespond);
}

/**
 * @description: On brushless motor controllers, reports the actual speed measured using the motor’s In-ternal sensors (Hall, SinCos, Resolver) as the actual RPM value. To report RPM accurately, the correct number of motor poles must be loaded in the BLPOL configuration parameter
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readBLMotorSpeedInRpm(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readBlMotorSpeedInRpmQuery, channel, RoboteqCommands::readBlMotorSpeedInRpmRespond);
}

/**
 * @description: Returns the encoder value as an absolute number. The counter is 32-bit with a range of +/- 2147483648 counts
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readEncoderCounterAbsolut(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readEncoderCounterAbsolutQuery, channel, RoboteqCommands::readEncoderCounterAbsolutRespond);
}

/**
 * @description: On brushless motor controllers, returns the running total of Internal sensor (Hall, SinCos, Resolver) transition value as an absolute number. The counter is 32-bit with a range of +/- 2147483648 counts.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readAbsoluteBrushlessCounter(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readAbsolutBrushlessCounterQuery, channel, RoboteqCommands::readAbsolutBrushlessCounterRespond);
}

/**
 * @description: This query is used in CAN-enabled controllers to read the content of a received CAN frame in the RawCAN mode. Data will be available for reading with this query only after a ?CF query is first used to check how many received frames are pending in the FIFO buffer. When the query is sent without arguments, the controller replies by outputting all elements of the frame separated by colons
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readRawCanReceivedFramesCount()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRawCanRecivedFramesCountQuery, RoboteqCommands::readRawCanRecivedFramesCountRespond);
}

/**
 * @description: Returns the motor command value that is computed from the Analog inputs whether or not the command  is  actually  applied  to  the  motor.  The Analog inputs must be con-figured as Motor Command. This  query  can  be  used,  for  example,  to  read  the com-mand  joystick  from within  a  MicroBasic  script  or  from  an  external  microcomputer,  even though  the  controller  may  be  currently  responding  to  Serial or Pulse  command  because  of  a higher priority setting. The returned value is the raw Analog input value with all the adjustments performed to convert it to a command (Min/Max/Center/Deadband/Linearity).
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readConvertedAnalogCommand(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readConvertedAnalogCommandQuery, channel, RoboteqCommands::readConvertedAnalogCommandRespond);
}

/**
 * @description: Returns the motor command value that is computed from the Pulse inputs whether or not the command  is  actually  applied  to  the  motor. The Pulse input must be configured as Motor Command. This  query  can  be  used,  for  example,  to  read  the command  joystick  from  within  a  MicroBasic  script  or  from  an  external  microcomputer,  even though the controller may be currently responding to Seria or Analog command because of a higher priority setting. The returned value is the raw Pulse input value with all the adjust-ments performed to convert it to a command (Min/Max/Center/Deadband/Linearity)
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readInternalPulseCommand(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readInternalPulseCommandQuery, channel, RoboteqCommands::readInternalPulseCommandRespond);
}

/**
 * @description: Returns the motor command value that is issued from the serial input or from a MicroBa-sic script whether  or  not  the  command  is  actually  applied  to  the  motor.  This  query  can  be  used,  for example,  to  read  from  an  external  microcomputer  the  command  generated  inside  MicroBasic script, even though the controller may be currently respond-ing to a Pulse or Analog command because of a higher priority setting.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readInternalSerialCommand(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readInternalSerialCommandQuery, channel, RoboteqCommands::readInternalSerialCommandRespond);
}

/**
 * @description: With CL it is possible to see which nodes in a RoboCAN are alive and what type of device is present at each node. A complete state of the network is represented in sixteen 32-bit numbers. Within each 32-bit word are 8 groups of 4-bits. The 4-bits contain the node information. E.g. bits 0-3 of first number is for node 0, bits 8-11 of first number is for node 2, bits 4-7 of second number is for node 5 and bits 12-15 of fourth number is for node 11, etc.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint32_t RoboteqSerial::readRoboCanAliveNodesMap(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRoboCanAliveNodesMapQuery, channel, RoboteqCommands::readRoboCanAliveNodesMapRespond);
}

/**
 * @description: Returns the amount of counts that have been measured from the last time this query was made. Relative counter read is sometimes easier to work with, compared to full counter reading, as smaller numbers are usually returned.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readEncoderCountRelative(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readEncoderCountRelativeQuery, channel, RoboteqCommands::readEncoderCountRelativeRespond);
}

/**
 * @description: Reports the status of each of the available digital inputs. The query response is a single digital number which must be converted to binary and gives the status of each of the in-puts. The total number of Digital input channels varies from one controller model to anoth-er and can be found in the product datasheet
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint32_t RoboteqSerial::readDigitalInputs()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readDigitalInputsQuery, RoboteqCommands::readDigitalInputsRespond);
}

/**
 * @description: Reports the status of an individual Digital Input. The query response is a boolean value (0 or 1). The total number of Digital input channels varies from one controller model to anoth-er and can be found in the product datasheet.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readIndividualDigitalInputs(uint8_t digitalInputNumber)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readIndividualDigitalInputsQuery, digitalInputNumber, RoboteqCommands::readIndividualDigitalInputsRespond);
}

/**
 * @description: Reads the actual state of all digital outputs. The response to that query is a single number which must be converted into binary in order to read the status of the individual output bits. When querying an individual output, the reply is 0 or 1 depending on its status. The total number of Digital output channels varies from one controller model to another and can be found in the product datasheet.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint16_t RoboteqSerial::readDigitalOutputStatus()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readDigitalOutputStatusQuery, RoboteqCommands::readDigitalOutputStatusRespond);
}

/**
 * @description: This query is used when chaining commands in Position Count mode, to detect that a destination has been reached and that the next destination values that were loaded in the buffer have become active. The Destination Reached bit is latched and is cleared once it has been read.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readDestinationReached(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readDestinationReachedQuery, channel, RoboteqCommands::readDestinationReachedRespond);
}

/**
 * @description: In closed-loop modes, returns the difference between the desired speed or position and the measured feedback. This query can be used to detect when the motor has reached the desired speed or position. In open loop mode, this query returns 0
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readClosedLoopError(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readClosedLoopErrorQuery, channel, RoboteqCommands::readClosedLoopErrorRespond);
}

/**
 * @description: Reports the value of the feedback sensors that are associated to each of the channels in closed-loop modes. The feedback source can be Encoder, Analog or Pulse. Selecting the feedback source is done using the encoder, pulse or analog configuration parameters. This query is useful for verifying that the correct feedback source is used by the channel in the closed-loop mode and that its value is in range with expectations
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readFeedback(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readFeedbackQuery, channel, RoboteqCommands::readFeedbackRespond);
}

/**
 * @description: Read in real time the angle correction that is currently applied by the Field Oriented algo-rithm in order achieve optimal performance
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readFocAngleAdjust(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readFocAngleAdjustQuery, channel, RoboteqCommands::readFocAngleAdjustRespond);
}

/**
 * @description: This query will report a string with the date and identification of the firmware revision of the controller.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
String RoboteqSerial::readFirmwareID()
{
    return this->handleQueryRequest(RoboteqCommands::readFirmwareIDQuery, RoboteqCommands::readFirmwareIDRespond);
}

/**
 *  @description: Report the runtime status of each motor. The response to that query is a single number which must be converted into binary in order to evaluate each of the individual status bits that compose it.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readRuntimeStatusFlag(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readRuntimeStatusFlagQuery, channel, RoboteqCommands::readRuntimeStatusFlagRespond);
}

/**
 *  @description: Report the state of status flags used by the controller to indicate a number of internal conditions during normal operation. The response to this query is the single number for all status flags. The status of individual flags is read by converting this number to binary and look at various bits of that number
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readStatusFlag()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readStatusFlagQuery, RoboteqCommands::readStatusFlagRespond);
}

/**
 *  @description: Reports that status of the hall sensor inputs. This function is mostly useful for trouble-shooting. When no sensors are connected, all inputs are pulled high and the value 7 will be replied. For 60 degrees spaced Hall sensors, 0-1- 3-4- 6-7 are valid combinations, while 2 and 5 are invalid combinations. For 120 degrees spaced sensors, 1-2- 3-4- 5-6 are valid combinations, while 0 and 7 are invalid combinations. In normal conditions, valid values should appear at one time or the other as the motor shaft is rotated
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readHallSensorStates(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readHallSensorStatesQuery, channel, RoboteqCommands::readHallSensorStatesRespond);
}

/**
 * @description: This query is used to determine if specific RoboCAN node is alive on CAN bus.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::isRoboCanNodeAlive(uint8_t nodeID)
{
    return this->handleQueryRequestToInt(RoboteqCommands::isRoboCanNodeAliveQuery, nodeID, RoboteqCommands::isRoboCanNodeAliveRespond);
}

/**
 * @description: On controller models with Spektrum radio support, this query is used to read the raw values of each of up to 6 receive channels. When signal is received, this query returns the value 0.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint16_t RoboteqSerial::readSpektrumReceiver(uint8_t radioChannel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readSpektrumReceiverQuery, radioChannel, RoboteqCommands::readSpektrumReceiverRespond);
}

/**
 * @description:Returns the status of the lock flag. If the configuration is locked, then it will not be possi-ble to read any configuration parameters until the lock is removed or until the parameters are reset to factory default. This feature is useful to protect the controller configuration from being copied by unauthorized people.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readLockStatus()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readLockStatusQuery, RoboteqCommands::readLockStatusRespond);
}

/**
 * @description: Reports the command value that is being used by the controller. The number that is reported will be depending on which mode is selected at the time. The choice of one command mode vs. another is based on the command priority mechanism.  In the Serial mode, the reported value will be the command that is entered in via the RS232, RS485, TCP or USB port and to which an optional exponential correction is applied.  In the Analog and Pulse modes, this query will report the Analog or Pulse input after it is being convert-ed using the min, max, center, deadband, and linearity corrections.  This query is useful for viewing which command is actually being used and the effect of the correction that is being applied to the raw input.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readMotorCommandApplied(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMotorCommandAppliedQuery, channel, RoboteqCommands::readMotorCommandAppliedRespond);
}

/**
 * @description: On brushless motor controllers operating in sinusoidal mode, this query returns the Torque (also known as Quadrature or Iq) current, and the Flux (also known as Direct, or Id) current. Current is reported in Amps x 10.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readFieldOrientedControlMotorAmps(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readFieldOrientedControlMotorAmpsQuery, channel, RoboteqCommands::readFieldOrientedControlMotorAmpsRespond);
}

/**
 * @description: When one or more MGS1600 Magnetic Guide Sensors are connected to the controller, this query reports whether a magnetic tape is within the detection range of the sensor. If no tape is detected, the output will be 0. If only one sensor is connected to any pulse in-put, no argument is needed for this query. If more than one sensor is connected to pulse inputs and these inputs are enabled and configured in Magsensor MultiPWM mode, then the argument following the query is used to select the sensor
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readMagsensorTrackDetect(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorTrackDetectQuery, channel, RoboteqCommands::readMagsensorTrackDetectRespond);
}

/**
 * @description: When one or more MGS1600 Magnetic Guide Sensors are connected to the controller, this query reports whether left or right markers are present under sensor.If only one sen-sor is connected to any pulse input this query will report the data of that sensor, regard-less which pulse input it is connected to. If more than one sensor is connected to pulse inputs and these inputs are enabled and configured in Magsensor MultiPWM mode, then the argument following the query is used to select the sensor
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint8_t RoboteqSerial::readMagsensorMarkers(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorMarkersQuery, channel, RoboteqCommands::readMagsensorMarkersRespond);
}

/**
 * @description: When one or more MGS1600 Magnetic Guide Sensors are connected to the controller, this query reports the state of the sensor. If only one sensor is connected to any pulse in-put, no argument is needed for this query. If more than one sensor is connected to pulse inputs and these inputs are enabled and configured in Magsensor MultiPWM mode, then the argument following the query is used to select the sensor
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readMagsensorStatus()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorStatusQuery, RoboteqCommands::readMagsensorStatusRespond);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readMagsensorTrackPosition(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorTrackPositionQuery, channel, RoboteqCommands::readMagsensorTrackPositionRespond);
}

/**
 * @description: When one or more MGS1600 Magnetic Guide Sensors are connected to the controller, this query reports the position of the tracks detected under the sensor. If only one sensor is connected to any pulse input, the argument following the query selects which track to read. If more than one sensor is connected to pulse inputs and these inputs are enabled and configured in Magsensor MultiPWM mode, then the argument following the query is used to select the sensor. The reported position of the magnetic track in millimeters, us-ing the center of the sensor as the 0 reference
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readMagsensorGyroscope(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMagsensorGyroscopeQuery, channel, RoboteqCommands::readMagsensorGyroscopeRespond);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readMotorPowerOutputApplied(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readMotorPowerOutputAppliedQuery, channel, RoboteqCommands::readMotorPowerOutputAppliedRespond);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint16_t RoboteqSerial::readPulseInput(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readPulseInputQuery, channel, RoboteqCommands::readPulseInputRespond);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readPulseInputAfterConversion(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readPulseInputAfterConversionQuery, channel, RoboteqCommands::readPulseInputAfterConversionRespond);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readEncoderMotorSpeedInRpm(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readEncoderMotorSpeedInRpmQuery, channel, RoboteqCommands::readEncoderMotorSpeedInRpmRespond);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint32_t RoboteqSerial::readScriptChecksum()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readScriptChecksumQuery, RoboteqCommands::readScriptChecksumRespond);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int16_t RoboteqSerial::readEncoderSpeedRelative(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readEncoderSpeedRelativeQuery, channel, RoboteqCommands::readEncoderSpeedRelativeRespond);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int8_t RoboteqSerial::readTemperature(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readTemperatureQuery, channel, RoboteqCommands::readTemperatureRespond);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint32_t RoboteqSerial::readTime(uint8_t dataElementInNewControllerModel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readTimeQuery, RoboteqCommands::readTimeRespond);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int32_t RoboteqSerial::readPositionRelativeTracking(uint8_t channel)
{
    return this->handleQueryRequestToInt(RoboteqCommands::readPositionRelativeTrackingQuery, channel, RoboteqCommands::readPositionRelativeTrackingRespond);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
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

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
uint16_t RoboteqSerial::readVolts()
{
    return this->handleQueryRequestToInt(RoboteqCommands::readVoltsQuery, RoboteqCommands::readVoltsRespond);
}

/**
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
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

/**
 * @description: Set the rate of speed change during acceleration for a motor channel. This command is identical to the MACC configuration command but is provided so that it can be changed rapidly during motor operation. 
 * Acceleration value is in 0.1 * RPM per second. When using controllers fitted with encoder, the speed and acceleration value are actual RPMs. 
 * Brush-less motor controllers use the hall sensor for measuring actual speed and acceleration will also be in actual RPM/s. 
 * When using the controller without speed sensor, the acceleration value is relative to the Max RPM configuration parameter, which itself is a user-provided number for the speed normally expected at full power. 
 * Assuming that the Max RPM pa-rameter is set to 1000, and acceleration value of 10000 means that the motor will go from 0 to full speed in exactly 1 second, regardless of the actual motor speed. 
 * In Closed Loop Torque mode acceleration value is in 0.1 * miliAmps per second. This command is not applicable if either of the acceleration (MAC) or deceleration (MDEC) configuration value is set to 0.
 * 
 * @params: uint8_t channel: MotorChannel
 */
void RoboteqSerial::setAcceleration(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setAccelerationCommand, channel, value);
}

/**
 * @description: This command is used for chaining commands in Position Count mode. It is similar to AC except that it stores an acceleration value in a buffer. This value will become the next acceleration the controller will use and becomes active upon reaching a previous desired position. If omitted, the command will be chained using the last used acceleration value. This command is not applicable if either of the acceleration (MAC) or deceleration (MDEC) configuration value is set to 0
 * 
 * @params: uint8_t channel: MotorChannel
 */
void RoboteqSerial::nextAcceleration(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::nextAccelerationCommand, channel, value);
}

/**
 * @description: Set the state of user boolean variables inside the controller. These variables can then be read from within a user MicroBasic script to perform specific actions.
 * 
 * @params: uint8_t channel: MotorChannel
 */
void RoboteqSerial::setUserBooleanVariable(uint8_t varNbr, bool value)
{
    this->sendMotorCommand(RoboteqCommands::setUserBooleanVariableCommand, varNbr, value);
}

/**
 * @description: When used on controllers with Spektrum RC radio interface the BND command is used to pair the receiver with its transmitter.
 * 
 * @params: uint8_t channel: MotorChannel
 */
void RoboteqSerial::multiPurposeBind(uint8_t channel)
{
    this->sendMotorCommand(RoboteqCommands::multiPurposeBindCommand, channel);
}

/**
 * @description: This command loads the encoder counter for the selected motor channel with the value contained in the command argument. Beware that changing the controller value while op-erating in closed-loop mode can have adverse effects.
 *
 *  @params: uint8_t channel: MotorChannel
 */
void RoboteqSerial::setEncoderCounters(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setEncoderCountersCommand, channel, value);
}

/**
 * @description: This command loads the brushless counter with the value contained in the command argument. Beware that changing the controller value while operating in closed-loop mode can have adverse effects. 
 * 
 * @params: uint8_t channel: MotorChannel
 */
void RoboteqSerial::setBrushlessCounter(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setBrushlessCounterCommand, channel, value);
}

/**
 * @description: This command is identical to the G (GO) command except that it is meant to be used for sending motor commands via CANOpen. See the G command for details
 * 
 * @params: uint8_t channel: MotorChannel
 */
void RoboteqSerial::setMotorCommandViaCan(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setMotorCommandViaCanCommand, channel, value);
}

/**
 * @description: This command is used in CAN-enabled controllers to build and send CAN frames in the RawCAN mode (See RawCAN section in manual). It can be used to enter the header, bytecount, and data, one element at a time. The frame is sent immediately after the byte-count is entered, and so it should be entered last.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::canSend(uint8_t element, uint8_t value)
{
    this->sendMotorCommand(RoboteqCommands::canSendCommand, element, value);
}

/**
 * @description: The D0 command will turn off the single digital output selected by the number that fol-lows. 
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::resetIndividualDigitalOutBits(uint8_t outputNbr)
{
    this->sendMotorCommand(RoboteqCommands::resetIndividualDigitalOutBitsCommand, outputNbr);
}

/**
 * @description: The D1 command will activate the single digital output that is selected by the parameter that follows
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::setIndividualOutBits(uint8_t outputNbr)
{
    this->sendMotorCommand(RoboteqCommands::setIndividualDigitalOutBitsCommand, outputNbr);
}

/**
 * @description: Set the rate of speed change during decceleration for a motor channel. This command is identical to the MDEC configuration command but is provided so that it can be changed rapidly during motor operation. Decceleration value is in 0.1 * RPM per second. When using controllers fitted with encoder, the speed and decceleration value are actual RPMs. Brushless motor controllers use the hall sensor for measuring actual speed and deccel-eration will also be in actual RPM/s. When using the controller without speed sensor, the decceleration value is relative to the Max RPM configuration parameter, which itself is a user-provided number for the speed normally expected at full power. Assuming that the Max RPM parameter is set to 1000, and decceleration value of 10000 means that the motor will go from full speed to 0 in exactly 1 second, regardless of the actual motor speed. In Closed Loop Torque mode deceleration value is in 0.1 * miliAmps per second. This command is not applicable if either of the acceleration (MAC) or deceleration (MDEC) configuration value is set to 0.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::setDeceleration(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setDecelerationCommand, channel, value);
}

/**
 * @description: The D command will turn ON or OFF one or many digital outputs at the same time. The number can be a value from 0 to 255 and binary representation of that number has 1bit affected to its respective output pin
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::setAllDigitalOutBits(uint8_t value)
{
    this->sendMotorCommand(RoboteqCommands::setAllDigitalOutBitsCommand, value);
}

/**
 * @description: This command is used for chaining commands in Position Count mode. It is similar to DC except that it stores a decceleration value in a buffer. This value will become the next decceleration the controller will use and becomes active upon reaching a previous desired position. If omitted, the command will be chained using the last used decceleration value.This command is not applicable if either of the acceleration (MAC) or deceleration (MDEC) configuration values is set to 0 (bypass command ramp). 
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::nextDecceleration(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::nextDeccelarationCommand, channel, value);
}

/**
 * @description: This  command  causes any changes to the controller’s configuration to be saved to Flash. Saved configurations are then loaded again next time the controller is powered on. This command is  a  duplication  of  the  EESAV  maintenance  command. It is provided as a Real-Time command as well in order to make it possible to save configuration changes from within MicroBasic scripts. 
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::saveConfigurationInEeprom()
{
    this->sendMotorCommand(RoboteqCommands::saveConfigurationInEeporomCommand);
}

/**
 * @description: The EX  command  will  cause  the  controller to  enter  an  emergency  stop  in the same way  as  if hardware  emergency  stop  was  detected  on  an  input  pin.  The  emergency  stop  condition  will remain until controller is reset or until the MG release command is received.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::emergencyStop()
{
    this->sendMotorCommand(RoboteqCommands::emergencyStopCommand);
}

/**
 * @description: G is the main command for activating the motors. The command is a number ranging 1000 to +1000 so that the controller respond the same way as when commanded using Analog or Pulse, which are also -1000 to +1000 commands. The effect of the command differs from one operating mode to another.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::goToSpeedOrRelativePosition(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::goToSpeedOrToRelativePositionCommand, channel, value);
}

/**
 * @description: This command loads the Home count value into the Encoder, SSI Sensor, or Brushless Counters. The Home count can be any user value and is set using the EHOME, SHOME and BHOME configuration parameters. When SSI sensors are used as absolute encoders (Absolute Feedback) then this command loads to the Home count value the SSI sensor counter. In this case the Home count value is used as offset to the SSI sensor Counter. Beware that loading the counter with the home value while the controller is operating in closed loop can have adverse effects
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::loadHomeCounter(uint8_t channel)
{
    this->sendMotorCommand(RoboteqCommands::loadHomeCounterCommand, channel);
}

/**
 * @description: The MG command will release the emergency stop condition and allow the controller to return to normal operation. Always make sure that the fault condition has been cleared before sending this command
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::emergencyStopRelease()
{
    this->sendMotorCommand(RoboteqCommands::emergencyStopReleaseCommand);
}

/**
 * @description: The MS command will stop the motor for the specified motor channel.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::stopInAllModes(uint8_t channel)
{
    this->sendMotorCommand(RoboteqCommands::stopInAllModesCommand, channel);
}

/**
 * @description: This command is used in the Position Count mode to make the motor move to a specified feedback sensor count value.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::goToAbsoluteDesiredPosition(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::goToAbsoluteDesiredPositionCommand, channel, value);
}

/**
 * @description: This command is used in the Position Count mode to make the motor move to a feedback sensor count position that is relative to its current desired position. 
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::goToRelativeDesiredPosition(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::goToRelativeDesiredPositionCommand, channel, value);
}

/**
 * @description: This command is similar to PR except that it stores a relative count value in a buffer. This value becomes active upon reaching a previous desired position and will become the next destination the controller will go to. See Position Command Chaining in manual.
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::nextGoToRelativeDesiredPosition(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::nextGoToRelativeDesiredPositionCommand, channel, value);
}

/**
 * @description: This command is similar to P except that it stores an absolute count value in a buffer. This value will become the next destination the controller will go to and becomes active upon reaching a previous desired position. See Position Command Chaining in manual. 
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::nextGoToAbsoluteDesiredPosition(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::nextGoToAbsoluteDesiredPositionCommand, channel, value);
}

/**
 * @description: This command is used to start, stop and restart a MicroBasic script if one is loaded in the controller. 
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::microBasicRun(uint8_t mode)
{
    this->sendMotorCommand(RoboteqCommands::microBasicRunCommand, mode);
}

/**
 * @description: Set the pulse width on products with pulse outputs. Command ranges from -1000 to +1000, resulting in pulse width of 1.0ms to 1.5ms respectively. 
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::setPulseOut(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setPulseOutCommand, channel, value);
}

/**
 * @description: In the Closed-Loop Speed mode, this command will cause the motor to spin at the de-sired RPM speed. In Closed-Loop Position modes, this commands determines the speed at which the motor will move from one position to the next. It will not actually start the motion. 
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::setMotorSpeed(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setMotorSpeedCommand, channel, value);
}

/**
 * @description: This command is used in Position Count mode. It is similar to S except that it stores a ve-locity value in a buffer. This value will become the next velocity the controller will use and becomes active upon reaching a previous desired position. If omitted, the command will be chained using the last used velocity value. See Position Command Chaining in manual. 
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::nextVelocity(uint8_t channel, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::nextVelocityCommand, channel, value);
}

/**
 * @description: This command is used to set the value of user variables inside the controller. These vari-ables can be then read from within a user MicroBasic script to perform specific actions. The total number of variables depends on the controller model and can be found in the product datasheet. Variables are signed 32-bit integers. 
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::setUserVarable(uint8_t varNbr, int32_t value)
{
    this->sendMotorCommand(RoboteqCommands::setUserVariableCommand, varNbr, value);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::sendMotorCommand(const char *commandMessage)
{
    this->sendQuery(commandMessage);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::sendMotorCommand(const char *commandMessage, uint8_t channel)
{
    String command = commandMessage;
    command += channel;
    command += "_";
    this->sendQuery(command.c_str());
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::sendMotorCommand(const char *commandMessage, uint8_t channel, int32_t value)
{
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
void RoboteqSerial::sendQuery(const char *message)
{
    this->_serial.write(message, strlen(message));
    this->_serial.flush();
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
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

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
String RoboteqSerial::handleQueryRequest(const char *queryMessage, uint8_t extraParameter, const char *respondMessage)
{
    String query = queryMessage;
    query += String(extraParameter);
    query += "_";

    this->sendQuery(query.c_str());
    return this->readQuery(respondMessage);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
String RoboteqSerial::handleQueryRequest(const char *queryMessage, const char *respondMessage)
{
    this->sendQuery(queryMessage);
    return this->readQuery(respondMessage);
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int RoboteqSerial::handleQueryRequestToInt(const char *queryMessage, const char *respondMessage)
{
    this->sendQuery(queryMessage);
    return this->readQuery(respondMessage).toInt();
}

/**
 * @description:
 * 
 * @params: uint8_t channel: MotorChannel
 * @return: 
 */
int RoboteqSerial::handleQueryRequestToInt(const char *queryMessage, uint8_t extraParameter, const char *respondMessage)
{
    String query = queryMessage;
    query += String(extraParameter);
    query += "_";

    this->sendQuery(query.c_str());
    return this->readQuery(respondMessage).toInt();
}
