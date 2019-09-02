/**
   Copyright 2019 Niclas Lind

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#pragma once

#include <stdint.h>

namespace RoboteqApi
{

enum ReadRawSinCosSensorValue : uint8_t
{
    SinInput1_SsiInput1 = 1,
    CosInput1,
    SinInput2_SsiInput2,
    CosInput2
};

enum ReadFieldOrientedControlMotorAmpsValue : uint8_t
{
    FluxAmps1 = 1,
    TorqueAmps1,
    FluxAmps2,
    TorqueAmps2
};

/**
 * @note: If only one sensor is enabled, only use variables for input1
 */
enum ReadMagsensorMarkersValue : uint8_t
{
    LeftMarkerOfSensorAtPulseInput1 = 1,
    RightMarkerOfSensorAtPulseInput1,
    LeftMarkerOfSensorAtPulseInput2,
    RightMarkerOfSensorAtPulseInput2
};

/**
 * @note: If only one sensor is enabled, only use variables for input1
 */
enum ReadMagsensorTrackPositionValue : uint8_t
{
    LeftTrackOfSensorAtPulseInput1 = 1,
    RightTrackOfSensorAtPulseInput1,
    ActiveTrackOfSensorAtPulseInput1,
    LeftTrackOfSensorAtPulseInput2,
    RightTrackOfSensorAtPulseInput2,
    ActiveTrackOfSensorAtPulseInput2
};

enum ReadTemperatureValue : uint8_t
{
    McuTemperature = 1,
    Channel1Side,
    Channel2Side,
};

enum ReadTimeValue : uint8_t
{
    Seconds = 1,
    Minutes,
    Hours,
    DayOfMonth,
    Month,
    YearInFull
};

enum ReadMcuIdValue : uint8_t
{
    McuType = 1,
    McuDeviceId,
    McuUniqueId1,
    McuUniqueId2,
    McuUniqueId3,
};

enum ReadVoltsValue : uint8_t
{
    InternalVolts = 1,
    BatteryVolts,
    FiveVoltsOutput,
};

}; // namespace RoboteqApi

namespace RoboteqCommands
{

const char *const readMotorAmpsQuery = "?A";
const char *const readMotorAmpsRespond = "A=";
const char *const readAnalogInputQuery = "?AIC";
const char *const readAnalogInputRespond = "AIC=";
const char *const readRotorAngleQuery = "?ÀNG";
const char *const readRotorAngleRespond = "ANG=";
const char *const readRawSinConSensorQuery = "?ASI";
const char *const readRawSinConSensorRespond = "ASI=";
const char *const readUserBooleanValueQuery = "?B";
const char *const readUserBooleanValueRespond = "B=";
const char *const readBatteryAmpsQuery = "?BA";
const char *const readBatteryAmpsRespond = "BA=";
const char *const readBrushlessCountRelativeQuery = "?BCR";
const char *const readBrushlessCountRelativeRespond = "BCR=";
const char *const readBlMotorSpeedInRpmQuery = "?BS";
const char *const readBlMotorSpeedInRpmRespond = "BS=";
const char *const readEncoderCounterAbsolutQuery = "?C";
const char *const readEncoderCounterAbsolutRespond = "C=";
const char *const readAbsolutBrushlessCounterQuery = "?CB";
const char *const readAbsolutBrushlessCounterRespond = "CB=";
const char *const readRawCanRecivedFramesCountQuery = "?CF";
const char *const readRawCanRecivedFramesCountRespond = "CF=";
const char *const readConvertedAnalogCommandQuery = "?CIA";
const char *const readConvertedAnalogCommandRespond = "CIA=";
const char *const readInternalPulseCommandQuery = "?CIP";
const char *const readInternalPulseCommandRespond = "CIP=";
const char *const readInternalSerialCommandQuery = "?CIS";
const char *const readInternalSerialCommandRespond = "CIS=";
const char *const readRoboCanAliveNodesMapQuery = "?CL";
const char *const readRoboCanAliveNodesMapRespond = "CL=";
const char *const readEncoderCountRelativeQuery = "?CR";
const char *const readEncoderCountRelativeRespond = "CR=";
const char *const readDigitalInputsQuery = "?D";
const char *const readDigitalInputsRespond = "D=";
const char *const readIndividualDigitalInputsQuery = "?DI";
const char *const readIndividualDigitalInputsRespond = "DI=";
const char *const readDigitalOutputStatusQuery = "?DO";
const char *const readDigitalOutputStatusRespond = "DO=";
const char *const readDestinationReachedQuery = "DR";
const char *const readDestinationReachedRespond = "DR=";
const char *const readClosedLoopErrorQuery = "?E";
const char *const readClosedLoopErrorRespond = "E=";
const char *const readFeedbackQuery = "?F";
const char *const readFeedbackRespond = "F=";
const char *const readFocAngleAdjustQuery = "?FC";
const char *const readFocAngleAdjustRespond = "FC=";
const char *const readFaulFlagQuery = "?FF";
const char *const readFaultFlagRespond = "FF=";
const char *const readFirmwareIDQuery = "?FID";
const char *const readFirmwareIDRespond = "FID=";
const char *const readRuntimeStatusFlagQuery = "?FM";
const char *const readRuntimeStatusFlagRespond = "FM=";
const char *const readStatusFlagQuery = "?FS";
const char *const readStatusFlagRespond = "FS=";
const char *const readHallSensorStatesQuery = "?HS";
const char *const readHallSensorStatesRespond = "HS=";
const char *const isRoboCanNodeAliveQuery = "?ICL";
const char *const isRoboCanNodeAliveRespond = "ICL=";
const char *const readSpektrumReceiverQuery = "?K";
const char *const readSpektrumReceiverRespond = "K=";
const char *const readLockStatusQuery = "?LK";
const char *const readLockStatusRespond = "LK=";
const char *const readMotorCommandAppliedQuery = "?M";
const char *const readMotorCommandAppliedRespond = "M=";
const char *const readFieldOrientedControlMotorAmpsQuery = "?MA";
const char *const readFieldOrientedControlMotorAmpsRespond = "MA=";
const char *const readMagsensorTrackDetectQuery = "?MGD";
const char *const readMagsensorTrackDetectRespond = "MGD=";
const char *const readMagsensorMarkersQuery = "?MGM";
const char *const readMagsensorMarkersRespond = "MGM=";
const char *const readMagsensorStatusQuery = "?MGS";
const char *const readMagsensorStatusRespond = "MGS=";
const char *const readMagsensorTrackPositionQuery = "?MGT";
const char *const readMagsensorTrackPositionRespond = "MGT=";
const char *const readMagsensorGyroscopeQuery = "?MGY";
const char *const readMagsensorGyroscopeRespond = "MGY=";
const char *const readMotorPowerOutputAppliedQuery = "?P";
const char *const readMotorPowerOutputAppliedRespond = "P=";
const char *const readPulseInputQuery = "?PI";
const char *const readPulseInputRespond = "PI=";
const char *const readPulseInputAfterConversionQuery = "?PIC";
const char *const readPulseInputAfterConversionRespond = "PIC=";
const char *const readEncoderMotorSpeedInRpmQuery = "?S";
const char *const readEncoderMotorSpeedInRpmRespond = "S=";
const char *const readScriptChecksumQuery = "?SCC";
const char *const readScriptChecksumRespond = "SCC=";
const char *const readEncoderSpeedRelativeQuery = "?SR";
const char *const readEncoderSpeedRelativeRespond = "SR=";
const char *const readTemperatureQuery = "?T";
const char *const readTemperatureRespond = "T=";
const char *const readTimeQuery = "?TM";
const char *const readTimeRespond = "TM=";
const char *const readPositionRelativeTrackingQuery = "?TR";
const char *const readPositionRelativeTrackingRespond = "TR=";
const char *const readControlUnitTypeAndControllerModelQuery = "?TRN";
const char *const readControlUnitTypeAndControllerModelRespond = "TRN=";
const char *const readMcuIDQuery = "?UID";
const char *const readMcuIDRespond = "UID=";
const char *const readVoltsQuery = "?V";
const char *const readVoltsRespond = "V=";
const char *const readUserIntegerVariableQuery = "?VAR";
const char *const readUserIntegerVariableRespond = "VAR=";
const char *const readSlipFrequencyQuery = "?SL";
const char *const readSlipFrequencyRespond = "SL=";
const char *const setAccelerationCommand = "!AC";
const char *const nextAccelerationCommand = "!AX";
const char *const setUserBooleanVariableCommand = "!B";
const char *const multiPurposeBindCommand = "!BND";
const char *const setEncoderCountersCommand = "!C";
const char *const setBrushlessCounterCommand = "!CB";
const char *const setMotorCommandViaCanCommand = "!CG";
const char *const canSendCommand = "!CS";
const char *const resetIndividualDigitalOutBitsCommand = "!D0";
const char *const setIndividualDigitalOutBitsCommand = "!D1";
const char *const setDecelerationCommand = "!DC";
const char *const setAllDigitalOutBitsCommand = "!DS";
const char *const nextDeccelarationCommand = "!DX";
const char *const saveConfigurationInEeporomCommand = "!EES_";
const char *const emergencyStopCommand = "!EX_";
const char *const goToSpeedOrToRelativePositionCommand = "!G";
const char *const loadHomeCounterCommand = "!H";
const char *const emergencyStopReleaseCommand = "!MG_";
const char *const stopInAllModesCommand = "!MS";
const char *const goToAbsoluteDesiredPositionCommand = "!P";
const char *const goToRelativeDesiredPositionCommand = "!PR";
const char *const nextGoToRelativeDesiredPositionCommand = "!PRX";
const char *const nextGoToAbsoluteDesiredPositionCommand = "!PX";
const char *const microBasicRunCommand = "!R";
const char *const setPulseOutCommand = "!RC";
const char *const setMotorSpeedCommand = "!S";
const char *const nextVelocityCommand = "!SX";
const char *const setUserVariableCommand = "!VAR";

} // namespace RoboteqCommands
