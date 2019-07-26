#pragma once

#include <Arduino.h>

namespace RoboteqCommands
{

 enum ReadRawSinCosSensorValue : uint8_t {
    SinInput1_SsiInput1 = 1,
    CosInput1,
    SinInput2_SsiInput2,
    CosInput2
};

 enum ReadFieldOrientedControlMotorAmpsValue : uint8_t {
    FluxAmps1 = 1,
    TorqueAmps1,
    FluxAmps2,
    TorqueAmps2
};

/**
 * @note: If only one sensor is enabled, only use variables for input1
 */
 enum ReadMagsensorMarkersValue : uint8_t {
    LeftMarkerOfSensorAtPulseInput1 = 1,
    RightMarkerOfSensorAtPulseInput1,
    LeftMarkerOfSensorAtPulseInput2,
    RightMarkerOfSensorAtPulseInput2
};

/**
 * @note: If only one sensor is enabled, only use variables for input1
 */
enum ReadMagsensorTrackPositionValue : uint8_t {
    LeftTrackOfSensorAtPulseInput1 = 1,
    RightTrackOfSensorAtPulseInput1,
    ActiveTrackOfSensorAtPulseInput1,
    LeftTrackOfSensorAtPulseInput2,
    RightTrackOfSensorAtPulseInput2,
    ActiveTrackOfSensorAtPulseInput2
};

enum ReadTemperatureValue : uint8_t {
    McuTemperature = 1,
    Channel1Side,
    Channel2Side,
};

enum ReadTimeValue : uint8_t {
    Seconds = 1,
    Minutes,
    Hours,
    DayOfMonth,
    Month,
    YearInFull
};

 enum ReadMcuIdValue : uint8_t {
    McuType = 1,
    McuDeviceId,
    McuUniqueId1,
    McuUniqueId2,
    McuUniqueId3,
};

 enum ReadVoltsValue : uint8_t {
    InternalVolts = 1,
    BatteryVolts,
    FiveVoltsOutput, 
};

const char *const readMotorAmpsQuery = "?A";
const char *const readMotorAmpsRespond = "A=";
const char *const readAnalogInputQuery = "?AIC_";
const char *const readAnalogInputRespond = "AIC=";
const char *const readRotorAngleQuery = "?ÀNG_";
const char *const readRotorAngleRespond = "ANG=";
const char *const readRawSinConSensorQuery = "?ASI_";
const char *const readRawSinConSensorRespond = "ASI=";
const char *const readUserBooleanValueQuery = "?B_";
const char *const readUserBooleanValueRespond = "B=";
const char *const readBatteryAmpsQuery = "?BA_";
const char *const readBatteryAmpsRespond = "BA=";
const char *const readBrushlessCountRelativeQuery = "?BCR_";
const char *const readBrushlessCountRelativeRespond = "BCR=";
const char *const readBlMotorSpeedInRpmQuery = "?BS_";
const char *const readBlMotorSpeedInRpmRespond = "BS=";
const char *const readEncoderCounterAbsolutQuery = "?C_";
const char *const readEncoderCounterAbsolutRespond = "C=";
const char *const readAbsolutBrushlessCounterQuery = "?CB_";
const char *const readAbsolutBrushlessCounterRespond = "CB=";
const char *const readRawCanRecivedFramesCountQuery = "?CF_";
const char *const readRawCanRecivedFramesCountRespond = "CF=";
const char *const readConvertedAnalogCommandQuery = "?CIA_";
const char *const readConvertedAnalogCommandRespond = "CIA=";
const char *const readInternalPulseCommandQuery = "?CIP_";
const char *const readInternalPulseCommandRespond = "CIP=";
const char *const readInternalSerialCommandQuery = "?CIS_";
const char *const readInternalSerialCommandRespond = "CIS=";
const char *const readRoboCanAliveNodesMapQuery = "?CL_";
const char *const readRoboCanAliveNodesMapRespond = "CL=";
const char *const readEncoderCountRelativeQuery = "?CR_";
const char *const readEncoderCountRelativeRespond = "CR=";
const char *const readDigitalInputsQuery = "?D_";
const char *const readDigitalInputsRespond = "D=";
const char *const readIndividualDigitalInputsQuery = "?DI_";
const char *const readIndividualDigitalInputsRespond = "DI=";
const char *const readDigitalOutputStatusQuery = "?DO_";
const char *const readDigitalOutputStatusRespond = "DO=";
const char *const readDestinationReachedQuery = "DR_";
const char *const readDestinationReachedRespond = "DR=";
const char *const readClosedLoopErrorQuery = "?E_";
const char *const readClosedLoopErrorRespond = "E=";
const char *const readFeedbackQuery = "?F_";
const char *const readFeedbackRespond = "F=";
const char *const readFocAngleAdjustQuery = "?FC_";
const char *const readFocAngleAdjustRespond = "FC=";
const char *const readFaulFlagQuery = "?FF_";
const char *const readFaultFlagRespond = "FF=";
const char *const readFirmwareIDQuery = "?FID_";
const char *const readFirmwareIDRespond = "FID=";
const char *const readRuntimeStatusFlagQuery = "?FM_";
const char *const readRuntimeStatusFlagRespond = "FM=";
const char *const readStatusFlagQuery = "?FS_";
const char *const readStatusFlagRespond = "FS=";
const char *const readHallSensorStatesQuery = "?HS_";
const char *const readHallSensorStatesRespond = "HS=";
const char *const isRoboCanNodeAliveQuery = "?ICL_";
const char *const isRoboCanNodeAliveRespond = "ICL=";
const char *const readSpektrumReceiverQuery = "?K_";
const char *const readSpektrumReceiverRespond = "K=";
const char *const readLockStatusQuery = "?LK_";
const char *const readLockStatusRespond = "LK=";
const char *const readMotorCommandAppliedQuery = "?M_";
const char *const readMotorCommandAppliedRespond = "M=";
const char *const readFieldOrientedControlMotorAmpsQuery = "?MA_";
const char *const readFieldOrientedControlMotorAmpsRespond = "MA=";
const char *const readMagsensorTrackDetectQuery = "?MGD_";
const char *const readMagsensorTrackDetectRespond = "MGD=";
const char *const readMagsensorMarkersQuery = "?MGM_";
const char *const readMagsensorMarkersRespond = "MGM=";
const char *const readMagsensorStatusQuery = "?MGS_";
const char *const readMagsensorStatusRespond = "MGS=";
const char *const readMagsensorTrackPositionQuery = "?MGT_";
const char *const readMagsensorTrackPositionRespond = "MGT=";
const char *const readMagsensorGyroscopeQuery = "?MGY=";
const char *const readMagsensorGyroscopeRespond = "MGY=";
const char *const readMotorPowerOutputAppliedQuery = "?P_";
const char *const readMotorPowerOutputAppliedRespond = "P=";
const char *const readPulseInputQuery = "?PI_";
const char *const readPulseInputRespond = "PI=";
const char *const readPulseInputAfterConversionQuery = "?PIC_";
const char *const readPulseInputAfterConversionRespond = "PIC=";
const char *const readEncoderMotorSpeedInRpmQuery = "?S_";
const char *const readEncoderMotorSpeedInRpmRespond = "S=";
const char *const readScriptChecksumQuery = "?SCC_";
const char *const readScriptChecksumRespond = "SCC=";
const char *const readEncoderSpeedRelativeQuery = "?SR_";
const char *const readEncoderSpeedRelativeRespond = "SR=";
const char *const readTemperatureQuery = "?T_";
const char *const readTemperatureRespond = "T=";
const char *const readTimeQuery = "?TM_";
const char *const readTimeRespond = "TM=";
const char *const readPositionRelativeTrackingQuery = "?TR_";
const char *const readPositionRelativeTrackingRespond = "TR=";
const char *const readControlUnitTypeAndControllerModelQuery = "?TRN_";
const char *const readControlUnitTypeAndControllerModelRespond = "TRN=";
const char *const readMcuIDQuery = "?UID_";
const char *const readMcuIDRespond = "UID=";
const char *const readVoltsQuery = "?V_";
const char *const readVoltsRespond = "V=";
const char *const readUserIntegerVariableQuery = "?VAR_";
const char *const readUserIntegerVariableRespond = "VAR=";
const char *const readSlipFrequencyQuery = "?SL_";
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
const char *const saveConfigurationInEeporomCommand = "!EES";
const char *const emergencyStopCommand = "!EX";
const char *const goToSpeedOrToRelativePositionCommand = "!G";
const char *const loadHomeCounterCommand = "!H";
const char *const emergencyStopReleaseCommand = "!MG";
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
