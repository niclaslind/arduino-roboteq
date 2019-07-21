#pragma once

#include <Arduino.h>

namespace RoboteqCommands
{

const char *readMotorAmpsQuery = "?A_";
const char *readMotorAmpsRespond = "A=";
const char *readAnalogInputQuery = "?AIC_";
const char *readAnalogInputRespond = "AIC=";
const char *readRotorAngleQuery = "?ÀNG_";
const char *readRotorAngleRespond = "ANG=";
const char *readRawSinConSensor = "?ASI_";
const char *readUserBooleanValueQuery = "?B_";
const char *readBatteryAmpsQuery = "?BA_";
const char *readBatteryAmpsRespond = "BA=";
const char *readBrushlessCountRelativeQuery = "?BCR_";
const char *readBlMotorSpeedInRpmQuery = "?BS_";
const char *readBlMotorSpeedInRpmRespond = "BS=";
const char *readEncoderCounterAbsolutQuery = "?C_";
const char *readEncoderCounterAbsolutRespond = "C=";
const char *readAbsolutBrushlessCounterQuery = "?CB_";
const char *readAbsolutBrushlessCounterRespond = "CB=";
const char *readRawCanRecivedFramesCountQuery = "?CF_";
const char *readRawCanRecivedFramesCountRespond = "CF=";
const char *readConvertedAnalogCommandQuery = "?CIA_";
const char *readConvertedAnalogCommandRespond = "CIA=";
const char *readInternalPulseCommandQuery = "?CIP_";
const char *readInternalPulseCommandRespond = "CIP=";
const char *readInternalSerialCommandQuery = "?CIS_";
const char *readInternalSerialCommandRespond = "CIS=";
const char *readRoboCanAliveNodesMapQuery = "?CL_";
const char *readRoboCanAliveNodesMapRespond = "CL=";
const char *readEncoderCountRelativeQuery = "?CR_";
const char *readEncoderCountRelativeRespond = "CR=";
const char *readDigitalInputsQuery = "?D_";
const char *readDigitalInputsRespond = "D=";
const char *readIndividualDigitalInputsQuery = "?DI_";
const char *readIndividualDigitalInputsRespond = "DI=";
const char *readDigitalOutputStatusQuery = "?DO_";
const char *readDigitalOutputStatusRespond = "DO=";
const char *readDestinationReachedQuery = "DR_";
const char *readDestinationReachedRespond = "DR=";
const char *readClosedLoopErrorQuery = "?E_";
const char *readClosedLoopErrorRespond = "E=";
const char *readFeedbackQuery = "?F_";
const char *readFeedbackRespond = "F=";
const char *readFocAngleAdjustQuery = "?FC_";
const char *readFocAngleAdjustRespond = "FC=";
const char *readFaulFlagQuery = "?FF_";
const char *readFaultFlagRespond = "FF=";
const char *readFirmwareIDQuery = "?FID_";
const char *readFirmwareIDRespond = "FID=";
const char *readRuntimeStatusFlagQuery = "?FM_";
const char *readRuntimeStatusFlagRespond = "FM=";
const char *readStatusFlagQuery = "?FS_";
const char *readHallSensorStatesQuery = "?HS_";
const char *readHallSensorStatesRespond = "HS=";
const char *isRoboCanNodeAliveQuery = "?ICL_";
const char *isRoboCanNodeAliveRespond = "ICL=";
const char *readSpektrumReceiverQuery = "?K_";
const char *readSpektrumReceiverRespond = "K=";
const char *readLockStatusQuery = "?LK_";
const char *readLockStatusRespond = "LK=";

const char *readVoltsQuery = "?V_";
const char *readVoltsRespond = "V=";

} // namespace RoboteqCommands
