/*
  Cmnd.h

   Created on: Feb 7, 2016
       Author: FZZRBN
 */
#ifndef SRC_CMND_H_
#define SRC_CMND_H_

/* Type: RollerState: State of the ball capture rollers. */
typedef enum RollerState
  {
  RollerOff,
  RollerFwd,
  RollerRvrs
  } RollerState;

typedef enum RetrieveArmState
  {
  RetrieveArmStored,
  RetrieveArmLift,
  RetrieveArmExtended,
  RetrieveArmOvrd
  } RetrieveArmState;

typedef enum PneumaticArm
  {
  FullRetracted,
  MovingForward,
  CenterNoCmnd,
  MovingBack,
  FullExtended
  } PneumaticArm;

typedef enum RobotCmndState
  {

  }RobotCmndState;

typedef struct RobotCmnd
  {
    float TankDriveRight;
    float TankDriveLeft;
    float LowerArm;
    float UpperArm;
    RetrieveArmState RetrieveArmDesiredState;
    float RetrieveArmDesiredPos;
    RollerState BallRetrieveRoller;
    PneumaticArm PneumaticArmState;
  } RobotCmnd;

typedef struct RobotSensor
  {
    bool LowArmLim1;
    bool LowArmLim2;
    bool HiArmLim1;
    bool HiArmLim2;
    float RetrieveArmPos;
    bool PneumaticLim1;
    bool PneumaticLim2;
  };

/* Controller mapping */
const int    A_btn = 1;
const int    B_btn = 2;
const int    X_btn = 3;
const int    Y_btn = 4;
const int    LB_btn = 5;
const int    RB_btn = 6;
const int    Back_btn = 7;
const int    Start_btn = 8;
const int    LStick_push = 9;
const int    RStick_push = 10;

/* Loop rate */
const double kUpdatePeriod = 0.005;

  bool GrpOpen, GrpClose, UpSwOn, LoSwOn;
  const int Val_1=1, Val_0=0;
int autonLoop, autonDrive;

#endif /* SRC_CMND_H_ */
