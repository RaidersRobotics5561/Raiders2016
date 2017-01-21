#include "WPILib.h"
#include "Commands/Command.h"
#include "Commands/ExampleCommand.h"
#include "CommandBase.h"
#include "SolenoidBase.h"
#include "DoubleSolenoid.h"
#include "Joystick.h"
#include "Encoder.h"
#include "Talon.h"
#include "ADXRS450_Gyro.h"
//#include "USBCamera.h"
#include "CameraServer.h"
#include "DigitalOutput.h"
#include "Timer.h"
#include "DriverStation.h"

class Robot: public IterativeRobot {
    Joystick V_XboxDrive; //joystick
    Joystick V_Logitech;
    RobotDrive V_MyDrive; // robot drive
    Talon V_LowArm;
    Victor V_FishTape;
    Talon V_Winch;
    Talon V_BallRotate;
    Talon V_BallRoller;
    LiveWindow *V_LW;
    DigitalInput V_BlSw;
    DigitalInput V_Sw1, V_Sw2, V_Sw3, V_Sw4;
    DigitalOutput V_Light1;
    DigitalOutput V_Light2;
    Encoder V_Ballarm;
    ADXRS450_Gyro V_GyroOne;

    const double C_UpdatePeriod = 0.005;
    int V_AutonState;
    int V_USB_Cam = 0;

    /* Controller button s */
    const int C_XB_A_Btn = 1;
    const int C_XB_B_Btn = 2;
    const int C_XB_X_Btn = 3;
    const int C_XB_Y_Btn = 4;
    const int C_XB_L_Bumper = 5;
    const int C_XB_R_Bumper = 6;
    const int C_XB_BackBtn = 7;
    const int C_XB_StartBtn = 8;
    const int C_XB_L_StickPush = 9;
    const int C_XB_R_StickPush = 10;

    /* Motor gains */
    const float K_WenchFwd = 0.4;
    const float K_WenchRvrs = -0.4;
    const float K_BallArmUp = 0.4;
    const float K_BallArmDwn = -0.4;
    const float K_WinchSpd = 0.4;
    const float K_LowArmGx = 0.6;
    const float K_RotationGx = 0.6;
    const float K_RotationMin = 0.5;
    const float K_RotationMax = 0.8;
    const float K_FishTapeDown = 1.0;
    const float K_FishTapeUp = -1.0;
    const float K_FishTapeStop = 0;
    const float K_BallRollerIn = 1.0;
    const float K_BallRollerOut = -1.0;
    const float K_BallRollerStop = 0;





public:
    Robot() :
      V_XboxDrive(0),// Initialize Joystick on port 0.
      V_Logitech(1),
      V_LowArm(4),
      V_FishTape(7),
      V_BallRotate(6), // Port 2 is dead
      V_BallRoller(3),
      V_Winch(5),
      V_MyDrive(0, 1), // Initialize the Victor on channel 2.
      V_LW(NULL),
      V_BlSw(9), // get DI value from input pos 1
      V_Ballarm(0,1,true,Encoder::EncodingType::k4X),
      V_Sw1(5),
      V_Sw2(6),
      V_Sw3(7),
      V_GyroOne(SPI::Port::kOnboardCS0),
      V_Sw4(8),
	  V_Light1(4),
	  V_Light2(3)
    {
    V_MyDrive.SetExpiration(0.1);
    }

private:
      BuiltInAccelerometer V_Accel;

 /******************************************************************************
  * Function:     RobotInit
  *
  * Description:  Run the functions that are expected during the robot init.
  ******************************************************************************/

  void RobotInit ()
  {
  V_LW = LiveWindow::GetInstance();

 // CameraServer::GetInstance()->SetQuality(50);
   //the camera name (ex "cam0") can be found through the roborio web interface
  //CameraServer::GetInstance()->StartAutomaticCapture(V_USB_Cam);

  V_AutonState = 0;
  V_Ballarm.Reset();
  V_Ballarm.SetMaxPeriod(.01);
  V_Ballarm.SetMinRate(.02);
  V_Ballarm.SetDistancePerPulse(.9);
  V_GyroOne.Calibrate();
  UpdateActuatorCmnds(0,0,false,false,false,false,false,false,false,0,0,0,0,0);

  UpdateSmartDashboad(false,
                      false,
                      false,
                      false,
                      false,
                      0,
                      0,
                      0,
                      0,
                      0,
                      0,
                      0,
                      0,
                      0);
  }


/******************************************************************************
 * Function:     DisabledPeriodic
 *
 * Description:  Run the functions that are expected during the period when the
 *               robot is disabled.
 ******************************************************************************/
void DisabledPeriodic()
{
  Scheduler::GetInstance()->Run();

  V_Light1.Set(1);
  V_Light2.Set(1);


  while(IsDisabled())
  {
    UpdateActuatorCmnds(0,0,false,false,false,false,false,false,false,0,0,0,0,0);

    ReadAutonSwitch();

    UpdateSmartDashboad(V_Sw1.Get(),
                        V_Sw2.Get(),
                        V_Sw3.Get(),
                        V_Sw4.Get(),
                        V_BlSw.Get(),
                        (double)V_AutonState,
                        (double)0,
                        V_Ballarm.GetDistance(),
                        V_GyroOne.GetAngle(),
                        V_Accel.GetX(),
                        V_Accel.GetY(),
                        V_Accel.GetZ(),
                        (double)0,
                        (double)0);
  wait(C_UpdatePeriod);
  }
}


/******************************************************************************
 * Function:     ReadAutonSwitch
 *
 * Description:  Read the switches on the robot that determine desired auton state.
 ******************************************************************************/
void ReadAutonSwitch()
  {
    /* This function reads the 4 switches and determines what the desired auton
     * state is.  This function also reports out what the switch state is along
     * with the auton state. */

    /* Read the 4 switches here: */
    if ((V_Sw1.Get() == false) && (V_Sw2.Get() == true) && (V_Sw3.Get() == true) && (V_Sw4.Get() == true))
      {
      /* Moat and rock wall */
      V_AutonState = 1;
      }
    else if ((V_Sw1.Get() == true) && (V_Sw2.Get() == false) && (V_Sw3.Get() == true) && (V_Sw4.Get() == true))
      {
      /* Uneven ramps */
      V_AutonState = 2;
      }
    else if ((V_Sw1.Get() == false) && (V_Sw2.Get() == false) && (V_Sw3.Get() == true) && (V_Sw4.Get() == true))
      {
      /* Low bar and rough terrain */
      V_AutonState = 3;
      }
    else if ((V_Sw1.Get() == true) && (V_Sw2.Get() == true) && (V_Sw3.Get() == false) && (V_Sw4.Get() == true))
      {
      /* Low power */
      V_AutonState = 4;
      }
    /*else if ((V_Sw1.Get() == false) && (V_Sw2.Get() == true) && (V_Sw3.Get() == false) && (V_Sw4.Get() == true))
      {
      /* Low bar, forward and then reverse
      V_AutonState = 5;
      }*/
    else if ((V_Sw1.Get() == true) && (V_Sw2.Get() == false) && (V_Sw3.Get() == false) && (V_Sw4.Get() == true))
      {
      /* Low bar, forward and then turn */
      V_AutonState = 6;
      }
    else if ((V_Sw1.Get() == false) && (V_Sw2.Get() == false) && (V_Sw3.Get() == false) && (V_Sw4.Get() == true))
      {
      /* Low bar, forward and then turn */
      V_AutonState = 7;
      }
    else if ((V_Sw1.Get() == true) && (V_Sw2.Get() == true) && (V_Sw3.Get() == true) && (V_Sw4.Get() == false))
    {
    	V_AutonState = 8;
    }
    else if ((V_Sw1.Get() == false) && (V_Sw2.Get() == true) && (V_Sw3.Get() == true) && (V_Sw4.Get() == false))
    {
    	V_AutonState = 9;
    }
    else
      {
      /* Default, no auton */
      V_AutonState = 0;
      }
  }

/******************************************************************************
 * Function:     UpdateActuatorCmnds
 *
 * Description:  Update the commands sent out to the RoboRIO.
 ******************************************************************************/
void UpdateActuatorCmnds(float L_BallRoller,
                         float L_DesiredPos,
                         bool L_LgtB3,
                         bool L_LgtB5,
                         bool L_LgtB6,
                         bool L_LgtB7,
                         bool L_LgtB8,
                         bool L_LgtB9,
                         bool L_LgtB11,
                         float L_DrvLeft,
                         float L_DrvRight,
                         float L_FishTape,
                         float L_LowArm,
                         float L_Winch)
  {
  float L_ArmError;
  float L_BallRotate;
  float L_ArmP = V_Ballarm.GetDistance();

  /* Set the output for the angle of the ball arm: */
  if (L_DesiredPos > 0)
    {
    /* If the desired position is above zero, the operator is commanding
     * the arm to a given position */
      L_ArmError = (L_DesiredPos - L_ArmP) * (-0.01);
      L_BallRotate = L_ArmError;
    }
  else if ((L_LgtB11 == true) && (L_ArmP < 211))
    {
    L_BallRotate = -1;
    }
  else if (L_LgtB5 && L_ArmP < 211)
    {
    L_BallRotate = -0.4;
    }
  else if (L_LgtB3 && L_ArmP > -10)
    {
    L_BallRotate = 0.4;
    }
  else if (L_LgtB6 && L_ArmP > 20)
    {
    L_BallRotate = 0.8;
    }
  else
    {
    /* If the desired position is zero, the operator is not wanting the arm
     * to be controlled.  Set the output to the motor to zero which will
     * disable the motor and allow the arm to naturally fall. */
      L_ArmError = 0;
      L_BallRotate = L_ArmError;
    }



  /* Limit the allowed arm command if it could go past the allowed vertical position: */
  if ((L_ArmP > 230 && L_LgtB5 == true) ||
      (L_ArmP < -10 && L_LgtB3 == true))
    {
    L_BallRotate = 0;
    }

  /* This is limiting for when the arm goes past the vertical position: */
  if (L_ArmP > 211)
    {
    L_BallRotate = 0.5;
    }

  V_BallRoller.Set(L_BallRoller);
  V_BallRotate.Set(L_BallRotate);
  V_MyDrive.TankDrive(L_DrvLeft, L_DrvRight);
  V_FishTape.Set(L_FishTape);
  V_LowArm.Set(L_LowArm);
  V_Winch.Set(L_Winch);
  }


/******************************************************************************
 * Function:     UpdateSmartDashboad
 *
 * Description:  Report data back to the smart dashboard.
 ******************************************************************************/
void UpdateSmartDashboad(bool   L_Switch1,
                         bool   L_Switch2,
                         bool   L_Switch3,
                         bool   L_Switch4,
                         bool   L_BallSwitch,
                         double L_AutoState,
                         double L_ArmAngleCmnd,
                         double L_ArmAngleAct,
                         double L_GyroAngle,
                         double L_AccelX,
                         double L_AccelY,
                         double L_AccelZ,
                         double L_LError,
                         double L_RError)
  {
    bool L_ArmUp;

    /* Check to see if it is ok to jump rough terrain: */
    if (L_ArmAngleAct > 200)
      {
      L_ArmUp = true;
      }


    double L_DisplayGyro = 0;

    /* Update the switch states on the robot: */
    SmartDashboard::PutBoolean("V_Sw1",L_Switch1);
    SmartDashboard::PutBoolean("V_Sw2",L_Switch2);
    SmartDashboard::PutBoolean("V_Sw3",L_Switch3);
    SmartDashboard::PutBoolean("V_Sw4",L_Switch4);

    /* Update the sensors on the robot: */
    SmartDashboard::PutNumber("XValue", L_AccelX);
    SmartDashboard::PutNumber("YValue", L_AccelY);
    SmartDashboard::PutNumber("ZValue", L_AccelZ);
    SmartDashboard::PutNumber("BallArmAngle",L_ArmAngleAct);
    SmartDashboard::PutNumber("GyroAngle",L_DisplayGyro);
    SmartDashboard::PutNumber("Drive Left Command",L_LError);
    SmartDashboard::PutNumber("Drive Right Command",L_RError);
    SmartDashboard::PutBoolean("Ball in the robot?", L_BallSwitch);

    /* Update the state variables: */
    SmartDashboard::PutNumber("Arm Angle Motor Command", L_ArmAngleCmnd);
    SmartDashboard::PutNumber("Auton state", L_AutoState);
    SmartDashboard::PutBoolean("Is the arm in the up position?", L_ArmUp);
    SmartDashboard::PutBoolean("Auton On", IsAutonomous());

    SmartDashboard::PutString("TestString", "Hello Team 5561!");
  }


/******************************************************************************
 * Function:     AutonomousInit
 *
 * Description:  Initialize anything that needs to be set for autonomous.
 ******************************************************************************/
   void AutonomousInit()
   {



   ReadAutonSwitch();
   UpdateActuatorCmnds(0,0,false,false,false,false,false,false,false,0,0,0,0,0);
   }

/******************************************************************************
 * Function:     AutonomousPeriodic
 *
 * Description:  This is the function that is called during the autonomous period.
 ******************************************************************************/
 void AutonomousPeriodic()
 {
 bool L_AutonDisabled;
 double L_AutonTime = 0;
 double L_AccelX, L_DesiredPos;
 float L_DriveL, L_DriveR;
 int V_AutonStateLocal;
 double L_ArmP;
 double L_ArmError;

// V_Light1.Set(1);
// V_Light2.Set(1);

 V_AutonStateLocal = 0;

 L_AutonDisabled = false;
 L_AutonTime = 0;
 while (IsAutonomous() && IsEnabled())
   {
   /* Read sensors here */
   //L_AccelX = V_Accel.GetX();
//   L_ArmP = V_Ballarm.GetDistance();


   if (L_AutonDisabled == false)
     {
     switch (V_AutonState)
       {
       case 2: /* Low bar and rough terrain */
         {
        if (V_AutonStateLocal == 0)
          {
          /* State 1: Raise arm to desired position */
          V_AutonStateLocal = 1;
          L_DriveL = 0.6;
          L_DriveR = 0.6;
          L_DesiredPos = 100;
          }
        else if (V_AutonStateLocal == 1 && L_AutonTime <  1.8)
          {
          L_DriveL = 0.6;
          L_DriveR = 0.6;
          L_DesiredPos = 100;
          }
        else if (L_AutonTime >=  1.8)
          {
          L_AutonDisabled = true;
          V_AutonStateLocal = 3;
          }
         break;
         }
       case 4: /* Low bar and rough terrain */
         {
             if (V_AutonStateLocal == 0)
               {
               /* State 1: Raise arm to desired position */
               V_AutonStateLocal = 1;
               L_DriveL = 0.8;
               L_DriveR = 0.8;
               L_DesiredPos = 180;
               }
             else if (V_AutonStateLocal == 1 && L_AutonTime <  1.8)
               {
               L_DriveL = 0.85;
               L_DriveR = 0.85;
               L_DesiredPos = 180;
               }
             else if (L_AutonTime >=  1.8)
               {
               L_AutonDisabled = true;
               V_AutonStateLocal = 3;
               }
         break;
         }
      case 0:
      default:
        {
        /* Default, no auton */
        L_DesiredPos = 0.0;
        L_DriveL = 0.0;
        L_DriveR = 0.0;
        break;
        }
      }
     }
   else
     {
     L_DriveL = 0.0;
     L_DriveR = 0.0;
     L_DesiredPos = 0;
     V_AutonStateLocal = 10;
     }

   UpdateSmartDashboad(V_Sw1.Get(),
                       V_Sw2.Get(),
                       V_Sw3.Get(),
                       V_Sw4.Get(),
                       V_BlSw.Get(),
                       V_AutonState,
                       0,
                       V_Ballarm.GetDistance(),
                       V_GyroOne.GetAngle(),
                       V_Accel.GetX(),
                       V_Accel.GetY(),
                       V_Accel.GetZ(),
                       (double)L_DriveL,
                       (double)L_DriveR);

   UpdateActuatorCmnds(0,
                       L_DesiredPos,
                       false,
                       false,
                       false,
                       false,
                       false,
                       false,
                       false,
                       L_DriveL,
                       L_DriveR,
                       0,
                       0,
                       0);

   Wait(C_UpdatePeriod); // Wait 5ms for the next update.
   L_AutonTime += C_UpdatePeriod;
   }
   Scheduler::GetInstance()->Run();
 }

/******************************************************************************
 * Function:     TeleopInit
 *
 * Description:  This is the function that is called during the init period for
 *               teleop.
 ******************************************************************************/
  void TeleopInit()
  {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        //if (autonomousCommand !=NULL)
        //      autonomousCommand->Cancel();
//	  bool matchColor = false;
//	  	   DriverStation::Alliance color;
//	  	   color = DriverStation::GetInstance().GetAlliance();
//	  	   if(color == DriverStation::Alliance::kBlue)
//	  	   {
//	  		   	  matchColor = true;
//	  	   		  V_Light1.Set(1);
//	  	   		  V_Light2.Set(0);
//	  	   		  SmartDashboard::PutBoolean("alliance",matchColor);
//	  	   	  }
//	  	   	  else if(color == DriverStation::Alliance::kRed)
//	  	   	  {
//	  	   		  matchColor = false;
//	  	   		  V_Light1.Set(0);
//	  	   		  V_Light2.Set(1);
//	  	   	  }
    UpdateActuatorCmnds(0,0,false,false,false,false,false,false,false,0,0,0,0,0);
  }

/******************************************************************************
 * Function:     TeleopPeriodic
 *
 * Description:  This is the function that is called during the periodic period
 *               for teleop.
 ******************************************************************************/
  void TeleopPeriodic()
    {
      float L_XB_DrvRY, L_XB_DrvLY;
      float L_BallRollerSpd;
      float L_FishTapeSpd;
      float K_WinchSpd;
      float L_LgtY, L_LgtZ;
      double L_LgtPOV;
      int L_LgtP1;
      bool L_LgtT0;
      bool L_LgtB1,L_LgtB3,L_LgtB4,L_LgtB5,L_LgtB6,L_LgtB7,L_LgtB8,L_LgtB9,L_LgtB11,L_LgtB10,L_LgtB12;
      bool L_XbxB4, L_XbxB5, L_XbxB2, L_XbxB1;
      bool L_BallSw;
      int L_XbxPOV;
      double L_ArmP;
      double L_ArmError;
      double L_DesiredPos;
      double L_LoArmCurve;


    while (IsOperatorControl() && IsEnabled())
      {

      /* Read sensor values here: */
      L_ArmP = V_Ballarm.GetDistance();

      L_BallSw = V_BlSw.Get();

      /* Read Xbox controller commands here: */
      L_XB_DrvLY = -V_XboxDrive.GetRawAxis(1);
      L_XB_DrvRY = -V_XboxDrive.GetRawAxis(5);

      L_XbxB1 = V_XboxDrive.GetRawButton(1);
      L_XbxB2 = V_XboxDrive.GetRawButton(2);
      L_XbxB4 = V_XboxDrive.GetRawButton(4);
      L_XbxB5 = V_XboxDrive.GetRawButton(5);

      L_XbxPOV = V_XboxDrive.GetPOV(0);

      /* Read Logictec joystick commands here: */
      L_LgtY = V_Logitech.GetRawAxis(1);
      L_LgtZ = V_Logitech.GetRawAxis(2);

      L_LgtP1  = V_Logitech.GetPOV(0);
      L_LgtT0  = V_Logitech.GetRawButton(1);
      L_LgtB1  = V_Logitech.GetRawButton(2);
      L_LgtB3  = V_Logitech.GetRawButton(3);
      L_LgtB4  = V_Logitech.GetRawButton(4);
      L_LgtB5  = V_Logitech.GetRawButton(5);
      L_LgtB6  = V_Logitech.GetRawButton(6);
      L_LgtB7  = V_Logitech.GetRawButton(7);
      L_LgtB8  = V_Logitech.GetRawButton(8);
      L_LgtB9  = V_Logitech.GetRawButton(9);
      L_LgtB10 = V_Logitech.GetRawButton(10);
      L_LgtB11 = V_Logitech.GetRawButton(11);
      L_LgtB12 = V_Logitech.GetRawButton(12);

      L_LgtPOV = (double)L_LgtP1;

     // DriverStation::
	 //double GetMatchTime();
	 // SmartDashboard::PutNumber("MatchTime", GetMatchTime());


      if(L_LgtB12 && L_LgtB10)
        {
        V_Ballarm.Reset();
        }


      /* Set the desired position of the ball arm. */
      if (L_LgtB7)
        {
        /* This is the middle position of the arm:  */
         L_DesiredPos = 120;
        }
      else if (L_LgtB8)
        {
        /* This is the upper position of the arm: */
          L_DesiredPos = 210;
        }
      else if((L_LgtB3) || (L_LgtB5) ||(L_LgtB6) || (L_LgtB9) || (L_LgtB11))
        {
        /* Default */
          L_DesiredPos = 0;
        }

      /* Set the ball roller state: */
      if (L_LgtT0 == true &&
          L_BallSw == true)
        {
          L_BallRollerSpd = K_BallRollerIn;
        }
      else if (L_LgtB1 == true)
        {
          L_BallRollerSpd = K_BallRollerOut;
        }
      else
        {
          L_BallRollerSpd = K_BallRollerStop;
        }


      /* Set the output to the fish tape: */
      if (L_LgtP1 == 180)
        {
          L_FishTapeSpd = K_FishTapeUp;
        }
      else if(L_LgtP1 == 0)
        {
          L_FishTapeSpd = K_FishTapeDown;
        }
      else
        {
          L_FishTapeSpd = K_FishTapeStop;
        }

      /* Set the output to the lower arm: */
      L_LoArmCurve = L_LgtY*K_LowArmGx;

//      SmartDashboard::


      /* Determine the winch state */
      if (L_LgtB4 == true)
        {
        K_WinchSpd = 1;
        }
      else if (L_XbxB4)
      {
    	  K_WinchSpd = -.5;
      }
      else
        {
        K_WinchSpd = 0.0;
        }

   /*   if(GetMatchTime() < 30)
      {
    	  V_Light1.Set(0);
    	  V_Light2.Set(0);
      }*/

      /* Output data to the smart dashboard: */
      ReadAutonSwitch();

      UpdateSmartDashboad(V_Sw1.Get(),
                          V_Sw2.Get(),
                          V_Sw3.Get(),
                          V_Sw4.Get(),
                          V_BlSw.Get(),
                          V_AutonState,
                          0,
                          V_Ballarm.GetDistance(),
                          V_GyroOne.GetAngle(),
                          V_Accel.GetX(),
                          V_Accel.GetY(),
                          V_Accel.GetZ(),
                          (double)L_XB_DrvLY,
                          (double)L_XB_DrvRY);

      UpdateActuatorCmnds(L_BallRollerSpd,
    		              L_DesiredPos,
                          L_LgtB3,
                          L_LgtB5,
                          L_LgtB6,
                          L_LgtB7,
                          L_LgtB8,
                          L_LgtB9,
                          L_LgtB11,
                          L_XB_DrvLY,
                          L_XB_DrvRY,
                          L_FishTapeSpd,
                          L_LoArmCurve,
                          K_WinchSpd);

      /* Force the program  to wait a period of time in order to conserve power: */
      Wait(C_UpdatePeriod); // Wait 5ms for the next update.

      Scheduler::GetInstance()->Run();
      }
  }


/******************************************************************************
 * Function:     TestPeriodic
 *
 * Description:  This is the function that is called during the periodic period
 *               for the test period.
 ******************************************************************************/
  void TestPeriodic()
  {
  V_LW->Run();
  }
};

START_ROBOT_CLASS(Robot);

// Ben Demick
// Conner Wallace
