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
#include "USBCamera.h"
#include "CameraServer.h"
#include "DigitalOutput.h"
#include "Timer.h"
#include "DriverStation.h"

class Robot: public IterativeRobot {
    Joystick xboxDrive; //joystick
    Joystick Logitech;
    RobotDrive myDrive; // robot drive
    Talon LowArm;
    Victor FishTape;
    Talon Wench;
    Talon BallRotate;
    Talon BallRoller;
    LiveWindow *lw;
    DigitalInput BlSw;
    DigitalInput Sw1, Sw2, Sw3, Sw4;
    DigitalOutput Light1;
    DigitalOutput Light2;
    Encoder ballarm;
    ADXRS450_Gyro gyroOne;

    const double kUpdatePeriod = 0.005;
    double AutonTime;
    int AutonState;
    bool AutonDisabled;

    /* Controller button s */
    const int A_btn = 1;
    const int B_btn = 2;
    const int X_btn = 3;
    const int Y_btn = 4;
    const int LB_btn = 5;
    const int RB_btn = 6;
    const int Back_btn = 7;
    const int Start_btn = 8;
    const int LStick_push = 9;
    const int RStick_push = 10;

    /* Motor gains */
    const float BallRollerFwd = 0.90;
    const float BallRollerRvrs = -0.9;
    const float WenchFwd = 0.4;
    const float WenchRvrs = -0.4;
    const float BallArmUp = 0.4;
    const float BallArmDwn = -0.4;
    const float FishTapeUp = 0.5;
    const float FishTapeDwn = -0.3;
    const float WenchSpd = 0.4;
    const float LowArmGx = 0.6;
    const float RotationGx = 0.6;
    const float RotationMin = 0.5;
    const float RotationMax = 0.8;





public:
    Robot() :
      xboxDrive(0),// Initialize Joystick on port 0.
      Logitech(1),
      LowArm(4),
      FishTape(7),
      BallRotate(6), // Port 2 is dead
      BallRoller(3),
      Wench(5),
      myDrive(0, 1), // Initialize the Victor on channel 2.
      lw(NULL),
      BlSw(9), // get DI value from input pos 1
      ballarm(0,1,true,Encoder::EncodingType::k4X),
      Sw1(5),
      Sw2(6),
      Sw3(7),
      gyroOne(SPI::Port::kOnboardCS0),
      Sw4(8),
	  Light1(4),
	  Light2(3)
    {
    myDrive.SetExpiration(0.1);
    }

private:
      BuiltInAccelerometer accel;

 /******************************************************************************
  * Function:     RobotInit
  *
  * Description:  Run the functions that are expected during the robot init.
  ******************************************************************************/

  void RobotInit ()
  {
  lw = LiveWindow::GetInstance();
  CameraServer::GetInstance()->SetQuality(50);
   //the camera name (ex "cam0") can be found through the roborio web interface
  CameraServer::GetInstance()->StartAutomaticCapture("cam1");
  AutonState = 0;
  ballarm.Reset();
  ballarm.SetMaxPeriod(.01);
  ballarm.SetMinRate(.02);
  ballarm.SetDistancePerPulse(.9);
  gyroOne.Calibrate();
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

  Light1.Set(1);
  Light2.Set(1);


  while(IsDisabled())
  {
    UpdateActuatorCmnds(0,0,false,false,false,false,false,false,false,0,0,0,0,0);

    ReadAutonSwitch();

    UpdateSmartDashboad(Sw1.Get(),
                        Sw2.Get(),
                        Sw3.Get(),
                        Sw4.Get(),
                        BlSw.Get(),
                        (double)AutonState,
                        (double)0,
                        ballarm.GetDistance(),
                        gyroOne.GetAngle(),
                        accel.GetX(),
                        accel.GetY(),
                        accel.GetZ(),
                        (double)0,
                        (double)0);
  wait(kUpdatePeriod);
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
    if ((Sw1.Get() == false) && (Sw2.Get() == true) && (Sw3.Get() == true) && (Sw4.Get() == true))
      {
      /* Moat and rock wall */
      AutonState = 1;
      }
    else if ((Sw1.Get() == true) && (Sw2.Get() == false) && (Sw3.Get() == true) && (Sw4.Get() == true))
      {
      /* Uneven ramps */
      AutonState = 2;
      }
    else if ((Sw1.Get() == false) && (Sw2.Get() == false) && (Sw3.Get() == true) && (Sw4.Get() == true))
      {
      /* Low bar and rough terrain */
      AutonState = 3;
      }
    else if ((Sw1.Get() == true) && (Sw2.Get() == true) && (Sw3.Get() == false) && (Sw4.Get() == true))
      {
      /* Low power */
      AutonState = 4;
      }
    /*else if ((Sw1.Get() == false) && (Sw2.Get() == true) && (Sw3.Get() == false) && (Sw4.Get() == true))
      {
      /* Low bar, forward and then reverse
      AutonState = 5;
      }*/
    else if ((Sw1.Get() == true) && (Sw2.Get() == false) && (Sw3.Get() == false) && (Sw4.Get() == true))
      {
      /* Low bar, forward and then turn */
      AutonState = 6;
      }
    else if ((Sw1.Get() == false) && (Sw2.Get() == false) && (Sw3.Get() == false) && (Sw4.Get() == true))
      {
      /* Low bar, forward and then turn */
      AutonState = 7;
      }
    else if ((Sw1.Get() == true) && (Sw2.Get() == true) && (Sw3.Get() == true) && (Sw4.Get() == false))
    {
    	AutonState = 8;
    }
    else if ((Sw1.Get() == false) && (Sw2.Get() == true) && (Sw3.Get() == true) && (Sw4.Get() == false))
    {
    	AutonState = 9;
    }
    else
      {
      /* Default, no auton */
      AutonState = 0;
      }
  }

/******************************************************************************
 * Function:     UpdateActuatorCmnds
 *
 * Description:  Update the commands sent out to the RoboRIO.
 ******************************************************************************/
void UpdateActuatorCmnds(float ballRoller,
                         float desiredPos,
                         bool LgtB3,
                         bool LgtB5,
                         bool LgtB6,
                         bool LgtB7,
                         bool LgtB8,
                         bool LgtB9,
                         bool LgtB11,
                         float DrvLeft,
                         float DrvRight,
                         float fishTape,
                         float lowArm,
                         float wench)
  {
  float ArmError;
  float ballRotate;
  float ArmP = ballarm.GetDistance();

  /* Set the output for the angle of the ball arm: */
  if (desiredPos > 0)
    {
    /* If the desired position is above zero, the operator is commanding
     * the arm to a given position */
      ArmError = (desiredPos - ArmP) * (-0.01);
      ballRotate = ArmError;
    }
  else if ((LgtB11 == true) && (ArmP < 211))
    {
    ballRotate = -1;
    }
  else if (LgtB5 && ArmP < 211)
    {
    ballRotate = -0.4;
    }
  else if (LgtB3 && ArmP > -10)
    {
    ballRotate = 0.4;
    }
  else if (LgtB6 && ArmP > 20)
    {
    ballRotate = 0.8;
    }
  else
    {
    /* If the desired position is zero, the operator is not wanting the arm
     * to be controlled.  Set the output to the motor to zero which will
     * disable the motor and allow the arm to naturally fall. */
      ArmError = 0;
      ballRotate = ArmError;
    }



  /* Limit the allowed arm command if it could go past the allowed vertical position: */
  if ((ArmP > 230 && LgtB5 == true) ||
      (ArmP < -10 && LgtB3 == true))
    {
    ballRotate = 0;
    }

  /* This is limiting for when the arm goes past the vertical position: */
  if (ArmP > 211)
    {
    ballRotate = 0.5;
    }

  BallRoller.Set(ballRoller);
  BallRotate.Set(ballRotate);
  myDrive.TankDrive(DrvLeft, DrvRight);
  FishTape.Set(fishTape);
  LowArm.Set(lowArm);
  Wench.Set(wench);
  }


/******************************************************************************
 * Function:     UpdateSmartDashboad
 *
 * Description:  Report data back to the smart dashboard.
 ******************************************************************************/
void UpdateSmartDashboad(bool   Switch1,
                         bool   Switch2,
                         bool   Switch3,
                         bool   Switch4,
                         bool   BallSwitch,
                         double AutoState,
                         double ArmAngleCmnd,
                         double ArmAngleAct,
                         double GyroAngle,
                         double AccelX,
                         double AccelY,
                         double AccelZ,
                         double LError,
                         double RError)
  {
    bool ArmUp;

    /* Check to see if it is ok to jump rough terrain: */
    if (ArmAngleAct > 200)
      {
      ArmUp = true;
      }


    double DisplayGyro = 0;

    /* Update the switch states on the robot: */
    SmartDashboard::PutBoolean("Sw1",Switch1);
    SmartDashboard::PutBoolean("Sw2",Switch2);
    SmartDashboard::PutBoolean("Sw3",Switch3);
    SmartDashboard::PutBoolean("Sw4",Switch4);

    /* Update the sensors on the robot: */
    SmartDashboard::PutNumber("XValue", AccelX);
    SmartDashboard::PutNumber("YValue", AccelY);
    SmartDashboard::PutNumber("ZValue", AccelZ);
    SmartDashboard::PutNumber("BallArmAngle",ArmAngleAct);
    SmartDashboard::PutNumber("GyroAngle",DisplayGyro);
    SmartDashboard::PutNumber("Drive Left Command",LError);
    SmartDashboard::PutNumber("Drive Right Command",RError);
    SmartDashboard::PutBoolean("Ball in the robot?", BallSwitch);

    /* Update the state variables: */
    SmartDashboard::PutNumber("Arm Angle Motor Command", ArmAngleCmnd);
    SmartDashboard::PutNumber("Auton state", AutoState);
    SmartDashboard::PutBoolean("Is the arm in the up position?", ArmUp);

    SmartDashboard::PutString("TestString", "Hello Team 5561!");
  }


/******************************************************************************
 * Function:     AutonomousInit
 *
 * Description:  Initialize anything that needs to be set for autonomous.
 ******************************************************************************/
   void AutonomousInit()
   {


   AutonDisabled = false;
   AutonTime = 0;

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
 double accelX, desiredPos;
 float DriveL, DriveR;
 int AutonStateLocal;
 double ArmP;
 double ArmError;

// Light1.Set(1);
// Light2.Set(1);

 AutonStateLocal = 0;


 while (IsAutonomous())
   {
   /* Read sensors here */
   accelX = accel.GetX();
//   ArmP = ballarm.GetDistance();


   if (AutonDisabled == false)
     {
     switch (AutonState)
       {
       case 2: /* Low bar and rough terrain */
         {
        if (AutonStateLocal == 0)
          {
          /* State 1: Raise arm to desired position */
          AutonStateLocal = 1;
          DriveL = 0.6;
          DriveR = 0.6;
          desiredPos = 100;
          }
        else if (AutonStateLocal == 1 && AutonTime <  1.8)
          {
          DriveL = 0.6;
          DriveR = 0.6;
          desiredPos = 100;
          }
        else if (AutonTime >=  1.8)
          {
          AutonDisabled = true;
          AutonStateLocal = 3;
          }
         break;
         }
       case 4: /* Low bar and rough terrain */
         {
             if (AutonStateLocal == 0)
               {
               /* State 1: Raise arm to desired position */
               AutonStateLocal = 1;
               DriveL = 0.8;
               DriveR = 0.8;
               desiredPos = 180;
               }
             else if (AutonStateLocal == 1 && AutonTime <  1.8)
               {
               DriveL = 0.85;
               DriveR = 0.85;
               desiredPos = 180;
               }
             else if (AutonTime >=  1.8)
               {
               AutonDisabled = true;
               AutonStateLocal = 3;
               }
         break;
         }
      case 0:
      default:
        {
        /* Default, no auton */
        desiredPos = 0.0;
        DriveL = 0.0;
        DriveR = 0.0;
        break;
        }
      }
     }
   else
     {
     DriveL = 0.0;
     DriveR = 0.0;
     desiredPos = 0;
     AutonStateLocal = 10;
     }

   UpdateSmartDashboad(Sw1.Get(),
                       Sw2.Get(),
                       Sw3.Get(),
                       Sw4.Get(),
                       BlSw.Get(),
                       AutonState,
                       0,
                       ballarm.GetDistance(),
                       gyroOne.GetAngle(),
                       accel.GetX(),
                       accel.GetY(),
                       accel.GetZ(),
                       (double)DriveL,
                       (double)DriveR);

   UpdateActuatorCmnds(0,
                       desiredPos,
                       false,
                       false,
                       false,
                       false,
                       false,
                       false,
                       false,
                       DriveL,
                       DriveR,
                       0,
                       0,
                       0);

   Wait(kUpdatePeriod); // Wait 5ms for the next update.
   AutonTime += kUpdatePeriod;
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
//	  	   		  Light1.Set(1);
//	  	   		  Light2.Set(0);
//	  	   		  SmartDashboard::PutBoolean("alliance",matchColor);
//	  	   	  }
//	  	   	  else if(color == DriverStation::Alliance::kRed)
//	  	   	  {
//	  	   		  matchColor = false;
//	  	   		  Light1.Set(0);
//	  	   		  Light2.Set(1);
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
      float DrvRY, DrvLY;
      float BallRollerSpd;
      float FishTapeSpd;
      float WenchSpd;
      float LgtY, LgtZ;
      double LgtPOV;
      int LgtP1;
      bool LgtT0;
      bool LgtB1,LgtB3,LgtB4,LgtB5,LgtB6,LgtB7,LgtB8,LgtB9,LgtB11,LgtB10,LgtB12;
      bool XbxB4, XbxB5, XbxB2, XbxB1;
      bool BallSw;
      int XbxPOV;
      double ArmP;
      double ArmError;
      double desiredPos;
      double LoArmCurve;


    while (IsOperatorControl() && IsEnabled())
      {

      /* Read sensor values here: */
      ArmP = ballarm.GetDistance();

      BallSw = BlSw.Get();

      /* Read Xbox controller commands here: */
      DrvLY = -xboxDrive.GetRawAxis(1);
      DrvRY = -xboxDrive.GetRawAxis(5);

      XbxB1 = xboxDrive.GetRawButton(1);
      XbxB2 = xboxDrive.GetRawButton(2);
      XbxB4 = xboxDrive.GetRawButton(4);
      XbxB5 = xboxDrive.GetRawButton(5);

      XbxPOV = xboxDrive.GetPOV(0);

      /* Read Logictec joystick commands here: */
      LgtY = Logitech.GetRawAxis(1);
      LgtZ = Logitech.GetRawAxis(2);

      LgtP1  = Logitech.GetPOV(0);
      LgtT0  = Logitech.GetRawButton(1);
      LgtB1  = Logitech.GetRawButton(2);
      LgtB3  = Logitech.GetRawButton(3);
      LgtB4  = Logitech.GetRawButton(4);
      LgtB5  = Logitech.GetRawButton(5);
      LgtB6  = Logitech.GetRawButton(6);
      LgtB7  = Logitech.GetRawButton(7);
      LgtB8  = Logitech.GetRawButton(8);
      LgtB9  = Logitech.GetRawButton(9);
      LgtB10 = Logitech.GetRawButton(10);
      LgtB11 = Logitech.GetRawButton(11);
      LgtB12 = Logitech.GetRawButton(12);

      LgtPOV = (double)LgtP1;

     // DriverStation::
	 //double GetMatchTime();
	 // SmartDashboard::PutNumber("MatchTime", GetMatchTime());


      if(LgtB12 && LgtB10)
        {
        ballarm.Reset();
        }


      /* Set the desired position of the ball arm. */
      if (LgtB7)
        {
        /* This is the middle position of the arm:  */
         desiredPos = 120;
        }
      else if (LgtB8)
        {
        /* This is the upper position of the arm: */
          desiredPos = 210;
        }
      else if((LgtB3) || (LgtB5) ||(LgtB6) || (LgtB9) || (LgtB11))
        {
        /* Default */
          desiredPos = 0;
        }

      /* Set the ball roller state: */
      if (LgtT0 == true &&
          BallSw == true)
        {
          BallRollerSpd = 1;
        }
      else if (LgtB1 == true)
        {
          BallRollerSpd = -1;
        }
      else
        {
          BallRollerSpd = 0.0;
        }


      /* Set the output to the fish tape: */
      if (LgtP1 == 180)
        {
          FishTapeSpd = -1.0;
        }
      else if(LgtP1 == 0)
        {
          FishTapeSpd = 1.0;
        }
      else
        {
          FishTapeSpd = 0.0;
        }

      /* Set the output to the lower arm: */
      LoArmCurve = LgtY*LowArmGx;

//      SmartDashboard::


      /* Determine the wench state */
      if (LgtB4 == true)
        {
        WenchSpd = 1;
        }
      else if (XbxB4)
      {
    	  WenchSpd = -.5;
      }
      else
        {
        WenchSpd = 0.0;
        }

   /*   if(GetMatchTime() < 30)
      {
    	  Light1.Set(0);
    	  Light2.Set(0);
      }*/

      /* Output data to the smart dashboard: */
      ReadAutonSwitch();

      UpdateSmartDashboad(Sw1.Get(),
                          Sw2.Get(),
                          Sw3.Get(),
                          Sw4.Get(),
                          BlSw.Get(),
                          AutonState,
                          0,
                          ballarm.GetDistance(),
                          gyroOne.GetAngle(),
                          accel.GetX(),
                          accel.GetY(),
                          accel.GetZ(),
                          (double)DrvLY,
                          (double)DrvRY);

      UpdateActuatorCmnds(BallRollerSpd,
    		              desiredPos,
                          LgtB3,
                          LgtB5,
                          LgtB6,
                          LgtB7,
                          LgtB8,
                          LgtB9,
                          LgtB11,
                          DrvLY,
                          DrvRY,
                          FishTapeSpd,
                          LoArmCurve,
                          WenchSpd);

      /* Force the program  to wait a period of time in order to conserve power: */
      Wait(kUpdatePeriod); // Wait 5ms for the next update.

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
  lw->Run();
  }
};

START_ROBOT_CLASS(Robot);

// Ben Demick
// Conner Wallace
