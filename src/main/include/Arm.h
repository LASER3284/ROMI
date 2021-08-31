/****************************************************************************
    Description:	Defines the CArm control class.

    Classes:		CArm

    Project:		2021 ROMI

    Copyright 2021 ROMI - Clayton Cowen
****************************************************************************/
#ifndef Arm_h
#define Arm_h

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Joystick.h>
#include <frc/Servo.h>
#include <units/units.h>

using namespace frc;
using namespace std;
using namespace units;
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CArm class definition.

    Arguments:		Joystick pDriveController

    Derived From:	Nothing
******************************************************************************/
class CArm
{
    public:
        CArm(Joystick *pDriveController);
        ~CArm();
        void Init();
        void Tick();
        void LiftServoUp(double dSpeed);
        void LiftServoDown(double dSpeed);
        void TiltServoUp(double dSpeed);
        void TiltServoDown(double dSpeed);
        void PinchServoClose();
        void PinchServoOpen();
        void ResetDefaultPosition();
        void EnableArm(bool bEnableArm);

    private:
        // Object Pointers.
        Joystick*                           m_pDriveController;
        Servo*								m_pLiftServo;
        Servo*								m_pTiltServo;
        Servo*								m_pPinchServo;

        // Member variables.
        double                              m_dPitchServoPosition;
        bool					            m_bArmEnabled;

        // Constants.
        const double kJoystickDeadzone              = 0.2;
        const double kLiftTiltSpeedMultiplier       = 0.025;
        const double kPitchSpeed                    = 0.1;
        const double kServoDeadband                 = 0.05;
        const double kLiftServoMaxPos               = 0.5;
        const double kLiftServoMinPos               = 0.05;
        const double kTiltServoMaxPos               = 0.65;
        const double kTiltServoMinPos               = 0.41;
        const double kPinchServoMaxPos              = 0.9;
        const double kPinchServoMinPos              = 0.1;
        const double kLiftHomePos                   = 0.5;
        const double kTiltHomePos                   = 0.56;
};
/////////////////////////////////////////////////////////////////////////////
#endif