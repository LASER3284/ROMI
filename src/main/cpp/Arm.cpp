/****************************************************************************
    Description:	Implements the CArm control class.

    Classes:		CArm

    Project:		2021 ROMI

    Copyright 2021 ROMI - Clayton Cowen
****************************************************************************/
#include "Arm.h"
#include "IOMap.h"

///////////////////////////////////////////////////////////////////////////////


/****************************************************************************
    Description:	CArm Constructor.

    Arguments:		Joystick* pDriveController

    Derived From:	Nothing
****************************************************************************/
CArm::CArm(Joystick* pDriveController)
{
	// Create objects.
	m_pDriveController 				= pDriveController;
    m_pLiftServo                    = new Servo(nLiftServoChannel);
    m_pTiltServo                    = new Servo(nTiltServoChannel);
    m_pPinchServo                   = new Servo(nPinchServoChannel);

	// Initialize class variables.
	m_bArmEnabled 				    = true;

    // Setup servos.
    m_pPinchServo->SetBounds(kPinchServoMaxPos, kServoDeadband, (kPinchServoMaxPos - kPinchServoMinPos) / 2, kServoDeadband, kPinchServoMinPos);
}

/****************************************************************************
    Description:	CArm Destructor.

    Arguments:		None

    Derived From:	Nothing
****************************************************************************/
CArm::~CArm()
{
    // Delete objects.
    delete m_pDriveController;
    

    // Set pointers to nullptrs.
	m_pDriveController				= nullptr;
}

/****************************************************************************
    Description:	Initialize drive parameters.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CArm::Init()
{

}

/****************************************************************************
    Description:	Main method that calls functionality, to be used in a loop.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CArm::Tick()
{
    // Create method variables.
    string sArmState = "";

    // If joystick control enabled, then control robot drive with controller.
    if (m_bArmEnabled)
    {
        // Get joystick values.
        double dXAxis = -m_pDriveController->GetRawAxis(eRightAxisX);
        double dYAxis = m_pDriveController->GetRawAxis(eRightAxisY);

        // Check for joystick deadzone.
        if (fabs(dXAxis) < kJoystickDeadzone)
        {
            dXAxis = 0.0;
        }
        if (fabs(dYAxis) < kJoystickDeadzone)
        {
            dYAxis = 0.0;
        }

        // Apply speed multiplier to the joysticks for easier control.
        double dSpeedX = fabs(dXAxis * kLiftTiltSpeedMultiplier);
        double dSpeedY = fabs(dYAxis * kLiftTiltSpeedMultiplier);

        // Set the servo powers.
        if (dYAxis > kJoystickDeadzone)
        {
            // Move the lift down.
            LiftServoUp(dSpeedY);

            // Add arm state info.
            sArmState += "Lift Up";
        }
        if (dYAxis < -kJoystickDeadzone)
        {
            // Move the lift up.
            LiftServoDown(dSpeedY);

            // Add arm state info.
            sArmState += "Lift Down";
        }
        if (dXAxis > kJoystickDeadzone)
        {
            // Move the tilt up.
            TiltServoUp(dSpeedX);

            // Add arm state info.
            if (sArmState.empty())
            {
                sArmState += "Tilt Up";
            }
            else
            {
                sArmState += " + Tilt Up";
            }
        }
        if (dXAxis < -kJoystickDeadzone)
        {
            // Move the tilt down.
            TiltServoDown(dSpeedX);

            // Add arm state info.
            if (sArmState.empty())
            {
                sArmState += "Tilt Down";
            }
            else
            {
                sArmState += " + Tilt Down";
            }
        }
        
        // Monitor the pincher and add state.
        if (m_pPinchServo->GetSpeed() > 0.05)
        {
            // Add arm state info.
            if (sArmState.empty())
            {
                sArmState += "Pinch Closed";
            }
            else
            {
                sArmState += " + Pinch Closed";
            }
        }
        if (m_pPinchServo->GetSpeed() < -0.05)
        {
            // Add arm state info.
            if (sArmState.empty())
            {
                sArmState += "Pinch Opened";
            }
            else
            {
                sArmState += " + Pinch Opened";
            }
        }
    }

    // Put servo positions on smartdashboard.
    SmartDashboard::PutNumber("LiftServoPosition", m_pLiftServo->Get());
    SmartDashboard::PutNumber("TiltServoPosition", m_pTiltServo->Get());
    SmartDashboard::PutNumber("PinchServoPosition", m_pPinchServo->Get());
    SmartDashboard::PutNumber("Pinch Speed", m_pPinchServo->GetSpeed());
    SmartDashboard::PutString("Arm State", sArmState);
}

/****************************************************************************
    Description:	Move the lift servo down.

    Arguments: 		None.

    Returns: 		Nothing
****************************************************************************/
void CArm::LiftServoDown(double dSpeed)
{
    if (m_pLiftServo->Get() < kLiftServoMaxPos)
    {
        m_pLiftServo->Set(m_pLiftServo->Get() + dSpeed);
    }
}

/****************************************************************************
    Description:	Move the lift servo up.

    Arguments: 		None.

    Returns: 		Nothing
****************************************************************************/
void CArm::LiftServoUp(double dSpeed)
{
    if (m_pLiftServo->Get() > kLiftServoMinPos)
    {
        m_pLiftServo->Set(m_pLiftServo->Get() - dSpeed);
    }
}

/****************************************************************************
    Description:	Move the tilt servo up.

    Arguments: 		None.

    Returns: 		Nothing
****************************************************************************/
void CArm::TiltServoUp(double dSpeed)
{
    if (m_pTiltServo->Get() < kTiltServoMaxPos)
    {
        m_pTiltServo->Set(m_pTiltServo->Get() + dSpeed);
    }
}

/****************************************************************************
    Description:	Move the tilt servo down.

    Arguments: 		None.

    Returns: 		Nothing
****************************************************************************/
void CArm::TiltServoDown(double dSpeed)
{
    if (m_pTiltServo->Get() > kTiltServoMinPos)
    {
        m_pTiltServo->Set(m_pTiltServo->Get() - dSpeed);
    }
}

/****************************************************************************
    Description:	Move the pinch servo closed.

    Arguments: 		None.

    Returns: 		Nothing
****************************************************************************/
void CArm::PinchServoClose()
{
    if (m_pPinchServo->Get() < kPinchServoMaxPos)
    {
        m_pPinchServo->Set(m_pPinchServo->Get() + kPitchSpeed);
    }
}

/****************************************************************************
    Description:	Move the pinch servo opened.

    Arguments: 		None.

    Returns: 		Nothing
****************************************************************************/
void CArm::PinchServoOpen()
{
    if (m_pPinchServo->Get() > kPinchServoMinPos)
    {
        m_pPinchServo->Set(m_pPinchServo->Get() - kPitchSpeed);
    }
}

/****************************************************************************
    Description:	Stop - Method that stops drive motors.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CArm::ResetDefaultPosition()
{
    // Set Servos back to a neutral position.
    m_pLiftServo->Set(kLiftHomePos);
    m_pTiltServo->Set(kTiltHomePos);
    m_pPinchServo->Set((kPinchServoMaxPos - kPinchServoMinPos) / 2);
}

/****************************************************************************
    Description:	SetJoystickControl - Sets the arm enabled/disabled.

    Arguments: 		bool bEnableArm - True if arm control enabled,
                    false otherwise.

    Returns: 		Nothing
****************************************************************************/
void CArm::EnableArm(bool bEnableArm)
{
    m_bArmEnabled = bEnableArm;
}