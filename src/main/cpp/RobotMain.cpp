/******************************************************************************
	Description:	2021 Romi Robot Software

	Classes:		CRobotMain

	Project:		2021 ROMI

	Copyright 2021 ROMI - Clayton Cowen
******************************************************************************/
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

#include "RobotMain.h"
///////////////////////////////////////////////////////////////////////////////


/******************************************************************************
	Description:	CRobotMain Constructor.

    Arguments:		None

    Derived From:	TimedRobot
******************************************************************************/
CRobotMain::CRobotMain()
{
	// Create object pointers.
	m_pDriveController				= new Joystick(0);
	m_pTimer						= new Timer();
	m_pAutonomousChooser			= new SendableChooser<string>();
    m_pDrive                        = new CDrive(m_pDriveController);
    m_pArm                          = new CArm(m_pDriveController);

	// Initialize class variables.
	m_nTeleopState					= eTeleopStopped;
	m_nAutoState					= eAutoIdle;
}

/******************************************************************************
    Description:	CRobotMain Destructor.

    Arguments:		None

    Derived From:	TimedRobot
******************************************************************************/
CRobotMain::~CRobotMain()
{
	// Delete objects.
	delete m_pDriveController;
	delete m_pTimer;
	delete m_pAutonomousChooser;

	// Set pointers to nullptrs.
	m_pDriveController 		        = nullptr;
	m_pTimer				        = nullptr;
	m_pAutonomousChooser	        = nullptr;	
}

/****************************************************************************
    Description:	Ran on initial startup of the robot.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CRobotMain::RobotInit()
{
    // Call Init of all classes required.
    m_pDrive->Init();
    m_pArm->Init();

    // Put Autonomous things on the Dashboard.
    m_pAutonomousChooser->AddOption("Test Path", "Test Path");
    SmartDashboard::PutData(m_pAutonomousChooser);

    // Start the Timer.
    m_pTimer->Start();
}

/******************************************************************************
    Description:	Runs every 20ms in a loop after the robot has started.

    Arguments:	 	None

    Returns: 		Nothing
******************************************************************************/
void CRobotMain::RobotPeriodic()
{
    // Update robot states and info.
    SmartDashboard::PutNumber("Robot Timer", m_pTimer->Get());
}

/****************************************************************************
    Description:	Ran only once, when robot enters Disabled mode.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CRobotMain::DisabledInit()
{
    // Set joystick control.
    m_pDrive->SetJoystickControl(false);
}

/******************************************************************************
    Description:	Runs every 20ms in a loop after the robot has entered
                    Disabled mode.

    Arguments:	 	None

    Returns: 		Nothing
******************************************************************************/
void CRobotMain::DisabledPeriodic()
{

}

/******************************************************************************
    Description:	Ran only once, after the robot has entered Autonomous mode.

    Arguments:	 	None

    Returns: 		Nothing
******************************************************************************/
void CRobotMain::AutonomousInit()
{
    // Set joystick control.
    m_pDrive->SetJoystickControl(false);
    // Reset the arm to the default poisiton.
    m_pArm->ResetDefaultPosition();

    // Get auto start time.
    m_dStartTime = m_pTimer->Get();

    // Get the select auto mode from SmartDashboard.
    string m_strAutonomousSelected = m_pAutonomousChooser->GetSelected();
    cout << "STRING : " << m_strAutonomousSelected << endl;
    if (m_strAutonomousSelected == "Autonomous Idle")
    {
        m_nAutoState = eAutoIdle;
        m_nSelectedTrajectory = eDoNothing;
    }
    if (m_strAutonomousSelected == "Test Path")
    {
        m_nAutoState = eAutoTestPath1;
        m_nSelectedTrajectory = eTestPath1;
    }

    // Set the selected trajectory path. 
    if (m_nAutoState == eAutoIdle)
    {
        // Stop all robot functions and do nothing.
        m_pDrive->SetSelectedTrajectory(m_nSelectedTrajectory);
    }
    else
    {
        m_pDrive->SetSelectedTrajectory(m_nSelectedTrajectory);
    }
}

/******************************************************************************
    Description:	Runs every 20ms in a loop after the robot has entered
                    Autonomous mode.

    Arguments:	 	None

    Returns: 		Nothing
******************************************************************************/
void CRobotMain::AutonomousPeriodic()
{
	// Autonomous state machine.
	switch (m_nAutoState)
    {
        case eAutoIdle :
            // Do nothing.

            // Put values on smartdashboard.
            SmartDashboard::PutString("Robot State", "eAutoIdle");
            break;

        case eAutoTestPath1 : 
            // Follow test path.
            m_pDrive->FollowTrajectory();

            // Move to the next state when the robot is done path following.
            if (m_pDrive->TrajectoryIsFinished())
            {
                // Stop the drive motors.
                m_pDrive->Stop();

                // Move to the next state.
                m_nAutoState = eAutoIdle;
            }

            // Put values on smartdashboard.
            SmartDashboard::PutString("Robot State", "eAutoTestPath1");
            break;
        
        default:
            // Move to eAutoIdle state.
            m_nAutoState = eAutoIdle;
            break;
    }
}

/******************************************************************************
    Description:	Ran only once, after robot has entered Teleop mode.

    Arguments:	 	None

    Returns: 		Nothing
******************************************************************************/
void CRobotMain::TeleopInit()
{
    // Set Joystick control.
    m_pDrive->SetJoystickControl(true);
}

/******************************************************************************
    Description:	Runs every 20ms in a loop after the robot has entered 
                    Teleop mode.

    Arguments:	 	None

    Returns: 		Nothing
******************************************************************************/
void CRobotMain::TeleopPeriodic()
{   
    /*/////////////////////////////////////////////////////////////////////////
                Drive Controller - Reset arm position.     (A button)
    *//////////////////////////////////////////////////////////////////////////
    if (m_pDriveController->GetRawButton(eButtonA))
    {
        m_pArm->ResetDefaultPosition();
    }

    /*/////////////////////////////////////////////////////////////////////////
                Drive Controller - Move the pincher closed.     (B button)
    *//////////////////////////////////////////////////////////////////////////
    if (m_pDriveController->GetRawButton(eButtonB))
    {
        m_pArm->PinchServoClose();
    }

    /*/////////////////////////////////////////////////////////////////////////
                Drive Controller - Move the pincher opened.     (X button)
    *//////////////////////////////////////////////////////////////////////////
    if (m_pDriveController->GetRawButton(eButtonX))
    {
        m_pArm->PinchServoOpen();
    }

    /*/////////////////////////////////////////////////////////////////////////
                Drive Controller - Reset odometry.    (Y button)
    *//////////////////////////////////////////////////////////////////////////
    if (m_pDriveController->GetRawButtonPressed(eButtonY))
    {
        m_pDrive->ResetOdometry();
    }

    /*/////////////////////////////////////////////////////////////////////////
                Drive Controller - Store the robots current position. (Back)
    *//////////////////////////////////////////////////////////////////////////
    if (m_pDriveController->GetPOV() == 0)
    {
        m_pDrive->StoreCurrentPosition();
    }

    /*/////////////////////////////////////////////////////////////////////////
                Drive Controller - Generate path back to a predetermined path 
                                    and then follow it. (Start)
    *//////////////////////////////////////////////////////////////////////////
    if (m_pDriveController->GetRawButton(eStart))
    {
        /*/////////////////////////////////////////////////////////////////////////
                Drive Controller - Generate path with preset points or user-defined
                                    points. (Back)
        *//////////////////////////////////////////////////////////////////////////
        if (m_pDriveController->GetRawButtonPressed(eBack))
        {
            m_pDrive->SetUseUserDefinePointForPathGeneration(true);
        }
        else
        {
            m_pDrive->SetUseUserDefinePointForPathGeneration(false);
        }
        

        if (m_nTeleopState != eTeleopGeneratePath &&
            m_nTeleopState != eTeleopFollowing)
        {
            // Disable joystick controls.
            m_pDrive->SetJoystickControl(false);

            // Disable motor safety.
            m_pDrive->SetMotorSafety(false);

            // Move to eTeleopGeneratePath state.
            m_nTeleopState = eTeleopGeneratePath;
        }
    }
    else
    {
        // Stop following the path if the button is released.
        if ((m_nTeleopState == eTeleopGeneratePath ||
            m_nTeleopState == eTeleopFollowing) && !m_pDriveController->GetRawButtonPressed(eBack))
        {
            // Move to TeleopIdle
            m_nTeleopState = eTeleopIdle;
            
            // Stop the motors.
            m_pDrive->Stop();

            // Enable joystick controls.
            m_pDrive->SetJoystickControl(true);

            // Enable motor safety.
            m_pDrive->SetMotorSafety(true);
        }
    }

    // Teleop state machine.
    switch (m_nTeleopState)
    {
        case eTeleopStopped :
            // Do stopping actions here.

            // Move to teleop idle state.
            m_nTeleopState = eTeleopIdle;

            // Put values on smartdashboard.
            SmartDashboard::PutString("Robot State", "eTeleopStopped");
            break;

        case eTeleopIdle :
            // Do nothing.

            // Put values on smartdashboard.
            SmartDashboard::PutString("Robot State", "eTeleopIdle");
            break;

        case eTeleopGeneratePath :
            // Generate a path from our current position.
            m_pDrive->GeneratePathFromCurrentPosition();

            // Move to eTeleopFollowing.
            m_nTeleopState = eTeleopFollowing;

            // Put values on smartdashboard.
            SmartDashboard::PutString("Robot State", "eTeleopGeneratePath");
            break;

        case eTeleopFollowing :
            // Follow Trajectory.
            m_pDrive->FollowTrajectory();

            // Put values on smartdashboard.
            SmartDashboard::PutString("Robot State", "eTeleopFollowPath");
            break;

        default:
            // Move to teleop stopped state.
            m_nTeleopState = eTeleopStopped;
            break;
    }

    // Call ticks.
    m_pDrive->Tick();
    m_pArm->Tick();
}

/******************************************************************************
    Description:	Ran only once, after the robot has entered Test mode.

    Arguments:	 	None

    Returns: 		Nothing
******************************************************************************/
void CRobotMain::TestInit()
{

}

/******************************************************************************
    Description:	Runs every 20ms in a loop after the robot has entered
                    Test mode.

    Arguments:	 	None

    Returns: 		Nothing
******************************************************************************/
void CRobotMain::TestPeriodic()
{

}
///////////////////////////////////////////////////////////////////////////////
#ifndef RUNNING_FRC_TESTS
int main() {  return frc::StartRobot<CRobotMain>();  }
#endif