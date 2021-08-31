/******************************************************************************
	Description:	2021 Romi Robot Software

	Classes:		CRobotMain

	Project:		2021 ROMI

	Copyright 2021 ROMI - Clayton Cowen
******************************************************************************/
#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>

#include "Drive.h"
#include "Arm.h"
#include "IOMap.h"

using namespace frc;
using namespace std;
///////////////////////////////////////////////////////////////////////////////

/******************************************************************************
	Description:	CRobotMain class definition.

	Arguments:		None

	Derived From:	TimedRobot
******************************************************************************/
class CRobotMain : public frc::TimedRobot 
{
	public:
		CRobotMain();
		~CRobotMain();

	private:
		// Class override methods.
		void RobotInit() override;
		void RobotPeriodic() override;

		void AutonomousInit() override;
		void AutonomousPeriodic() override;

		void TeleopInit() override;
		void TeleopPeriodic() override;

		void DisabledInit() override;
		void DisabledPeriodic() override;

		void TestInit() override;
		void TestPeriodic() override;

		// Create objects.
		Joystick*							m_pDriveController;
		Timer*								m_pTimer;
		SendableChooser<string>*			m_pAutonomousChooser;
		CDrive*								m_pDrive;
		CArm*								m_pArm;

		// Declare variables.
		int									m_nTeleopState;
		int									m_nAutoState;
		int									m_nSelectedTrajectory;
		double 								m_dStartTime;

		// Constants.

		// Teleop States.
		enum TeleopStates
		{
			eTeleopStopped,
			eTeleopIdle,
			eTeleopGeneratePath,
			eTeleopFollowing
		};

		// Auto States.
		enum AutoStates
		{
			eAutoIdle,
			eAutoTestPath1
		};
};
///////////////////////////////////////////////////////////////////////////////
