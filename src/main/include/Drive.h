/****************************************************************************
    Description:	Defines the CDrive control class.

    Classes:		CDrive

    Project:		2021 ROMI

    Copyright 2021 ROMI - Clayton Cowen
****************************************************************************/
#ifndef Drive_h
#define Drive_h

#include <frc/Joystick.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/Encoder.h>
#include <frc/Spark.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/RamseteCommand.h>
#include <units/units.h>

#include "RomiGyro.h"
#include "TrajectoryConstants.h"
#include "IOMap.h"

using namespace frc;
using namespace std;
using namespace units;

// Constants.
const double kDrivebaseWidth                            = 5.822;
const DifferentialDriveKinematics kDriveKinematics      = DifferentialDriveKinematics(inch_t(kDrivebaseWidth));
const auto kGainS                                       = 1.22_V;
const auto kGainV                                       = 0.0741 * 1_V * 1_s / 1_in;
const auto kGainA                                       = 0.000117 * 1_V * 1_s * 1_s / 1_in;
const double kBeta                                      = 0.5;
const double kZeta                                      = 0.2;
const double kDriveProportional                         = 0.03;
const double kDriveIntegral                             = 0.0;
const double kDriveDerivative                           = 0.0000473;

const double kJoystickDeadzone                          = 0.2;
const double kCountsPerRevolution                       = 1440.0;
const inch_t kWheelDiameter                             = 2.75_in;
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CDrive class definition.

    Arguments:		Joystick pDriveController

    Derived From:	Nothing
******************************************************************************/
class CDrive
{
    public:
        CDrive(Joystick *pDriveController);
        ~CDrive();
        void Init();
        void Tick();
        void Stop();
        void GenerateTrajectory(vector<Pose2d> vWaypoints);
        void FollowTrajectory();
        void GeneratePathFromCurrentPosition();
        void SetDrivePowers(volt_t dLeftVoltage, volt_t dRightVoltage);
        DifferentialDriveWheelSpeeds GetWheelSpeeds();
        bool TrajectoryIsFinished();
        int GetTotalTrajectoryTime();
        Pose2d GetRobotPose();
        void SetSelectedTrajectory(int nAutoState);
        void StoreCurrentPosition();
        void ResetEncoders();
        void ResetOdometry();
        void SetMotorSafety(bool bSafetyEnabled);
        void SetJoystickControl(bool bJoystickControl);
        void SetUseUserDefinePointForPathGeneration(bool bUseUserDefinedPoint);
        double GetYaw();

    private:
        // Object Pointers.
        Joystick*                       m_pDriveController;
        Spark                           m_LeftDriveMotor{nLeftDriveMotorChannel};
        Spark                           m_RightDriveMotor{nRightDriveMotorChannel};
        Encoder*                        m_pLeftDriveEncoder;
        Encoder*                        m_pRightDriveEncoder;
        CRomiGyro*                      m_pGyro;
        DifferentialDrive               m_RobotDrive{m_LeftDriveMotor, m_RightDriveMotor};
        DifferentialDriveOdometry*      m_pOdometry;
        BuiltInAccelerometer*           m_pAccelerometer;
        frc2::RamseteCommand*           m_pRamseteCommand;
        Trajectory                      m_Trajectory;
        Field2d*                        m_pField;
        CTrajectoryConstants			m_TrajectoryConstants;

        // Member variables.
        bool					m_bJoystickControl;
        bool                    m_bUseUserDefinedPoint;
};
/////////////////////////////////////////////////////////////////////////////
#endif