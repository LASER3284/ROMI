/****************************************************************************
    Description:	Implements the CDrive control class.

    Classes:		CDrive

    Project:		2021 ROMI

    Copyright 2021 ROMI - Clayton Cowen
****************************************************************************/
#include "Drive.h"
#include "IOMap.h"

///////////////////////////////////////////////////////////////////////////////


/****************************************************************************
    Description:	CDrive Constructor.

    Arguments:		Joystick* pDriveController

    Derived From:	Nothing
****************************************************************************/
CDrive::CDrive(Joystick* pDriveController)
{
	// Create objects.
	m_pDriveController 				= pDriveController;
	m_pLeftDriveEncoder				= new Encoder(nLeftDriveEncoderChannelA, nLeftDriveEncoderChannelB);
	m_pRightDriveEncoder			= new Encoder(nRightDriveEncoderChannelA, nRightDriveEncoderChannelB);
    m_pGyro                         = new CRomiGyro();
    m_pOdometry                     = new DifferentialDriveOdometry(Rotation2d(degree_t(GetYaw())), m_TrajectoryConstants.GetSelectedTrajectoryStartPoint());
    m_pAccelerometer                = new BuiltInAccelerometer();
    m_pField                        = new Field2d();
    
	// Initialize class variables.
	m_bJoystickControl 				= true;
    m_bUseUserDefinedPoint          = false;

    // Setup encoders.
    m_pLeftDriveEncoder->SetDistancePerPulse(double(wpi::math::pi * kWheelDiameter) / kCountsPerRevolution);
    m_pRightDriveEncoder->SetDistancePerPulse(double(wpi::math::pi * kWheelDiameter) / kCountsPerRevolution);
}

/****************************************************************************
    Description:	CDrive Destructor.

    Arguments:		None

    Derived From:	Nothing
****************************************************************************/
CDrive::~CDrive()
{
    // Delete objects.
    delete m_pDriveController;
    delete m_pLeftDriveEncoder;
    delete m_pRightDriveEncoder;

    // Set pointers to nullptrs.
	m_pDriveController				= nullptr;
    m_pLeftDriveEncoder             = nullptr;
    m_pRightDriveEncoder            = nullptr;
}

/****************************************************************************
    Description:	Initialize drive parameters.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CDrive::Init()
{

}

/****************************************************************************
    Description:	Main method that calls functionality, to be used in a loop.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CDrive::Tick()
{
    // If joystick control enabled, then control robot drive with controller.
    if (m_bJoystickControl)
    {
        // Get joystick values.
        double dXAxis = m_pDriveController->GetRawAxis(eLeftAxisX);
        double dYAxis = -m_pDriveController->GetRawAxis(eLeftAxisY);

        // Check for joystick deadzone.
        if (fabs(dXAxis) < kJoystickDeadzone)
        {
            dXAxis = 0.0;
        }
        if (fabs(dYAxis) < kJoystickDeadzone)
        {
            dYAxis = 0.0;
        }

        // Set drivetrain powers.
        m_RobotDrive.ArcadeDrive(dYAxis, dXAxis, true);
    }

    // Update odometry and robot pose on field.
    m_pOdometry->Update(Rotation2d(degree_t(GetYaw())), inch_t(m_pLeftDriveEncoder->GetDistance()), inch_t(m_pRightDriveEncoder->GetDistance()));
    m_pField->SetRobotPose(GetRobotPose());

    // Put values on smartdashboard.
    SmartDashboard::PutNumber("Robot Pos X", double(GetRobotPose().X()) * 39.3701);
    SmartDashboard::PutNumber("Robot Pos Y", double(GetRobotPose().Y()) * 39.3701);
    SmartDashboard::PutNumber("Robot Pos Z", double(GetRobotPose().Rotation().Degrees()));
    SmartDashboard::PutNumber("Wheel Distance Left", m_pRightDriveEncoder->GetDistance());
    SmartDashboard::PutNumber("Wheel Distance Right", m_pLeftDriveEncoder->GetDistance());
    SmartDashboard::PutNumber("Robot Speed", m_pLeftDriveEncoder->GetRate());
    SmartDashboard::PutData("Robot Position", ref(m_pField));
}

/****************************************************************************
    Description:	Stop - Method that stops drive motors.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CDrive::Stop()
{
    // Set drive powers to zero.
    m_LeftDriveMotor.Set(0);
    m_RightDriveMotor.Set(0);
}

/****************************************************************************
    Description:	GenerateTrajectory - MotionProfiling method that 
                    generates a new trajectory.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CDrive::GenerateTrajectory(vector<Pose2d> vWaypoints)
{
    // Create voltage constraint to make sure we don't accelerate too fast.
    auto m_VoltageConstraint = DifferentialDriveVoltageConstraint(SimpleMotorFeedforward<meters>(kGainS, kGainV, kGainA), kDriveKinematics, 5_V);

    // Create the trajectory config.
    auto m_Config = TrajectoryConfig(m_TrajectoryConstants.kMaxSpeed, m_TrajectoryConstants.kMaxAcceleration);
    // Add kinematics to ensure max speed is actually obeyed.
    m_Config.SetKinematics(kDriveKinematics);
    // Apply the voltage constraint.
    m_Config.AddConstraint(m_VoltageConstraint);

    // Set trajectory parameters.
    if (m_TrajectoryConstants.GetIsTrajectoryReversed())
    {
        m_Config.SetReversed(true);
    }
    else
    {
        m_Config.SetReversed(false);
    }

    // Generate the trajectory.
    m_Trajectory = TrajectoryGenerator::GenerateTrajectory(vWaypoints, m_Config);
    
    // Setup the RamseteCommand with new trajectory.
    m_pRamseteCommand = new frc2::RamseteCommand(
        m_Trajectory, 
        [this]() { return GetRobotPose(); }, 
        RamseteController(kBeta, kZeta), 
        SimpleMotorFeedforward<meters>(kGainS, kGainV, kGainA), 
        kDriveKinematics, 
        [this]() { return GetWheelSpeeds(); }, 
        frc2::PIDController(kDriveProportional, kDriveIntegral, kDriveDerivative), 
        frc2::PIDController(kDriveProportional, kDriveIntegral, kDriveDerivative), 
        [this](auto left, auto right) { SetDrivePowers(left, right); }
    );

    // Go RamseteCommand!
    m_pRamseteCommand->Schedule();
}

/****************************************************************************
    Description:	FollowTrajectory - MotionProfiling method that follows
                    a pre-generated trajectory.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CDrive::FollowTrajectory()
{
    // Update RamseteCommand.
    m_pRamseteCommand->Execute();
}

/****************************************************************************
    Description:	SetDrivePowers - Method that sets the left and 
                    right drivetrain voltages.

    Arguments: 		dLeftVoltage - Left motor voltage.
                    dRightVoltage - Right motor voltage.

    Returns: 		Nothing
****************************************************************************/
void CDrive::SetDrivePowers(volt_t dLeftVoltage, volt_t dRightVoltage)
{
    // Set drivetrain voltages.
    m_LeftDriveMotor.SetVoltage(dLeftVoltage);
    m_RightDriveMotor.SetVoltage(-dRightVoltage);

    SmartDashboard::PutNumber("Left Voltage", double(dLeftVoltage));
    SmartDashboard::PutNumber("Right Voltage", double(dRightVoltage));
}

/****************************************************************************
    Description:	DriveToPresetPosition - Drive back to a preset position 
                                        by creating and following a trajectory 
                                        based on our current position.

    Arguments: 		None

    Returns: 	    Nothing
****************************************************************************/
void CDrive::GeneratePathFromCurrentPosition()
{
    // Create waypoints vector.
    vector<Pose2d> vWaypoints;

    // Build waypoints.
    vWaypoints.emplace_back(GetRobotPose());
    if (m_bUseUserDefinedPoint)
    {
        vWaypoints.emplace_back(m_TrajectoryConstants.GetStoredPoint());
    }
    else
    {
        for (Pose2d Point : m_TrajectoryConstants.kPresetPoints)
        {
            vWaypoints.emplace_back(Point);
        }
    }
    
    // Pass built waypoints vector to the drive class for trajectory generation.
    GenerateTrajectory(vWaypoints);
}

/****************************************************************************
    Description:	GetWheelSpeeds - Method that gets the wheel velocity 
                    in meters per second.

    Arguments: 		None

    Returns: 		DifferentialDriveWheelSpeeds - Drivetrain speeds.
****************************************************************************/
DifferentialDriveWheelSpeeds CDrive::GetWheelSpeeds()
{
    // Return wheel speeds.
    return {meters_per_second_t(m_pLeftDriveEncoder->GetRate() / 39.3701), meters_per_second_t(m_pRightDriveEncoder->GetRate() / 39.3701)};
}

/****************************************************************************
    Description:	GetIsTrajectoryFinished - Returns of the generated
                    trajectory has successfully reached its last point.

    Arguments: 		None

    Returns: 		bool bTrajectoryIsFinished
****************************************************************************/
bool CDrive::TrajectoryIsFinished()
{
    // Return if we have reached the end of the trajectory.
    return m_pRamseteCommand->IsFinished();
}

/****************************************************************************
    Description:	GetTotalTrajectoryTime - Returns the trajectory time.

    Arguments: 		None

    Returns: 		INT nTotalTime
****************************************************************************/
int CDrive::GetTotalTrajectoryTime()
{   
    // Return the total trajectory time.
    return int(m_Trajectory.TotalTime());
}

/****************************************************************************
    Description:	GetRobotPose - Return the Pose2d of the robot from the
                                odometry class.

    Arguments: 		None

    Returns: 		POSE2D m_pRobotPose
****************************************************************************/
Pose2d CDrive::GetRobotPose()
{
    // Get the robot pose..
    return m_pOdometry->GetPose();
}

/****************************************************************************
    Description:	SetSelectedTrajectory - Select trajectory for auto.

    Arguments: 		int nAutoState - The auto state.

    Returns: 		Nothing
****************************************************************************/
void CDrive::SetSelectedTrajectory(int nAutoState)
{
    // Get and generate new trajectory.
    m_TrajectoryConstants.SelectTrajectory(nAutoState);
    GenerateTrajectory(m_TrajectoryConstants.GetSelectedTrajectory());

    // Reset robot position.
    ResetOdometry();
}

/****************************************************************************
    Description:	StoreCurrentPosition - Store the robots 
                        current position as an endpoint for path generation.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CDrive::StoreCurrentPosition()
{
    m_TrajectoryConstants.SetStoredPoint(GetRobotPose());
}

/****************************************************************************
    Description:	ResetEncoders - Method that reset encoder values back 
                    to zero.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CDrive::ResetEncoders()
{
    // Reset drive encoders.
    m_pLeftDriveEncoder->Reset();
    m_pRightDriveEncoder->Reset();
}

/****************************************************************************
    Description:	Resetodometry - Reset the robot's position on field.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CDrive::ResetOdometry()
{
    // Reset the drive encoders.
    m_pLeftDriveEncoder->Reset();
    m_pRightDriveEncoder->Reset();

    // Reset robot position to start point of the selected trajectory.
    m_pOdometry->ResetPosition(m_TrajectoryConstants.GetSelectedTrajectoryStartPoint(), Rotation2d(degree_t(GetYaw())));
}

/****************************************************************************
    Description:	SetMotorSafety - Enables/Disables motor safety.

    Arguments: 		bool bSafetyEnabled - True if safety enabled,
                    false otherwise.

    Returns: 		Nothing
****************************************************************************/
void CDrive::SetMotorSafety(bool bSafetyEnabled)
{
    m_LeftDriveMotor.SetSafetyEnabled(bSafetyEnabled);
    m_RightDriveMotor.SetSafetyEnabled(bSafetyEnabled);
    m_RobotDrive.SetSafetyEnabled(bSafetyEnabled);
}

/****************************************************************************
    Description:	SetJoystickControl - Sets the desired joystick control.

    Arguments: 		bool bJoystickControl - True if joystick control enabled,
                    false otherwise.

    Returns: 		Nothing
****************************************************************************/
void CDrive::SetJoystickControl(bool bJoystickControl)
{
    m_bJoystickControl = bJoystickControl;
}

/****************************************************************************
    Description:	SetUseUserDefinePointForPathGeneration - Sets the desired joystick control.

    Arguments: 		bool bUseUserDefinedPoint - True if using user defined point.

    Returns: 		Nothing
****************************************************************************/
void CDrive::SetUseUserDefinePointForPathGeneration(bool bUseUserDefinePoint)
{
    m_bUseUserDefinedPoint = bUseUserDefinePoint;
}

/****************************************************************************
    Description:	GetYaw - Get the gyro yaw.

    Arguments: 		None

    Returns: 		double dYaw
****************************************************************************/
double CDrive::GetYaw()
{
    // Wrap gyro to -180 - 180.
    double dYaw = fmod(m_pGyro->GetAngleZ() + 180, 360);
    if (dYaw < 0)
    {
        dYaw += 360;
    }

    return -(dYaw - 180);
}