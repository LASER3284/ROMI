/****************************************************************************
    Description:	Defines the robot I/O map.

    Classes:		None

    Project:		2021 ROMI

    Copyright 2021 ROMI - Clayton Cowen
****************************************************************************/
#ifndef IOMap_h
#define IOMap_h
/////////////////////////////////////////////////////////////////////////////

// PWM Channels.
const int nLeftDriveMotorChannel					=	0;		// PWM channel for the left drive motor.
const int nRightDriveMotorChannel                   =   1;      // PWM channel for the right drive motor.
const int nLiftServoChannel                         =   2;      // PWM channel for the lift servo motor.
const int nTiltServoChannel                         =   3;      // PWM channel for the tilt servo motor.
const int nPinchServoChannel                        =   4;      // PWM channel for the pitch servo motor.


// Relay Channels.

// Analog Channels.

// Digital Channels.
const int nLeftDriveEncoderChannelA			        =	4;		// Encoder clock A signal for left drive encoder.
const int nLeftDriveEncoderChannelB			        =	5;		// Encoder clock B signal for left drive encoder.
const int nRightDriveEncoderChannelA			    =	6;		// Encoder clock A signal for right drive encoder.
const int nRightDriveEncoderChannelB			    =	7;		// Encoder clock B signal for right drive encoder.

// Xbox Controller Button Assignments.
enum XboxButtons 		{eButtonA = 1, eButtonB, eButtonX, eButtonY, eButtonLB, eButtonRB, eBack, eStart, eButtonLS, eButtonRS};
// Xbox Controller Axis Assignments.
enum XboxAxis			{eLeftAxisX = 0, eLeftAxisY, eRightAxisX, eRightAxisY, eLeftTrigger, eRightTrigger};
// Logitech Flight Stick Button Assignments.
enum LogButtons	 		{eButtonTrigger = 1, eButton2, eButton3, eButton4, eButton5, eButton6, eButton7, eButton8, eButton9, eButton10, eButton11, eButton12};
// Shared Robot states for Motion.
enum State {eIdle, eHomingReverse, eHomingForward, eFinding, eManualForward, eManualReverse};
/////////////////////////////////////////////////////////////////////////////
#endif