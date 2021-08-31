/****************************************************************************
    Description:	Defines the Poses used for autonomous.
    
    Classes:		CTrajectoryConstants

    Project:		2021 ROMI

    Copyright 2021 ROMI - Clayton Cowen
****************************************************************************/
#ifndef TrajectoryConstants_h
#define TrajectoryConstants_h

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/Trajectory.h>
#include <units/units.h>

using namespace frc;
using namespace units;
using namespace std;

enum TrajectoryList 
{
    eDoNothing,
    eTestPath1
};
/////////////////////////////////////////////////////////////////////////////

class CTrajectoryConstants
{
public:
    void SelectTrajectory(int nTrajectory)
    {
        // Retrieve the correct trajectory.
        switch(nTrajectory)
        {
            case eTestPath1 :
                m_StartPoint =
                {
                    0.0_in,					// X starting position on field in feet.
                    0.0_in,					// Y starting position on field in feet.
                    Rotation2d(0_deg)		// Starting rotation on field in degrees.
                };

                m_Waypoints =
                {
                    m_StartPoint,
                    Pose2d
                    {
                        2.0_in,					// X of point 1 on field in feet.
                        0.0_in,					// Y of point 1 on field in feet.
                        Rotation2d(0_deg)
                    },
                    Pose2d
                    {
                        10.0_in,				// X of point 2 on field in feet.
                        0.0_in,					// Y of point 2 on field in feet.
                        Rotation2d(0_deg)
                    },
                };

                m_bIsReversed = false;
                break;

            default :
                m_StartPoint =
                {
                    0.0_in,					// X starting position on field in feet.
                    0.0_in,					// Y starting position on field in feet.
                    Rotation2d(0_deg)		// Starting rotation on field in degrees.
                };

                m_Waypoints =
                {
                    m_StartPoint,
                    Pose2d
                    {
                        0.1_in,				    // X ending position on field in feet.
                        0.1_in,					// Y ending position on field in feet.
                        Rotation2d(0_deg)		// Ending rotation on field in degrees.
                    }
                };

                m_bIsReversed = false;
                break;
        }
    }

    // One-line methods.
    void SelectTrajectory(vector<Pose2d> vWaypoints)    {   m_Waypoints = vWaypoints;                   };
    void SetStoredPoint(Pose2d Point)                   {   m_StoredPoint = Point;                      };
    Pose2d GetSelectedTrajectoryStartPoint()            {   return m_StartPoint;                        };
    vector<Pose2d> GetSelectedTrajectory()              {   return m_Waypoints;                         };
    Pose2d GetStoredPoint()                             {   return m_StoredPoint;                       };
    bool GetIsTrajectoryReversed()                      {   return m_bIsReversed;                       };

    // Configure trajectory properties.
    const meters_per_second_t kMaxSpeed = 0.4_mps;       // 0.6604
    const meters_per_second_squared_t kMaxAcceleration = 0.4_mps_sq;      // 2

    // Preset Teleop Trajectory.
    const vector<Pose2d> kPresetPoints
    {
        
        Pose2d
        {
            12.0_in,				    // X ending position on field in feet.
            0.0_in,				        // Y ending position on field in feet.
            Rotation2d(0_deg)		    // Ending rotation on field in degrees.
        },
        Pose2d
        {
            36.0_in,
            15.0_in,
            Rotation2d(90_deg)
        }
    };

private:
    bool m_bIsReversed = false;

    Pose2d m_StartPoint
    {
        0.0_in,
        0.0_in,
        Rotation2d(0_deg)
    };

    vector<Pose2d> m_Waypoints
    {
        Pose2d
        {
            0.1_in,
            0.1_in,
            Rotation2d(0_deg)
        },
        Pose2d
        {
            0.2_in,
            0.2_in,
            Rotation2d(0_deg)
        }
    };

    Pose2d m_StoredPoint
    {
        24.0_in,
        0.0_in,
        Rotation2d(0_deg)
    };
};
/////////////////////////////////////////////////////////////////////////////
#endif