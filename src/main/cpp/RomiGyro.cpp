/****************************************************************************
    Description:	Implements the CDrive control class.

    Classes:		CDrive

    Project:		2021 ROMI

    Copyright 2021 ROMI - Clayton Cowen
****************************************************************************/
#include "RomiGyro.h"
#include "IOMap.h"

///////////////////////////////////////////////////////////////////////////////


/****************************************************************************
    Description:	CRomiGyro Constructor.

    Arguments:		None

    Derived From:	Nothing
****************************************************************************/
CRomiGyro::CRomiGyro() : m_simDevice("Gyro:RomiGyro") 
{
  if (m_simDevice) 
  {
    m_simDevice.CreateBoolean("init", hal::SimDevice::kOutput, true);
    m_simRateX = m_simDevice.CreateDouble("rate_x", hal::SimDevice::kInput, 0.0);
    m_simRateY = m_simDevice.CreateDouble("rate_y", hal::SimDevice::kInput, 0.0);
    m_simRateZ = m_simDevice.CreateDouble("rate_z", hal::SimDevice::kInput, 0.0);
    m_simAngleX = m_simDevice.CreateDouble("angle_x", hal::SimDevice::kInput, 0.0);
    m_simAngleY = m_simDevice.CreateDouble("angle_y", hal::SimDevice::kInput, 0.0);
    m_simAngleZ = m_simDevice.CreateDouble("angle_z", hal::SimDevice::kInput, 0.0);
  }
}

/****************************************************************************
    Description:	CRomiGyro Destructor.

    Arguments:		None

    Derived From:	Nothing
****************************************************************************/
CRomiGyro::~CRomiGyro()
{

}

/****************************************************************************
    Description:	Get gyro rate X.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
double CRomiGyro::GetRateX() 
{
  if (m_simRateX) 
  {
    return m_simRateX.Get();
  }

  return 0.0;
}

/****************************************************************************
    Description:	Get gyro rate Y.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
double CRomiGyro::GetRateY() 
{
  if (m_simRateY) 
  {
    return m_simRateY.Get();
  }

  return 0.0;
}

/****************************************************************************
    Description:	Get gyro rate Z.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
double CRomiGyro::GetRateZ() 
{
  if (m_simRateZ) 
  {
    return m_simRateZ.Get();
  }

  return 0.0;
}

/****************************************************************************
    Description:	Get gyro angle X.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
double CRomiGyro::GetAngleX() 
{
  if (m_simAngleX) 
  {
    return m_simAngleX.Get() - m_dAngleXOffset;
  }

  return 0.0;
}

/****************************************************************************
    Description:	Get gyro angle Y.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
double CRomiGyro::GetAngleY() 
{
  if (m_simAngleY) 
  {
    return m_simAngleY.Get() - m_dAngleYOffset;
  }

  return 0.0;
}

/****************************************************************************
    Description:	Get gyro angle Z.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
double CRomiGyro::GetAngleZ() 
{
  if (m_simAngleZ) 
  {
    return m_simAngleZ.Get() - m_dAngleZOffset;
  }

  return 0.0;
}

/****************************************************************************
    Description:	Reset the gyro.

    Arguments: 		None

    Returns: 		Nothing
****************************************************************************/
void CRomiGyro::Reset() 
{
  if (m_simAngleX) 
  {
    m_dAngleXOffset = m_simAngleX.Get();
    m_dAngleYOffset = m_simAngleY.Get();
    m_dAngleZOffset = m_simAngleZ.Get();
  }
}
