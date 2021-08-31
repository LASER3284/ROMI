/****************************************************************************
    Description:	Defines the RomiGyro utility.

    Classes:		CRomiGyro

    Project:		2021 ROMI

    Copyright 2021 ROMI - Clayton Cowen
****************************************************************************/
#ifndef RomiGyro_h
#define RomiGyro_h

#include <hal/SimDevice.h>
/////////////////////////////////////////////////////////////////////////////


/******************************************************************************
    Description:	CRomiGyro class definition.

    Arguments:		None

    Derived From:	Nothing
******************************************************************************/
class CRomiGyro 
{
	public:
		CRomiGyro();
		~CRomiGyro();
		double GetRateX();
		double GetRateY();
		double GetRateZ();
		double GetAngleX();
		double GetAngleY();
		double GetAngleZ();
		void Reset();

	private:
		hal::SimDevice m_simDevice;
		hal::SimDouble m_simRateX;
		hal::SimDouble m_simRateY;
		hal::SimDouble m_simRateZ;
		hal::SimDouble m_simAngleX;
		hal::SimDouble m_simAngleY;
		hal::SimDouble m_simAngleZ;

		double m_dAngleXOffset = 0;
		double m_dAngleYOffset = 0;
		double m_dAngleZOffset = 0;
};
/////////////////////////////////////////////////////////////////////////////
#endif