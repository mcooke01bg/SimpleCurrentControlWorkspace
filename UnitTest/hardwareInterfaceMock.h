#ifndef __HARDWAREINTERFACEMOCK_H
#define __HARDWAREINTERFACEMOCK_H
/*==================================================================================================
COPYRIGHT 2026 MATIMAR LTD. ALL RIGHTS RESERVED

This software is protected by copyright and intellectual property laws in Bulgaria and other
countries. The software may be licensed and is not sold to you. You may use the software only
in accordance with the terms of the license agreement you entered into with Matimar Ltd.

==================================================================================================*/

/*==================================================================================================
  FILE DESCRIPTION

  Contains a mock implementation of the HardwareInterface for unit testing. Uses Google Mock 
  framework to create mock methods for the HardwareInterface class.

===================================================================================================*/

/*==================================================================================================
  DEPENDENCIES
===================================================================================================*/
#include "gmock/gmock.h"
#include "..\CurrentControl\hardwareInterface.h"

class HardwareInterfaceMock : public HardwareInterface {
    public:
        HardwareInterfaceMock() = default;
        ~HardwareInterfaceMock() override = default;
        MOCK_METHOD(void, readCurrentsFromADC, (int32_t *currADC1, int32_t *currADC2), (override));
        MOCK_METHOD(void, setDutyUVW, (uint32_t dutyPhaseA, uint32_t dutyPhaseB, uint32_t dutyPhaseC), (override));
        MOCK_METHOD(uint16_t, motorElectricalAngle, (), (override));
};
#endif /* __HARDWAREINTERFACEMOCK_H */