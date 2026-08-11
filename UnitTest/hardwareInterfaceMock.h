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