#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "..\CurrentControl\simpleCurrentControl.h"
#include "..\CurrentControl\hardwareInterface.h"
#include "..\UnitTest\hardwareInterfaceMock.h"

using ::testing::_;
using ::testing::AtLeast;
using ::testing::InvokeWithoutArgs;
using ::testing::Return;

// Test for currentControlLoop function
ParkAndClarke_t testStruct;

TEST(currentControl_test, Test1) {
    HardwareInterfaceMock * mockHardwareInterface = new HardwareInterfaceMock();
    SimpleCurrentControl *currentControl = new SimpleCurrentControl(mockHardwareInterface);
    // test input values
    int32_t testInputs[3][4] = {
        {8192, 2048, 2048, 0},
        {8192, 2598, 2323, 1100},
        {8192, 2598, 2323, 1100}
    };

    EXPECT_CALL(*mockHardwareInterface, readCurrentsFromADC)
        .Times(AtLeast(1))
        .WillRepeatedly([](int32_t *currADC1, int32_t *currADC2) {
            *currADC1 = 2048; // Simulate ADC reading for U phase
            *currADC2 = 2048; // Simulate ADC reading for V phase
        });
    EXPECT_CALL(*mockHardwareInterface, setDutyUVW)
        .Times(AtLeast(1));
    EXPECT_CALL(*mockHardwareInterface, motorElectricalAngle)
        .Times(AtLeast(100))
        .WillRepeatedly(Return(0)); // Simulate a fixed electrical angle for subsequent calls
    ;

    // initialise the control loop
    currentControl->currentControlLoop (0, &testStruct, CC_MODE_RESET);

    // run the control loop several times
    for (int i = 0; i < 3; i++) {
        currentControl->currentControlLoop (testInputs[i][4], &testStruct, CC_MODE_Q_AXIS);
    }

}