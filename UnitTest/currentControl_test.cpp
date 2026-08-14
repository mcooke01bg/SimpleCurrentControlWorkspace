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
ParkAndClarke_t testStruct = {0};

TEST(currentControl_test, Test1) {

    int i = 0;

    HardwareInterfaceMock mockHardwareInterface;

    // test input values
    int32_t testInputs[3][4] = {
        {8192, 2048, 2048, 0},
        {8192, 2598, 2323, 1100},
        {8192, 2598, 2323, 1100}
    };
    
    EXPECT_CALL(mockHardwareInterface, readCurrentsFromADC)
        .Times(4)
        .WillOnce([](int32_t *currADC1, int32_t *currADC2) {
            *currADC1 = 2048; // Simulate ADC reading for U phase
            *currADC2 = 2048; // Simulate ADC reading for V phase
        })
        .WillOnce([](int32_t *currADC1, int32_t *currADC2) {
            *currADC1 = 2048; // Simulate ADC reading for U phase
            *currADC2 = 2048; // Simulate ADC reading for V phase
        })
        .WillOnce([](int32_t *currADC1, int32_t *currADC2) {
            *currADC1 = 2048; // Simulate ADC reading for U phase
            *currADC2 = 2048; // Simulate ADC reading for V phase
        })        
        .WillOnce([](int32_t *currADC1, int32_t *currADC2) {
            *currADC1 = 2048; // Simulate ADC reading for U phase
            *currADC2 = 2048; // Simulate ADC reading for V phase
        });

    EXPECT_CALL(mockHardwareInterface, setDutyUVW (1984,1984,1984))
        .Times(3);
    EXPECT_CALL(mockHardwareInterface, setDutyUVW (1983,1985,1985))
        .Times(1);
    EXPECT_CALL(mockHardwareInterface, setDutyUVW (1983,1986,1986))
        .Times(1);

    EXPECT_CALL(mockHardwareInterface, motorElectricalAngle)
        .Times(4)
        .WillRepeatedly(Return(16384)); // Simulate a fixed electrical angle for subsequent calls
    ;

    SimpleCurrentControl *currentControl = new SimpleCurrentControl(&mockHardwareInterface);

    // initialise the control loop
    currentControl->currentControlLoop (0, &testStruct, CC_MODE_RESET);

    // run the control loop several times
    //for (int i = 0; i < 3; i++) {}
    currentControl->currentControlLoop (0, &testStruct, CC_MODE_Q_AXIS);
        EXPECT_EQ(testStruct.iU, 0);
        EXPECT_EQ(testStruct.iV, 0);
        EXPECT_EQ(testStruct.iAlpha, 0);
        EXPECT_EQ(testStruct.iBeta, 0);
        EXPECT_EQ(testStruct.iQ, 0);
        EXPECT_EQ(testStruct.iD, 0);
        EXPECT_EQ(testStruct.voltD, 0);
        EXPECT_EQ(testStruct.voltQ, 0);
        EXPECT_EQ(testStruct.voltAlpha, 0);
        EXPECT_EQ(testStruct.voltBeta, 0);
        EXPECT_EQ(testStruct.sector, 2);
        EXPECT_EQ(testStruct.wX, 0);
        EXPECT_EQ(testStruct.wY, 0);
        EXPECT_EQ(testStruct.wZ, 0);
        EXPECT_EQ(testStruct.sineValue, 32767);
    currentControl->currentControlLoop (110, &testStruct, CC_MODE_Q_AXIS);
        EXPECT_EQ(testStruct.iU, 0);
        EXPECT_EQ(testStruct.iV, 0);
        EXPECT_EQ(testStruct.iAlpha, 0);
        EXPECT_EQ(testStruct.iBeta, 0);
        EXPECT_EQ(testStruct.iQ, 0);
        EXPECT_EQ(testStruct.iD, 0);
        EXPECT_EQ(testStruct.voltD, 0);
        EXPECT_EQ(testStruct.voltQ, 29);
        EXPECT_EQ(testStruct.voltAlpha, -28);
        EXPECT_EQ(testStruct.voltBeta, 0);
        EXPECT_EQ(testStruct.sector, 4);
        EXPECT_EQ(testStruct.wX, 0);
        EXPECT_EQ(testStruct.wY, -384874);
        EXPECT_EQ(testStruct.wZ, 384874);
        EXPECT_EQ(testStruct.sineValue, 32767);
    currentControl->currentControlLoop (110, &testStruct, CC_MODE_Q_AXIS);
        EXPECT_EQ(testStruct.iU, 0);
        EXPECT_EQ(testStruct.iV, 0);
        EXPECT_EQ(testStruct.iAlpha, 0);
        EXPECT_EQ(testStruct.iBeta, 0);
        EXPECT_EQ(testStruct.iQ, 0);
        EXPECT_EQ(testStruct.iD, 0);
        EXPECT_EQ(testStruct.voltD, 0);
        EXPECT_EQ(testStruct.voltQ, 32);
        EXPECT_EQ(testStruct.voltAlpha, -31);
        EXPECT_EQ(testStruct.voltBeta, 0);
        EXPECT_EQ(testStruct.sector, 4);
        EXPECT_EQ(testStruct.wX, 0);
        EXPECT_EQ(testStruct.wY, -426110);
        EXPECT_EQ(testStruct.wZ, 426110);
        EXPECT_EQ(testStruct.sineValue, 32767);
}