#include <gtest/gtest.h>
#include "..\CurrentControl\simpleCurrentControl.h"

// Sample function to test
int add(int a, int b) {
    return a + b;
}

TEST(currentControl_test, TestAdd) {
    EXPECT_EQ(add(2, 2), 4);
    EXPECT_EQ(add(-1, 1), 0);
}

TEST(currentControl_test, AnotherTest) {
    EXPECT_TRUE(true);
}

TEST(currentControl_test, FailingTest) {
    EXPECT_EQ(add(2, 2), 5); // This test is expected to fail
}

// Test for currentControlLoop function
ParkAndClarke_t testStruct;

TEST(currentControl_test, Test1) {
      currentControlLoop (0, &testStruct, CC_MODE_RESET);
}