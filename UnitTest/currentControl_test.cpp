#include <gtest/gtest.h>

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