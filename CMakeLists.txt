cmake_minimum_required(VERSION 3.10)
project(TestCurrentControl CXX)

# Option A: Hard-code the vcpkg toolchain file here
set(CMAKE_TOOLCHAIN_FILE "C:/msys64/mingw64/bin/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "" FORCE)
# Option B: Omit the line above and specify via command line

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mconsole")

add_executable(currentControl CurrentControl/simpleCurrentControl.cpp)

find_package(GTest CONFIG REQUIRED)

add_executable(currentControl_test UnitTest/currentControl_test.cpp)
target_link_libraries(currentControl_test PRIVATE GTest::gtest GTest::gtest_main)

enable_testing()
#add_test(NAME currentControl_test COMMAND currentControl_test)

include(GoogleTest)
gtest_discover_tests(currentControl_test)  # Automatically discover tests