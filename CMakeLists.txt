cmake_minimum_required(VERSION 3.10)
project(TestCurrentControl CXX)

# Option A: Hard-code the vcpkg toolchain file here
set(CMAKE_TOOLCHAIN_FILE "C:/msys64/mingw64/bin/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "" FORCE)
# Option B: Omit the line above and specify via command line

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# force linking with console
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -mconsole")
# For Windows: Prevent overriding the parent project's compiler/linker settings
# set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

add_library(currentControl)
target_sources(currentControl 
    PRIVATE CurrentControl/simpleCurrentControl.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS CurrentControl
        FILES CurrentControl/currentControl.h)

find_package(GTest CONFIG REQUIRED)

add_executable(currentControl_test)
target_sources(currentControl_test 
    PRIVATE 
        UnitTest/currentControl_test.cpp)
target_link_libraries(currentControl_test PRIVATE GTest::gtest GTest::gtest_main GTest::gmock_main currentControl)

enable_testing()
#add_test(NAME currentControl_test COMMAND currentControl_test)

include(GoogleTest)
gtest_discover_tests(currentControl_test)  # Automatically discover tests