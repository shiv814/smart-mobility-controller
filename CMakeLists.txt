cmake_minimum_required(VERSION 3.16)
project(SmartMobilityController VERSION 3.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

function(enable_strict_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
endfunction()

add_library(mobility
    src/controller.cpp
    src/arbitration.cpp
    src/energy.cpp
    src/odometry.cpp
    src/diagnostics.cpp
)
target_include_directories(mobility PUBLIC src)
enable_strict_warnings(mobility)

add_executable(mobility_simulator src/simulator.cpp)
target_link_libraries(mobility_simulator PRIVATE mobility)
enable_strict_warnings(mobility_simulator)

add_executable(mobility_v3_demo src/v3_demo.cpp)
target_link_libraries(mobility_v3_demo PRIVATE mobility)
enable_strict_warnings(mobility_v3_demo)

add_executable(controller_tests tests/test_controller.cpp)
target_link_libraries(controller_tests PRIVATE mobility)
enable_strict_warnings(controller_tests)

add_executable(safety_v3_tests tests/test_safety_v3.cpp)
target_link_libraries(safety_v3_tests PRIVATE mobility)
enable_strict_warnings(safety_v3_tests)

enable_testing()
add_test(NAME controller_tests COMMAND controller_tests)
add_test(NAME safety_v3_tests COMMAND safety_v3_tests)
add_test(NAME mobility_v3_demo_smoke COMMAND mobility_v3_demo)

install(TARGETS mobility mobility_simulator mobility_v3_demo)
