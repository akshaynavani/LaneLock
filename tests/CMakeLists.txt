include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
# Keep gtest from overriding the parent's runtime library settings on Windows.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# Fast unit tests for individual components.
add_executable(lanelock_unit_tests
  test_math.cpp
  test_road.cpp
  test_vehicle.cpp
  test_sensors.cpp
  test_actuator.cpp
  test_pid.cpp
  test_controller.cpp
  test_metrics_io.cpp
)
target_link_libraries(lanelock_unit_tests PRIVATE lanelock::core GTest::gtest_main)
gtest_discover_tests(lanelock_unit_tests TEST_PREFIX "unit." DISCOVERY_TIMEOUT 60)

# Closed-loop behaviour tests and the 500+ scenario regression suite.
add_executable(lanelock_regression_tests
  test_closed_loop.cpp
  test_regression.cpp
)
target_link_libraries(lanelock_regression_tests PRIVATE lanelock::core GTest::gtest_main)
target_compile_definitions(lanelock_regression_tests PRIVATE
  LANELOCK_DEFAULT_BASELINE="${PROJECT_SOURCE_DIR}/baselines/metrics_baseline.csv")
gtest_discover_tests(lanelock_regression_tests TEST_PREFIX "regression." DISCOVERY_TIMEOUT 120)
