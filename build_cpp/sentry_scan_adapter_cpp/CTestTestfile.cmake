# CMake generated Testfile for 
# Source directory: /home/hzq/nav/adapt_VI_26_Sentry/src/sentry_scan_adapter_cpp
# Build directory: /home/hzq/nav/adapt_VI_26_Sentry/build_cpp/sentry_scan_adapter_cpp
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_cmd_gate_logic "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/hzq/nav/adapt_VI_26_Sentry/build_cpp/sentry_scan_adapter_cpp/test_results/sentry_scan_adapter_cpp/test_cmd_gate_logic.gtest.xml" "--package-name" "sentry_scan_adapter_cpp" "--output-file" "/home/hzq/nav/adapt_VI_26_Sentry/build_cpp/sentry_scan_adapter_cpp/ament_cmake_gtest/test_cmd_gate_logic.txt" "--command" "/home/hzq/nav/adapt_VI_26_Sentry/build_cpp/sentry_scan_adapter_cpp/test_cmd_gate_logic" "--gtest_output=xml:/home/hzq/nav/adapt_VI_26_Sentry/build_cpp/sentry_scan_adapter_cpp/test_results/sentry_scan_adapter_cpp/test_cmd_gate_logic.gtest.xml")
set_tests_properties(test_cmd_gate_logic PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/hzq/nav/adapt_VI_26_Sentry/build_cpp/sentry_scan_adapter_cpp/test_cmd_gate_logic" TIMEOUT "60" WORKING_DIRECTORY "/home/hzq/nav/adapt_VI_26_Sentry/build_cpp/sentry_scan_adapter_cpp" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/hzq/nav/adapt_VI_26_Sentry/src/sentry_scan_adapter_cpp/CMakeLists.txt;57;ament_add_gtest;/home/hzq/nav/adapt_VI_26_Sentry/src/sentry_scan_adapter_cpp/CMakeLists.txt;0;")
subdirs("gtest")
