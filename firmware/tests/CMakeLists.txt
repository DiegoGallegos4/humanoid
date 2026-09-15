include(GoogleTest)

add_executable(hal_sim_test hal_sim_test.cpp)
target_link_libraries(hal_sim_test PRIVATE
    nori_core nori_hal nori_hal_sim GTest::gtest_main)
gtest_discover_tests(hal_sim_test)

add_executable(control_loop_test control_loop_test.cpp)
target_link_libraries(control_loop_test PRIVATE
    nori_control nori_hal nori_hal_sim GTest::gtest_main)
gtest_discover_tests(control_loop_test)

add_executable(protocol_test protocol_test.cpp)
target_link_libraries(protocol_test PRIVATE
    nori_protocol nori_hal nori_hal_sim GTest::gtest_main)
gtest_discover_tests(protocol_test)
