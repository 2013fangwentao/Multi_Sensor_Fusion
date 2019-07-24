cmake_minimum_required( VERSION 2.8 )
PROJECT(test_mscnav)

add_executable(test_filter ./NavFilter.cc)
target_link_libraries( test_filter filter tools)

add_executable(test_mech ./NavPureMech.cc)
target_link_libraries( test_mech imu tools)

add_executable(test_data_queue ./NavdataQueue.cc)
target_link_libraries( test_data_queue data imu tools)