cmake_minimum_required( VERSION 2.8 )
PROJECT(test_mscnav)

add_executable(SimDataProcess ./SimDataProcess.cc)
target_link_libraries( SimDataProcess tools)

add_executable(RefProcess ./RefProcess.cc)
target_link_libraries( RefProcess tools)