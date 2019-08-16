cmake_minimum_required( VERSION 2.8 )
PROJECT(mscnav_loose_couple)

add_executable(${PROJECT_NAME} ./NavProcessing.cc)
target_link_libraries( ${PROJECT_NAME} process filter data imu tools)
