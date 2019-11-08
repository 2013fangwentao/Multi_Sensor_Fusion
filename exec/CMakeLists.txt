cmake_minimum_required( VERSION 2.8 )
PROJECT(mscnav_bin)

add_executable(mscnav_bin ./mscnav.cc)
target_link_libraries( mscnav_bin process filter data imu tools)