cmake_minimum_required( VERSION 2.8 )
PROJECT(test_mscnav)

add_executable(test_filter ./NavFilter.cc)
target_link_libraries( test_filter filter tools)