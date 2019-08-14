cmake_minimum_required( VERSION 2.8 )
PROJECT(test_mscnav)

add_executable(test_filter ./NavFilter.cc)
target_link_libraries( test_filter filter tools)

add_executable(test_mech ./NavPureMech.cc)
target_link_libraries( test_mech imu tools)

add_executable(test_data_queue ./NavdataQueue.cc)
target_link_libraries( test_data_queue data imu tools)

add_executable(test_data_align ./NavAlign.cc)
target_link_libraries( test_data_align data imu tools)

add_executable(test_processing ./NavProcessing.cc)
target_link_libraries( test_processing process filter data imu tools)

add_executable(TimeConvert ./NavTime.cc)
target_link_libraries( TimeConvert tools)

add_executable(test_image ./NavImage.cc)
target_link_libraries( test_image orbfeature camera tools process filter ${OpenCV_LIBS})

find_package(gflags REQUIRED)
INCLUDE_DIRECTORIES(${gflags_INCLUDE_DIRS})
add_executable(CoorTarn ./NavCoorChange.cc)
target_link_libraries(CoorTarn tools)