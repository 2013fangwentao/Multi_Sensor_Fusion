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

add_executable(test_image ./NavImage.cc)
target_link_libraries( test_image camera tools process filter ${OpenCV_LIBS} ${GLOG_LIBRARY})


add_executable(NavTestCamera ./NavTestCamera.cc)
target_link_libraries( NavTestCamera tools ${GLOG_LIBRARY}  ${OpenCV_LIBS})

add_executable(trianglePoints ./trianglePoints.cc)
target_link_libraries( trianglePoints tools ${GLOG_LIBRARY}  ${OpenCV_LIBS})
