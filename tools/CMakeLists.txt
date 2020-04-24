cmake_minimum_required( VERSION 2.8 )
project(little_tools)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


add_executable(CoorTran ./CoorTransformation.cc)
target_link_libraries(CoorTran tools ${GFLAGS_LIBRARIES})
add_executable(CoorTranFile ./CoorTransformationFile.cc)
target_link_libraries(CoorTranFile tools)

add_executable(TimeTran ./TimeTransformation.cc)
target_link_libraries(TimeTran tools ${GFLAGS_LIBRARIES})

add_executable(ImuTran ./RawImuToImuData.cc)
target_link_libraries(ImuTran tools ${GFLAGS_LIBRARIES})

add_executable(PreprocessKitti ./PreprocessKittiData.cc)
target_link_libraries(PreprocessKitti tools)

add_executable(ImageUndistort ./ImageUndistort.cc)
target_link_libraries(ImageUndistort tools ${OpenCV_LIBS} ${GLOG_LIBRARY})

add_executable(picture_preprocess ./picture_preprocess.cc)
target_link_libraries(picture_preprocess tools)

add_executable(PrintKeypoints ./PrintKeypoints.cc)
target_link_libraries(PrintKeypoints tools ${OpenCV_LIBS})

add_executable(calibration ./calibration.cc)
target_link_libraries(calibration ${OpenCV_LIBS})

add_executable(ImagePreview ./ImagePreview.cc)
target_link_libraries(ImagePreview tools ${OpenCV_LIBS})
