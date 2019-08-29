cmake_minimum_required( VERSION 2.8 )
project(little_tools)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


add_executable(CoorTran ./CoorTransformation.cc)
target_link_libraries(CoorTran tools ${GFLAGS_LIBRARIES})

add_executable(TimeTran ./TimeTransformation.cc)
target_link_libraries(TimeTran tools ${GFLAGS_LIBRARIES})

add_executable(ImuTran ./RawImuToImuData.cc)
target_link_libraries(ImuTran tools ${GFLAGS_LIBRARIES})

add_executable(PreprocessKitti ./PreprocessKittiData.cc)
target_link_libraries(PreprocessKitti tools)
