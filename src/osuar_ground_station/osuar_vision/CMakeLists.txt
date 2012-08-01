cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(vision src/vision.cpp)
rosbuild_add_executable(squares src/squares.cpp)
rosbuild_add_executable(hough src/hough.cpp)
rosbuild_add_executable(blob src/blob.cpp)

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIR})
    #target_link_libraries(vision ${OpenCV_LIBS})
    target_link_libraries(squares ${OpenCV_LIBS})
    target_link_libraries(hough ${OpenCV_LIBS})
    target_link_libraries(blob ${OpenCV_LIBS})
endif (OpenCV_FOUND)

# We are currently using an old version of cvblobs from here:
# http://opencv.willowgarage.com/wiki/cvBlobsLib
# We eventually need to use the cvblobs library from here instead:
# http://code.google.com/p/cvblob/
#find_package(cvBlob REQUIRED)
#if (cvBlob_FOUND)
#    target_link_libraries(vision ${cvBlob_LIBS})
#endif (cvBlob_FOUND)

#rosbuild_add_library(cvblob include/cvblobs/blob.cpp
#                            include/cvblobs/BlobContour.cpp
#                            include/cvblobs/BlobOperators.cpp
#                            include/cvblobs/BlobProperties.cpp
#                            include/cvblobs/BlobResult.cpp
#                            include/cvblobs/ComponentLabeling.cpp)
