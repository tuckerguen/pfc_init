cmake_minimum_required(VERSION 2.8)
project( PFC_INIT )
set (CMAKE_CXX_STANDARD 11)

find_package(OpenCV 3 REQUIRED
COMPONENTS
    opencv_core
    opencv_highgui
    opencv_imgproc
    opencv_imgcodecs
    opencv_cudaarithm
    opencv_cudaimgproc
    opencv_cudafilters
    opencv_sfm
  PATHS /usr/local
  NO_DEFAULT_PATH
  CONFIG
)

include_directories(
  "../include/"
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

find_package (Threads)

add_executable(main main.cpp 
  pfc_initializer.cpp
  needle_pose.cpp 
  needle_image.cpp 
  needle_template.cpp 
  template_match.cpp
  pose_helper.cpp
  csv_reader.cpp
  matcher.cpp
)

target_link_libraries(main ${OpenCV_LIBS} Threads::Threads)