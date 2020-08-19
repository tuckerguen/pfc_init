cmake_minimum_required(VERSION 2.8)
project( PFC_INIT )
set (CMAKE_CXX_STANDARD 11)

set(OpenCV_INCLUDE_DIRS
  /usr/local/include
  /usr/local/include/opencv2
)

set(OpenCV_LIB_DIR
  /usr/local/lib
)

set(OpenCV_LIBS
  opencv_core
  opencv_highgui
  opencv_imgcodecs
  opencv_imgproc
  opencv_features2d
  opencv_cudaimgproc
  opencv_sfm
)


include_directories("../include/")
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

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

add_executable(NeedleGenerator NeedleGeneration/NeedleGenerator.cpp)

target_link_libraries(main ${OpenCV_LIBS})
target_link_libraries (main Threads::Threads)

target_link_libraries(NeedleGenerator ${OpenCV_LIBS})

