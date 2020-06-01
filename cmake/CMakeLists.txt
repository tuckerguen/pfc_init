cmake_minimum_required(VERSION 2.8)
project( OpenCV_Practice )

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
)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

add_executable(pfc_init ../src/pfc_init.cpp)
add_executable(TestCannyThreshold ../src/TestCannyThreshold.cpp )
target_link_libraries(TestCannyThreshold ${OpenCV_LIBS})
target_link_libraries(pfc_init ${OpenCV_LIBS})
