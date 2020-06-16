# cmake_minimum_required(VERSION 2.8)
# project( OpenCV_Practice )

# set(OpenCV_INCLUDE_DIRS
#         /home/tucker/Downloads/opencv-3.2.0_1/include
#         /home/tucker/Downloads/opencv-3.2.0_1/include/opencv2
# )

# set(OpenCV_LIB_DIR
#         /home/tucker/Downloads/opencv-3.2.0_1/build/lib
# )

# # set(OpenCV_INCLUDE_DIRS
# #         /usr/local/include
# #         /usr/local/include/opencv2
# # )

# # set(OpenCV_LIB_DIR
# #         /usr/local/lib
# # )

# set(OpenCV_LIBS
#         opencv_core
#         opencv_highgui
#         opencv_imgcodecs
#         opencv_imgproc
#         opencv_features2d
#         opencv_cudaimgproc
#         opencv_sfm
# )



# include_directories(${OpenCV_INCLUDE_DIRS})
# link_directories(${OpenCV_LIB_DIR})

# add_executable(main main.cpp PfcInit.cpp NeedlePose.cpp NeedleImage.cpp NeedleTemplate.cpp TemplateMatch.cpp)
# #add_executable(TestCannyThreshold ../src/TestCannyThreshold.cpp )
# # add_executable(batch_init ../src/batch_init.cpp)
# #add_executable(manual_localize ../src/manual_localize.cpp)

# # add_executable(NeedleMatch ../src/old/NeedleMatch.cpp)

# # target_link_libraries(NeedleMatch ${OpenCV_LIBS})
# #target_link_libraries(manual_localize ${OpenCV_LIBS})
# # target_link_libraries(batch_init ${OpenCV_LIBS} pfc_init)
# #target_link_libraries(TestCannyThreshold ${OpenCV_LIBS})
# target_link_libraries(main ${OpenCV_LIBS})

cmake_minimum_required(VERSION 2.8)
project( OpenCV_Practice )
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

# add_library(pfc_init
#   ../src/pfc_init.cpp
# )

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

# add_executable()
add_executable(main main.cpp PfcInit.cpp NeedlePose.cpp NeedleImage.cpp NeedleTemplate.cpp TemplateMatch.cpp)
target_link_libraries(main ${OpenCV_LIBS})


