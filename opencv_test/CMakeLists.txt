cmake_minimum_required(VERSION 2.8)
project( webcam_test )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

#add_executable(webcam_test
#  webcam_test.cpp
#  )

add_executable(image_scan_test
  image_scan_test.cpp
  )

# target_link_libraries( webcam_test ${OpenCV_LIBS} )
target_link_libraries( image_scan_test ${OpenCV_LIBS} )