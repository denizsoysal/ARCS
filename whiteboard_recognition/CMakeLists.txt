cmake_minimum_required(VERSION 2.8)
project( webcam_test )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )




# add_executable(webcam_test
#  webcam_test.cpp
#  )

# add_executable(combination
#   combination.cpp
#   )

  # add_executable(histogram
  # histogram.cpp
  # )

  # add_executable(threshold
  # threshold.cpp
  # )

  add_executable(closing
  closing.cpp
  )

  add_executable(closing_adaptive
  closing_adaptive.cpp
  )
  
    add_executable(template_matching
  template_matching.cpp
  )

#   add_executable(edge_detection
#   edge_detection.cpp
#   )

  # add_executable(cropping
  # cropping.cpp
  # )

  # add_executable(watershed
  # watershed.cpp
  # )

  # add_executable(correlation
  # correlation.cpp
  # )

  add_executable(region_growing
  region_growing.cpp
  )





# target_link_libraries( webcam_test ${OpenCV_LIBS} )
# target_link_libraries( combination ${OpenCV_LIBS} )
# target_link_libraries( histogram ${OpenCV_LIBS} )
# target_link_libraries( threshold ${OpenCV_LIBS} )
target_link_libraries( closing ${OpenCV_LIBS} )
# target_link_libraries( edge_detection ${OpenCV_LIBS} )
# target_link_libraries( cropping ${OpenCV_LIBS} )
# target_link_libraries( watershed ${OpenCV_LIBS} )
# target_link_libraries( correlation ${OpenCV_LIBS} )
target_link_libraries( region_growing ${OpenCV_LIBS} )


target_link_libraries( closing_adaptive ${OpenCV_LIBS} )

target_link_libraries( template_matching ${OpenCV_LIBS} )

