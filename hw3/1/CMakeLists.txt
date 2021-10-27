cmake_minimum_required( VERSION 3.15 )

project(taxi)

find_package( OpenCV 4 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIR} )

add_executable( main main.cpp )
target_link_libraries( main ${OpenCV_LIBS} )
