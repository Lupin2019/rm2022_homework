cmake_minimum_required( VERSION 3.15 )

project(homework_1_01)

find_package( OpenCV 4 REQUIRED )

include_directories ( ${OpenCV_INCLUDE_DIR} )

add_executable( homework_1_01 homework_1_01.cpp )
target_link_libraries( homework_1_01 ${OpenCV_LIBS})


