cmake_minimum_required( VERSION 3.15 )

project(homework_1_02&03)

find_package( OpenCV 4 REQUIRED )

include_directories ( ${OpenCV_INCLUDE_DIR} )

add_executable( homework_1_02&03 homework_1_02&03.cpp )
target_link_libraries( homework_1_02&03 ${OpenCV_LIBS})


