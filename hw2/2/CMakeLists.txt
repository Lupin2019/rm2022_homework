cmake_minimum_required( VERSION 3.15 )

project(taxi)

# OpenCV
find_package( OpenCV 4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIR} )

# thread
find_package(Threads)

add_executable( main main.cpp )
target_link_libraries( main ${OpenCV_LIBS} )
target_link_libraries( main ${CMAKE_THREAD_LIBS_INIT} )
# target_link_libraries( main MVSDK )