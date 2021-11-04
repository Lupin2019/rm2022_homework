cmake_minimum_required( VERSION 3.15 )

project(1)

# OpenCV
find_package( OpenCV 4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIR} )

# thread
find_package(Threads)

# EIGEN3
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})


add_executable( main main.cpp )
target_link_libraries( main ${OpenCV_LIBS} )
target_link_libraries( main ${CMAKE_THREAD_LIBS_INIT} )
target_link_libraries( main ${E})
# target_link_libraries( main MVSDK )