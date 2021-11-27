cmake_minimum_required(VERSION 3.16)
project(demo)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}) 
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_play play.cpp)
target_link_libraries(${PROJECT_NAME}_play ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_examine_E examine_E.cpp)
target_link_libraries(${PROJECT_NAME}_examine_E  ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_matchGraph matchGraph.cpp)
target_link_libraries(${PROJECT_NAME}_matchGraph  ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_chooseROI chooseROI.cpp)
target_link_libraries(${PROJECT_NAME}_chooseROI  ${OpenCV_LIBS})

add_executable(${PROJECT_NAME}_main main.cpp)
target_link_libraries(${PROJECT_NAME}_main  ${OpenCV_LIBS})

