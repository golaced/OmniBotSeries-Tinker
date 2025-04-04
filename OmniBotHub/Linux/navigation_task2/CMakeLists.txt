cmake_minimum_required(VERSION 2.6)
SET(PROJECT_NAME navigation_task_tinker)
PROJECT(${PROJECT_NAME})
find_package(Threads)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3  -pthread" )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../build)

include_directories( 
     "./build"
     "./inc"
     "/usr/local/include"
     
)

AUX_SOURCE_DIRECTORY(src DIR_SRCS)

ADD_EXECUTABLE(${PROJECT_NAME} ${DIR_SRCS})

add_subdirectory(yaml-cpp lib/ EXCLUDE_FROM_ALL)
target_link_libraries(${PROJECT_NAME} yaml-cpp)
#target_link_libraries(${PROJECT_NAME} wiringPi)





