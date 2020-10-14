
cmake_minimum_required(VERSION 2.8)

PROJECT(armor)
FIND_PACKAGE(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
FIND_PACKAGE(Threads REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "opencv_version: ${OpenCV_VERSION}")
message(STATUS "opencv_libraries: ${OpenCV_LIBS}")
message(STATUS "opencv_include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "boost library status:")
message(STATUS "boost_version: ${Boost_VERSION}")
message(STATUS "boost_libraries: ${Boost_LIBS}")
message(STATUS "boost_include path: ${Boost_INCLUDE_DIRS}")

message(STATUS　"thread library status:" )
message(STATUS　"thread_version:" )
message(STATUS　"thread_libraries:" )
message(STATUS　"thread_include path:" )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++11")


include_directories(${Boost_INCLUDE_DIRS})


find_package(OpenCV REQUIRED)   
include_directories(${OpenCV_INCLUDE_DIRS})

aux_source_directory(. DIR_SRCS)

add_executable(armor ${DIR_SRCS})

target_link_libraries(armor -lboost_system -lpthread ${OpenCV_LIBS} ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

