cmake_minimum_required(VERSION 3.5)

project(object_detector)

# set compilation
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-Wall -Wextra -Wno-deprecated-copy -Wno-unused-parameter -O2)

# useful macros
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/util.cmake)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/modules)

# find dependencies
find_package(Glog REQUIRED)
find_package(Threads)
find_package(OpenCV 4 REQUIRED)

# ROS
catkin_package(
    # INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp usfs_common usfs_inference
    DEPENDS)
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    usfs_common
    usfs_inference
    cv_bridge
    image_transport
    image_geometry
    pcl_ros
    )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(PCL 1.7 REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 
add_definitions(${PCL_DEFINITIONS}) 

# common include directories
include_directories(
    include
    ${GLOG_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_DEFINITIONS}
)

# find all source files
file(GLOB_RECURSE
    src_files
    ${PROJECT_SOURCE_DIR}/src/*.cc
    ${PROJECT_SOURCE_DIR}/src/*/*.cc
    )

# common library
# add_library(
#     ${PROJECT_NAME}
#     SHARED
#     ${src_files}
# )
# target_link_libraries(
#     ${PROJECT_NAME}
#     ${GLOG_LIBRARIES}
#     ${catkin_LIBRARIES}
#     Threads::Threads
#     ${OpenCV_LIBS}
# )

add_executable(detector src/main.cpp) 
target_link_libraries(detector ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${Eigen3_INCLUDE_DIRS} ${OpenCV_LIBS})  
