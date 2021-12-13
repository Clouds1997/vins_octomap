cmake_minimum_required(VERSION 2.8.3)
project(pose_graph)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    camera_model
    cv_bridge
    roslib
    pcl_ros
    pcl_conversions
    message_filters
    )

find_package(OpenCV)

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(octomap REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(
    ${catkin_INCLUDE_DIRS} 
    ${CERES_INCLUDE_DIRS}  
    ${EIGEN3_INCLUDE_DIR}
    ${OCTOMAP_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
     )

catkin_package()

add_executable(pose_graph
    src/pose_graph_node.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/sub_octomap_construction.cpp
    src/ros_puber.cpp
    src/octomap_fusion.cpp
    # src/sub_octomap.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
    )

target_link_libraries(pose_graph 
    ${catkin_LIBRARIES}  
    ${OpenCV_LIBS} 
    ${CERES_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    )
message("catkin_lib  ${catkin_LIBRARIES}")
