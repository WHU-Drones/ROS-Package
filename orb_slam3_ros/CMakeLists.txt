cmake_minimum_required(VERSION 3.0.2)
project(orb_slam3_ros)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-deprecated -O3 -march=native ")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(cv_bridge_DIR /usr/local/cv_bridge-3.4.12/share/cv_bridge/cmake)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  tf
  sensor_msgs
  dynamic_reconfigure
  nav_msgs
)

## System dependencies are found with CMake's conventions
set(OpenCV_DIR /usr/local/opencv-3.4.12/share/OpenCV)
find_package(OpenCV 3)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV 3 not found.")
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
set (DYNAMIC_RECONFIGURE_PATH ros/config/dynamic_reconfigure.cfg)
execute_process(COMMAND chmod a+x ${DYNAMIC_RECONFIGURE_PATH}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        RESULT_VARIABLE cmd_result
        OUTPUT_VARIABLE cmd_ver)
message(STATUS "Chmod a+x the dynamic_reconfigure file")
generate_dynamic_reconfigure_options(
        ${DYNAMIC_RECONFIGURE_PATH}
)

set(LIBS_ORBSLAM
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
-lboost_serialization
-lcrypto
)

set(LIBS_ROS
${PROJECT_SOURCE_DIR}/orb_slam3/lib/lib${PROJECT_NAME}.so
${OpenCV_LIBS}
${catkin_LIBRARIES}
${Pangolin_LIBRARIES}
)

catkin_package (
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs dynamic_reconfigure
  LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
)

include_directories(
${PROJECT_SOURCE_DIR}/orb_slam3
${PROJECT_SOURCE_DIR}/orb_slam3/include
${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
${PROJECT_SOURCE_DIR}/ros/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)

include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam3/lib)

add_library(${PROJECT_NAME} SHARED
orb_slam3/src/System.cc
orb_slam3/src/Tracking.cc
orb_slam3/src/LocalMapping.cc
orb_slam3/src/LoopClosing.cc
orb_slam3/src/ORBextractor.cc
orb_slam3/src/ORBmatcher.cc
orb_slam3/src/FrameDrawer.cc
orb_slam3/src/Converter.cc
orb_slam3/src/MapPoint.cc
orb_slam3/src/KeyFrame.cc
orb_slam3/src/Atlas.cc
orb_slam3/src/Map.cc
orb_slam3/src/MapDrawer.cc
orb_slam3/src/Optimizer.cc
orb_slam3/src/PnPsolver.cc
orb_slam3/src/Frame.cc
orb_slam3/src/KeyFrameDatabase.cc
orb_slam3/src/Sim3Solver.cc
orb_slam3/src/Initializer.cc
orb_slam3/src/Viewer.cc
orb_slam3/src/ImuTypes.cc
orb_slam3/src/G2oTypes.cc
orb_slam3/src/CameraModels/Pinhole.cpp
orb_slam3/src/CameraModels/KannalaBrandt8.cpp
orb_slam3/src/OptimizableTypes.cpp
orb_slam3/src/MLPnPsolver.cpp
orb_slam3/src/TwoViewReconstruction.cc
)

add_dependencies (${PROJECT_NAME} g2o DBoW2)

target_link_libraries(${PROJECT_NAME}
${LIBS_ORBSLAM}
)

add_executable (${PROJECT_NAME}_stereo
ros/src/StereoNode.cc
ros/src/CommonNode.cc
)
add_dependencies (${PROJECT_NAME}_stereo ${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_stereo
${LIBS_ROS}
)

add_executable (${PROJECT_NAME}_stereo_inertial
ros/src/StereoInertialNode.cc
ros/src/CommonNode.cc
)
add_dependencies (${PROJECT_NAME}_stereo_inertial ${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_stereo_inertial
${LIBS_ROS}
)