cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# You should set the PYTHONPATH to your own python site-packages path

set(PYTHON_EXECUTABLE "/usr/bin/python3")
set(PYTHON_VERSION "3.10")
find_package(PythonInterp ${PYTHON_VERSION} REQUIRED)

set(ENV{PYTHONPATH} "/opt/ros/humble/local/lib/python3.10/dist-packages:$ENV{PYTHONPATH}")
#set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)
find_package(OpenCV 4.0 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(pcl_ros REQUIRED)

#-------Added for custom Map message--------
find_package(rosidl_default_generators REQUIRED)

# Add message files
rosidl_generate_interfaces(${PROJECT_NAME} "msg/Map.msg" DEPENDENCIES geometry_msgs)

#-------------

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
)

link_directories(
  include
)


   # add_executable(mono_realsense_D435i
    #        src/monocular/mono_realsense_D435i.cc)


add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
  #src/monocular/mono_realsense_D435i.cc
)

ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin Sophus realsense2 tf2 tf2_ros geometry_msgs)

#---------- for ros2 image pub node --------
add_executable(realsense_publisher src/realsense_publisher/ros2_img_pub.cpp)
target_link_libraries(realsense_publisher realsense2::realsense2 ${OpenCV_LIBRARIES})
ament_target_dependencies(realsense_publisher rclcpp sensor_msgs cv_bridge)

install(TARGETS realsense_publisher
        DESTINATION lib/${PROJECT_NAME})
#------------------------------------------



add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters
ORB_SLAM3 Pangolin OpenCV)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin OpenCV)


install(TARGETS mono rgbd stereo stereo-inertial
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()

