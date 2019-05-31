cmake_minimum_required(VERSION 2.8.3)
project(taraxl_ros_package)

set(XUNIT_LIBS /usr/local/taraxl-sdk/lib/libecon_xunit.so)
set(TARA_LIBS /usr/local/taraxl-sdk/lib/libecon_taraxl.so)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(OpenCV 3.4.2 REQUIRED PATHS  /usr/local/taraxl-opencv)

find_package(Boost REQUIRED)
find_package(cv_bridge REQUIRED)

#find_package(PCL REQUIRED)
find_package(PCL 1.8 REQUIRED HINTS "/usr/local/taraxl-pcl")
#find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  stereo_msgs
  dynamic_reconfigure
  image_geometry
)


generate_dynamic_reconfigure_options(
  cfg/taraxl.cfg
  cfg/steereocam.cfg
)


catkin_package(
  CATKIN_DEPENDS cv_bridge  image_transport  roscpp sensor_msgs stereo_msgs
  DEPENDS Boost
)

include_directories(
  include/
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

  )

add_executable(taraxl_ros_package src/TaraXLRos.cpp)

add_dependencies(taraxl_ros_package ${PROJECT_NAME}_gencfg)

target_link_libraries(taraxl_ros_package ${catkin_LIBRARIES} ${TARA_LIBS} ${XUNIT_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} )
