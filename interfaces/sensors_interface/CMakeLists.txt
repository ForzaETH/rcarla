cmake_minimum_required(VERSION 2.8.3)
project(sensors_interface)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)

  catkin_python_setup()

  catkin_package()

  include_directories(${catkin_INCLUDE_DIRS})

  install(PROGRAMS 
    src/sensors_interface/imu.py
    src/sensors_interface/wheel_odom.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


endif()
