cmake_minimum_required(VERSION 2.8.3)
project(rcarla_bringup)

find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS)

  catkin_package()

  include_directories(${catkin_INCLUDE_DIRS})

  install(DIRECTORY launch/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

endif()


if (${ROS_VERSION} EQUAL 2)
    cmake_minimum_required(VERSION 3.5)
    find_package(ament_cmake REQUIRED)
    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
    ament_package()
endif()
