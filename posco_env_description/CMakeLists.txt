cmake_minimum_required(VERSION 3.0.2)
project(posco_env_description)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS xacro)

install(DIRECTORY robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)