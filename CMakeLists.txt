cmake_minimum_required(VERSION 2.8.3)
project(tobi_sim)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tobi_sim
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#############
## Install ##
#############

install(DIRECTORY config launch media models scenes worlds assets model_makros
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/mujoco_person_tracker
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

