cmake_minimum_required(VERSION 3.0)
project(racecar_safety_node)

##set(CMAKE_CXX_FLAGS                "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# node executable
add_executable(racecar_safety_node src/racecar_safety_node.cpp src/racecar_safety.cpp)

target_link_libraries(racecar_safety_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS racecar_safety_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

# TODO