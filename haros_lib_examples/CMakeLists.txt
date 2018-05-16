cmake_minimum_required(VERSION 2.8.3)
project(haros_lib_examples)

find_package(catkin REQUIRED COMPONENTS
  haros
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES haros_lib_examples
 CATKIN_DEPENDS haros roscpp std_msgs
 DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(publisher src/publisher.cpp)

add_dependencies(publisher
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(publisher ${catkin_LIBRARIES})

add_executable(subscriber src/subscriber.cpp)

add_dependencies(subscriber
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(subscriber ${catkin_LIBRARIES})

add_executable(not_thread_safe src/not_thread_safe.cpp)

add_dependencies(not_thread_safe
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(not_thread_safe ${catkin_LIBRARIES})

add_executable(thread_safe src/thread_safe.cpp)

add_dependencies(thread_safe
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(thread_safe ${catkin_LIBRARIES})
