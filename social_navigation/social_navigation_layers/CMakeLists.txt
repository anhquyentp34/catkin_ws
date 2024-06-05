cmake_minimum_required(VERSION 2.8.3)
project(social_navigation_layers)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  people_msgs
  pluginlib
  roscpp
  gazebo_ros
  message_generation
)


## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system thread signals)

add_message_files(
  FILES
  RobotPosition.msg
  RecognitionState.msg
)

add_service_files(
  FILES
  GetApproachingPose.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs # Or other packages containing msgs
 )
 
 ## dynamic reconfigure config
generate_dynamic_reconfigure_options(
  cfg/ProxemicLayer.cfg
)


 catkin_package(
    CATKIN_DEPENDS angles costmap_2d dynamic_reconfigure geometry_msgs people_msgs pluginlib roscpp gazebo_ros
    INCLUDE_DIRS include
    LIBRARIES social_layers
)



## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


## add cpp library
add_library(social_layers 
            src/social_layer.cpp
            src/proxemic_layer.cpp 
            src/passing_layer.cpp
            src/human_group_layer.cpp
            src/human_object_layer.cpp
            src/gazebo_ros_communication.cpp
)

## Add cmake target dependencies of the executable/library
add_dependencies(social_layers people_msgs_gencpp ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(social_layers ${catkin_LIBRARIES})

add_executable(gazebo_ros_communication src/gazebo_ros_communication.cpp)
target_link_libraries(gazebo_ros_communication ${catkin_LIBRARIES})

# add_executable(mybot_send_multi_goals_dsz src/mybot_send_multi_goals_dsz.cpp)
# target_link_libraries(mybot_send_multi_goals_dsz ${catkin_LIBRARIES})

# add_executable(send_multi_goals_dsz_node src/send_multi_goals_dsz_node.cpp)
# target_link_libraries(send_multi_goals_dsz_node ${catkin_LIBRARIES})

# add_executable(send_multi_goals_dsz_node_thread src/send_multi_goals_dsz_node_thread.cpp)
# target_link_libraries(send_multi_goals_dsz_node_thread ${catkin_LIBRARIES})

add_executable(approaching_pose_dsz_node src/approaching_pose_dsz_node.cpp)
target_link_libraries(approaching_pose_dsz_node ${catkin_LIBRARIES})
add_dependencies(approaching_pose_dsz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_executable(mybot_convert_laser_pointcloud src/mybot_convert_laser_pointcloud.cpp)
# target_link_libraries(mybot_convert_laser_pointcloud ${catkin_LIBRARIES})
# add_dependencies(mybot_convert_laser_pointcloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS social_layers
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
