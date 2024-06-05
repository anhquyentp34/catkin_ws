#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
//#include <gazebo_plugins/gazebo_ros_camera.h>
//#include <gazebo_ros/PhysicsConfig.h>
//#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <tf/tf.h>
#include <math.h>
// people_msgs
#include <people_msgs/Person.h>
#include <people_msgs/People.h>
//
#include <iostream>
//
ros::ros::Publisher model_state_pub, human_pose_pub;        
//
double cur_hx, cur_hy, cur_htheta;

//
void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    //Find the humans state
    
}
 