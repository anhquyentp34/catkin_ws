// This program is used to send the goal to the robot.
// The goal is in local map

#include <iostream>
#include <vector>
//
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Int8.h>
#include <social_navigation_layers/RobotPosition.h>
#include <visualization_msgs/Marker.h>
#include <social_navigation_layers/GetApproachingPose.h>

#include <boost/thread.hpp>
//
#define N_GOALS 10
#define PI 3.139601
#define PI2 1.569799
#define PI4 0.784899
//
 ros::Publisher goal_pose_pub, goal_index_pub;
 ros::Publisher end_approach_pose_command_pub; // end approaching pose command -- approaching_pose_dsz node

 gazebo_msgs::ModelState robot_state_;
 ros::Subscriber cur_pos_robot_sub;
 ros::Subscriber approach_pose_command_sub;  // receive approaching pose command from recognition node


// geometry_msgs::Pose2D new_goal_;
// unsigned int counter_ = 0;
// std_msgs::Int8 update_index_old;
 geometry_msgs::Pose update_goal_pose_old;
// bool subgoal_flag = false;
//
 // current position of robot
 geometry_msgs::Pose2D cur_rpos;

 std_msgs::Bool bApproaching;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// N_GOALS x [x,y,w]
// unsigned int approching_index [] ={0, 1, 3, 8, 12, 16, 19, 22, 25};
/* Original
double_t landmarks_pose[N_GOALS][3] = {{0.5,-0.5,-0.9849},{2.0,-3.0,0.01},{3.8,-5.5,-PI2},{3.0,-8.0,PI},{3.8,-5.5,PI2},{3.8,-4.0,PI2},{3.5,-1.5,PI4},{-3.2,-4.0,PI},{-3.2,-1.0,PI2},{-5.0,1.0,PI2},
                                       {-3.2,-1.0,-PI2},{-3.2,-4.0,PI},{-8.5,-4.0,PI},{-13.0,-7.0,PI2},{-11.0,-6.0,PI2},{-13.0,-2.5,PI2},{-11.8,1.2,PI2},{-11.8,6.6,PI2},{-12.8,6.6,PI},{-14.0,8.0,-3*PI4},
                                       {-12.8,6.6,0.01},{-10.8,6.6,0.01},{-8.0,8.5,PI4},{-10.8,6.6,PI},{-12.8,9.7,PI},{-14.0,11.5,PI},{-12.8,9.7,0.01},{-10.8,12.0,0.01},
                                       {-9.0,12.0,PI4}};
                                       */
/* dsz but do not use approaching pose prediction */
// landmarks_pose: [x, y,  z w] represent via quaternion
 // Original_Tien
double_t landmarks_pose[N_GOALS][4] = {{0.945546233456496, 4.844177606842447, 0.008298503988575642, 0.999965566822954}, {1.857125583330584, 5.351967386235168, 0.2774615212998381, 0.9607367507272632}, {6.696751552865482, 2.332338853660102, 0.016696672699354414, 0.999860600844323},
                                       {5.280866581849011,-1.1399355679074732, -0.6918911709326544, 0.7220018058048334}, {6.972651073014089, -4.603245977323878, -0.5566936207707035, 0.8307178898959647}, {3.8429206531658955, -7.746525935356161, -0.9742618110405334, 0.22541943915292642},
                                       {-1.7296581091340602,-7.6707138250513385, 0.9930704503662334, 0.1175205539869785}, {-6.741427001424995, -7.890714071782519, 0.9997861699247933, 0.020678840081402763}, {-5.286867013274685, -3.682149454960467, -0.9943849600700378, 0.10582320722086112},
                                       {-8.864045540438656, 2.047309735095137, -0.0070359868513256586, 0.9999752471381619}};



/* Human are treated like obstacles
double_t landmarks_pose[N_GOALS][3] = {{0.5,-0.5,-0.9849},{2.0,-3.0,0.01},{3.8,-5.5,-PI2},{2.0,-8.0,PI},{3.8,-5.5,PI2},{3.8,-4.0,PI2},{4.6,-0.0,PI2},{-3.2,-4.0,PI},{-3.2,-1.0,PI2},{-6.0,3.5,PI},
                                       {-3.2,-1.0,-PI2},{-3.2,-4.0,PI},{-8.5,-4.0,PI},{-13.5,-6.0,PI},{-11.0,-6.0,PI2},{-13.0,-2.5,PI2},{-11.8,1.2,PI2},{-11.8,6.6,PI2},{-12.8,6.6,PI},{-15.5,6.8,3*PI4},
                                       {-12.8,6.6,0.01},{-10.8,6.6,0.01},{-8.5,9.2,PI4},{-10.8,6.6,PI},{-12.8,9.7,PI},{-16.0,11.5,PI},{-12.8,9.7,0.01},{-10.8,12.0,0.01},
                                       {-8.0,12.2,PI4}}; */

std::vector<geometry_msgs::Pose> landmarks(N_GOALS);

// which room do robot stay in?
int robotInRoom [10] = {0, 1, 1, 1, 2, 2, 2, 3, 3, 4 };


void spinThread()
{
  ros::spin();
}

// init the goal pose
void initLandmarks()
{
    geometry_msgs::Pose goal_pose;
    for(unsigned int i =0; i<landmarks.size();i++)
    {
        goal_pose.position.x = landmarks_pose[i][0];
        goal_pose.position.y = landmarks_pose[i][1];

        // convert when roll, pitch, yaw

        //geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,landmarks_pose[i][2]);
        //goal_pose.orientation = quat;
        goal_pose.orientation.z = landmarks_pose[i][2];
        goal_pose.orientation.w = landmarks_pose[i][3];
        landmarks[i] = goal_pose;
    }
}
//
// double convertQuaternionToAngle(geometry_msgs::Quaternion quat_in){
//     tf::Quaternion quat;
//     double dummy;
//     double cur_ptheta;
//     tf::quaternionMsgToTF(quat_in, quat);
//     tf::Matrix3x3 mat(quat);
//     mat.getRPY(dummy, dummy, cur_ptheta);

//     return cur_ptheta;
// }

//
// void goalPoseVisualize(geometry_msgs::Pose goal_pose, int n_goal)
// {
//     visualization_msgs::Marker marker;
//     geometry_msgs::Point p;

//     std::stringstream ss;
//     ss << "goal_pose_" << n_goal;

//     marker.header.frame_id = "map";
//     marker.header.stamp = ros::Time();
//     marker.ns = ss.str();
//     marker.id = n_goal; // this number should be different
//     marker.type = visualization_msgs::Marker::ARROW;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.pose.orientation.x = 0.0;
//     marker.pose.orientation.y = 0.0;
//     marker.pose.orientation.z = 0.0;
//     marker.pose.orientation.w = 0.0;
//     marker.scale.x = 0.05;
//     marker.scale.y = 0.12;
//     marker.scale.z = 0.12;

//     marker.color.a = 1.0;
//     marker.color.r = 1.0;
//     marker.color.g = 0.0;
//     marker.color.b = 0.0;
//     //
//     p.x = goal_pose.position.x;
//     p.y = goal_pose.position.y;
//     p.z = 0;
//     marker.points.push_back(p);
//     //
//     double ang = convertQuaternionToAngle(goal_pose.orientation);
//     p.x = goal_pose.position.x + 0.4*cos(ang);
//     p.y = goal_pose.position.y + 0.4*sin(ang);
//     p.z = 0;
//     marker.points.push_back(p);
//     goal_pose_pub.publish(marker);
//     marker.points.clear();

// }

// update landmarks from the new goals
// bool updateLandmarks()
// {
// }
//
/*
void newGoalCallback(const geometry_msgs::Pose2D::ConstPtr &msg){
    ROS_INFO("Got a new approaching pose");
    new_goal_ = *msg;
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = new_goal_.x;
    goal_pose.position.y = new_goal_.y;
    goal_pose.orientation.w = new_goal_.theta;
    landmarks[counter_+1] = goal_pose;
}
*/
//
// void sleepWhenApproachingHumans(double nsecond, unsigned int goal_index){
//     //if((goal_index==3)||(goal_index==6)||(goal_index==9)||(goal_index==13)||(goal_index==17)||(goal_index==20)||(goal_index==23)||(goal_index==26))
//     if((goal_index==2)||(goal_index==4)||(goal_index==7)||(goal_index==10)||(goal_index==14)||(goal_index==16)||(goal_index==20)||(goal_index==23)||(goal_index==26)||(goal_index==29))
//     {
//         if((goal_index==2)){
//             ros::Duration(nsecond+2).sleep();// sleep nscond
//         }else{
//             ros::Duration(nsecond).sleep();// sleep nscond
//         }
//     }
// }
//

double convertQuaternionToAngle(geometry_msgs::Quaternion quat_in){
    tf::Quaternion quat;
    double dummy;
    double cur_ptheta;
    tf::quaternionMsgToTF(quat_in, quat);
    tf::Matrix3x3 mat(quat);
    mat.getRPY(dummy, dummy, cur_ptheta);

    return cur_ptheta;
}

void curPosRobotCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{
    // Robot pose
        double cur_rx, cur_ry, cur_rtheta;

        for(int i =0; i<msg->name.size(); i++)
        {
            if(msg->name[i] == "diff_bot")
            {
                robot_state_.model_name = msg->name[i];
                robot_state_.pose = msg->pose[i];
                //robot_state_.twist = msg->twist[i];

                cur_rtheta = convertQuaternionToAngle(msg->pose[i].orientation);
                cur_rx = msg->pose[i].position.x; cur_ry = msg->pose[i].position.y;

                cur_rpos.x = cur_rx;
                cur_rpos.y = cur_ry;
                cur_rpos.theta = cur_rtheta;
                break;
            }
        }
        //ROS_INFO("%d",cur_rx);
        //ROS_INFO("%d",cur_ry);
}
// compute distance from robot to detected human
double calDisFroRobToHuman(geometry_msgs::Pose2D curRobot, double humanX, double humanY)
{
    double dis = 0;
    double dx = 0; double dy = 0;

    dx = curRobot.x - humanX;
    dy = curRobot.y - humanY;

    dis = sqrt(dx*dx + dy*dy);

    return dis;
}
void approaPoseCommandCallback(std_msgs::Bool msg)
{
    bApproaching.data = msg.data;
    //ROS_INFO("b", msg);
   // return 0;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "send_multi_goals_node");
  ros::NodeHandle nh;

  // goal_pose_pub = nh.advertise<visualization_msgs::Marker>("send_goal_pose/goal_pose",0);
  // goal_index_pub = nh.advertise<std_msgs::Int8>("/send_goal_pose/goal_index",100);
   goal_index_pub = nh.advertise<social_navigation_layers::RobotPosition>("/send_goal_pose/goal_index",100);

   end_approach_pose_command_pub = nh.advertise<std_msgs::Bool>("/send_goal_pose/end_approach_pose",10);

   cur_pos_robot_sub =  nh.subscribe("/gazebo/model_states", 100, &curPosRobotCallback);
   approach_pose_command_sub = nh.subscribe("/detect_and_recog/appr_pose_command", 10, approaPoseCommandCallback);


  //ros::Subscriber new_goal_sub = nh.subscribe("/mybot_description/approaching_pose",5,&newGoalCallback);

  boost::thread spin_thread(&spinThread);
  // Init client srv
  ros::ServiceClient app_pose_client = nh.serviceClient<social_navigation_layers::GetApproachingPose>("/get_approaching_pose");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient action_goal("move_base", false);
  //
  initLandmarks();

  //wait for the action server to come up
  while(!action_goal.waitForServer(ros::Duration(10.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  unsigned int counter = 0;
  //geometry_msgs::Pose update_goal_pose_old;

  while(counter< N_GOALS) //N_GOALS
  {
       social_navigation_layers::GetApproachingPose srv;

      //if (counter == 3 || counter == 6 || counter == 9) // approach female person in room 1
      if (bApproaching.data == true) // start approaching pose
      {
        if(app_pose_client.call(srv))
        {
            std_msgs::Int8 update_index;
            geometry_msgs::Pose update_goal_pose;

            update_goal_pose.position.x = srv.response.app_pose.x;
            update_goal_pose.position.y = srv.response.app_pose.y;
            //update_goal_pose.orientation.w = srv.response.app_pose.theta;
            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,srv.response.app_pose.theta);
            update_goal_pose.orientation = quat;
            update_index = srv.response.index;

            if((update_goal_pose.position.x!=update_goal_pose_old.position.x)||
               (update_goal_pose.position.y!=update_goal_pose_old.position.y))
            {
               ROS_INFO("Receive a new goal [%d] (x, y, w) = (%f, %f, %f)", update_index.data, update_goal_pose.position.x,
                       update_goal_pose.position.y, srv.response.app_pose.theta);

               move_base_msgs::MoveBaseGoal new_approach_goal;

               new_approach_goal.target_pose.header.frame_id = "map";
               new_approach_goal.target_pose.header.stamp = ros::Time::now();
               new_approach_goal.target_pose.pose = update_goal_pose;

               ROS_INFO("Sending goal [%d] (x, y, w) = (%f, %f, %f)", counter, new_approach_goal.target_pose.pose.position.x,
                        new_approach_goal.target_pose.pose.position.y, new_approach_goal.target_pose.pose.orientation.w);

               action_goal.sendGoal(new_approach_goal);

               action_goal.waitForResult();

               if(action_goal.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
               {
                 ROS_INFO("The robot aproached the goal [%d]",counter);
                 ros::Duration(4).sleep();
               }
               else
               {
                 ROS_INFO("The robot failed to move to the goal for some reason");
               }
                std_msgs::Bool end_approach_pose_msg;
                end_approach_pose_msg.data = false;
                end_approach_pose_command_pub.publish(end_approach_pose_msg);

                update_goal_pose_old = update_goal_pose;


            }
        }
        else
        {
            ROS_ERROR("Failed to call service approaching pose");
        }
      }
      bApproaching.data = false;

      move_base_msgs::MoveBaseGoal next_goal;
      geometry_msgs::Pose goal_pose;

      goal_pose = landmarks[counter];
      //goalPoseVisualize(goal_pose, 1);

      //std_msgs::Int8 goal_ind;
      //goal_ind.data = counter;
      //goal_index_pub.publish(goal_ind);
      // ROS_INFO("Goal index [%d] ", goal_ind.data);

      social_navigation_layers::RobotPosition msg;
      msg.goal_index = counter;
      msg.room = robotInRoom[counter];

      goal_index_pub.publish(msg);


      //we'll send a goal to the robot to move 1 meter forward
      //goal.target_pose.header.frame_id = "base_link"; // use "base_link" this one for local map and "map" for global map

      next_goal.target_pose.header.frame_id = "map";
      next_goal.target_pose.header.stamp = ros::Time::now();
      next_goal.target_pose.pose = goal_pose;

      ROS_INFO("Sending goal [%d] (x, y, w) = (%f, %f, %f)",counter,next_goal.target_pose.pose.position.x,next_goal.target_pose.pose.position.y, next_goal.target_pose.pose.orientation.w);
      action_goal.sendGoal(next_goal);

      action_goal.waitForResult();

      if(action_goal.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("The robot aproached the goal [%d]",counter);
      }
      else
      {
        ROS_INFO("The robot failed to move to the goal for some reason");
      }

      // if((counter==2)||(counter==15)){// moving human
      //     bool flag = action_goal.waitForResult(ros::Duration(1.0));
      //     ROS_INFO("Wait for result: %d", flag);
      //     if(counter==1){
      //         if((flag)||(next_goal.target_pose.pose.position.x<2.0)){
      //           subgoal_flag = false;
      //           action_goal.cancelGoal();
      //         }else{
      //           subgoal_flag = true;
      //         }
      //     }else{
      //         if((flag)||(next_goal.target_pose.pose.position.x>-9.5)){
      //           subgoal_flag = false;
      //           action_goal.cancelGoal();
      //         }else{
      //           subgoal_flag = true;
      //         }
      //     }
      // }else{
      //   action_goal.waitForResult();
      //   subgoal_flag = false;
      // }

      /* Select the next landmark*/
      //ros::Duration(1).sleep();// sleep 2 second
      //sleepWhenApproachingHumans(6, counter);// sleep 1 second

      //if(!subgoal_flag)
        counter++;

        ros::Duration(5).sleep();

      //sleepWhenApproachingHumans(10, counter);// sleep 1 second
  }
  ROS_INFO("All mission was complete!");

  spin_thread.join();

  return 0;
}

