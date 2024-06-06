#include <ros/ros.h>
#include <std_msgs/String.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>
#include <math.h>
// Thu vien people_msgs
#include <people_msgs/Person.h>
#include <people_msgs/People.h>
//
#include <iostream>
//
#include "/home/quyenanhpt/catkin_ws/src/social_navigation/social_navigation_layers/include/social_navigation_layers/circle_fitting/mystuff.h"
#include "/home/quyenanhpt/catkin_ws/src/social_navigation/social_navigation_layers/include/social_navigation_layers/circle_fitting/data.h"
#include "/home/quyenanhpt/catkin_ws/src/social_navigation/social_navigation_layers/include/social_navigation_layers/circle_fitting/circle.h"
#include "/home/quyenanhpt/catkin_ws/src/social_navigation/social_navigation_layers/include/social_navigation_layers/circle_fitting/Utilities.cpp"
#include "/home/quyenanhpt/catkin_ws/src/social_navigation/social_navigation_layers/include/social_navigation_layers/circle_fitting/CircleFitByTaubin.cpp"
#include "/home/quyenanhpt/catkin_ws/src/social_navigation/social_navigation_layers/include/social_navigation_layers/circle_fitting/CircleFitByPratt.cpp"
#include "/home/quyenanhpt/catkin_ws/src/social_navigation/social_navigation_layers/include/social_navigation_layers/circle_fitting/CircleFitByKasa.cpp"
#include "/home/quyenanhpt/catkin_ws/src/social_navigation/social_navigation_layers/include/social_navigation_layers/circle_fitting/CircleFitByHyper.cpp"

// Khai bao cac publisher cua node
ros::Publisher model_state_pub, human_pose_pub;

// Cac bien toan cuc de luu toa do va goc hien tai cua con nguoi
double cur_hx, cur_hy, cur_htheta;

// Khai bao vector luu toa do nhom nguoi
std::vector<geometry_msgs::Pose2D> vhgroup_;

// Ham phat hien nhom nguoi dua tren trang thai mo hinh tu Gazebo
std::vector<geometry_msgs::Pose2D> humanGroupDetection(const gazebo_msgs::ModelStates &human_states) 
    {
    geometry_msgs::Pose2D pose;
    std::vector<geometry_msgs::Pose2D> vpose;

    Circle circle;
    reals BEDataX[3] = {0, 0, 0}, BEDataX1[3] = {0, 0, 0}, BEDataX2[2] = {0, 0};
    reals BEDataY[3] = {0, 0, 0}, BEDataY1[3] = {0, 0, 0}, BEDataY2[2] = {0, 0};

    int k = 0, k1 = 0;
    for (int i = 0; i < human_states.name.size(); i++) {
        ROS_INFO("Checking human state: %s", human_states.name[i].c_str());

        // Phat hien nhom nguoi dung gan nhau
        if ((human_states.name[i] == "actor13") || (human_states.name[i] == "actor14"))
        {
            if (k < 2) 
            {
                BEDataX2[k] = human_states.pose[i].position.x;
                BEDataY2[k] = human_states.pose[i].position.y;
                k++;
            }
            if (k == 2) 
            {
                pose.x = (BEDataX2[0] + BEDataX2[1]) / 2;
                pose.y = (BEDataY2[0] + BEDataY2[1]) / 2;

                double deltaX = BEDataX2[0] - BEDataX2[1];
                double deltaY = BEDataY2[0] - BEDataY2[1];
                pose.theta = sqrt(deltaX * deltaX + deltaY * deltaY) / 2;
                vpose.push_back(pose);
                k = 0;  // Dat lai bien dem
            }
        }

        // Phat hien nhom nguoi tao thanh hinh tron
        if ((human_states.name[i] == "actor1") || (human_states.name[i] == "actor2") || (human_states.name[i] == "actor3") ||
            (human_states.name[i] == "actor10") || (human_states.name[i] == "actor11") || (human_states.name[i] == "actor12")) {
            if (k1 < 3) {
                BEDataX1[k1] = human_states.pose[i].position.x;
                BEDataY1[k1] = human_states.pose[i].position.y;
                k1++;
            }
            if (k1 == 3) {
                Data data1(3, BEDataX1, BEDataY1);
                circle = CircleFitByTaubin(data1);
                pose.x = circle.a;
                pose.y = circle.b;
                pose.theta = circle.r;
                vpose.push_back(pose);
                k1 = 0;  // Dat lai bien dem
            }
        }
    }
    return vpose;
    }   

// Ham callback xu ly du lieu tu topic /gazebo/model_states
void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg) {
    tf::Quaternion quat;
    double dummy;
    gazebo_msgs::ModelStates human_state;

    people_msgs::People people;
    people_msgs::Person person;

    double cur_rx, cur_ry, cur_rtheta;

    // Tim toa do va goc cua robot
    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "robot") {
            tf::quaternionMsgToTF(msg->pose[i].orientation, quat);
            tf::Matrix3x3 mat(quat);
            mat.getRPY(dummy, dummy, cur_rtheta);
            cur_rx = msg->pose[i].position.x;
            cur_ry = msg->pose[i].position.y;
            ROS_INFO("Robot pose (x,y,theta) = (%f, %f, %f)", cur_rx, cur_ry, cur_rtheta);
            break;
        }
    }

    // Duyet qua tat ca cac mo hinh de tim nguoi
    for (size_t i = 0; i < msg->name.size(); i++) 
    {
        if ((msg->name[i] == "actor0") || (msg->name[i] == "actor1") || (msg->name[i] == "actor2") || 
            (msg->name[i] == "actor3") || (msg->name[i] == "actor4") || (msg->name[i] == "actor5") ||
            (msg->name[i] == "actor6") || (msg->name[i] == "actor7") || (msg->name[i] == "actor8") ||
            (msg->name[i] == "actor9") || (msg->name[i] == "actor10") || (msg->name[i] == "actor11") ||
            (msg->name[i] == "actor12") || (msg->name[i] == "actor13") || (msg->name[i] == "actor14")) 
        {
            tf::quaternionMsgToTF(msg->pose[i].orientation, quat);
            tf::Matrix3x3 mat(quat);
            double cur_ptheta;
            mat.getRPY(dummy, dummy, cur_ptheta);

            person.position.x = msg->pose[i].position.x;
            person.position.y = msg->pose[i].position.y;
            person.position.z = 0;

            // Thiet lap van toc cho cac dien vien
            if ((msg->name[i] == "actor4") || (msg->name[i] == "actor5") || (msg->name[i] == "actor6"))
            {
                person.velocity.x = 0.25 * cos(cur_ptheta);
                person.velocity.y = 0.25 * sin(cur_ptheta);
                person.velocity.z = 0;
            } 
            else 
            {
                person.velocity.x = 0.1 * cos(cur_ptheta);
                person.velocity.y = 0.1 * sin(cur_ptheta);
                person.velocity.z = 0;
            }

            // Dat do tin cay cho cac dien vien
            if ((msg->name[i] == "actor_30") || (msg->name[i] == "actor31") || (msg->name[i] == "actor32") ||
                (msg->name[i] == "actor33") || (msg->name[i] == "actor34") || (msg->name[i] == "actor35") ||
                (msg->name[i] == "actor36") || (msg->name[i] == "actor37") || (msg->name[i] == "actor38") ||
                (msg->name[i] == "actor39") || (msg->name[i] == "actor40")) 
                {
                person.reliability = 0;
                } 
                else 
                {
                person.reliability = 1;
                }

            person.name = "people_info";
            people.people.push_back(person);
        }
    }

    // Lay cac nhom nguoi tu ham humanGroupDetection va them vao message people
    vhgroup_ = humanGroupDetection(*msg);
    int group_people_flag = true;
    if (group_people_flag && !vhgroup_.empty()) 
    {
        for (const auto &group_pose : vhgroup_) 
        {
            person.position.x = group_pose.x;
            person.position.y = group_pose.y;
            person.position.z = 0;

            person.velocity.x = 0;
            person.velocity.y = 0;
            person.velocity.z = 0;
            person.reliability = group_pose.theta;
            person.name = "group_people_info";
            people.people.push_back(person);
        }
    }
    // object poeple them vao message people
    int object_people_flag = true;
    if(object_people_flag){
        person.position.x = -4.8;
        person.position.y = 2.8;
        //person.position.x = -5.8;
        //person.position.y = 1.8;
        person.position.z = 0;

        person.velocity.x = 1; //os(-M_PI_4l);
        person.velocity.y = 1; //sin(-M_PI_4l);
        person.velocity.z = 0;
        // distance from human to object
        person.reliability = 3.0;
        person.name = "object_people_info";
        people.people.push_back(person);
    }
    //
    people.header.frame_id = "map";  // global map
    //people.header.seq = human_body.header.seq;
    //people.header.stamp = human_body.header.stamp;
    // pulish
    human_pose_pub.publish(people);
}

// Ham chinh cua chuong trinh
int main(int argc, char **argv) 
{
    ros::init(argc, argv, "gazebo_ros_communication_node");
    ros::NodeHandle nh;

    // Khoi tao cac publisher va subscriber
    model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 5);
    human_pose_pub = nh.advertise<people_msgs::People>("/human_information", 5);
    ros::Subscriber model_states_sub = nh.subscribe("/gazebo/model_states", 100, &modelStatesCallback);

    // Vong lap xu ly chinh cua chuong trinh
    ros::Rate rate(30);
    while (ros::ok()) 
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
