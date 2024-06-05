#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <angles/angles.h>

#include <geometry_msgs/Pose2D.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

// circle fitting
#include "circle_fitting/mystuff.h"
#include "circle_fitting/data.h"
#include "circle_fitting/circle.h"
#include "circle_fitting/Utilities.cpp"
#include "circle_fitting/CircleFitByTaubin.cpp"
#include "circle_fitting/CircleFitByPratt.cpp"
#include "circle_fitting/CircleFitByKasa.cpp"
#include "circle_fitting/CircleFitByHyper.cpp"

// people_msgs
#include <people_msgs/Person.h>
#include <people_msgs/People.h>

// service to get pose

#include <social_navigation_layers/GetApproachingPose.h>\

#include <social_navigation_layers/RobotPosition.h>


// declare global variables
ros::Publisher human_pose_pub; // publish "human_infomation" to display dsz on local_costmap
ros::Publisher app_pose_pub;   // publish pose to send goal
ros::Publisher approaching_pub; // publish approachinng_areas
ros::Publisher app_pose_vis_pub; // publish app_pose_vis
ros::Publisher detected_human_vis_pub; // publish detected human



ros::Subscriber mode_state_sub;
ros::Subscriber local_cosmap_sub;
ros::Subscriber detected_human_sub;
ros::Subscriber approach_pose_command_sub;
ros::Subscriber end_approach_pose_command_sub;
ros::Subscriber recognized_human_vis_sub;
ros::Subscriber goal_index_sub;

// Server
ros::ServiceServer app_pose_service;
std_msgs::Int8 goal_index_;
nav_msgs::OccupancyGrid local_costmap_;

geometry_msgs::Pose2D cur_rpos_;

gazebo_msgs::ModelStates human_states_, detected_human_states_;
gazebo_msgs::ModelState robot_state_;

geometry_msgs::Pose2D app_pose_;

std::vector <geometry_msgs::Pose2D> vhgroup_;

bool appr_visualize_flag;
bool recognized_visualze_flag;

// room and target of robot
int numRoomRobot;
int numTargeRobot;

double convertQuaternionToAngle(geometry_msgs::Quaternion quat_in){
    tf::Quaternion quat;
    double dummy;
    double cur_ptheta;
    tf::quaternionMsgToTF(quat_in, quat);
    tf::Matrix3x3 mat(quat);
    mat.getRPY(dummy, dummy, cur_ptheta);

    return cur_ptheta;
}

gazebo_msgs::ModelStates getHumanPoseFromGazebo(gazebo_msgs::ModelStates msg){

    gazebo_msgs::ModelStates human_states;

    for(int i =0; i<msg.name.size(); i++){
        if((msg.name[i] == "actor1")||(msg.name[i] == "actor2")||(msg.name[i] == "actor3")||(msg.name[i] == "actor4")||(msg.name[i] == "man02")
           ||(msg.name[i] == "actor41")||(msg.name[i] == "rp_denis")||(msg.name[i] == "rp_denis_0")||(msg.name[i] == "female")||(msg.name[i] == "rp_mei")
           ||(msg.name[i] == "person_standing")||(msg.name[i] == "kumaeye")|| (msg.name[i] == "person_standing_0"))
        {
            human_states.name.push_back(msg.name[i]);
            human_states.pose.push_back(msg.pose[i]);
            human_states.twist.push_back(msg.twist[i]);
        }
    }
    ROS_INFO("Number of Human: %lu", human_states.name.size());

    return human_states;
}
// use circle fitting to draw circle 2, 3, 4, ... N persons of group
std::vector <geometry_msgs::Pose2D> humanGroupDetection(gazebo_msgs::ModelStates human_states){

    geometry_msgs::Pose2D pose;
    std::vector <geometry_msgs::Pose2D> vpose;

    Circle circle;
    reals BEDataX[3]= {0,0,0};reals BEDataX1[3]= {0,0,0};reals BEDataX2[2]= {0,0};
    reals BEDataX3[2]= {0,0};reals BEDataX4[2]= {0,0};reals BEDataX5[3]= {0,0,0};
    reals BEDataY[3]= {0,0,0}; reals BEDataY1[3]= {0,0,0}; reals BEDataY2[2]= {0,0};
    reals BEDataY3[2]= {0,0}; reals BEDataY4[2]= {0,0}; reals BEDataY5[3]= {0,0,0};

    int k =0; int k1 =0; int k2 =0; int k3 =0; int k4 =0; int k5 =0;
    for(int i = 0; i<human_states.name.size(); i++)
    {
        if((human_states.name[i] == "person_standing")||(human_states.name[i] == "rp_denis_0"))
        {
            BEDataX2[k] = human_states.pose[i].position.x;
            BEDataY2[k] = human_states.pose[i].position.y;

            k = k+ 1;
            if(k==2)
            {
                //Data data1(2,BEDataX2,BEDataY2);
                //Circle circle;
                //cout.precision(7);
                ///circle = CircleFitByTaubin(data1);
                //cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
                //pose.x = circle.a; pose.y = circle.b; pose.theta = circle.r;
                pose.x = (BEDataX2[0] + BEDataX2[1])/2;
                pose.y = (BEDataY2[0] + BEDataY2[1])/2;

                double deltaX = BEDataX2[0] - BEDataX2[1];
                double deltaY = BEDataY2[0] - BEDataY2[1];

                pose.theta = sqrt(deltaX*deltaX + deltaY*deltaY)/2;
                vpose.push_back(pose);
            }
        }

        // if((human_states.name[i] == "p5")||(human_states.name[i] == "p6")||(human_states.name[i] == "p7"))
        // {
        //     BEDataX1[k1] = human_states.pose[i].position.x;
        //     BEDataY1[k1] = human_states.pose[i].position.y;
        //     k1 = k1+ 1;
        //     if(k1==3){
        //         Data data1(3,BEDataX1,BEDataY1);
        //         //Circle circle;
        //         cout.precision(7);
        //         circle = CircleFitByTaubin (data1);
        //         //cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
        //         pose.x = circle.a; pose.y = circle.b; pose.theta = circle.r;
        //         vpose.push_back(pose);
        //     }
        // }

        // if((human_states.name[i] == "p10")||(human_states.name[i] == "p11"))
        // {
        //     BEDataX1[k2] = human_states.pose[i].position.x;
        //     BEDataY1[k2] = human_states.pose[i].position.y;
        //     k2 = k2+ 1;
        //     if(k2==2){
        //         Data data1(2,BEDataX2,BEDataY2);
        //         //Circle circle;
        //         cout.precision(7);
        //         circle = CircleFitByTaubin (data1);
        //         cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
        //         pose.x = circle.a; pose.y = circle.b; pose.theta = circle.r;
        //         vpose.push_back(pose);
        //     }
        // }
        // if((human_states.name[i] == "p12")||(human_states.name[i] == "p13"))
        // {
        //     BEDataX1[k3] = human_states.pose[i].position.x;
        //     BEDataY1[k3] = human_states.pose[i].position.y;
        //     k3 = k3+ 1;
        //     if(k3==2){
        //         Data data1(2,BEDataX3,BEDataY3);
        //         //Circle circle;
        //         cout.precision(7);
        //         circle = CircleFitByTaubin (data1);
        //         cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
        //         pose.x = circle.a; pose.y = circle.b; pose.theta = circle.r;
        //         vpose.push_back(pose);
        //     }
        // }

        // if((human_states.name[i] == "p0")||(human_states.name[i] == "p1"))
        // {
        //     BEDataX1[k4] = human_states.pose[i].position.x;
        //     BEDataY1[k4] = human_states.pose[i].position.y;
        //     k4 = k4+ 1;
        //     if(k4==2){
        //         Data data1(2,BEDataX4,BEDataY4);
        //         //Circle circle;
        //         cout.precision(7);
        //         circle = CircleFitByTaubin (data1);
        //         //cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
        //         pose.x = circle.a; pose.y = circle.b; pose.theta = circle.r;
        //         vpose.push_back(pose);
        //     }
        // }
        // if((human_states.name[i] == "p15")||(human_states.name[i] == "p16"))
        // {
        //     BEDataX1[k5] = human_states.pose[i].position.x;
        //     BEDataY1[k5] = human_states.pose[i].position.y;
        //     k5 = k5+ 1;
        //     if(k5==2){
        //         Data data1(2,BEDataX5,BEDataY5);
        //         //Circle circle;
        //         cout.precision(7);
        //         circle = CircleFitByTaubin (data1);
        //         //cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
        //         pose.x = circle.a; pose.y = circle.b; pose.theta = circle.r;
        //         vpose.push_back(pose);
        //     }
        // }

    }
    return vpose;
}

void localCostmapCallback(const nav_msgs::OccupancyGridConstPtr & local_costmap)
{

    local_costmap_ = *local_costmap;

    double step = local_costmap->info.resolution;
    double x_localmap = local_costmap->info.origin.position.x;
    double y_localmap = local_costmap->info.origin.position.y;
    unsigned int value_localmap = local_costmap->data.size();
    double xsize_localmap = local_costmap->info.height;
    double ysize_localmap = local_costmap->info.width;
    //ROS_INFO("Local map info:%f,%f,%f,%d,%f,%f",step,x_localmap,y_localmap,value_localmap,xsize_localmap,ysize_localmap);

    //for(int i =0; i<local_costmap->data.size(); i++)
    //{
    //    ROS_INFO("%d=%d,",i, local_costmap->data[i]);
    //}
}

bool checkObstacles(nav_msgs::OccupancyGrid local_costmap, double xc, double yc)
{
    bool flag=true;
    double step = local_costmap.info.resolution; // m/cell
    double x_localmap = local_costmap.info.origin.position.x; // m
    double y_localmap = local_costmap.info.origin.position.y;  // m
    unsigned int size_localmap = local_costmap.data.size();

    double col_localmap = local_costmap.info.width; //cell
    double row_localmap = local_costmap.info.height;
    //ROS_INFO("Local map info:%f,%f,%f,%d,%f,%f",step,x_localmap,y_localmap,size_localmap,col_localmap,row_localmap);

    unsigned int x = (int)((xc-x_localmap)/step);  // cell
    unsigned int y = (int)((yc-y_localmap)/step);  // cell

    if((x>=col_localmap)||(y>=col_localmap))
    {
        flag = true;
    }else{
        unsigned int index = x + y*col_localmap;
        //ROS_INFO("x = %d, y = %d, index = %d, data = %d", x, y, index, local_costmap.data[index]);
        if(local_costmap.data[index]>0){
            flag = true;
            //ROS_INFO("x = %d, y = %d, index = %d, data = %d", x, y, index, local_costmap.data[index]);
        }
        else{
            flag = false;
            //ROS_INFO("x = %d, y = %d, index = %d, data = %d", x, y, index, local_costmap.data[index]);
        } // unknow when local_costmap.data[index] = -1 ???
    }
    return flag;
}

std::vector <geometry_msgs::Point> estimateApproachingAreas(double xc,double yc,double rc, double r_step, unsigned int npoints,std_msgs::Int8 goal_index){

    int8_t ind;
    ind = goal_index.data;

    geometry_msgs::Point approaching_point;
    std::vector <geometry_msgs::Point> approaching_areas;
    double xa=0, ya=0;

    for(int i=0;i<npoints;i++)
    {
        if((ind==2)||(ind==18)||(ind==21)||(ind==24)) // use this because of the approaching filter not very good at this time :)
        {
            if(ind==21)//[-3pi/4-pi/4]
            {
                int j = i-3*(npoints/4);
                xa = xc + (r_step+rc)*cos(j*M_PIl/(npoints));
                ya = yc + (r_step+rc)*sin(j*M_PIl/(npoints));
            }
            else //[-pi-pi]
            {
                int j = i-npoints/2;
                xa = xc + (r_step+rc)*cos(j*M_PIl/(npoints));
                ya = yc + (r_step+rc)*sin(j*M_PIl/(npoints));
            }
        }else{//[0-2pi]
            xa = xc + (r_step+rc)*cos(i*M_PIl/(0.5*npoints));
            ya = yc + (r_step+rc)*sin(i*M_PIl/(0.5*npoints));
        }

        bool flag = checkObstacles(local_costmap_, xa, ya);

        if(!flag)
        {
            approaching_point.x = xa; approaching_point.y = ya; approaching_point.z = i;
            approaching_areas.push_back(approaching_point);
            //ROS_INFO("flag = %d, id_point=%d",flag, id_point);
        }
    }
    return approaching_areas;

}

std::vector <geometry_msgs::Point> subFilterApproachingAreasByFiewOfView(std::vector <geometry_msgs::Point> approaching_areas,
                                                                         geometry_msgs::Quaternion quat_in,geometry_msgs::Point hp, string name)
{
    std::vector <geometry_msgs::Point> approaching_areas_fov;
    //double ang = convertQuaternionToAngle(human_states.pose[i].orientation);
    //geometry_msgs::Point hp = human_states.pose[i].position;
    //double ang = convertQuaternionToAngle(quat_in)-M_PI_2; // bản cũ của thầy

    double ang  = 0;

    if (name == "rp_denis" || name == "actor41")
        ang = convertQuaternionToAngle(quat_in); //+ M_PI_2; // Tiến edited
    else if (name == "rp_mei")
        ang = convertQuaternionToAngle(quat_in) - M_PI_2;
    else if (name == "rp_denis_0")
        ang = convertQuaternionToAngle(quat_in);
    else if (name == "person_standing")
        ang = convertQuaternionToAngle(quat_in)- M_PI_2;
    // else if (name == "kumaeye")
    //     ang = convertQuaternionToAngle(quat_in); // - M_PI_2;
    else if (name == "man02")
        ang = convertQuaternionToAngle(quat_in) - M_PI_2;
    else if (name == "female")
        ang = convertQuaternionToAngle(quat_in);


    for(int j=0;j<approaching_areas.size();j++)
    {
        geometry_msgs::Point point = approaching_areas[j];
        double dx1 = point.x - hp.x;
        double dy1 = point.y - hp.y;
        double ang1 = atan2(dy1,dx1);
        double diff1 = angles::shortest_angular_distance(ang,ang1);
        if(fabs(diff1)<M_PI/2.5){
            approaching_areas_fov.push_back(point);
        }
    }
    return approaching_areas_fov;
}


std::vector <geometry_msgs::Point> filterApproachingAreasByFiewOfView(std::vector <geometry_msgs::Point> approaching_areas,
                                                                      gazebo_msgs::ModelStates human_states, std_msgs::Int8 goal_index ){
    int8_t ind;
    ind = goal_index.data;
    std::vector <geometry_msgs::Point> approaching_areas_fov;

    //ROS_INFO("Me no chu mai khong debug duoc la sao: %lu", approaching_areas.size());

    for(int i = 0; i<human_states.name.size(); i++){
        //if(ind==0){
        // if((ind==0)||(ind==1)){
        //     if((human_states.name[i] == "p0")||(human_states.name[i] == "p1")){
        //         approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
        //         approaching_areas = approaching_areas_fov;
        //     }
        //     //approaching_areas = approaching_areas_fov;
        // }
        // if(ind==5){
        //     if((human_states.name[i] == "p2")||(human_states.name[i] == "p3")||(human_states.name[i] == "p4")){
        //         //approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
        //         approaching_areas_fov = approaching_areas;// because field of view
        //         approaching_areas = approaching_areas_fov;
        //     }
        //     //approaching_areas = approaching_areas_fov;
        // }
        // if(ind==2){
        //     if((human_states.name[i] == "p5")||(human_states.name[i] == "p6")||(human_states.name[i] == "p7")){
        //         approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
        //         approaching_areas = approaching_areas_fov;
        //     }
        //     //approaching_areas = approaching_areas_fov;
        // }
        // if(ind==8){
        //     if(human_states.name[i] == "p8"){
        //         approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
        //         approaching_areas = approaching_areas_fov;
        //     }
        //     //approaching_areas = approaching_areas_fov;
        // }
        // if(ind==12){
        //     if((human_states.name[i] == "p15")||(human_states.name[i] == "p16")){
        //         ROS_INFO("Buc minh qua:p15p16 ind = %d",goal_index_.data);
        //         approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
        //         approaching_areas = approaching_areas_fov;
        //     }
        //     //approaching_areas = approaching_areas_fov;
        // }
        // //if((ind==14)||(ind==15)){
        // if((ind==15)){
        //     if(human_states.name[i] == "p17"){
        //         approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
        //         //ROS_INFO("Buc minh qua:p17 ind = %d, inputsize = %lu, outputsize =%lu ",goal_index_.data,approaching_areas.size(),approaching_areas_fov.size());
        //         approaching_areas = approaching_areas_fov;
        //     }
        //     //approaching_areas = approaching_areas_fov;
        // }
        // if(ind==18){
        //     if(human_states.name[i] == "p9"){
        //         approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
        //         approaching_areas = approaching_areas_fov;
        //     }
        //     //approaching_areas = approaching_areas_fov;
        // }
        // if(ind==21){
        //     if((human_states.name[i] == "p10")||(human_states.name[i] == "p11")){
        //         approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
        //         approaching_areas = approaching_areas_fov;
        //     }
        //     //approaching_areas = approaching_areas_fov;
        // }
        // if(ind==24){
        //     if((human_states.name[i] == "p12")||(human_states.name[i] == "p13")){
        //         approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position);
        //         approaching_areas = approaching_areas_fov;
        //     }
        //     //approaching_areas = approaching_areas_fov;
        // }
        if(ind==0 ) // ind == 27
        {
            if(human_states.name[i] == "rp_denis" || human_states.name[i] == "rp_mei" || human_states.name[i] == "actor41" || human_states.name[i] == "female" || human_states.name[i] == "man02" || human_states.name[i] == "person_standing_0")
            {
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position, human_states.name[i]);
                approaching_areas = approaching_areas_fov;
            }
            if (human_states.name[i] == "person_standing" || human_states.name[i] == "rp_denis_0")
            {
                approaching_areas_fov = subFilterApproachingAreasByFiewOfView(approaching_areas,human_states.pose[i].orientation,human_states.pose[i].position, human_states.name[i]);
                approaching_areas = approaching_areas_fov;
                //approaching_areas_fov = approaching_areas;
            }
            //approaching_areas = approaching_areas_fov;
        }
    }
    return approaching_areas_fov;
}

// Select the potential approaching areas
std::vector<std::vector <geometry_msgs::Point> > selectPotentialApproachingAreas2D(std::vector <geometry_msgs::Point> approaching_areas, unsigned char minpoints){

    geometry_msgs::Point p1, p2;
    std::vector <geometry_msgs::Point> tmp1, tmp2;
    std::vector<std::vector <geometry_msgs::Point> > potential_ap_2d;

    unsigned int index=1;
    unsigned int npoint= 1;

    // separate the area
    for(int i=1;i<approaching_areas.size();i++)
    {
        p1 = approaching_areas[i-1];
        p2 = approaching_areas[i];

        if((p2.z-p1.z)>1){
        //if(((p2.z-p1.z)>2)&&((p2.z-p1.z)<200-2)){
            if(npoint>minpoints)
            {
                // add the center point to the end
                tmp1.push_back(tmp1[int(npoint/2)]);
                potential_ap_2d.push_back(tmp1);
            }
            npoint = 1;
            tmp1.clear();
            index++;
            p2.z = index;
            tmp1.push_back(p2);
        }
        else
        {
            p2.z = index;
            tmp1.push_back(p2);
            npoint++;
        }
    }
    // Add the last area
    if(npoint>minpoints){
        // add the center point to the end
        tmp1.push_back(tmp1[int(npoint/2)]);
        potential_ap_2d.push_back(tmp1);
    }
    ROS_INFO("Size of potential_ap_2d with one area: %lu", potential_ap_2d.size());
    ROS_INFO("Number of approaching areas: %d", index);

    return potential_ap_2d;
}

// from  "mybot_convert_laser_pointcloud"
void detectedHumanCallback(const gazebo_msgs::ModelStates::ConstPtr &msg){
    detected_human_states_ = *msg;
    ROS_INFO("Detected humans: %lu", msg->name.size());

}

geometry_msgs::Pose2D selectApproachingHumans(gazebo_msgs::ModelStates human_states, std_msgs::Int8 goal_index){

    geometry_msgs::Pose2D pose;
    std::vector <geometry_msgs::Pose2D> vpose;
    geometry_msgs::Point p0, p1;

    Circle circle;
    reals BEDataX[3]= {0,0,0};reals BEDataX1[3]= {0,0,0};reals BEDataX2[2]= {0,0};
    reals BEDataX3[2]= {0,0};reals BEDataX4[2]= {0,0};reals BEDataX5[3]= {0,0,0};
    reals BEDataY[3]= {0,0,0}; reals BEDataY1[3]= {0,0,0}; reals BEDataY2[2]= {0,0};
    reals BEDataY3[2]= {0,0}; reals BEDataY4[2]= {0,0}; reals BEDataY5[3]= {0,0,0};
    int8_t ind;

    geometry_msgs::Pose2D shgroup; //single human
    shgroup.theta = 0;
    ind = goal_index.data;

    int k =0; int k1 =0; int k2 =0; int k3 =0; int k4 =0; int k5 =0;
    for(int i = 0; i<human_states.name.size(); i++)
    {
        // //if(ind==0){
        // if((ind==0)||(ind==1))
        // {
        //     if((human_states.name[i] == "man02")||(human_states.name[i] == "p1")){
        //         if(human_states.name[i] == "man02"){p0=human_states.pose[i].position;}
        //         if(human_states.name[i] == "p1"){p1=human_states.pose[i].position;}
        //         k = k+ 1;
        //         if(k==2){
        //             double xg = (p0.x+p1.x)/2; double yg = (p0.y+p1.y)/2;
        //             shgroup.x = xg-0.35;//5.0
        //             shgroup.y = yg;//3.9
        //             shgroup.theta = 1.0; // 0.8661
        //         }
        //     }
        // }
        // if(ind==5){
        //     if((human_states.name[i] == "p2")||(human_states.name[i] == "p3")||(human_states.name[i] == "p4")){
        //         BEDataX[k1] = human_states.pose[i].position.x;
        //         BEDataY[k1] = human_states.pose[i].position.y;
        //         k1 = k1+ 1;
        //         if(k1==3){
        //             Data data1(3,BEDataX,BEDataY);
        //             cout.precision(7);
        //             circle = CircleFitByTaubin (data1);
        //             //cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
        //             shgroup.x = circle.a; shgroup.y = circle.b; shgroup.theta = circle.r+0.3;//0.2

        //         }
        //     }
        // }
        // if(ind==2){
        //     if((human_states.name[i] == "p5")||(human_states.name[i] == "p6")||(human_states.name[i] == "p7")){
        //         BEDataX1[k2] = human_states.pose[i].position.x;
        //         BEDataY1[k2] = human_states.pose[i].position.y;
        //         k2 = k2+ 1;
        //         if(k2==3){
        //             Data data1(3,BEDataX1,BEDataY1);
        //             cout.precision(7);
        //             circle = CircleFitByTaubin (data1);
        //             shgroup.x = circle.a; shgroup.y = circle.b; shgroup.theta = circle.r-0.15;//0
        //         }
        //     }
        // }
        // if(ind==8){
        //     if(human_states.name[i] == "p8"){
        //         shgroup.x = human_states.pose[i].position.x;
        //         shgroup.y = human_states.pose[i].position.y;
        //         shgroup.theta = 1.1;
        //     }
        // }
        // if(ind==12){
        //     if((human_states.name[i] == "p15")||(human_states.name[i] == "p16")){
        //         k3 = k3+ 1;
        //         if(k3==2){
        //             shgroup.x = -13.2;//-13.5
        //             shgroup.y = -5.6;//-5.5
        //             shgroup.theta = 0.9;
        //         }
        //     }
        // }
        // //if((ind==14)||(ind==15)){
        // if((ind==15)){
        //     if(human_states.name[i] == "p17"){
        //         shgroup.x = human_states.pose[i].position.x;
        //         shgroup.y = human_states.pose[i].position.y;
        //         shgroup.theta = 1.0;
        //     }
        // }
        // if(ind==18){
        //     if(human_states.name[i] == "p9"){
        //         shgroup.x = human_states.pose[i].position.x;
        //         shgroup.y = human_states.pose[i].position.y;
        //         shgroup.theta = 1.1;//1.1
        //     }
        // }
        // if(ind==21){
        //     if((human_states.name[i] == "p10")||(human_states.name[i] == "p11")){
        //         k4 = k4+ 1;
        //         if(k4==2){
        //             shgroup.x = -9.0;
        //             shgroup.y = 9.2;
        //             shgroup.theta = 0.85;
        //         }
        //     }
        // }
        // if(ind==24){
        //     if((human_states.name[i] == "p12")||(human_states.name[i] == "p13")){
        //         k5 = k5+ 1;
        //         if(k5==2){
        //             shgroup.x = -16.0;
        //             shgroup.y = 11.5;
        //             shgroup.theta = 0.9;
        //         }
        //     }
        // }

        if(ind==0) //if(ind==27)
        {        
            if((human_states.name[i] == "person_standing")||(human_states.name[i] == "rp_denis_0"))
            {
                  BEDataX[k1] = human_states.pose[i].position.x;
                  BEDataY[k1] = human_states.pose[i].position.y;
                  k1 = k1+ 1;

                  if(k1==2)
                  {
                      Data data1(2,BEDataX,BEDataY);
                      cout.precision(7);
                      circle = CircleFitByTaubin (data1);
                      //cout << "\n Taubin fit:  center ("<< circle.a <<","<< circle.b <<") radius "<< circle.r << "  sigma " << circle.s << endl;
                      shgroup.x = circle.a; shgroup.y = circle.b; shgroup.theta = circle.r;//0.2

                  }
            }

            if(human_states.name[i] == "rp_denis" || human_states.name[i] == "actor41" || human_states.name[i] == "man02" || human_states.name[i] == "female")
            {
                shgroup.x = human_states.pose[i].position.x;
                shgroup.y = human_states.pose[i].position.y;
                shgroup.theta = 1.0;// 1.1
            }

            if( human_states.name[i] == "rp_mei")
            {
                shgroup.x = human_states.pose[i].position.x;
                shgroup.y = human_states.pose[i].position.y;
                shgroup.theta = 1.12;// 1.1
            }


        }
    }
    return shgroup;
}

std::vector <geometry_msgs::Point> selectedApproachingArea(std::vector<std::vector <geometry_msgs::Point> > approaching_areas_2d, geometry_msgs::Pose2D cur_rpos){

    geometry_msgs::Point tmp;
    std::vector <geometry_msgs::Point> approaching_areas, selected_app_area;
    double max_dis = 1000000;
    int index = 1000000;
    //if(approaching_areas_2d.size()>1){
        for(int i=0;i<approaching_areas_2d.size();i++){
            approaching_areas = approaching_areas_2d[i];
            tmp = approaching_areas.back();// get the last element
            double dis = sqrt((cur_rpos.x-tmp.x)*(cur_rpos.x-tmp.x)+(cur_rpos.y-tmp.y)*(cur_rpos.y-tmp.y));
            if(dis<max_dis){
                max_dis = dis;
                index = i;
                selected_app_area=approaching_areas;
            }
        }
    //}
        return selected_app_area;
}

geometry_msgs::Pose2D calculateApproachingPose(std::vector <geometry_msgs::Point> selected_app_area, geometry_msgs::Pose2D center_area){

    geometry_msgs::Pose2D app_pose;
    geometry_msgs::Point tmp;
    tmp = selected_app_area.back();
    app_pose.x = tmp.x;
    app_pose.y = tmp.y;

    double angle = atan2((center_area.y-app_pose.y),(center_area.x-app_pose.x));
    app_pose.theta = angle;
    ROS_INFO("Approaching pose of robot: x= %f, y= %f, theta = %f", app_pose.x, app_pose.y, app_pose.theta);

    return app_pose;
}

visualization_msgs::Marker createMarkerPoint(float_t x, float_t y, float_t z, int id_point, int id_zone)
{
    visualization_msgs::Marker marker;
    //
    std::stringstream ss;
    ss << "approaching_area_" << id_zone;
    //
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ss.str();
    marker.id = id_point; // this number should be different
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.5); // the marker will be deleted after 0.5s
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.02;

    marker.color.a = 0.9;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 1;
    return marker;
}

void approachingPoseVisualize(geometry_msgs::Pose2D app_pose, int n_goal)
{
    visualization_msgs::Marker marker;
    geometry_msgs::Point p;

    std::stringstream ss;
    ss << "approaching_pose_" << n_goal;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ss.str();
    marker.id = n_goal; // this number should be different
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.5);

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;

    marker.scale.x = 0.05;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    //
    p.x = app_pose.x;
    p.y = app_pose.y;
    p.z = 0;
    marker.points.push_back(p);
    //
    p.x = app_pose.x + 0.5*cos(app_pose.theta);
    p.y = app_pose.y + 0.5*sin(app_pose.theta);
    p.z = 0;

    marker.points.push_back(p);
    app_pose_vis_pub.publish(marker);
    marker.points.clear();

}

//
visualization_msgs::Marker createMarkerHumanBody(float_t x, float_t y, float_t z, int id_point, int id_zone){

    visualization_msgs::Marker marker;
    //
    std::stringstream ss;
    ss << "human_body_" << id_zone;
    //
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ss.str();
    marker.id = id_point; // this number should be different
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.5); // the marker will be deleted after 0.5s
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 2*z;

    if (recognized_visualze_flag == false)
    {
        if ((numRoomRobot == 1 && numTargeRobot == 3) || (numRoomRobot == 2 && numTargeRobot == 6) || (numRoomRobot == 3 && numTargeRobot == 9))
        {
            marker.color.a = 1;
            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 0;
        }else
        {
            marker.color.a = 1;
            marker.color.r = 1;
            marker.color.g = 0;
            marker.color.b = 0;
        }

    }
    else
    {
        if ((numRoomRobot == 1 && numTargeRobot == 2)||(numRoomRobot == 2 && numTargeRobot == 5) || (numRoomRobot == 3 && numTargeRobot == 8))
        {
            marker.color.a = 1;
            marker.color.r = 0;
            marker.color.g = 1;
            marker.color.b = 0;
        }

    }

    return marker;
}

void modelStateCallback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
  gazebo_msgs::ModelStates human_states;

  visualization_msgs::Marker marker, hmarker;
  visualization_msgs::MarkerArray approaching_marker, hmarker_array;

  geometry_msgs::Pose2D app_pose, center_area;

  people_msgs::People people;
  people_msgs::Person person;

  // Robot pose
      double cur_rx, cur_ry, cur_rtheta;
      for(int i =0; i<msg->name.size(); i++){
          if(msg->name[i] == "diff_bot"){
              robot_state_.model_name = msg->name[i]; robot_state_.pose = msg->pose[i]; robot_state_.twist = msg->twist[i];
              cur_rtheta = convertQuaternionToAngle(msg->pose[i].orientation);
              cur_rx = msg->pose[i].position.x; cur_ry = msg->pose[i].position.y;

              cur_rpos_.x = cur_rx;
              cur_rpos_.y = cur_ry;
              cur_rpos_.theta = cur_rtheta;

              //ROS_INFO("Robot pose (x,y,theta) = (%f, %f, %f)", cur_rx, cur_ry, cur_rtheta);
              //ROS_INFO("Robot vel (lx,ly,lz,ax,ay,az) = (%f, %f, %f, %f, %f, %f, %f, %f)",msg->twist[i].linear.x,msg->twist[i].linear.y,
              //         msg->twist[i].linear.z,msg->twist[i].angular.x,msg->twist[i].angular.y,msg->twist[i].angular.z);
              break;
          }
      }

  // Model states
  //human_states = getHumanPoseFromGazebo(*msg);

  // human detected from Convert laser pointcloud

  human_states =  detected_human_states_;
  ROS_INFO("Number of Human: %lu", human_states.name.size());

  // I. Build Dynamic social zone
  // people
    for(int i =0; i<human_states.name.size(); i++)
     {
        double cur_ptheta;
        cur_ptheta = convertQuaternionToAngle(human_states.pose[i].orientation);

        //ROS_INFO("cur_ptheta_p%d=%f",k,cur_ptheta);
        person.position.x = human_states.pose[i].position.x;
        person.position.y = human_states.pose[i].position.y;
        person.position.z = 0;

        //1. Single person
        // Moving people
        if((human_states.name[i] == "actor1")||(human_states.name[i] == "actor2")||(human_states.name[i] == "actor3")||(human_states.name[i] == "actor4"))
        {
            if((human_states.name[i] == "actor1") || (human_states.name[i] == "actor2") || (human_states.name[i] == "actor3")|| (human_states.name[i] == "actor4"))
            {
                //if(goal_index_.data>14){ // when p17 is moving
                    person.velocity.x = 0.8*cos(cur_ptheta);// 0.25
                    person.velocity.y = 0.8*sin(cur_ptheta);
                    person.velocity.z = 0;
                //}else{
                //    person.velocity.x = 0.12*cos(cur_ptheta);
                //    person.velocity.y = 0.12*sin(cur_ptheta);
                //    person.velocity.z = 0;
                //}
            }
            else
            {
                person.velocity.x = 0.8*cos(cur_ptheta);// 0.25
                person.velocity.y = 0.8*sin(cur_ptheta);
                person.velocity.z = 0;
            }
        }
        else
        { // stationary people
            person.velocity.x = 0.12*cos(cur_ptheta);
            person.velocity.y = 0.12*sin(cur_ptheta);
            person.velocity.z = 0;
        }
        // sitting people
        if(human_states.name[i] == "female")
        {
            person.reliability = 0;
            person.velocity.x = 0.18*cos(cur_ptheta);
            person.velocity.y = 0.18*sin(cur_ptheta);
        }
        else
        {
            person.reliability = 1;
        }

        person.name = "people_info";
        people.people.push_back(person);
        if ((human_states.name[i] != "actor1") && (human_states.name[i] != "actor2") && (human_states.name[i] != "actor3") && (human_states.name[i] != "actor4"))
        {
            hmarker = createMarkerHumanBody(person.position.x, person.position.y, 0.6, i, 9999);
            hmarker_array.markers.push_back(hmarker);
        }

      //2. Object poeple
        int object_people_flag = true;
        if(object_people_flag)
        {
          if(human_states.name[i] == "person_standing_0")
          {
              //
            double cur_ptheta;
            cur_ptheta = convertQuaternionToAngle(human_states.pose[i].orientation);
            person.position.x = human_states.pose[i].position.x;
            person.position.y = human_states.pose[i].position.y;
            person.position.z = 0;

            // distance from human to object
            if(human_states.name[i] == "person_standing_0"){
                person.reliability = 1.9;
            }else
            {
                person.reliability = 2.2;
            }

            person.velocity.z = 0;
            person.velocity.x = 1*cos(cur_ptheta);
            person.velocity.y = 1*sin(cur_ptheta);
            person.name = "object_people_info";
            people.people.push_back(person);
          }
        }

     }
    // 3. Group people
     vhgroup_ = humanGroupDetection(human_states); // Trong trường hợp này coi như đã xác định được group, chính xác phải chạy code matlab to xác định

    int group_people_flag = true;
     if(group_people_flag)
     {
       for(int i=0;i<vhgroup_.size();i++)
       {
         geometry_msgs::Pose2D hgroup_info = vhgroup_[i];
         person.position.x = hgroup_info.x;
         person.position.y = hgroup_info.y;
         person.position.z = 0;

         person.velocity.x = 0;
         person.velocity.y = 0;
         person.velocity.z = 0;
         // radius of the human group
         person.reliability = hgroup_info.theta;
         person.name = "group_people_info";
         people.people.push_back(person);
       }
     }

  // II- Find the o-space of a group

  ROS_INFO("Estimate human group");
  geometry_msgs::Pose2D hgroup, shgroup;
  goal_index_.data = 0;

  // Lựa chọn người để tiếp cận dựa vào landmark của các phòng
  if (human_states.name.size()> 0)
    shgroup = selectApproachingHumans(human_states, goal_index_);
  // shgroup = vhgroup_[0];

  std::vector <geometry_msgs::Point> approaching_areas, approaching_areas1, approaching_areas2;
  std::vector <geometry_msgs::Point> selected_app_area;
  std::vector<std::vector <geometry_msgs::Point> > approaching_areas_2d;
  geometry_msgs::Point  approaching_point, p1;

  double r_step = 0.1; // use to increase the radius of the approaching area
  bool approaching_flag = true;

  if((approaching_flag)&&(shgroup.theta!=0))
  {
      hgroup = shgroup;
      unsigned char n_rstep = 2;// control the step of rstep

      while((approaching_areas_2d.empty())&&(n_rstep<6))
       {
           approaching_areas = estimateApproachingAreas(hgroup.x, hgroup.y, hgroup.theta, r_step*n_rstep,200,goal_index_);
           ROS_INFO("Size of approaching areas: filtered dsz = %lu",approaching_areas.size());

           // Loại các điểm thừa - ko nằm trong field of view
           approaching_areas = filterApproachingAreasByFiewOfView(approaching_areas, human_states, goal_index_);

           // filter the area with small number of points
           //approaching_areas1 = selectPotentialApproachingAreas(approaching_areas);

           // Tìm ra các vùng tiếp cận riêng rẽ: mấy vùng  và gồm những vùng nào
           approaching_areas_2d = selectPotentialApproachingAreas2D(approaching_areas,9);
           ROS_INFO("Size of approaching areas: filtered fov = %lu, filtered numberofpoint = %lu", approaching_areas.size(), approaching_areas_2d.size());

           //ROS_INFO("Number of rstep: %d", n_rstep);
           n_rstep++;

           approaching_areas.clear();
       }

      // Tạm đóng để debug
      if(!approaching_areas_2d.empty())
       {
           selected_app_area = selectedApproachingArea(approaching_areas_2d, cur_rpos_);
           app_pose = calculateApproachingPose(selected_app_area, hgroup);
           //app_pose = predictApproachingPoseOfMovingHumans(app_pose);

           app_pose_ = app_pose;
           if (appr_visualize_flag == true)
                approachingPoseVisualize(app_pose, 99999);
           //
           int id_point=0;
           int id_approaching =0;
           //approaching_areas2 = selected_app_area;

           for(int m=0;m<approaching_areas_2d.size();m++)
           {
               approaching_areas2 = approaching_areas_2d[m];
               for(int n=0;n<approaching_areas2.size()-1;n++)
               {// (approaching_areas2.size()-1) because the end element is the center point
                   p1 = approaching_areas2[n];
                   marker=createMarkerPoint(p1.x,p1.y,0,id_point,id_approaching);
                   approaching_marker.markers.push_back(marker);

                   //ROS_INFO("Aproaching areas:x= %f,y= %f,index= %f,nrstep = %d",p1.x, p1.y, p1.z,n_rstep);
                   id_point++;
               }
           }
           approaching_areas_2d.clear();
       }
       else
       {
       ROS_INFO("Could not find the suitable approaching pose :(");
       }

  }
  //
  people.header.frame_id = "map"; // global map
  //if (!human_pose_pub)
  human_pose_pub.publish(people);
  detected_human_vis_pub.publish(hmarker_array);

  if (appr_visualize_flag == true)
      approaching_pub.publish(approaching_marker);

  approaching_marker.markers.clear();

  //
}
void apprPoseCommandCallback(std_msgs::Bool msg)
{
    appr_visualize_flag = msg.data;
}

void endApprPoseCommandCallback(std_msgs::Bool msg)
{
    appr_visualize_flag = msg.data;
    recognized_visualze_flag = msg.data;
}
//
bool getApproachingPose(social_navigation_layers::GetApproachingPose::Request &req,social_navigation_layers::GetApproachingPose::Response &res)
{
    res.index = goal_index_;
    res.app_pose = app_pose_;

    return true;
}
//
void goalIndexCallback(social_navigation_layers::RobotPosition msg)
{
    numRoomRobot = msg.room;

    numTargeRobot = msg.goal_index;
}

//
void recognizedHumanVisCallback(std_msgs::Bool msg)
{
    recognized_visualze_flag = msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "approaching_pose_dsz_node");
  ros::NodeHandle nh;

  app_pose_service = nh.advertiseService("/get_approaching_pose", getApproachingPose);

  appr_visualize_flag = false;

  // Publish topic
  human_pose_pub = nh.advertise<people_msgs::People>("/human_information", 5);  // pub to local_costmap to add layers
  approaching_pub = nh.advertise<visualization_msgs::MarkerArray>("/mybot_description/approaching_areas",0);
  app_pose_vis_pub = nh.advertise<visualization_msgs::Marker>("mybot_description/approaching_pose_vis",0);
  app_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/mybot_description/approaching_pose",5);
  detected_human_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/mybot_description/detected_human_vis",0);


  // Subscribe topic from other nodes
  mode_state_sub = nh.subscribe("/gazebo/model_states", 100, &modelStateCallback);
  local_cosmap_sub = nh.subscribe("/move_base/local_costmap/costmap",5,&localCostmapCallback);
  detected_human_sub = nh.subscribe("/mybot_description/detected_human",10,&detectedHumanCallback);
  approach_pose_command_sub = nh.subscribe("/detect_and_recog/appr_pose_command", 10, apprPoseCommandCallback);
  end_approach_pose_command_sub = nh.subscribe("/send_goal_pose/end_approach_pose",10, endApprPoseCommandCallback);
  recognized_human_vis_sub = nh.subscribe("/detect_and_recog/appr_pose_command", 10, recognizedHumanVisCallback);
  goal_index_sub = nh.subscribe("/send_goal_pose/goal_index", 100, goalIndexCallback);
  ros::Rate rate(30);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
