#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/ModelStates.h>
#include <vector>
#include <cmath>

typedef double reals;

struct Circle {
  double a, b, r, s;
};

struct Data {
  int n;
  reals* x;
  reals* y;
  Data(int n_, reals* x_, reals* y_) : n(n_), x(x_), y(y_) {}
};

Circle CircleFitByTaubin(const Data& data) {
  // Implement circle fitting algorithm here
  // For now, return a dummy circle
  return {0.0, 0.0, 1.0, 0.0};
}

std::vector<geometry_msgs::Pose2D> humanGroupDetection(const gazebo_msgs::ModelStates& human_states) {
  geometry_msgs::Pose2D pose;
  std::vector<geometry_msgs::Pose2D> vpose;

  Circle circle;
  reals BEDataX[3] = {0, 0, 0};
  reals BEDataX1[3] = {0, 0, 0};
  reals BEDataX2[2] = {0, 0};
  reals BEDataX3[2] = {0, 0};
  reals BEDataX4[2] = {0, 0};
  reals BEDataX5[3] = {0, 0, 0};
  reals BEDataY[3] = {0, 0, 0};
  reals BEDataY1[3] = {0, 0, 0};
  reals BEDataY2[2] = {0, 0};
  reals BEDataY3[2] = {0, 0};
  reals BEDataY4[2] = {0, 0};
  reals BEDataY5[3] = {0, 0, 0};

  int k = 0;
  int k1 = 0;
  int k2 = 0;
  int k3 = 0;
  int k4 = 0;
  int k5 = 0;

  for (int i = 0; i < human_states.name.size(); i++) {
    if ((human_states.name[i] == "person_standing") || (human_states.name[i] == "rp_denis_0")) {
      BEDataX2[k] = human_states.pose[i].position.x;
      BEDataY2[k] = human_states.pose[i].position.y;

      k = k + 1;
      if (k == 2) {
        pose.x = (BEDataX2[0] + BEDataX2[1]) / 2;
        pose.y = (BEDataY2[0] + BEDataY2[1]) / 2;

        double deltaX = BEDataX2[0] - BEDataX2[1];
        double deltaY = BEDataY2[0] - BEDataY2[1];

        pose.theta = sqrt(deltaX * deltaX + deltaY * deltaY) / 2;
        vpose.push_back(pose);
      }
    }

    if ((human_states.name[i] == "actor1") || (human_states.name[i] == "actor2") || (human_states.name[i] == "actor3")) {
      BEDataX1[k1] = human_states.pose[i].position.x;
      BEDataY1[k1] = human_states.pose[i].position.y;
      k1 = k1 + 1;
      if (k1 == 3) {
        Data data1(3, BEDataX1, BEDataY1);
        circle = CircleFitByTaubin(data1);
        pose.x = circle.a;
        pose.y = circle.b;
        pose.theta = circle.r;
        vpose.push_back(pose);
      }
    }
  }
  return vpose;
}

ros::Publisher group_center_pub;
ros::Publisher group_radius_pub;

void modelStateCallback(const gazebo_msgs::ModelStatesConstPtr& msg) {
  std::vector<geometry_msgs::Pose2D> group_poses = humanGroupDetection(*msg);

  if (!group_poses.empty()) {
    geometry_msgs::Point center;
    center.x = group_poses[0].x;
    center.y = group_poses[0].y;
    group_center_pub.publish(center);

    std_msgs::Float64 radius_msg;
    radius_msg.data = group_poses[0].theta;
    group_radius_pub.publish(radius_msg);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "human_group_layer_from_gazebo");
  ros::NodeHandle nh;

  group_center_pub = nh.advertise<geometry_msgs::Point>("group_center", 10);
  group_radius_pub = nh.advertise<std_msgs::Float64>("group_radius", 10);

  ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, modelStateCallback);

  ros::spin();

  return 0;
}
