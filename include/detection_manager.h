/*
 * detection_manager.h
 *
 *  Created on: May 7, 2016
 *      Author: teddy
 */

#ifndef WEED_DETECTION_INCLUDE_DETECTION_MANAGER_H_
#define WEED_DETECTION_INCLUDE_DETECTION_MANAGER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#include "weed_detection/detected_balls.h"


struct registered_ball {
  size_t ID;
  float predicted_orientation;
  float distance_to_reach;
  float extra_distance_by_offset;
  bool sprayed;
  double last_update;
  /*
   * Camera that catched the ball.
   * Front Camera = 0
   * Arm Camera = 1
   */
  int source;
};

/*
 * functor to compare ids
 * TODO try lambda (c++11)
 */
class idMatcher
{
  size_t _ID;

public:
  idMatcher(const size_t &ID) : _ID(ID) {}

  bool operator()(const registered_ball &item) const
  {
    return item.ID == _ID;
  }
};

class detectionManager{

public:
detectionManager();
void front_detection_callback(const weed_detection::detected_balls::ConstPtr& msg);
void arm_detection_callback(const geometry_msgs::Pose2D::ConstPtr& msg);
void arm_state_callback(const sensor_msgs::JointState::ConstPtr& msg);

private:
float computeOrientation(geometry_msgs::Pose pose);
void processNearestBall();
void processDatabase();
void reachBall();

ros::NodeHandle node_;
ros::NodeHandle pnh_;
ros::Subscriber front_detections_sub_;
ros::Subscriber arm_detections_sub_;
ros::Publisher move_to_upcoming_pub_;
ros::Publisher move_precisely_;
ros::Publisher robot_vel_;
ros::Subscriber arm_state_sub_;
ros::ServiceClient spray_;

std::vector<registered_ball> ball_database_;
int ball_life;
int arm_lenght;
int y_calibration_offset_;
int discard_threshold_;
int stop_threshold_;
int slow_threshold_;
int normal_vel_;
double arm_angle_;
double spray_threshold_;
std::pair<double, double> arm_pose_;
geometry_msgs::Pose2D nearest_ball_pose_;
geometry_msgs::Pose2D sprayer_pose_;

double front_cam_angle_thres_;


registered_ball* nearest_ball;

};


#endif /* WEED_DETECTION_INCLUDE_DETECTION_MANAGER_H_ */
