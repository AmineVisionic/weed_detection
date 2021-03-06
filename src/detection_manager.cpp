#include "detection_manager.h"

//weed_detection::detected_balls front_detected_;


detectionManager::detectionManager() : arm_angle_(0), pnh_("~")
{
  nearest_ball = NULL;
  ball_life = 2;
  arm_lenght = 400;
  y_calibration_offset_ = 0;
  stop_threshold_ = 0;
  slow_threshold_ = 0;
  normal_vel_ = 0;
  front_cam_angle_thres_ = 5;
  spray_threshold_ = 0;
  arm_drive_height_ = 0;
  arm_spray_height_ = -0.1;
  max_pan_angle_ = 0.5;

  if (!pnh_.hasParam("ball_life"))
    ROS_WARN_STREAM("[Detection Manager] Used default parameter for ball_life [2]");
  pnh_.param("ball_life", ball_life, 2);

  if (!pnh_.hasParam("arm_lenght"))
    ROS_WARN_STREAM("[Detection Manager] Used default parameter for arm_lenght [400]");
  pnh_.param("arm_lenght", arm_lenght, 400);

  if (!pnh_.hasParam("y_calibration_offset_"))
    ROS_WARN_STREAM("[Detection Manager] Used default parameter for y_calibration_offset_ [350]");
  pnh_.param("y_calibration_offset_", y_calibration_offset_, 350);

  if (!pnh_.hasParam("stop_threshold_"))
    ROS_WARN_STREAM("[Detection Manager] Used default parameter for stop_threshold_ [0]");
  pnh_.param("stop_threshold_", stop_threshold_, 0);

  if (!pnh_.hasParam("slow_threshold_"))
    ROS_WARN_STREAM("[Detection Manager] Used default parameter for slow_threshold_ [0]");
  pnh_.param("slow_threshold_", slow_threshold_, 0);

  if (!pnh_.hasParam("normal_vel_"))
    ROS_WARN_STREAM("[Detection Manager] Used default parameter for normal_vel_ [0]");
  pnh_.param("normal_vel_", normal_vel_, 0);

  if (!pnh_.hasParam("front_cam_angle_thres_"))
    ROS_WARN_STREAM("[Detection Manager] Used default parameter for front_cam_angle_thres_ [0]");
  pnh_.param("front_cam_angle_thres_", front_cam_angle_thres_, 5.0);

  if (!pnh_.hasParam("spray_threshold_"))
    ROS_WARN_STREAM("[Detection Manager] Used default parameter for spray_threshold_ [0]");
  pnh_.param("spray_threshold_", spray_threshold_, 0.0);

  if (!pnh_.hasParam("discard_threshold_"))
    ROS_WARN_STREAM("[Detection Manager] Used default parameter for discard_threshold_ [-10]");
  pnh_.param("discard_threshold_", discard_threshold_, -10);

  if (!pnh_.hasParam("arm_drive_height"))
    ROS_WARN_STREAM("[Detection Manager] Used default parameter for arm_drive_height [0]");
  pnh_.param("arm_drive_height", arm_drive_height_, 0.0);

  if (!pnh_.hasParam("arm_spray_height"))
    ROS_WARN_STREAM("[Detection Manager] Used default parameter for arm_spray_height [-0.05]");
  pnh_.param("arm_spray_height", arm_spray_height_, -0.05);

  if (!pnh_.hasParam("max_pan_angle"))
    ROS_WARN_STREAM("[Detection Manager] Used default parameter for max_pan_angle [0.5]");
  pnh_.param("max_pan_angle", max_pan_angle_, 0.5);


  front_detections_sub_ = node_.subscribe<weed_detection::detected_balls>("front_detections", 1, &detectionManager::front_detection_callback, this);
  arm_detections_sub_ = node_.subscribe<geometry_msgs::Pose2D>("arm_detections", 1, &detectionManager::arm_detection_callback, this);
  arm_target_pub_ = node_.advertise<sensor_msgs::JointState>("/ptu/cmd", 1, true);
  move_precisely_ = node_.advertise<geometry_msgs::Pose2D>("move_precisely", 1, true);
  robot_vel_ = node_.advertise<std_msgs::Float64>("robot_velocity", 1, true);
  arm_state_sub_ = node_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &detectionManager::arm_state_callback, this);
  //spray_ = node_.serviceClient<std_srvs::Trigger>("spray");
  process_database_timer_ = pnh_.createTimer(ros::Duration(1/30.0), &detectionManager::processDatabase, this);
  //process_database_timer_.start();
}

void detectionManager::arm_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  arm_angle_ = msg->position.at(0);
}

void detectionManager::arm_detection_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  nearest_ball_pose_ = *msg;
}

float detectionManager::computeOrientation(geometry_msgs::Pose pose)
{
  return max_pan_angle_ * (pose.position.x - 320)/320 *(-1);
}

void detectionManager::reachBall()
{
  // If reached ball, spray
//  if(abs(nearest_ball_pose_.x - sprayer_pose_.x) < spray_threshold_ &&
//      abs(nearest_ball_pose_.y - sprayer_pose_.y) < spray_threshold_)
//  {
//    siren();
//    ROS_DEBUG_STREAM_NAMED("control_flow", "Spray!");
//    //std_srvs::Trigger spray;
//    spray();
//    nearest_ball->sprayed = true;
//  }
  if(//abs(nearest_ball->orientation - sprayer_pose_.x) < 0.1 &&
      nearest_ball->distance_to_reach < 10)
  {
//    if (nearest_ball_pose_.x > 20 && nearest_ball_pose_.x < 620 && nearest_ball_pose_.y > 20 && nearest_ball_pose_.y < 220)
//    {
    siren();
    ROS_DEBUG_STREAM_NAMED("control_flow", "Spray!");
    //std_srvs::Trigger spray;
    spray();
//    }
    nearest_ball->sprayed = true;
  }

  // If outside front cam "trusted range", use front cam info to move
//  if (abs(arm_lenght * sin(arm_angle_) - nearest_ball->orientation) > front_cam_angle_thres_)
//  {
    //ROS_DEBUG_STREAM_NAMED("control_flow", "Front cam based move");
    sensor_msgs::JointState msg;
    std::vector<double> position;
    position.push_back(nearest_ball->orientation);
    position.push_back(0);
    msg.position = position;
    std::vector<double> vel;
    vel.push_back(0.5);
    vel.push_back(0.5);
    msg.velocity = vel;
    arm_target_pub_.publish(msg);
//  }
//  // else use arm cam
//  else
//  {
//    //move_precisely_.publish(nearest_ball_pose_);
//    ROS_DEBUG_STREAM_NAMED("control_flow", "Arm cam");
//  }

}


void detectionManager::processNearestBall()
{
  if (nearest_ball == NULL)
  {
    ROS_DEBUG_STREAM_NAMED("processNearestBall", "[processNearestBall] Skipping because there is no nearest_ball");
    return;
  }
  ROS_DEBUG_STREAM_NAMED("processNearestBall", "[processNearestBall] Distance to reach nearest ball: " << nearest_ball->distance_to_reach << " (" << nearest_ball->ID << ")");
  // Slow down or stop the robot depending on distance_to_reach
  if (nearest_ball->distance_to_reach < stop_threshold_)
  {
    ROS_DEBUG_STREAM_NAMED("processNearestBall", "[processNearestBall] Stopping robot");
    std_msgs::Float64 msg;
    msg.data = 0.0;
    robot_vel_.publish(msg);
  }
  else if (nearest_ball->distance_to_reach < slow_threshold_)
  {
    float vel_ratio = (float)nearest_ball->distance_to_reach / (slow_threshold_ - stop_threshold_);
    ROS_DEBUG_STREAM_NAMED("processNearestBall", "[processNearestBall] Slowing robot (" << vel_ratio << ")");
    std_msgs::Float64 msg;
    msg.data = normal_vel_ * vel_ratio;
    robot_vel_.publish(msg);
  }

  //ROS_DEBUG_STREAM_NAMED("processNearestBall", "[processNearestBall] ...");
  // If nearest ball is outside the working range of the arm
//  if (nearest_ball->distance_to_reach > nearest_ball->extra_distance_by_offset)
//  {
//    sensor_msgs::JointState msg;
//    std::vector<double> position;
//    position.push_back(0);
//    position.push_back(nearest_ball->orientation);
//    position.push_back(0);
//    msg.position = position;
//    arm_target_pub_.publish(msg);
//  }
//  else
//  {
//    reachBall();
//  }
  reachBall();
}

void detectionManager::processDatabase(const ros::TimerEvent& event)
{
  ROS_DEBUG_STREAM_NAMED("processDatabase", "[processDatabase] Processing " << ball_database_.size() << " balls...");
  for (int i = 0; i < ball_database_.size(); i++)
  {
    ROS_DEBUG_STREAM_NAMED("processDatabase", "Processing ball " << ball_database_.at(i).ID);
    //Delete ball if outdated
    if (ros::Time::now().toSec() - ball_database_.at(i).last_update > ball_life)
      {
      ROS_DEBUG_STREAM_NAMED("processDatabase", "[processDatabase] Deleted ball "<< ball_database_.at(i).ID);
      //delete &ball_database_.at(i);
      ball_database_.erase(ball_database_.begin() + i);
      i--;
      }
    else
    {
      if (nearest_ball != NULL)
            {
            if (nearest_ball->sprayed)
              nearest_ball = NULL;
            }
      //check if i is the first ball to be reached by the arm and hasn't been allready sprayed
      if (nearest_ball == NULL && !ball_database_.at(i).sprayed && ball_database_.at(i).distance_to_reach > discard_threshold_)
      {
        nearest_ball = &ball_database_.at(i);
      }
      else if (nearest_ball != NULL)
      {
      if ((ball_database_.at(i).distance_to_reach < nearest_ball->distance_to_reach) && ball_database_.at(i).distance_to_reach > discard_threshold_ && !ball_database_.at(i).sprayed)
      {
        nearest_ball = &ball_database_.at(i);
      }
      ROS_DEBUG_STREAM_NAMED("processDatabase", "[processDatabase] Nearest ball is " << nearest_ball->ID);
      }
      }
  }
  processNearestBall();
}


//auto pred = [id](const registered_ball & item) {
//    return item.ID == id;
//};

void detectionManager::front_detection_callback(const weed_detection::detected_balls::ConstPtr& msg){
  //TODO Mutex

  ROS_DEBUG_STREAM_NAMED("front_callback", "Received a message with " << msg->ids.size() << " detections.");
  for (int i = 0; i < msg->ids.size(); i++)
  {
    std::vector<registered_ball>::iterator it = std::find_if(ball_database_.begin(),
                                                             ball_database_.end(),
                                                             idMatcher(msg->ids.at(i)));
    if (it != ball_database_.end())
    {
    /*
     * Compute the distance in movement direction from the ball to the working range of the arm
     */
      it->extra_distance_by_offset = arm_lenght - sqrt(pow(arm_lenght, 2) -
                                                       pow(abs(msg->predictions.at(i).position.x - 320), 2));
      it->distance_to_reach = (y_calibration_offset_ - msg->locations.at(i).position.y) + it->extra_distance_by_offset;
      if (it->distance_to_reach == 0)
      {
        ROS_DEBUG_STREAM_NAMED("control_flow", "y pos " << msg->locations.at(i).position.y << " extra dist " << it->extra_distance_by_offset);
      }
      it->orientation = computeOrientation(msg->locations.at(i));
      it->last_update = ros::Time::now().toSec();
    }
    else
    {
      registered_ball new_entry;
      new_entry.ID = msg->ids.at(i);
      new_entry.extra_distance_by_offset = arm_lenght - sqrt(pow(arm_lenght, 2) -
                                                             pow(abs(msg->locations.at(i).position.x - 320), 2));
      new_entry.distance_to_reach = (y_calibration_offset_ - msg->locations.at(i).position.y) + new_entry.extra_distance_by_offset;
      if (new_entry.distance_to_reach == 0)
      {
        ROS_DEBUG_STREAM_NAMED("control_flow", "y pos " << msg->locations.at(i).position.y << " extra dist " << new_entry.extra_distance_by_offset);
      }
      new_entry.orientation = computeOrientation(msg->locations.at(i));
      new_entry.last_update = ros::Time::now().toSec();
      new_entry.sprayed = false;
      ball_database_.push_back(new_entry);
      //ROS_DEBUG_STREAM_NAMED("control_flow", "hi");
    }
  }
  //processDatabase();
}

void detectionManager::siren()
{
  ;
}

void detectionManager::spray()
{
  ROS_DEBUG_STREAM("Spraying");
  sensor_msgs::JointState msg;
  std::vector<double> position;
  position.push_back(arm_angle_);
  position.push_back(arm_spray_height_);
  msg.position = position;
  std::vector<double> vel;
  vel.push_back(0.5);
  vel.push_back(0.5);
  msg.velocity = vel;
  arm_target_pub_.publish(msg);
  ros::Duration(0.5).sleep();
  position.clear();
  position.push_back(arm_angle_);
  position.push_back(arm_drive_height_);
  msg.position = position;
  arm_target_pub_.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection_manager");
  detectionManager dmNode;

//  ros::Rate loop_rate(30);
//
//  while (ros::ok())
//  {
//
//
//    ros::spinOnce();
//
//    loop_rate.sleep();
//  }
  ros::spin();

  return 0;
}
