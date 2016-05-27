#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"

#include "ball_detector.h"
using namespace cv;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_camera_node");

  ros::NodeHandle n;
  ros::NodeHandle pn  = ros::NodeHandle("~");

  ros::Publisher arm_detections_pub = n.advertise<geometry_msgs::Pose2D>("arm_detections", 1000);

  ros::Rate loop_rate(30);

  BallDetector ballDetector("arm_camera");

  if (ballDetector.initialize_camera() == -1)
  {
    return -1;
  }

  double arm_camera_position_x;
  if (!pn.hasParam("arm_camera_position_x"))
    ROS_WARN_STREAM("Used default parameter for arm_camera_position_x [0]");
  pn.param("arm_camera_position_x", arm_camera_position_x, 0.0);

  double arm_camera_position_y;
  if (!pn.hasParam("arm_camera_position_y"))
    ROS_WARN_STREAM("Used default parameter for arm_camera_position_y [0]");
  pn.param("arm_camera_position_y", arm_camera_position_y, 0.0);

  long count = 0;
  while (ros::ok())
  {
    vector<tracked_ball>* balls = ballDetector.processFrame();

    if (balls->size() > 0)
    {
      geometry_msgs::Pose2D msg;
      msg.x = balls->at(0).location.x;
      msg.y = balls->at(0).location.y;
      for (int i = 1; i < balls->size(); i++)
      {
        if ((pow(balls->at(i).location.x - arm_camera_position_x, 2) + pow(balls->at(i).location.y - arm_camera_position_y, 2)) <
            (pow(msg.x - arm_camera_position_x, 2) + pow(msg.y - arm_camera_position_y, 2)))
        {
          msg.x = balls->at(i).location.x;
          msg.y = balls->at(i).location.y;
        }
      }
      arm_detections_pub.publish(msg);
    }


    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }


  return 0;
}
