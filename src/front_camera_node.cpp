#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose.h"

#include "ball_detector.h"
#include "weed_detection/detected_balls.h"
using namespace cv;

int main(int argc, char **argv)
{
  // Initialize the ROS-Node
  ros::init(argc, argv, "front_camera_node");

  ros::NodeHandle n;

  // Advertise the topic for the front detections
  ros::Publisher front_detections_pub = n.advertise<weed_detection::detected_balls>("front_detections", 1000);

  // Define the loop rate the same as video FPS
  ros::Rate loop_rate(30);

  // Initialize the ballDetector object
  BallDetector ballDetector("front_camera");

  // Initialize the camera and check if there where some errors
  if (ballDetector.initialize_camera() == -1)
  {
    return -1;
  }

  long count = 0;
  while (ros::ok())
  {
    // Get the location of the balls in the current frame
    vector<tracked_ball>* balls = ballDetector.processFrame();

    // Save the location info in a ROS-Message
    weed_detection::detected_balls msg;
    msg.header.seq = count;
    for (int i = 0; i < balls->size(); i++)
    {
      msg.ids.push_back(balls->at(i).id);

      geometry_msgs::Pose pose;
      pose.position.x = balls->at(i).location.x;
      pose.position.y = balls->at(i).location.y;
      msg.locations.push_back(pose);

      geometry_msgs::Pose predict;
      predict.position.x = balls->at(i).prediction.x;
      predict.position.y = balls->at(i).prediction.y;
      msg.predictions.push_back(predict);
    }
    // Send the message
    front_detections_pub.publish(msg);


    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }


  return 0;
}
