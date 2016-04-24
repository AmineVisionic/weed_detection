#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud.h"

#include "ball_detector.h"
using namespace cv;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "front_camera_node");

  ros::NodeHandle n;

  ros::Publisher front_detections_pub = n.advertise<std_msgs::String>("front_detections", 1000);

  ros::Rate loop_rate(10);

  BallDetector ballDetector("front_camera");
  if (ballDetector.initialize_camera() == -1)
    return -1;


  long count = 0;
  while (ros::ok())
  {
    const std::vector<cv::Point2f>* detected_balls = ballDetector.processFrame();
    sensor_msgs::PointCloud msg;
    msg.header.seq = count;
    for (int i = 0; i < detected_balls->size(); i++)
    {
      geometry_msgs::Point32 point;
      point.x = detected_balls->at(i).x;
      point.y = detected_balls->at(i).y;
      msg.points.push_back(point);
    }
    front_detections_pub.publish(msg);


    ros::spinOnce();

    loop_rate.sleep();
    count++;
  }


  return 0;
}
