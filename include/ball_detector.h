#include "ros/ros.h"
#include "std_msgs/String.h"

#include <opencv2/opencv.hpp>
//#include <highgui.h>

#define DEBUG

class BallDetector{
public:
  BallDetector(std::string name);

  /*
   * Initializes the camera. Not initialized in constructor to give back if succeeded.
   */
  int initialize_camera();

  /*
   * Applies various filters on the image to extract the areas of the right color.
   */
  std::vector<cv::Point2f>* processFrame();


private:

  void loadParameters();

  /*
   * Builds the settings window.
   */
  void settingsWindow();

  /*
   * Detects the balls in the areas defined by processFrame
   */
  std::vector<cv::Point2f>* processContours(std::vector<std::vector<cv::Point> > contours,
                                                         std::vector<cv::Vec4i> hierarchy,
                                                         cv::Mat frame);

  ros::NodeHandle nh_;

  int camera_device_;
  int H_min_;
  int H_max_;
  int S_min_;
  int S_max_;
  int V_min_;
  int V_max_;
  int min_contour_area_;
  int min_contour_circle_radius_;
  int max_contour_circle_radius_;
  int min_area_ratio_;
  bool show_gui_;

  std::string name_;

  cv::VideoCapture capture_;

};
