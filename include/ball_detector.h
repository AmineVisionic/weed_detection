#include "ros/ros.h"
#include "std_msgs/String.h"

#include <opencv2/opencv.hpp>
#include <highgui.h>

#include "Ctracker.h"

#define DEBUG

struct tracked_ball{
  size_t id;
  Point2d location;
  Point2d prediction;
};

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
  vector<tracked_ball>* processFrame();


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
                                                         cv::Mat* frame);

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
  double kalman_dt_;
  double kalman_accel_noise_mag_;
  double hungarian_dist_thres_;
  int hungarian_max_skipped_frames_;

  int kalman_dt_int_;
  int kalman_accel_noise_mag_int_;
  int hungarian_dist_thres_int_;

  int detect_manager_yoffset_;
  int detect_manager_arm_lenght_;

  std::string name_;

  cv::VideoCapture* cam_;

  CTracker* tracker_;

};
