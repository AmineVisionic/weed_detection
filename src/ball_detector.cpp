#include "ball_detector.h"

using namespace cv;

BallDetector::BallDetector(std::string name)
{
  name_ = name;
  nh_ = ros::NodeHandle("~");
  loadParameters();
  if (show_gui_)
    settingsWindow();
}

int BallDetector::initialize_camera()
{
  VideoCapture capture_(camera_device_); // start the camera
  if (!capture_.isOpened())  // check if camera works
    ROS_ERROR("Camera device not found.");
    ROS_WARN("Camera device not found.");
    return -1;
  return 0;
}

void BallDetector::loadParameters()
{
  if (!nh_.hasParam("camera_device"))
    ROS_WARN_STREAM("[" << name_ << "] Used default parameter for camera_device [0]");
  nh_.param("camera_device", camera_device_, 0);

  //Setting HSV threshold values
  if (!nh_.hasParam("H_min"))
    ROS_WARN_STREAM("[" << name_ << "] Used default parameter for H_min [0]");
  nh_.param("H_min", H_min_, 0);

  if (!nh_.hasParam("H_max"))
    ROS_WARN_STREAM("[" << name_ << "] Used default parameter for H_max [0]");
  nh_.param("H_max", H_max_, 0);

  if (!nh_.hasParam("S_min"))
    ROS_WARN_STREAM("[" << name_ << "] Used default parameter for S_min [0]");
  nh_.param("S_min", S_min_, 0);

  if (!nh_.hasParam("S_max"))
    ROS_WARN_STREAM("[" << name_ << "] Used default parameter for S_max [0]");
  nh_.param("S_max", S_max_, 0);

  if (!nh_.hasParam("V_min"))
    ROS_WARN_STREAM("[" << name_ << "] Used default parameter for V_min [0]");
  nh_.param("V_min", V_min_, 0);

  if (!nh_.hasParam("V_max"))
    ROS_WARN_STREAM("[" << name_ << "] Used default parameter for V_max [0]");
  nh_.param("V_max", V_max_, 0);

  //Setting other parameters
  if (!nh_.hasParam("min_contour_area"))
    ROS_WARN_STREAM("[" << name_ << "] Used default parameter for min_contour_area [0]");
  nh_.param("min_contour_area", min_contour_area_, 0);

  if (!nh_.hasParam("min_contour_radius"))
    ROS_WARN_STREAM("[" << name_ << "] Used default parameter for min_contour_radius [0]");
  nh_.param("min_contour_radius", min_contour_circle_radius_, 0);

  if (!nh_.hasParam("max_contour_radius"))
    ROS_WARN_STREAM("[" << name_ << "] Used default parameter for max_contour_radius [0]");
  nh_.param("max_contour_radius", max_contour_circle_radius_, 0);

  if (!nh_.hasParam("min_area_ratio"))
    ROS_WARN_STREAM("[" << name_ << "] Used default parameter for min_area_ratio [0]");
  nh_.param("min_area_ratio", min_area_ratio_, 0);

  if (!nh_.hasParam("show_gui"))
    ROS_WARN_STREAM("[" << name_ << "] Used default parameter for show_gui [true]");
  nh_.param("show_gui", show_gui_, true);

  if (show_gui_)
    settingsWindow();
}

void BallDetector::settingsWindow()
{
  namedWindow("Settings", CV_WINDOW_AUTOSIZE);

  createTrackbar("H_min", "Settings", &H_min_, 179);
  createTrackbar("H_max", "Settings", &H_max_, 179);
  createTrackbar("S_min", "Settings", &S_min_, 255);
  createTrackbar("S_max", "Settings", &S_max_, 255);
  createTrackbar("V_min", "Settings", &V_min_, 255);
  createTrackbar("V_max", "Settings", &V_max_, 255);

  createTrackbar("min_contour_area", "Settings", &min_contour_area_, 5000);
  createTrackbar("min_contour_radius", "Settings", &min_contour_circle_radius_, 500);
  createTrackbar("max_contour_radius", "Settings", &max_contour_circle_radius_, 500);
  createTrackbar("min_area_ratio", "Settings", &min_area_ratio_, 100);
}

std::vector<Point2f>* BallDetector::processFrame()
{
  Mat orig_frame;
  capture_.read(orig_frame); // get a new frame from camera
  if (show_gui_)
    imshow("Original", orig_frame);
  //GaussianBlur(orig_frame, orig_frame, Size(7,7), 1.5, 1.5);
  Mat hsv_frame;
  cvtColor(orig_frame, hsv_frame, COLOR_BGR2HSV);

  //Extraction of areas of the right color
  Mat thres_frame;
  inRange(hsv_frame, Scalar(H_min_, S_min_, V_min_), Scalar(H_max_, S_max_, V_max_), thres_frame);

  if (show_gui_)
    imshow("Thresholded", thres_frame);

  //morphological opening
  Mat morph_frame;
  erode(thres_frame, morph_frame, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  dilate(morph_frame, morph_frame, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  //morphological closing
//        dilate(morph_frame, morph_frame, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
//        erode(morph_frame, morph_frame, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

  if (show_gui_)
    imshow("Thresholded", morph_frame);

  //TODO Maybe Canny

  //Vectors for storing the extracted contours
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  findContours(morph_frame, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

  std::vector<Point2f>* detected_balls = processContours(contours, hierarchy, orig_frame);

  return detected_balls;
}

std::vector<Point2f>* BallDetector::processContours(std::vector<std::vector<Point> > contours,
                                                                 std::vector<cv::Vec4i> hierarchy,
                                                                 Mat frame)
{
  std::vector<Point2f> circle_midpoint(contours.size());
  std::vector<float> circle_radius(contours.size());
  std::vector<float> contour_area(contours.size());
  float min_area_ratio_float = min_area_ratio_ / 100.0;
  std::vector<Point2f>* detected_balls;
  for (int i = 0; i < contours.size(); i++)
  {
    //compute the areas of the contours
    contour_area.at(i) = contourArea(contours.at(i));
    //discard the contour if it's smaller than min_contour_area_
    if (contour_area.at(i) < min_contour_area_)
      continue;
    //compute the circle of the minimum area enclosing the contour
    minEnclosingCircle(contours.at(i), circle_midpoint.at(i), circle_radius.at(i));
    //discard the contour if the enclosing circle is too small or too big
    if (circle_radius.at(i) < min_contour_circle_radius_ || circle_radius.at(i) > max_contour_circle_radius_)
      continue;
    /*
     * compute the ratio area_of_contour/area_of_sorrounding_circle
     * to know how circle-like the contour is then discard contour if bad
     */
    if (contour_area.at(i) / (pow(circle_radius.at(i), 2) * M_PI) < min_area_ratio_float)
      continue;
    detected_balls->push_back(circle_midpoint.at(i));

    if (show_gui_){
      Scalar color = Scalar(255, 255, 255);
      Scalar color2 = Scalar(255, 0, 0);
      drawContours(frame, contours, i, color, 2, 8, hierarchy, 0, Point());
      circle(frame, circle_midpoint.at(i), 4, color2, -1, 8, 0);
      circle(frame, circle_midpoint.at(i), circle_radius.at(i), color2, 2, 8, 0);
    }
  }

  if (show_gui_){
        imshow("Extracted balls", frame);
      }
  return detected_balls;
}
