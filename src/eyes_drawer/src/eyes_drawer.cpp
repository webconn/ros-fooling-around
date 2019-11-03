#include <eyes_drawer/eyes_drawer.h>

#include <ros/assert.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>

EyeDrawer::EyeDrawer(ros::NodeHandle& nh,
                     image_transport::ImageTransport& it,
                     const std::string& eye_name,
                     const std::string& eye_origin_frame,
                     const std::string& display_origin_frame,
                     const std::string& poi_frame,
                     double meters_per_pixel,
                     int img_w, int img_h)
  : nh_(nh)
  , it_(it)
  , eye_name_(eye_name)
  , eye_origin_frame_(eye_origin_frame)
  , display_origin_frame_(display_origin_frame)
  , poi_frame_(poi_frame)
  , meters_per_pixel_(meters_per_pixel)
  , img_w_(img_w)
  , img_h_(img_h) {
}

void EyeDrawer::init() {
  img_pub_ = it_.advertise(std::string("eyes/") + eye_name_, 1);
}

tf::Vector3 EyeDrawer::get_point_in_display_coords(const tf::Vector3& eye_origin,
                                                   const tf::Vector3& point_of_interest) {
  static const tf::Vector3 kDisplayNorm(0, 0, 1);

  const tf::Vector3 or_minus_pi = eye_origin - point_of_interest;
  const tfScalar d_nom = eye_origin.dot(kDisplayNorm);
  const tfScalar d_den = or_minus_pi.dot(kDisplayNorm);

  // if observed object is out of sight, just drop back to normal position
  if (std::abs(d_den) < 1e-6) {
    return tf::Vector3();
  }

  return eye_origin - or_minus_pi * (d_nom / d_den);
}

void EyeDrawer::redraw() {
  tf::StampedTransform poi_to_display;

  // we have bunch of static transforms, so we can deside to make display origin as
  // origin for all our translations, so final point will be in display coordinates
  listener_.lookupTransform(display_origin_frame_, eye_origin_frame_, ros::Time(0), eye_to_display_);

  listener_.lookupTransform(display_origin_frame_, poi_frame_, ros::Time(0), poi_to_display);

  tf::Vector3 result = get_point_in_display_coords(
      eye_to_display_.getOrigin(), poi_to_display.getOrigin());

  // we must get result in display plane, so z coordinate must be 0
  ROS_ASSERT(std::abs(result.z()) < 1e-6);

  draw_and_publish(result.x(), result.y());
}

void EyeDrawer::draw_and_publish(double x_orig, double y_orig) {
  cv::Mat img = cv::Mat::zeros(img_h_, img_w_, CV_8UC3);

  int x = img_w_ / 2 + static_cast<int>(x_orig / meters_per_pixel_);
  int y = img_h_ / 2 + static_cast<int>(y_orig / meters_per_pixel_);

  if (x < 0) {
    x = 0;
  }

  if (x >= img_w_) {
    x = img_w_ - 1;
  }

  if (y < 0) {
    y = 0;
  }

  if (y >= img_h_) {
    y = img_h_ - 1;
  }

  cv::circle(img, cv::Point(x, y), 20, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_8);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  img_pub_.publish(msg);
}
