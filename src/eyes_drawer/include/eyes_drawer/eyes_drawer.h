#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3.h>

class EyeDrawer {
  public:
    EyeDrawer(ros::NodeHandle& nh,
              image_transport::ImageTransport& it,
              const std::string& eye_name,
              const std::string& eye_origin_frame,
              const std::string& display_origin_frame,
              const std::string& poi_frame,
              double meters_per_pixel,
              int img_w, int img_h);

    void init();
    void redraw();

  private:
    tf::Vector3 get_point_in_display_coords(const tf::Vector3& eye_origin,
                                            const tf::Vector3& point_of_interest);
    void draw_and_publish(double x_orig, double y_orig);

    ros::NodeHandle& nh_;
    image_transport::ImageTransport& it_;

    image_transport::Publisher img_pub_;

    tf::StampedTransform eye_to_display_;
    tf::TransformListener listener_;

    std::string eye_name_;
    std::string eye_origin_frame_;
    std::string display_origin_frame_;
    std::string poi_frame_;

    double meters_per_pixel_;
    int img_w_, img_h_;
};
