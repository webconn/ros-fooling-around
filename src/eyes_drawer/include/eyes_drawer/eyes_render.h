#pragma once

#include <image_transport/publisher.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cairo/cairo.h>

class EyesRender {
  public:
    EyesRender(image_transport::ImageTransport& it,
               const std::string&               image_topic,
               size_t                           img_width,
               size_t                           img_height);
    ~EyesRender();

    void draw_and_publish(size_t x, size_t y);

  private:
    void make_image_msg(cairo_surface_t* s, sensor_msgs::Image* image);

    image_transport::Publisher img_pub_;
    cairo_surface_t* surface_;

    const size_t img_width_;
    const size_t img_height_;
};
