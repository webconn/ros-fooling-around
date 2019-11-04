#include <eyes_drawer/eyes_render.h>
#include <cairo/cairo.h>
#include <cstdlib>
#include <iterator>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <algorithm>

EyesRender::EyesRender(image_transport::ImageTransport& it,
                       const std::string&               image_topic,
                       size_t                           img_width,
                       size_t                           img_height)
  : img_width_(img_width)
  , img_height_(img_height) {
  img_pub_ = it.advertise(image_topic, 1);
  surface_ = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, img_width_, img_height_);
}

EyesRender::~EyesRender() {
  cairo_surface_destroy(surface_);
}

void EyesRender::draw_and_publish(size_t x, size_t y) {
  cairo_t *cr = cairo_create(surface_);

  cairo_select_font_face(cr, "serif", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_BOLD);
  cairo_set_font_size(cr, 32.0);
  cairo_set_source_rgb(cr, 0x79 / 255.0, 0x23 / 255.0, 0x59 / 255.0);
  cairo_move_to(cr, 10.0, 50.0);
  cairo_show_text(cr, "Hello, world");

  cairo_destroy(cr);
  cairo_surface_flush(surface_);

  sensor_msgs::Image msg;
  make_image_msg(surface_, &msg);

  img_pub_.publish(msg);
}

void EyesRender::make_image_msg(cairo_surface_t *s, sensor_msgs::Image *msg) {
  msg->header.stamp = ros::Time::now();
  msg->encoding = "bgra8";
  msg->is_bigendian = false;
  msg->width = cairo_image_surface_get_width(s);
  msg->height = cairo_image_surface_get_height(s);
  msg->step = cairo_image_surface_get_stride(s);

  size_t data_size = msg->width * msg->height * sizeof(uint32_t);
  const uint8_t *data = cairo_image_surface_get_data(s);

  msg->data.assign(data, data + data_size);
}
