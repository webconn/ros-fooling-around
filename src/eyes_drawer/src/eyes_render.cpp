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

struct Color {
  double r = 0, g = 0, b = 0;
};

static const Color grad_top{ 33 / 255.0, 70 / 255.0, 110 / 255.0 },
grad_bottom{ 61 / 255.0, 117 / 255.0, 180 / 255.0 };

static const double kIrisXScale = 0.65;
static const double kIrisHeight = 120;
static const double kIrisXOffset = -10;
static const double kIrisYOffset = -10;

static const double kEyeXScale = 0.7;
static const double kEyeHeight = 60;

void EyesRender::draw_and_publish(size_t x, size_t y) {
  cairo_t *cr = cairo_create(surface_);

  cairo_set_source_rgb(cr, 1, 1, 1);
  cairo_paint(cr);

  cairo_translate(cr, x, y);

  // iris
  {
    // source
    const double half_grad_height = kIrisHeight / 2.0;

    // fill - gradient
    cairo_pattern_t *pat = cairo_pattern_create_linear(0.0, -half_grad_height,
                                                       0.0, half_grad_height);
    cairo_pattern_add_color_stop_rgb(pat, 0, grad_top.r, grad_top.g, grad_top.b);
    cairo_pattern_add_color_stop_rgb(pat, 1, grad_bottom.r, grad_bottom.g, grad_bottom.b);
    cairo_set_source(cr, pat);

    // mask - iris shape
    {
      cairo_matrix_t save_matrix;
      cairo_get_matrix(cr, &save_matrix);

      cairo_translate(cr, kIrisXOffset, kIrisYOffset);
      cairo_scale(cr, kIrisXScale, 1.0);

      cairo_arc(cr,
                0,
                0,
                kIrisHeight,
                0, M_PI * 2);
      cairo_fill(cr);

      cairo_set_matrix(cr, &save_matrix);
    }

    cairo_pattern_destroy(pat);

    // print eye
    cairo_set_source_rgb(cr, 0, 0, 0);

    // eye
    {
      cairo_matrix_t save_matrix;
      cairo_get_matrix(cr, &save_matrix);

      cairo_scale(cr, kEyeXScale, 1.0);

      cairo_arc(cr, 0, 0, kEyeHeight, 0, M_PI * 2);

      cairo_set_matrix(cr, &save_matrix);
    }

    cairo_fill(cr);
  }

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
