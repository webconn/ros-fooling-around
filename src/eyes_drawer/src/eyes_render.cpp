#include <cmath>
#include <eyes_drawer/eyes_render.h>
#include <cairo/cairo.h>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <algorithm>

EyesRender::EyesRender(image_transport::ImageTransport& it,
                       const std::string&               image_topic,
                       size_t                           img_width,
                       size_t                           img_height,
                       std::shared_ptr<IEyesRenderer>   renderer)
  : renderer_(renderer) {
  img_pub_ = it.advertise(image_topic, 1);
  surface_ = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, img_width, img_height);
  renderer_->init(img_width, img_height);
}

EyesRender::~EyesRender() {
  cairo_surface_destroy(surface_);
}

static const Color grad_top{ 33 / 255.0, 70 / 255.0, 110 / 255.0 },
grad_bottom{ 61 / 255.0, 117 / 255.0, 180 / 255.0 };

static const double kIrisXScale = 0.7;
static const double kIrisHeight = 120;
static const double kIrisXOffset = 0;
static const double kIrisYOffset = -5;

static const double kEyeXScale = 0.7;
static const double kEyeHeight = 80;
static const double kEyeRotation = 0.2;

static const double kMarkStepAngle = 0.3;
static const double kMarkStartAngle = M_PI + (M_PI / 2 - 3 * kMarkStepAngle);

static const int kNumMarks = 6;
static const Color kMarks[kNumMarks] = {
  { 89,  145,  206         },
  { 117, 168,  221         },
  { 63,  120,  185         },
  { 63,  120,  183         },
  { 75,  131,  192         },
  { 71,  125,  187         }
};

static const int kNumFlares = 2;
static const Flare kFlares[kNumFlares] = {
  { 40, -40, 50.0,  0.6  },
  { 5,  20,  10.0,  0.95 }
};

void EyesRender::draw_and_publish(int x, int y, double iris_size, double eye_size, int flare_x, int flare_y, double flare_size) {
  renderer_->draw(surface_, x, y, iris_size, eye_size, flare_x, flare_y, flare_size);

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

void PonyEyesRenderer::make_background(cairo_t *cr) {
  const double half_grad_height = kIrisHeight / 2.0;

  cairo_push_group(cr);

  // fill - gradient and these beautiful eyemarks
  cairo_pattern_t *pat = cairo_pattern_create_linear(0.0, -half_grad_height,
                                                     0.0, half_grad_height);
  cairo_pattern_add_color_stop_rgb(pat, 0, grad_top.r, grad_top.g, grad_top.b);
  cairo_pattern_add_color_stop_rgb(pat, 1, grad_bottom.r, grad_bottom.g, grad_bottom.b);

  cairo_set_source(cr, pat);
  cairo_paint(cr);

  // beautiful eyemarks
  double current_angle = kMarkStartAngle;

  for (int i = 0; i < kNumMarks; i++, current_angle += kMarkStepAngle) {
    cairo_move_to(cr, img_diam_ * std::cos(current_angle), img_diam_ * std::sin(-current_angle));
    cairo_line_to(cr, 0, 0);
    cairo_line_to(cr, img_diam_ * std::cos(current_angle + kMarkStepAngle), img_diam_ * std::sin(-(current_angle + kMarkStepAngle)));
    cairo_close_path(cr);

    cairo_set_source_rgb(cr, kMarks[i].r / 255.0, kMarks[i].g / 255.0, kMarks[i].b / 255.0);
    cairo_fill(cr);
  }

  cairo_pop_group_to_source(cr);

  cairo_pattern_destroy(pat);
}

void PonyEyesRenderer::make_iris(cairo_t *cr) {
  // source - gradient and eyemarks
  make_background(cr);

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
}

void PonyEyesRenderer::make_eye(cairo_t *cr) {
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

void PonyEyesRenderer::make_flare(cairo_t *cr) {
  cairo_set_source_rgb(cr, 1, 1, 1);

  for (int i = 0; i < kNumFlares; i++) {
    cairo_matrix_t save_matrix;
    cairo_get_matrix(cr, &save_matrix);

    cairo_translate(cr, kFlares[i].dx, kFlares[i].dy);
    cairo_scale(cr, kFlares[i].x_scale, 1.0);

    cairo_arc(cr, 0, 0, kFlares[i].diam, 0, M_PI * 2);

    cairo_set_matrix(cr, &save_matrix);
  }

  cairo_fill(cr);
}

void PonyEyesRenderer::draw(cairo_surface_t *surface,
                            int x, int y, double iris_size, double eye_size,
                            int flare_x, int flare_y, double flare_size) {
  cairo_t *cr = cairo_create(surface);

  cairo_set_source_rgb(cr, 1, 1, 1);
  cairo_paint(cr);

  {
    cairo_matrix_t save_matrix;
    cairo_get_matrix(cr, &save_matrix);

    cairo_translate(cr, x, y);
    cairo_rotate(cr, kEyeRotation);

    make_iris(cr);
    make_eye(cr);
    cairo_set_matrix(cr, &save_matrix);
  }

  {
    cairo_matrix_t save_matrix;
    cairo_get_matrix(cr, &save_matrix);

    cairo_translate(cr, flare_x, flare_y);

    make_flare(cr);

    cairo_set_matrix(cr, &save_matrix);
  }

  cairo_destroy(cr);
}
