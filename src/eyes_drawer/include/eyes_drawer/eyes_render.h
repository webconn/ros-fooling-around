#pragma once

#include <cmath>
#include <image_transport/publisher.h>
#include <memory>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cairo/cairo.h>

class IEyesRenderer {
  public:
    virtual void init(size_t img_width, size_t img_height) {
      img_width_ = img_width;
      img_height_ = img_height;
      img_diam_ = std::sqrt(img_width * img_width + img_height * img_height);
    }

    virtual void draw(cairo_surface_t *surface,
                      int x, int y, double iris_size, double eye_size,
                      int flare_x, int flare_y, double flare_size) = 0;

  protected:
    size_t img_width_;
    size_t img_height_;
    size_t img_diam_;
};

class EyesRender {
  public:
    EyesRender(image_transport::ImageTransport& it,
               const std::string&               image_topic,
               size_t                           img_width,
               size_t                           img_height,
               std::shared_ptr<IEyesRenderer>   renderer);
    virtual ~EyesRender();

    void draw_and_publish(int x, int y, double iris_size, double eye_size, int flare_x, int flare_y, double flare_size);

  private:
    void make_image_msg(cairo_surface_t *s, sensor_msgs::Image *image);
    std::shared_ptr<IEyesRenderer> renderer_;

    image_transport::Publisher img_pub_;
    cairo_surface_t *surface_;
};

struct Color {
  double r = 0, g = 0, b = 0;
};


struct PonyEyesConfig {
  double rotation;

  Color grad_top, grad_bottom;

  double iris_radius, iris_x_scale;
  int iris_dx, iris_dy;

  double eye_radius, eye_x_scale;

  double mark_step_angle;
  std::vector<Color> marks;

  struct Flare {
    int dx, dy;
    double diam;
    double x_scale;
  };
  std::vector<Flare> flares;
};

class PonyEyesRenderer : public IEyesRenderer {
  public:
    void draw(cairo_surface_t *surface,
              int x, int y, double iris_size, double eye_size,
              int flare_x, int flare_y, double flare_size) override;

  private:
    void make_background(cairo_t *cr);
    void make_iris(cairo_t *cr);
    void make_eye(cairo_t *cr);

    void make_flare(cairo_t *cr);
};
