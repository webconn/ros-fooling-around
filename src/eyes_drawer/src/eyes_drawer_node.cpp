#include <eyes_drawer/eyes_drawer.h>
#include <eyes_drawer/eyes_render.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "eyes_drawer");

  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  int img_width, img_height;
  if (!nh.getParam("img_width", img_width) || !nh.getParam("img_height", img_height)) {
    ROS_FATAL("can't get image size parameters");
    return 1;
  }

  EyeDrawer left_drawer(nh, it, "left", "/left_origin", "/left_display", "/point_of_interest", 0.0002, img_width, img_height);
  EyeDrawer right_drawer(nh, it, "right", "/right_origin", "/right_display", "/point_of_interest", 0.0002, img_width, img_height);

  EyesRender render(it, "/test_image", img_width, img_height);

  left_drawer.init();
  right_drawer.init();

  ros::Rate loop_rate(20);

  while (nh.ok()) {
    try {
      left_drawer.redraw();
      right_drawer.redraw();
      render.draw_and_publish(0, 0);
    } catch (tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    /* pub_left.publish(msg_left); */
    /* pub_right.publish(msg_right); */

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
