#include "ros/ros.h"
#include "std_msgs/String.h"

void qrCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("qr data received: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "qr_localizer");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("qrcode", 1000, qrCallback);

  ros::spin();

  return 0;
}
