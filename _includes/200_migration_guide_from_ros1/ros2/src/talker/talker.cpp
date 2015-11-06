#include <sstream>
// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/String.h"
#include "std_msgs/msg/string.hpp"
int main(int argc, char **argv)
{
//  ros::init(argc, argv, "talker");
//  ros::NodeHandle n;
  rclcpp::init(argc, argv);
  auto node = rclcpp::node::Node::make_shared("talker");
//  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
//  ros::Rate loop_rate(10);
  auto chatter_pub = node->create_publisher<std_msgs::msg::String>("chatter", rmw_qos_profile_default);
  rclcpp::rate::Rate loop_rate(10);
  int count = 0;
//  std_msgs::String msg;
  auto msg = std::make_shared<std_msgs::msg::String>();
//  while (ros::ok())
  while (rclcpp::ok())
  {
    std::stringstream ss;
    ss << "hello world " << count++;
//    msg.data = ss.str();
    msg->data = ss.str();
//    ROS_INFO("%s", msg.data.c_str());
    printf("%s\n", msg->data.c_str());
//    chatter_pub.publish(msg);
    chatter_pub->publish(msg);
//    ros::spinOnce();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  return 0;
}
