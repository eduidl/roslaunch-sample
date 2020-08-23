#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  const auto chatter_pub = nh.advertise<std_msgs::String>("chatter", 10);

  ros::NodeHandle pnh("~");
  std::string msg_content = "Hello world!";
  pnh.getParam("message", msg_content);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = msg_content;
    ROS_INFO("publish: %s", msg.data.c_str());
    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
