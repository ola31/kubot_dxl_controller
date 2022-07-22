#include "ros/ros.h"
#include "std_msgs/String.h"
#include "kubot_dxl_controller/dxl_controller.h"

//string a = "../config/joint_id.yaml";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dxl_controller_test_node");
  ros::NodeHandle nh;

  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  dxl_controller x;

  x.Initialize();
  x.setJointIdFrom_yaml("/home/ola/catkin_ws/src/kubot_dxl_controller/config/joint_id.yaml");
  x.ping_dxls();


  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
