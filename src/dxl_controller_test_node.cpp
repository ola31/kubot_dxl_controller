/* Author: Park Kiwoong (KUDOS 8th) */

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

  int encoder_read[12];

  x.Initialize();
  x.Read_Dxl_Encoder_Once(encoder_read);
  //x.Torque_ON_dxls();
  //x.getJointIdFrom_yaml(JOINT_ID_FILEPATH);
  //x.Sync_Position_command_TxOnly(encoder_read);

  for(int i=0;i<12;i++){
    ROS_INFO("joint %d : %d",i+1,encoder_read[i]);
  }

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    x.Read_Dxl_Encoder_Once(encoder_read);
    for(int i=0;i<12;i++){
      ROS_INFO("joint %d : %d",i+1,encoder_read[i]);
    }
    std::cout<<"---------------------------------"<<std::endl;

    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
x.Close_port();
  return 0;
}
