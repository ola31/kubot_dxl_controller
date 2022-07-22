#include "kubot_dxl_controller/dxl_controller.h"


dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);



void dxl_controller::Initialize(void){



}

void dxl_controller::ping_dxls(void){

  uint8_t error = 0;
  std::string joint_name;
  int joint_id;

  for(int i=0;i<LEG_DOF;i++){  //Left leg ping
    joint_name = left_joint_name[i];
    joint_id = joints[joint_name];

    packetHandler->ping(portHandler,joint_id, &error);
    if(error==0)
      ROS_INFO("joint[ %17s ] : id(%2d) found",joint_name.c_str(),joint_id);
    else
      ROS_ERROR("joint[ %17s ] : id(%2d) not found",joint_name.c_str(),joint_id);
  }
  for(int i=0;i<LEG_DOF;i++){  //Right leg ping
    joint_name = right_joint_name[i];
    joint_id = joints[joint_name];
    packetHandler->ping(portHandler,joint_id, &error);
    if(error==0)
      ROS_INFO("joint[ %17s ] : id(%2d) found",joint_name.c_str(),joint_id);
    else
      ROS_ERROR("joint[ %17s ] : id(%2d) not found",joint_name.c_str(),joint_id);
  }
}

void dxl_controller::setJointIdFrom_yaml(char file_path[]){  //JOINT_ID_FILEPATH
  YAML::Node doc = YAML::LoadFile("/home/ola/catkin_ws/src/kubot_dxl_controller/config/joint_id.yaml");
  std::cout<<"Read joint_id from yaml Start.. : "<<file_path<<std::endl;
  try {
        unsigned int id_value;
        for(unsigned i=0;i<LEG_DOF;i++){
          id_value=doc[left_joint_name[i].c_str()].as<int>();
          joints.insert({left_joint_name[i], id_value});
          //std::cout<<'\t'<<YAML_KEY[i].c_str()<<"\t:\t"<<value.c_str()<<std::endl;
        }
        for(unsigned i=0;i<LEG_DOF;i++){
          id_value=doc[right_joint_name[i].c_str()].as<int>();
          joints.insert({right_joint_name[i], id_value});
          //std::cout<<'\t'<<YAML_KEY[i].c_str()<<"\t:\t"<<value.c_str()<<std::endl;
        }
  } catch (YAML::Exception &e) {
        std::cerr << "joint_id YAML Exception: " << e.what() << std::endl;
  }


}

