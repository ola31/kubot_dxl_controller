#include "kubot_dxl_controller/dxl_controller.h"


dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);


bool dxl_controller::Initialize(void){
  // Open port
  if (!OpenPort()) return false;
  // Set port baudrate
  if (!SetBaudRate()) return false;

  return true;
}

bool dxl_controller::OpenPort(void){
  // Open port
  if (portHandler->openPort()){
    printf("Succeeded to open the port!\n");
    return true;
  }
  else{
    printf("Failed to open the port!\n");
    return false;
  }
}
bool dxl_controller::SetBaudRate(void){
  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)){
    printf("Succeeded to change the baudrate!\n");
    return true;
  }
  else{
    printf("Failed to change the baudrate!\n");
    return false;
  }
}

void dxl_controller::ping_dxls(void){

  uint8_t error = 0;
  uint16_t model_num;
  int dxl_comm_result = COMM_TX_FAIL;
  std::string joint_name;
  int joint_id;

  for(int i=0;i<LEG_DOF;i++){  //Left leg ping
    joint_name = left_joint_name[i];
    joint_id = joints[joint_name];

    dxl_comm_result = packetHandler->ping(portHandler,joint_id, &model_num, &error);
    if(dxl_comm_result == COMM_SUCCESS)
      ROS_INFO("JOINT[ %17s ] : ID[%2d] : MODEL[%d] Found",joint_name.c_str(), joint_id, model_num);
    else
      ROS_ERROR("JOINT[ %17s ] : ID(%2d) Not Found",joint_name.c_str(),joint_id);
  }
  std::cout<<std::endl;
  for(int i=0;i<LEG_DOF;i++){  //Right leg ping
    joint_name = right_joint_name[i];
    joint_id = joints[joint_name];

    dxl_comm_result = packetHandler->ping(portHandler,joint_id, &error);
    if(dxl_comm_result == COMM_SUCCESS)
      ROS_INFO("JOINT[ %17s ] : ID[%2d] : MODEL[%d] Found",joint_name.c_str(), joint_id, model_num);
    else
      ROS_ERROR("JOINT[ %17s ] : ID(%2d) Not Found",joint_name.c_str(),joint_id);
  }
  /*
  for(auto iter = joints.begin() ; iter !=  joints.end(); iter++){
    std::cout<<iter->first<<" "<<iter->second<<std::endl;
  }
  */
}

void dxl_controller::setJointIdFrom_yaml(const char file_path[]){  //JOINT_ID_FILEPATH
  std::string local_path(file_path);
  std::string path = this->pkg_path+local_path;

  YAML::Node doc = YAML::LoadFile(path);
  std::cout<<"Read joint_id from yaml Start.. : "<<path<<std::endl;
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

