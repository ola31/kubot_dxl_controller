/* Author: Park Kiwoong (KUDOS 8th) */

#include "kubot_dxl_controller/dxl_controller.h"


//DXL SDK
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
//dynamixel::PacketHandler *packetHandler = (dynamixel::PacketHandler *)(kubot_sync_read_write::getInstance());

dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
//K_GroupSyncWrite k_groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

bool dxl_controller::Initialize(void){
  // Open port
  if (!OpenPort()) return false;

  // Set port baudrate
  if (!SetBaudRate()) return false;

  //get joint DXL ID
  if(!getJointIdFrom_yaml(JOINT_ID_FILEPATH)) return false;

  //get joint DXL PD gain
  if(!getJoint_PD_gainFrom_yaml(JOINT_PD_GAIN_FILEPATH)) return false;

  //ping dxls to check communiation
  if(!ping_dxls()) return false;

  set_Dxl_Encoder_SyncRead();

  return true;
}
bool dxl_controller::OpenPort(void){
  // Open port
  if (portHandler->openPort()){
    ROS_INFO("Succeeded to open the port!\n");
    return true;
  }
  else{
    ROS_ERROR("Failed to open the port!\n");
    return false;
  }
}
void dxl_controller::Close_port(void){
  // Close port
  portHandler->closePort();
}
bool dxl_controller::SetBaudRate(void){
  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)){
    ROS_INFO("Succeeded to change the baudrate!\n");
    return true;
  }
  else{
    ROS_ERROR("Failed to change the baudrate!\n");
    return false;
  }
}
bool dxl_controller::Torque_ON_dxls(){
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  for(int i=0;i<LEG_DOF;i++){
    uint8_t dxl_id = joints[left_joint_name[i]];
    uint8_t dxl_error=0;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
      packetHandler->getTxRxResult(dxl_comm_result);
    else if (dxl_error != 0)
      packetHandler->getRxPacketError(dxl_error);
    else
      ROS_INFO("Dynamixel[%2d] Torque ON", dxl_id);
  }
  for(int i=0;i<LEG_DOF;i++){
    uint8_t dxl_id = joints[right_joint_name[i]];
    uint8_t dxl_error=0;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
      packetHandler->getTxRxResult(dxl_comm_result);
    else if (dxl_error != 0)
      packetHandler->getRxPacketError(dxl_error);
    else
      ROS_INFO("Dynamixel[%2d] Torque ON", dxl_id);
  }
}
bool dxl_controller::Torque_OFF_dxls(){
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  for(int i=0;i<LEG_DOF;i++){
    uint8_t dxl_id = joints[left_joint_name[i]];
    uint8_t dxl_error=0;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
      packetHandler->getTxRxResult(dxl_comm_result);
    else if (dxl_error != 0)
      packetHandler->getRxPacketError(dxl_error);
  }
  for(int i=0;i<LEG_DOF;i++){
    uint8_t dxl_id = joints[right_joint_name[i]];
    uint8_t dxl_error=0;
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
      packetHandler->getTxRxResult(dxl_comm_result);
    else if (dxl_error != 0)
      packetHandler->getRxPacketError(dxl_error);
  }
}
bool dxl_controller::ping_dxls(void){

  uint8_t error = 0;
  uint16_t model_num;
  int dxl_comm_result = COMM_TX_FAIL;
  std::string joint_name;
  int joint_id;

  bool result = true;

  for(int i=0;i<LEG_DOF;i++){  //Left leg ping
    joint_name = left_joint_name[i];
    joint_id = joints[joint_name];

    dxl_comm_result = packetHandler->ping(portHandler,joint_id, &model_num, &error);
    if(dxl_comm_result == COMM_SUCCESS)
      ROS_INFO("JOINT[ %17s ] : ID[%2d] : MODEL[%d] Found",joint_name.c_str(), joint_id, model_num);
    else{
      ROS_ERROR("JOINT[ %17s ] : ID(%2d) Not Found",joint_name.c_str(),joint_id);
      result = false;
    }
  }
  std::cout<<std::endl;
  for(int i=0;i<LEG_DOF;i++){  //Right leg ping
    joint_name = right_joint_name[i];
    joint_id = joints[joint_name];

    dxl_comm_result = packetHandler->ping(portHandler,joint_id, &error);
    if(dxl_comm_result == COMM_SUCCESS)
      ROS_INFO("JOINT[ %17s ] : ID[%2d] : MODEL[%d] Found",joint_name.c_str(), joint_id, model_num);
    else{
      ROS_ERROR("JOINT[ %17s ] : ID(%2d) Not Found",joint_name.c_str(),joint_id);
      result = false;
    }
  }
  /*
  for(auto iter = joints.begin() ; iter !=  joints.end(); iter++){
    std::cout<<iter->first<<" "<<iter->second<<std::endl;
  }
  */

  return result;
}
bool dxl_controller::getJointIdFrom_yaml(const char file_path[]){  //JOINT_ID_FILEPATH
  std::string local_path(file_path);
  std::string path = this->pkg_path+local_path;

  YAML::Node doc = YAML::LoadFile(path);
  //std::cout<<"Read joint_id from yaml Start.. : "<<path<<std::endl;
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
        //std::cerr << "joint_id YAML Exception: " << e.what() << std::endl;
        return false;
  }
  std::ofstream fclose(file_path);
  return true;


}
bool dxl_controller::getJoint_PD_gainFrom_yaml(const char file_path[]){
  std::string local_path(file_path);
  std::string path = this->pkg_path+local_path;
  YAML::Node doc = YAML::LoadFile(path);
  int Pgain = 850; //default
  int Dgain = 0;
  try {
        for(unsigned i=0;i<LEG_DOF;i++){
          YAML::Node each_joint = doc[left_joint_name[i].c_str()];
          Pgain=each_joint["P_gain"].as<int>();
          Dgain=each_joint["D_gain"].as<int>();
          jointP_gain.insert({left_joint_name[i], Pgain});
          jointD_gain.insert({left_joint_name[i], Dgain});
        }
        for(unsigned i=0;i<LEG_DOF;i++){
          YAML::Node each_joint = doc[right_joint_name[i].c_str()];
          Pgain=each_joint["P_gain"].as<int>();
          Dgain=each_joint["D_gain"].as<int>();
          jointP_gain.insert({right_joint_name[i], Pgain});
          jointD_gain.insert({right_joint_name[i], Dgain});
        }
/*
        for(int i=0;i<LEG_DOF;i++){
          std::cout<<left_joint_name[i]<<std::endl;
          std::cout<<jointP_gain[left_joint_name[i]]<<std::endl;
          std::cout<<jointD_gain[left_joint_name[i]]<<std::endl;
          std::cout<<"------------------------"<<std::endl;
        }
        for(int i=0;i<LEG_DOF;i++){
          std::cout<<right_joint_name[i]<<std::endl;
          std::cout<<jointP_gain[right_joint_name[i]]<<std::endl;
          std::cout<<jointD_gain[right_joint_name[i]]<<std::endl;
          std::cout<<"------------------------"<<std::endl;
        }
*/

  } catch (YAML::Exception &e) {
        std::cerr << "joint_PDgain YAML Exception: " << e.what() << std::endl;
        return false;
  }
  std::ofstream fclose(file_path);
  return true;
}
bool dxl_controller::setDXL_PD_gain(void){
  uint8_t dxl_error=0;
  uint8_t dxl_comm_result = COMM_TX_FAIL;             // Communication result

  for(int i=0;i<LEG_DOF;i++){  //left leg dxls
    int dxl_id = joints[left_joint_name[i]];
    //set p gain
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_POSITION_P_GAIN, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
      packetHandler->getTxRxResult(dxl_comm_result);
    else if (dxl_error != 0){
      packetHandler->getRxPacketError(dxl_error);
      ROS_ERROR("DXL_SET_P_GAIN_FAILED[ID:%d]",dxl_id);
    }
    //set d gain
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_POSITION_D_GAIN, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
      packetHandler->getTxRxResult(dxl_comm_result);
    else if (dxl_error != 0){
      packetHandler->getRxPacketError(dxl_error);
      ROS_ERROR("DXL_SET_D_GAIN_FAILED[ID:%d]",dxl_id);
    }

  }
  for(int i=0;i<LEG_DOF;i++){  //right leg dxls
    int dxl_id = joints[right_joint_name[i]];
    //set p gain
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_POSITION_P_GAIN, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
      packetHandler->getTxRxResult(dxl_comm_result);
    else if (dxl_error != 0){
      packetHandler->getRxPacketError(dxl_error);
      ROS_ERROR("DXL_SET_P_GAIN_FAILED[ID:%d]",dxl_id);
    }
    //set d gain
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, dxl_id, ADDR_POSITION_D_GAIN, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
      packetHandler->getTxRxResult(dxl_comm_result);
    else if (dxl_error != 0){
      packetHandler->getRxPacketError(dxl_error);
      ROS_ERROR("DXL_SET_D_GAIN_FAILED[ID:%d]",dxl_id);
    }
  }
}
void dxl_controller::set_Dxl_Encoder_SyncRead(void){
  bool dxl_addparam_result = true;                // addParam result
  // Add parameter storage for 12 Dynamixel present position value
  std::map<std::string,int>::iterator iter;
  for(iter = joints.begin();iter!=joints.end();iter++){
    int dxl_id = iter->second;
    dxl_addparam_result = groupSyncRead.addParam(dxl_id);
    if (dxl_addparam_result != true){
      ROS_ERROR("[set_Dxl_Encoder_SyncRead()] [ID:%03d] groupSyncWrite addparam failed", dxl_id);
      return;
    }
  }
}
void dxl_controller::Read_Dxl_Encoder_Once(int *Encoder_array){  // inappropriate in realtime thread
  uint8_t dxl_comm_result = COMM_TX_FAIL;
  int Encoder_value[TOTAL_DXL_NUM];
  bool dxl_getdata_result = false;                 // GetParam result
  // Syncread present position
  dxl_comm_result = groupSyncRead.txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

  // Check if groupsyncread data of 12 Dynamixel is available
  std::map<std::string,int>::iterator iter;
  for(iter = joints.begin();iter!=joints.end();iter++){
    int dxl_id = iter->second;
    dxl_getdata_result = groupSyncRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    if (dxl_getdata_result != true){
      ROS_ERROR("[ID:%03d] groupSyncRead getdata failed", dxl_id);
      return;
    }
  }

  // Get 12 Dynamixels present position value
  for(iter = joints.begin();iter!=joints.end();iter++){
    int dxl_id = iter->second;
    Encoder_array[dxl_id-1] = groupSyncRead.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    if (dxl_getdata_result != true){
      ROS_ERROR("[ID:%03d] groupSyncRead getdata failed", dxl_id);
      return;
    }
    //ROS_WARN("%d",Encoder_array[dxl_id-1]);
  }
  //Encoder_array = Encoder_value; //Encoder Output
}
void dxl_controller::Sync_Position_command_TxOnly(int (&dxl_goal_posi)[TOTAL_DXL_NUM]){

  /*** [ CAUTION!!] ********************************************************************************************
   * 'dxl_goal_posi[]' is a array of joint positions
   * 'dxl_goal_posi[]' must be passed to this function in the following format.
   * int dxl_goal_posi[TOTAL_DXL_NUM] = {ID1_dxl's posi, ID2'2 posi, ID3's posi, ... , ID(TOTAL_DXL_NUM)'s posi};
   *************************************************************************************************************/

  bool dxl_addparam_result = false;                // addParam result
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint8_t param_goal_position[4];

  std::map<std::string,int>::iterator iter;
  for(iter = joints.begin();iter!=joints.end();iter++){
    int dxl_id = iter->second;
    int dxl_goal_position = dxl_goal_posi[dxl_id-1];

    // Allocate goal position value into byte array
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(dxl_goal_position));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(dxl_goal_position));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(dxl_goal_position));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(dxl_goal_position));

    dxl_addparam_result = groupSyncWrite.addParam(dxl_id, param_goal_position);
    if (dxl_addparam_result != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dxl_id);
      return;
    }
  }
  //ROS_INFO("dd");
  // Syncwrite goal position
   //k_groupSyncWrite.clearParam();
  dxl_comm_result = groupSyncWrite.txPacket();
   //ROS_INFO("ddd");
  if (dxl_comm_result != COMM_SUCCESS) packetHandler->getTxRxResult(dxl_comm_result);

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}
