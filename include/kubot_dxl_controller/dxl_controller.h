#ifndef DXL_CONTROLLER_H
#define DXL_CONTROLLER_H

#include <ros/ros.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk/protocol2_packet_handler.h"
#include <map>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <string>
#include <ros/package.h>

#include "kubot_dxl_controller/k_group_sync_write.h"
#include "kubot_dxl_controller/kubot_sync_read_write.h"



//Dynamixel Control

#define DEVICENAME                      "/dev/ttyUSB0"
#define PROTOCOL_VERSION                2.0

#define BAUDRATE                        2000000
#define ADDR_TORQUE_ENABLE              64
#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0

#define ADDR_GOAL_POSITION              116
#define ADDR_PRESENT_POSITION           132

#define ADDR_POSITION_P_GAIN             84
#define ADDR_POSITION_D_GAIN             80
// Data Byte Length
#define LEN_GOAL_POSITION            4
#define LEN_PRESENT_POSITION         4

/****** may need to modified if robot has arms and head *****/
#define LEG_DOF                      6  //leg demension of freedom
#define TOTAL_DXL_NUM               12  //this must be modified if robot has more than 12 dxls
/************************************************************/

#define JOINT_ID_FILEPATH         "/config/joint_id.yaml"
#define JOINT_PD_GAIN_FILEPATH    "/config/joint_PD_gain.yaml"


class dxl_controller
{
private:

  //joint info Mapping
  std::map<std::string, int> joints; //name:id
  std::map<std::string,int> jointP_gain; //name:P gain
  std::map<std::string,int> jointD_gain; //name:D gain

  /****** may need to modified if robot has arms and head *****/
  const std::string left_joint_name[LEG_DOF] = {"left_hip_yaw","left_hip_roll","left_hip_pitch","left_knee","left_ankle_pitch","left_ankle_roll"};
  const std::string right_joint_name[LEG_DOF] = {"right_hip_yaw","right_hip_roll","right_hip_pitch","right_knee","right_ankle_pitch","right_ankle_roll"};
  /************************************************************/

  const std::string pkg_path = ros::package::getPath("kubot_dxl_controller");  //get package path

public:

  bool Initialize(void);
  bool OpenPort(void);
  void Close_port(void);
  bool SetBaudRate(void);
  bool Torque_ON_dxls();
  bool Torque_OFF_dxls();
  bool ping_dxls(void);
  const char* get_dxl_model_name(int dxls_model_num);
  bool getJointIdFrom_yaml(const char file_path[]);
  bool getJoint_PD_gainFrom_yaml(const char file_path[]);
  bool setDXL_PD_gain(void);
  void set_Dxl_Encoder_SyncRead(void);
  void Read_Dxl_Encoder_Once(int *Encoder_array);
  void Sync_Position_command_TxOnly(int (&dxl_goal_posi)[TOTAL_DXL_NUM]);
};


#endif // DXL_CONTROLLER_H
