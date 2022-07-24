#ifndef DXL_CONTROLLER_H
#define DXL_CONTROLLER_H

#include <ros/ros.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <map>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <string>
#include <ros/package.h>



//Dynamixel Control

#define DEVICENAME                      "/dev/ttyUSB0"
#define PROTOCOL_VERSION                2.0

#define BAUDRATE                        2000000
#define ADDR_TORQUE_ENABLE              64

#define ADDR_GOAL_POSITION              116
#define ADDR_PRESENT_POSITION           13
// Data Byte Length
#define LEN_GOAL_POSITION            4
#define LEN_PRESENT_POSITION         4

#define LEG_DOF                      6  //leg demension of freedom

#define JOINT_ID_FILEPATH    "/config/joint_id.yaml"


class dxl_controller
{
private:

  std::map<std::string, int> joints;
  //std::map<string,int,int> jointPD_gain;

  const std::string left_joint_name[LEG_DOF] = {"left_hip_yaw","left_hip_roll","left_hip_pitch","left_knee","left_ankle_pitch","left_ankle_roll"};
  const std::string right_joint_name[LEG_DOF] = {"right_hip_yaw","right_hip_roll","right_hip_pitch","right_knee","right_ankle_pitch","right_ankle_roll"};

  const std::string pkg_path = ros::package::getPath("kubot_dxl_controller");  //get package path

public:

  bool Initialize(void);
  bool OpenPort(void);
  bool SetBaudRate(void);
  void ping_dxls(void);
  void setJointIdFrom_yaml(const char file_path[]);
};


#endif // DXL_CONTROLLER_H
