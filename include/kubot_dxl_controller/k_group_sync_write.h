#ifndef K_GROUP_SYNC_WRITE_H
#define K_GROUP_SYNC_WRITE_H



#include <map>
#include <vector>
#include "dynamixel_sdk/port_handler.h"
#include "dynamixel_sdk/packet_handler.h"
#include "kubot_dxl_controller/kubot_sync_read_write.h"

//namespace dynamixel
//{

using namespace dynamixel;

////////////////////////////////////////////////////////////////////////////////
/// @brief The class for writing multiple Dynamixel data from same address with same length at once
////////////////////////////////////////////////////////////////////////////////
class WINDECLSPEC K_GroupSyncWrite
{
 private:
  PortHandler    *port_;
  PacketHandler  *ph_;
  kubot_sync_read_write *k_ph;

  std::vector<uint8_t>            id_list_;
  std::map<uint8_t, uint8_t* >    data_list_; // <id, data>

  bool            is_param_changed_;

  uint8_t        *param_;
  uint16_t        start_address_;
  uint16_t        data_length_;

  void    makeParam();

 public:
  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that Initializes instance for Sync Write
  /// @param port PortHandler instance
  /// @param ph PacketHandler instance
  /// @param start_address Address of the data for write
  /// @param data_length Length of the data for write
  ////////////////////////////////////////////////////////////////////////////////
  K_GroupSyncWrite(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that calls clearParam function to clear the parameter list for Sync Write
  ////////////////////////////////////////////////////////////////////////////////
  ~K_GroupSyncWrite(){clearParam(); }

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that returns PortHandler instance
  /// @return PortHandler instance
  ////////////////////////////////////////////////////////////////////////////////
  PortHandler     *getPortHandler()   { return port_; }

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that returns PacketHandler instance
  /// @return PacketHandler instance
  ////////////////////////////////////////////////////////////////////////////////
  PacketHandler   *getPacketHandler() { return ph_; }

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that adds id, start_address, data_length to the Sync Write list
  /// @param id Dynamixel ID
  /// @param data Data for write
  /// @return false
  /// @return   when the ID exists already in the list
  /// @return or true
  ////////////////////////////////////////////////////////////////////////////////
  bool    addParam    (uint8_t id, uint8_t *data);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that removes id from the Sync Write list
  /// @param id Dynamixel ID
  ////////////////////////////////////////////////////////////////////////////////
  void    removeParam (uint8_t id);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that changes the data for write in id -> start_address -> data_length to the Sync Write list
  /// @param id Dynamixel ID
  /// @param data for replacement
  /// @return false
  /// @return   when the ID doesn't exist in the list
  /// @return or true
  ////////////////////////////////////////////////////////////////////////////////
  bool    changeParam (uint8_t id, uint8_t *data);

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that clears the Sync Write list
  ////////////////////////////////////////////////////////////////////////////////
  void    clearParam  ();

  ////////////////////////////////////////////////////////////////////////////////
  /// @brief The function that transmits the Sync Write instruction packet which might be constructed by GroupSyncWrite::addParam function
  /// @return COMM_NOT_AVAILABLE
  /// @return   when the list for Sync Write is empty
  /// @return or the other communication results which come from PacketHandler::syncWriteTxOnly
  ////////////////////////////////////////////////////////////////////////////////
  int     txPacket();
};

//}


#endif // K_GROUP_SYNC_WRITE_H
