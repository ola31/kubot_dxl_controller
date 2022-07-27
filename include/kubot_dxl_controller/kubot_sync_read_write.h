#ifndef KUBOT_SYNC_READ_WRITE_H
#define KUBOT_SYNC_READ_WRITE_H

#include <ros/ros.h>

#include "dynamixel_sdk/packet_handler.h"

#include "dynamixel_sdk/protocol2_packet_handler.h"

using namespace dynamixel;

class kubot_sync_read_write : public PacketHandler
{
  //Protocol2PacketHandler();
  private:
    static Protocol2PacketHandler *unique_instance_;
    uint16_t    updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);
    void        addStuffing(uint8_t *packet);
    void        removeStuffing(uint8_t *packet);
  public:

    int txPacket(PortHandler *port, uint8_t *txpacket);
    int syncWriteTxOnly(PortHandler *port, uint16_t start_address, uint16_t data_length, uint8_t *param, uint16_t param_length);
};

#endif // KUBOT_SYNC_READ_WRITE_H
