#ifndef KUBOT_SYNC_READ_WRITE_H
#define KUBOT_SYNC_READ_WRITE_H

#include <ros/ros.h>

#include "dynamixel_sdk/protocol2_packet_handler.h"

using namespace dynamixel;

class kubot_sync_read_write : private Protocol2PacketHandler
{
private:


public:

  int txPacket(PortHandler *port, uint8_t *txpacket);
};

#endif // KUBOT_SYNC_READ_WRITE_H
