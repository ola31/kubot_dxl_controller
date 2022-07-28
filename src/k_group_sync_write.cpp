#include <algorithm>
#include "kubot_dxl_controller/k_group_sync_write.h"


using namespace dynamixel;

K_GroupSyncWrite::K_GroupSyncWrite(PortHandler *port, PacketHandler *ph, uint16_t start_address, uint16_t data_length)
  : port_(port),
    ph_(ph),
    //k_ph((kubot_sync_read_write)ph),
    is_param_changed_(false),
    param_(0),
    start_address_(start_address),
    data_length_(data_length)
{
  clearParam();
}

void K_GroupSyncWrite::makeParam()
{
  if (id_list_.size() == 0) return;

  if (param_ != 0)
    delete[] param_;
  param_ = 0;

  param_ = new uint8_t[id_list_.size() * (1 + data_length_)]; // ID(1) + DATA(data_length)

  int idx = 0;
  for (unsigned int i = 0; i < id_list_.size(); i++)
  {
    uint8_t id = id_list_[i];
    if (data_list_[id] == 0)
      return;

    param_[idx++] = id;
    for (int c = 0; c < data_length_; c++)
      param_[idx++] = (data_list_[id])[c];
  }
}

bool K_GroupSyncWrite::addParam(uint8_t id, uint8_t *data)
{
  if (std::find(id_list_.begin(), id_list_.end(), id) != id_list_.end())   // id already exist
    return false;

  id_list_.push_back(id);
  data_list_[id]    = new uint8_t[data_length_];
  for (int c = 0; c < data_length_; c++)
    data_list_[id][c] = data[c];

  is_param_changed_   = true;
  return true;
}

void K_GroupSyncWrite::removeParam(uint8_t id)
{
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return;

  id_list_.erase(it);
  delete[] data_list_[id];
  data_list_.erase(id);

  is_param_changed_   = true;
}

bool K_GroupSyncWrite::changeParam(uint8_t id, uint8_t *data)
{
  std::vector<uint8_t>::iterator it = std::find(id_list_.begin(), id_list_.end(), id);
  if (it == id_list_.end())    // NOT exist
    return false;

  delete[] data_list_[id];
  data_list_[id]    = new uint8_t[data_length_];
  for (int c = 0; c < data_length_; c++)
    data_list_[id][c] = data[c];

  is_param_changed_   = true;
  return true;
}

void K_GroupSyncWrite::clearParam()
{
  if (id_list_.size() == 0)
    return;

  for (unsigned int i = 0; i < id_list_.size(); i++)
    delete[] data_list_[id_list_[i]];

  id_list_.clear();
  data_list_.clear();
  if (param_ != 0)
    delete[] param_;
  param_ = 0;
}

int K_GroupSyncWrite::txPacket1()
{

  if (id_list_.size() == 0)
    return COMM_NOT_AVAILABLE;
  //printf("dddd\n");


  if (is_param_changed_ == true || param_ == 0)
    makeParam();

  return ph_->syncWriteTxOnly(port_, start_address_, data_length_, param_, id_list_.size() * (1 + data_length_));

}


