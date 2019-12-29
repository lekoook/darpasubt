#ifndef _ROS_talker_pkg_LoraPacket_h
#define _ROS_talker_pkg_LoraPacket_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"

namespace talker_pkg
{

  class LoraPacket : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef std_msgs::String _to_type;
      _to_type to;
      typedef std_msgs::String _from_type;
      _from_type from;
      typedef int32_t _rssi_type;
      _rssi_type rssi;
      uint32_t data_length;
      typedef uint8_t _data_type;
      _data_type st_data;
      _data_type * data;

    LoraPacket():
      header(),
      to(),
      from(),
      rssi(0),
      data_length(0), data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->to.serialize(outbuffer + offset);
      offset += this->from.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_rssi;
      u_rssi.real = this->rssi;
      *(outbuffer + offset + 0) = (u_rssi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rssi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rssi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rssi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rssi);
      *(outbuffer + offset + 0) = (this->data_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data_length);
      for( uint32_t i = 0; i < data_length; i++){
      *(outbuffer + offset + 0) = (this->data[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->to.deserialize(inbuffer + offset);
      offset += this->from.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_rssi;
      u_rssi.base = 0;
      u_rssi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rssi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rssi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rssi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rssi = u_rssi.real;
      offset += sizeof(this->rssi);
      uint32_t data_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      data_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->data_length);
      if(data_lengthT > data_length)
        this->data = (uint8_t*)realloc(this->data, data_lengthT * sizeof(uint8_t));
      data_length = data_lengthT;
      for( uint32_t i = 0; i < data_length; i++){
      this->st_data =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_data);
        memcpy( &(this->data[i]), &(this->st_data), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "talker_pkg/LoraPacket"; };
    const char * getMD5(){ return "47239ccc011e5edd53de6d7bc7d74d2d"; };

  };

}
#endif
