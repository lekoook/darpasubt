#ifndef _ROS_vehicle_drive_Dropper_h
#define _ROS_vehicle_drive_Dropper_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace vehicle_drive
{

  class Dropper : public ros::Msg
  {
    public:
      uint32_t dropper_angles_length;
      typedef uint8_t _dropper_angles_type;
      _dropper_angles_type st_dropper_angles;
      _dropper_angles_type * dropper_angles;

    Dropper():
      dropper_angles_length(0), dropper_angles(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->dropper_angles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dropper_angles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dropper_angles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dropper_angles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dropper_angles_length);
      for( uint32_t i = 0; i < dropper_angles_length; i++){
      *(outbuffer + offset + 0) = (this->dropper_angles[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dropper_angles[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t dropper_angles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dropper_angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dropper_angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dropper_angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dropper_angles_length);
      if(dropper_angles_lengthT > dropper_angles_length)
        this->dropper_angles = (uint8_t*)realloc(this->dropper_angles, dropper_angles_lengthT * sizeof(uint8_t));
      dropper_angles_length = dropper_angles_lengthT;
      for( uint32_t i = 0; i < dropper_angles_length; i++){
      this->st_dropper_angles =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_dropper_angles);
        memcpy( &(this->dropper_angles[i]), &(this->st_dropper_angles), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "vehicle_drive/Dropper"; };
    const char * getMD5(){ return "632290e64ddf4d9a92b02ad7583e184e"; };

  };

}
#endif
