#ifndef _ROS_riki_msgs_Blue_connect_h
#define _ROS_riki_msgs_Blue_connect_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class Blue_connect : public ros::Msg
  {
    public:
      typedef int32_t _connect_stats_type;
      _connect_stats_type connect_stats;

    Blue_connect():
      connect_stats(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_connect_stats;
      u_connect_stats.real = this->connect_stats;
      *(outbuffer + offset + 0) = (u_connect_stats.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_connect_stats.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_connect_stats.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_connect_stats.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->connect_stats);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_connect_stats;
      u_connect_stats.base = 0;
      u_connect_stats.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_connect_stats.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_connect_stats.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_connect_stats.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->connect_stats = u_connect_stats.real;
      offset += sizeof(this->connect_stats);
     return offset;
    }

    const char * getType(){ return "riki_msgs/Blue_connect"; };
    const char * getMD5(){ return "689b4c4cbf04148501838010cd8ac299"; };

  };

}
#endif