#ifndef _ROS_riki_msgs_Bluetooth_h
#define _ROS_riki_msgs_Bluetooth_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

  class Bluetooth : public ros::Msg
  {
    public:
      typedef int32_t _connect_stats_type;
      _connect_stats_type connect_stats;
      typedef int32_t _angle_x_type;
      _angle_x_type angle_x;
      typedef int32_t _angle_y_type;
      _angle_y_type angle_y;

    Bluetooth():
      connect_stats(0),
      angle_x(0),
      angle_y(0)
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
      union {
        int32_t real;
        uint32_t base;
      } u_angle_x;
      u_angle_x.real = this->angle_x;
      *(outbuffer + offset + 0) = (u_angle_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_x);
      union {
        int32_t real;
        uint32_t base;
      } u_angle_y;
      u_angle_y.real = this->angle_y;
      *(outbuffer + offset + 0) = (u_angle_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_y);
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
      union {
        int32_t real;
        uint32_t base;
      } u_angle_x;
      u_angle_x.base = 0;
      u_angle_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_x = u_angle_x.real;
      offset += sizeof(this->angle_x);
      union {
        int32_t real;
        uint32_t base;
      } u_angle_y;
      u_angle_y.base = 0;
      u_angle_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_y = u_angle_y.real;
      offset += sizeof(this->angle_y);
     return offset;
    }

    const char * getType(){ return "riki_msgs/Bluetooth"; };
    const char * getMD5(){ return "d231f5dc5332675e1ab8a685bdfd06a0"; };

  };

}
#endif