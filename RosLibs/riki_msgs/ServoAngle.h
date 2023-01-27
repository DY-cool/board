#ifndef _ROS_SERVICE_ServoAngle_h
#define _ROS_SERVICE_ServoAngle_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace riki_msgs
{

static const char SERVOANGLE[] = "riki_msgs/ServoAngle";

  class ServoAngleRequest : public ros::Msg
  {
    public:
      typedef int32_t _Servo1_type;
      _Servo1_type Servo1;
      typedef int32_t _Servo2_type;
      _Servo2_type Servo2;

    ServoAngleRequest():
      Servo1(0),
      Servo2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_Servo1;
      u_Servo1.real = this->Servo1;
      *(outbuffer + offset + 0) = (u_Servo1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Servo1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Servo1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Servo1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Servo1);
      union {
        int32_t real;
        uint32_t base;
      } u_Servo2;
      u_Servo2.real = this->Servo2;
      *(outbuffer + offset + 0) = (u_Servo2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Servo2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Servo2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Servo2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Servo2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_Servo1;
      u_Servo1.base = 0;
      u_Servo1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Servo1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Servo1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Servo1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Servo1 = u_Servo1.real;
      offset += sizeof(this->Servo1);
      union {
        int32_t real;
        uint32_t base;
      } u_Servo2;
      u_Servo2.base = 0;
      u_Servo2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Servo2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Servo2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Servo2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Servo2 = u_Servo2.real;
      offset += sizeof(this->Servo2);
     return offset;
    }

    const char * getType(){ return SERVOANGLE; };
    const char * getMD5(){ return "b1d39e7bb90536edb373b61068d04284"; };

  };

  class ServoAngleResponse : public ros::Msg
  {
    public:

    ServoAngleResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return SERVOANGLE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ServoAngle {
    public:
    typedef ServoAngleRequest Request;
    typedef ServoAngleResponse Response;
  };

}
#endif
