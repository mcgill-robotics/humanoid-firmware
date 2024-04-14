#ifndef _ROS_humanoid_msgs_PressureSensors_h
#define _ROS_humanoid_msgs_PressureSensors_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace humanoid_msgs
{

  class PressureSensors : public ros::Msg
  {
    public:
      typedef float _right_left_forward_type;
      _right_left_forward_type right_left_forward;
      typedef float _right_left_back_type;
      _right_left_back_type right_left_back;
      typedef float _right_right_forward_type;
      _right_right_forward_type right_right_forward;
      typedef float _right_right_back_type;
      _right_right_back_type right_right_back;
      typedef float _left_left_forward_type;
      _left_left_forward_type left_left_forward;
      typedef float _left_left_back_type;
      _left_left_back_type left_left_back;
      typedef float _left_right_forward_type;
      _left_right_forward_type left_right_forward;
      typedef float _left_right_back_type;
      _left_right_back_type left_right_back;

    PressureSensors():
      right_left_forward(0),
      right_left_back(0),
      right_right_forward(0),
      right_right_back(0),
      left_left_forward(0),
      left_left_back(0),
      left_right_forward(0),
      left_right_back(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_right_left_forward;
      u_right_left_forward.real = this->right_left_forward;
      *(outbuffer + offset + 0) = (u_right_left_forward.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_left_forward.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_left_forward.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_left_forward.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_left_forward);
      union {
        float real;
        uint32_t base;
      } u_right_left_back;
      u_right_left_back.real = this->right_left_back;
      *(outbuffer + offset + 0) = (u_right_left_back.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_left_back.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_left_back.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_left_back.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_left_back);
      union {
        float real;
        uint32_t base;
      } u_right_right_forward;
      u_right_right_forward.real = this->right_right_forward;
      *(outbuffer + offset + 0) = (u_right_right_forward.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_right_forward.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_right_forward.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_right_forward.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_right_forward);
      union {
        float real;
        uint32_t base;
      } u_right_right_back;
      u_right_right_back.real = this->right_right_back;
      *(outbuffer + offset + 0) = (u_right_right_back.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_right_back.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_right_back.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_right_back.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_right_back);
      union {
        float real;
        uint32_t base;
      } u_left_left_forward;
      u_left_left_forward.real = this->left_left_forward;
      *(outbuffer + offset + 0) = (u_left_left_forward.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_left_forward.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_left_forward.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_left_forward.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_left_forward);
      union {
        float real;
        uint32_t base;
      } u_left_left_back;
      u_left_left_back.real = this->left_left_back;
      *(outbuffer + offset + 0) = (u_left_left_back.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_left_back.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_left_back.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_left_back.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_left_back);
      union {
        float real;
        uint32_t base;
      } u_left_right_forward;
      u_left_right_forward.real = this->left_right_forward;
      *(outbuffer + offset + 0) = (u_left_right_forward.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_right_forward.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_right_forward.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_right_forward.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_right_forward);
      union {
        float real;
        uint32_t base;
      } u_left_right_back;
      u_left_right_back.real = this->left_right_back;
      *(outbuffer + offset + 0) = (u_left_right_back.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_right_back.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_right_back.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_right_back.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_right_back);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_right_left_forward;
      u_right_left_forward.base = 0;
      u_right_left_forward.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_left_forward.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_left_forward.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_left_forward.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_left_forward = u_right_left_forward.real;
      offset += sizeof(this->right_left_forward);
      union {
        float real;
        uint32_t base;
      } u_right_left_back;
      u_right_left_back.base = 0;
      u_right_left_back.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_left_back.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_left_back.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_left_back.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_left_back = u_right_left_back.real;
      offset += sizeof(this->right_left_back);
      union {
        float real;
        uint32_t base;
      } u_right_right_forward;
      u_right_right_forward.base = 0;
      u_right_right_forward.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_right_forward.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_right_forward.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_right_forward.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_right_forward = u_right_right_forward.real;
      offset += sizeof(this->right_right_forward);
      union {
        float real;
        uint32_t base;
      } u_right_right_back;
      u_right_right_back.base = 0;
      u_right_right_back.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_right_back.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_right_back.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_right_back.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_right_back = u_right_right_back.real;
      offset += sizeof(this->right_right_back);
      union {
        float real;
        uint32_t base;
      } u_left_left_forward;
      u_left_left_forward.base = 0;
      u_left_left_forward.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_left_forward.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_left_forward.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_left_forward.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_left_forward = u_left_left_forward.real;
      offset += sizeof(this->left_left_forward);
      union {
        float real;
        uint32_t base;
      } u_left_left_back;
      u_left_left_back.base = 0;
      u_left_left_back.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_left_back.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_left_back.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_left_back.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_left_back = u_left_left_back.real;
      offset += sizeof(this->left_left_back);
      union {
        float real;
        uint32_t base;
      } u_left_right_forward;
      u_left_right_forward.base = 0;
      u_left_right_forward.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_right_forward.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_right_forward.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_right_forward.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_right_forward = u_left_right_forward.real;
      offset += sizeof(this->left_right_forward);
      union {
        float real;
        uint32_t base;
      } u_left_right_back;
      u_left_right_back.base = 0;
      u_left_right_back.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_right_back.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_right_back.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_right_back.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_right_back = u_left_right_back.real;
      offset += sizeof(this->left_right_back);
     return offset;
    }

    virtual const char * getType() override { return "humanoid_msgs/PressureSensors"; };
    virtual const char * getMD5() override { return "945eced6179d93a1274f5acf26fbad42"; };

  };

}
#endif
