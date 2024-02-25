#ifndef _ROS_servo_node_ServoState_h
#define _ROS_servo_node_ServoState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace servo_node
{

  class ServoState : public ros::Msg
  {
    public:
      typedef float _left_ankle_type;
      _left_ankle_type left_ankle;
      typedef float _left_knee_type;
      _left_knee_type left_knee;
      typedef float _left_hip_type;
      _left_hip_type left_hip;
      typedef float _right_ankle_type;
      _right_ankle_type right_ankle;
      typedef float _right_knee_type;
      _right_knee_type right_knee;
      typedef float _right_hip_type;
      _right_hip_type right_hip;
      typedef float _waist_type;
      _waist_type waist;

    ServoState():
      left_ankle(0),
      left_knee(0),
      left_hip(0),
      right_ankle(0),
      right_knee(0),
      right_hip(0),
      waist(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_ankle;
      u_left_ankle.real = this->left_ankle;
      *(outbuffer + offset + 0) = (u_left_ankle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_ankle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_ankle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_ankle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_ankle);
      union {
        float real;
        uint32_t base;
      } u_left_knee;
      u_left_knee.real = this->left_knee;
      *(outbuffer + offset + 0) = (u_left_knee.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_knee.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_knee.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_knee.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_knee);
      union {
        float real;
        uint32_t base;
      } u_left_hip;
      u_left_hip.real = this->left_hip;
      *(outbuffer + offset + 0) = (u_left_hip.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_hip.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_hip.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_hip.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_hip);
      union {
        float real;
        uint32_t base;
      } u_right_ankle;
      u_right_ankle.real = this->right_ankle;
      *(outbuffer + offset + 0) = (u_right_ankle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_ankle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_ankle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_ankle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_ankle);
      union {
        float real;
        uint32_t base;
      } u_right_knee;
      u_right_knee.real = this->right_knee;
      *(outbuffer + offset + 0) = (u_right_knee.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_knee.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_knee.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_knee.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_knee);
      union {
        float real;
        uint32_t base;
      } u_right_hip;
      u_right_hip.real = this->right_hip;
      *(outbuffer + offset + 0) = (u_right_hip.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_hip.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_hip.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_hip.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_hip);
      union {
        float real;
        uint32_t base;
      } u_waist;
      u_waist.real = this->waist;
      *(outbuffer + offset + 0) = (u_waist.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_waist.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_waist.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_waist.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->waist);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_ankle;
      u_left_ankle.base = 0;
      u_left_ankle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_ankle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_ankle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_ankle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_ankle = u_left_ankle.real;
      offset += sizeof(this->left_ankle);
      union {
        float real;
        uint32_t base;
      } u_left_knee;
      u_left_knee.base = 0;
      u_left_knee.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_knee.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_knee.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_knee.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_knee = u_left_knee.real;
      offset += sizeof(this->left_knee);
      union {
        float real;
        uint32_t base;
      } u_left_hip;
      u_left_hip.base = 0;
      u_left_hip.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_hip.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_hip.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_hip.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_hip = u_left_hip.real;
      offset += sizeof(this->left_hip);
      union {
        float real;
        uint32_t base;
      } u_right_ankle;
      u_right_ankle.base = 0;
      u_right_ankle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_ankle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_ankle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_ankle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_ankle = u_right_ankle.real;
      offset += sizeof(this->right_ankle);
      union {
        float real;
        uint32_t base;
      } u_right_knee;
      u_right_knee.base = 0;
      u_right_knee.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_knee.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_knee.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_knee.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_knee = u_right_knee.real;
      offset += sizeof(this->right_knee);
      union {
        float real;
        uint32_t base;
      } u_right_hip;
      u_right_hip.base = 0;
      u_right_hip.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_hip.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_hip.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_hip.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_hip = u_right_hip.real;
      offset += sizeof(this->right_hip);
      union {
        float real;
        uint32_t base;
      } u_waist;
      u_waist.base = 0;
      u_waist.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_waist.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_waist.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_waist.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->waist = u_waist.real;
      offset += sizeof(this->waist);
     return offset;
    }

    virtual const char * getType() override { return "servo_node/ServoState"; };
    virtual const char * getMD5() override { return "7ce4a32deea9979dbb9ca8577d26cdb8"; };

  };

}
#endif
