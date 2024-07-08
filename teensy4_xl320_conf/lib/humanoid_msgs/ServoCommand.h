#ifndef _ROS_humanoid_msgs_ServoCommand_h
#define _ROS_humanoid_msgs_ServoCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace humanoid_msgs
{

  class ServoCommand : public ros::Msg
  {
    public:
      typedef float _right_shoulder_pitch_type;
      _right_shoulder_pitch_type right_shoulder_pitch;
      typedef float _right_shoulder_roll_type;
      _right_shoulder_roll_type right_shoulder_roll;
      typedef float _right_elbow_type;
      _right_elbow_type right_elbow;
      typedef float _left_shoulder_pitch_type;
      _left_shoulder_pitch_type left_shoulder_pitch;
      typedef float _left_shoulder_roll_type;
      _left_shoulder_roll_type left_shoulder_roll;
      typedef float _left_elbow_type;
      _left_elbow_type left_elbow;
      typedef float _left_hip_roll_type;
      _left_hip_roll_type left_hip_roll;
      typedef float _left_hip_pitch_type;
      _left_hip_pitch_type left_hip_pitch;
      typedef float _left_knee_type;
      _left_knee_type left_knee;
      typedef float _right_hip_roll_type;
      _right_hip_roll_type right_hip_roll;
      typedef float _right_hip_pitch_type;
      _right_hip_pitch_type right_hip_pitch;
      typedef float _right_knee_type;
      _right_knee_type right_knee;

    ServoCommand():
      right_shoulder_pitch(0),
      right_shoulder_roll(0),
      right_elbow(0),
      left_shoulder_pitch(0),
      left_shoulder_roll(0),
      left_elbow(0),
      left_hip_roll(0),
      left_hip_pitch(0),
      left_knee(0),
      right_hip_roll(0),
      right_hip_pitch(0),
      right_knee(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_right_shoulder_pitch;
      u_right_shoulder_pitch.real = this->right_shoulder_pitch;
      *(outbuffer + offset + 0) = (u_right_shoulder_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_shoulder_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_shoulder_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_shoulder_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_shoulder_pitch);
      union {
        float real;
        uint32_t base;
      } u_right_shoulder_roll;
      u_right_shoulder_roll.real = this->right_shoulder_roll;
      *(outbuffer + offset + 0) = (u_right_shoulder_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_shoulder_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_shoulder_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_shoulder_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_shoulder_roll);
      union {
        float real;
        uint32_t base;
      } u_right_elbow;
      u_right_elbow.real = this->right_elbow;
      *(outbuffer + offset + 0) = (u_right_elbow.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_elbow.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_elbow.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_elbow.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_elbow);
      union {
        float real;
        uint32_t base;
      } u_left_shoulder_pitch;
      u_left_shoulder_pitch.real = this->left_shoulder_pitch;
      *(outbuffer + offset + 0) = (u_left_shoulder_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_shoulder_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_shoulder_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_shoulder_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_shoulder_pitch);
      union {
        float real;
        uint32_t base;
      } u_left_shoulder_roll;
      u_left_shoulder_roll.real = this->left_shoulder_roll;
      *(outbuffer + offset + 0) = (u_left_shoulder_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_shoulder_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_shoulder_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_shoulder_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_shoulder_roll);
      union {
        float real;
        uint32_t base;
      } u_left_elbow;
      u_left_elbow.real = this->left_elbow;
      *(outbuffer + offset + 0) = (u_left_elbow.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_elbow.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_elbow.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_elbow.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_elbow);
      union {
        float real;
        uint32_t base;
      } u_left_hip_roll;
      u_left_hip_roll.real = this->left_hip_roll;
      *(outbuffer + offset + 0) = (u_left_hip_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_hip_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_hip_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_hip_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_hip_roll);
      union {
        float real;
        uint32_t base;
      } u_left_hip_pitch;
      u_left_hip_pitch.real = this->left_hip_pitch;
      *(outbuffer + offset + 0) = (u_left_hip_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_hip_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_hip_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_hip_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_hip_pitch);
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
      } u_right_hip_roll;
      u_right_hip_roll.real = this->right_hip_roll;
      *(outbuffer + offset + 0) = (u_right_hip_roll.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_hip_roll.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_hip_roll.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_hip_roll.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_hip_roll);
      union {
        float real;
        uint32_t base;
      } u_right_hip_pitch;
      u_right_hip_pitch.real = this->right_hip_pitch;
      *(outbuffer + offset + 0) = (u_right_hip_pitch.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_hip_pitch.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_hip_pitch.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_hip_pitch.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_hip_pitch);
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_right_shoulder_pitch;
      u_right_shoulder_pitch.base = 0;
      u_right_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_shoulder_pitch = u_right_shoulder_pitch.real;
      offset += sizeof(this->right_shoulder_pitch);
      union {
        float real;
        uint32_t base;
      } u_right_shoulder_roll;
      u_right_shoulder_roll.base = 0;
      u_right_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_shoulder_roll = u_right_shoulder_roll.real;
      offset += sizeof(this->right_shoulder_roll);
      union {
        float real;
        uint32_t base;
      } u_right_elbow;
      u_right_elbow.base = 0;
      u_right_elbow.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_elbow.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_elbow.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_elbow.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_elbow = u_right_elbow.real;
      offset += sizeof(this->right_elbow);
      union {
        float real;
        uint32_t base;
      } u_left_shoulder_pitch;
      u_left_shoulder_pitch.base = 0;
      u_left_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_shoulder_pitch = u_left_shoulder_pitch.real;
      offset += sizeof(this->left_shoulder_pitch);
      union {
        float real;
        uint32_t base;
      } u_left_shoulder_roll;
      u_left_shoulder_roll.base = 0;
      u_left_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_shoulder_roll = u_left_shoulder_roll.real;
      offset += sizeof(this->left_shoulder_roll);
      union {
        float real;
        uint32_t base;
      } u_left_elbow;
      u_left_elbow.base = 0;
      u_left_elbow.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_elbow.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_elbow.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_elbow.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_elbow = u_left_elbow.real;
      offset += sizeof(this->left_elbow);
      union {
        float real;
        uint32_t base;
      } u_left_hip_roll;
      u_left_hip_roll.base = 0;
      u_left_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_hip_roll = u_left_hip_roll.real;
      offset += sizeof(this->left_hip_roll);
      union {
        float real;
        uint32_t base;
      } u_left_hip_pitch;
      u_left_hip_pitch.base = 0;
      u_left_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_hip_pitch = u_left_hip_pitch.real;
      offset += sizeof(this->left_hip_pitch);
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
      } u_right_hip_roll;
      u_right_hip_roll.base = 0;
      u_right_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_hip_roll = u_right_hip_roll.real;
      offset += sizeof(this->right_hip_roll);
      union {
        float real;
        uint32_t base;
      } u_right_hip_pitch;
      u_right_hip_pitch.base = 0;
      u_right_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_hip_pitch = u_right_hip_pitch.real;
      offset += sizeof(this->right_hip_pitch);
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
     return offset;
    }

    virtual const char * getType() override { return "humanoid_msgs/ServoCommand"; };
    virtual const char * getMD5() override { return "194e7ec966ee325787f88586266091be"; };

  };

}
#endif
