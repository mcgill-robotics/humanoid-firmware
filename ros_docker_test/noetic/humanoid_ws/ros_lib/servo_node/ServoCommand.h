#ifndef _ROS_servo_node_ServoCommand_h
#define _ROS_servo_node_ServoCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace servo_node
{

  class ServoCommand : public ros::Msg
  {
    public:
      typedef float _left_leg_ankle_setpoint_type;
      _left_leg_ankle_setpoint_type left_leg_ankle_setpoint;
      typedef float _left_leg_knee_setpoint_type;
      _left_leg_knee_setpoint_type left_leg_knee_setpoint;
      typedef float _left_leg_hip_roll_setpoint_type;
      _left_leg_hip_roll_setpoint_type left_leg_hip_roll_setpoint;
      typedef float _left_leg_hip_pitch_setpoint_type;
      _left_leg_hip_pitch_setpoint_type left_leg_hip_pitch_setpoint;
      typedef float _left_leg_hip_yaw_setpoint_type;
      _left_leg_hip_yaw_setpoint_type left_leg_hip_yaw_setpoint;
      typedef float _right_leg_ankle_setpoint_type;
      _right_leg_ankle_setpoint_type right_leg_ankle_setpoint;
      typedef float _right_leg_knee_setpoint_type;
      _right_leg_knee_setpoint_type right_leg_knee_setpoint;
      typedef float _right_leg_hip_roll_setpoint_type;
      _right_leg_hip_roll_setpoint_type right_leg_hip_roll_setpoint;
      typedef float _right_leg_hip_pitch_setpoint_type;
      _right_leg_hip_pitch_setpoint_type right_leg_hip_pitch_setpoint;
      typedef float _right_leg_hip_yaw_setpoint_type;
      _right_leg_hip_yaw_setpoint_type right_leg_hip_yaw_setpoint;
      typedef float _torso_roll_setpoint_type;
      _torso_roll_setpoint_type torso_roll_setpoint;
      typedef float _torso_yaw_setpoint_type;
      _torso_yaw_setpoint_type torso_yaw_setpoint;
      typedef float _left_arm_elbow_setpoint_type;
      _left_arm_elbow_setpoint_type left_arm_elbow_setpoint;
      typedef float _right_arm_elbow_setpoint_type;
      _right_arm_elbow_setpoint_type right_arm_elbow_setpoint;
      typedef float _left_arm_shoulder_setpoint_type;
      _left_arm_shoulder_setpoint_type left_arm_shoulder_setpoint;
      typedef float _right_arm_shoulder_setpoint_type;
      _right_arm_shoulder_setpoint_type right_arm_shoulder_setpoint;

    ServoCommand():
      left_leg_ankle_setpoint(0),
      left_leg_knee_setpoint(0),
      left_leg_hip_roll_setpoint(0),
      left_leg_hip_pitch_setpoint(0),
      left_leg_hip_yaw_setpoint(0),
      right_leg_ankle_setpoint(0),
      right_leg_knee_setpoint(0),
      right_leg_hip_roll_setpoint(0),
      right_leg_hip_pitch_setpoint(0),
      right_leg_hip_yaw_setpoint(0),
      torso_roll_setpoint(0),
      torso_yaw_setpoint(0),
      left_arm_elbow_setpoint(0),
      right_arm_elbow_setpoint(0),
      left_arm_shoulder_setpoint(0),
      right_arm_shoulder_setpoint(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_leg_ankle_setpoint;
      u_left_leg_ankle_setpoint.real = this->left_leg_ankle_setpoint;
      *(outbuffer + offset + 0) = (u_left_leg_ankle_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_leg_ankle_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_leg_ankle_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_leg_ankle_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_ankle_setpoint);
      union {
        float real;
        uint32_t base;
      } u_left_leg_knee_setpoint;
      u_left_leg_knee_setpoint.real = this->left_leg_knee_setpoint;
      *(outbuffer + offset + 0) = (u_left_leg_knee_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_leg_knee_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_leg_knee_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_leg_knee_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_knee_setpoint);
      union {
        float real;
        uint32_t base;
      } u_left_leg_hip_roll_setpoint;
      u_left_leg_hip_roll_setpoint.real = this->left_leg_hip_roll_setpoint;
      *(outbuffer + offset + 0) = (u_left_leg_hip_roll_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_leg_hip_roll_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_leg_hip_roll_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_leg_hip_roll_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_hip_roll_setpoint);
      union {
        float real;
        uint32_t base;
      } u_left_leg_hip_pitch_setpoint;
      u_left_leg_hip_pitch_setpoint.real = this->left_leg_hip_pitch_setpoint;
      *(outbuffer + offset + 0) = (u_left_leg_hip_pitch_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_leg_hip_pitch_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_leg_hip_pitch_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_leg_hip_pitch_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_hip_pitch_setpoint);
      union {
        float real;
        uint32_t base;
      } u_left_leg_hip_yaw_setpoint;
      u_left_leg_hip_yaw_setpoint.real = this->left_leg_hip_yaw_setpoint;
      *(outbuffer + offset + 0) = (u_left_leg_hip_yaw_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_leg_hip_yaw_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_leg_hip_yaw_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_leg_hip_yaw_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_hip_yaw_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_leg_ankle_setpoint;
      u_right_leg_ankle_setpoint.real = this->right_leg_ankle_setpoint;
      *(outbuffer + offset + 0) = (u_right_leg_ankle_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_leg_ankle_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_leg_ankle_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_leg_ankle_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_ankle_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_leg_knee_setpoint;
      u_right_leg_knee_setpoint.real = this->right_leg_knee_setpoint;
      *(outbuffer + offset + 0) = (u_right_leg_knee_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_leg_knee_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_leg_knee_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_leg_knee_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_knee_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_leg_hip_roll_setpoint;
      u_right_leg_hip_roll_setpoint.real = this->right_leg_hip_roll_setpoint;
      *(outbuffer + offset + 0) = (u_right_leg_hip_roll_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_leg_hip_roll_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_leg_hip_roll_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_leg_hip_roll_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_hip_roll_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_leg_hip_pitch_setpoint;
      u_right_leg_hip_pitch_setpoint.real = this->right_leg_hip_pitch_setpoint;
      *(outbuffer + offset + 0) = (u_right_leg_hip_pitch_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_leg_hip_pitch_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_leg_hip_pitch_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_leg_hip_pitch_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_hip_pitch_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_leg_hip_yaw_setpoint;
      u_right_leg_hip_yaw_setpoint.real = this->right_leg_hip_yaw_setpoint;
      *(outbuffer + offset + 0) = (u_right_leg_hip_yaw_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_leg_hip_yaw_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_leg_hip_yaw_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_leg_hip_yaw_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_hip_yaw_setpoint);
      union {
        float real;
        uint32_t base;
      } u_torso_roll_setpoint;
      u_torso_roll_setpoint.real = this->torso_roll_setpoint;
      *(outbuffer + offset + 0) = (u_torso_roll_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torso_roll_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torso_roll_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torso_roll_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torso_roll_setpoint);
      union {
        float real;
        uint32_t base;
      } u_torso_yaw_setpoint;
      u_torso_yaw_setpoint.real = this->torso_yaw_setpoint;
      *(outbuffer + offset + 0) = (u_torso_yaw_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torso_yaw_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torso_yaw_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torso_yaw_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torso_yaw_setpoint);
      union {
        float real;
        uint32_t base;
      } u_left_arm_elbow_setpoint;
      u_left_arm_elbow_setpoint.real = this->left_arm_elbow_setpoint;
      *(outbuffer + offset + 0) = (u_left_arm_elbow_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_arm_elbow_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_arm_elbow_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_arm_elbow_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_arm_elbow_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_arm_elbow_setpoint;
      u_right_arm_elbow_setpoint.real = this->right_arm_elbow_setpoint;
      *(outbuffer + offset + 0) = (u_right_arm_elbow_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_arm_elbow_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_arm_elbow_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_arm_elbow_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_arm_elbow_setpoint);
      union {
        float real;
        uint32_t base;
      } u_left_arm_shoulder_setpoint;
      u_left_arm_shoulder_setpoint.real = this->left_arm_shoulder_setpoint;
      *(outbuffer + offset + 0) = (u_left_arm_shoulder_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_arm_shoulder_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_arm_shoulder_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_arm_shoulder_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_arm_shoulder_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_arm_shoulder_setpoint;
      u_right_arm_shoulder_setpoint.real = this->right_arm_shoulder_setpoint;
      *(outbuffer + offset + 0) = (u_right_arm_shoulder_setpoint.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_arm_shoulder_setpoint.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_arm_shoulder_setpoint.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_arm_shoulder_setpoint.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_arm_shoulder_setpoint);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_leg_ankle_setpoint;
      u_left_leg_ankle_setpoint.base = 0;
      u_left_leg_ankle_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_leg_ankle_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_leg_ankle_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_leg_ankle_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_leg_ankle_setpoint = u_left_leg_ankle_setpoint.real;
      offset += sizeof(this->left_leg_ankle_setpoint);
      union {
        float real;
        uint32_t base;
      } u_left_leg_knee_setpoint;
      u_left_leg_knee_setpoint.base = 0;
      u_left_leg_knee_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_leg_knee_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_leg_knee_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_leg_knee_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_leg_knee_setpoint = u_left_leg_knee_setpoint.real;
      offset += sizeof(this->left_leg_knee_setpoint);
      union {
        float real;
        uint32_t base;
      } u_left_leg_hip_roll_setpoint;
      u_left_leg_hip_roll_setpoint.base = 0;
      u_left_leg_hip_roll_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_leg_hip_roll_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_leg_hip_roll_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_leg_hip_roll_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_leg_hip_roll_setpoint = u_left_leg_hip_roll_setpoint.real;
      offset += sizeof(this->left_leg_hip_roll_setpoint);
      union {
        float real;
        uint32_t base;
      } u_left_leg_hip_pitch_setpoint;
      u_left_leg_hip_pitch_setpoint.base = 0;
      u_left_leg_hip_pitch_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_leg_hip_pitch_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_leg_hip_pitch_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_leg_hip_pitch_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_leg_hip_pitch_setpoint = u_left_leg_hip_pitch_setpoint.real;
      offset += sizeof(this->left_leg_hip_pitch_setpoint);
      union {
        float real;
        uint32_t base;
      } u_left_leg_hip_yaw_setpoint;
      u_left_leg_hip_yaw_setpoint.base = 0;
      u_left_leg_hip_yaw_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_leg_hip_yaw_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_leg_hip_yaw_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_leg_hip_yaw_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_leg_hip_yaw_setpoint = u_left_leg_hip_yaw_setpoint.real;
      offset += sizeof(this->left_leg_hip_yaw_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_leg_ankle_setpoint;
      u_right_leg_ankle_setpoint.base = 0;
      u_right_leg_ankle_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_leg_ankle_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_leg_ankle_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_leg_ankle_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_leg_ankle_setpoint = u_right_leg_ankle_setpoint.real;
      offset += sizeof(this->right_leg_ankle_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_leg_knee_setpoint;
      u_right_leg_knee_setpoint.base = 0;
      u_right_leg_knee_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_leg_knee_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_leg_knee_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_leg_knee_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_leg_knee_setpoint = u_right_leg_knee_setpoint.real;
      offset += sizeof(this->right_leg_knee_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_leg_hip_roll_setpoint;
      u_right_leg_hip_roll_setpoint.base = 0;
      u_right_leg_hip_roll_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_leg_hip_roll_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_leg_hip_roll_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_leg_hip_roll_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_leg_hip_roll_setpoint = u_right_leg_hip_roll_setpoint.real;
      offset += sizeof(this->right_leg_hip_roll_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_leg_hip_pitch_setpoint;
      u_right_leg_hip_pitch_setpoint.base = 0;
      u_right_leg_hip_pitch_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_leg_hip_pitch_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_leg_hip_pitch_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_leg_hip_pitch_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_leg_hip_pitch_setpoint = u_right_leg_hip_pitch_setpoint.real;
      offset += sizeof(this->right_leg_hip_pitch_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_leg_hip_yaw_setpoint;
      u_right_leg_hip_yaw_setpoint.base = 0;
      u_right_leg_hip_yaw_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_leg_hip_yaw_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_leg_hip_yaw_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_leg_hip_yaw_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_leg_hip_yaw_setpoint = u_right_leg_hip_yaw_setpoint.real;
      offset += sizeof(this->right_leg_hip_yaw_setpoint);
      union {
        float real;
        uint32_t base;
      } u_torso_roll_setpoint;
      u_torso_roll_setpoint.base = 0;
      u_torso_roll_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_torso_roll_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_torso_roll_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_torso_roll_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->torso_roll_setpoint = u_torso_roll_setpoint.real;
      offset += sizeof(this->torso_roll_setpoint);
      union {
        float real;
        uint32_t base;
      } u_torso_yaw_setpoint;
      u_torso_yaw_setpoint.base = 0;
      u_torso_yaw_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_torso_yaw_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_torso_yaw_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_torso_yaw_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->torso_yaw_setpoint = u_torso_yaw_setpoint.real;
      offset += sizeof(this->torso_yaw_setpoint);
      union {
        float real;
        uint32_t base;
      } u_left_arm_elbow_setpoint;
      u_left_arm_elbow_setpoint.base = 0;
      u_left_arm_elbow_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_arm_elbow_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_arm_elbow_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_arm_elbow_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_arm_elbow_setpoint = u_left_arm_elbow_setpoint.real;
      offset += sizeof(this->left_arm_elbow_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_arm_elbow_setpoint;
      u_right_arm_elbow_setpoint.base = 0;
      u_right_arm_elbow_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_arm_elbow_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_arm_elbow_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_arm_elbow_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_arm_elbow_setpoint = u_right_arm_elbow_setpoint.real;
      offset += sizeof(this->right_arm_elbow_setpoint);
      union {
        float real;
        uint32_t base;
      } u_left_arm_shoulder_setpoint;
      u_left_arm_shoulder_setpoint.base = 0;
      u_left_arm_shoulder_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_arm_shoulder_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_arm_shoulder_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_arm_shoulder_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_arm_shoulder_setpoint = u_left_arm_shoulder_setpoint.real;
      offset += sizeof(this->left_arm_shoulder_setpoint);
      union {
        float real;
        uint32_t base;
      } u_right_arm_shoulder_setpoint;
      u_right_arm_shoulder_setpoint.base = 0;
      u_right_arm_shoulder_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_arm_shoulder_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_arm_shoulder_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_arm_shoulder_setpoint.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_arm_shoulder_setpoint = u_right_arm_shoulder_setpoint.real;
      offset += sizeof(this->right_arm_shoulder_setpoint);
     return offset;
    }

    virtual const char * getType() override { return "servo_node/ServoCommand"; };
    virtual const char * getMD5() override { return "209cbbdc214ffab527afe09827fa5db3"; };

  };

}
#endif
