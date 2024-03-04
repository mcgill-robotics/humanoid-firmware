#ifndef _ROS_servo_node_ServoFeedback_h
#define _ROS_servo_node_ServoFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace servo_node
{

  class ServoFeedback : public ros::Msg
  {
    public:
      float left_leg_ankle_fb[3];
      float left_leg_knee_fb[3];
      float left_leg_hip_roll_fb[3];
      float left_leg_hip_pitch_fb[3];
      float left_leg_hip_yaw_fb[3];
      float right_leg_ankle_fb[3];
      float right_leg_knee_fb[3];
      float right_leg_hip_roll_fb[3];
      float right_leg_hip_pitch_fb[3];
      float right_leg_hip_yaw_fb[3];
      float torso_roll_fb[3];
      float torso_yaw_fb[3];
      float left_arm_elbow_fb[3];
      float right_arm_elbow_fb[3];
      float left_arm_shoulder_fb[3];
      float right_arm_shoulder_fb[3];

    ServoFeedback():
      left_leg_ankle_fb(),
      left_leg_knee_fb(),
      left_leg_hip_roll_fb(),
      left_leg_hip_pitch_fb(),
      left_leg_hip_yaw_fb(),
      right_leg_ankle_fb(),
      right_leg_knee_fb(),
      right_leg_hip_roll_fb(),
      right_leg_hip_pitch_fb(),
      right_leg_hip_yaw_fb(),
      torso_roll_fb(),
      torso_yaw_fb(),
      left_arm_elbow_fb(),
      right_arm_elbow_fb(),
      left_arm_shoulder_fb(),
      right_arm_shoulder_fb()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_leg_ankle_fbi;
      u_left_leg_ankle_fbi.real = this->left_leg_ankle_fb[i];
      *(outbuffer + offset + 0) = (u_left_leg_ankle_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_leg_ankle_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_leg_ankle_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_leg_ankle_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_ankle_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_leg_knee_fbi;
      u_left_leg_knee_fbi.real = this->left_leg_knee_fb[i];
      *(outbuffer + offset + 0) = (u_left_leg_knee_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_leg_knee_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_leg_knee_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_leg_knee_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_knee_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_leg_hip_roll_fbi;
      u_left_leg_hip_roll_fbi.real = this->left_leg_hip_roll_fb[i];
      *(outbuffer + offset + 0) = (u_left_leg_hip_roll_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_leg_hip_roll_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_leg_hip_roll_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_leg_hip_roll_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_hip_roll_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_leg_hip_pitch_fbi;
      u_left_leg_hip_pitch_fbi.real = this->left_leg_hip_pitch_fb[i];
      *(outbuffer + offset + 0) = (u_left_leg_hip_pitch_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_leg_hip_pitch_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_leg_hip_pitch_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_leg_hip_pitch_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_hip_pitch_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_leg_hip_yaw_fbi;
      u_left_leg_hip_yaw_fbi.real = this->left_leg_hip_yaw_fb[i];
      *(outbuffer + offset + 0) = (u_left_leg_hip_yaw_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_leg_hip_yaw_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_leg_hip_yaw_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_leg_hip_yaw_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_hip_yaw_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_leg_ankle_fbi;
      u_right_leg_ankle_fbi.real = this->right_leg_ankle_fb[i];
      *(outbuffer + offset + 0) = (u_right_leg_ankle_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_leg_ankle_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_leg_ankle_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_leg_ankle_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_ankle_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_leg_knee_fbi;
      u_right_leg_knee_fbi.real = this->right_leg_knee_fb[i];
      *(outbuffer + offset + 0) = (u_right_leg_knee_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_leg_knee_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_leg_knee_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_leg_knee_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_knee_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_leg_hip_roll_fbi;
      u_right_leg_hip_roll_fbi.real = this->right_leg_hip_roll_fb[i];
      *(outbuffer + offset + 0) = (u_right_leg_hip_roll_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_leg_hip_roll_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_leg_hip_roll_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_leg_hip_roll_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_hip_roll_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_leg_hip_pitch_fbi;
      u_right_leg_hip_pitch_fbi.real = this->right_leg_hip_pitch_fb[i];
      *(outbuffer + offset + 0) = (u_right_leg_hip_pitch_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_leg_hip_pitch_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_leg_hip_pitch_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_leg_hip_pitch_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_hip_pitch_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_leg_hip_yaw_fbi;
      u_right_leg_hip_yaw_fbi.real = this->right_leg_hip_yaw_fb[i];
      *(outbuffer + offset + 0) = (u_right_leg_hip_yaw_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_leg_hip_yaw_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_leg_hip_yaw_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_leg_hip_yaw_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_hip_yaw_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_torso_roll_fbi;
      u_torso_roll_fbi.real = this->torso_roll_fb[i];
      *(outbuffer + offset + 0) = (u_torso_roll_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torso_roll_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torso_roll_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torso_roll_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torso_roll_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_torso_yaw_fbi;
      u_torso_yaw_fbi.real = this->torso_yaw_fb[i];
      *(outbuffer + offset + 0) = (u_torso_yaw_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torso_yaw_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torso_yaw_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torso_yaw_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torso_yaw_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_arm_elbow_fbi;
      u_left_arm_elbow_fbi.real = this->left_arm_elbow_fb[i];
      *(outbuffer + offset + 0) = (u_left_arm_elbow_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_arm_elbow_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_arm_elbow_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_arm_elbow_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_arm_elbow_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_arm_elbow_fbi;
      u_right_arm_elbow_fbi.real = this->right_arm_elbow_fb[i];
      *(outbuffer + offset + 0) = (u_right_arm_elbow_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_arm_elbow_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_arm_elbow_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_arm_elbow_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_arm_elbow_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_arm_shoulder_fbi;
      u_left_arm_shoulder_fbi.real = this->left_arm_shoulder_fb[i];
      *(outbuffer + offset + 0) = (u_left_arm_shoulder_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_arm_shoulder_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_arm_shoulder_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_arm_shoulder_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_arm_shoulder_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_arm_shoulder_fbi;
      u_right_arm_shoulder_fbi.real = this->right_arm_shoulder_fb[i];
      *(outbuffer + offset + 0) = (u_right_arm_shoulder_fbi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_arm_shoulder_fbi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_arm_shoulder_fbi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_arm_shoulder_fbi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_arm_shoulder_fb[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_leg_ankle_fbi;
      u_left_leg_ankle_fbi.base = 0;
      u_left_leg_ankle_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_leg_ankle_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_leg_ankle_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_leg_ankle_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_leg_ankle_fb[i] = u_left_leg_ankle_fbi.real;
      offset += sizeof(this->left_leg_ankle_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_leg_knee_fbi;
      u_left_leg_knee_fbi.base = 0;
      u_left_leg_knee_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_leg_knee_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_leg_knee_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_leg_knee_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_leg_knee_fb[i] = u_left_leg_knee_fbi.real;
      offset += sizeof(this->left_leg_knee_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_leg_hip_roll_fbi;
      u_left_leg_hip_roll_fbi.base = 0;
      u_left_leg_hip_roll_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_leg_hip_roll_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_leg_hip_roll_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_leg_hip_roll_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_leg_hip_roll_fb[i] = u_left_leg_hip_roll_fbi.real;
      offset += sizeof(this->left_leg_hip_roll_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_leg_hip_pitch_fbi;
      u_left_leg_hip_pitch_fbi.base = 0;
      u_left_leg_hip_pitch_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_leg_hip_pitch_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_leg_hip_pitch_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_leg_hip_pitch_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_leg_hip_pitch_fb[i] = u_left_leg_hip_pitch_fbi.real;
      offset += sizeof(this->left_leg_hip_pitch_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_leg_hip_yaw_fbi;
      u_left_leg_hip_yaw_fbi.base = 0;
      u_left_leg_hip_yaw_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_leg_hip_yaw_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_leg_hip_yaw_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_leg_hip_yaw_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_leg_hip_yaw_fb[i] = u_left_leg_hip_yaw_fbi.real;
      offset += sizeof(this->left_leg_hip_yaw_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_leg_ankle_fbi;
      u_right_leg_ankle_fbi.base = 0;
      u_right_leg_ankle_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_leg_ankle_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_leg_ankle_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_leg_ankle_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_leg_ankle_fb[i] = u_right_leg_ankle_fbi.real;
      offset += sizeof(this->right_leg_ankle_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_leg_knee_fbi;
      u_right_leg_knee_fbi.base = 0;
      u_right_leg_knee_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_leg_knee_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_leg_knee_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_leg_knee_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_leg_knee_fb[i] = u_right_leg_knee_fbi.real;
      offset += sizeof(this->right_leg_knee_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_leg_hip_roll_fbi;
      u_right_leg_hip_roll_fbi.base = 0;
      u_right_leg_hip_roll_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_leg_hip_roll_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_leg_hip_roll_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_leg_hip_roll_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_leg_hip_roll_fb[i] = u_right_leg_hip_roll_fbi.real;
      offset += sizeof(this->right_leg_hip_roll_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_leg_hip_pitch_fbi;
      u_right_leg_hip_pitch_fbi.base = 0;
      u_right_leg_hip_pitch_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_leg_hip_pitch_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_leg_hip_pitch_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_leg_hip_pitch_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_leg_hip_pitch_fb[i] = u_right_leg_hip_pitch_fbi.real;
      offset += sizeof(this->right_leg_hip_pitch_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_leg_hip_yaw_fbi;
      u_right_leg_hip_yaw_fbi.base = 0;
      u_right_leg_hip_yaw_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_leg_hip_yaw_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_leg_hip_yaw_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_leg_hip_yaw_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_leg_hip_yaw_fb[i] = u_right_leg_hip_yaw_fbi.real;
      offset += sizeof(this->right_leg_hip_yaw_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_torso_roll_fbi;
      u_torso_roll_fbi.base = 0;
      u_torso_roll_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_torso_roll_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_torso_roll_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_torso_roll_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->torso_roll_fb[i] = u_torso_roll_fbi.real;
      offset += sizeof(this->torso_roll_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_torso_yaw_fbi;
      u_torso_yaw_fbi.base = 0;
      u_torso_yaw_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_torso_yaw_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_torso_yaw_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_torso_yaw_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->torso_yaw_fb[i] = u_torso_yaw_fbi.real;
      offset += sizeof(this->torso_yaw_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_arm_elbow_fbi;
      u_left_arm_elbow_fbi.base = 0;
      u_left_arm_elbow_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_arm_elbow_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_arm_elbow_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_arm_elbow_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_arm_elbow_fb[i] = u_left_arm_elbow_fbi.real;
      offset += sizeof(this->left_arm_elbow_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_arm_elbow_fbi;
      u_right_arm_elbow_fbi.base = 0;
      u_right_arm_elbow_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_arm_elbow_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_arm_elbow_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_arm_elbow_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_arm_elbow_fb[i] = u_right_arm_elbow_fbi.real;
      offset += sizeof(this->right_arm_elbow_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_left_arm_shoulder_fbi;
      u_left_arm_shoulder_fbi.base = 0;
      u_left_arm_shoulder_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_arm_shoulder_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_arm_shoulder_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_arm_shoulder_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_arm_shoulder_fb[i] = u_left_arm_shoulder_fbi.real;
      offset += sizeof(this->left_arm_shoulder_fb[i]);
      }
      for( uint32_t i = 0; i < 3; i++){
      union {
        float real;
        uint32_t base;
      } u_right_arm_shoulder_fbi;
      u_right_arm_shoulder_fbi.base = 0;
      u_right_arm_shoulder_fbi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_arm_shoulder_fbi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_arm_shoulder_fbi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_arm_shoulder_fbi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_arm_shoulder_fb[i] = u_right_arm_shoulder_fbi.real;
      offset += sizeof(this->right_arm_shoulder_fb[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "servo_node/ServoFeedback"; };
    virtual const char * getMD5() override { return "7395634712f219d3baa14ef32ef88cfd"; };

  };

}
#endif
