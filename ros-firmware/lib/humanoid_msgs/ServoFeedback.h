#ifndef _ROS_humanoid_msgs_ServoFeedback_h
#define _ROS_humanoid_msgs_ServoFeedback_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace humanoid_msgs
{

  class ServoFeedback : public ros::Msg
  {
    public:
      uint32_t left_leg_ankle_fb_length;
      typedef float _left_leg_ankle_fb_type;
      _left_leg_ankle_fb_type st_left_leg_ankle_fb;
      _left_leg_ankle_fb_type * left_leg_ankle_fb;
      uint32_t left_leg_knee_fb_length;
      typedef float _left_leg_knee_fb_type;
      _left_leg_knee_fb_type st_left_leg_knee_fb;
      _left_leg_knee_fb_type * left_leg_knee_fb;
      uint32_t left_leg_hip_roll_fb_length;
      typedef float _left_leg_hip_roll_fb_type;
      _left_leg_hip_roll_fb_type st_left_leg_hip_roll_fb;
      _left_leg_hip_roll_fb_type * left_leg_hip_roll_fb;
      uint32_t left_leg_hip_pitch_fb_length;
      typedef float _left_leg_hip_pitch_fb_type;
      _left_leg_hip_pitch_fb_type st_left_leg_hip_pitch_fb;
      _left_leg_hip_pitch_fb_type * left_leg_hip_pitch_fb;
      uint32_t left_leg_hip_yaw_fb_length;
      typedef float _left_leg_hip_yaw_fb_type;
      _left_leg_hip_yaw_fb_type st_left_leg_hip_yaw_fb;
      _left_leg_hip_yaw_fb_type * left_leg_hip_yaw_fb;
      uint32_t right_leg_ankle_fb_length;
      typedef float _right_leg_ankle_fb_type;
      _right_leg_ankle_fb_type st_right_leg_ankle_fb;
      _right_leg_ankle_fb_type * right_leg_ankle_fb;
      uint32_t right_leg_knee_fb_length;
      typedef float _right_leg_knee_fb_type;
      _right_leg_knee_fb_type st_right_leg_knee_fb;
      _right_leg_knee_fb_type * right_leg_knee_fb;
      uint32_t right_leg_hip_roll_fb_length;
      typedef float _right_leg_hip_roll_fb_type;
      _right_leg_hip_roll_fb_type st_right_leg_hip_roll_fb;
      _right_leg_hip_roll_fb_type * right_leg_hip_roll_fb;
      uint32_t right_leg_hip_pitch_fb_length;
      typedef float _right_leg_hip_pitch_fb_type;
      _right_leg_hip_pitch_fb_type st_right_leg_hip_pitch_fb;
      _right_leg_hip_pitch_fb_type * right_leg_hip_pitch_fb;
      uint32_t right_leg_hip_yaw_fb_length;
      typedef float _right_leg_hip_yaw_fb_type;
      _right_leg_hip_yaw_fb_type st_right_leg_hip_yaw_fb;
      _right_leg_hip_yaw_fb_type * right_leg_hip_yaw_fb;
      uint32_t torso_roll_fb_length;
      typedef float _torso_roll_fb_type;
      _torso_roll_fb_type st_torso_roll_fb;
      _torso_roll_fb_type * torso_roll_fb;
      uint32_t torso_yaw_fb_length;
      typedef float _torso_yaw_fb_type;
      _torso_yaw_fb_type st_torso_yaw_fb;
      _torso_yaw_fb_type * torso_yaw_fb;
      uint32_t left_arm_elbow_fb_length;
      typedef float _left_arm_elbow_fb_type;
      _left_arm_elbow_fb_type st_left_arm_elbow_fb;
      _left_arm_elbow_fb_type * left_arm_elbow_fb;
      uint32_t right_arm_elbow_fb_length;
      typedef float _right_arm_elbow_fb_type;
      _right_arm_elbow_fb_type st_right_arm_elbow_fb;
      _right_arm_elbow_fb_type * right_arm_elbow_fb;
      uint32_t left_arm_shoulder_fb_length;
      typedef float _left_arm_shoulder_fb_type;
      _left_arm_shoulder_fb_type st_left_arm_shoulder_fb;
      _left_arm_shoulder_fb_type * left_arm_shoulder_fb;
      uint32_t right_arm_shoulder_fb_length;
      typedef float _right_arm_shoulder_fb_type;
      _right_arm_shoulder_fb_type st_right_arm_shoulder_fb;
      _right_arm_shoulder_fb_type * right_arm_shoulder_fb;

    ServoFeedback():
      left_leg_ankle_fb_length(0), st_left_leg_ankle_fb(), left_leg_ankle_fb(nullptr),
      left_leg_knee_fb_length(0), st_left_leg_knee_fb(), left_leg_knee_fb(nullptr),
      left_leg_hip_roll_fb_length(0), st_left_leg_hip_roll_fb(), left_leg_hip_roll_fb(nullptr),
      left_leg_hip_pitch_fb_length(0), st_left_leg_hip_pitch_fb(), left_leg_hip_pitch_fb(nullptr),
      left_leg_hip_yaw_fb_length(0), st_left_leg_hip_yaw_fb(), left_leg_hip_yaw_fb(nullptr),
      right_leg_ankle_fb_length(0), st_right_leg_ankle_fb(), right_leg_ankle_fb(nullptr),
      right_leg_knee_fb_length(0), st_right_leg_knee_fb(), right_leg_knee_fb(nullptr),
      right_leg_hip_roll_fb_length(0), st_right_leg_hip_roll_fb(), right_leg_hip_roll_fb(nullptr),
      right_leg_hip_pitch_fb_length(0), st_right_leg_hip_pitch_fb(), right_leg_hip_pitch_fb(nullptr),
      right_leg_hip_yaw_fb_length(0), st_right_leg_hip_yaw_fb(), right_leg_hip_yaw_fb(nullptr),
      torso_roll_fb_length(0), st_torso_roll_fb(), torso_roll_fb(nullptr),
      torso_yaw_fb_length(0), st_torso_yaw_fb(), torso_yaw_fb(nullptr),
      left_arm_elbow_fb_length(0), st_left_arm_elbow_fb(), left_arm_elbow_fb(nullptr),
      right_arm_elbow_fb_length(0), st_right_arm_elbow_fb(), right_arm_elbow_fb(nullptr),
      left_arm_shoulder_fb_length(0), st_left_arm_shoulder_fb(), left_arm_shoulder_fb(nullptr),
      right_arm_shoulder_fb_length(0), st_right_arm_shoulder_fb(), right_arm_shoulder_fb(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->left_leg_ankle_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_leg_ankle_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_leg_ankle_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_leg_ankle_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_ankle_fb_length);
      for( uint32_t i = 0; i < left_leg_ankle_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->left_leg_knee_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_leg_knee_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_leg_knee_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_leg_knee_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_knee_fb_length);
      for( uint32_t i = 0; i < left_leg_knee_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->left_leg_hip_roll_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_leg_hip_roll_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_leg_hip_roll_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_leg_hip_roll_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_hip_roll_fb_length);
      for( uint32_t i = 0; i < left_leg_hip_roll_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->left_leg_hip_pitch_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_leg_hip_pitch_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_leg_hip_pitch_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_leg_hip_pitch_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_hip_pitch_fb_length);
      for( uint32_t i = 0; i < left_leg_hip_pitch_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->left_leg_hip_yaw_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_leg_hip_yaw_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_leg_hip_yaw_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_leg_hip_yaw_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_leg_hip_yaw_fb_length);
      for( uint32_t i = 0; i < left_leg_hip_yaw_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->right_leg_ankle_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_leg_ankle_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_leg_ankle_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_leg_ankle_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_ankle_fb_length);
      for( uint32_t i = 0; i < right_leg_ankle_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->right_leg_knee_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_leg_knee_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_leg_knee_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_leg_knee_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_knee_fb_length);
      for( uint32_t i = 0; i < right_leg_knee_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->right_leg_hip_roll_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_leg_hip_roll_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_leg_hip_roll_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_leg_hip_roll_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_hip_roll_fb_length);
      for( uint32_t i = 0; i < right_leg_hip_roll_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->right_leg_hip_pitch_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_leg_hip_pitch_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_leg_hip_pitch_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_leg_hip_pitch_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_hip_pitch_fb_length);
      for( uint32_t i = 0; i < right_leg_hip_pitch_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->right_leg_hip_yaw_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_leg_hip_yaw_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_leg_hip_yaw_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_leg_hip_yaw_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_leg_hip_yaw_fb_length);
      for( uint32_t i = 0; i < right_leg_hip_yaw_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->torso_roll_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->torso_roll_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->torso_roll_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->torso_roll_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torso_roll_fb_length);
      for( uint32_t i = 0; i < torso_roll_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->torso_yaw_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->torso_yaw_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->torso_yaw_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->torso_yaw_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torso_yaw_fb_length);
      for( uint32_t i = 0; i < torso_yaw_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->left_arm_elbow_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_arm_elbow_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_arm_elbow_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_arm_elbow_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_arm_elbow_fb_length);
      for( uint32_t i = 0; i < left_arm_elbow_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->right_arm_elbow_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_arm_elbow_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_arm_elbow_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_arm_elbow_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_arm_elbow_fb_length);
      for( uint32_t i = 0; i < right_arm_elbow_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->left_arm_shoulder_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_arm_shoulder_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_arm_shoulder_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_arm_shoulder_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_arm_shoulder_fb_length);
      for( uint32_t i = 0; i < left_arm_shoulder_fb_length; i++){
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
      *(outbuffer + offset + 0) = (this->right_arm_shoulder_fb_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_arm_shoulder_fb_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_arm_shoulder_fb_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_arm_shoulder_fb_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_arm_shoulder_fb_length);
      for( uint32_t i = 0; i < right_arm_shoulder_fb_length; i++){
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
      uint32_t left_leg_ankle_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      left_leg_ankle_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      left_leg_ankle_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      left_leg_ankle_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->left_leg_ankle_fb_length);
      if(left_leg_ankle_fb_lengthT > left_leg_ankle_fb_length)
        this->left_leg_ankle_fb = (float*)realloc(this->left_leg_ankle_fb, left_leg_ankle_fb_lengthT * sizeof(float));
      left_leg_ankle_fb_length = left_leg_ankle_fb_lengthT;
      for( uint32_t i = 0; i < left_leg_ankle_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_left_leg_ankle_fb;
      u_st_left_leg_ankle_fb.base = 0;
      u_st_left_leg_ankle_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_left_leg_ankle_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_left_leg_ankle_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_left_leg_ankle_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_left_leg_ankle_fb = u_st_left_leg_ankle_fb.real;
      offset += sizeof(this->st_left_leg_ankle_fb);
        memcpy( &(this->left_leg_ankle_fb[i]), &(this->st_left_leg_ankle_fb), sizeof(float));
      }
      uint32_t left_leg_knee_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      left_leg_knee_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      left_leg_knee_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      left_leg_knee_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->left_leg_knee_fb_length);
      if(left_leg_knee_fb_lengthT > left_leg_knee_fb_length)
        this->left_leg_knee_fb = (float*)realloc(this->left_leg_knee_fb, left_leg_knee_fb_lengthT * sizeof(float));
      left_leg_knee_fb_length = left_leg_knee_fb_lengthT;
      for( uint32_t i = 0; i < left_leg_knee_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_left_leg_knee_fb;
      u_st_left_leg_knee_fb.base = 0;
      u_st_left_leg_knee_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_left_leg_knee_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_left_leg_knee_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_left_leg_knee_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_left_leg_knee_fb = u_st_left_leg_knee_fb.real;
      offset += sizeof(this->st_left_leg_knee_fb);
        memcpy( &(this->left_leg_knee_fb[i]), &(this->st_left_leg_knee_fb), sizeof(float));
      }
      uint32_t left_leg_hip_roll_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      left_leg_hip_roll_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      left_leg_hip_roll_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      left_leg_hip_roll_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->left_leg_hip_roll_fb_length);
      if(left_leg_hip_roll_fb_lengthT > left_leg_hip_roll_fb_length)
        this->left_leg_hip_roll_fb = (float*)realloc(this->left_leg_hip_roll_fb, left_leg_hip_roll_fb_lengthT * sizeof(float));
      left_leg_hip_roll_fb_length = left_leg_hip_roll_fb_lengthT;
      for( uint32_t i = 0; i < left_leg_hip_roll_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_left_leg_hip_roll_fb;
      u_st_left_leg_hip_roll_fb.base = 0;
      u_st_left_leg_hip_roll_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_left_leg_hip_roll_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_left_leg_hip_roll_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_left_leg_hip_roll_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_left_leg_hip_roll_fb = u_st_left_leg_hip_roll_fb.real;
      offset += sizeof(this->st_left_leg_hip_roll_fb);
        memcpy( &(this->left_leg_hip_roll_fb[i]), &(this->st_left_leg_hip_roll_fb), sizeof(float));
      }
      uint32_t left_leg_hip_pitch_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      left_leg_hip_pitch_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      left_leg_hip_pitch_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      left_leg_hip_pitch_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->left_leg_hip_pitch_fb_length);
      if(left_leg_hip_pitch_fb_lengthT > left_leg_hip_pitch_fb_length)
        this->left_leg_hip_pitch_fb = (float*)realloc(this->left_leg_hip_pitch_fb, left_leg_hip_pitch_fb_lengthT * sizeof(float));
      left_leg_hip_pitch_fb_length = left_leg_hip_pitch_fb_lengthT;
      for( uint32_t i = 0; i < left_leg_hip_pitch_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_left_leg_hip_pitch_fb;
      u_st_left_leg_hip_pitch_fb.base = 0;
      u_st_left_leg_hip_pitch_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_left_leg_hip_pitch_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_left_leg_hip_pitch_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_left_leg_hip_pitch_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_left_leg_hip_pitch_fb = u_st_left_leg_hip_pitch_fb.real;
      offset += sizeof(this->st_left_leg_hip_pitch_fb);
        memcpy( &(this->left_leg_hip_pitch_fb[i]), &(this->st_left_leg_hip_pitch_fb), sizeof(float));
      }
      uint32_t left_leg_hip_yaw_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      left_leg_hip_yaw_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      left_leg_hip_yaw_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      left_leg_hip_yaw_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->left_leg_hip_yaw_fb_length);
      if(left_leg_hip_yaw_fb_lengthT > left_leg_hip_yaw_fb_length)
        this->left_leg_hip_yaw_fb = (float*)realloc(this->left_leg_hip_yaw_fb, left_leg_hip_yaw_fb_lengthT * sizeof(float));
      left_leg_hip_yaw_fb_length = left_leg_hip_yaw_fb_lengthT;
      for( uint32_t i = 0; i < left_leg_hip_yaw_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_left_leg_hip_yaw_fb;
      u_st_left_leg_hip_yaw_fb.base = 0;
      u_st_left_leg_hip_yaw_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_left_leg_hip_yaw_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_left_leg_hip_yaw_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_left_leg_hip_yaw_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_left_leg_hip_yaw_fb = u_st_left_leg_hip_yaw_fb.real;
      offset += sizeof(this->st_left_leg_hip_yaw_fb);
        memcpy( &(this->left_leg_hip_yaw_fb[i]), &(this->st_left_leg_hip_yaw_fb), sizeof(float));
      }
      uint32_t right_leg_ankle_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      right_leg_ankle_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      right_leg_ankle_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      right_leg_ankle_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->right_leg_ankle_fb_length);
      if(right_leg_ankle_fb_lengthT > right_leg_ankle_fb_length)
        this->right_leg_ankle_fb = (float*)realloc(this->right_leg_ankle_fb, right_leg_ankle_fb_lengthT * sizeof(float));
      right_leg_ankle_fb_length = right_leg_ankle_fb_lengthT;
      for( uint32_t i = 0; i < right_leg_ankle_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_right_leg_ankle_fb;
      u_st_right_leg_ankle_fb.base = 0;
      u_st_right_leg_ankle_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_right_leg_ankle_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_right_leg_ankle_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_right_leg_ankle_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_right_leg_ankle_fb = u_st_right_leg_ankle_fb.real;
      offset += sizeof(this->st_right_leg_ankle_fb);
        memcpy( &(this->right_leg_ankle_fb[i]), &(this->st_right_leg_ankle_fb), sizeof(float));
      }
      uint32_t right_leg_knee_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      right_leg_knee_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      right_leg_knee_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      right_leg_knee_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->right_leg_knee_fb_length);
      if(right_leg_knee_fb_lengthT > right_leg_knee_fb_length)
        this->right_leg_knee_fb = (float*)realloc(this->right_leg_knee_fb, right_leg_knee_fb_lengthT * sizeof(float));
      right_leg_knee_fb_length = right_leg_knee_fb_lengthT;
      for( uint32_t i = 0; i < right_leg_knee_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_right_leg_knee_fb;
      u_st_right_leg_knee_fb.base = 0;
      u_st_right_leg_knee_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_right_leg_knee_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_right_leg_knee_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_right_leg_knee_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_right_leg_knee_fb = u_st_right_leg_knee_fb.real;
      offset += sizeof(this->st_right_leg_knee_fb);
        memcpy( &(this->right_leg_knee_fb[i]), &(this->st_right_leg_knee_fb), sizeof(float));
      }
      uint32_t right_leg_hip_roll_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      right_leg_hip_roll_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      right_leg_hip_roll_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      right_leg_hip_roll_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->right_leg_hip_roll_fb_length);
      if(right_leg_hip_roll_fb_lengthT > right_leg_hip_roll_fb_length)
        this->right_leg_hip_roll_fb = (float*)realloc(this->right_leg_hip_roll_fb, right_leg_hip_roll_fb_lengthT * sizeof(float));
      right_leg_hip_roll_fb_length = right_leg_hip_roll_fb_lengthT;
      for( uint32_t i = 0; i < right_leg_hip_roll_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_right_leg_hip_roll_fb;
      u_st_right_leg_hip_roll_fb.base = 0;
      u_st_right_leg_hip_roll_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_right_leg_hip_roll_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_right_leg_hip_roll_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_right_leg_hip_roll_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_right_leg_hip_roll_fb = u_st_right_leg_hip_roll_fb.real;
      offset += sizeof(this->st_right_leg_hip_roll_fb);
        memcpy( &(this->right_leg_hip_roll_fb[i]), &(this->st_right_leg_hip_roll_fb), sizeof(float));
      }
      uint32_t right_leg_hip_pitch_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      right_leg_hip_pitch_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      right_leg_hip_pitch_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      right_leg_hip_pitch_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->right_leg_hip_pitch_fb_length);
      if(right_leg_hip_pitch_fb_lengthT > right_leg_hip_pitch_fb_length)
        this->right_leg_hip_pitch_fb = (float*)realloc(this->right_leg_hip_pitch_fb, right_leg_hip_pitch_fb_lengthT * sizeof(float));
      right_leg_hip_pitch_fb_length = right_leg_hip_pitch_fb_lengthT;
      for( uint32_t i = 0; i < right_leg_hip_pitch_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_right_leg_hip_pitch_fb;
      u_st_right_leg_hip_pitch_fb.base = 0;
      u_st_right_leg_hip_pitch_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_right_leg_hip_pitch_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_right_leg_hip_pitch_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_right_leg_hip_pitch_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_right_leg_hip_pitch_fb = u_st_right_leg_hip_pitch_fb.real;
      offset += sizeof(this->st_right_leg_hip_pitch_fb);
        memcpy( &(this->right_leg_hip_pitch_fb[i]), &(this->st_right_leg_hip_pitch_fb), sizeof(float));
      }
      uint32_t right_leg_hip_yaw_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      right_leg_hip_yaw_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      right_leg_hip_yaw_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      right_leg_hip_yaw_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->right_leg_hip_yaw_fb_length);
      if(right_leg_hip_yaw_fb_lengthT > right_leg_hip_yaw_fb_length)
        this->right_leg_hip_yaw_fb = (float*)realloc(this->right_leg_hip_yaw_fb, right_leg_hip_yaw_fb_lengthT * sizeof(float));
      right_leg_hip_yaw_fb_length = right_leg_hip_yaw_fb_lengthT;
      for( uint32_t i = 0; i < right_leg_hip_yaw_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_right_leg_hip_yaw_fb;
      u_st_right_leg_hip_yaw_fb.base = 0;
      u_st_right_leg_hip_yaw_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_right_leg_hip_yaw_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_right_leg_hip_yaw_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_right_leg_hip_yaw_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_right_leg_hip_yaw_fb = u_st_right_leg_hip_yaw_fb.real;
      offset += sizeof(this->st_right_leg_hip_yaw_fb);
        memcpy( &(this->right_leg_hip_yaw_fb[i]), &(this->st_right_leg_hip_yaw_fb), sizeof(float));
      }
      uint32_t torso_roll_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      torso_roll_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      torso_roll_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      torso_roll_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->torso_roll_fb_length);
      if(torso_roll_fb_lengthT > torso_roll_fb_length)
        this->torso_roll_fb = (float*)realloc(this->torso_roll_fb, torso_roll_fb_lengthT * sizeof(float));
      torso_roll_fb_length = torso_roll_fb_lengthT;
      for( uint32_t i = 0; i < torso_roll_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_torso_roll_fb;
      u_st_torso_roll_fb.base = 0;
      u_st_torso_roll_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_torso_roll_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_torso_roll_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_torso_roll_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_torso_roll_fb = u_st_torso_roll_fb.real;
      offset += sizeof(this->st_torso_roll_fb);
        memcpy( &(this->torso_roll_fb[i]), &(this->st_torso_roll_fb), sizeof(float));
      }
      uint32_t torso_yaw_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      torso_yaw_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      torso_yaw_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      torso_yaw_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->torso_yaw_fb_length);
      if(torso_yaw_fb_lengthT > torso_yaw_fb_length)
        this->torso_yaw_fb = (float*)realloc(this->torso_yaw_fb, torso_yaw_fb_lengthT * sizeof(float));
      torso_yaw_fb_length = torso_yaw_fb_lengthT;
      for( uint32_t i = 0; i < torso_yaw_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_torso_yaw_fb;
      u_st_torso_yaw_fb.base = 0;
      u_st_torso_yaw_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_torso_yaw_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_torso_yaw_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_torso_yaw_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_torso_yaw_fb = u_st_torso_yaw_fb.real;
      offset += sizeof(this->st_torso_yaw_fb);
        memcpy( &(this->torso_yaw_fb[i]), &(this->st_torso_yaw_fb), sizeof(float));
      }
      uint32_t left_arm_elbow_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      left_arm_elbow_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      left_arm_elbow_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      left_arm_elbow_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->left_arm_elbow_fb_length);
      if(left_arm_elbow_fb_lengthT > left_arm_elbow_fb_length)
        this->left_arm_elbow_fb = (float*)realloc(this->left_arm_elbow_fb, left_arm_elbow_fb_lengthT * sizeof(float));
      left_arm_elbow_fb_length = left_arm_elbow_fb_lengthT;
      for( uint32_t i = 0; i < left_arm_elbow_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_left_arm_elbow_fb;
      u_st_left_arm_elbow_fb.base = 0;
      u_st_left_arm_elbow_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_left_arm_elbow_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_left_arm_elbow_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_left_arm_elbow_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_left_arm_elbow_fb = u_st_left_arm_elbow_fb.real;
      offset += sizeof(this->st_left_arm_elbow_fb);
        memcpy( &(this->left_arm_elbow_fb[i]), &(this->st_left_arm_elbow_fb), sizeof(float));
      }
      uint32_t right_arm_elbow_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      right_arm_elbow_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      right_arm_elbow_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      right_arm_elbow_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->right_arm_elbow_fb_length);
      if(right_arm_elbow_fb_lengthT > right_arm_elbow_fb_length)
        this->right_arm_elbow_fb = (float*)realloc(this->right_arm_elbow_fb, right_arm_elbow_fb_lengthT * sizeof(float));
      right_arm_elbow_fb_length = right_arm_elbow_fb_lengthT;
      for( uint32_t i = 0; i < right_arm_elbow_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_right_arm_elbow_fb;
      u_st_right_arm_elbow_fb.base = 0;
      u_st_right_arm_elbow_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_right_arm_elbow_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_right_arm_elbow_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_right_arm_elbow_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_right_arm_elbow_fb = u_st_right_arm_elbow_fb.real;
      offset += sizeof(this->st_right_arm_elbow_fb);
        memcpy( &(this->right_arm_elbow_fb[i]), &(this->st_right_arm_elbow_fb), sizeof(float));
      }
      uint32_t left_arm_shoulder_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      left_arm_shoulder_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      left_arm_shoulder_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      left_arm_shoulder_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->left_arm_shoulder_fb_length);
      if(left_arm_shoulder_fb_lengthT > left_arm_shoulder_fb_length)
        this->left_arm_shoulder_fb = (float*)realloc(this->left_arm_shoulder_fb, left_arm_shoulder_fb_lengthT * sizeof(float));
      left_arm_shoulder_fb_length = left_arm_shoulder_fb_lengthT;
      for( uint32_t i = 0; i < left_arm_shoulder_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_left_arm_shoulder_fb;
      u_st_left_arm_shoulder_fb.base = 0;
      u_st_left_arm_shoulder_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_left_arm_shoulder_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_left_arm_shoulder_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_left_arm_shoulder_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_left_arm_shoulder_fb = u_st_left_arm_shoulder_fb.real;
      offset += sizeof(this->st_left_arm_shoulder_fb);
        memcpy( &(this->left_arm_shoulder_fb[i]), &(this->st_left_arm_shoulder_fb), sizeof(float));
      }
      uint32_t right_arm_shoulder_fb_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      right_arm_shoulder_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      right_arm_shoulder_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      right_arm_shoulder_fb_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->right_arm_shoulder_fb_length);
      if(right_arm_shoulder_fb_lengthT > right_arm_shoulder_fb_length)
        this->right_arm_shoulder_fb = (float*)realloc(this->right_arm_shoulder_fb, right_arm_shoulder_fb_lengthT * sizeof(float));
      right_arm_shoulder_fb_length = right_arm_shoulder_fb_lengthT;
      for( uint32_t i = 0; i < right_arm_shoulder_fb_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_right_arm_shoulder_fb;
      u_st_right_arm_shoulder_fb.base = 0;
      u_st_right_arm_shoulder_fb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_right_arm_shoulder_fb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_right_arm_shoulder_fb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_right_arm_shoulder_fb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_right_arm_shoulder_fb = u_st_right_arm_shoulder_fb.real;
      offset += sizeof(this->st_right_arm_shoulder_fb);
        memcpy( &(this->right_arm_shoulder_fb[i]), &(this->st_right_arm_shoulder_fb), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "humanoid_msgs/ServoFeedback"; };
    virtual const char * getMD5() override { return "288e47e019902deafae28fa169ff72ea"; };

  };

}
#endif
