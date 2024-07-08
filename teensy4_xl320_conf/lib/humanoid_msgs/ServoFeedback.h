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
      uint32_t right_shoulder_pitch_length;
      typedef float _right_shoulder_pitch_type;
      _right_shoulder_pitch_type st_right_shoulder_pitch;
      _right_shoulder_pitch_type * right_shoulder_pitch;
      uint32_t right_shoulder_roll_length;
      typedef float _right_shoulder_roll_type;
      _right_shoulder_roll_type st_right_shoulder_roll;
      _right_shoulder_roll_type * right_shoulder_roll;
      uint32_t right_elbow_length;
      typedef float _right_elbow_type;
      _right_elbow_type st_right_elbow;
      _right_elbow_type * right_elbow;
      uint32_t left_shoulder_pitch_length;
      typedef float _left_shoulder_pitch_type;
      _left_shoulder_pitch_type st_left_shoulder_pitch;
      _left_shoulder_pitch_type * left_shoulder_pitch;
      uint32_t left_shoulder_roll_length;
      typedef float _left_shoulder_roll_type;
      _left_shoulder_roll_type st_left_shoulder_roll;
      _left_shoulder_roll_type * left_shoulder_roll;
      uint32_t left_elbow_length;
      typedef float _left_elbow_type;
      _left_elbow_type st_left_elbow;
      _left_elbow_type * left_elbow;
      uint32_t left_hip_roll_length;
      typedef float _left_hip_roll_type;
      _left_hip_roll_type st_left_hip_roll;
      _left_hip_roll_type * left_hip_roll;
      uint32_t left_hip_pitch_length;
      typedef float _left_hip_pitch_type;
      _left_hip_pitch_type st_left_hip_pitch;
      _left_hip_pitch_type * left_hip_pitch;
      uint32_t left_knee_length;
      typedef float _left_knee_type;
      _left_knee_type st_left_knee;
      _left_knee_type * left_knee;
      uint32_t right_hip_roll_length;
      typedef float _right_hip_roll_type;
      _right_hip_roll_type st_right_hip_roll;
      _right_hip_roll_type * right_hip_roll;
      uint32_t right_hip_pitch_length;
      typedef float _right_hip_pitch_type;
      _right_hip_pitch_type st_right_hip_pitch;
      _right_hip_pitch_type * right_hip_pitch;
      uint32_t right_knee_length;
      typedef float _right_knee_type;
      _right_knee_type st_right_knee;
      _right_knee_type * right_knee;

    ServoFeedback():
      right_shoulder_pitch_length(0), st_right_shoulder_pitch(), right_shoulder_pitch(nullptr),
      right_shoulder_roll_length(0), st_right_shoulder_roll(), right_shoulder_roll(nullptr),
      right_elbow_length(0), st_right_elbow(), right_elbow(nullptr),
      left_shoulder_pitch_length(0), st_left_shoulder_pitch(), left_shoulder_pitch(nullptr),
      left_shoulder_roll_length(0), st_left_shoulder_roll(), left_shoulder_roll(nullptr),
      left_elbow_length(0), st_left_elbow(), left_elbow(nullptr),
      left_hip_roll_length(0), st_left_hip_roll(), left_hip_roll(nullptr),
      left_hip_pitch_length(0), st_left_hip_pitch(), left_hip_pitch(nullptr),
      left_knee_length(0), st_left_knee(), left_knee(nullptr),
      right_hip_roll_length(0), st_right_hip_roll(), right_hip_roll(nullptr),
      right_hip_pitch_length(0), st_right_hip_pitch(), right_hip_pitch(nullptr),
      right_knee_length(0), st_right_knee(), right_knee(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->right_shoulder_pitch_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_shoulder_pitch_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_shoulder_pitch_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_shoulder_pitch_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_shoulder_pitch_length);
      for( uint32_t i = 0; i < right_shoulder_pitch_length; i++){
      union {
        float real;
        uint32_t base;
      } u_right_shoulder_pitchi;
      u_right_shoulder_pitchi.real = this->right_shoulder_pitch[i];
      *(outbuffer + offset + 0) = (u_right_shoulder_pitchi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_shoulder_pitchi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_shoulder_pitchi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_shoulder_pitchi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_shoulder_pitch[i]);
      }
      *(outbuffer + offset + 0) = (this->right_shoulder_roll_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_shoulder_roll_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_shoulder_roll_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_shoulder_roll_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_shoulder_roll_length);
      for( uint32_t i = 0; i < right_shoulder_roll_length; i++){
      union {
        float real;
        uint32_t base;
      } u_right_shoulder_rolli;
      u_right_shoulder_rolli.real = this->right_shoulder_roll[i];
      *(outbuffer + offset + 0) = (u_right_shoulder_rolli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_shoulder_rolli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_shoulder_rolli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_shoulder_rolli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_shoulder_roll[i]);
      }
      *(outbuffer + offset + 0) = (this->right_elbow_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_elbow_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_elbow_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_elbow_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_elbow_length);
      for( uint32_t i = 0; i < right_elbow_length; i++){
      union {
        float real;
        uint32_t base;
      } u_right_elbowi;
      u_right_elbowi.real = this->right_elbow[i];
      *(outbuffer + offset + 0) = (u_right_elbowi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_elbowi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_elbowi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_elbowi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_elbow[i]);
      }
      *(outbuffer + offset + 0) = (this->left_shoulder_pitch_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_shoulder_pitch_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_shoulder_pitch_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_shoulder_pitch_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_shoulder_pitch_length);
      for( uint32_t i = 0; i < left_shoulder_pitch_length; i++){
      union {
        float real;
        uint32_t base;
      } u_left_shoulder_pitchi;
      u_left_shoulder_pitchi.real = this->left_shoulder_pitch[i];
      *(outbuffer + offset + 0) = (u_left_shoulder_pitchi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_shoulder_pitchi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_shoulder_pitchi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_shoulder_pitchi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_shoulder_pitch[i]);
      }
      *(outbuffer + offset + 0) = (this->left_shoulder_roll_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_shoulder_roll_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_shoulder_roll_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_shoulder_roll_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_shoulder_roll_length);
      for( uint32_t i = 0; i < left_shoulder_roll_length; i++){
      union {
        float real;
        uint32_t base;
      } u_left_shoulder_rolli;
      u_left_shoulder_rolli.real = this->left_shoulder_roll[i];
      *(outbuffer + offset + 0) = (u_left_shoulder_rolli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_shoulder_rolli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_shoulder_rolli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_shoulder_rolli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_shoulder_roll[i]);
      }
      *(outbuffer + offset + 0) = (this->left_elbow_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_elbow_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_elbow_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_elbow_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_elbow_length);
      for( uint32_t i = 0; i < left_elbow_length; i++){
      union {
        float real;
        uint32_t base;
      } u_left_elbowi;
      u_left_elbowi.real = this->left_elbow[i];
      *(outbuffer + offset + 0) = (u_left_elbowi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_elbowi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_elbowi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_elbowi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_elbow[i]);
      }
      *(outbuffer + offset + 0) = (this->left_hip_roll_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_hip_roll_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_hip_roll_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_hip_roll_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_hip_roll_length);
      for( uint32_t i = 0; i < left_hip_roll_length; i++){
      union {
        float real;
        uint32_t base;
      } u_left_hip_rolli;
      u_left_hip_rolli.real = this->left_hip_roll[i];
      *(outbuffer + offset + 0) = (u_left_hip_rolli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_hip_rolli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_hip_rolli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_hip_rolli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_hip_roll[i]);
      }
      *(outbuffer + offset + 0) = (this->left_hip_pitch_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_hip_pitch_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_hip_pitch_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_hip_pitch_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_hip_pitch_length);
      for( uint32_t i = 0; i < left_hip_pitch_length; i++){
      union {
        float real;
        uint32_t base;
      } u_left_hip_pitchi;
      u_left_hip_pitchi.real = this->left_hip_pitch[i];
      *(outbuffer + offset + 0) = (u_left_hip_pitchi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_hip_pitchi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_hip_pitchi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_hip_pitchi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_hip_pitch[i]);
      }
      *(outbuffer + offset + 0) = (this->left_knee_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_knee_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_knee_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_knee_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_knee_length);
      for( uint32_t i = 0; i < left_knee_length; i++){
      union {
        float real;
        uint32_t base;
      } u_left_kneei;
      u_left_kneei.real = this->left_knee[i];
      *(outbuffer + offset + 0) = (u_left_kneei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_kneei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_kneei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_kneei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_knee[i]);
      }
      *(outbuffer + offset + 0) = (this->right_hip_roll_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_hip_roll_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_hip_roll_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_hip_roll_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_hip_roll_length);
      for( uint32_t i = 0; i < right_hip_roll_length; i++){
      union {
        float real;
        uint32_t base;
      } u_right_hip_rolli;
      u_right_hip_rolli.real = this->right_hip_roll[i];
      *(outbuffer + offset + 0) = (u_right_hip_rolli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_hip_rolli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_hip_rolli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_hip_rolli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_hip_roll[i]);
      }
      *(outbuffer + offset + 0) = (this->right_hip_pitch_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_hip_pitch_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_hip_pitch_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_hip_pitch_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_hip_pitch_length);
      for( uint32_t i = 0; i < right_hip_pitch_length; i++){
      union {
        float real;
        uint32_t base;
      } u_right_hip_pitchi;
      u_right_hip_pitchi.real = this->right_hip_pitch[i];
      *(outbuffer + offset + 0) = (u_right_hip_pitchi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_hip_pitchi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_hip_pitchi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_hip_pitchi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_hip_pitch[i]);
      }
      *(outbuffer + offset + 0) = (this->right_knee_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_knee_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_knee_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_knee_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_knee_length);
      for( uint32_t i = 0; i < right_knee_length; i++){
      union {
        float real;
        uint32_t base;
      } u_right_kneei;
      u_right_kneei.real = this->right_knee[i];
      *(outbuffer + offset + 0) = (u_right_kneei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_kneei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_kneei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_kneei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_knee[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t right_shoulder_pitch_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      right_shoulder_pitch_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      right_shoulder_pitch_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      right_shoulder_pitch_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->right_shoulder_pitch_length);
      if(right_shoulder_pitch_lengthT > right_shoulder_pitch_length)
        this->right_shoulder_pitch = (float*)realloc(this->right_shoulder_pitch, right_shoulder_pitch_lengthT * sizeof(float));
      right_shoulder_pitch_length = right_shoulder_pitch_lengthT;
      for( uint32_t i = 0; i < right_shoulder_pitch_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_right_shoulder_pitch;
      u_st_right_shoulder_pitch.base = 0;
      u_st_right_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_right_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_right_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_right_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_right_shoulder_pitch = u_st_right_shoulder_pitch.real;
      offset += sizeof(this->st_right_shoulder_pitch);
        memcpy( &(this->right_shoulder_pitch[i]), &(this->st_right_shoulder_pitch), sizeof(float));
      }
      uint32_t right_shoulder_roll_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      right_shoulder_roll_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      right_shoulder_roll_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      right_shoulder_roll_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->right_shoulder_roll_length);
      if(right_shoulder_roll_lengthT > right_shoulder_roll_length)
        this->right_shoulder_roll = (float*)realloc(this->right_shoulder_roll, right_shoulder_roll_lengthT * sizeof(float));
      right_shoulder_roll_length = right_shoulder_roll_lengthT;
      for( uint32_t i = 0; i < right_shoulder_roll_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_right_shoulder_roll;
      u_st_right_shoulder_roll.base = 0;
      u_st_right_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_right_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_right_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_right_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_right_shoulder_roll = u_st_right_shoulder_roll.real;
      offset += sizeof(this->st_right_shoulder_roll);
        memcpy( &(this->right_shoulder_roll[i]), &(this->st_right_shoulder_roll), sizeof(float));
      }
      uint32_t right_elbow_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      right_elbow_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      right_elbow_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      right_elbow_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->right_elbow_length);
      if(right_elbow_lengthT > right_elbow_length)
        this->right_elbow = (float*)realloc(this->right_elbow, right_elbow_lengthT * sizeof(float));
      right_elbow_length = right_elbow_lengthT;
      for( uint32_t i = 0; i < right_elbow_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_right_elbow;
      u_st_right_elbow.base = 0;
      u_st_right_elbow.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_right_elbow.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_right_elbow.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_right_elbow.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_right_elbow = u_st_right_elbow.real;
      offset += sizeof(this->st_right_elbow);
        memcpy( &(this->right_elbow[i]), &(this->st_right_elbow), sizeof(float));
      }
      uint32_t left_shoulder_pitch_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      left_shoulder_pitch_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      left_shoulder_pitch_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      left_shoulder_pitch_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->left_shoulder_pitch_length);
      if(left_shoulder_pitch_lengthT > left_shoulder_pitch_length)
        this->left_shoulder_pitch = (float*)realloc(this->left_shoulder_pitch, left_shoulder_pitch_lengthT * sizeof(float));
      left_shoulder_pitch_length = left_shoulder_pitch_lengthT;
      for( uint32_t i = 0; i < left_shoulder_pitch_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_left_shoulder_pitch;
      u_st_left_shoulder_pitch.base = 0;
      u_st_left_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_left_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_left_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_left_shoulder_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_left_shoulder_pitch = u_st_left_shoulder_pitch.real;
      offset += sizeof(this->st_left_shoulder_pitch);
        memcpy( &(this->left_shoulder_pitch[i]), &(this->st_left_shoulder_pitch), sizeof(float));
      }
      uint32_t left_shoulder_roll_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      left_shoulder_roll_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      left_shoulder_roll_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      left_shoulder_roll_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->left_shoulder_roll_length);
      if(left_shoulder_roll_lengthT > left_shoulder_roll_length)
        this->left_shoulder_roll = (float*)realloc(this->left_shoulder_roll, left_shoulder_roll_lengthT * sizeof(float));
      left_shoulder_roll_length = left_shoulder_roll_lengthT;
      for( uint32_t i = 0; i < left_shoulder_roll_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_left_shoulder_roll;
      u_st_left_shoulder_roll.base = 0;
      u_st_left_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_left_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_left_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_left_shoulder_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_left_shoulder_roll = u_st_left_shoulder_roll.real;
      offset += sizeof(this->st_left_shoulder_roll);
        memcpy( &(this->left_shoulder_roll[i]), &(this->st_left_shoulder_roll), sizeof(float));
      }
      uint32_t left_elbow_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      left_elbow_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      left_elbow_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      left_elbow_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->left_elbow_length);
      if(left_elbow_lengthT > left_elbow_length)
        this->left_elbow = (float*)realloc(this->left_elbow, left_elbow_lengthT * sizeof(float));
      left_elbow_length = left_elbow_lengthT;
      for( uint32_t i = 0; i < left_elbow_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_left_elbow;
      u_st_left_elbow.base = 0;
      u_st_left_elbow.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_left_elbow.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_left_elbow.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_left_elbow.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_left_elbow = u_st_left_elbow.real;
      offset += sizeof(this->st_left_elbow);
        memcpy( &(this->left_elbow[i]), &(this->st_left_elbow), sizeof(float));
      }
      uint32_t left_hip_roll_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      left_hip_roll_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      left_hip_roll_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      left_hip_roll_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->left_hip_roll_length);
      if(left_hip_roll_lengthT > left_hip_roll_length)
        this->left_hip_roll = (float*)realloc(this->left_hip_roll, left_hip_roll_lengthT * sizeof(float));
      left_hip_roll_length = left_hip_roll_lengthT;
      for( uint32_t i = 0; i < left_hip_roll_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_left_hip_roll;
      u_st_left_hip_roll.base = 0;
      u_st_left_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_left_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_left_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_left_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_left_hip_roll = u_st_left_hip_roll.real;
      offset += sizeof(this->st_left_hip_roll);
        memcpy( &(this->left_hip_roll[i]), &(this->st_left_hip_roll), sizeof(float));
      }
      uint32_t left_hip_pitch_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      left_hip_pitch_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      left_hip_pitch_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      left_hip_pitch_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->left_hip_pitch_length);
      if(left_hip_pitch_lengthT > left_hip_pitch_length)
        this->left_hip_pitch = (float*)realloc(this->left_hip_pitch, left_hip_pitch_lengthT * sizeof(float));
      left_hip_pitch_length = left_hip_pitch_lengthT;
      for( uint32_t i = 0; i < left_hip_pitch_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_left_hip_pitch;
      u_st_left_hip_pitch.base = 0;
      u_st_left_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_left_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_left_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_left_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_left_hip_pitch = u_st_left_hip_pitch.real;
      offset += sizeof(this->st_left_hip_pitch);
        memcpy( &(this->left_hip_pitch[i]), &(this->st_left_hip_pitch), sizeof(float));
      }
      uint32_t left_knee_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      left_knee_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      left_knee_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      left_knee_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->left_knee_length);
      if(left_knee_lengthT > left_knee_length)
        this->left_knee = (float*)realloc(this->left_knee, left_knee_lengthT * sizeof(float));
      left_knee_length = left_knee_lengthT;
      for( uint32_t i = 0; i < left_knee_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_left_knee;
      u_st_left_knee.base = 0;
      u_st_left_knee.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_left_knee.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_left_knee.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_left_knee.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_left_knee = u_st_left_knee.real;
      offset += sizeof(this->st_left_knee);
        memcpy( &(this->left_knee[i]), &(this->st_left_knee), sizeof(float));
      }
      uint32_t right_hip_roll_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      right_hip_roll_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      right_hip_roll_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      right_hip_roll_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->right_hip_roll_length);
      if(right_hip_roll_lengthT > right_hip_roll_length)
        this->right_hip_roll = (float*)realloc(this->right_hip_roll, right_hip_roll_lengthT * sizeof(float));
      right_hip_roll_length = right_hip_roll_lengthT;
      for( uint32_t i = 0; i < right_hip_roll_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_right_hip_roll;
      u_st_right_hip_roll.base = 0;
      u_st_right_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_right_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_right_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_right_hip_roll.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_right_hip_roll = u_st_right_hip_roll.real;
      offset += sizeof(this->st_right_hip_roll);
        memcpy( &(this->right_hip_roll[i]), &(this->st_right_hip_roll), sizeof(float));
      }
      uint32_t right_hip_pitch_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      right_hip_pitch_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      right_hip_pitch_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      right_hip_pitch_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->right_hip_pitch_length);
      if(right_hip_pitch_lengthT > right_hip_pitch_length)
        this->right_hip_pitch = (float*)realloc(this->right_hip_pitch, right_hip_pitch_lengthT * sizeof(float));
      right_hip_pitch_length = right_hip_pitch_lengthT;
      for( uint32_t i = 0; i < right_hip_pitch_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_right_hip_pitch;
      u_st_right_hip_pitch.base = 0;
      u_st_right_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_right_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_right_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_right_hip_pitch.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_right_hip_pitch = u_st_right_hip_pitch.real;
      offset += sizeof(this->st_right_hip_pitch);
        memcpy( &(this->right_hip_pitch[i]), &(this->st_right_hip_pitch), sizeof(float));
      }
      uint32_t right_knee_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      right_knee_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      right_knee_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      right_knee_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->right_knee_length);
      if(right_knee_lengthT > right_knee_length)
        this->right_knee = (float*)realloc(this->right_knee, right_knee_lengthT * sizeof(float));
      right_knee_length = right_knee_lengthT;
      for( uint32_t i = 0; i < right_knee_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_right_knee;
      u_st_right_knee.base = 0;
      u_st_right_knee.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_right_knee.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_right_knee.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_right_knee.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_right_knee = u_st_right_knee.real;
      offset += sizeof(this->st_right_knee);
        memcpy( &(this->right_knee[i]), &(this->st_right_knee), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "humanoid_msgs/ServoFeedback"; };
    virtual const char * getMD5() override { return "b34c91e024cb21498a83e7318b1f6c21"; };

  };

}
#endif
