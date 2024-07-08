#ifndef _ROS_humanoid_msgs_UnityState_h
#define _ROS_humanoid_msgs_UnityState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

namespace humanoid_msgs
{

  class UnityState : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _local_lin_accel_type;
      _local_lin_accel_type local_lin_accel;
      typedef geometry_msgs::Quaternion _quat_type;
      _quat_type quat;
      typedef geometry_msgs::Vector3 _global_vel_type;
      _global_vel_type global_vel;
      typedef geometry_msgs::Vector3 _ang_vel_type;
      _ang_vel_type ang_vel;

    UnityState():
      local_lin_accel(),
      quat(),
      global_vel(),
      ang_vel()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->local_lin_accel.serialize(outbuffer + offset);
      offset += this->quat.serialize(outbuffer + offset);
      offset += this->global_vel.serialize(outbuffer + offset);
      offset += this->ang_vel.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->local_lin_accel.deserialize(inbuffer + offset);
      offset += this->quat.deserialize(inbuffer + offset);
      offset += this->global_vel.deserialize(inbuffer + offset);
      offset += this->ang_vel.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "humanoid_msgs/UnityState"; };
    virtual const char * getMD5() override { return "18fdd413929dedd88e59a7502509dced"; };

  };

}
#endif
