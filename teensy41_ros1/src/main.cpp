#include <Arduino.h>

// Include the ROS library
#include "ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

#define PID_PERIOD_US 10000

static unsigned long lastTime;
ros::NodeHandle nh;

void servo_cb(const std_msgs::Float32MultiArray &input_msg);

std_msgs::Float32MultiArray servo_fb_msg;
std_msgs::Float32MultiArray servo_cmd_msg;
ros::Publisher servo_fb_pub("servo_fb", &servo_fb_msg);
ros::Subscriber<std_msgs::Float32MultiArray> servo_cmd_sub("servo_cmd", servo_cb);

float servo_current_angles[1] = {0};
float servo_target_angles[1] = {0};

void servo_setup()
{
}

void servo_loop()
{
}

void setup()
{
  nh.initNode();

  servo_setup();
  servo_fb_msg.data = servo_current_angles;
  servo_fb_msg.data_length = 1;
  servo_cmd_msg.data_length = 1;

  nh.advertise(servo_fb_pub);
  nh.subscribe(servo_cmd_sub);

  nh.negotiateTopics();
  while (!nh.connected())
  {
    nh.negotiateTopics();
  }

  lastTime = micros();
}

void loop()
{
  while (micros() < lastTime + PID_PERIOD_US)
    ;
  lastTime += PID_PERIOD_US;

  servo_loop();
  servo_fb_pub.publish(&servo_fb_msg);

  nh.spinOnce();
}

void servo_cb(const std_msgs::Float32MultiArray &input_msg)
{
  servo_target_angles[0] = input_msg.data[0];
}
