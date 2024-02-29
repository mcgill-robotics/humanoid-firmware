#include <Arduino.h>

#include "HardwareSerial.h"
#include "ServoChain.h"

// Include the ROS library
#include "ServoCommand.h"
#include "ServoFeedback.h"
#include "ros.h"

#define CONTROL_LOOP_US 10000
// left leg definitions
#define LEFT_LEG_SERIAL Serial1
#define LEFT_LEG_DIR_PIN 3
#define LEFT_LEG_ANKLE_ID 16
#define LEFT_LEG_KNEE_ID 17
#define LEFT_LEG_HIP_PITCH_ID 18
#define LEFT_LEG_HIP_ROLL_ID 19

static unsigned long lastTime;
ros::NodeHandle nh;
void servo_cmd_cb(const servo_node::ServoCommand &input_msg);

servo_node::ServoCommand servo_cmd_msg;
servo_node::ServoFeedback servo_fb_msg;
ros::Publisher servo_fb_pub("servosFeedback", &servo_fb_msg);
ros::Subscriber<servo_node::ServoCommand> servo_cmd_sub("servosCommand",
                                                        servo_cmd_cb);

XL320Chain left_leg_bus(LEFT_LEG_DIR_PIN, &LEFT_LEG_SERIAL);
uint8_t left_leg_ids[4] = {LEFT_LEG_HIP_ROLL_ID, LEFT_LEG_HIP_PITCH_ID,
                           LEFT_LEG_KNEE_ID, LEFT_LEG_ANKLE_ID};

float map_float(float x, float in_min, float in_max, float out_min,
                float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint16_t left_leg_setpoints[4] = {0}; // hip roll, hip pitch, knee, ankle
float left_leg_positions[4] = {0.0};  // hip roll, hip pitch, knee, ankle
float left_leg_velocities[4] = {0.0}; // hip roll, hip pitch, knee, ankle
float left_leg_loads[4] = {0.0};      // hip roll, hip pitch, knee, ankle

int updatePositions(int num_ids)
{
  unsigned short rcv_buf[3 * num_ids];
  // left leg updates
  int error = left_leg_bus.getServoData(left_leg_ids, rcv_buf, num_ids);
  if (error)
    return error;
  for (int i = 0; i < num_ids; i++)
  {
    left_leg_positions[i] = map_float(rcv_buf[3 * i], 0, 1023, 0, 359.99);
    left_leg_velocities[i] = map_float(rcv_buf[3 * i + 1], 0, 1023, 0, 359.99);
    left_leg_loads[i] = map_float(rcv_buf[3 * i + 2], 0, 1023, 0, 359.99);
  }
  // do other limbs later
  return 0;
}

void sendSetpoints(int num_ids)
{
  // left leg write
  left_leg_bus.setServoPositions(left_leg_ids, left_leg_setpoints, num_ids);
  // do other limbs later
}

void setup() { LEFT_LEG_SERIAL.begin(1000000, SERIAL_8N1_HALF_DUPLEX); }

void loop() {}