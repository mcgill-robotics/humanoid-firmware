#include <Arduino.h>

#include "HardwareSerial.h"
#include "ServoChain.h"
#include "control_table_xl320.h"

// Include the ROS library
#include "ServoCommand.h"
#include "ServoFeedback.h"
#include "ros.h"

#define CONTROL_LOOP_US 10000

#define LEFT_LEG_ON
// #define RIGHT_LEG_ON

static unsigned long lastTime;
ros::NodeHandle nh;
void servo_cmd_cb(const servo_node::ServoCommand &input_msg);

servo_node::ServoCommand servo_cmd_msg;
servo_node::ServoFeedback servo_fb_msg;
ros::Publisher servo_fb_pub("servosFeedback", &servo_fb_msg);
ros::Subscriber<servo_node::ServoCommand> servo_cmd_sub("servosCommand",
                                                        servo_cmd_cb);

#ifdef LEFT_LEG_ON
XL320Chain left_leg_bus(LEFT_LEG_DIR_PIN, &LEFT_LEG_SERIAL);
uint8_t left_leg_ids[LEFT_LEG_NUM_IDS] = {LEFT_LEG_HIP_ROLL_ID,
                                          LEFT_LEG_HIP_PITCH_ID,
                                          LEFT_LEG_KNEE_ID, LEFT_LEG_ANKLE_ID};
#endif
#ifdef RIGHT_LEG_ON
XL320Chain right_leg_bus(RIGHT_LEG_DIR_PIN, &RIGHT_LEG_SERIAL);
uint8_t right_leg_ids[RIGHT_LEG_NUM_IDS] = {
    RIGHT_LEG_HIP_ROLL_ID, RIGHT_LEG_HIP_PITCH_ID, RIGHT_LEG_KNEE_ID,
    RIGHT_LEG_ANKLE_ID};
#endif

float map_float(float x, float in_min, float in_max, float out_min,
                float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#ifdef LEFT_LEG_ON
uint16_t left_leg_setpoints[LEFT_LEG_NUM_IDS] = {
    0};  // hip roll, hip pitch, knee, ankle
float left_leg_feedback[LEFT_LEG_NUM_IDS][3];  // in each ID, array with
                                               // position, velocity and load
#endif

#ifdef RIGHT_LEG_ON
uint16_t right_leg_setpoints[RIGHT_LEG_NUM_IDS] = {
    0};  // hip roll, hip pitch, knee, ankle
float right_leg_feedback[RIGHT_LEG_NUM_IDS][3];  // in each ID, array with
                                                 // position, velocity and load
#endif

int updatePositions() {
// left leg updates
#ifdef LEFT_LEG_ON
  unsigned short rcv_buf[3 * LEFT_LEG_NUM_IDS];
  int error =
      left_leg_bus.getServoData(left_leg_ids, rcv_buf, LEFT_LEG_NUM_IDS);
  if (error) return error;
  for (int i = 0; i < LEFT_LEG_NUM_IDS; i++) {
    left_leg_feedback[i][0] = map_float(rcv_buf[3 * i], 0, 1023, 0, 359.99);
    left_leg_feedback[i][1] = map_float(rcv_buf[3 * i + 1], 0, 1023, 0, 359.99);
    left_leg_feedback[i][2] = map_float(rcv_buf[3 * i + 2], 0, 1023, 0, 359.99);
  }
#endif
// right leg updates
#ifdef RIGHT_LEG_ON
  unsigned short rcv_buf[3 * RIGHT_LEG_NUM_IDS];
  int error =
      right_leg_bus.getServoData(right_leg_ids, rcv_buf, RIGHT_LEG_NUM_IDS);
  if (error) return error;
  for (int i = 0; i < RIGHT_LEG_NUM_IDS; i++) {
    right_leg_feedback[i][0] = map_float(rcv_buf[3 * i], 0, 1023, 0, 359.99);
    right_leg_feedback[i][1] =
        map_float(rcv_buf[3 * i + 1], 0, 1023, 0, 359.99);
    right_leg_feedback[i][2] =
        map_float(rcv_buf[3 * i + 2], 0, 1023, 0, 359.99);
  }
#endif
  return 0;
}

void sendSetpoints() {
// left leg write
#ifdef LEFT_LEG_ON
  left_leg_bus.setServoPositions(left_leg_ids, left_leg_setpoints,
                                 LEFT_LEG_NUM_IDS);
#endif
  // right leg write
#ifdef RIGHT_LEG_ON
  right_leg_bus.setServoPositions(right_leg_ids, right_leg_setpoints,
                                  RIGHT_LEG_NUM_IDS);
#endif
}

void setup() {
#ifdef LEFT_LEG_ON
  LEFT_LEG_SERIAL.begin(1000000, SERIAL_8N1_HALF_DUPLEX);
  while (!LEFT_LEG_SERIAL)
    ;
  servo_fb_msg.left_leg_hip_roll_fb = left_leg_feedback[0];
  servo_fb_msg.left_leg_hip_roll_fb_length = 3;
  servo_fb_msg.left_leg_hip_pitch_fb = left_leg_feedback[1];
  servo_fb_msg.left_leg_hip_pitch_fb_length = 3;
  servo_fb_msg.left_leg_knee_fb = left_leg_feedback[2];
  servo_fb_msg.left_leg_knee_fb_length = 3;
  servo_fb_msg.left_leg_ankle_fb = left_leg_feedback[3];
  servo_fb_msg.left_leg_ankle_fb_length = 3;

  // add config msgs (maxspeed, max angles, PID, Torque ON)
#endif
#ifdef RIGHT_LEG_ON
  RIGHT_LEG_SERIAL.begin(1000000, SERIAL_8N1_HALF_DUPLEX);
  while (!RIGHT_LEG_SERIAL)
    ;
  servo_fb_msg.right_leg_hip_roll_fb = right_leg_feedback[0];
  servo_fb_msg.right_leg_hip_roll_fb_length = 3;
  servo_fb_msg.right_leg_hip_pitch_fb = right_leg_feedback[1];
  servo_fb_msg.right_leg_hip_pitch_fb_length = 3;
  servo_fb_msg.right_leg_knee_fb = right_leg_feedback[2];
  servo_fb_msg.right_leg_knee_fb_length = 3;
  servo_fb_msg.right_leg_ankle_fb = right_leg_feedback[3];
  servo_fb_msg.right_leg_ankle_fb_length = 3;
#endif

  // ROS stuff
  nh.initNode();
  nh.subscribe(servo_cmd_sub);
  nh.advertise(servo_fb_pub);
  nh.negotiateTopics();
  while (!nh.connected()) {
    nh.negotiateTopics();
  }

  lastTime = micros();
}

void loop() {
  while (micros() < lastTime + CONTROL_LOOP_US)
    ;
  lastTime += CONTROL_LOOP_US;

  sendSetpoints();

  int error = updatePositions();
  if (error) {
    // add error handling
  }

  servo_fb_pub.publish(&servo_fb_msg);

  nh.spinOnce();
}

void servo_cmd_cb(const servo_node::ServoCommand &input_msg) {
#ifdef LEFT_LEG_ON
  left_leg_setpoints[0] =
      map_float(input_msg.left_leg_hip_roll_setpoint, 0, 300.00, 0, 1023);
  left_leg_setpoints[1] =
      map_float(input_msg.left_leg_hip_pitch_setpoint, 0, 300.00, 0, 1023);
  left_leg_setpoints[2] =
      map_float(input_msg.left_leg_knee_setpoint, 0, 300.00, 0, 1023);
  left_leg_setpoints[3] =
      map_float(input_msg.left_leg_ankle_setpoint, 0, 300.00, 0, 1023);
#endif
#ifdef RIGHT_LEG_ON
  right_leg_setpoints[0] =
      map_float(input_msg.right_leg_hip_roll_setpoint, 0, 300.00, 0, 1023);
  right_leg_setpoints[1] =
      map_float(input_msg.right_leg_hip_pitch_setpoint, 0, 300.00, 0, 1023);
  right_leg_setpoints[2] =
      map_float(input_msg.right_leg_knee_setpoint, 0, 300.00, 0, 1023);
  right_leg_setpoints[3] =
      map_float(input_msg.right_leg_ankle_setpoint, 0, 300.00, 0, 1023);
#endif
}