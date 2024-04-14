#include "common.h"
#if COMPILE_MODE == COMPILE_FOR_SERIAL
#include <Arduino.h>
#include "helpers.cpp"

#include "HardwareSerial.h"
#include "ServoChain.h"
#include "control_table_xl320.h"

// Include the ROS library
#include "ServoCommand.h"
#include "ServoFeedback.h"
#include "ros.h"

#define CONTROL_LOOP_US 10000
#define LIGHTS_MILLIS 500

static unsigned long lastTime;
// static unsigned long lastMillis;
ros::NodeHandle nh;
void servo_cmd_cb(const humanoid_msgs::ServoCommand &input_msg);

humanoid_msgs::ServoCommand servo_cmd_msg;
humanoid_msgs::ServoFeedback servo_fb_msg;
ros::Publisher servo_fb_pub("servosFeedback", &servo_fb_msg);
ros::Subscriber<humanoid_msgs::ServoCommand> servo_cmd_sub("servosCommand",
                                                           servo_cmd_cb);

#if LEFT_LEG_ON == 1
XL320Chain left_leg_bus(LEFT_LEG_DIR_PIN, &LEFT_LEG_SERIAL);
uint8_t left_leg_ids[LEFT_LEG_NUM_IDS] = {
    LEFT_LEG_ANKLE_ID,
    LEFT_LEG_KNEE_ID,
    LEFT_LEG_HIP_PITCH_ID,
    LEFT_LEG_HIP_ROLL_ID,
    LEFT_LEG_HIP_YAW_ID,
};

#endif
#if RIGHT_LEG_ON == 1
XL320Chain right_leg_bus(RIGHT_LEG_DIR_PIN, &RIGHT_LEG_SERIAL);
uint8_t right_leg_ids[RIGHT_LEG_NUM_IDS] = {
    RIGHT_LEG_ANKLE_ID,
    RIGHT_LEG_KNEE_ID,
    RIGHT_LEG_HIP_PITCH_ID,
    RIGHT_LEG_HIP_ROLL_ID,
    RIGHT_LEG_HIP_YAW_ID,
};
#endif

#if LEFT_LEG_ON == 1
uint16_t left_leg_setpoints[LEFT_LEG_NUM_IDS] = {
    512, 512, 512, 512, 512};                 // hip roll, hip pitch, knee, ankle
float left_leg_feedback[LEFT_LEG_NUM_IDS][3]; // in each ID, array with
                                              // position, velocity and load
// uint8_t left_leg_colors[LEFT_LEG_NUM_IDS] = {3, 2, 3, 2};
#endif

#if RIGHT_LEG_ON == 1
uint16_t right_leg_setpoints[RIGHT_LEG_NUM_IDS] = {
    512, 512, 512, 512, 512};                   // hip roll, hip pitch, knee, ankle
float right_leg_feedback[RIGHT_LEG_NUM_IDS][3]; // in each ID, array with
                                                // position, velocity and load
#endif

float map_float(float x, float in_min, float in_max, float out_min,
                float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void rawToFloat(unsigned short *input, float *output)
{
  output[0] = map_float(input[0], 0, 1023, 0, 300.00);                                                                             // pos in degrees
  output[1] = (input[1] >= 1024) ? map_float(input[1] & 0x3FF, 0, 1023, 0, 33.3) : map_float(input[1] & 0x3FF, 0, 1023, 0, -33.3); // velocity in RPM
  output[2] = (input[2] >= 1024) ? map_float(input[2] & 0x3FF, 0, 1023, 0, 1.0) : map_float(input[2] & 0x3FF, 0, 1023, 0, -1.0);   // velocity in RPM
}

float raw2deg(float raw) { return map_float(raw, 0, 1023, 0, 300); }

float deg2raw(float deg) { return map_float(deg, 0, 300, 0, 1023); }

void process_serial_cmd()
{
  static String inputString = "";             // A String to hold incoming data
  static boolean inputStringComplete = false; // Whether the string is complete

  while (SerialUSB.available())
  {
    char inChar = (char)SerialUSB.read(); // Read each character
    if (inChar == '\n')
    {
      inputStringComplete = true; // If newline, input is complete
    }
    else
    {
      inputString += inChar; // Add character to input
    }
  }

  if (inputStringComplete)
  {
    SerialUSB.print("Received: ");
    SerialUSB.println(inputString); // Echo the input for debugging

    // Process the completed command
    if (inputString.startsWith("i "))
    {
      // Increment command
      float inc_deg = inputString.substring(2).toFloat(); // Extract number
      float servo_setpoint_raw = (servo_setpoint_raw + deg2raw(inc_deg));
      SerialUSB.print("Incremented position by: ");
      SerialUSB.println(inc_deg);
    }
    else if (inputString.startsWith("s "))
    {
      // Set command
      float servo_setpoint_deg = inputString.substring(2).toFloat(); // Extract and set new position
      float servo_setpoint_raw = deg2raw(servo_setpoint_deg);

      for (int i = 0; i < LEFT_LEG_NUM_IDS; i++)
      {
        left_leg_setpoints[i] = servo_setpoint_raw;
      }
      for (int i = 0; i < RIGHT_LEG_NUM_IDS; i++)
      {
        right_leg_setpoints[i] = servo_setpoint_raw;
      }

      SerialUSB.print("Set position to: ");
      SerialUSB.println(servo_setpoint_deg);
    }
    else
    {
      SerialUSB.println("Unknown command");
    }

    // Clear the string for the next command
    inputString = "";
    inputStringComplete = false;
  }
}

// Get the feedback from the servos
int updatePositions()
{
  unsigned short rcv_buf[3 * (RIGHT_LEG_NUM_IDS + LEFT_LEG_NUM_IDS)];
  int error = 0;
#if LEFT_LEG_ON == 1
  error =
      left_leg_bus.getServoData(left_leg_ids, rcv_buf, LEFT_LEG_NUM_IDS);
  if (error)
    return error;
  for (int i = 0; i < LEFT_LEG_NUM_IDS; i++)
  {
    rawToFloat(&rcv_buf[3 * i], left_leg_feedback[i]);
  }
#endif
#if RIGHT_LEG_ON == 1
  error =
      right_leg_bus.getServoData(right_leg_ids, rcv_buf, RIGHT_LEG_NUM_IDS);
  if (error)
    return error;
  for (int i = 0; i < RIGHT_LEG_NUM_IDS; i++)
  {
    right_leg_feedback[i][0] = map_float(rcv_buf[3 * i], 0, 1023, 0, 359.99);
    right_leg_feedback[i][1] =
        map_float(rcv_buf[3 * i + 1], 0, 1023, 0, 359.99);
    right_leg_feedback[i][2] =
        map_float(rcv_buf[3 * i + 2], 0, 1023, 0, 359.99);
  }
#endif
  return 0;
}

// Apply the setpoints to the servos
void sendSetpoints()
{
#if LEFT_LEG_ON == 1
  left_leg_bus.setServoPositions(left_leg_ids, left_leg_setpoints,
                                 LEFT_LEG_NUM_IDS);
#endif
#if RIGHT_LEG_ON == 1
  right_leg_bus.setServoPositions(right_leg_ids, right_leg_setpoints,
                                  RIGHT_LEG_NUM_IDS);
#endif
}

// void toggleLights()
// {
//   // left leg lights
// #if LEFT_LEG_ON == 1
//   for (int i = 0; i < LEFT_LEG_NUM_IDS; i++)
//   {
//     if (left_leg_colors[i] == 2)
//     {
//       left_leg_colors[i] = 3;
//     }
//     else
//     {
//       left_leg_colors[i] = 2;
//     }
//   }
//   left_leg_bus.setLEDs(left_leg_ids, left_leg_colors, LEFT_LEG_NUM_IDS);
// #endif
//   // right leg lights
// #if RIGHT_LEG_ON == 1
//   for (int i = 0; i < RIGHT_LEG_NUM_IDS; i++)
//   {
//     if (right_leg_colors[i] == 2)
//     {
//       right_leg_colors[i] = 3;
//     }
//     else
//     {
//       right_leg_colors[i] = 2;
//     }
//   }
//   right_leg_bus.setLEDs(right_leg_ids, right_leg_colors, RIGHT_LEG_NUM_IDS);
// #endif
// }

void setup()
{
  SerialUSB.begin(115200);
  lastTime = micros();
  // lastMillis = millis();
}

void loop()
{
  process_serial_cmd();
  if (micros() - lastTime > CONTROL_LOOP_US)
  {
    lastTime = micros();
    sendSetpoints();

    int error = updatePositions();
    for (int i = 0; i < LEFT_LEG_NUM_IDS; i++)
    {
      SerialUSB.printf("Left Leg ID: %d, Pos: %f, Vel: %f, Load: %f\n",
                       left_leg_ids[i], left_leg_feedback[i][0],
                       left_leg_feedback[i][1], left_leg_feedback[i][2]);
    }
    if (error)
    {
      // add error handling
    }
  }
}
#endif // COMPILE_MODE == COMPILE_FOR_SERIAL