#include "common.h"
#if COMPILE_CFG == COMPILE_CFG_SINGLE_SERVO
#include <Arduino.h>

#include "XL320.h"
#include "HardwareSerial.h"

#define DEBUG_PRINT 0

// SERVO CONFIGURATION
// #define SERVO_BAUD 9600
#define SERVO_BAUD 1000000
XL320 robot;
char rgb[] = "rgbypcwo";
int servo_id = 14;
int led_color = 0;
float servo_setpoint_raw = 0;
float servo_setpoint_deg = 0;
float servo_pos_raw = 0;
float servo_pos_deg = 0;

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float raw2deg(float raw)
{
  return map_float(raw, 0, 1023, 0, 300);
}

float deg2raw(float deg)
{
  return map_float(deg, 0, 300, 0, 1023);
}

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
      servo_setpoint_raw = (servo_setpoint_raw + deg2raw(inc_deg));
      SerialUSB.print("Incremented position by: ");
      SerialUSB.println(inc_deg);
    }
    else if (inputString.startsWith("s "))
    {
      // Set command
      servo_setpoint_deg = inputString.substring(2).toFloat(); // Extract and set new position
      servo_setpoint_raw = deg2raw(servo_setpoint_deg);
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

void servo_setup()
{
  // Talking standard serial, so connect servo data line to Digital TX 1
  // Comment out this line to talk software serial
  Serial1.begin(SERVO_BAUD, SERIAL_8N1_HALF_DUPLEX);

  // Initialise your robot
  robot.begin(Serial1); // Hand in the serial object you're using

  // I like fast moving servos, so set the joint speed to max!
  // robot.setJointSpeed(servo_id, 1023 / 2);
  robot.setJointSpeed(servo_id, 1023 / 2);

  servo_setpoint_raw = 512;
  servo_setpoint_deg = raw2deg(servo_setpoint_raw);
}

void servo_loop()
{
  // LED test.. let's randomly set the colour (0-7)
  robot.LED(servo_id, &rgb[random(0, 7)]);

  // SETPOINT TEST
  robot.moveJoint(servo_id, servo_setpoint_raw);
  delay(100);
  //  Serial1.clear();
  byte buffer[256];
  XL320::Packet p = XL320::Packet(buffer, robot.readPacket(buffer, 256));

  // Driver diagnostic
#if DEBUG_PRINT == 1
  p.toStream(SerialUSB);
#endif

  delay(100);
  Serial1.clear();

  // Get state
  servo_pos_raw = robot.getJointPosition(servo_id);
  // servo_pos_raw = robot.getJointPosition(servo_id, &SerialUSB);

  servo_pos_deg = raw2deg(servo_pos_raw);

  delay(100);
}

uint32_t last_print = 0;
void print_servo_state()
{
  // Print the servo state
  SerialUSB.print("Servo ID: ");
  SerialUSB.print(servo_id);
  SerialUSB.print(", setpoint_deg: ");
  SerialUSB.print(servo_setpoint_deg);
  SerialUSB.print(", pos_deg: ");
  SerialUSB.print(servo_pos_deg);
  SerialUSB.print(", setpoint_raw: ");
  SerialUSB.print(servo_setpoint_raw);
  SerialUSB.print(", pos_raw: ");
  SerialUSB.println(servo_pos_raw);
}

void setup()
{
  SerialUSB.begin(115200);
  while (!SerialUSB)
    ;
  servo_setup();
}

void loop()
{
  process_serial_cmd();
  servo_loop();
  if (millis() - last_print > 500)
  {
    print_servo_state();
    last_print = millis();
  }
}
#endif // COMPILE_CFG == COMPILE_CFG_SINGLE_SERVO