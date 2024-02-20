#include <Arduino.h>

#include "XL320.h"
#include "HardwareSerial.h"

// Name your robot!
XL320 robot;

// Set some variables for incrementing position & LED colour
char rgb[] = "rgbypcwo";

// Set the default servo_id to talk to
int servo_id = 16;
int led_color = 0;
int servo_setpoint_raw = 0;
float servo_setpoint_deg = 0;
int servo_pos_raw = 0;
float servo_pos_deg = 0;

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
      servo_setpoint_raw = (servo_setpoint_raw + map_float(inc_deg, 0, 359, 0, 1023));
      SerialUSB.print("Incremented position by: ");
      SerialUSB.println(inc_deg);
    }
    else if (inputString.startsWith("s "))
    {
      // Set command
      float setpoint_deg = inputString.substring(2).toFloat(); // Extract and set new position
      servo_setpoint_raw = map_float(setpoint_deg, 0, 359, 0, 1023);
      SerialUSB.print("Set position to: ");
      SerialUSB.println(setpoint_deg);
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

void servo_loop()
{
  // LED test.. let's randomly set the colour (0-7)
  robot.LED(servo_id, &rgb[random(0, 7)]);

  // SETPOINT TEST
  SerialUSB.printf("setpoint_deg=%d, ", servo_setpoint_raw);
  SerialUSB.printf("Sending %d, ", servo_setpoint_raw);
  robot.moveJoint(servo_id, servo_setpoint_raw);
  delay(100);
  //  Serial1.clear();
  byte buffer[256];
  XL320::Packet p = XL320::Packet(buffer, robot.readPacket(buffer, 256));

  // what is this for???
#if DEBUG_PRINT == 1
  p.toStream(SerialUSB);
#endif

  delay(100);
  Serial1.clear();

  // Get state
  SerialUSB.printf("Sent %d, ", servo_setpoint_raw);
  servo_pos_raw = robot.getJointPosition(servo_id);
  SerialUSB.printf("Received %d, ", servo_pos_raw);
  // servo_pos_raw = robot.getJointPosition(servo_id);
  // SerialUSB.printf("Received again %d, ", servo_pos_raw);
  SerialUSB.println();

  // Change the servo position by 100 each loop
  // servo_setpoint_raw = (servo_setpoint_raw + 100) % 1023;

  delay(100);
  // Set a delay to account for the receive delay period
}

void print_servo_state()
{
  // Print the servo state
  SerialUSB.print("Servo ID: ");
  SerialUSB.print(servo_id);
  SerialUSB.print(", Setpoint: ");
  SerialUSB.print(servo_setpoint_raw);
  SerialUSB.print(", Position: ");
  SerialUSB.println(servo_pos_raw);
}

void setup()
{
  // Talking standard serial, so connect servo data line to Digital TX 1
  // Comment out this line to talk software serial
  Serial1.begin(1000000, SERIAL_8N1_HALF_DUPLEX);

  // Initialise your robot
  robot.begin(Serial1); // Hand in the serial object you're using

  // I like fast moving servos, so set the joint speed to max!
  robot.setJointSpeed(servo_id, 1023 / 2);
}

void loop()
{
  process_serial_cmd();
  servo_loop();
}
