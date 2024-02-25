#include <Arduino.h>

#include "XL320.h"
#include "HardwareSerial.h"

// SERVO CONFIGURATION
XL320 robot;
char rgb[] = "rgbypcwo";
int servo_id = 16;
int led_color = 0;
float servo_setpoint_raw = 0;
float servo_setpoint_deg = 0;
float servo_pos_raw = 0;
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
      servo_setpoint_raw = (servo_setpoint_raw + map_float(inc_deg, 0, 359.99, 0, 1023));
      SerialUSB.print("Incremented position by: ");
      SerialUSB.println(inc_deg);
    }
    else if (inputString.startsWith("s "))
    {
      // Set command
      servo_setpoint_deg = inputString.substring(2).toFloat(); // Extract and set new position
      servo_setpoint_raw = map_float(servo_setpoint_deg, 0, 359.99, 0, 1023);
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
  Serial1.begin(1000000, SERIAL_8N1_HALF_DUPLEX);

  // Initialise your robot
  robot.begin(Serial1); // Hand in the serial object you're using

  // I like fast moving servos, so set the joint speed to max!
  robot.setJointSpeed(servo_id, 1023 / 2);
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
  servo_pos_deg = map_float(servo_pos_raw, 0, 1023, 0, 359.99);

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
  int id_found = robot.queryID();
  SerialUSB.printf("Found servo ID: %d\n", id_found);
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
