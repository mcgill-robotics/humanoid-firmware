#include <Arduino.h>

#include "HardwareSerial.h"
#include "XL320.h"

#define DEBUG_PRINT 0

// SERVO CONFIGURATION
XL320 robot;
char rgb[] = "rgbypcwo";
int servo_id1 = 17;
int servo_id2 = 20;
int led_color = 0;
int setNewID = 0;
int setNewBaud = -1;
float servo_setpoint_raw[2] = {0};
float servo_setpoint_deg[2] = {0};
float servo_pos_raw[2] = {0};
float servo_pos_deg[2] = {0};

float map_float(float x, float in_min, float in_max, float out_min,
                float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float raw2deg(float raw) { return map_float(raw, 0, 1023, 0, 300); }

float deg2raw(float deg) { return map_float(deg, 0, 300, 0, 1023); }

bool validID(int id) { return (id >= 1 && id <= 253 && id != 200); }

bool validBaud(int newBaud) { return (newBaud >= 0 && newBaud <= 3); }

float servoSine(float offset, float amplitude, float w, float time_offset)
{
  float time = millis() / 1000.0 + time_offset;
  return offset + amplitude * sin(w * time);
}

// void process_serial_cmd()
// {
//   static String inputString = "";             // A String to hold incoming data
//   static boolean inputStringComplete = false; // Whether the string is complete

//   while (SerialUSB.available())
//   {
//     char inChar = (char)SerialUSB.read(); // Read each character
//     if (inChar == '\n')
//     {
//       inputStringComplete = true; // If newline, input is complete
//     }
//     else
//     {
//       inputString += inChar; // Add character to input
//     }
//   }

//   if (inputStringComplete)
//   {
//     SerialUSB.print("Received: ");
//     SerialUSB.println(inputString); // Echo the input for debugging

//     // Process the completed command
//     if (inputString.startsWith("i "))
//     {
//       // Increment command
//       float inc_deg = inputString.substring(2).toFloat(); // Extract number
//       servo_setpoint_raw = (servo_setpoint_raw + deg2raw(inc_deg));
//       SerialUSB.print("Incremented position by: ");
//       SerialUSB.println(inc_deg);
//     }
//     else if (inputString.startsWith("s "))
//     {
//       // Set command
//       servo_setpoint_deg =
//           inputString.substring(2).toFloat(); // Extract and set new position
//       servo_setpoint_raw = deg2raw(servo_setpoint_deg);
//       SerialUSB.print("Set position to: ");
//       SerialUSB.println(servo_setpoint_deg);
//     }
//     else if (inputString.startsWith("d "))
//     {
//       // Set ID command
//       int new_ID = inputString.substring(2).toInt(); // Extract ID
//       if (validID(new_ID))
//       {
//         setNewID = new_ID;
//         SerialUSB.print("Will set new ID to: ");
//         SerialUSB.println(setNewID);
//       }
//       else
//       {
//         SerialUSB.println("Invalid ID");
//       }
//     }
//     else if (inputString.startsWith("b ")){
//       // Set baudrate command
//       int new_baud = inputString.substring(2).toInt(); // Extract ID
//       if (validBaud(new_baud))
//       {
//         setNewBaud = new_baud;
//         SerialUSB.print("Will set new Baud Rate to: ");
//         SerialUSB.println(setNewBaud);
//       }
//       else
//       {
//         SerialUSB.println("Invalid Baud Rate");
//       }
//     }
//     else
//     {
//       SerialUSB.println("Unknown command");
//     }

//     // Clear the string for the next command
//     inputString = "";
//     inputStringComplete = false;
//   }
// }

void servo_setup()
{
  // Talking standard serial, so connect servo data line to Digital TX 1
  // Comment out this line to talk software serial
  Serial1.begin(1000000, SERIAL_8N1_HALF_DUPLEX);

  // Initialise your robot
  robot.begin(Serial1); // Hand in the serial object you're using

  // I like fast moving servos, so set the joint speed to max!
  // robot.setJointSpeed(servo_id, 1023 / 2);
  // robot.setJointSpeed(servo_id, 1023);

  servo_setpoint_raw[0] = 512;
  servo_setpoint_raw[1] = 512;
  servo_setpoint_deg[0] = raw2deg(servo_setpoint_raw[0]);
  servo_setpoint_deg[1] = raw2deg(servo_setpoint_raw[1]);
}

void servo_loop()
{
  // LED test.. let's randomly set the colour (0-7)
  // robot.LED(servo_id, &rgb[random(0, 7)]);
  servo_setpoint_deg[0] = servoSine(150, 50, 1.59, 0);
  servo_setpoint_deg[1] = servoSine(150, 50, 1.59, -5);
  servo_setpoint_raw[0] = deg2raw(servo_setpoint_deg[0]);
  servo_setpoint_raw[1] = deg2raw(servo_setpoint_deg[1]);

  // SETPOINT TEST
  robot.moveJoint(servo_id1, servo_setpoint_raw[0]);
  robot.moveJoint(servo_id2, servo_setpoint_raw[1]);
  // delay(100);
  //  Serial2.clear();
  // byte buffer[256];
  // XL320::Packet p = XL320::Packet(buffer, robot.readPacket(buffer, 256));

  // Driver diagnostic
#if DEBUG_PRINT == 1
  p.toStream(SerialUSB);
#endif

  // delay(100);
  // Serial2.clear();

  // Get state
  // delay(100);
  // int temp_pos = robot.getJointPosition(servo_id1);
  // if (temp_pos >= 0){
  //   servo_pos_raw[0] = temp_pos;
  // }
  // temp_pos = robot.getJointPosition(servo_id2);
  // if (temp_pos >= 0){
  //   servo_pos_raw[1] = temp_pos;
  // }
  // // servo_pos_raw = robot.getJointPosition(servo_id);

  // servo_pos_deg[0] = raw2deg(servo_pos_raw[0]);
  // servo_pos_deg[1] = raw2deg(servo_pos_raw[1]);

  // delay(100);
}

uint32_t last_print = 0;
void print_servo_state()
{
  // Print the servo state
  SerialUSB.print("Servo ID: ");
  SerialUSB.print(servo_id1);
  SerialUSB.print(", setpoint_deg: ");
  SerialUSB.print(servo_setpoint_deg[0]);
  SerialUSB.print(", pos_deg: ");
  SerialUSB.print(servo_pos_deg[0]);
  SerialUSB.print(", setpoint_raw: ");
  SerialUSB.print(servo_setpoint_raw[0]);
  SerialUSB.print(", pos_raw: ");
  SerialUSB.println(servo_pos_raw[0]);

  SerialUSB.print("Servo ID: ");
  SerialUSB.print(servo_id2);
  SerialUSB.print(", setpoint_deg: ");
  SerialUSB.print(servo_setpoint_deg[1]);
  SerialUSB.print(", pos_deg: ");
  SerialUSB.print(servo_pos_deg[1]);
  SerialUSB.print(", setpoint_raw: ");
  SerialUSB.print(servo_setpoint_raw[1]);
  SerialUSB.print(", pos_raw: ");
  SerialUSB.println(servo_pos_raw[1]);
}

void setup()
{
  SerialUSB.begin(115200);
  while (!SerialUSB)
    ;
  servo_setup();
  // Serial1.begin(1000000, SERIAL_8N1_HALF_DUPLEX);
  // Serial2.begin(1000000, SERIAL_8N1_HALF_DUPLEX);

  byte idBuf[16];
  int numIDs = robot.broadcastPing(&SerialUSB, idBuf);
  if (numIDs)
  {
    SerialUSB.print(numIDs);
    SerialUSB.print(" IDs found: ");
    for (int i = 0; i < numIDs; i++)
    {
      SerialUSB.print(idBuf[i]);
      SerialUSB.print(",");
    }
    SerialUSB.println();
  }
  else
  {
    SerialUSB.println("No IDs found.");
  }
  delay(3000);
  // robot.sendPacket(servo_id, XL_ID, 20);
  // SerialUSB.print("New ID sent to robot: ");
  // SerialUSB.println(20);
  // SerialUSB.println("Don't forget to power cycle the servo!");

  // robot.sendPacket(servo_id, XL_BAUD_RATE, 3);
  // delay(20);
  // robot.sendPacket(servo_id, XL_BAUD_RATE, 3);
  //   delay(20);
  // robot.sendPacket(servo_id, XL_BAUD_RATE, 3);
  //   delay(20);
  // robot.sendPacket(servo_id, XL_BAUD_RATE, 3);
  //   delay(20);
  // robot.sendPacket(servo_id, XL_BAUD_RATE, 3);
  // // SerialUSB.print("New Baud Rate sent to robot: ");
  // // SerialUSB.println(3);
  // SerialUSB.println("Don't forget to power cycle the servo!");
  // int id_found = robot.queryID();
  // SerialUSB.printf("Found servo ID: %d\n", id_found);
  SerialUSB.println("Starting Loop");
}

void loop()
{
  // process_serial_cmd();

  servo_loop();
  // if (millis() - last_print > 500)
  // {
  //   print_servo_state();
  //   last_print = millis();
  // }
  delay(2);
}
