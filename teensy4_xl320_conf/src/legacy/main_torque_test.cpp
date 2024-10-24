#include "common.h"
#if COMPILE_CFG == 11

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

#include <cmath>

// ------------------ Dynamixel ------------------
#define DXL_SERIAL Serial2
#define DEBUG_SERIAL Serial5
const int DXL_DIR_PIN = -1; // DYNAMIXEL Shield DIR PIN

// Uncomment the next line to enable debug printing
#define ENABLE_DEBUG_PRINT // Comment this line to disable debug prints

#ifdef ENABLE_DEBUG_PRINT
#define DEBUG_PRINT(x) DEBUG_SERIAL.print(x)
#define DEBUG_PRINTLN(x) DEBUG_SERIAL.println(x)
#define DEBUG_PRINTF(...) DEBUG_SERIAL.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(x)    // Empty definition
#define DEBUG_PRINTLN(x)  // Empty definition
#define DEBUG_PRINTF(...) // Empty definition
#endif

const uint8_t DXL_ID = 15;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// This namespace is required to use Control table item names
using namespace ControlTableItem;

int last_time = 0;

int op_mode = OP_POSITION;

float present_position = 0;
float present_velocity = 0;
float present_pwm = 0;
float present_current = 0;
float present_load = 0;

float desired_position = 0;
float desired_pwm = 0;

void process_serial_cmd()
{
  static String inputString = "";

  while (DEBUG_SERIAL.available())
  {
    char inChar = (char)DEBUG_SERIAL.read(); // Read each character
    DEBUG_SERIAL.printf("Received: %c\r\n", inChar);
    // Process the completed command
    if (inChar == 'w')
    {
      DEBUG_SERIAL.println("moving up");
      if (op_mode == OP_PWM)
      {
        desired_pwm = fmod(present_pwm + 100, 885);
        dxl.setGoalPWM(DXL_ID, desired_pwm);
      }
      else if (op_mode == OP_POSITION)
      {
        desired_position = fmod(present_position + 200, 4096);
        dxl.setGoalPosition(DXL_ID, desired_position);
      }
    }
    else if (inChar == 's')
    {
      DEBUG_SERIAL.println("moving down");
      if (op_mode == OP_PWM)
      {
        desired_pwm = fmod(present_pwm - 100, 885);
        dxl.setGoalPWM(DXL_ID, desired_pwm);
      }
      else if (op_mode == OP_POSITION)
      {
        desired_position = fmod(present_position - 200, 4096);
        dxl.setGoalPosition(DXL_ID, desired_position);
      }
    }
    else if (inChar == 'q')
    {
      DEBUG_SERIAL.println("switching mode");
      // switch mode
      if (op_mode == OP_PWM)
      {
        dxl.torqueOff(DXL_ID);
        dxl.setOperatingMode(DXL_ID, OP_POSITION);
        dxl.torqueOn(DXL_ID);
        op_mode = OP_POSITION;
      }
      else if (op_mode == OP_POSITION)
      {
        dxl.torqueOff(DXL_ID);
        dxl.setOperatingMode(DXL_ID, OP_PWM);
        dxl.torqueOn(DXL_ID);
        op_mode = OP_PWM;
      }
    }
    else if (inChar == 'r')
    {
      desired_position = 2048;
      DEBUG_SERIAL.println("resetting");
      dxl.torqueOff(DXL_ID);
      dxl.setOperatingMode(DXL_ID, OP_POSITION);
      dxl.torqueOn(DXL_ID);
      op_mode = OP_POSITION;
      dxl.setGoalPosition(DXL_ID, desired_position);
    }
    else
    {
      DEBUG_SERIAL.println("Unknown command");
    }
  }
}

void setup()
{
  // put your setup code here, to run once:

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information, detects servo model number
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  // dxl.torqueOff(DXL_ID);
  // dxl.setOperatingMode(DXL_ID, OP_PWM);
  // op_mode = OP_PWM;
  // dxl.torqueOn(DXL_ID);

  last_time = millis();
}

void servo_loop()
{
}

void loop()
{
  // dxl.setGoalPWM(DXL_ID, 300);
  // delay(250);
  // // DEBUG_SERIAL.print("Present PWM(raw) : ");
  // DEBUG_SERIAL.println(dxl.getPresentPWM(DXL_ID));
  // delay(250);

  // dxl.setGoalPWM(DXL_ID, -40.8, UNIT_PERCENT);
  // // DEBUG_SERIAL.print("Present PWM(ratio) : ");
  // delay(250);
  // DEBUG_SERIAL.println(dxl.getPresentPWM(DXL_ID, UNIT_PERCENT));
  // delay(250);

  process_serial_cmd();

  if (last_time + 200 < millis())
  {
    present_position = dxl.getPresentPosition(DXL_ID, UNIT_RAW);
    present_velocity = dxl.getPresentVelocity(DXL_ID, UNIT_RAW);
    present_current = dxl.getPresentCurrent(DXL_ID, UNIT_RAW);
    present_pwm = dxl.getPresentPWM(DXL_ID, UNIT_RAW);
    present_load = dxl.readControlTableItem(PRESENT_LOAD, DXL_ID);

    last_time = millis();
    DEBUG_SERIAL.println("=======================================================");
    DEBUG_SERIAL.println("Dynamixel Status");
    DEBUG_SERIAL.println("=======================================================");
    DEBUG_SERIAL.printf("desired_position=%f, present_position=%f\r\n", desired_position, present_position);
    DEBUG_SERIAL.printf("desired_pwm=%f, present_pwm=%f\r\n", desired_pwm, present_pwm);
    DEBUG_SERIAL.printf("present_velocity=%f\r\n", present_velocity);
    DEBUG_SERIAL.printf("present_load=%f\r\n", present_load);
    DEBUG_SERIAL.printf("present_current=%f\r\n", present_current);
  }
}

#endif // COMPILE_CFG
