#include "common.h"
#if COMPILE_CFG == -1

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

// ------------------ Dynamixel ------------------
#define DXL_SERIAL Serial4
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

const uint8_t DXL_ID = 24;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup()
{
  // put your setup code here, to run once:

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_PWM);
  dxl.torqueOn(DXL_ID);
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
  
  dxl.setGoalPWM(DXL_ID, 300);

  float present_position = dxl.getPresentPosition(DXL_ID, UNIT_RAW);
  float present_velocity = dxl.getPresentVelocity(DXL_ID, UNIT_RAW);
  float present_current = dxl.getPresentCurrent(DXL_ID, UNIT_RAW);

  DEBUG_SERIAL.printf("Current position: %f\r\n", present_position);
  DEBUG_SERIAL.printf("Current velocity: %f rpm\r\n", present_velocity);
  DEBUG_SERIAL.printf("Current current: %f mA\r\n", present_current);
}

#endif // COMPILE_CFG
