#include <Arduino.h>
#include "cmd_utils.hpp"

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL SerialUSB
// const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
const int DXL_DIR_PIN = -1; // DYNAMIXEL Shield DIR PIN

#define TIMEOUT 10 // default communication timeout 10ms
#define MODEL_NUMBER_ADDR 0
#define MODEL_NUMBER_LENGTH 2

// const uint8_t DEFAULT_DXL_ID = 1;
const uint8_t DEFAULT_DXL_ID = 0xFE;
const float DXL_PROTOCOL_VERSION = 2.0;
// uint32_t baud_rates[] = {9600, 57600, 115200, 1000000, 2000000, 3000000};
uint32_t baud_rates[] = {57600};
size_t num_baud_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);
uint8_t target_id = DEFAULT_DXL_ID;
uint8_t new_id = 101;

float servo_setpoint_raw = 0;
float servo_setpoint_deg = 0;
float servo_pos_raw = 0;
float servo_pos_deg = 0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// This namespace is required to use Control table item names
using namespace ControlTableItem;

// Read a number from the serial port
int readSerialInput()
{
  // Clear the serial buffer
  while (SerialUSB.available() > 0)
  {
    SerialUSB.read();
  }
  while (true)
  {
    if (SerialUSB.available() > 0)
    {
      String input = SerialUSB.readStringUntil('\n');
      input.trim(); // Remove any leading or trailing whitespace
      if (input.length() > 0)
      {
        return input.toInt();
      }
    }
  }
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

void mass_scan()
{
  uint32_t baud_rates[] = {9600, 57600, 115200, 1000000, 2000000, 3000000};
  for (size_t i = 0; i < num_baud_rates; i++)
  {
    uint32_t baud_rate = baud_rates[i];
    dxl.begin(baud_rate);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    DEBUG_SERIAL.print("Trying baud rate: ");
    DEBUG_SERIAL.println(baud_rate);

    for (int id = 0; id < DXL_BROADCAST_ID; id++)
    {
      // iterate until all ID in each buadrate is scanned.
      if (dxl.ping(id))
      {
        DEBUG_SERIAL.print("ID : ");
        DEBUG_SERIAL.print(id);
        DEBUG_SERIAL.print(", Model Number: ");
        DEBUG_SERIAL.println(dxl.getModelNumber(id));
      }
    }
  }
}

void setup()
{
  // put your setup code here, to run once:

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL)
    ;

  // Prompt the user for the new ID
  DEBUG_SERIAL.println("Enter new ID for the DYNAMIXEL:");
  new_id = readSerialInput();
  DEBUG_SERIAL.printf("New ID entered: %d\n", new_id);

  DEBUG_SERIAL.println("Cycling through baud rates to find DYNAMIXEL...");

  for (size_t i = 0; i < num_baud_rates; i++)
  {
    uint32_t baud_rate = baud_rates[i];
    dxl.begin(baud_rate);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    DEBUG_SERIAL.print("Trying baud rate: ");
    DEBUG_SERIAL.println(baud_rate);

    for (target_id = 0; target_id < DXL_BROADCAST_ID; target_id++)
    {
      if (dxl.ping(target_id) == true)
      {
        DEBUG_SERIAL.printf("PING SUCCESS target_id=%d, baud_rate=%d, model_num=%d\n",
                            target_id, baud_rate, dxl.getModelNumber(target_id));

        // Turn off torque when configuring items in EEPROM area
        dxl.torqueOff(target_id);

        // Set a new ID for DYNAMIXEL. Do not use ID 200
        if (dxl.setID(target_id, new_id) == true)
        {
          DEBUG_SERIAL.printf("setID SUCCESS, target_id=%d, new_id=%d\n", target_id, new_id);
          if (dxl.ping(new_id))
          {
            DEBUG_SERIAL.printf("Can PING new_id, target_id=%d, new_id=%d\n", target_id, new_id);
            DEBUG_SERIAL.println(new_id);
            DEBUG_SERIAL.print(", Model Number: ");
            DEBUG_SERIAL.println(dxl.getModelNumber(new_id));
          }
        }
        else
        {
          DEBUG_SERIAL.print("Failed to change ID to ");
          DEBUG_SERIAL.println(new_id);
        }
        // Exit the loop if ping succeeds
        break;
      }
      else
      {
        DEBUG_SERIAL.printf("PING FAILED, target_id=%d\n", target_id);
      }
    }
  }

  // Check servo info
  uint16_t model_num_from_read = 0;
  uint16_t model_num_from_table = 0;

  model_num_from_read = dxl.getModelNumber(target_id);
  model_num_from_table = dxl.getModelNumberFromTable(target_id);
  int ret = dxl.read(target_id, MODEL_NUMBER_ADDR, MODEL_NUMBER_LENGTH, (uint8_t *)&model_num_from_read, sizeof(model_num_from_read), TIMEOUT);
  DEBUG_SERIAL.printf("DYNAMIXEL Detected! ret=%d, model_num_from_read=%d, ID=%d, model_num_from_table=%d\n",
                      ret, model_num_from_read, target_id, model_num_from_table);

  ret = dxl.setModelNumber(target_id, XL430_W250);
  model_num_from_read = dxl.getModelNumber(target_id);
  model_num_from_table = dxl.getModelNumberFromTable(target_id);
  uint32_t baud_rate = dxl.p_dxl_port_->getBaud();
  DEBUG_SERIAL.printf("DYNAMIXEL Detected! ret=%d, baud_rate=%d, model_num_from_read=%d, ID=%d, model_num_from_table=%d, model_number_idx_[target_id]=%d\n",
                      ret, baud_rate, model_num_from_read, target_id, model_num_from_table, dxl.model_number_idx_[target_id]);

  dxl.torqueOn(new_id);
}

void loop()
{
  float current_pos_raw = dxl.getPresentPosition(new_id, UNIT_RAW);
  float current_pos_deg = dxl.getPresentPosition(new_id, UNIT_DEGREE);
  DEBUG_SERIAL.printf("current_pos_raw=%f, current_pos_deg=%f\n",
                      current_pos_raw, current_pos_deg);

  // Turn on the LED on DYNAMIXEL
  DEBUG_SERIAL.println("LED ON at 210 deg...");
  dxl.setGoalPosition(new_id, 210.0, UNIT_DEGREE);
  dxl.ledOn(new_id);
  delay(500);

  // Turn off the LED on DYNAMIXEL
  DEBUG_SERIAL.println("LED OFF at 180...");
  dxl.setGoalPosition(new_id, 180.0, UNIT_DEGREE);
  dxl.ledOff(new_id);
  delay(500);
}