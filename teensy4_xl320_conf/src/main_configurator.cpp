#include "common.h"
#if COMPILE_CFG == 2

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

volatile int app_choice = 0;

const uint8_t BROADCAST_ID = 0xFE;
const float DXL_PROTOCOL_VERSION = 2.0;
// uint32_t baud_rates[] = {9600, 57600, 115200, 1000000, 2000000, 3000000};
uint32_t baud_rates[] = {57600};
size_t num_baud_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);
// uint8_t target_id = BROADCAST_ID;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ------------------- sync_read_app -------------------
// const uint8_t DXL_ID_CNT = 12;
// const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {11, 12, 13, 14, 15, 16, 21, 22, 23, 24, 25, 26};
const uint8_t DXL_ID_CNT = 3;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {11, 12, 13};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 4;
// Starting address of the Data to write; Goal Position = 116
const uint16_t SW_START_ADDR = 116;
// Length of the Data to write; Length of Position data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data
{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data
{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

int32_t goal_position[2] = {1024, 2048};
uint8_t goal_position_index = 0;
// ------------------- END sync_read_app -------------------

// ------------------- set_id_app -------------------
uint8_t new_id;

float servo_setpoint_raw = 0;
float servo_setpoint_deg = 0;
float servo_pos_raw = 0;
float servo_pos_deg = 0;
// ------------------- END set_id_app -------------------

// ------------------- mass_scan_app -------------------
struct DeviceInfo
{
  int id;
  int modelNumber;
  uint32_t baudRate;
};

const int MAX_DEVICES = 32; // Maximum number of devices you expect to find
DeviceInfo foundDevices[MAX_DEVICES];
int foundDeviceCount = 0;
// ------------------- END mass_scan_app -------------------

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

void mass_scan_app_setup()
{
  for (size_t i = 0; i < num_baud_rates; i++)
  {
    uint32_t baud_rate = baud_rates[i];
    dxl.begin(baud_rate);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    DEBUG_SERIAL.print("Trying baud rate: ");
    DEBUG_SERIAL.println(baud_rate);

    for (int id = 0; id < DXL_BROADCAST_ID; id++)
    {
      // Iterate until all IDs in each baud rate are scanned.
      if (dxl.ping(id))
      {
        int modelNumber = dxl.getModelNumber(id);
        DEBUG_SERIAL.print("ID : ");
        DEBUG_SERIAL.print(id);
        DEBUG_SERIAL.print(", Model Number: ");
        DEBUG_SERIAL.print(modelNumber);
        DEBUG_SERIAL.print(", Baud Rate: ");
        DEBUG_SERIAL.println(baud_rate);

        // Save the found device info
        if (foundDeviceCount < MAX_DEVICES)
        {
          foundDevices[foundDeviceCount].id = id;
          foundDevices[foundDeviceCount].modelNumber = modelNumber;
          foundDevices[foundDeviceCount].baudRate = baud_rate;
          foundDeviceCount++;
        }
        else
        {
          DEBUG_SERIAL.println("Max device limit reached!");
        }
      }
    }
  }

  DEBUG_SERIAL.print("Found ");
  DEBUG_SERIAL.print(foundDeviceCount);
  DEBUG_SERIAL.println(" devices.");
}

void mass_scan_app_loop()
{
  DEBUG_SERIAL.println("Found devices:");
  for (int i = 0; i < foundDeviceCount; i++)
  {
    DEBUG_SERIAL.print("ID: ");
    DEBUG_SERIAL.print(foundDevices[i].id);
    DEBUG_SERIAL.print(", Model Number: ");
    DEBUG_SERIAL.print(foundDevices[i].modelNumber);
    DEBUG_SERIAL.print(", Baud Rate: ");
    DEBUG_SERIAL.println(foundDevices[i].baudRate);

    dxl.torqueOn(foundDevices[i].id);
    dxl.setGoalPosition(foundDevices[i].id, 2048, UNIT_RAW);
  }
}

void factory_reset_app_setup()
{
  for (size_t i = 0; i < num_baud_rates; i++)
  {
    uint32_t baud_rate = baud_rates[i];
    dxl.begin(baud_rate);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    // Factory reset whole bus
    for (uint8_t target_id = 0; target_id < DXL_BROADCAST_ID; target_id++)
    {
      if (dxl.ping(target_id) == true)
      {
        int ret = dxl.factoryReset(target_id, 0xFF, TIMEOUT);
        DEBUG_SERIAL.printf("Factory Reset target_id=%d, ret=%d\r\n", target_id, ret);
      }
    }
  }
}

void set_id_app_setup()
{
  int target_id = 0;

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

    // Ping all IDs until the target ID is found
    for (target_id = 0; target_id < DXL_BROADCAST_ID; target_id++)
    {
      if (dxl.ping(target_id) == true)
      {
        DEBUG_SERIAL.printf("PING SUCCESS target_id=%d, baud_rate=%d, model_num=%d\r\n",
                            target_id, baud_rate, dxl.getModelNumber(target_id));

        // Turn off torque when configuring items in EEPROM area
        dxl.torqueOff(target_id);

        // Set a new ID for DYNAMIXEL. Do not use ID 200
        if (dxl.setID(target_id, new_id) == true)
        {
          DEBUG_SERIAL.printf("setID SUCCESS, target_id=%d, new_id=%d\r\n", target_id, new_id);
          // Try to ping the new ID
          if (dxl.ping(new_id))
          {
            DEBUG_SERIAL.printf("Can PING new_id, target_id=%d, new_id=%d\r\n", target_id, new_id);
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
  DEBUG_SERIAL.printf("DYNAMIXEL Detected! ret=%d, model_num_from_read=%d, ID=%d, model_num_from_table=%d\r\n",
                      ret, model_num_from_read, target_id, model_num_from_table);

  ret = dxl.setModelNumber(target_id, XL430_W250);
  model_num_from_read = dxl.getModelNumber(target_id);
  model_num_from_table = dxl.getModelNumberFromTable(target_id);
  uint32_t baud_rate = dxl.p_dxl_port_->getBaud();
  DEBUG_SERIAL.printf("DYNAMIXEL Detected! ret=%d, baud_rate=%d, model_num_from_read=%d, ID=%d, model_num_from_table=%d, model_number_idx_[target_id]=%d\r\n",
                      ret, baud_rate, model_num_from_read, target_id, model_num_from_table, dxl.model_number_idx_[target_id]);

  dxl.torqueOn(new_id);
}

void set_id_2x_app_setup()
{
  const uint8_t target_id[2] = {1, 2};
  const uint8_t target_id_cnt = sizeof(target_id) / sizeof(target_id[0]);
  uint8_t new_id[2];

  // Prompt the user for the ID of SERVO A
  DEBUG_SERIAL.println("Enter new ID for SERVO A:");
  new_id[0] = readSerialInput();
  DEBUG_SERIAL.printf("New ID entered: %d\r\n", new_id[0]);

  // Prompt the user for the ID of SERVO B
  DEBUG_SERIAL.println("Enter new ID for SERVO B:");
  new_id[1] = readSerialInput();
  DEBUG_SERIAL.printf("New ID entered: %d\r\n", new_id[1]);

  // new_id[0] = 22;
  // new_id[1] = 23;
  for (size_t i = 0; i < num_baud_rates; i++)
  {
    uint32_t baud_rate = baud_rates[i];
    dxl.begin(baud_rate);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    DEBUG_SERIAL.print("Trying baud rate: ");
    DEBUG_SERIAL.println(baud_rate);

    for (int i = 0; i < target_id_cnt; i++)
    {
      if (dxl.ping(target_id[i]) == true)
      {
        DEBUG_SERIAL.printf("PING SUCCESS target_id=%d, baud_rate=%d, model_num=%d\r\n",
                            target_id[i], baud_rate, dxl.getModelNumber(target_id[i]));

        // Turn off torque when configuring items in EEPROM area
        dxl.torqueOff(target_id[i]);

        // Set a new ID for DYNAMIXEL. Do not use ID 200
        if (dxl.setID(target_id[i], new_id[i]) == true)
        {
          DEBUG_SERIAL.printf("setID SUCCESS, target_id=%d, new_id=%d\r\n", target_id[i], new_id[i]);
          if (dxl.ping(new_id[i]))
          {
            DEBUG_SERIAL.printf("Can PING new_id, target_id=%d, new_id=%d\r\n", target_id[i], new_id[i]);
            DEBUG_SERIAL.println(new_id[i]);
            DEBUG_SERIAL.print(", Model Number: ");
            DEBUG_SERIAL.println(dxl.getModelNumber(new_id[i]));
          }
        }
        else
        {
          DEBUG_SERIAL.printf("Failed to change target_id=%d to new_id=%d\r\n", target_id[i], new_id[i]);
        }
      }
      else
      {
        DEBUG_SERIAL.printf("PING FAILED, target_id=%d\r\n", target_id[i]);
      }
    }
  }
}

void set_id_app_loop()
{
  float current_pos_raw = dxl.getPresentPosition(new_id, UNIT_RAW);
  float current_pos_deg = dxl.getPresentPosition(new_id, UNIT_DEGREE);
  DEBUG_SERIAL.printf("new_id=%d, current_pos_raw=%f, current_pos_deg=%f\r\n",
                      new_id, current_pos_raw, current_pos_deg);

  // Turn on the LED on DYNAMIXEL
  DEBUG_SERIAL.println("LED ON at 210 deg...");
  dxl.setGoalPosition(new_id, 210.0, UNIT_DEGREE);
  dxl.ledOn(new_id);
  delay(500);

  // Turn off the LED on DYNAMIXEL
  DEBUG_SERIAL.println("LED OFF at 180 deg...");
  dxl.setGoalPosition(new_id, 180.0, UNIT_DEGREE);
  dxl.ledOff(new_id);
  delay(500);
}

void sync_read_app_setup()
{
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Prepare the SyncRead structure
  for (i = 0; i < DXL_ID_CNT; i++)
  {
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
  }
  dxl.torqueOn(BROADCAST_ID);

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;

  for (i = 0; i < DXL_ID_CNT; i++)
  {
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t *)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  for (i = 0; i < DXL_ID_CNT; i++)
  {
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t *)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;
}

void sync_read_app_loop()
{
  static uint32_t try_count = 0;
  uint8_t i, recv_cnt;

  // Insert a new Goal Position to the SyncWrite Packet
  for (i = 0; i < DXL_ID_CNT; i++)
  {
    sw_data[i].goal_position = goal_position[goal_position_index];
  }

  // Update the SyncWrite packet status
  sw_infos.is_info_changed = true;

  DEBUG_SERIAL.print("\n>>>>>> Sync Instruction Test : ");
  DEBUG_SERIAL.println(try_count++);

  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  if (dxl.syncWrite(&sw_infos) == true)
  {
    DEBUG_SERIAL.println("[SyncWrite] Success");
    for (i = 0; i < sw_infos.xel_count; i++)
    {
      DEBUG_SERIAL.print("  ID: ");
      DEBUG_SERIAL.println(sw_infos.p_xels[i].id);
      DEBUG_SERIAL.print("\t Goal Position: ");
      DEBUG_SERIAL.println(sw_data[i].goal_position);
    }
    if (goal_position_index == 0)
      goal_position_index = 1;
    else
      goal_position_index = 0;
  }
  else
  {
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println();

  delay(250);

  // Transmit predefined SyncRead instruction packet
  // and receive a status packet from each DYNAMIXEL
  recv_cnt = dxl.syncRead(&sr_infos);
  if (recv_cnt > 0)
  {
    DEBUG_SERIAL.print("[SyncRead] Success, Received ID Count: ");
    DEBUG_SERIAL.println(recv_cnt);
    for (i = 0; i < recv_cnt; i++)
    {
      DEBUG_SERIAL.print("  ID: ");
      DEBUG_SERIAL.print(sr_infos.p_xels[i].id);
      DEBUG_SERIAL.print(", Error: ");
      DEBUG_SERIAL.println(sr_infos.p_xels[i].error);
      DEBUG_SERIAL.print("\t Present Position: ");
      DEBUG_SERIAL.println(sr_data[i].present_position);
    }
  }
  else
  {
    DEBUG_SERIAL.print("[SyncRead] Fail, Lib error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println("=======================================================");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(750);
}

void setup()
{
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL)
    ;

  DEBUG_SERIAL.printf("App choice\r\n");
  DEBUG_SERIAL.printf("\t0: mass_scan_app, also sets the servos to default position\r\n");
  DEBUG_SERIAL.printf("\t1: set_id_app, for single servo only\r\n");
  DEBUG_SERIAL.printf("\t2: sync_read_app, test 2 servos sync read write\r\n");
  DEBUG_SERIAL.printf("\t3: factory_reset_app, factory reset config for the whole bus\r\n");
  DEBUG_SERIAL.printf("\t4: set_id_2x_app, for 2XL430\r\n");

  app_choice = readSerialInput();
  switch (app_choice)
  {
  case 0:
    mass_scan_app_setup();
    break;
  case 1:
    set_id_app_setup();
    break;
  case 2:
    sync_read_app_setup();
    break;
  case 3:
    factory_reset_app_setup();
  case 4:
    set_id_2x_app_setup();
  }
}

void loop()
{
  switch (app_choice)
  {
  case 0:
    mass_scan_app_loop();
    break;
  case 1:
    set_id_app_loop();
    break;
  case 2:
    sync_read_app_loop();
    break;
  case 3:
    delay(1000);
    DEBUG_SERIAL.println("factory_reset_app");
  case 4:
    delay(1000);
    DEBUG_SERIAL.println("set_id_2x_app");
    break;
  }
}
#endif // COMPILE_CFG