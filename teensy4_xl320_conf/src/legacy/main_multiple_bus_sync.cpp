#include "common.h"
#if COMPILE_CFG == 0

#include <Arduino.h>
#include "cmd_utils.hpp"

#include <Dynamixel2Arduino.h>

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

// ------------------- sync_read_app -------------------
const uint8_t DXL_ID_CNT = 3;
const uint16_t user_pkt_buf_cap = 128;

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

typedef struct
{
  Dynamixel2Arduino dxl;       // Dynamixel2Arduino object
  uint8_t id_list[DXL_ID_CNT]; // List of Dynamixel IDs
  size_t id_count;             // Number of IDs

  // SyncRead related data
  sr_data_t sr_data[DXL_ID_CNT];                         // SyncRead data array
  DYNAMIXEL::InfoSyncReadInst_t sr_infos;                // SyncRead information structure
  DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT]; // SyncRead XEL info array

  // SyncWrite related data
  sw_data_t sw_data[DXL_ID_CNT];                          // SyncWrite data array
  DYNAMIXEL::InfoSyncWriteInst_t sw_infos;                // SyncWrite information structure
  DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT]; // SyncWrite XEL info array

  uint8_t user_pkt_buf[user_pkt_buf_cap]; // User packet buffer
} DynamixelBusConfig_t;

DynamixelBusConfig_t bus1 = {
    .dxl = Dynamixel2Arduino(Serial1, DXL_DIR_PIN), // Dynamixel2Arduino object for Serial1
    .id_list = {11, 12, 13},                        // IDs of Dynamixel motors
    .id_count = DXL_ID_CNT,                         // Number of IDs in the list
    .sr_data = {},                                  // Initialize SyncRead data to zero
    .sr_infos = {},                                 // Initialize SyncRead info structure
    .info_xels_sr = {},                             // Initialize SyncRead XEL info array
    .sw_data = {},                                  // Initialize SyncWrite data to zero
    .sw_infos = {},                                 // Initialize SyncWrite info structure
    .info_xels_sw = {},                             // Initialize SyncWrite XEL info array
    .user_pkt_buf = {0}                             // Initialize user packet buffer to zero
};

DynamixelBusConfig_t bus2 = {
    .dxl = Dynamixel2Arduino(Serial2, DXL_DIR_PIN), // Dynamixel2Arduino object for Serial2
    .id_list = {21, 22, 23},                        // Adjust IDs for bus2
    .id_count = DXL_ID_CNT,                         // Number of IDs in the list
    .sr_data = {},                                  // Initialize SyncRead data to zero
    .sr_infos = {},                                 // Initialize SyncRead info structure
    .info_xels_sr = {},                             // Initialize SyncRead XEL info array
    .sw_data = {},                                  // Initialize SyncWrite data to zero
    .sw_infos = {},                                 // Initialize SyncWrite info structure
    .info_xels_sw = {},                             // Initialize SyncWrite XEL info array
    .user_pkt_buf = {0}                             // Initialize user packet buffer to zero
};

DynamixelBusConfig_t bus3 = {
    .dxl = Dynamixel2Arduino(Serial3, DXL_DIR_PIN), // Dynamixel2Arduino object for Serial2
    .id_list = {14, 15, 16},                        // Adjust IDs for bus2
    .id_count = DXL_ID_CNT,                         // Number of IDs in the list
    .sr_data = {},                                  // Initialize SyncRead data to zero
    .sr_infos = {},                                 // Initialize SyncRead info structure
    .info_xels_sr = {},                             // Initialize SyncRead XEL info array
    .sw_data = {},                                  // Initialize SyncWrite data to zero
    .sw_infos = {},                                 // Initialize SyncWrite info structure
    .info_xels_sw = {},                             // Initialize SyncWrite XEL info array
    .user_pkt_buf = {0}                             // Initialize user packet buffer to zero
};

DynamixelBusConfig_t bus4 = {
    .dxl = Dynamixel2Arduino(Serial4, DXL_DIR_PIN), // Dynamixel2Arduino object for Serial2
    .id_list = {24, 25, 26},                        // Adjust IDs for bus2
    .id_count = DXL_ID_CNT,                         // Number of IDs in the list
    .sr_data = {},                                  // Initialize SyncRead data to zero
    .sr_infos = {},                                 // Initialize SyncRead info structure
    .info_xels_sr = {},                             // Initialize SyncRead XEL info array
    .sw_data = {},                                  // Initialize SyncWrite data to zero
    .sw_infos = {},                                 // Initialize SyncWrite info structure
    .info_xels_sw = {},                             // Initialize SyncWrite XEL info array
    .user_pkt_buf = {0}                             // Initialize user packet buffer to zero
};

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

void sync_read_app_setup(DynamixelBusConfig_t *config)
{
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize the Dynamixel bus
  config->dxl.begin(57600);
  config->dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Prepare the SyncRead structure
  for (size_t i = 0; i < config->id_count; i++)
  {
    config->dxl.torqueOff(config->id_list[i]);
    config->dxl.setOperatingMode(config->id_list[i], OP_POSITION); // Assuming OP_POSITION is defined somewhere
  }
  config->dxl.torqueOn(BROADCAST_ID);

  // Fill the members of structure for SyncRead using external user packet buffer
  config->sr_infos.packet.p_buf = config->user_pkt_buf;
  config->sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  config->sr_infos.packet.is_completed = false;
  config->sr_infos.addr = SR_START_ADDR;
  config->sr_infos.addr_length = SR_ADDR_LEN;
  config->sr_infos.p_xels = config->info_xels_sr;
  config->sr_infos.xel_count = 0;

  // Configure SyncRead XELs
  for (size_t i = 0; i < config->id_count; i++)
  {
    config->info_xels_sr[i].id = config->id_list[i];
    config->info_xels_sr[i].p_recv_buf = (uint8_t *)&config->sr_data[i];
    config->sr_infos.xel_count++;
  }
  config->sr_infos.is_info_changed = true;

  // Fill the members of structure for SyncWrite using internal packet buffer
  config->sw_infos.packet.p_buf = nullptr;
  config->sw_infos.packet.is_completed = false;
  config->sw_infos.addr = SW_START_ADDR;
  config->sw_infos.addr_length = SW_ADDR_LEN;
  config->sw_infos.p_xels = config->info_xels_sw;
  config->sw_infos.xel_count = 0;

  // Configure SyncWrite XELs
  for (size_t i = 0; i < config->id_count; i++)
  {
    config->info_xels_sw[i].id = config->id_list[i];
    config->info_xels_sw[i].p_data = (uint8_t *)&config->sw_data[i].goal_position;
    config->sw_infos.xel_count++;
  }
  config->sw_infos.is_info_changed = true;

  for (size_t i = 0; i < config->id_count; i++)
  {
    config->dxl.torqueOn(config->id_list[i]);
  }
}

void sync_read_app_loop(DynamixelBusConfig_t *config)
{
  static uint32_t try_count = 0;
  uint8_t recv_cnt;

  // Insert a new Goal Position to the SyncWrite Packet
  for (size_t i = 0; i < config->id_count; i++)
  {
    config->sw_data[i].goal_position = goal_position[goal_position_index];
  }
  DEBUG_SERIAL.printf("current goal_position %d\n", config->sw_data[0].goal_position);

  // Update the SyncWrite packet status
  config->sw_infos.is_info_changed = true;

  DEBUG_SERIAL.print("\n>>>>>> Sync Instruction Test : ");
  DEBUG_SERIAL.println(try_count++);

  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  if (config->dxl.syncWrite(&config->sw_infos) == true)
  {
    DEBUG_SERIAL.println("[SyncWrite] Success");
    for (size_t i = 0; i < config->sw_infos.xel_count; i++)
    {
      DEBUG_SERIAL.print("  ID: ");
      DEBUG_SERIAL.println(config->sw_infos.p_xels[i].id);
      DEBUG_SERIAL.print("\t Goal Position: ");
      DEBUG_SERIAL.println(config->sw_data[i].goal_position);
    }
    // goal_position_index = (goal_position_index == 0) ? 1 : 0;
  }
  else
  {
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(config->dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println();

  delay(50);

  // Transmit predefined SyncRead instruction packet
  // and receive a status packet from each DYNAMIXEL
  recv_cnt = config->dxl.syncRead(&config->sr_infos);
  if (recv_cnt > 0)
  {
    DEBUG_SERIAL.print("[SyncRead] Success, Received ID Count: ");
    DEBUG_SERIAL.println(recv_cnt);
    for (size_t i = 0; i < recv_cnt; i++)
    {
      DEBUG_SERIAL.print("  ID: ");
      DEBUG_SERIAL.print(config->sr_infos.p_xels[i].id);
      DEBUG_SERIAL.print(", Error: ");
      DEBUG_SERIAL.println(config->sr_infos.p_xels[i].error);
      DEBUG_SERIAL.print("\t Present Position: ");
      DEBUG_SERIAL.println(config->sr_data[i].present_position);
    }
  }
  else
  {
    DEBUG_SERIAL.print("[SyncRead] Fail, Lib error code: ");
    DEBUG_SERIAL.println(config->dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println("=======================================================");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(50);
}

void setup()
{
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL)
    ;

  DEBUG_SERIAL.printf("Multiple bus sync read write app\n");

  sync_read_app_setup(&bus1);
  sync_read_app_setup(&bus2);
  sync_read_app_setup(&bus3);
  sync_read_app_setup(&bus4);
}

void loop()
{
  sync_read_app_loop(&bus1);
  sync_read_app_loop(&bus2);
  sync_read_app_loop(&bus3);
  sync_read_app_loop(&bus4);
  goal_position_index = (goal_position_index == 0) ? 1 : 0;
}
#endif // COMPILE_CFG