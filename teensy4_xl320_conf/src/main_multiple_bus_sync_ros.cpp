#include "common.h"
#if COMPILE_CFG == 3

#include <Arduino.h>
#include "cmd_utils.hpp"

#include <Dynamixel2Arduino.h>

#include <unordered_map>
#include <string>
#include <vector>
#include <optional>

// ROS library
#include "ros.h"
#include "ServoCommand.h"
#include "ServoFeedback.h"
#include "ros_helpers.h"

// ------------------ ROS ------------------
ros::NodeHandle nh;
void servo_cmd_cb(const humanoid_msgs::ServoCommand &input_msg);

humanoid_msgs::ServoFeedback servo_fb_msg;
ros::Publisher servo_fb_pub("servosFeedback", &servo_fb_msg);
ros::Subscriber<humanoid_msgs::ServoCommand> servo_cmd_sub("servosCommand", servo_cmd_cb);

// Define the joints and their corresponding Dynamixel IDs and goal positions
struct Joint
{
  uint8_t id;
  int32_t goal_position_raw;
  int32_t current_position_raw;
};

// Map for joints with nullopt for current_position_raw
std::unordered_map<std::string, Joint> joints_map = {
    {"right_shoulder_pitch", {21, 2048, INT32_MIN}},
    {"right_shoulder_roll", {22, 2048, INT32_MIN}},
    {"right_elbow", {23, 2048, INT32_MIN}},
    {"left_shoulder_pitch", {11, 2048, INT32_MIN}},
    {"left_shoulder_roll", {12, 2048, INT32_MIN}},
    {"left_elbow", {13, 2048, INT32_MIN}},
    {"left_hip_roll", {14, 2048, INT32_MIN}},
    {"left_hip_pitch", {15, 2048, INT32_MIN}},
    {"left_knee", {16, 2048, INT32_MIN}},
    {"right_hip_roll", {24, 2048, INT32_MIN}},
    {"right_hip_pitch", {25, 2048, INT32_MIN}},
    {"right_knee", {26, 2048, INT32_MIN}},
};
std::unordered_map<uint8_t, std::string> joint_id_to_name;

// Function to populate the reverse lookup map
void populate_joint_id_to_name()
{
  for (const auto &joint : joints_map)
  {
    joint_id_to_name[joint.second.id] = joint.first;
  }
}

// ------------------ Dynamixel ------------------
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

// Update sw_data with the goal positions from the ROS message
void servo_cmd_cb(const humanoid_msgs::ServoCommand &input_msg)
{
  DEBUG_SERIAL.printf("%s() start\r\n", __func__);
  std::unordered_map<std::string, int32_t> joint_positions = {
      {"right_shoulder_pitch", input_msg.right_shoulder_pitch},
      {"right_shoulder_roll", input_msg.right_shoulder_roll},
      {"right_elbow", input_msg.right_elbow},
      {"left_shoulder_pitch", input_msg.left_shoulder_pitch},
      {"left_shoulder_roll", input_msg.left_shoulder_roll},
      {"left_elbow", input_msg.left_elbow},
      {"left_hip_roll", input_msg.left_hip_roll},
      {"left_hip_pitch", input_msg.left_hip_pitch},
      {"left_knee", input_msg.left_knee},
      {"right_hip_roll", input_msg.right_hip_roll},
      {"right_hip_pitch", input_msg.right_hip_pitch},
      {"right_knee", input_msg.right_knee}};

  // TODO apply the goal positions to the sw_data
  // uint8_t index = 0;
  // for (const auto &joint : joint_positions)
  // {
  //   auto it = joints_map.find(joint.first);
  //   if (it != joints_map.end())
  //   {
  //     it->second.goal_position_raw = deg2raw(joint.second);
  //     sw_data[index].goal_position_raw = deg2raw(joint.second);
  //   }
  //   else
  //   {
  //     DEBUG_SERIAL.printf("Joint not found: %s\r\n", joint.first.c_str());
  //   }
  //   index++;
  // }
  DEBUG_SERIAL.printf("%s() end\r\n", __func__);
}

void ros_setup()
{
  populate_joint_id_to_name();

  DEBUG_SERIAL.printf("%s() start\r\n", __func__);
  nh.initNode();

  nh.advertise(servo_fb_pub);
  nh.subscribe(servo_cmd_sub);

  nh.negotiateTopics();
  while (!nh.connected())
  {
    nh.negotiateTopics();
    DEBUG_SERIAL.printf("Waiting for ROS connection...\r\n"); // Debugging line
  }

  servo_fb_msg.right_shoulder_pitch_length = 3;
  servo_fb_msg.right_shoulder_roll_length = 3;
  servo_fb_msg.right_elbow_length = 3;
  servo_fb_msg.left_shoulder_pitch_length = 3;
  servo_fb_msg.left_shoulder_roll_length = 3;
  servo_fb_msg.left_elbow_length = 3;
  servo_fb_msg.left_hip_roll_length = 3;
  servo_fb_msg.left_hip_pitch_length = 3;
  servo_fb_msg.left_knee_length = 3;
  servo_fb_msg.right_hip_roll_length = 3;
  servo_fb_msg.right_hip_pitch_length = 3;
  servo_fb_msg.right_knee_length = 3;

  DEBUG_SERIAL.printf("%s() end\r\n", __func__);
}

void process_bus_feedback(DynamixelBusConfig_t *bus_config)
{
  for (size_t i = 0; i < bus_config->id_count; i++)
  {
    uint8_t id = bus_config->info_xels_sr[i].id;
    auto it = joint_id_to_name.find(id);
    if (it != joint_id_to_name.end())
    {
      std::string joint_name = it->second;
      float feedback[3] = {
          static_cast<float>(bus_config->sr_data[i].present_position),
          static_cast<float>(0.0),
          static_cast<float>(0.0),
      };

      // Map the feedback to the corresponding message fields
      if (joint_name == "right_shoulder_pitch")
        servo_fb_msg.right_shoulder_pitch = feedback;
      else if (joint_name == "right_shoulder_roll")
        servo_fb_msg.right_shoulder_roll = feedback;
      else if (joint_name == "right_elbow")
        servo_fb_msg.right_elbow = feedback;
      else if (joint_name == "left_shoulder_pitch")
        servo_fb_msg.left_shoulder_pitch = feedback;
      else if (joint_name == "left_shoulder_roll")
        servo_fb_msg.left_shoulder_roll = feedback;
      else if (joint_name == "left_elbow")
        servo_fb_msg.left_elbow = feedback;
      else if (joint_name == "left_hip_roll")
        servo_fb_msg.left_hip_roll = feedback;
      else if (joint_name == "left_hip_pitch")
        servo_fb_msg.left_hip_pitch = feedback;
      else if (joint_name == "left_knee")
        servo_fb_msg.left_knee = feedback;
      else if (joint_name == "right_hip_roll")
        servo_fb_msg.right_hip_roll = feedback;
      else if (joint_name == "right_hip_pitch")
        servo_fb_msg.right_hip_pitch = feedback;
      else if (joint_name == "right_knee")
        servo_fb_msg.right_knee = feedback;
    }
    else
    {
      DEBUG_SERIAL.printf("Joint ID %d not found in joint_id_to_name map.\n", id);
    }
  }
}

// Publish servo feedback
void ros_loop()
{
  nh.spinOnce();

  process_bus_feedback(&bus1);
  process_bus_feedback(&bus2);
  process_bus_feedback(&bus3);
  process_bus_feedback(&bus4);

  servo_fb_pub.publish(&servo_fb_msg);
}

void setup()
{
  DEBUG_SERIAL.begin(115200);
  // while (!DEBUG_SERIAL)
  //   ;

  DEBUG_SERIAL.printf("Multiple bus sync read write app\n");

  sync_read_app_setup(&bus1);
  sync_read_app_setup(&bus2);
  sync_read_app_setup(&bus3);
  sync_read_app_setup(&bus4);

  ros_setup();
}

void loop()
{
  sync_read_app_loop(&bus1);
  sync_read_app_loop(&bus2);
  sync_read_app_loop(&bus3);
  sync_read_app_loop(&bus4);
  goal_position_index = (goal_position_index == 0) ? 1 : 0;

  ros_loop();
}
#endif // COMPILE_CFG