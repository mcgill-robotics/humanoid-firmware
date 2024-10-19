#include "common.h"
#if COMPILE_CFG == 99

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

// ------------------ Dynamixel ------------------
#define DEBUG_SERIAL Serial5
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
    .id_list = {11, 12, 13},                        // Adjust IDs for bus2
    // .id_list = {11, 12, 13},                        // IDs of Dynamixel motors
    .id_count = DXL_ID_CNT, // Number of IDs in the list
    .sr_data = {},          // Initialize SyncRead data to zero
    .sr_infos = {},         // Initialize SyncRead info structure
    .info_xels_sr = {},     // Initialize SyncRead XEL info array
    .sw_data = {},          // Initialize SyncWrite data to zero
    .sw_infos = {},         // Initialize SyncWrite info structure
    .info_xels_sw = {},     // Initialize SyncWrite XEL info array
    .user_pkt_buf = {0}     // Initialize user packet buffer to zero
};

DynamixelBusConfig_t bus2 = {
    .dxl = Dynamixel2Arduino(Serial2, DXL_DIR_PIN), // Dynamixel2Arduino object for Serial2
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

DynamixelBusConfig_t bus3 = {
    .dxl = Dynamixel2Arduino(Serial3, DXL_DIR_PIN), // Dynamixel2Arduino object for Serial2
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

DynamixelBusConfig_t bus4 = {
    .dxl = Dynamixel2Arduino(Serial4, DXL_DIR_PIN), // Dynamixel2Arduino object for Serial2
    .id_list = {24, 25, 26},                        // IDs of Dynamixel motors
    // .id_list = {24, 25, 26},                        // Adjust IDs for bus2
    .id_count = DXL_ID_CNT, // Number of IDs in the list
    .sr_data = {},          // Initialize SyncRead data to zero
    .sr_infos = {},         // Initialize SyncRead info structure
    .info_xels_sr = {},     // Initialize SyncRead XEL info array
    .sw_data = {},          // Initialize SyncWrite data to zero
    .sw_infos = {},         // Initialize SyncWrite info structure
    .info_xels_sw = {},     // Initialize SyncWrite XEL info array
    .user_pkt_buf = {0}     // Initialize user packet buffer to zero
};

int32_t goal_position[2] = {1024, 2048};
uint8_t goal_position_index = 0;

// Define the joints and their corresponding Dynamixel IDs and goal positions
class Joint
{
public:
  // Constructor
  Joint(const std::string &name, uint8_t id, DynamixelBusConfig_t *bus)
      : name(name), id(id), goal_position_raw(2048), current_position_raw(INT32_MIN), bus(bus)
  {
    // Initialize feedback to zero
    feedback[0] = 0.0f;
    feedback[1] = 0.0f;
    feedback[2] = 0.0f;
  }

  // Members
  std::string name;
  uint8_t id;
  int32_t goal_position_raw;
  int32_t current_position_raw;
  DynamixelBusConfig_t *bus; // Pointer to the associated bus

  // Persistent feedback storage
  float feedback[3];
};

static int32_t degToRaw(float degrees)
{
  return static_cast<int32_t>(degrees * (4095.0 / 360.0));
}

static float rawToDeg(int32_t raw)
{
  return static_cast<float>(raw) * (360.0 / 4095.0);
}

// Declare Joint objects statically
Joint left_knee("left_knee", 11, &bus1);
Joint left_hip_roll("left_hip_roll", 12, &bus1);
Joint left_hip_pitch("left_hip_pitch", 13, &bus1);

Joint left_elbow("left_elbow", 14, &bus2);
Joint left_shoulder_roll("left_shoulder_roll", 15, &bus2);
Joint left_shoulder_pitch("left_shoulder_pitch", 16, &bus2);

Joint right_knee("right_knee", 21, &bus3);
Joint right_hip_roll("right_hip_roll", 22, &bus3);
Joint right_hip_pitch("right_hip_pitch", 23, &bus3);

Joint right_elbow("right_elbow", 24, &bus4);
Joint right_shoulder_roll("right_shoulder_roll", 25, &bus4);
Joint right_shoulder_pitch("right_shoulder_pitch", 26, &bus4);

// Populate joints_map with pointers to these objects
std::unordered_map<std::string, Joint *> joints_map = {
    {"right_shoulder_pitch", &right_shoulder_pitch},
    {"right_shoulder_roll", &right_shoulder_roll},
    {"right_elbow", &right_elbow},
    {"left_shoulder_pitch", &left_shoulder_pitch},
    {"left_shoulder_roll", &left_shoulder_roll},
    {"left_elbow", &left_elbow},
    {"left_hip_roll", &left_hip_roll},
    {"left_hip_pitch", &left_hip_pitch},
    {"left_knee", &left_knee},
    {"right_hip_roll", &right_hip_roll},
    {"right_hip_pitch", &right_hip_pitch},
    {"right_knee", &right_knee}};

std::unordered_map<uint8_t, std::string> joint_id_to_name;

// Function to populate the reverse lookup map
void populate_joint_id_to_name()
{
  for (const auto &pair : joints_map)
  {
    joint_id_to_name[pair.second->id] = pair.first;
  }
}
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
    // Initialize the goal positions to the center
    config->sw_data[i].goal_position = 2048;

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
  // for (size_t i = 0; i < config->id_count; i++)
  // {
  //   config->sw_data[i].goal_position = goal_position[goal_position_index];
  // }
  DEBUG_PRINTF("current goal_position %d\n", config->sw_data[0].goal_position);

  // Update the SyncWrite packet status
  config->sw_infos.is_info_changed = true;

  DEBUG_PRINT("\n>>>>>> Sync Instruction Test : ");
  DEBUG_PRINTLN(try_count++);

  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  if (config->dxl.syncWrite(&config->sw_infos) == true)
  {
    DEBUG_PRINTLN("[SyncWrite] Success");
    for (size_t i = 0; i < config->sw_infos.xel_count; i++)
    {
      DEBUG_PRINTF("\tID=%d, name=%s\r\n",
                   config->info_xels_sw[i].id,
                   joint_id_to_name[config->info_xels_sw[i].id].c_str());

      DEBUG_PRINTF("\t\tgoal_position_raw=%d, goal_position_deg=%f\r\n",
                   config->sw_data[i].goal_position,
                   rawToDeg(config->sw_data[i].goal_position));
    }
    // goal_position_index = (goal_position_index == 0) ? 1 : 0;
  }
  else
  {
    DEBUG_PRINT("[SyncWrite] Fail, Lib error code: ");
    DEBUG_PRINT(config->dxl.getLastLibErrCode());
  }
  DEBUG_PRINTLN();

  // delay(5);

  // Transmit predefined SyncRead instruction packet
  // and receive a status packet from each DYNAMIXEL
  recv_cnt = config->dxl.syncRead(&config->sr_infos);
  if (recv_cnt > 0)
  {
    DEBUG_PRINT("[SyncRead] Success, Received ID Count: ");
    DEBUG_PRINTLN(recv_cnt);
    for (size_t i = 0; i < recv_cnt; i++)
    {
      DEBUG_PRINTF("\tID=%d, name=%s\r\n",
                   config->info_xels_sr[i].id,
                   joint_id_to_name[config->info_xels_sr[i].id].c_str());
      DEBUG_PRINTF("\t\tpresent_position_raw=%d, present_position_deg=%f\r\n",
                   config->sr_data[i].present_position,
                   rawToDeg(config->sr_data[i].present_position));
    }
  }
  else
  {
    DEBUG_PRINT("[SyncRead] Fail, Lib error code: ");
    DEBUG_PRINTLN(config->dxl.getLastLibErrCode());
  }
  DEBUG_PRINTLN("=======================================================");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  // delay(5);
}

// Map of joint names to their desired positions from the ROS message
std::unordered_map<std::string, int32_t> joint_initial_positions = {
    {"left_knee", 180.0f},
    {"left_hip_roll", 90.0f},
    {"left_hip_pitch", 180.0f},

    {"left_elbow", 180.0f},
    {"left_shoulder_roll", 90.0f},
    {"left_shoulder_pitch", 180.0f},

    {"right_knee", 180.0f},
    {"right_hip_roll", 90.0f},
    {"right_hip_pitch", 180.0f},

    {"right_elbow", 180.0f},
    {"right_shoulder_roll", 90.0f},
    {"right_shoulder_pitch", 180.0f},
};

// Update sw_data with the goal positions from the ROS message
void servo_cmd_cb(const humanoid_msgs::ServoCommand &input_msg)
{
  DEBUG_PRINTF("%s() start\r\n", __func__);

  // Map of joint names to their desired positions from the ROS message
  std::unordered_map<std::string, float> joint_positions = {
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

  // Iterate through each joint and update its goal position
  for (const auto &joint : joint_positions)
  {
    auto it = joints_map.find(joint.first);
    if (it != joints_map.end())
    {
      Joint *joint_ptr = it->second;
      joint_ptr->goal_position_raw = degToRaw(joint.second);

      // Update the corresponding SyncWrite data
      DynamixelBusConfig_t *bus = joint_ptr->bus;

      // Find the index of this joint's ID in the bus's id_list
      int index = -1;
      for (size_t i = 0; i < bus->id_count; i++)
      {
        if (bus->id_list[i] == joint_ptr->id)
        {
          index = i;
          break;
        }
      }

      if (index != -1)
      {
        bus->sw_data[index].goal_position = joint_ptr->goal_position_raw;
      }
      else
      {
        DEBUG_PRINTF("Joint ID %d not found in bus's id_list.\n", joint_ptr->id);
      }
    }
    else
    {
      DEBUG_PRINTF("Joint not found: %s\r\n", joint.first.c_str());
    }
  }

  DEBUG_PRINTF("%s() end\r\n", __func__);
}

void ros_setup()
{
  populate_joint_id_to_name();

  DEBUG_PRINTF("%s() start\r\n", __func__);
  nh.initNode();

  nh.advertise(servo_fb_pub);
  nh.subscribe(servo_cmd_sub);

  nh.negotiateTopics();
  while (!nh.connected())
  {
    nh.negotiateTopics();
    DEBUG_PRINTF("Waiting for ROS connection...\r\n"); // Debugging line
  }

  // Setup length of float arrays in the message
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

  // Setup pointers to the feedback arrays
  servo_fb_msg.right_shoulder_pitch = right_shoulder_pitch.feedback;
  servo_fb_msg.right_shoulder_roll = right_shoulder_roll.feedback;
  servo_fb_msg.right_elbow = right_elbow.feedback;
  servo_fb_msg.left_shoulder_pitch = left_shoulder_pitch.feedback;
  servo_fb_msg.left_shoulder_roll = left_shoulder_roll.feedback;
  servo_fb_msg.left_elbow = left_elbow.feedback;
  servo_fb_msg.left_hip_roll = left_hip_roll.feedback;
  servo_fb_msg.left_hip_pitch = left_hip_pitch.feedback;
  servo_fb_msg.left_knee = left_knee.feedback;
  servo_fb_msg.right_hip_roll = right_hip_roll.feedback;
  servo_fb_msg.right_hip_pitch = right_hip_pitch.feedback;
  servo_fb_msg.right_knee = right_knee.feedback;

  DEBUG_PRINTF("%s() end\r\n", __func__);
}

void process_bus_feedback(DynamixelBusConfig_t *bus_config)
{
  for (size_t i = 0; i < bus_config->id_count; i++)
  {
    uint8_t id = bus_config->info_xels_sr[i].id;
    auto it = joint_id_to_name.find(id);
    if (it != joint_id_to_name.end())
    {
      // FIXME check this
      // std::string joint_name = it->second;
      // std::string joint_name = it->second.c_str();
      std::string joint_name = joint_id_to_name[bus_config->info_xels_sr[i].id].c_str();

      Joint *joint_ptr = joints_map[joint_name];
      joint_ptr->current_position_raw = bus_config->sr_data[i].present_position;
      float current_position_deg = rawToDeg(bus_config->sr_data[i].present_position);

      // DEBUG_PRINTF("joint_name=%s, bus_config->sr_data[i].present_position %d\r\n", joint_name, bus_config->sr_data[i].present_position);
      DEBUG_PRINTF("joint_name=%s, present_position=%d\r\n",
                   joint_name, bus_config->sr_data[i].present_position);

      float feedback[3] = {
          // static_cast<float>(bus_config->sr_data[i].present_position),
          static_cast<float>(current_position_deg),
          static_cast<float>(0.0),
          static_cast<float>(0.0),
      };

      // Assign feedback directly to the joint's feedback array
      joint_ptr->feedback[0] = current_position_deg;
      joint_ptr->feedback[1] = 0.0f; // Placeholder for additional data
      joint_ptr->feedback[2] = 0.0f; // Placeholder for additional data
    }
    else
    {
      DEBUG_PRINTF("Joint ID %d not found in joint_id_to_name map.\n", id);
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
  while (!DEBUG_SERIAL)
    ;

  DEBUG_PRINTF("Multiple bus sync read write app\n");

  sync_read_app_setup(&bus1);
  sync_read_app_setup(&bus2);
  sync_read_app_setup(&bus3);
  sync_read_app_setup(&bus4);

  ros_setup();

  // Iterate through each joint and update its goal position
  for (const auto &joint : joint_initial_positions)
  {
    auto it = joints_map.find(joint.first);
    if (it != joints_map.end())
    {
      Joint *joint_ptr = it->second;
      joint_ptr->goal_position_raw = degToRaw(joint.second);

      // Update the corresponding SyncWrite data
      DynamixelBusConfig_t *bus = joint_ptr->bus;

      // Find the index of this joint's ID in the bus's id_list
      int index = -1;
      for (size_t i = 0; i < bus->id_count; i++)
      {
        if (bus->id_list[i] == joint_ptr->id)
        {
          index = i;
          break;
        }
      }

      if (index != -1)
      {
        bus->sw_data[index].goal_position = joint_ptr->goal_position_raw;
      }
      else
      {
        DEBUG_PRINTF("Joint ID %d not found in bus's id_list.\n", joint_ptr->id);
      }
    }
    else
    {
      DEBUG_PRINTF("Joint not found: %s\r\n", joint.first.c_str());
    }
  }
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