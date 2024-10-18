#include "common.h"
#if COMPILE_CFG == 1

#include <Arduino.h>
#include "cmd_utils.hpp"

#include <Dynamixel2Arduino.h>

#include <unordered_map>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>

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

const int DXL_DIR_PIN = -1; // DYNAMIXEL Shield DIR PIN

const uint8_t BROADCAST_ID = 0xFE;
const float DXL_PROTOCOL_VERSION = 2.0;

const uint16_t SR_START_ADDR = 132;
const uint16_t SR_ADDR_LEN = 4;
const uint16_t SW_START_ADDR = 116;
const uint16_t SW_ADDR_LEN = 4;

// Structures for SyncRead and SyncWrite data
typedef struct sr_data
{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;

typedef struct sw_data
{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

// Map of joint names to their initial positions
std::unordered_map<std::string, float> joint_initial_positions = {
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

// Function to convert degrees to raw position
static int32_t degToRaw(float degrees)
{
  return static_cast<int32_t>(degrees * (4095.0 / 360.0));
}

// Function to convert raw position to degrees
static float rawToDeg(int32_t raw)
{
  return static_cast<float>(raw) * (360.0 / 4095.0);
}

// Forward declaration of classes
class Joint;
class DynamixelBus;

// Map of joints
std::unordered_map<std::string, std::shared_ptr<Joint>> joints_map;
std::unordered_map<uint8_t, std::string> joint_id_to_name;

// ------------------ Class Definitions ------------------

class Joint
{
public:
  // Constructor
  Joint(const std::string &name, uint8_t id, DynamixelBus *bus)
      : name(name), id(id), goal_position_raw(2048), current_position_raw(INT32_MIN), bus(bus)
  {
    feedback.resize(3, 0.0f);
  }

  void updateGoalPosition(float degrees);
  void updateFeedback(int32_t raw_position);

  std::string name;
  uint8_t id;
  int32_t goal_position_raw;
  int32_t current_position_raw;
  DynamixelBus *bus;
  std::vector<float> feedback;
};

class DynamixelBus
{
public:
  static constexpr uint16_t user_pkt_buf_cap = 128;

  DynamixelBus(HardwareSerial &serial, int dir_pin, const std::vector<uint8_t> &ids)
      : dxl(serial, dir_pin), id_list(ids)
  {
    id_count = ids.size();
    user_pkt_buf.resize(user_pkt_buf_cap, 0);
  }

  void setup();
  void loop();
  void updateGoalPosition(uint8_t id, int32_t position_raw);
  void processFeedback();

  // Members
  Dynamixel2Arduino dxl;
  std::vector<uint8_t> id_list;
  size_t id_count;

  // SyncRead related data
  std::vector<sr_data_t> sr_data;
  DYNAMIXEL::InfoSyncReadInst_t sr_infos;
  std::vector<DYNAMIXEL::XELInfoSyncRead_t> info_xels_sr;

  // SyncWrite related data
  std::vector<sw_data_t> sw_data;
  DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
  std::vector<DYNAMIXEL::XELInfoSyncWrite_t> info_xels_sw;

  std::vector<uint8_t> user_pkt_buf;
};

// ------------------ Function Implementations ------------------

void Joint::updateGoalPosition(float degrees)
{
  goal_position_raw = degToRaw(degrees);
  bus->updateGoalPosition(id, goal_position_raw);
}

void Joint::updateFeedback(int32_t raw_position)
{
  current_position_raw = raw_position;
  feedback[0] = rawToDeg(raw_position);
  feedback[1] = 0.0f; // Placeholder for additional data
  feedback[2] = 0.0f; // Placeholder for additional data
}

void DynamixelBus::setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize the Dynamixel bus
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Initialize IDs
  for (auto id : id_list)
  {
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.torqueOn(id);
  }

  // Prepare SyncRead and SyncWrite structures
  sr_data.resize(id_count);
  info_xels_sr.resize(id_count);
  sw_data.resize(id_count);
  info_xels_sw.resize(id_count);

  // Fill SyncRead info
  sr_infos.packet.p_buf = user_pkt_buf.data();
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr.data();
  sr_infos.xel_count = id_count;
  sr_infos.is_info_changed = true;

  // Fill SyncWrite info
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw.data();
  sw_infos.xel_count = id_count;
  sw_infos.is_info_changed = true;

  // Configure XELs
  for (size_t i = 0; i < id_count; ++i)
  {
    info_xels_sr[i].id = id_list[i];
    info_xels_sr[i].p_recv_buf = reinterpret_cast<uint8_t *>(&sr_data[i]);

    info_xels_sw[i].id = id_list[i];
    info_xels_sw[i].p_data = reinterpret_cast<uint8_t *>(&sw_data[i].goal_position);
    sw_data[i].goal_position = 2048; // Initialize to center
  }
}

void DynamixelBus::loop()
{
  static uint32_t try_count = 0;
  uint8_t recv_cnt;

  DEBUG_PRINT("\n>>>>>> Sync Instruction Test : ");
  DEBUG_PRINTLN(try_count++);

  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  if (dxl.syncWrite(&sw_infos))
  {
    DEBUG_PRINTLN("[SyncWrite] Success");
    for (size_t i = 0; i < sw_infos.xel_count; i++)
    {
      uint8_t id = info_xels_sw[i].id;
      auto joint_name_it = joint_id_to_name.find(id);
      std::string joint_name = (joint_name_it != joint_id_to_name.end()) ? joint_name_it->second : "Unknown";

      DEBUG_PRINTF("\tID=%d, name=%s\r\n", id, joint_name.c_str());
      DEBUG_PRINTF("\t\tgoal_position_raw=%d, goal_position_deg=%f\r\n",
                   sw_data[i].goal_position,
                   rawToDeg(sw_data[i].goal_position));
    }
  }
  else
  {
    DEBUG_PRINT("[SyncWrite] Fail, Lib error code: ");
    DEBUG_PRINT(dxl.getLastLibErrCode());
  }
  DEBUG_PRINTLN();

  // Transmit predefined SyncRead instruction packet
  recv_cnt = dxl.syncRead(&sr_infos);
  if (recv_cnt > 0)
  {
    DEBUG_PRINT("[SyncRead] Success, Received ID Count: ");
    DEBUG_PRINTLN(recv_cnt);
    for (size_t i = 0; i < recv_cnt; i++)
    {
      uint8_t id = info_xels_sr[i].id;
      auto joint_name_it = joint_id_to_name.find(id);
      std::string joint_name = (joint_name_it != joint_id_to_name.end()) ? joint_name_it->second : "Unknown";

      DEBUG_PRINTF("\tID=%d, name=%s\r\n", id, joint_name.c_str());
      DEBUG_PRINTF("\t\tpresent_position_raw=%d, present_position_deg=%f\r\n",
                   sr_data[i].present_position,
                   rawToDeg(sr_data[i].present_position));
    }
  }
  else
  {
    DEBUG_PRINT("[SyncRead] Fail, Lib error code: ");
    DEBUG_PRINTLN(dxl.getLastLibErrCode());
  }
  DEBUG_PRINTLN("=======================================================");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void DynamixelBus::updateGoalPosition(uint8_t id, int32_t position_raw)
{
  auto it = std::find(id_list.begin(), id_list.end(), id);
  if (it != id_list.end())
  {
    size_t index = std::distance(id_list.begin(), it);
    sw_data[index].goal_position = position_raw;
    sw_infos.is_info_changed = true;
  }
  else
  {
    DEBUG_PRINTF("ID %d not found in bus id_list.\n", id);
  }
}

void DynamixelBus::processFeedback()
{
  for (size_t i = 0; i < id_count; ++i)
  {
    uint8_t id = info_xels_sr[i].id;
    auto joint_name_it = joint_id_to_name.find(id);
    if (joint_name_it != joint_id_to_name.end())
    {
      auto joint = joints_map[joint_name_it->second];
      joint->updateFeedback(sr_data[i].present_position);
    }
    else
    {
      DEBUG_PRINTF("Joint ID %d not found in joint_id_to_name map.\n", id);
    }
  }
}

// Create instances of DynamixelBus
DynamixelBus bus1(Serial1, DXL_DIR_PIN, {11, 12, 13});
DynamixelBus bus2(Serial2, DXL_DIR_PIN, {14, 15, 16});
DynamixelBus bus3(Serial3, DXL_DIR_PIN, {21, 22, 23});
DynamixelBus bus4(Serial4, DXL_DIR_PIN, {24, 25, 26});

// Vector of buses for easy iteration
std::vector<DynamixelBus *> buses = {&bus1, &bus2, &bus3, &bus4};

// Create joints and populate maps
void servo_setup()
{
  // Bus 1 Joints
  auto left_knee = std::make_shared<Joint>("left_knee", 11, &bus1);
  auto left_hip_roll = std::make_shared<Joint>("left_hip_roll", 12, &bus1);
  auto left_hip_pitch = std::make_shared<Joint>("left_hip_pitch", 13, &bus1);

  // Bus 2 Joints
  auto left_elbow = std::make_shared<Joint>("left_elbow", 14, &bus2);
  auto left_shoulder_roll = std::make_shared<Joint>("left_shoulder_roll", 15, &bus2);
  auto left_shoulder_pitch = std::make_shared<Joint>("left_shoulder_pitch", 16, &bus2);

  // Bus 3 Joints
  auto right_knee = std::make_shared<Joint>("right_knee", 21, &bus3);
  auto right_hip_roll = std::make_shared<Joint>("right_hip_roll", 22, &bus3);
  auto right_hip_pitch = std::make_shared<Joint>("right_hip_pitch", 23, &bus3);

  // Bus 4 Joints
  auto right_elbow = std::make_shared<Joint>("right_elbow", 24, &bus4);
  auto right_shoulder_roll = std::make_shared<Joint>("right_shoulder_roll", 25, &bus4);
  auto right_shoulder_pitch = std::make_shared<Joint>("right_shoulder_pitch", 26, &bus4);

  // Add joints to map
  std::vector<std::shared_ptr<Joint>> all_joints = {
      left_knee, left_hip_roll, left_hip_pitch,
      left_elbow, left_shoulder_roll, left_shoulder_pitch,
      right_knee, right_hip_roll, right_hip_pitch,
      right_elbow, right_shoulder_roll, right_shoulder_pitch};

  for (auto &joint : all_joints)
  {
    joints_map[joint->name] = joint;
    joint_id_to_name[joint->id] = joint->name;
  }

  // Set initial joint positions
  for (const auto &[name, degrees] : joint_initial_positions)
  {
    if (auto it = joints_map.find(name); it != joints_map.end())
    {
      it->second->updateGoalPosition(degrees);
    }
  }
}

// ROS Callback function
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

  // Update joint goal positions
  for (const auto &[name, position] : joint_positions)
  {
    if (auto it = joints_map.find(name); it != joints_map.end())
    {
      it->second->updateGoalPosition(position);
    }
    else
    {
      DEBUG_PRINTF("Joint not found: %s\r\n", name.c_str());
    }
  }

  DEBUG_PRINTF("%s() end\r\n", __func__);
}

// ROS Setup function
void ros_setup()
{
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
  servo_fb_msg.right_shoulder_pitch = joints_map["right_shoulder_pitch"]->feedback.data();
  servo_fb_msg.right_shoulder_roll = joints_map["right_shoulder_roll"]->feedback.data();
  servo_fb_msg.right_elbow = joints_map["right_elbow"]->feedback.data();
  servo_fb_msg.left_shoulder_pitch = joints_map["left_shoulder_pitch"]->feedback.data();
  servo_fb_msg.left_shoulder_roll = joints_map["left_shoulder_roll"]->feedback.data();
  servo_fb_msg.left_elbow = joints_map["left_elbow"]->feedback.data();
  servo_fb_msg.left_hip_roll = joints_map["left_hip_roll"]->feedback.data();
  servo_fb_msg.left_hip_pitch = joints_map["left_hip_pitch"]->feedback.data();
  servo_fb_msg.left_knee = joints_map["left_knee"]->feedback.data();
  servo_fb_msg.right_hip_roll = joints_map["right_hip_roll"]->feedback.data();
  servo_fb_msg.right_hip_pitch = joints_map["right_hip_pitch"]->feedback.data();
  servo_fb_msg.right_knee = joints_map["right_knee"]->feedback.data();

  DEBUG_PRINTF("%s() end\r\n", __func__);
}

// ROS Loop function
void ros_loop()
{
  nh.spinOnce();
  for (auto bus : buses)
  {
    bus->processFeedback();
  }
  servo_fb_pub.publish(&servo_fb_msg);
}

// Setup function
void setup()
{
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL)
    ;

  DEBUG_PRINTF("Multiple bus sync read write app\n");

  servo_setup();

  for (auto bus : buses)
  {
    bus->setup();
  }

  ros_setup();
}

// Main loop function
void loop()
{
  for (auto bus : buses)
  {
    bus->loop();
  }
  ros_loop();
}

#endif // COMPILE_CFG
