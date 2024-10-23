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

const uint16_t SR_START_ADDR_POSITION = 132;
const uint16_t SR_START_ADDR_VELOCITY = 128;
const uint16_t SR_START_ADDR_LOAD = 126;
const uint16_t SR_ADDR_LEN = 4;
const uint16_t GOAL_POSITION_ADDR = 116;
const uint16_t GOAL_PWM_ADDR = 100;
const uint16_t GOAL_POSITION_LEN = 4;
const uint16_t GOAL_PWM_LEN = 2;

// Structures for SyncRead and SyncWrite data
typedef struct sr_data
{
  int32_t data_raw;
} __attribute__((packed)) sr_data_t;

typedef struct sw_data
{
  int32_t data_raw;
} __attribute__((packed)) sw_data_t;

// Function to convert degrees to raw goal
static int32_t degToRaw(float degrees)
{
  return static_cast<int32_t>(degrees * (4095.0 / 360.0));
}

// Function to convert raw goal to degrees
static float rawToDeg(int32_t raw)
{
  return static_cast<float>(raw) * (360.0 / 4095.0);
}

// Forward declaration of classes
class Joint;
class DynamixelBus;

// Map of joints
std::unordered_map<uint8_t, std::string> joint_id_to_name;
std::unordered_map<uint8_t, std::shared_ptr<Joint>> joint_id_to_joint;
std::unordered_map<std::string, std::shared_ptr<Joint>> joint_name_to_joint;

// ------------------ Configuration Structures ------------------

struct JointConfig
{
  std::string name;
  uint8_t id;
  uint8_t bus_number; // 1-based bus number
  float initial_position_deg;
};

struct BusConfig
{
  uint8_t bus_number; // 1-based bus number
  HardwareSerial &serial_port;
};

// ------------------ Configuration Data ------------------

// List of bus configurations
std::vector<BusConfig> bus_configs = {
    {1, Serial1},
    {2, Serial2},
    {3, Serial3},
    {4, Serial4},
};

// List of joint configurations
std::vector<JointConfig> joint_configs = {
    {"left_knee", 11, 1, 180.0f},
    {"left_hip_roll", 12, 1, 90.0f},
    {"left_hip_pitch", 13, 1, 180.0f},

    {"left_elbow", 14, 2, 180.0f},
    {"left_shoulder_roll", 15, 2, 90.0f},
    {"left_shoulder_pitch", 16, 2, 180.0f},

    {"right_knee", 21, 3, 180.0f},
    {"right_hip_roll", 22, 3, 90.0f},
    {"right_hip_pitch", 23, 3, 180.0f},

    {"right_elbow", 24, 4, 180.0f},
    {"right_shoulder_roll", 25, 4, 90.0f},
    {"right_shoulder_pitch", 26, 4, 180.0f},
};

// ------------------ Class Definitions ------------------

class Joint
{
public:
  // Constructor
  Joint(const JointConfig &cfg)
      : name(cfg.name), id(cfg.id), bus_number(cfg.bus_number),
        goal_raw(degToRaw(cfg.initial_position_deg)), present_position_raw(INT32_MIN)
  {
    feedback.resize(3, 0.0f);
  }

  void update_goal(float data_raw);
  void update_present_position(int32_t raw_position);
  void update_present_velocity(int32_t raw_velocity);
  void update_present_load(int32_t raw_load);

  std::string name;
  uint8_t id;
  uint8_t bus_number;
  int32_t goal_raw;
  int32_t present_position_raw;
  int32_t present_velocity_raw;
  int32_t present_load_raw;
  std::vector<float> feedback;
  int operating_mode = OP_POSITION;
};

class DynamixelBus
{
public:
  static constexpr uint16_t user_pkt_buf_cap = 128;

  DynamixelBus(const BusConfig &bus_cfg)
      : bus_number(bus_cfg.bus_number), dxl(bus_cfg.serial_port, DXL_DIR_PIN)
  {
    user_pkt_buf.resize(user_pkt_buf_cap, 0);
  }

  void setup();
  void tick();
  void update_goal(uint8_t id, int32_t position_raw);
  void processFeedback();

  void addJoint(std::shared_ptr<Joint> joint);

  // Members
  uint8_t bus_number;
  Dynamixel2Arduino dxl;
  std::vector<uint8_t> id_list;
  size_t id_count;

  // Joints managed by this bus
  std::vector<std::shared_ptr<Joint>> joints;

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

// ------------------ Global Variables ------------------

// Map of bus number to DynamixelBus instance
std::unordered_map<uint8_t, std::shared_ptr<DynamixelBus>> bus_map;

// Vector of buses for easy iteration
std::vector<std::shared_ptr<DynamixelBus>> buses;

// ------------------ Function Implementations ------------------

void Joint::update_goal(float data_raw)
{
  if (operating_mode == OP_POSITION)
  {
    goal_raw = degToRaw(data_raw);
  }
  auto bus_it = bus_map.find(bus_number);
  if (bus_it != bus_map.end())
  {
    bus_it->second->update_goal(id, goal_raw);
  }
  else
  {
    DEBUG_PRINTF("Bus number %d not found for joint %s\n", bus_number, name.c_str());
  }
}

void Joint::update_present_position(int32_t raw_position)
{
  present_position_raw = raw_position;
  feedback[0] = rawToDeg(raw_position);
}

void Joint::update_present_velocity(int32_t raw_velocity)
{
  present_velocity_raw = raw_velocity;
  feedback[1] = raw_velocity;
}

void Joint::update_present_load(int32_t raw_load)
{
  present_load_raw = raw_load;
  feedback[2] = raw_load;
}

void DynamixelBus::addJoint(std::shared_ptr<Joint> joint)
{
  joints.push_back(joint);
  id_list.push_back(joint->id);
}

void DynamixelBus::setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  id_count = id_list.size();

  // Initialize the Dynamixel bus
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Initialize IDs
  for (auto id : id_list)
  {
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.torqueOn(id);
    auto joint = joint_name_to_joint[joint_id_to_name[id]];
    joint->operating_mode = OP_POSITION;
  }

  // Allocate the vectors
  sr_data.resize(id_count);
  info_xels_sr.resize(id_count);
  sw_data.resize(id_count);
  info_xels_sw.resize(id_count);

  // Fill SyncRead info
  sr_infos.packet.p_buf = user_pkt_buf.data();
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR_POSITION;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr.data();
  sr_infos.xel_count = id_count;
  sr_infos.is_info_changed = true;

  // Fill SyncWrite info
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;

  sw_infos.addr = GOAL_POSITION_ADDR;
  sw_infos.addr_length = GOAL_POSITION_LEN;

  sw_infos.p_xels = info_xels_sw.data();
  sw_infos.xel_count = id_count;
  sw_infos.is_info_changed = true;

  // Configure XELs
  for (size_t i = 0; i < id_count; ++i)
  {
    info_xels_sr[i].id = id_list[i];
    info_xels_sr[i].p_recv_buf = reinterpret_cast<uint8_t *>(&sr_data[i]);

    info_xels_sw[i].id = id_list[i];
    info_xels_sw[i].p_data = reinterpret_cast<uint8_t *>(&sw_data[i].data_raw);

    // Set initial goal positions from joints
    sw_data[i].data_raw = joints[i]->goal_raw;
  }

  // Go to initial positions
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
                   sw_data[i].data_raw,
                   rawToDeg(sw_data[i].data_raw));
    }
  }
  else
  {
    DEBUG_PRINT("[SyncWrite] Fail, Lib error code: ");
    DEBUG_PRINT(dxl.getLastLibErrCode());
  }
  DEBUG_PRINTLN();
}

void switch_mode(int operating_mode)
{
  for (auto &bus : buses)
  {
    for (auto &joint : bus->joints)
    {
      joint->operating_mode = operating_mode;
      bus->dxl.torqueOff(joint->id);
      bus->dxl.setOperatingMode(joint->id, operating_mode);
      bus->dxl.torqueOn(joint->id);
    }
    if (operating_mode == OP_POSITION)
    {
      bus->sw_infos.addr = GOAL_POSITION_ADDR;
      bus->sw_infos.addr_length = GOAL_POSITION_LEN;
      bus->sw_infos.is_info_changed = true;
    }
    else if (operating_mode == OP_PWM)
    {
      bus->sw_infos.addr = GOAL_PWM_ADDR;
      bus->sw_infos.addr_length = GOAL_PWM_LEN;
      bus->sw_infos.is_info_changed = true;
    }
    else
    {
      DEBUG_PRINTLN("Invalid operating mode");
      return;
    }
  }
}

void process_serial_cmd()
{
  static String inputString = "";

  while (DEBUG_SERIAL.available())
  {
    char inChar = (char)DEBUG_SERIAL.read(); // Read each character
    DEBUG_SERIAL.printf("Received: %c\r\n", inChar);
    // Process the completed command
    if (inChar == 'p')
    {
      DEBUG_PRINTLN("Switching to Position mode");
      switch_mode(OP_POSITION);
    }
    else if (inChar == 't')
    {
      DEBUG_PRINTLN("Switching to PWM mode");
      switch_mode(OP_PWM);
    }
    else
    {
      DEBUG_SERIAL.println("Unknown command");
    }
  }
}

void DynamixelBus::tick()
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
                   sw_data[i].data_raw,
                   rawToDeg(sw_data[i].data_raw));
    }
  }
  else
  {
    DEBUG_PRINT("[SyncWrite] Fail, Lib error code: ");
    DEBUG_PRINT(dxl.getLastLibErrCode());
  }
  DEBUG_PRINTLN();

  // SyncRead Position
  sr_infos.addr = SR_START_ADDR_POSITION;
  sr_infos.is_info_changed = true;
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

      auto joint = joint_name_to_joint[joint_name];
      joint->update_present_position(sr_data[i].data_raw);

      DEBUG_PRINTF("\tID=%d, name=%s\r\n", id, joint_name.c_str());
      DEBUG_PRINTF("\t\tpresent_position_raw=%d, present_position_deg=%f\r\n",
                   sr_data[i].data_raw,
                   rawToDeg(sr_data[i].data_raw));
    }
  }
  else
  {
    DEBUG_PRINT("[SyncRead] Fail, Lib error code: ");
    DEBUG_PRINTLN(dxl.getLastLibErrCode());
  }

  // SyncRead Velocity
  sr_infos.addr = SR_START_ADDR_VELOCITY;
  sr_infos.addr_length = 2;
  sr_infos.is_info_changed = true;
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

      auto joint = joint_name_to_joint[joint_name_it->second];
      joint->update_present_velocity(sr_data[i].data_raw);

      DEBUG_PRINTF("\tID=%d, name=%s\r\n", id, joint_name.c_str());
      DEBUG_PRINTF("\t\tpresent_velocity_raw=%d\r\n",
                   sr_data[i].data_raw);
    }
  }
  else
  {
    DEBUG_PRINT("[SyncRead] Fail, Lib error code: ");
    DEBUG_PRINTLN(dxl.getLastLibErrCode());
  }

  // SyncRead Load
  sr_infos.addr = SR_START_ADDR_LOAD;
  sr_infos.is_info_changed = true;
  sr_infos.addr_length = 2;
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

      auto joint = joint_name_to_joint[joint_name_it->second];
      joint->update_present_load(sr_data[i].data_raw);

      DEBUG_PRINTF("\tID=%d, name=%s\r\n", id, joint_name.c_str());
      DEBUG_PRINTF("\t\tpresent_load_raw=%d\r\n",
                   sr_data[i].data_raw);
      for (int i = 0; i < 3; i++)
      {
        DEBUG_PRINTF("%s feedback[%d]: %f\r\n", joint_name.c_str(), i, joint->feedback[i]);
      }
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

void DynamixelBus::update_goal(uint8_t id, int32_t position_raw)
{
  auto it = std::find(id_list.begin(), id_list.end(), id);
  if (it != id_list.end())
  {
    size_t index = std::distance(id_list.begin(), it);
    sw_data[index].data_raw = position_raw;
    sw_infos.is_info_changed = true;
  }
  else
  {
    DEBUG_PRINTF("ID %d not found in bus id_list.\n", id);
  }
}

// ------------------ ROS Callback ------------------

void servo_cmd_cb(const humanoid_msgs::ServoCommand &input_msg)
{
  DEBUG_PRINTF("%s() start\r\n", __func__);

  // Map of joint names to their desired positions from the ROS message
  std::unordered_map<std::string, float> joint_goals = {
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
  for (const auto &[name, goal] : joint_goals)
  {
    if (auto it = joint_name_to_joint.find(name); it != joint_name_to_joint.end())
    {
      it->second->update_goal(goal);
    }
    else
    {
      DEBUG_PRINTF("Joint not found: %s\r\n", name.c_str());
    }
  }

  DEBUG_PRINTF("%s() end\r\n", __func__);
}

// ------------------ Servo Setup Function ------------------
void servo_setup()
{
  // Create buses and add them to bus_map and buses vector
  for (const auto &bus_cfg : bus_configs)
  {
    auto bus = std::make_shared<DynamixelBus>(bus_cfg);
    bus_map[bus_cfg.bus_number] = bus;
    buses.push_back(bus);
  }

  // Create joints and assign them to buses
  for (const auto &joint_cfg : joint_configs)
  {
    auto joint = std::make_shared<Joint>(joint_cfg);
    joint_name_to_joint[joint->name] = joint;
    joint_id_to_name[joint->id] = joint->name;

    // Add joint to the appropriate bus
    auto bus_it = bus_map.find(joint->bus_number);
    if (bus_it != bus_map.end())
    {
      bus_it->second->addJoint(joint);
    }
    else
    {
      DEBUG_PRINTF("Bus number %d not found for joint %s\n", joint->bus_number, joint->name.c_str());
    }
  }

  // Setup each bus
  for (auto &bus : buses)
  {
    bus->setup();
  }
}

// ------------------ ROS Setup Function ------------------

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
  servo_fb_msg.right_shoulder_pitch = joint_name_to_joint["right_shoulder_pitch"]->feedback.data();
  servo_fb_msg.right_shoulder_roll = joint_name_to_joint["right_shoulder_roll"]->feedback.data();
  servo_fb_msg.right_elbow = joint_name_to_joint["right_elbow"]->feedback.data();
  servo_fb_msg.left_shoulder_pitch = joint_name_to_joint["left_shoulder_pitch"]->feedback.data();
  servo_fb_msg.left_shoulder_roll = joint_name_to_joint["left_shoulder_roll"]->feedback.data();
  servo_fb_msg.left_elbow = joint_name_to_joint["left_elbow"]->feedback.data();
  servo_fb_msg.left_hip_roll = joint_name_to_joint["left_hip_roll"]->feedback.data();
  servo_fb_msg.left_hip_pitch = joint_name_to_joint["left_hip_pitch"]->feedback.data();
  servo_fb_msg.left_knee = joint_name_to_joint["left_knee"]->feedback.data();
  servo_fb_msg.right_hip_roll = joint_name_to_joint["right_hip_roll"]->feedback.data();
  servo_fb_msg.right_hip_pitch = joint_name_to_joint["right_hip_pitch"]->feedback.data();
  servo_fb_msg.right_knee = joint_name_to_joint["right_knee"]->feedback.data();

  DEBUG_PRINTF("%s() end\r\n", __func__);
}

void servo_loop()
{
  for (auto &bus : buses)
  {
    bus->tick();
  }
}

void ros_loop()
{
  nh.spinOnce();

  servo_fb_pub.publish(&servo_fb_msg);
}

void setup()
{
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL)
    ;
  DEBUG_PRINTF("Multiple bus sync read write app\n");

  servo_setup();
  ros_setup();
}

void loop()
{
  process_serial_cmd();
  servo_loop();
  ros_loop();
}

#endif // COMPILE_CFG
