#include "common.h"
#if COMPILE_CFG == 99

#include <Arduino.h>
#include <unordered_map>
#include <string>
#include <vector>
#include "cmd_utils.hpp"
#include <Dynamixel2Arduino.h>

// Include the ROS library
#include "ros.h"
#include "ServoCommand.h"
#include "ServoFeedback.h"
#include "ros_helpers.h"

// Function to check available memory
extern "C"
{
  char *sbrk(int incr);
}

int freeMemory()
{
  char top;
  return &top - reinterpret_cast<char *>(sbrk(0));
}

// ------------------ ROS ------------------
ros::NodeHandle nh;
void servo_cmd_cb(const humanoid_msgs::ServoCommand &input_msg);

humanoid_msgs::ServoFeedback servo_fb_msg;
ros::Publisher servo_fb_pub("servosFeedback", &servo_fb_msg);
ros::Subscriber<humanoid_msgs::ServoCommand> servo_cmd_sub("servosCommand", servo_cmd_cb);

// ------------------ Dynamixel ------------------
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL SerialUSB
const int DXL_DIR_PIN = -1;

const uint8_t BROADCAST_ID = 0xFE;
const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// Reduced buffer size for testing
const uint16_t user_pkt_buf_cap = 256;
uint8_t user_pkt_buf[user_pkt_buf_cap];

const uint16_t SR_START_ADDR = 132;
const uint16_t SR_ADDR_LEN = 4;
const uint16_t SW_START_ADDR = 116;
const uint16_t SW_ADDR_LEN = 4;

typedef struct sr_data
{
  int32_t present_position_raw;
  int32_t present_velocity_raw;
  int32_t present_load_raw;
} sr_data_t;

typedef struct sw_data
{
  int32_t goal_position_raw;
} sw_data_t;

// Define the joints and their corresponding Dynamixel IDs and goal positions
struct Joint
{
  uint8_t id;
  int32_t goal_position_raw;
};

// Map for joints
std::unordered_map<std::string, Joint> joints = {
    {"right_shoulder_pitch", {21, 2048}},
    {"right_shoulder_roll", {22, 2048}},
    {"right_elbow", {23, 2048}},
    // {"left_shoulder_pitch", {11, 2048}},
    // {"left_shoulder_roll", {12, 2048}},
    // {"left_elbow", {13, 2048}},
    // {"left_hip_roll", {14, 2048}},
    // {"left_hip_pitch", {15, 2048}},
    // {"left_knee", {16, 2048}},
    // {"right_hip_roll", {24, 2048}},
    // {"right_hip_pitch", {25, 2048}},
    // {"right_knee", {26, 2048}},
};

const uint8_t DXL_ID_CNT = joints.size();

// sr_infos => info_xels_sr => sr_data
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t *info_xels_sr = new DYNAMIXEL::XELInfoSyncRead_t[DXL_ID_CNT];
sw_data_t *sw_data = new sw_data_t[DXL_ID_CNT];

DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t *info_xels_sw = new DYNAMIXEL::XELInfoSyncWrite_t[DXL_ID_CNT];
sr_data_t *sr_data = new sr_data_t[DXL_ID_CNT];

// const uint8_t DXL_ID_CNT = joints.size();
// const uint8_t DXL_ID_CNT = 12;
// sr_data_t sr_data[DXL_ID_CNT];
// DYNAMIXEL::InfoSyncReadInst_t sr_infos;
// DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];
// sw_data_t sw_data[DXL_ID_CNT];
// DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
// DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

int servo_loop_last_millis = 0;

//------------------------------------------ Prototypes ------------------------------------------
void setup();
void loop();
void servo_setup();
void servo_loop();
void ros_setup();
void ros_loop();

void setup()
{
  Serial2.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial2.printf("Free memory before allocation: %d bytes\r\n", freeMemory());

  Serial2.printf("DXL_ID_CNT=%d\r\n", DXL_ID_CNT); // Debugging line
  Serial2.printf("Setup start\r\n");               // Debugging line
  servo_setup();
  ros_setup();
  Serial2.printf("Setup end\r\n"); // Debugging line

  Serial2.printf("Free memory after allocation: %d bytes\r\n", freeMemory());

  servo_loop_last_millis = millis();
}

void loop()
{
  delay(1);
  servo_loop();
  ros_loop();
}

void ros_setup()
{
  Serial2.printf("%s() start\r\n", __func__);
  nh.initNode();

  nh.advertise(servo_fb_pub);
  nh.subscribe(servo_cmd_sub);

  nh.negotiateTopics();
  while (!nh.connected())
  {
    nh.negotiateTopics();
    Serial2.printf("Waiting for ROS connection...\r\n"); // Debugging line
  }

  servo_fb_msg.right_shoulder_pitch_length = 3;
  servo_fb_msg.right_shoulder_roll_length = 3;
  servo_fb_msg.right_elbow_length = 3;
  // servo_fb_msg.left_shoulder_pitch_length = 3;
  // servo_fb_msg.left_shoulder_roll_length = 3;
  // servo_fb_msg.left_elbow_length = 3;
  // servo_fb_msg.left_hip_roll_length = 3;
  // servo_fb_msg.left_hip_pitch_length = 3;
  // servo_fb_msg.left_knee_length = 3;
  // servo_fb_msg.right_hip_roll_length = 3;
  // servo_fb_msg.right_hip_pitch_length = 3;
  // servo_fb_msg.right_knee_length = 3;

  Serial2.printf("%s() end\r\n", __func__);
}

// Publish servo feedback
void ros_loop()
{
  // Serial2.printf("%s() start\r\n", __func__);
  nh.spinOnce();

  for (uint8_t i = 0; i < DXL_ID_CNT; i++)
  {
    std::string joint_name;
    for (const auto &joint : joints)
    {
      // Get name of the joint by comparing the ID
      if (joint.second.id == info_xels_sr[i].id)
      {
        joint_name = joint.first;
        break;
      }
    }

    if (!joint_name.empty())
    {
      float feedback[3] = {
          static_cast<float>(sr_data[i].present_position_raw),
          static_cast<float>(0.0),
          static_cast<float>(0.0),
      };

      // Comment out if not used because these are float pointers which could cause segmentation fault
      if (joint_name == "right_shoulder_pitch")
        servo_fb_msg.right_shoulder_pitch = feedback;
      else if (joint_name == "right_shoulder_roll")
        servo_fb_msg.right_shoulder_roll = feedback;
      else if (joint_name == "right_elbow")
        servo_fb_msg.right_elbow = feedback;
      // else if (joint_name == "left_shoulder_pitch")
      //   servo_fb_msg.left_shoulder_pitch = feedback;
      // else if (joint_name == "left_shoulder_roll")
      //   servo_fb_msg.left_shoulder_roll = feedback;
      // else if (joint_name == "left_elbow")
      //   servo_fb_msg.left_elbow = feedback;
      // else if (joint_name == "left_hip_roll")
      //   servo_fb_msg.left_hip_roll = feedback;
      // else if (joint_name == "left_hip_pitch")
      //   servo_fb_msg.left_hip_pitch = feedback;
      // else if (joint_name == "left_knee")
      //   servo_fb_msg.left_knee = feedback;
      // else if (joint_name == "right_hip_roll")
      //   servo_fb_msg.right_hip_roll = feedback;
      // else if (joint_name == "right_hip_pitch")
      //   servo_fb_msg.right_hip_pitch = feedback;
      // else if (joint_name == "right_knee")
      //   servo_fb_msg.right_knee = feedback;
    }
  }

  servo_fb_pub.publish(&servo_fb_msg);
  // Serial2.printf("%s() end\r\n", __func__);
}

// Update sw_data with the goal positions from the ROS message
void servo_cmd_cb(const humanoid_msgs::ServoCommand &input_msg)
{
  Serial2.printf("%s() start\r\n", __func__);
  std::unordered_map<std::string, int32_t> joint_positions = {
      {"right_shoulder_pitch", input_msg.right_shoulder_pitch},
      {"right_shoulder_roll", input_msg.right_shoulder_roll},
      {"right_elbow", input_msg.right_elbow},
      // {"left_shoulder_pitch", input_msg.left_shoulder_pitch},
      // {"left_shoulder_roll", input_msg.left_shoulder_roll},
      // {"left_elbow", input_msg.left_elbow},
      // {"left_hip_roll", input_msg.left_hip_roll},
      // {"left_hip_pitch", input_msg.left_hip_pitch},
      // {"left_knee", input_msg.left_knee},
      // {"right_hip_roll", input_msg.right_hip_roll},
      // {"right_hip_pitch", input_msg.right_hip_pitch},
      // {"right_knee", input_msg.right_knee}
  };

  uint8_t index = 0;
  for (const auto &joint : joint_positions)
  {
    auto it = joints.find(joint.first);
    if (it != joints.end())
    {
      it->second.goal_position_raw = deg2raw(joint.second);
      sw_data[index].goal_position_raw = deg2raw(joint.second);
    }
    else
    {
      Serial2.printf("Joint not found: %s\r\n", joint.first.c_str());
    }
    index++;
  }
  Serial2.printf("%s() end\r\n", __func__);
}

void servo_setup()
{
  Serial2.printf("%s() start\r\n", __func__);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  // Clear the data buffers
  // memset(sr_data, 0, sizeof(sr_data_t) * DXL_ID_CNT);
  // memset(sw_data, 0, sizeof(sw_data_t) * DXL_ID_CNT);

  uint8_t index = 0;
  for (auto &joint : joints)
  {
    Serial2.printf("Joint: %s, ID: %d, Goal Position: %d\r\n", joint.first.c_str(), joint.second.id, joint.second.goal_position_raw);
    dxl.torqueOff(joint.second.id);
    dxl.setOperatingMode(joint.second.id, OP_POSITION);

    info_xels_sr[index].id = joint.second.id;
    info_xels_sr[index].p_recv_buf = (uint8_t *)&sr_data[index];

    info_xels_sw[index].id = joint.second.id;
    info_xels_sw[index].p_data = (uint8_t *)&sw_data[index].goal_position_raw;

    // Initialize the goal position to the default value
    sw_data[index].goal_position_raw = 2048;

    sr_infos.xel_count++;
    sw_infos.xel_count++;

    index++;
  }

  Serial2.printf("sr_infos.xel_count=%d, index=%d\r\n", sr_infos.xel_count, index);

  sr_infos.is_info_changed = true;
  sw_infos.is_info_changed = true;

  dxl.torqueOn(BROADCAST_ID);

  Serial2.printf("%s() end\r\n", __func__);
}

void servo_loop()
{
  if (millis() - servo_loop_last_millis < 500)
  {
    return;
  }

  Serial2.printf("%s() start\r\n", __func__);
  static uint32_t try_count = 0;
  uint8_t i, recv_cnt;

  // Update the SyncWrite packet status
  sw_infos.is_info_changed = true;

  Serial2.printf("\n>>>>>> Sync Instruction Test : %u\r\n", try_count++);

  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  if (dxl.syncWrite(&sw_infos) == true)
  {
    Serial2.printf("[SyncWrite] Success\r\n");
    for (i = 0; i < sw_infos.xel_count; i++)
    {
      Serial2.printf("  ID: %u\r\n", sw_infos.p_xels[i].id);
      Serial2.printf("\t Goal Position: %d\r\n", sw_data[i].goal_position_raw);
    }
  }
  else
  {
    Serial2.printf("[SyncWrite] Fail, Lib error code: %d\r\n", dxl.getLastLibErrCode());
  }
  Serial2.printf("\r\n");

  // Transmit predefined SyncRead instruction packet
  // and receive a status packet from each DYNAMIXEL
  recv_cnt = dxl.syncRead(&sr_infos);
  if (1)
  // if (recv_cnt > 0)
  {
    Serial2.printf("[SyncRead] Success, Received ID Count: %u\r\n", recv_cnt);
    // for (i = 0; i < recv_cnt; i++)
    for (i = 0; i < DXL_ID_CNT; i++)
    {
      Serial2.printf("  ID: %u, Error: %u\r\n", sr_infos.p_xels[i].id, sr_infos.p_xels[i].error);
      Serial2.printf("\t Present Position: %d\r\n", sr_data[i].present_position_raw);
    }
  }
  // else
  // {
  //   Serial2.printf("[SyncRead] Fail, Lib error code: %d\r\n", dxl.getLastLibErrCode());
  // }
  Serial2.printf("=======================================================\r\n");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  Serial2.printf("%s() end\r\n", __func__);

  servo_loop_last_millis = millis();
}
#endif // COMPILE_CFG == 1
