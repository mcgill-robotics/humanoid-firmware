#include "common.h"
#if COMPILE_CFG == 1

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

// ------------------ ROS ------------------
ros::NodeHandle nh;
void servo_cmd_cb(const humanoid_msgs::ServoCommand &input_msg);

humanoid_msgs::ServoFeedback servo_fb_msg;
ros::Publisher servo_fb_pub("servosFeedback", &servo_fb_msg);
ros::Subscriber<humanoid_msgs::ServoCommand> servo_cmd_sub("servosCommand",
                                                           servo_cmd_cb);

// ------------------ Dynamixel ------------------
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL SerialUSB
const int DXL_DIR_PIN = -1;

const uint8_t BROADCAST_ID = 0xFE;
const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

const uint16_t user_pkt_buf_cap = 128;
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
} __attribute__((packed)) sr_data_t;

typedef struct sw_data
{
  int32_t goal_position_raw;
} __attribute__((packed)) sw_data_t;

// Define the joints and their corresponding Dynamixel IDs and goal positions
struct Joint
{
  uint8_t id;
  int32_t *goal_position_raw;
};

// Map for joints
std::unordered_map<std::string, Joint> joints = {
    {"right_shoulder_pitch", {21, nullptr}},
    {"right_shoulder_roll", {22, nullptr}},
    {"right_elbow", {23, nullptr}},
    {"left_shoulder_pitch", {11, nullptr}},
    {"left_shoulder_roll", {12, nullptr}},
    {"left_elbow", {13, nullptr}},
    {"left_hip_roll", {14, nullptr}},
    {"left_hip_pitch", {15, nullptr}},
    {"left_knee", {16, nullptr}},
    {"right_hip_roll", {24, nullptr}},
    {"right_hip_pitch", {25, nullptr}},
    {"right_knee", {26, nullptr}},
};

const uint8_t DXL_ID_CNT = joints.size();
sr_data_t *sr_data = new sr_data_t[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t *info_xels_sr = new DYNAMIXEL::XELInfoSyncRead_t[DXL_ID_CNT];
sw_data_t *sw_data = new sw_data_t[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t *info_xels_sw = new DYNAMIXEL::XELInfoSyncWrite_t[DXL_ID_CNT];

//------------------------------------------ Prototypes ------------------------------------------
void setup();
void loop();
void servo_setup();
void servo_loop();
void ros_setup();
void ros_loop();

void setup()
{
  servo_setup();
  ros_setup();
}

void loop()
{
  delay(10);
  servo_loop();
  ros_loop();
}

void ros_setup()
{
  nh.initNode();

  nh.advertise(servo_fb_pub);
  nh.subscribe(servo_cmd_sub);

  nh.negotiateTopics();
  while (!nh.connected())
  {
    nh.negotiateTopics();
  }
}

// Publish servo feedback
void ros_loop()
{
  nh.spinOnce();

  for (uint8_t i = 0; i < DXL_ID_CNT; i++)
  {
    std::string joint_name;
    for (const auto &joint : joints)
    {
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
          // static_cast<float>(sr_data[i].present_velocity_raw),
          // static_cast<float>(sr_data[i].present_load_raw),
      };

      if (joint_name == "right_shoulder_pitch")
        memcpy(servo_fb_msg.right_shoulder_pitch, feedback, sizeof(feedback));
      else if (joint_name == "right_shoulder_roll")
        memcpy(servo_fb_msg.right_shoulder_roll, feedback, sizeof(feedback));
      else if (joint_name == "right_elbow")
        memcpy(servo_fb_msg.right_elbow, feedback, sizeof(feedback));
      else if (joint_name == "left_shoulder_pitch")
        memcpy(servo_fb_msg.left_shoulder_pitch, feedback, sizeof(feedback));
      else if (joint_name == "left_shoulder_roll")
        memcpy(servo_fb_msg.left_shoulder_roll, feedback, sizeof(feedback));
      else if (joint_name == "left_elbow")
        memcpy(servo_fb_msg.left_elbow, feedback, sizeof(feedback));
      else if (joint_name == "left_hip_roll")
        memcpy(servo_fb_msg.left_hip_roll, feedback, sizeof(feedback));
      else if (joint_name == "left_hip_pitch")
        memcpy(servo_fb_msg.left_hip_pitch, feedback, sizeof(feedback));
      else if (joint_name == "left_knee")
        memcpy(servo_fb_msg.left_knee, feedback, sizeof(feedback));
      else if (joint_name == "right_hip_roll")
        memcpy(servo_fb_msg.right_hip_roll, feedback, sizeof(feedback));
      else if (joint_name == "right_hip_pitch")
        memcpy(servo_fb_msg.right_hip_pitch, feedback, sizeof(feedback));
      else if (joint_name == "right_knee")
        memcpy(servo_fb_msg.right_knee, feedback, sizeof(feedback));
    }
  }

  servo_fb_pub.publish(&servo_fb_msg);
}

// Save setpoints from ROS message
void servo_cmd_cb(const humanoid_msgs::ServoCommand &input_msg)
{
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

  for (const auto &joint : joint_positions)
  {
    auto it = joints.find(joint.first);
    if (it != joints.end())
    {
      *(it->second.goal_position_raw) = joint.second;
    }
  }
}

void servo_setup()
{
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  uint8_t index = 0;
  for (auto &joint : joints)
  {
    joint.second.goal_position_raw = &sw_data[index].goal_position_raw;
    dxl.torqueOff(joint.second.id);
    dxl.setOperatingMode(joint.second.id, OP_POSITION);
    info_xels_sr[index].id = joint.second.id;
    info_xels_sr[index].p_recv_buf = (uint8_t *)&sr_data[index];
    info_xels_sw[index].id = joint.second.id;
    info_xels_sw[index].p_data = (uint8_t *)&sw_data[index].goal_position_raw;
    index++;
  }
  dxl.torqueOn(BROADCAST_ID);

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = DXL_ID_CNT;
  sr_infos.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = DXL_ID_CNT;
  sw_infos.is_info_changed = true;
}

void servo_loop()
{
  static uint32_t try_count = 0;
  uint8_t i, recv_cnt;

  // Update the SyncWrite packet status
  sw_infos.is_info_changed = true;

  ros_printf("\n>>>>>> Sync Instruction Test : %u\n", try_count++);

  // Build a SyncWrite Packet and transmit to DYNAMIXEL
  if (dxl.syncWrite(&sw_infos) == true)
  {
    ros_printf("[SyncWrite] Success\n");
    for (i = 0; i < sw_infos.xel_count; i++)
    {
      ros_printf("  ID: %u\n", sw_infos.p_xels[i].id);
      ros_printf("\t Goal Position: %d\n", sw_data[i].goal_position_raw);
    }
  }
  else
  {
    ros_printf("[SyncWrite] Fail, Lib error code: %d\n", dxl.getLastLibErrCode());
  }
  ros_printf("\n");

  delay(250);

  // Transmit predefined SyncRead instruction packet
  // and receive a status packet from each DYNAMIXEL
  recv_cnt = dxl.syncRead(&sr_infos);
  if (recv_cnt > 0)
  {
    ros_printf("[SyncRead] Success, Received ID Count: %u\n", recv_cnt);
    for (i = 0; i < recv_cnt; i++)
    {
      ros_printf("  ID: %u, Error: %u\n", sr_infos.p_xels[i].id, sr_infos.p_xels[i].error);
      ros_printf("\t Present Position: %d\n", sr_data[i].present_position_raw);
    }
  }
  else
  {
    ros_printf("[SyncRead] Fail, Lib error code: %d\n", dxl.getLastLibErrCode());
  }
  ros_printf("=======================================================\n");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(750);
}
#endif // COMPILE_CFG
