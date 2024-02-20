#include "Arduino.h"
#include "ServoChain.h"

#define DIR_PIN_UART1 (uint8_t)16
#define LEFT_LEG_ANKLE_ID (uint8_t)14
#define LEFT_LEG_KNEE_ID (uint8_t)15
#define LEFT_LEG_HIPROLL_ID (uint8_t)16
#define LEFT_LEG_HIPYAW_ID (uint8_t)17
#define LEFT_LEG_HIPPITCH_ID (uint8_t)18
#define LEFT_LEG_PRESSURE_ID (uint8_t)19

// uncomment the line below to debug
// #define DEBUG_XL320
XL320Chain chain1(DIR_PIN_UART1, &Serial1);

uint8_t left_leg_ids[6] = {LEFT_LEG_HIPPITCH_ID, LEFT_LEG_HIPYAW_ID,
                           LEFT_LEG_HIPROLL_ID,  LEFT_LEG_KNEE_ID,
                           LEFT_LEG_ANKLE_ID,    LEFT_LEG_PRESSURE_ID};
int left_leg_length = sizeof(left_leg_ids);

unsigned short left_leg_goal_positions[5];
unsigned short left_leg_feedback[18];

void setup() {
// verify all the servos in the chain respond
#ifndef DEBUG_XL320
  int id_err = chain1.verifyIDs(left_leg_ids, left_leg_length);
#else
  int id_err = chain1.verifyIDs(left_leg_ids, left_leg_length, &SerialUSB);
#endif
  if (id_err) {
    // TODO: Handle ID verification errors
  }
  chain1.torqueON(left_leg_ids, left_leg_length);
}

void loop() {
  chain1.setServoPositions(left_leg_ids, left_leg_goal_positions,
                           left_leg_length - 1);

  chain1.getServoData(left_leg_ids, left_leg_feedback, left_leg_length);
}