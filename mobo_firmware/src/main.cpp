#include <Arduino.h>

#include "MoboError.h"
#include "ServoChain.h"
#include "XL320Command.h"
#include "XL320Config.h"
#include "ros.h"

// defines speed of control loop (default is 10000 for 100Hz)
#define CONTROL_LOOP_PERIOD_US 10000

// ros variables
unsigned long lastTime;
ros::NodeHandle nh;

// ros function declarations
void commandCB();
void configCB();
void throwError(const char* msg, int value);

void setup() { nh.initNode(); }

void loop() {
  // continuous loop here

  if (micros() >= lastTime + CONTROL_LOOP_PERIOD_US) {
    // control loop here (specified speed)
    // ROS messages only sent in here
    lastTime += CONTROL_LOOP_PERIOD_US;
    nh.spinOnce();
  }
}