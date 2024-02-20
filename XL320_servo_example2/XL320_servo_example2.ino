
// ========================================
// Dynamixel XL-320 Arduino library example
// ========================================

// Read more:
// https://github.com/hackerspace-adelaide/XL320

#include "HardwareSerial.h"
#include "XL320.h"

// Name your robot!
XL320 robot;

// Set some variables for incrementing position & LED colour
char rgb[] = "rgbypcwo";
int servoPosition = 0;
int ledColour = 0;

// Set the default servoID to talk to
int servoID = 16;
int readPosition = 0;

void setup() {
  // Talking standard serial, so connect servo data line to Digital TX 1
  // Comment out this line to talk software serial
  Serial1.begin(1000000, SERIAL_8N1_HALF_DUPLEX);

  // Initialise your robot
  robot.begin(Serial1);  // Hand in the serial object you're using

  // I like fast moving servos, so set the joint speed to max!
  robot.setJointSpeed(servoID, 1023);
}

void loop() {
  // LED test.. let's randomly set the colour (0-7)
  //  robot.LED(servoID, &rgb[random(0,7)] );

  // LED test.. select a random servoID and colour
  //  robot.LED(random(1,4), &rgb[random(0,7)] );

  // LED colour test.. cycle between RGB, increment the colour and return 1
  // after 3
  //  robot.LED(servoID, &rgb[ledColour]);
  //  ledColour = (ledColour + 1) % 3;

  // Set a delay to account for the receive delay period
  //  delay(100);

  //  readPosition = robot.getJointPosition(servoID);
  //  delay(200);
  //  SerialUSB.println(readPosition);
  // Servo test.. let's randomly set the position (0-1023)
  //  robot.moveJoint(servoID, random(0, 1023));

  // Servo test.. select a random servoID and colour
  //  robot.moveJoint(servoID, random(0, 1023));

  // Servo test.. increment the servo position by 100 each loop
  SerialUSB.println("Sending new pos");
  robot.moveJoint(servoID, servoPosition);
  SerialUSB.println("Done Sending");
  delay(100);
  //  Serial1.clear();
  byte buffer[256];
  XL320::Packet p = XL320::Packet(buffer, robot.readPacket(buffer, 256));
  p.toStream(SerialUSB);
  delay(1000);
  Serial1.clear();
  readPosition = robot.getJointPosition(servoID);
  SerialUSB.println(servoPosition);
  SerialUSB.println(readPosition);
  readPosition = robot.getJointPosition(servoID);
  SerialUSB.println(readPosition);
  servoPosition = (servoPosition + 100) % 1023;

  delay(1000);
  // Set a delay to account for the receive delay period
}
