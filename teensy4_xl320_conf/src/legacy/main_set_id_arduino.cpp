// #include "common.h"
// #if COMPILE_CFG == COMPILE_CFG_SET_ID
// // ========================================
// // Dynamixel XL-320 Arduino library example
// // ========================================

// // Read more:
// // https://github.com/hackerspace-adelaide/XL320
// #include <Arduino.h>
// #include <SoftwareSerial.h>
// #include "XL320.h"

// // Name your robot!

// #define SERVO_BAUD 1000000
// XL320 robot;
// int servoID = 254;

// SoftwareSerial xl320Serial(A0, A1);

// void setup()
// {
//   Serial.begin(115200);

//   // Talking standard serial, so connect servo data line to Digital TX 1
//   // Set the default servo baud rate which is 1000000 (1Mbps) if it's a brand new servo
//   // Serial1.begin(SERVO_BAUD, SERIAL_8N1_HALF_DUPLEX);
//   // Serial1.begin(9600, SERIAL_8N1_HALF_DUPLEX);

//   xl320Serial.begin(9600);  // Adjust baud rate as needed for your setup
//   robot.begin(xl320Serial); // Hand in the serial object you're using
//   delay(100);

//   // Current servoID

//   // NOTE: comment out either the XL_BAUD_RATE or XL_ID, only send one at a time

//   // ===================================
//   // Set the serial connection baud rate
//   // ===================================

//   // writePacket(1, XL_BAUD_RATE, x) sets the baud rate:
//   // 0: 9600, 1:57600, 2:115200, 3:1Mbps
//   // robot.sendPacket(servoID, XL_BAUD_RATE, 3);

//   // ================
//   // Set the servo ID
//   // ================

//   // writePacket(1, XL_ID, x) sets the ID:
//   // ID can be between 1 and 253 (but not 200)
//   robot.sendPacket(servoID, XL_ID, 10);
// }

// void loop()
// {
//   Serial.println("Looping");
//   digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
//   robot.sendPacket(servoID, XL_ID, 10);
//   delay(1000);
//   // NOTE: load this sketch to the Arduino > Servo
//   // Then power cycle the Arduino + Servo

//   // NOTE: When setting the servo ID, the baud rate defaults down to 9600
//   // So then you'll need to power cycle, and re-write the baud rate to whatever you want via 9600
// }
// #endif // COMPILE_CFG == COMPILE_CFG_SET_ID