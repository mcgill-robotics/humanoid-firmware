// #include "common.h"
// #if COMPILE_CFG == COMPILE_CFG_SET_ID_BAUD
// // ========================================
// // Dynamixel XL-320 Arduino library example
// // ========================================

// // Read more:
// // https://github.com/hackerspace-adelaide/XL320
// #include <Arduino.h>

// #include "XL320.h"

// // Name your robot!

// #define SERVO_BAUD_LOW 9600
// #define SERVO_BAUD_HIGH 1000000

// XL320 robot;
// int servoID = 254;
// char rgb[] = "rgbypcwo";

// // Desired servo ID and baud rate
// int newServoID = 13;
// int newBaudRate = 3;

// void waitForEnter()
// {
//     SerialUSB.println("POWER CYCLE and PRESS ENTER to continue...");
//     // Clear the serial buffer
//     while (SerialUSB.available() > 0)
//     {
//         SerialUSB.read();
//     }
//     // Wait for the user to press enter
//     while (true)
//     {
//         if (SerialUSB.available() > 0)
//         {
//             char c = SerialUSB.read();
//             SerialUSB.print("Read char: "); // Debug print
//             SerialUSB.println(c);
//             if (c == '\n' || c == '\r')
//             {
//                 SerialUSB.println("Enter detected, continuing...");
//                 break;
//             }
//         }
//     }
//     SerialUSB.flush();
// }

// void setup()
// {
//     while (!SerialUSB)
//         ;
//     SerialUSB.begin(115200);
//     SerialUSB.println("XL-320 Set ID");

// start_setup:
//     // Talking standard serial, so connect servo data line to Digital TX 1
//     // Set the default servo baud rate which is 1000000 (1Mbps) if it's a brand new servo
//     Serial1.begin(SERVO_BAUD_LOW, SERIAL_8N1_HALF_DUPLEX);
//     // Serial1.begin(9600, SERIAL_8N1_HALF_DUPLEX);

//     // Initialise your robot
//     robot.begin(Serial1); // Hand in the serial object you're using

//     SerialUSB.printf("SETTING ID\r\n");
//     // ONE AT A TIME
//     // ================
//     // Set the servo ID
//     // ================
//     // writePacket(1, XL_ID, x) sets the ID:
//     // ID can be between 1 and 253 (but not 200)
//     robot.sendPacket(servoID, XL_ID, 10);
//     delay(200);
//     SerialUSB.printf("\tSet the servo ID to %d (9600)\r\n", newServoID);
//     robot.sendPacket(servoID, XL_ID, newServoID);
//     delay(200);
//     SerialUSB.println("\tChange the Serial1 baud rate to 1Mbps");
//     Serial1.begin(SERVO_BAUD_HIGH, SERIAL_8N1_HALF_DUPLEX); // Start with the new baud rate
//     delay(200);
//     SerialUSB.printf("\tSet the servo ID to %d (1Mbps)\r\n", newServoID);
//     robot.sendPacket(servoID, XL_ID, newServoID);

//     // Prompt for enter key
//     waitForEnter();
//     Serial1.end();
//     SerialUSB.flush();

//     SerialUSB.printf("SETTING BAUD\r\n");
//     // ===================================
//     // Set the serial connection baud rate
//     // ===================================
//     // writePacket(1, XL_BAUD_RATE, x) sets the baud rate:
//     // 0: 9600, 1:57600, 2:115200, 3:1Mbps
//     Serial1.begin(SERVO_BAUD_LOW, SERIAL_8N1_HALF_DUPLEX);
//     delay(200);
//     SerialUSB.println("\tSet the baud rate to 3 (9600)");
//     robot.sendPacket(servoID, XL_BAUD_RATE, 3);
//     delay(200);
//     SerialUSB.println("\tChange the Serial1 baud rate to 1Mbps");
//     Serial1.begin(SERVO_BAUD_HIGH, SERIAL_8N1_HALF_DUPLEX); // Start with the new baud rate
//     delay(200);
//     SerialUSB.println("\tSet the baud rate to 3 again (1Mbps)");
//     robot.sendPacket(servoID, XL_BAUD_RATE, 3);

//     robot.LED(newServoID, &rgb[random(0, 7)]);

//     // Prompt for enter key
//     waitForEnter();
//     Serial1.end();
//     goto start_setup;
// }

// void loop()
// {
//     delay(1000);
//     // NOTE: load this sketch to the Arduino > Servo
//     // Then power cycle the Arduino + Servo

//     // NOTE: When setting the servo ID, the baud rate defaults down to 9600
//     // So then you'll need to power cycle, and re-write the baud rate to whatever you want via 9600
//     digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ^ 1);
//     // SerialUSB.println("Set ID to 10");
//     // robot.sendPacket(servoID, XL_ID, 10);
// }
// #endif // COMPILE_CFG == COMPILE_CFG_SET_ID