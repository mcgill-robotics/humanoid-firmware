// #include "arduino_freertos.h"
// #include "avr/pgmspace.h"

// #include "XL320.h"
// #include "HardwareSerial.h"

// // Name your robot!
// XL320 robot;

// // Set some variables for incrementing position & LED colour
// char rgb[] = "rgbypcwo";

// // Set the default servo_id to talk to
// int servo_id = 16;
// int led_color = 0;
// int servo_setpoint_raw = 0;
// float servo_setpoint_deg = 0;
// int servo_pos_raw = 0;
// float servo_pos_deg = 0;

// float map_float(float x, float in_min, float in_max, float out_min, float out_max)
// {
//   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

// static void process_serial_cmd_task(void *)
// {
//   Serial.begin(0);
//   while (true)
//   {
//     static String inputString = "";             // A String to hold incoming data
//     static boolean inputStringComplete = false; // Whether the string is complete

//     while (SerialUSB.available())
//     {
//       char inChar = (char)SerialUSB.read(); // Read each character
//       if (inChar == '\n')
//       {
//         inputStringComplete = true; // If newline, input is complete
//       }
//       else
//       {
//         inputString += inChar; // Add character to input
//       }
//     }

//     if (inputStringComplete)
//     {
//       SerialUSB.print("Received: ");
//       SerialUSB.println(inputString); // Echo the input for debugging

//       // Process the completed command
//       if (inputString.startsWith("i "))
//       {
//         // Increment command
//         float inc_deg = inputString.substring(2).toFloat(); // Extract number
//         servo_setpoint_raw = (servo_setpoint_raw + map_float(inc_deg, 0, 359, 0, 1023));
//         SerialUSB.print("Incremented position by: ");
//         SerialUSB.println(inc_deg);
//       }
//       else if (inputString.startsWith("s "))
//       {
//         // Set command
//         float setpoint_deg = inputString.substring(2).toFloat(); // Extract and set new position
//         servo_setpoint_raw = map_float(setpoint_deg, 0, 359, 0, 1023);
//         SerialUSB.print("Set position to: ");
//         SerialUSB.println(setpoint_deg);
//       }
//       else
//       {
//         SerialUSB.println("Unknown command");
//       }

//       // Clear the string for the next command
//       inputString = "";
//       inputStringComplete = false;
//     }
//   }
// }

// void servo_setup()
// {
//   // Talking standard serial, so connect servo data line to Digital TX 1
//   // Comment out this line to talk software serial
//   Serial1.begin(1000000, SERIAL_8N1_HALF_DUPLEX);

//   // Initialise your robot
//   robot.begin(Serial1); // Hand in the serial object you're using

//   // I like fast moving servos, so set the joint speed to max!
//   robot.setJointSpeed(servo_id, 1023 / 2);
// }

// static void task1(void *)
// {
//   pinMode(arduino::LED_BUILTIN, arduino::OUTPUT);
//   while (true)
//   {
//     digitalWriteFast(arduino::LED_BUILTIN, arduino::LOW);
//     vTaskDelay(pdMS_TO_TICKS(500));

//     digitalWriteFast(arduino::LED_BUILTIN, arduino::HIGH);
//     vTaskDelay(pdMS_TO_TICKS(500));
//   }
// }

// static void task2(void *)
// {
//   Serial.begin(0);
//   while (true)
//   {
//     Serial.println("TICK");
//     vTaskDelay(pdMS_TO_TICKS(1'000));

//     Serial.println("TOCK");
//     vTaskDelay(pdMS_TO_TICKS(1'000));
//   }
// }

// static void servo_loop_task(void *)
// {
//   SerialUSB.begin(0);
//   while (true)
//   {
//     // LED test.. let's randomly set the colour (0-7)
//     robot.LED(servo_id, &rgb[random(0, 7)]);

//     // SETPOINT TEST
//     SerialUSB.printf("setpoint_deg=%d, ", servo_setpoint_raw);
//     SerialUSB.printf("Sending %d, ", servo_setpoint_raw);
//     robot.moveJoint(servo_id, servo_setpoint_raw);
//     vTaskDelay(pdMS_TO_TICKS(100));
//     //  Serial1.clear();
//     byte buffer[256];
//     XL320::Packet p = XL320::Packet(buffer, robot.readPacket(buffer, 256));

//     // what is this for???
// #if DEBUG_PRINT == 1
//     p.toStream(SerialUSB);
// #endif

//     vTaskDelay(pdMS_TO_TICKS(250));
//     Serial1.clear();

//     // Get state
//     SerialUSB.printf("Sent %d, ", servo_setpoint_raw);
//     servo_pos_raw = robot.getJointPosition(servo_id);
//     SerialUSB.printf("Received %d, ", servo_pos_raw);
//     // servo_pos_raw = robot.getJointPosition(servo_id);
//     // SerialUSB.printf("Received again %d, ", servo_pos_raw);
//     SerialUSB.println();

//     // Change the servo position by 100 each loop
//     // servo_setpoint_raw = (servo_setpoint_raw + 100) % 1023;

//     vTaskDelay(pdMS_TO_TICKS(250));
//     // Set a delay to account for the receive delay period
//   }
// }

// FLASHMEM __attribute__((noinline)) void setup()
// {
//   SerialUSB.begin(0);
//   delay(2'000);

//   if (CrashReport)
//   {
//     SerialUSB.print(CrashReport);
//     SerialUSB.println();
//     SerialUSB.flush();
//   }

//   SerialUSB.println(PSTR("\r\nBooting FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ " (newlib " _NEWLIB_VERSION ") on " __DATE__ ". ***\r\n"));

//   // servo_setup();

//   // TASKS
//   xTaskCreate(task1, "task1", 128, nullptr, 2, nullptr);
//   xTaskCreate(task2, "task2", 128, nullptr, 2, nullptr);
//   xTaskCreate(servo_loop_task, "servo_loop", 128, nullptr, 2, nullptr);
//   xTaskCreate(process_serial_cmd_task, "process_serial_cmd", 128, nullptr, 2, nullptr);

//   SerialUSB.println("setup(): starting scheduler...");
//   SerialUSB.flush();

//   vTaskStartScheduler();
// }

// void loop()
// {
// }
