/*******************************************************************************
 * Copyright 2016 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DXL_SERIAL Serial
#define DEBUG_SERIAL soft_serial
const int DXL_DIR_PIN = 2;     // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
#define DXL_SERIAL Serial
#define DEBUG_SERIAL SerialUSB
const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL SerialUSB
const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
#define DXL_SERIAL Serial3       // OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 22; // OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
// For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
// Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB) // When using OpenRB-150
// OpenRB does not require the DIR control pin.
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL SerialUSB
// const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
const int DXL_DIR_PIN = -1; // DYNAMIXEL Shield DIR PIN
#endif

#define TIMEOUT 10 // default communication timeout 10ms
#define MODEL_NUMBER_ADDR 0
#define MODEL_NUMBER_LENGTH 2

// const uint8_t DEFAULT_DXL_ID = 1;
const uint8_t DEFAULT_DXL_ID = 0xFE;
const float DXL_PROTOCOL_VERSION = 2.0;
// uint32_t baud_rates[] = {9600, 57600, 115200, 1000000, 2000000, 3000000};
uint32_t baud_rates[] = {57600};
size_t num_baud_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);
uint8_t target_id = DEFAULT_DXL_ID;
uint8_t new_id = 100;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup()
{
    // put your setup code here, to run once:

    // Use UART port of DYNAMIXEL Shield to debug.
    DEBUG_SERIAL.begin(115200);
    while (!DEBUG_SERIAL)
        ;

    DEBUG_SERIAL.println("Cycling through baud rates to find DYNAMIXEL...");

    for (size_t i = 0; i < num_baud_rates; i++)
    {
        uint32_t baud_rate = baud_rates[i];
        dxl.begin(baud_rate);
        dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

        DEBUG_SERIAL.print("Trying baud rate: ");
        DEBUG_SERIAL.println(baud_rate);

        // for (int id = 0; id < DXL_BROADCAST_ID; id++)
        // {
        //     // iterate until all ID in each buadrate is scanned.
        //     if (dxl.ping(id))
        //     {
        //         DEBUG_SERIAL.print("ID : ");
        //         DEBUG_SERIAL.print(id);
        //         DEBUG_SERIAL.print(", Model Number: ");
        //         DEBUG_SERIAL.println(dxl.getModelNumber(id));
        //     }
        // }

        if (dxl.ping(target_id) == true)
        {
            DEBUG_SERIAL.print("Ping succeeded at baud rate: ");
            DEBUG_SERIAL.println(baud_rate);
            DEBUG_SERIAL.print("Model Number: ");
            DEBUG_SERIAL.println(dxl.getModelNumber(target_id));

            // Turn off torque when configuring items in EEPROM area
            dxl.torqueOff(target_id);

            // Set a new ID for DYNAMIXEL. Do not use ID 200
            if (dxl.setID(target_id, new_id) == true)
            {

                if (dxl.ping(new_id))
                {
                    DEBUG_SERIAL.print("Can PING, ID has been successfully changed to ");
                    DEBUG_SERIAL.println(new_id);
                    DEBUG_SERIAL.print(", Model Number: ");
                    DEBUG_SERIAL.println(dxl.getModelNumber(new_id));
                }
            }
            else
            {
                DEBUG_SERIAL.print("Failed to change ID to ");
                DEBUG_SERIAL.println(new_id);
            }
            // Exit the loop if ping succeeds
            break;
        }
        else
        {
            DEBUG_SERIAL.println("Ping failed!");
        }
    }

    if (target_id == DEFAULT_DXL_ID)
    {
        DEBUG_SERIAL.println("Failed to communicate with DYNAMIXEL at all baud rates.");
    }

    // Check servo info
    uint16_t model_num_from_read = 0;
    uint16_t model_num_from_table = 0;

    model_num_from_read = dxl.getModelNumber(target_id);
    model_num_from_table = dxl.getModelNumberFromTable(target_id);
    int ret = dxl.read(target_id, MODEL_NUMBER_ADDR, MODEL_NUMBER_LENGTH, (uint8_t *)&model_num_from_read, sizeof(model_num_from_read), TIMEOUT);
    DEBUG_SERIAL.printf("DYNAMIXEL Detected! ret=%d, model_num_from_read=%d, ID=%d, model_num_from_table=%d\n",
                        ret, model_num_from_read, target_id, model_num_from_table);

    ret = dxl.setModelNumber(target_id, XL430_W250);
    model_num_from_read = dxl.getModelNumber(target_id);
    model_num_from_table = dxl.getModelNumberFromTable(target_id);
    uint32_t baud_rate = dxl.p_dxl_port_->getBaud();
    DEBUG_SERIAL.printf("DYNAMIXEL Detected! ret=%d, baud_rate=%d, model_num_from_read=%d, ID=%d, model_num_from_table=%d, model_number_idx_[target_id]=%d\n",
                        ret, baud_rate, model_num_from_read, target_id, model_num_from_table, dxl.model_number_idx_[target_id]);

    dxl.torqueOn(new_id);
}

void loop()
{
    float current_pos_raw = dxl.getPresentPosition(new_id, UNIT_RAW);
    float current_pos_deg = dxl.getPresentPosition(new_id, UNIT_DEGREE);
    DEBUG_SERIAL.printf("current_pos_raw=%f, current_pos_deg=%f\n",
                        current_pos_raw, current_pos_deg);

    // Turn on the LED on DYNAMIXEL
    DEBUG_SERIAL.println("LED ON at 210 deg...");
    dxl.setGoalPosition(new_id, 210.0, UNIT_DEGREE);
    dxl.ledOn(new_id);
    delay(500);

    // Turn off the LED on DYNAMIXEL
    DEBUG_SERIAL.println("LED OFF at 180...");
    dxl.setGoalPosition(new_id, 180.0, UNIT_DEGREE);
    dxl.ledOff(new_id);
    delay(500);
}