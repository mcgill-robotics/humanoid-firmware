#include <Arduino.h>

#include "HardwareSerial.h"
#include "ServoChain.h"

#define DEBUG_PRINT 0

// SERVO CONFIGURATION
XL320Chain robot_bus(9, &Serial1);
char rgb[] = "rgbypcwo";
int servo_id1 = 20;
int servo_id2 = 21;
int servo_id3 = 22;
int servo_id4 = 23;
int servo_id5 = 19;
int led_color = 0;
unsigned short servo_setpoint_raw[4] = {0};
float servo_setpoint_deg[4] = {0};
unsigned short servo_pos_raw[4] = {0};
float servo_pos_deg[4] = {0};
uint8_t servo_ids[5] = {servo_id1, servo_id2, servo_id3, servo_id4, servo_id5};

// union pressure_data
// {
//   uint8_t bytes[8];
//   uint16_t shorts[4] = {0, 0, 0, 0};
//   struct
//   {
//     uint16_t left_front : 10;
//     uint16_t left_back : 10;
//     uint16_t right_front : 10;
//     uint16_t right_back : 10;
//   };
// } pressure_meas;

uint8_t pressure_meas[4];

float map_float(float x, float in_min, float in_max, float out_min,
                float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float raw2deg(float raw) { return map_float(raw, 0, 1023, 0, 300); }

float deg2raw(float deg) { return map_float(deg, 0, 300, 0, 1023); }

bool validID(int id) { return (id >= 1 && id <= 253 && id != 200); }

bool validBaud(int newBaud) { return (newBaud >= 0 && newBaud <= 3); }

float servoSine(float offset, float amplitude, float w, float time_offset)
{
  float time = millis() / 1000.0 + time_offset;
  return offset + amplitude * sin(w * time);
}

void servo_setup()
{
  robot_bus.begin();

  servo_setpoint_raw[0] = 512;
  servo_setpoint_raw[1] = 512;
  servo_setpoint_raw[2] = 512;
  servo_setpoint_raw[3] = 512;
  servo_setpoint_deg[0] = raw2deg(servo_setpoint_raw[0]);
  servo_setpoint_deg[1] = raw2deg(servo_setpoint_raw[1]);
  servo_setpoint_deg[2] = raw2deg(servo_setpoint_raw[2]);
  servo_setpoint_deg[3] = raw2deg(servo_setpoint_raw[3]);

  robot_bus.torqueON(servo_ids, 4);
}

void servo_loop()
{
  // servo_setpoint_deg[0] = servoSine(150, 50, 1.59, 0);
  // servo_setpoint_deg[1] = servoSine(150, 50, 1.59, -5);
  // servo_setpoint_deg[2] = servoSine(150, 20, 1.59, -7.5);
  // servo_setpoint_deg[3] = servoSine(150, 20, 1.59, -2.5);
  // servo_setpoint_raw[0] = deg2raw(servo_setpoint_deg[0]);
  // servo_setpoint_raw[1] = deg2raw(servo_setpoint_deg[1]);
  // servo_setpoint_raw[2] = deg2raw(servo_setpoint_deg[2]);
  // servo_setpoint_raw[3] = deg2raw(servo_setpoint_deg[3]);

  robot_bus.setServoPositions(servo_ids, servo_setpoint_raw, 4);

#if DEBUG_PRINT == 1
  p.toStream(SerialUSB);
#endif

  unsigned short rcv_buf[255];
  robot_bus.getServoData(servo_ids, rcv_buf, 5, &SerialUSB);
  // robot_bus.sendPacket(servo_id5, 0x02, nullptr, 0);
  // Serial1.flush();
  // uint8_t rec_buf[255];
  // if (robot_bus.readPacket(rec_buf, 255) > 0)
  // {
  //   SerialUSB.println("Valid!");
  // }
  // else
  // {
  //   SerialUSB.println("Invalid!");
  // }
  servo_pos_raw[0] = rcv_buf[0];
  servo_pos_raw[1] = rcv_buf[3];
  servo_pos_raw[2] = rcv_buf[6];
  servo_pos_raw[3] = rcv_buf[9];
  pressure_meas[0] = rcv_buf[12] && 0xFF;
  pressure_meas[1] = rcv_buf[12] >> 8;
  pressure_meas[2] = rcv_buf[13] && 0xFF;
  pressure_meas[3] = rcv_buf[13] >> 8;
  servo_pos_deg[0] = map_float(servo_pos_raw[0], 0, 1023, 0, 300.0);
  servo_pos_deg[1] = map_float(servo_pos_raw[1], 0, 1023, 0, 300.0);
  servo_pos_deg[2] = map_float(servo_pos_raw[2], 0, 1023, 0, 300.0);
  servo_pos_deg[3] = map_float(servo_pos_raw[3], 0, 1023, 0, 300.0);
}

uint32_t last_print = 0;
void print_servo_state()
{
  // Print the servo state
  SerialUSB.print("Servo ID: ");
  SerialUSB.print(servo_id1);
  SerialUSB.print(", setpoint_deg: ");
  SerialUSB.print(servo_setpoint_deg[0]);
  SerialUSB.print(", pos_deg: ");
  SerialUSB.print(servo_pos_deg[0]);
  SerialUSB.print(", setpoint_raw: ");
  SerialUSB.print(servo_setpoint_raw[0]);
  SerialUSB.print(", pos_raw: ");
  SerialUSB.println(servo_pos_raw[0]);

  SerialUSB.print("Servo ID: ");
  SerialUSB.print(servo_id2);
  SerialUSB.print(", setpoint_deg: ");
  SerialUSB.print(servo_setpoint_deg[1]);
  SerialUSB.print(", pos_deg: ");
  SerialUSB.print(servo_pos_deg[1]);
  SerialUSB.print(", setpoint_raw: ");
  SerialUSB.print(servo_setpoint_raw[1]);
  SerialUSB.print(", pos_raw: ");
  SerialUSB.println(servo_pos_raw[1]);

  SerialUSB.print("Servo ID: ");
  SerialUSB.print(servo_id3);
  SerialUSB.print(", setpoint_deg: ");
  SerialUSB.print(servo_setpoint_deg[2]);
  SerialUSB.print(", pos_deg: ");
  SerialUSB.print(servo_pos_deg[2]);
  SerialUSB.print(", setpoint_raw: ");
  SerialUSB.print(servo_setpoint_raw[2]);
  SerialUSB.print(", pos_raw: ");
  SerialUSB.println(servo_pos_raw[2]);

  SerialUSB.print("Servo ID: ");
  SerialUSB.print(servo_id4);
  SerialUSB.print(", setpoint_deg: ");
  SerialUSB.print(servo_setpoint_deg[3]);
  SerialUSB.print(", pos_deg: ");
  SerialUSB.print(servo_pos_deg[3]);
  SerialUSB.print(", setpoint_raw: ");
  SerialUSB.print(servo_setpoint_raw[3]);
  SerialUSB.print(", pos_raw: ");
  SerialUSB.println(servo_pos_raw[3]);

  SerialUSB.print("Pressure ID: ");
  SerialUSB.print(servo_id5);
  SerialUSB.print(", pressure1: ");
  SerialUSB.print(pressure_meas[0]);
  SerialUSB.print(", pressure2: ");
  SerialUSB.print(pressure_meas[1]);
  SerialUSB.print(", pressure3: ");
  SerialUSB.print(pressure_meas[2]);
  SerialUSB.print(", pressure4: ");
  SerialUSB.println(pressure_meas[3]);

  SerialUSB.println();
}

void setup()
{
  SerialUSB.begin(115200);
  while (!SerialUSB)
    ;
  SerialUSB.println("Detected usb serial!");
  servo_setup();
  SerialUSB.println("Done servo setup");

  // bool valid = robot_bus.validateIDs(servo_ids, 4, nullptr);
  // SerialUSB.println("First validate run");
  // while (!valid)
  // {
  //   SerialUSB.println("IDs are not valid");
  //   delay(1000);
  //   valid = robot_bus.validateIDs(servo_ids, 4, nullptr);
  // }
  // SerialUSB.println("IDs are valid");
  delay(3000);
  SerialUSB.println("Starting Loop");
}

void loop()
{
  servo_loop();
  if (millis() - last_print > 250)
  {
    // print_servo_state();
    last_print = millis();
    // SerialUSB.print("Pressure ID: ");
    // SerialUSB.print(servo_id5);
    // SerialUSB.print("Raw: ");
    // SerialUSB.print(pressure_meas.bytes[0], BIN);
    // SerialUSB.print(" ");
    // SerialUSB.print(pressure_meas.bytes[1], BIN);
    // SerialUSB.print(" ");
    // SerialUSB.print(pressure_meas.bytes[2], BIN);
    // SerialUSB.print(" ");
    // SerialUSB.print(pressure_meas.bytes[3], BIN);
    // SerialUSB.print(" ");
    // SerialUSB.print(pressure_meas.bytes[4], BIN);
    // SerialUSB.print(" ");
    // SerialUSB.print(pressure_meas.bytes[5], BIN);
    // SerialUSB.print(" ");
    // SerialUSB.print(pressure_meas.bytes[6], BIN);
    // SerialUSB.print(" ");
    // SerialUSB.println(pressure_meas.bytes[7], BIN);
    // SerialUSB.print(", pressure1: ");
    // SerialUSB.print(pressure_meas.left_front);
    // SerialUSB.print(", pressure2: ");
    // SerialUSB.print(pressure_meas.left_back);
    // SerialUSB.print(", pressure3: ");
    // SerialUSB.print(pressure_meas.right_front);
    // SerialUSB.print(", pressure4: ");
    // SerialUSB.println(pressure_meas.right_back);
  }
  // delay(100);
  // robot_bus.sendPacket(servo_id5, 0x02, nullptr, 0);
  // byte txbuffer[20];
  // XL320Chain::Packet p(txbuffer, 20, 19, 0x02, 0, nullptr);
  // Serial1.write(txbuffer, p.getSize());
  // while (Serial1.available())
  // {
  //   SerialUSB.write(Serial1.read());
  // }
  // Serial1.flush();
  // delayMicroseconds(50);
  // byte rxbuffer[255];
  // robot_bus.readPacket(rxbuffer, 255);
  // XL320Chain::Packet s(rxbuffer, 255);
  // pressure_meas.bytes[0] = s.getParameter(1);
  // pressure_meas.bytes[1] = s.getParameter(2);
  // pressure_meas.bytes[2] = s.getParameter(3);
  // pressure_meas.bytes[3] = s.getParameter(4);
  // pressure_meas.bytes[4] = s.getParameter(5);
  // pressure_meas.bytes[5] = s.getParameter(6);
  delay(8);
}
