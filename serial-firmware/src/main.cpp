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
int led_color = 0;
unsigned short servo_setpoint_raw[4] = {0};
float servo_setpoint_deg[4] = {0};
unsigned short servo_pos_raw[4] = {0};
float servo_pos_deg[4] = {0};
uint8_t servo_ids[4] = {servo_id1, servo_id2, servo_id3, servo_id4};

float map_float(float x, float in_min, float in_max, float out_min,
                float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float raw2deg(float raw) { return map_float(raw, 0, 1023, 0, 300); }

float deg2raw(float deg) { return map_float(deg, 0, 300, 0, 1023); }

bool validID(int id) { return (id >= 1 && id <= 253 && id != 200); }

bool validBaud(int newBaud) { return (newBaud >= 0 && newBaud <= 3); }

float servoSine(float offset, float amplitude, float w, float time_offset) {
  float time = millis() / 1000.0 + time_offset;
  return offset + amplitude * sin(w * time);
}

void servo_setup() {
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

void servo_loop() {
  servo_setpoint_deg[0] = servoSine(150, 50, 1.59, 0);
  servo_setpoint_deg[1] = servoSine(150, 50, 1.59, -5);
  servo_setpoint_deg[2] = servoSine(150, 20, 1.59, -7.5);
  servo_setpoint_deg[3] = servoSine(150, 20, 1.59, -2.5);
  servo_setpoint_raw[0] = deg2raw(servo_setpoint_deg[0]);
  servo_setpoint_raw[1] = deg2raw(servo_setpoint_deg[1]);
  servo_setpoint_raw[2] = deg2raw(servo_setpoint_deg[2]);
  servo_setpoint_raw[3] = deg2raw(servo_setpoint_deg[3]);

  robot_bus.setServoPositions(servo_ids, servo_setpoint_raw, 4);

#if DEBUG_PRINT == 1
  p.toStream(SerialUSB);
#endif

  unsigned short rcv_buf[255];
  robot_bus.getServoData(servo_ids, rcv_buf, 4);
  servo_pos_raw[0] = rcv_buf[0];
  servo_pos_raw[1] = rcv_buf[3];
  servo_pos_raw[2] = rcv_buf[6];
  servo_pos_raw[3] = rcv_buf[9];
  servo_pos_deg[0] = map_float(servo_pos_raw[0], 0, 1023, 0, 300.0);
  servo_pos_deg[1] = map_float(servo_pos_raw[1], 0, 1023, 0, 300.0);
  servo_pos_deg[2] = map_float(servo_pos_raw[2], 0, 1023, 0, 300.0);
  servo_pos_deg[3] = map_float(servo_pos_raw[3], 0, 1023, 0, 300.0);
}

uint32_t last_print = 0;
void print_servo_state() {
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

  SerialUSB.println();
}

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB)
    ;
  SerialUSB.println("Detected usb serial!");
  servo_setup();
  SerialUSB.println("Done servo setup");

  bool valid = robot_bus.validateIDs(servo_ids, 4, nullptr);
  SerialUSB.println("First validate run");
  while (!valid) {
    SerialUSB.println("IDs are not valid");
    delay(1000);
    valid = robot_bus.validateIDs(servo_ids, 4, nullptr);
  }
  SerialUSB.println("IDs are valid");
  delay(3000);
  SerialUSB.println("Starting Loop");
}

void loop() {
  servo_loop();
  if (millis() - last_print > 1500) {
    print_servo_state();
    last_print = millis();
  }
  delay(8);
}
