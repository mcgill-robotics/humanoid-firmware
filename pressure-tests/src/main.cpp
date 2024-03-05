#include <Arduino.h>
#include "ServoChain.h"

#define LEFT_FORWARDS_PIN 14
#define RIGHT_FORWARDS_PIN 15
#define LEFT_BACKWARDS_PIN 16
#define RIGHT_BACKWARDS_PIN 17
#define DIRECTION_PIN 4
#define SERIAL_PRESSURE Serial1
// #ifdef LEFT_LEG
#define SERIAL_ID 19
// #else
// #define SERIAL_ID 9
// #endif
#define HALF_DUPLEX_PCB

#define CONTROL_LOOP_US 10000

// union pressure_data
// {
//   uint8_t bytes[8];
//   struct
//   {
//     uint16_t left_front : 10;
//     uint16_t left_back : 10;
//     uint16_t right_front : 10;
//     uint16_t right_back : 10;
//   };
// } pressure_meas;

uint8_t pressure_meas[6];

static unsigned long lastTime;
uint8_t rxbuffer[255];
bool waitingForWrite = false;
int IDsLeft = -1;

XL320Chain bus(9, &Serial1);

void writePressure()
{
  SerialUSB.println("Writing");
  waitingForWrite = false;
  IDsLeft = -1;
  byte txbuffer[20];
  XL320Chain::Packet s(txbuffer, 20, SERIAL_ID, 0x55, 6, pressure_meas);
  SERIAL_PRESSURE.write(txbuffer, s.getSize());
  SERIAL_PRESSURE.flush();
}

void setup()
{
#ifdef HALF_DUPLEX_PCB

#else
  bus.beginFullDuplex();
#endif

  // while (!SerialUSB)
  //   ;
  bus.begin();
  pinMode(LEFT_BACKWARDS_PIN, INPUT);
  pinMode(LEFT_FORWARDS_PIN, INPUT);
  pinMode(RIGHT_BACKWARDS_PIN, INPUT);
  pinMode(RIGHT_FORWARDS_PIN, INPUT);
  analogReadResolution(8);
  // SerialUSB.println("Starting loop");

  lastTime = micros();
  // while (Serial1.available())
  //   Serial1.read();
}

void loop()
{
  while (micros() < lastTime + CONTROL_LOOP_US)
  {
    //
    if (SERIAL_PRESSURE.available())
    {
      int err = bus.readPacket(rxbuffer, 255);
      if (err > 0)
      {
        XL320Chain::Packet p(rxbuffer, 255);
        if (p.isValid())
        {
          SerialUSB.println("Valid PACket");
          switch (p.getInstruction())
          {
          case 0x02: // single read
            if (p.getId() == SERIAL_ID)
              writePressure();
            break;

          case 0x82:                                                          // sync read
            if (p.getParameterCount() == 5 && p.getParameter(5) == SERIAL_ID) // check if only 1 ID
              writePressure();                                                // write if so
            for (int i = 1; i <= p.getParameterCount() - 4; i++)              // find position of ID
            {
              if (p.getParameter(i) == SERIAL_ID)
              {
                SerialUSB.println("Received sync read");
                waitingForWrite = true; // if ID in list, wait for position to write feedback
                IDsLeft = i - 1;
              }
            }
            break;

          case 0x55: // status packet
            if (waitingForWrite)
            {
              SerialUSB.println(IDsLeft);
              IDsLeft -= 1; // every status packet received when wanting to write, decrement
              if (IDsLeft == 0)
              {
                writePressure(); // write when our turn
              }
            }
            break;
          default:
            break;
          }
        }
      }
      else
      {
        SerialUSB.println("bad read");
      }
    }
  }
  lastTime += CONTROL_LOOP_US;

  pressure_meas[0] = analogRead(LEFT_FORWARDS_PIN); // get pressure readings every control loop
  pressure_meas[1] = analogRead(LEFT_BACKWARDS_PIN);
  pressure_meas[2] = analogRead(RIGHT_FORWARDS_PIN);
  pressure_meas[3] = analogRead(RIGHT_BACKWARDS_PIN);
  // if (Serial1.available() >= 10)
  // {
  //   while (Serial1.available())
  //     SerialUSB.println(Serial1.read(), HEX);
  //   SerialUSB.println();
  // }
}