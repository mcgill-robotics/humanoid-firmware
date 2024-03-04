#include <Arduino.h>
#include "ServoChain.h"

#define LEFT_FORWARDS_PIN 14
#define RIGHT_FORWARDS_PIN 15
#define LEFT_BACKWARDS_PIN 16
#define RIGHT_BACKWARDS_PIN 17

XL320Chain bus;

void setup()
{
  while (!SerialUSB)
    ;
  pinMode(14, INPUT);
  SerialUSB.println("Starting loop");
}

int lastReading = analogRead(14);

void loop()
{
  // put your main code here, to run repeatedly:
  int reading = analogRead(14);
  if (abs(lastReading - reading) >= 10)
  {
    lastReading = reading;
    SerialUSB.print("Reading: ");
    SerialUSB.println(lastReading);
  }
  delay(10);
}