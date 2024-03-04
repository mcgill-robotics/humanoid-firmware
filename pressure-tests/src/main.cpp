#include <Arduino.h>

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