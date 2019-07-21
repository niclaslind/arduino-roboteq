#include <Arduino.h>

#include "RoboteqSerial.hpp"

RoboteqSerial mcu(Serial1);

bool stringComplete;
String inputString;
long value;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop()
{
  //int s = mcu.readFaultFlags();
  //int b = mcu.readVoltage();

  Serial.println(mcu.readBrushlessCountRelative(1));
}
