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
  int s = mcu.queryFaultFlags();
  int b = mcu.queryVoltage();

  Serial.print("Fault flag: ");
  Serial.println(s);
  Serial.print("Battery: ");
  Serial.println(b);
  // Serial1.println("?FF");
  // Serial1.flush();
  // delay(20);
  // if (stringComplete)
  // {
  //   int s = bitRead(16, 5);
  //   value = inputString.toInt();

  //   Serial.println(inputString.toInt());

  //   inputString = "";
  //   stringComplete = false;
  // }
}

// void serialEvent1()
// {
//   while (Serial1.available())
//   {
//     // get the new byte:
//     char inChar = (char)Serial1.read();
//     // add it to the inputString:
//     inputString += inChar;
//     // if the incoming character is a newline, set a flag so the main loop can
//     // do something about it:
//     if (inChar == 0x0D)
//     {

//       stringComplete = true;
//     }
//   }
// }

// void serialEvent1()
// {
//   while (Serial1.available())
//   {
//     inputString = Serial1.readStringUntil(0x0D);
//   }
//   Serial.println(inputString);
// }
