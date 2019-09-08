#include "RoboteqSerial.hpp"

RoboteqSerial roboteq{Serial};

void setup() {
  Serial.begin(115200);
}

void loop() {
  
  // Read value of the five volts output
  int volts = roboteq.readVolts(ReadVoltsValue::FiveVoltsOutput);

  // Read BatteryVolts
  int batteryVolts = roboteq.readVolts(ReadVoltsValue::BatteryVolts);

  // Read InternalVolts
  int internalVolts = roboteq.readVolts(ReadVoltsValue::InternalVolts);
}
