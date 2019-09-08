#include "RoboteqSerial.hpp"

RoboteqSerial roboteq{Serial};

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Get amp of channel 1
  int channel1Amps = roboteq.readMotorAmps(1);

  // Get amp of channel 2
  int channel2Amps = roboteq.readMotorAmps(2);
}
