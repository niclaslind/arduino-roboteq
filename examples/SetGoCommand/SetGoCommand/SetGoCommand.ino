#include "RoboteqSerial.hpp"

RoboteqSerial roboteq{Serial};

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Set channel 1 motor command
  roboteq.goToSpeedOrRelativePosition(1, 1000);

  // Set channel 2 motor command
  roboteq.goToSpeedOrRelativePosition(2, -500);
}
