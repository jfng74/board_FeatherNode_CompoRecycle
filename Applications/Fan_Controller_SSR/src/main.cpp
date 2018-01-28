#define VERSION 13_10_2016

#include "node_feather_fan_controller.h"
// #include "EmonLib.h"             // Include Emon Library
//EnergyMonitor emon1;             // Create an instance

NodeFeatherFanController *node;

void setup(void) {
  node = new NodeFeatherFanController();
//  emon1.current(16, 111.1);
}

void loop(void) {
  node->loop();
//  double Irms = emon1.calcIrms(1480);  // Calculate Irms only

}
