#define VERSION 13_10_2016

#include "node_feather_fan_controller.h"

NodeFeatherFanController *node;

void setup(void) {
  node = new NodeFeatherFanController();
}

void loop(void) {
  node->loop();
}
