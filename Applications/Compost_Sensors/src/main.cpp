	#define VERSION 13_10_2016

#include "node_feather_compost.h"

NodeFeatherCompost *node;

void setup(void) {
	node = new NodeFeatherCompost();

}

void loop(void) {
	node->loop();
}
