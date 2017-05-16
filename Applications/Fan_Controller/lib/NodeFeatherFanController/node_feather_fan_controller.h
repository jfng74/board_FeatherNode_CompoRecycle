 #ifndef NODE_FEATHER_FAN_CONTROLLER_H
#define NODE_FEATHER_FAN_CONTROLLER_H

#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <RTCZero.h>

#define NB_NODES 4
#define MAX_ASK_ALL_DATA_RETRY 3
#define DELAY_READY_FOR_COMMAND 15000

/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define VBATPIN A7

#define disk1 0x50

#define NODE_ADDR 0xFE

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0

class NodeFeatherFanController{
public:

private:

};

#endif
