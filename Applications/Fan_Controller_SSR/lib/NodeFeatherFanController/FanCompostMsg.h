#ifndef FAN_COMPOST_MSG_H
#define FAN_COMPOST_MSG_H

#define FEATHER_MSG_HEADER 0xAA

#define FEATHER_MSG_QUERY_DATA 0x01
#define FEATHER_MSG_RESPONSE_DATA 0x02
#define FEATHER_MSG_SET_DATA 0x03
#define FEATHER_MSG_GET_DATA 0x04
#define FEATHER_MSG_GET_ALL_DATA 0x05
#define FEATHER_MSG_RESPONSE_ALL_DATA 0x06
#define FEATHER_MSG_READY_FOR_COMMANDS  0x07
#define FEATHER_MSG_REQUEST_ALL_NODE_SLEEP 0x08
#define FEATHER_MSG_SEND_ALL_TEMP 0x09
#define FEATHER_MSG_ALL_DATA_READY 0x10
#define FEATHER_MSG_SET_CLOCK 0x11
#define FEATHER_MSG_INCREASE_RF 0x12
#define FEATHER_MSG_DECREASE_RF 0x13
#define FEATHER_MSG_NODE_READY 0x14
#define FEATHER_MSG_SET_NODE_DELAY 0x15
#define FEATHER_MSG_SSR_READY 0x16
#define FEATHER_MSG_GET_DATETIME 0x17
#define FEATHER_MSG_COMPOST_NODE_DATA 0x18
#define FEATHER_MSG_SEND_COMPOST_NODE_DATA 0x19
#define FEATHER_MSG_SEND_SSR_NODE_CFG 0x20
#define FEATHER_MSG_END 0x55

// Definition des variables
#define TEMP_1 0x00
#define TEMP_2 0x01
#define TEMP_3 0x02
#define TEMP_4 0x03
#define HUMIDITY_1 0x04
#define HUMIDITY_2 0x05
#define OXYGEN_1 0x06
#define CO2_1 0x07
#define TURN_ON_RELAY 0x08
#define TURN_OFF_RELAY 0x09
#define RELAY_STATE 0x10
#define MODE_AUTO 0x11
#define READ_BATTERY_VOLTAGE 0x12
#define READ_ALL_DATA 0x13
#define RELAY_THRESHOLD  0x14
#define DELAY_BETWEEN_READS  0x15
#define LAST_RSSI  0x16
#define SEND_ALL_TEMP 0x17
#define SEND_ALL_CFG 0x18
#define RELAIS_CFG 0x19


#endif
