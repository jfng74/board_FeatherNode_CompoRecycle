#ifndef NODE_FEATHER_FAN_CONTROLLER_H
#define NODE_FEATHER_FAN_CONTROLLER_H

#include <SPI.h>
#include <Wire.h>
#include <DS3231.h>
#include <RH_RF95.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_BME280.h>
#include <RTCZero.h>


#define NB_NODES 4
#define NODE_00 0x00
#define NODE_01 0x01
#define NODE_02 0x02
#define NODE_03 0x03

#define PC_1 1
#define PC_2 2
#define PC_3 3
#define PC_4 4

#define MAX_ASK_ALL_DATA_RETRY 3
#define DELAY_READY_FOR_COMMAND 15000

#define SSR_1_PIN 18
#define SSR_2_PIN 19

#define OPERATION_TIME 5



/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define VBATPIN A7

#define disk1 0x50

#define NODE_ADDR 0xFE

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0

struct node_data {
  uint8_t node_address;
  float temp[4];
  float humidity_1;
  float pression;
  uint16_t conductivite;
  float battery_voltage;
  int8_t last_rssi;
  uint8_t txpower;
  bool new_data_received;
  bool clock_ok=false;
};

class NodeFeatherFanController{
public:
  NodeFeatherFanController();
  void initialisation(void);
  void readEEpromConfiguration(void);
  void loop(void);
private:
  DS3231 clock;
  RTCDateTime dt;
  RH_RF95 *rf95;
  char radiopacket[140];
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  node_data nodes_data[4];
  byte float_array_t_consigne[4];
  float temp_avg[8];

  uint8_t minutes_alarme;
  uint8_t delay_minutes;
  float setpoint;
  uint8_t relais_etat;
  float t_consigne;
  float t_avg;
  uint8_t i_t_avg;
  uint8_t i_node;
  bool ModeAuto;

  /* Point de consignes (en degres Celsius)*/
  float PC1;
  float PC2;
  float PC3;
  float PC4;

  /* Temps de ventilation (en secondes)*/
  uint8_t TV1;
  uint8_t TV2;
  uint8_t TV3;
  uint8_t TV4;

  /* Temps d'arret de ventilation */
  uint8_t TA1;
  uint8_t TA2;
  uint8_t TA3;
  uint8_t TA4;






  void clear_nodes_data(void);
  byte readEEPROM(int deviceaddress, unsigned int eeaddress );
  void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data );
  void eeprom_initialisation(void);
  byte ReceiveRFData(void);
  void parseRF_data(void);
  uint8_t getNodeID(uint8_t node_adress);
  void SendSetDateTime(byte node_adress);
  void SendSSRReady(byte node_adress);
  void SendSetpoint (byte node_address);
  void SendBatVoltage(byte node_address);
  float ReadBattVoltage(void);
  void FanMotor(bool motor_state);
  uint8_t ventilation(void);
  void SendRelayState(byte node_address);
  void SendLastRssi (byte node_address);
  void clear_temp_avg(void);
  void blink_led(uint8_t nb_flash, uint32_t delais);
  void SSR_ready_for_commands(byte node_address);
};

#endif
