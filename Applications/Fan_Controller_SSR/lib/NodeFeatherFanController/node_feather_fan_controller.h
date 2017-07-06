#ifndef NODE_FEATHER_FAN_CONTROLLER_H
#define NODE_FEATHER_FAN_CONTROLLER_H

#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>
#include <DS3231.h>
#include <RH_RF95.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_BME280.h>
#include <RTCZero.h>

#define NODE_SSR_ADDR 0xFE

#define NB_NODES 4
#define MAX_NODE_TEXT 16

#define MAX_ASK_ALL_DATA_RETRY 3
#define DELAY_READY_FOR_COMMAND 15000
#define OPERATION_TIME 5

/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define VBATPIN A7

// Definition des pins pour le SSR
#define SSR_1_PIN 18
#define SSR_2_PIN 19
#define LED_RED_PIN 11
#define LED_GREEN_PIN 12

#define I2C_EEPROM_ADDRESS 0x50
#define EEPROM_INIT_VALUE 0xff

#define EEPROM_ADDR_INIT 0x00

#define EEPROM_NODE_FAN_CONFIG 0x01

#define EEPROM_ADDR_SSR_PC1 (EEPROM_ADDR_INIT + 1)
#define EEPROM_ADDR_SSR_PC2 (EEPROM_ADDR_SSR_PC1 + 4)
#define EEPROM_ADDR_SSR_PC3 (EEPROM_ADDR_SSR_PC2 + 4)
#define EEPROM_ADDR_SSR_PC4 (EEPROM_ADDR_SSR_PC3 + 4)

#define EEPROM_ADDR_SSR_TV1 (EEPROM_ADDR_SSR_PC4 + 4)
#define EEPROM_ADDR_SSR_TV2 (EEPROM_ADDR_SSR_TV1 + 1)
#define EEPROM_ADDR_SSR_TV3 (EEPROM_ADDR_SSR_TV2 + 1)
#define EEPROM_ADDR_SSR_TV4 (EEPROM_ADDR_SSR_TV3 + 1)

#define EEPROM_ADDR_SSR_TA1 (EEPROM_ADDR_SSR_TV4 + 1)
#define EEPROM_ADDR_SSR_TA2 (EEPROM_ADDR_SSR_TA1 + 1)
#define EEPROM_ADDR_SSR_TA3 (EEPROM_ADDR_SSR_TA2 + 1)
#define EEPROM_ADDR_SSR_TA4 (EEPROM_ADDR_SSR_TA3 + 1)

#define EEPROM_ADDR_MINUTES_DELAY (EEPROM_ADDR_SSR_TA4 + 1)

#define EEPROM_ADDR_COMPOST_NODE_00_CONFIG (EEPROM_ADDR_MINUTES_DELAY + 1)
#define EEPROM_ADDR_COMPOST_NODE_01_CONFIG (EEPROM_ADDR_MINUTES_DELAY + 2)
#define EEPROM_ADDR_COMPOST_NODE_02_CONFIG (EEPROM_ADDR_MINUTES_DELAY + 3)
#define EEPROM_ADDR_COMPOST_NODE_03_CONFIG (EEPROM_ADDR_MINUTES_DELAY + 4)

#define EEPROM_ADDR_COMPOST_NODE_00_ADDRESS (EEPROM_ADDR_COMPOST_NODE_03_CONFIG + 1)
#define EEPROM_ADDR_COMPOST_NODE_01_ADDRESS (EEPROM_ADDR_COMPOST_NODE_03_CONFIG + 2)
#define EEPROM_ADDR_COMPOST_NODE_02_ADDRESS (EEPROM_ADDR_COMPOST_NODE_03_CONFIG + 3)
#define EEPROM_ADDR_COMPOST_NODE_03_ADDRESS (EEPROM_ADDR_COMPOST_NODE_03_CONFIG + 4)

#define PC0_ID 0
#define PC1_ID 1
#define PC2_ID 2
#define PC3_ID 3
#define PC4_ID 4

#define SSR_DEFAULT_PC1 20
#define SSR_DEFAULT_PC2 30
#define SSR_DEFAULT_PC3 40
#define SSR_DEFAULT_PC4 50

#define SSR_DEFAULT_TV1 20
#define SSR_DEFAULT_TV2 30
#define SSR_DEFAULT_TV3 40
#define SSR_DEFAULT_TV4 50

#define SSR_DEFAULT_TA1 20
#define SSR_DEFAULT_TA2 30
#define SSR_DEFAULT_TA3 40
#define SSR_DEFAULT_TA4 50

#define SSR_DEFAULT_MINUTE_DELAY 1

#define SET_T_AVG_SURFACE 1
#define SET_T_AVG_PROFONDEUR 2

#define SSR_DEFAULT_NODE_COMPOST_00_CONFIG (SET_T_AVG_PROFONDEUR | SET_T_AVG_SURFACE)
#define SSR_DEFAULT_NODE_COMPOST_01_CONFIG (SET_T_AVG_PROFONDEUR | SET_T_AVG_SURFACE)
#define SSR_DEFAULT_NODE_COMPOST_02_CONFIG (SET_T_AVG_PROFONDEUR | SET_T_AVG_SURFACE)
#define SSR_DEFAULT_NODE_COMPOST_03_CONFIG (SET_T_AVG_PROFONDEUR | SET_T_AVG_SURFACE)

#define SSR_DEFAULT_NODE_COMPOST_00_ADDR 0x00
#define SSR_DEFAULT_NODE_COMPOST_01_ADDR 0x01
#define SSR_DEFAULT_NODE_COMPOST_02_ADDR 0x02
#define SSR_DEFAULT_NODE_COMPOST_03_ADDR 0x03


#define EEPROM_INIT_VALUE 0xff
#define EEPROM_MAX_ADDRESS 0xff



// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0

struct node_fan_config{
    float PC1;
    float PC2;
    float PC3;
    float PC4;
    uint16_t TV1;
    uint16_t TV2;
    uint16_t TV3;
    uint16_t TV4;
    uint16_t TA1;
    uint16_t TA2;
    uint16_t TA3;
    uint16_t TA4;
    uint8_t delais_minute;
    uint8_t node_compost_addr[4];
    uint8_t node_compost_cfg[4];
    char node_compost_text[4][16];
};

struct node_data {
  uint8_t node_address;
  uint8_t node_cfg;
  uint32_t timestamp;
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

struct CompostNodeData{
  uint8_t node_address;
  uint8_t node_cfg;
  uint32_t timestamp;
  float ntc_1;
  float ntc_2;
  float bme_humidity;
  float bme_pression;
  float bme_temp;
  uint16_t conductivite;
  float batt_voltage;
  uint8_t delay_minutes;
  uint8_t txpower;
  int8_t last_rssi;
  uint8_t new_data;
  uint8_t clock_ok;
};

struct SsrData{
  float t_avg;
  uint8_t current_PC;
  uint8_t ssr_state;
};

class NodeFeatherFanController{
public:
  NodeFeatherFanController();
  void initialisation(void);
  void loop(void);
  void readEEpromConfiguration(void);

private:
  DS3231 clock;
  RTCDateTime dt;
  node_fan_config nfc;
  RH_RF95 *rf95;
  char radiopacket[200];
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  SsrData ssrd;
  CompostNodeData cnd;
  CompostNodeData cnd_array[NB_NODES];
  uint8_t compost_node_new_data[NB_NODES];
  uint8_t compost_node_clock_ok[NB_NODES];
  node_data nodes_data[4];
//  byte float_array_t_consigne[4];
  float temp_avg[NB_NODES*2];

//  uint8_t delay_minutes;


  uint8_t i_t_avg;
  uint8_t i_node;

//  float ssr_setpoint;
  uint8_t ssr_etat;
  bool ssr_ModeAuto;
  bool do_avg;
  //Variables temporaires pour fonctions
  uint8_t f_minutes_alarme;
  float t_consigne;
  float ssr_setpoint;

  byte alarmHours;
  byte alarmMinutes;
  byte alarmSeconds;

  void clear_compost_nodes_new_data(void);
  void clear_compost_nodes_clock_ok(void);
  byte readEEPROM(int deviceaddress, unsigned int eeaddress );
  void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data );
  void writeEEPROM_NFCConfig(void);
  void readEEPROM_NFCConfig(void);
  void print_NFC(void);
  void resetEEPROM(int deviceaddress);
  void readAllEEPROM(int deviceaddress);
  void eeprom_initialisation(void);
  byte ReceiveRFData(void);
  void parseRF_data(void);
  int16_t getNodeID(uint8_t node_adress);
  void SendAllCfg(void);
  void SendAllTemp(void);
  void SendSetDateTime(byte node_adress);
  void SendSSRReady(byte node_adress);
  void SendSetpoint (byte node_address);
  void SendBatVoltage(byte node_address);
  void SendCompostNodeData(byte node_address, byte compost_node_id);
  void SendSsrNodeData(byte node_address);
  float ReadBattVoltage(void);
  void setFanMotor(bool motor_state);
  void set_current_PC(void);
  void SendRelayState(byte node_address);
  void SendLastRssi (byte node_address);
  void clear_temp_avg(void);
  void blink_led(uint8_t nb_flash, uint32_t delais);
  void SendSSR_ready_for_commands(byte node_address);
  void SendGetDateTime(byte node_address);
  void setRTCAlarm(byte seconds);
  void resetAlarm(void);
};

#endif
