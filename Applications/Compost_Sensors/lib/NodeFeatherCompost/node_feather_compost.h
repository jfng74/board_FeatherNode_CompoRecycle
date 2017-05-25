#ifndef NODE_FEATHER_COMPOST_H
#define NODE_FEATHER_COMPOST_H

#include <SPI.h>
#include <Wire.h>
#include <DS3231.h>
#include <RH_RF95.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_BME280.h>
#include <RTCZero.h>
#include "NTC_config.h"

/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define VBATPIN A7
#define NTC_1 A0
#define NTC_2 A1
//#define BME280_ADDRESS 0x76

#define SEALEVELPRESSURE_HPA (1013.25)

class NodeFeatherCompost{
public:
  NodeFeatherCompost();
  void send_batt_voltage(void);
  void send_all_data(void);
  void send_node_ready(void);
  void send_temp(uint8_t analog_pin);
  void send_humidity(void);
  void power_off(void);
  void blink_led(uint8_t nb_flash, uint32_t delais);
  void parse_data(uint8_t Thebuf[]);
  void loop(void);

private:
  Adafruit_HTU21DF htu;
  Adafruit_BME280 bme;
  DS3231 clock;
  RTCDateTime dt;

  void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data );
  byte readEEPROM(int deviceaddress, unsigned int eeaddress );

  float read_batt_voltage(void);
  float read_temp(uint8_t analog_pin);
  uint16_t read_conductivite(uint8_t analog_pin);
  byte ReceiveRFData(void);
  bool clock_ok;
  bool htu_ok;
  bool bme_ok;
  RTCZero rtc;
  RH_RF95 *rf95;
//  RH_RF95 rf95_jf(RFM95_CS, RFM95_INT);
  int samples[NUMSAMPLES];
  char radiopacket[35];
  // Should be a message for us now
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t txpower;

  float ntc_1;
  float ntc_2;
  float batt_voltage;
  float bme_humidity;
  float bme_pression;
  float bme_temp;
  uint16_t conductivite;
  int i;
  uint8_t delay_minutes;

};

#endif
