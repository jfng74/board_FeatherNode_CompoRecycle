#include <Arduino.h>
#include "node_feather_compost.h"
#include "FanCompostMsg.h"

// Node Address
#define NODE_ADDR 0x00

//RFM variables
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0

// RH_RF95 rf95(RFM95_CS, RFM95_INT);

NodeFeatherCompost::NodeFeatherCompost(){
  htu_ok = false;
  bme_ok = false;
  rf95 = new RH_RF95(RFM95_CS, RFM95_INT);
// Initialisation du RTC
//  rtc.begin();
//	rtc.setTime(0,0,0);
//	rtc.setAlarmTime(0,5,0);

  htu_ok = htu.begin();
  if (!htu_ok) {
    Serial.println("Couldn't find sensor!");
  }

  bme_ok  = bme.begin(0x76);
  if (!bme_ok) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
//		while (1);
  }
  // Initialisation de la pin de sortie pour la led rouge et eteint la led
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // Initialisation de la la pin de reset pour le RFM et place la pin en mode non-reset
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Initialisation du port serie a 9600 baud
  Serial.begin(9600);

  // Initialisation du voltage de reference pour le ADC
  analogReference(AR_EXTERNAL);

  // Manual reset pour le RFM
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  // Initialisation du RFM
	while (!rf95->init()) {
		Serial.println("LoRa radio init failed");
		while (1);
	}
	Serial.println("LoRa radio init OK!");

	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	if (!rf95->setFrequency(RF95_FREQ)) {
		Serial.println("setFrequency failed");
		while (1);
	}
	Serial.print("Set Freq to: ");
	Serial.println(RF95_FREQ);
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
	// you can set transmitter powers from 5 to 23 dBm:
	rf95->setTxPower(5, false);
	blink_led(10,100);
}

void NodeFeatherCompost::loop(void){
  if (rf95->available()) {
    if (rf95->recv(buf, &len)) {
      RH_RF95::printBuffer("Received: ", buf, len);
      parse_data(buf);  //  Check et envoie les donnees
      Serial.print("RSSI: ");
      Serial.println(rf95->lastRssi(), DEC);
      //      Serial.println("coucou!");
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
}

void NodeFeatherCompost::send_batt_voltage(void){
  float measuredvbat = analogRead(VBATPIN);
	measuredvbat *= 2;    // we divided by 2, so multiply back
	measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
	measuredvbat /= 1024; // convert to voltage

	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=NODE_ADDR;
	radiopacket[2]=FEATHER_MSG_RESPONSE_DATA;
	radiopacket[3]=READ_BATTERY_VOLTAGE;
	float result;
	result = measuredvbat;
	byte* float_array;
	float_array = (byte*) &result;
	radiopacket[4]= float_array[3];
	radiopacket[5]= float_array[2];
	radiopacket[6]= float_array[1];
	radiopacket[7]= float_array[0];
	radiopacket[8]= FEATHER_MSG_END;
	rf95->send((uint8_t *)radiopacket, 10);
	rf95->waitPacketSent();

}

void NodeFeatherCompost::send_all(void){
  float t_1, t_2, t_3, h_1, batt_voltage;

	// Lecture de la temperature de surface
	t_1 = read_temp(THERMISTORPIN_A0);
	byte* float_array_t_1;
	float_array_t_1 = (byte*) &t_1;

	// Lecture de la temperature de profondeur
	t_2 = read_temp(THERMISTORPIN_A1);
	byte* float_array_t_2;
	float_array_t_2 = (byte*) &t_2;

	// Lecture de la temperature ambiante
	if(htu_ok)
		t_3 = htu.readTemperature();
	else if(bme_ok)
		t_3 = bme.readTemperature();
	else
		t_3 = 0;
	byte* float_array_t_3;
	float_array_t_3 = (byte*) &t_3;

	// Lecture de l'humidite ambiante
	if(htu_ok)
		h_1 = htu.readHumidity();
	else if(bme_ok)
		h_1 = bme.readHumidity();
	else
		h_1 = 0.0;
	byte* float_array_h_1;
	float_array_h_1 = (byte*) &h_1;

	// Lecture du voltage de la batterie
	batt_voltage = read_batt_voltage();
	byte* float_array_batt_voltage;
	float_array_batt_voltage = (byte*) &batt_voltage;

	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_RESPONSE_ALL_DATA;
	radiopacket[2]=NODE_ADDR;
	radiopacket[3]=READ_ALL_DATA;

	// t_1
	radiopacket[4]= float_array_t_1[3];
	radiopacket[5]= float_array_t_1[2];
	radiopacket[6]= float_array_t_1[1];
	radiopacket[7]= float_array_t_1[0];

	// t_2
	radiopacket[8]= float_array_t_2[3];
	radiopacket[9]= float_array_t_2[2];
	radiopacket[10]= float_array_t_2[1];
	radiopacket[11]= float_array_t_2[0];

	// t_3
	radiopacket[12]= float_array_t_3[3];
	radiopacket[13]= float_array_t_3[2];
	radiopacket[14]= float_array_t_3[1];
	radiopacket[15]= float_array_t_3[0];

	// h_1
	radiopacket[16]= float_array_h_1[3];
	radiopacket[17]= float_array_h_1[2];
	radiopacket[18]= float_array_h_1[1];
	radiopacket[19]= float_array_h_1[0];

	// batt_voltage
	radiopacket[20]= float_array_batt_voltage[3];
	radiopacket[21]= float_array_batt_voltage[2];
	radiopacket[22]= float_array_batt_voltage[1];
	radiopacket[23]= float_array_batt_voltage[0];

	radiopacket[24]= FEATHER_MSG_END;

	rf95->send((uint8_t *)radiopacket, sizeof(radiopacket));
	rf95->waitPacketSent();

}

void NodeFeatherCompost::send_temp(uint8_t analog_pin){
  radiopacket[0]=FEATHER_MSG_HEADER;
  radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
  radiopacket[2]=NODE_ADDR;
  radiopacket[3]=TEMP_1;
  float result;
  result = read_temp(analog_pin);
  byte* float_array;
  float_array = (byte*) &result;
  radiopacket[4]= float_array[3];
  radiopacket[5]= float_array[2];
  radiopacket[6]= float_array[1];
  radiopacket[7]= float_array[0];
  radiopacket[8]= FEATHER_MSG_END;
  rf95->send((uint8_t *)radiopacket, sizeof(radiopacket));
  rf95->waitPacketSent();

}

void NodeFeatherCompost::send_humidity(void){
  radiopacket[0]=FEATHER_MSG_HEADER;
  radiopacket[1]=NODE_ADDR;
  radiopacket[2]=FEATHER_MSG_RESPONSE_DATA;
  radiopacket[3]=HUMIDITY_1;
  float result;
  result = htu.readHumidity();
  byte* float_array;
  float_array = (byte*) &result;
  radiopacket[4]= float_array[3];
  radiopacket[5]= float_array[2];
  radiopacket[6]= float_array[1];
  radiopacket[7]= float_array[0];
  radiopacket[8]= FEATHER_MSG_END;
  rf95->send((uint8_t *)radiopacket, 10);
  rf95->waitPacketSent();
}

float NodeFeatherCompost::read_batt_voltage(void){
  float measuredvbat = analogRead(VBATPIN);
	measuredvbat *= 2;    // we divided by 2, so multiply back
	measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
	measuredvbat /= 1024; // convert to voltage
	return measuredvbat;
}

float NodeFeatherCompost::read_temp(uint8_t analog_pin){
	uint8_t i;
	float average;

	// take N samples in a row, with a slight delay
	for (i=0; i< NUMSAMPLES; i++) {
		samples[i] = analogRead(analog_pin);
		delay(10);
	}
	// average all the samples out
	average = 0;
	for (i=0; i< NUMSAMPLES; i++) {
		average += samples[i];
	}
	average /= NUMSAMPLES;

	// convert the value to resistance
	average = 1023 / average - 1;
	average = SERIESRESISTOR / average;

	float steinhart;
	steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
	steinhart = log(steinhart);                  // ln(R/Ro)
	steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
	steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
	steinhart = 1.0 / steinhart;                 // Invert
	steinhart -= 273.15;                         // convert to C
	return steinhart;
}

void NodeFeatherCompost::blink_led(uint8_t nb_flash, uint32_t delais){
	int i;
	for(i=0; i<nb_flash;i++){
		digitalWrite(13, HIGH);
		delay(delais);
		digitalWrite(13, LOW);
		delay(delais);
	}
}

void NodeFeatherCompost::parse_data(uint8_t Thebuf[]){
	if(Thebuf[0]==FEATHER_MSG_HEADER && Thebuf[1]==FEATHER_MSG_GET_ALL_DATA && Thebuf[2]==NODE_ADDR && Thebuf[3]==READ_ALL_DATA) {
		Serial.println("sendAll");
		send_all();
	    //blink_led(2,100);
	    blink_led(2,100);
	}

	else if (Thebuf[0]==FEATHER_MSG_HEADER && Thebuf[1]==FEATHER_MSG_REQUEST_ALL_NODE_SLEEP) {
		rtc.setTime(0,0,0);
	    rtc.setAlarmTime(0,Thebuf[2],0);
	    rtc.enableAlarm(rtc.MATCH_HHMMSS);
	    //rtc.attachInterrupt();
	    if(rf95->sleep()){
	    	blink_led(Thebuf[2],100);
	    }
	    rtc.standbyMode();
	}

	else if (Thebuf[0]==FEATHER_MSG_HEADER && Thebuf[1]==FEATHER_MSG_QUERY_DATA && Thebuf[2]==NODE_ADDR && Thebuf[3]==TEMP_1) {
		send_temp(THERMISTORPIN_A0);
		Serial.println("Send temp 1");

    }
	else if (Thebuf[0]==FEATHER_MSG_HEADER && Thebuf[1]==FEATHER_MSG_QUERY_DATA && Thebuf[2]==NODE_ADDR && Thebuf[3]==TEMP_2) {
		send_temp(THERMISTORPIN_A1);
		Serial.println("Send temp 2");
    }
 }
