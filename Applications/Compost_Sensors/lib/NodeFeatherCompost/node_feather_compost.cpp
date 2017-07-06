#include <Arduino.h>
#include "node_feather_compost.h"
#include "FanCompostMsg.h"

NodeFeatherCompost::NodeFeatherCompost(){
  htu_ok = false;
  bme_ok = false;
  clock_ok = false;

  cnd.node_address = NODE_ADDR;
  cnd.delay_minutes = 1;
  cnd.txpower = 5;
//  txpower = 5;
//  delay_minutes = 1;

  // Initialisation du port serie a 9600 baud
  Serial1.begin(9600);
  Serial1.println("################################");
  Serial1.print("Initialisation du node : "); Serial1.println(NODE_ADDR);
  rf95 = new RH_RF95(RFM95_CS, RFM95_INT);


// Initialisation du RTC
//  rtc.begin();
//	rtc.setTime(0,0,0);
//	rtc.setAlarmTime(0,5,0);

  bme_ok  = bme.begin(0x76);
  if (!bme_ok) {
    Serial1.println("Could not find a valid BME280 sensor, check wiring!");
//		while (1);
  }
  // Initialisation de la pin de sortie pour la led rouge et eteint la led
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // Initialisation de la la pin de reset pour le RFM et place la pin en mode non-reset
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);


  // Initialisation du voltage de reference pour le ADC
  analogReference(AR_EXTERNAL);

  // Manual reset pour le RFM
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  // Initialisation du RFM
	while (!rf95->init()) {
		Serial1.println("LoRa radio init failed");
		while (1);
	}
	Serial1.println("LoRa radio init OK!");

	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	if (!rf95->setFrequency(RF95_FREQ)) {
		Serial1.println("setFrequency failed");
		while (1);
	}
	Serial1.print("Set Freq to: ");
	Serial1.println(RF95_FREQ);
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
	// you can set transmitter powers from 5 to 23 dBm:
	rf95->setTxPower(cnd.txpower, false);

  // Initialisation des valeurs a partir du EEPROM

	if (readEEPROM(I2C_EEPROM_ADDRESS, 4) == 255)
		{
		Serial1.println("eprom not initialized!");
		// setpoint 22c = 65,176,0,0;
		writeEEPROM(I2C_EEPROM_ADDRESS, 4, cnd.delay_minutes); // delais entre les lectures = 1 minute

	}
	else {
		cnd.delay_minutes = readEEPROM(I2C_EEPROM_ADDRESS, 4);
    Serial1.print("eprom read delay_minutes : "); Serial1.println(cnd.delay_minutes);
	}

  Serial1.println("Fin initialisation");
}

void NodeFeatherCompost::loop(void){

  Serial1.println("Lecture des valeurs");
  cnd.ntc_1 = read_temp(NTC_1);
  cnd.ntc_2 = read_temp(NTC_2);
  cnd.batt_voltage = read_batt_voltage();
  if(bme_ok){
    cnd.bme_humidity = bme.readHumidity();
    cnd.bme_pression = bme.readPressure();
    cnd.bme_temp = bme.readTemperature();
  }
  else{
    cnd.bme_humidity = 0;
    cnd.bme_pression = 0;
    cnd.bme_temp = 0;
  }
  cnd.conductivite = read_conductivite(A3);

  send_node_ready();
  if(ReceiveRFData()){
      parse_rf_data(buf);  //  Check et envoie les donnees
  }
  //send_all_data();
  send_compost_node_data();


//  dt = clock.getDateTime();
//  Serial.println(clock.dateFormat("d-m-Y H:i:s - l", dt));
//  digitalWrite(alarmLED, true);
//  delay(1000);
//  digitalWrite(alarmLED, false);
//  delay(1000);
//  if (clock.isAlarm1(false)) // check if alarm 1 is set
//  { clock.armAlarm1(true); // if yes, then arm it again for next period

/* Enlever les commentaires pour activer le rtc
  Serial1.println("clock.begin");
  clock.begin();
  clock.setBattery(true,false);
  clock.enableOutput(false);
  Serial1.println("clock.setAlarm1");
  clock.setAlarm1(0, 0, 0, 5, DS3231_MATCH_S,true);
*/
//  }

/*
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
  */
//    delay(1000);
  power_off();
  delay(2000);
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


void NodeFeatherCompost::send_compost_node_data(void){
  Serial1.println("send_compost_node_data()");
  byte* cnd_pointer = (byte*)&cnd;
  radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_COMPOST_NODE_DATA;
	radiopacket[2]=NODE_ADDR;
  for(uint16_t i=0;i<sizeof(struct CompostNodeData);i++){
    radiopacket[3+i] = cnd_pointer[i];
  }
  radiopacket[3+sizeof(struct CompostNodeData)] = FEATHER_MSG_END;

  Serial1.print("sending : ");Serial1.println(sizeof(struct CompostNodeData)+3);

  rf95->send((uint8_t *)radiopacket, sizeof(radiopacket));
  rf95->waitPacketSent();
}

void NodeFeatherCompost::send_all_data(void){
  Serial1.println("send_all_data()");
	byte* float_array_ntc_1;
	float_array_ntc_1 = (byte*) &cnd.ntc_1;

  byte* float_array_ntc_2;
	float_array_ntc_2 = (byte*) &cnd.ntc_2;

  byte* float_array_bme_humidity;
	float_array_bme_humidity = (byte*) &cnd.bme_humidity;

  byte* float_array_bme_pression;
	float_array_bme_pression = (byte*) &cnd.bme_pression;

  byte* float_array_bme_temp;
	float_array_bme_temp = (byte*) &cnd.bme_temp;

  byte* uint16_array_conductivite;
	uint16_array_conductivite = (byte*) &cnd.conductivite;

  byte* float_array_batt_voltage;
	float_array_batt_voltage = (byte*) &cnd.batt_voltage;

  Serial1.print("Conductivite : ");Serial1.println(cnd.conductivite);


	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_RESPONSE_ALL_DATA;
	radiopacket[2]=NODE_ADDR;
	radiopacket[3]=READ_ALL_DATA;

	// NTC_1
	radiopacket[4]= float_array_ntc_1[3];
	radiopacket[5]= float_array_ntc_1[2];
	radiopacket[6]= float_array_ntc_1[1];
	radiopacket[7]= float_array_ntc_1[0];

	// NTC_2
	radiopacket[8]= float_array_ntc_2[3];
	radiopacket[9]= float_array_ntc_2[2];
	radiopacket[10]= float_array_ntc_2[1];
	radiopacket[11]= float_array_ntc_2[0];

	// BME_TEMP
	radiopacket[12]= float_array_bme_temp[3];
	radiopacket[13]= float_array_bme_temp[2];
	radiopacket[14]= float_array_bme_temp[1];
	radiopacket[15]= float_array_bme_temp[0];

	// BME_HUMIDITY
	radiopacket[16]= float_array_bme_humidity[3];
	radiopacket[17]= float_array_bme_humidity[2];
	radiopacket[18]= float_array_bme_humidity[1];
	radiopacket[19]= float_array_bme_humidity[0];

	// batt_voltage
	radiopacket[20]= float_array_batt_voltage[3];
	radiopacket[21]= float_array_batt_voltage[2];
	radiopacket[22]= float_array_batt_voltage[1];
	radiopacket[23]= float_array_batt_voltage[0];

  //Pression Atmospherique
  radiopacket[24]= float_array_bme_pression[3];
	radiopacket[25]= float_array_bme_pression[2];
	radiopacket[26]= float_array_bme_pression[1];
	radiopacket[27]= float_array_bme_pression[0];

  //Conductivite
  radiopacket[28]= uint16_array_conductivite[1];
	radiopacket[29]= uint16_array_conductivite[0];

  //TX Power
  radiopacket[30]= cnd.txpower;

	radiopacket[31]= FEATHER_MSG_END;

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

void NodeFeatherCompost::parse_rf_data(uint8_t Thebuf[]){
  Serial1.print("parse_rf_data() : ");
  byte uint32_array[4];

	if (Thebuf[0]==FEATHER_MSG_HEADER && Thebuf[1]==FEATHER_MSG_SET_NODE_DELAY
    && Thebuf[2]==NODE_ADDR){
    Serial1.println("FEATHER_MSG_SET_NODE_DELAY");
		cnd.delay_minutes = Thebuf[3];
    writeEEPROM(I2C_EEPROM_ADDRESS, 4, cnd.delay_minutes);
		Serial1.print("Set new delay minute : ");Serial1.println(cnd.delay_minutes);
  }
  else if(Thebuf[0]==FEATHER_MSG_HEADER && Thebuf[1]==FEATHER_MSG_SET_CLOCK
    && Thebuf[2]==NODE_ADDR && Thebuf[7]==FEATHER_MSG_END){
    Serial1.println("FEATHER_MSG_SET_CLOCK");
    uint32_t timeFromPC;
    uint32_array[3]=buf[3];
    uint32_array[2]=buf[4];
    uint32_array[1]=buf[5];
    uint32_array[0]=buf[6];
    memcpy(&timeFromPC,&uint32_array,sizeof(timeFromPC));
    Serial1.print("timeFromPC : ");Serial1.println(timeFromPC);
    clock.setDateTime(timeFromPC);
    dt = clock.getDateTime();
    Serial1.println(clock.dateFormat("d-m-Y H:i:s",dt));
    clock_ok = true;
  }
  else if(Thebuf[0]==FEATHER_MSG_HEADER && Thebuf[1]==FEATHER_MSG_SSR_READY
    && Thebuf[2]==NODE_ADDR){
    Serial1.println("FEATHER_MSG_SSR_READY");
    if(Thebuf[3]!= cnd.delay_minutes){
      Serial1.println("FEATHER_MSG_SSR_READY : setting delay_minutes");
      cnd.delay_minutes = Thebuf[3];
      writeEEPROM(I2C_EEPROM_ADDRESS, 4, cnd.delay_minutes);
    }
  }
  else if (Thebuf[0]==FEATHER_MSG_HEADER && Thebuf[1]==FEATHER_MSG_INCREASE_RF
    && Thebuf[2]==NODE_ADDR && Thebuf[3]==FEATHER_MSG_END){
    Serial1.println("FEATHER_MSG_INCREASE_RF");
    cnd.txpower++;
    if (cnd.txpower>23){
      cnd.txpower = 5;
    }
    rf95->setTxPower(cnd.txpower, false);
  	Serial1.println("txpower set to:");
    Serial1.println(cnd.txpower);
  }
  else if (Thebuf[0]==FEATHER_MSG_HEADER && Thebuf[1]==FEATHER_MSG_DECREASE_RF
    && Thebuf[2]==NODE_ADDR && Thebuf[3]==FEATHER_MSG_END){
    Serial1.println("FEATHER_MSG_DECREASE_RF");
    cnd.txpower--;
    if (cnd.txpower<5){
      cnd.txpower = 23;
    }
    rf95->setTxPower(cnd.txpower, false);
    Serial1.println("txpower set to:");
    Serial1.println(cnd.txpower);
  }
}

byte NodeFeatherCompost::ReceiveRFData(void){
 	if (rf95->waitAvailableTimeout(3000)) {
 		if (rf95->recv(buf, &len)) {
 			return 1;
 		}
 		else {
 			Serial1.println("recv failed");
 		}
 	}
 	else {
 		Serial1.println("No reply from the node!");
 		return 0;
 	}
 	return 1;
}

uint16_t NodeFeatherCompost::read_conductivite(uint8_t analog_pin){
  return analogRead(analog_pin);
}

void NodeFeatherCompost::power_off(void){
//  Serial1.println("clock.begin");
  clock.begin();
  dt = clock.getDateTime();
  Serial1.println(clock.dateFormat("d-m-Y H:i:s - l", dt));
  clock.setBattery(true,false);
  clock.enableOutput(false);
//  Serial1.println("clock.setAlarm1");
  uint8_t minutes_alarme = dt.minute + cnd.delay_minutes;
  if(minutes_alarme >59)
    minutes_alarme = minutes_alarme - 60;
  clock.setAlarm1(0, 0, minutes_alarme, OPERATION_TIME * NODE_ADDR, DS3231_MATCH_M_S,true);
}

void NodeFeatherCompost::send_node_ready(void){
  Serial1.println("send_node_ready()");
  radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_NODE_READY;
	radiopacket[2]=NODE_ADDR;
  radiopacket[4]= FEATHER_MSG_END;

  rf95->send((uint8_t *)radiopacket, 5);
  rf95->waitPacketSent();

}

void NodeFeatherCompost::writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ){
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();

  delay(5);
}

byte NodeFeatherCompost::readEEPROM(int deviceaddress, unsigned int eeaddress ){
  byte rdata = 0xFF;

  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();

  Wire.requestFrom(deviceaddress,1);

  if (Wire.available()) rdata = Wire.read();

  return rdata;
}
