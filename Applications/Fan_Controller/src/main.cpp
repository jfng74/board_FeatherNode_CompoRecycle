#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <DS3231.h>
#include <RH_RF95.h>
#include <LiquidCrystal_PCF8574.h>
#include <stdlib.h>
#include "FanCompostMsg.h"

#define NB_NODES 4
#define OPERATION_TIME 5
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

#define NODE_00 0x00
#define NODE_01 0x01
#define NODE_02 0x02
#define NODE_03 0x03

DS3231 clock;
RTCDateTime dt;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
//LiquidCrystal_PCF8574 lcd(0x3f);
//char lcd_buffer[20];


struct node_data {
  float temp[4];
  float humidity_1;
  float pression;
  uint16_t conductivite;
  float battery_voltage;
  int8_t last_rssi;
  uint8_t txpower;
  bool new_data_received;
};

struct node_data nodes_data[4];

bool ModeAuto = true;
bool Wakeup = true;
float temp_avg[8];
float t_avg;
float t_consigne;
float setpoint;
byte float_array_t_consigne[4];

uint16_t delay_ready_for_command;


uint8_t i;
uint8_t ii;
uint8_t nodes_list[NB_NODES] = {0,1,2,3};
uint8_t nb_retry = 0;
uint8_t relais_etat = 0;
uint8_t delay_minutes=1;
unsigned int eeprom_address = 0;

char radiopacket[76];
// Should be a message for us now
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);


// constants won't change. They're used here to
// set pin numbers:
const int buttonPin = 5;    // the number of the pushbutton pin
const int ledPin = 13;      // the number of the LED pin

// Variables will change:
int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are unsigned long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data );
byte readEEPROM(int deviceaddress, unsigned int eeaddress );
void SendLastRssi (byte node_address);
void print2digits(int number);
void clear_nodes_data(void);
void blink_led(uint8_t nb_flash, uint32_t delais);
void QueryNode(byte node_address);
byte ReceiveRFData(void);
void parseRF_data(byte node_address);
void parseRF_data();
void SetNodesToSleep(byte timetowake);
void SendallDataReady(byte node_address);
bool check_delay_ready_for_command(void);
void SSR_ready_for_commands(byte node_address);
void FanMotor(bool motor_state);
void SendSetpoint (byte node_address);
void SendBatVoltage(byte node_address);
void SendRelayState(byte node_address);
void SendIncreasePower (byte node_address);
void SendSSRReady(byte node_adress);


void setup() {
  // Initialisation du port serie a 9600 baud
	Serial1.begin(9600);
  Serial1.println("Debut Setup");

//  lcd.begin(20,4);


/*
  Serial1.println("clock.begin");
  clock.begin();
  Serial1.println("clock.setAlarm1");
  clock.setAlarm1(0, 0, 0, 5, DS3231_MATCH_M_S,true);
  dt = clock.getDateTime();
  Serial.println(clock.dateFormat("d-m-Y H:i:s - l", dt));
*/

  clock.begin();
  dt = clock.getDateTime();
  clock.setBattery(true,false);
  clock.enableOutput(false);
  //  Serial1.println("clock.setAlarm1");
  uint8_t minutes_alarme = dt.minute + delay_minutes;
  if(minutes_alarme >59)
    minutes_alarme = minutes_alarme - 60;
    clock.setAlarm1(0, 0, minutes_alarme, (OPERATION_TIME * NB_NODES)+5, DS3231_MATCH_M_S,true);

	// Initialisation du delais d'attente pour envoyer un ready for command
	delay_ready_for_command = 0;
	// Initialisation de la pin de sortie pour le SSR
	pinMode(18, OUTPUT);
	digitalWrite(18, LOW); // motor is off SSR is off.
  pinMode(buttonPin, INPUT_PULLUP);
	// Initialisation de la pin de sortie pour la led rouge et eteint la led
	pinMode(ledPin, OUTPUT);
	digitalWrite(ledPin, LOW);

	// Initialisation de la la pin de reset pour le RFM et place la pin en mode non-reset
	pinMode(RFM95_RST, OUTPUT);
	digitalWrite(RFM95_RST, HIGH);
	// manual RFM reset
	digitalWrite(RFM95_RST, LOW);
	delay(10);
	digitalWrite(RFM95_RST, HIGH);
	delay(10);

	while (!rf95.init()) {
		Serial1.println("LoRa radio init failed");
		while (1);
	}
	Serial1.println("LoRa radio init OK!");

	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	if (!rf95.setFrequency(RF95_FREQ)) {
		Serial1.println("setFrequency failed");
	while (1);
	}

	Serial1.print("Set Freq to: ");
	Serial1.println(RF95_FREQ);
	rf95.setTxPower(23, false); // RF power between 5 (min) and 23 (max)


	Wire.begin(); // initialisation du i2c pour eeprom externe
	// Initialisation du voltage de reference pour le ADC
	// analogReference(AR_EXTERNAL);

	setpoint = 20;
	delay_minutes = 1;


	// Initialisation des valeurs a partir du EEPROM

	if (readEEPROM(disk1, 0) == 255)
		{
		Serial1.print("setpoint not initialized!");
		// setpoint 22c = 65,176,0,0;
		writeEEPROM(disk1, 0, 0);
		writeEEPROM(disk1, 1, 0);
		writeEEPROM(disk1, 2, 176);
		writeEEPROM(disk1, 3, 65);
		writeEEPROM(disk1, 4, 1); // delais entre les lectures = 1 minute

	}
	else {
		float_array_t_consigne[3]=readEEPROM(disk1, 3);
		float_array_t_consigne[2]=readEEPROM(disk1, 2);
		float_array_t_consigne[1]=readEEPROM(disk1, 1);
		float_array_t_consigne[0]=readEEPROM(disk1, 0);
		memcpy(&t_consigne,&float_array_t_consigne,sizeof(t_consigne));
		setpoint = t_consigne;
		Serial1.print("New setpoint: ");
		Serial1.println(setpoint);
		delay_minutes = readEEPROM(disk1, 4);
	}

  Serial1.println("Fin Setup");
}

void loop() {
  //Serial1.println("Debut Loop");
  uint8_t i_node=0;


  if(ReceiveRFData()){
    parseRF_data();
  }
  if(clock.isAlarm1(true)){
    Serial1.println("Alarm1 true : calcule moyenne");
    i=0;
    t_avg=0;
    for(i_node=0;i_node<NB_NODES;i_node++){
      if(nodes_data[i].new_data_received){
        temp_avg[i++] = nodes_data[i_node].temp[0];
        temp_avg[i++] = nodes_data[i_node].temp[1];
      }
    }
    for(int j=0;j<i;j++){
      t_avg += temp_avg[j];
    }

    t_avg = t_avg / i;

    Serial1.print("t_avg : ");Serial1.println(t_avg);

    // Active ou desactive le SSR
    if ((setpoint <= t_avg) && (ModeAuto))
    {
      FanMotor(true);
      relais_etat = 1;
    }

    else if ((setpoint > t_avg) && (ModeAuto))
    {
      FanMotor(false);
      relais_etat = 0;
    }
    clear_nodes_data();
    // //  Serial1.println("clock.begin");
      dt = clock.getDateTime();
//      clock.setBattery(true,false);
//      clock.enableOutput(false);
    //  Serial1.println("clock.setAlarm1");
      uint8_t minutes_alarme = dt.minute + delay_minutes;
      if(minutes_alarme >59)
        minutes_alarme = minutes_alarme - 60;
      clock.setAlarm1(0, 0, minutes_alarme, (OPERATION_TIME * NB_NODES) + 5, DS3231_MATCH_M_S,true);

  }
/*
  int reading;
  reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

 if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        SendIncreasePower(0x00);
        Serial1.println("Button press");
        ledState = !ledState;
      }
    }
  }

  // set the LED:
  digitalWrite(ledPin, ledState);

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = reading;
*/

/*

		for(ii=0;ii<NB_NODES;ii++){
			Serial1.print("Query node : ");
			Serial1.println(nodes_list[ii]);
			for(nb_retry = 0;nb_retry < MAX_ASK_ALL_DATA_RETRY;nb_retry++){
				QueryNode(nodes_list[ii]);
				if(ReceiveRFData()){
					parseRF_data(nodes_list[ii]);
					temp_avg[i++] = nodes_data[nodes_list[ii]].temp[TEMP_1];
					temp_avg[i++] = nodes_data[nodes_list[ii]].temp[TEMP_2];
					Serial1.println( temp_avg[i-2] );
					Serial1.println( temp_avg[i-1] );
					Serial1.print("Nb retry : ");
					Serial1.println(nb_retry+1);
					break;
				}
			}
      */
/*

		Serial1.println("Query node 0...");
		QueryNode(0);
		if (ReceiveRFData()){
//			LastRSSI_node_0 = rf95.lastRssi();
			Serial1.print("RSSI 0: ");
			Serial1.println(rf95.lastRssi(), DEC);
			parseRF_data(0);
			temp_avg[i++] = nodes_data[0].temp[TEMP_1];
			temp_avg[i++] = nodes_data[0].temp[TEMP_2];

			Serial1.println( temp_avg[i-2] );
			Serial1.println( temp_avg[i-1] );
		}

		Serial1.println("Query node 1...");
		QueryNode(1);
		if (ReceiveRFData()){
//			LastRSSI_node_1 = rf95.lastRssi();
			Serial1.print("RSSI 1: ");
			Serial1.println(rf95.lastRssi(), DEC);
			parseRF_data(1);
			temp_avg[i++] = nodes_data[1].temp[TEMP_1];
			temp_avg[i++] = nodes_data[1].temp[TEMP_2];
			Serial1.println( temp_avg[i-2] );
			Serial1.println( temp_avg[i-1] );
		}

		Serial1.println("Query node 2...");
		QueryNode(2);
		if (ReceiveRFData()){
//			LastRSSI_node_2 = rf95.lastRssi();
			Serial1.print("RSSI 2: ");
			Serial1.println(rf95.lastRssi(), DEC);
			parseRF_data(2);
			temp_avg[i++] = nodes_data[2].temp[TEMP_1];
			temp_avg[i++] = nodes_data[2].temp[TEMP_2];
			Serial1.println( temp_avg[i-2] );
			Serial1.println( temp_avg[i-1] );
		}

		Serial1.println("Query node 3...");
		QueryNode(3);
		if (ReceiveRFData()){
//			LastRSSI_node_3 = rf95.lastRssi();
			Serial1.print("RSSI 3: ");
			Serial1.println(rf95.lastRssi(), DEC);
			parseRF_data(3);
			temp_avg[i++] = nodes_data[3].temp[TEMP_1];
			temp_avg[i++] = nodes_data[3].temp[TEMP_2];
			Serial1.println( temp_avg[i-2] );
			Serial1.println( temp_avg[i-1] );
		}
*/
//		SetNodesToSleep(DelayBetweenReads);
/*
		for(int j=0;j<i;j++){
			t_avg += temp_avg[j];
		}

		t_avg = t_avg / i;
		Serial1.println("Temperature moyenne :");
		Serial1.println( t_avg );
*/
		Wakeup = false;
//		SendallDataReady(254);
		delay_ready_for_command = 0;
//	}
	// Envoie SSR_READY
/*
  if(	check_delay_ready_for_command()){
		blink_led(2,100);
		SSR_ready_for_commands(254); // send ready for commands to Raspberry
	}

	// Receive RF DATA
	//ReceiveRFData();	// check if there is data coming in from the Raspberry, timeout 3 sec.
	// Parse RF DATA
	//parseRF_data(254); // and parse it


	// Active ou desactive le SSR
	if ((setpoint <= t_avg) && (ModeAuto))
	{
		FanMotor(true);
		relais_etat = 1;
	}

	else if ((setpoint > t_avg) && (ModeAuto))
	{
		FanMotor(false);
		relais_etat = 0;
	}
  */
//  Serial1.println("Fin Loop");

}

bool check_delay_ready_for_command(void){
	delay_ready_for_command += 100;
	if (delay_ready_for_command == DELAY_READY_FOR_COMMAND){
		delay_ready_for_command = 0;
		return true;
	}
	else{
		return false;
	}


}

void clear_nodes_data(void){
	for(int i=0;i<4;i++){
		nodes_data[i].temp[0]=0;
		nodes_data[i].temp[1]=0;
		nodes_data[i].temp[2]=0;
		nodes_data[i].temp[3]=0;
		nodes_data[i].humidity_1=0;
		nodes_data[i].battery_voltage=0;
		nodes_data[i].last_rssi=0;
    nodes_data[i].conductivite=0;
    nodes_data[i].pression=0;
    nodes_data[i].txpower=0;
    nodes_data[i].new_data_received=false;
	}
}

void blink_led(uint8_t nb_flash, uint32_t delais){
	int i;
	for(i=0; i<nb_flash;i++){
		digitalWrite(13, HIGH);
		delay(delais);
		digitalWrite(13, LOW);
		delay(delais);
	}
}


void FanMotor(bool motor_state){
	if (motor_state)
		digitalWrite(18, HIGH);
	else
		digitalWrite(18, LOW);
}

byte ReceiveRFData(void){
	if (rf95.waitAvailableTimeout(100)) {
	// Should be a reply message for us now
		if (rf95.recv(buf, &len)) {
			return 1;
		}
		else {
			Serial1.println("recv failed");
		}
	}
	else {
		//Serial1.println("No reply from the node!");
		return 0;
	}
	delay(400);
	return 1;
}

void parseRF_data(){
  byte float_array[4];
  byte uint16_array[2];
  uint8_t node_id;
  uint16_t conductivite;
	float t_1,t_2,t_3,h_1,batt_voltage,pression;
/*
  Serial1.print("buf[0] : ");
  Serial1.println(buf[0]);
  Serial1.print("buf[1] : ");
  Serial1.println(buf[1]);
  Serial1.print("buf[2] : ");
  Serial1.println(buf[2]);
  Serial1.print("buf[3] : ");
  Serial1.println(buf[3]);
  Serial1.print("buf[4] : ");
  Serial1.println(buf[4]);
  Serial1.print("buf[5] : ");
  Serial1.println(buf[5]);
  Serial1.print("buf[6] : ");
  Serial1.println(buf[6]);
  Serial1.print("buf[7] : ");
  Serial1.println(buf[7]);
  Serial1.print("buf[8] : ");
  Serial1.println(buf[8]);
  Serial1.print("buf[9] : ");
  Serial1.println(buf[9]);
*/
  if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_NODE_READY && buf[3] == FEATHER_MSG_END){
      Serial1.println("FEATHER_MSG_NODE_READY");
      SendSSRReady(buf[2]);
  }
  else if (buf[0]==FEATHER_MSG_HEADER
    && buf[1]==FEATHER_MSG_RESPONSE_ALL_DATA
    && buf[3]==READ_ALL_DATA ) {
      Serial1.println("parseRF_data : FEATHER_MSG_RESPONSE_ALL_DATA");
      dt = clock.getDateTime();
      Serial1.println(clock.dateFormat("d-m-Y H:i:s - l", dt));
      switch (buf[2]){
        case NODE_00:
        node_id = 0;
        break;
        case NODE_01:
        node_id = 1;
        break;
        case NODE_02:
        node_id = 2;
        break;
        case NODE_03:
        node_id = 3;
        break;
      }
      Serial1.print("Node ID : ");Serial1.println(node_id);
      nodes_data[node_id].last_rssi = rf95.lastRssi();
      Serial1.print("LAST_RSSI : ");
      Serial1.println(nodes_data[node_id].last_rssi);
      // NTC_1
      float_array[3]=buf[4];
      float_array[2]=buf[5];
      float_array[1]=buf[6];
      float_array[0]=buf[7];
      memcpy(&t_1,&float_array,sizeof(t_1));
      nodes_data[node_id].temp[0]=t_1;
      Serial1.print("NTC_1 : ");
      Serial1.println(t_1,2);

      // NTC_2
      float_array[3]=buf[8];
      float_array[2]=buf[9];
      float_array[1]=buf[10];
      float_array[0]=buf[11];
      memcpy(&t_2,&float_array,sizeof(t_2));
      nodes_data[node_id].temp[1]=t_2;
      Serial1.print("NTC_2 : ");
      Serial1.println(t_2,2);

      // BME_TEMP
      float_array[3]=buf[12];
      float_array[2]=buf[13];
      float_array[1]=buf[14];
      float_array[0]=buf[15];
      memcpy(&t_3,&float_array,sizeof(t_3));
      nodes_data[node_id].temp[2]=t_3;
      Serial1.print("BME_TEMP : ");
      Serial1.println(t_3,2);

      // BME_HUMIDITY
      float_array[3]=buf[16];
      float_array[2]=buf[17];
      float_array[1]=buf[18];
      float_array[0]=buf[19];
      memcpy(&h_1,&float_array,sizeof(h_1));
      nodes_data[node_id].humidity_1=h_1;
      Serial1.print("BME_HUMIDITY : ");
      Serial1.println(h_1,2);

      // BATT_VOLTAGE
      float_array[3]=buf[20];
      float_array[2]=buf[21];
      float_array[1]=buf[22];
      float_array[0]=buf[23];
      memcpy(&batt_voltage,&float_array,sizeof(batt_voltage));
      nodes_data[node_id].battery_voltage=batt_voltage;
      Serial1.print("BATT_VOLTAGE : ");
      Serial1.println(batt_voltage,2);

      // BME_PRESSION
      float_array[3]=buf[24];
      float_array[2]=buf[25];
      float_array[1]=buf[26];
      float_array[0]=buf[27];
      memcpy(&pression,&float_array,sizeof(pression));
      nodes_data[node_id].pression=pression;
      Serial1.print("BME_PRESSION : ");
      Serial1.println(pression,2);

      // CONDUCTIVITE
      uint16_array[1]=buf[28];
      uint16_array[0]=buf[29];
      memcpy(&conductivite,&uint16_array,sizeof(conductivite));

      nodes_data[node_id].conductivite=conductivite;
      Serial1.print("CONDUCTIVITE : ");
      Serial1.println(conductivite);
      Serial1.print("txpower : ");
      Serial1.println(buf[30]);

//      lcd.setBacklight(255);lcd.home();lcd.clear();
//      sprintf(lcd_buffer, "RSSI: %d TXP: %d" ,nodes_data[node_id].last_rssi, buf[32]);
//      lcd.print(lcd_buffer);

//      lcd.setCursor(0,1);
//      sprintf(lcd_buffer, "T_1: %d T_2: %d" ,(int)t_1, (int)t_2);
//      lcd.print(lcd_buffer);
//      lcd.print("T1: ");lcd.print(t_1,2);lcd.print(" T2: "); lcd.print(t_2,2);

//      lcd.setCursor(0,2);
//      sprintf(lcd_buffer, "T:%d H:%d: P:%d" ,(int)t_3, (int)h_1, (int)pression);
//      lcd.print(lcd_buffer);
//      lcd.print("T3: ");lcd.print(t_3,2);lcd.print(" H1: ");lcd.print(h_1,2);

//      lcd.setCursor(0,3);
  //    sprintf(lcd_buffer, "BATT_VOLTAGE : %d" ,(int)batt_voltage);
//      lcd.print(lcd_buffer);
//      lcd.print("P:");lcd.print(pression,1);lcd.print(" batV:");lcd.print(batt_voltage,2);

    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_SEND_ALL_TEMP
      && buf[2]==NODE_ADDR
      && buf[3]==SEND_ALL_TEMP
      && buf[4]==FEATHER_MSG_END) {
        Serial1.println("parseRF_data : FEATHER_MSG_RESPONSE_ALL_DATA");

        byte* float_array;
        radiopacket[0]=FEATHER_MSG_HEADER;
        radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
        radiopacket[2]=NODE_ADDR;
        radiopacket[3]=SEND_ALL_TEMP;
        // Node 0
        float_array = (byte*) &nodes_data[0].temp[TEMP_1];
        radiopacket[4]= float_array[3];
        radiopacket[5]= float_array[2];
        radiopacket[6]= float_array[1];
        radiopacket[7]= float_array[0];
        float_array = (byte*) &nodes_data[0].temp[TEMP_2];
        radiopacket[8]= float_array[3];
        radiopacket[9]= float_array[2];
        radiopacket[10]= float_array[1];
        radiopacket[11]= float_array[0];
        float_array = (byte*) &nodes_data[0].temp[TEMP_3];
        radiopacket[12]= float_array[3];
        radiopacket[13]= float_array[2];
        radiopacket[14]= float_array[1];
        radiopacket[15]= float_array[0];
        float_array = (byte*) &nodes_data[0].humidity_1;
        radiopacket[16]= float_array[3];
        radiopacket[17]= float_array[2];
        radiopacket[18]= float_array[1];
        radiopacket[19]= float_array[0];
        float_array = (byte*) &nodes_data[0].battery_voltage;
        radiopacket[20]= float_array[3];
        radiopacket[21]= float_array[2];
        radiopacket[22]= float_array[1];
        radiopacket[23]= float_array[0];
        radiopacket[24]= nodes_data[0].last_rssi;
        // Node 1
        float_array = (byte*) &nodes_data[1].temp[TEMP_1];
        radiopacket[25]= float_array[3];
        radiopacket[26]= float_array[2];
        radiopacket[27]= float_array[1];
        radiopacket[28]= float_array[0];
        float_array = (byte*) &nodes_data[1].temp[TEMP_2];
        radiopacket[29]= float_array[3];
        radiopacket[30]= float_array[2];
        radiopacket[31]= float_array[1];
        radiopacket[32]= float_array[0];
        float_array = (byte*) &nodes_data[1].battery_voltage;
        radiopacket[33]= float_array[3];
        radiopacket[34]= float_array[2];
        radiopacket[35]= float_array[1];
        radiopacket[36]= float_array[0];
        radiopacket[37]= nodes_data[1].last_rssi;
        // Node 2
        float_array = (byte*) &nodes_data[2].temp[TEMP_1];
        radiopacket[38]= float_array[3];
        radiopacket[39]= float_array[2];
        radiopacket[40]= float_array[1];
        radiopacket[41]= float_array[0];
        float_array = (byte*) &nodes_data[2].temp[TEMP_2];
        radiopacket[42]= float_array[3];
        radiopacket[43]= float_array[2];
        radiopacket[44]= float_array[1];
        radiopacket[45]= float_array[0];
        float_array = (byte*) &nodes_data[2].battery_voltage;
        radiopacket[46]= float_array[3];
        radiopacket[47]= float_array[2];
        radiopacket[48]= float_array[1];
        radiopacket[49]= float_array[0];
        radiopacket[50]= nodes_data[2].last_rssi;
        // Node 3
        float_array = (byte*) &nodes_data[3].temp[TEMP_1];
        radiopacket[51]= float_array[3];
        radiopacket[52]= float_array[2];
        radiopacket[53]= float_array[1];
        radiopacket[54]= float_array[0];
        float_array = (byte*) &nodes_data[3].temp[TEMP_2];
        radiopacket[55]= float_array[3];
        radiopacket[56]= float_array[2];
        radiopacket[57]= float_array[1];
        radiopacket[58]= float_array[0];
        float_array = (byte*) &nodes_data[3].battery_voltage;
        radiopacket[59]= float_array[3];
        radiopacket[60]= float_array[2];
        radiopacket[61]= float_array[1];
        radiopacket[62]= float_array[0];
        radiopacket[63]= nodes_data[3].last_rssi;
        // Relais
        // Etat du relais
        radiopacket[64] = relais_etat;
        // Temperature moyenne
        float_array = (byte*) &t_avg;
        radiopacket[65]= float_array[3];
        radiopacket[66]= float_array[2];
        radiopacket[67]= float_array[1];
        radiopacket[68]= float_array[0];
        // Temperature consigne
        float_array = (byte*) &setpoint;
        radiopacket[69]= float_array[3];
        radiopacket[70]= float_array[2];
        radiopacket[71]= float_array[1];
        radiopacket[72]= float_array[0];
        // Delais des prises de temperature
        radiopacket[73] = delay_minutes;
        // Mode
        if(ModeAuto)
          radiopacket[74] = 1;
        else
          radiopacket[74] = 0;
        radiopacket[75]= FEATHER_MSG_END;

        rf95.send((uint8_t *)radiopacket, 76);
        rf95.waitPacketSent();
      }
      else if (buf[0]==FEATHER_MSG_HEADER
        && buf[1]==FEATHER_MSG_SET_DATA
        && buf[2]==NODE_ADDR
        && buf[3]==RELAY_THRESHOLD) {
          float_array_t_consigne[3]=buf[4];
          float_array_t_consigne[2]=buf[5];
          float_array_t_consigne[1]=buf[6];
          float_array_t_consigne[0]=buf[7];
          writeEEPROM(disk1, 0, buf[7]);
          writeEEPROM(disk1, 1, buf[6]);
          writeEEPROM(disk1, 2, buf[5]);
          writeEEPROM(disk1, 3, buf[4]);

          memcpy(&t_consigne,&float_array_t_consigne,sizeof(t_consigne));
          setpoint = t_consigne;
          Serial1.print("New setpoint: ");
          Serial1.println(setpoint);
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_SET_DATA
      && buf[2]==NODE_ADDR
      && buf[3]==DELAY_BETWEEN_READS
      && buf[5]==FEATHER_MSG_END){
        delay_minutes = buf[4];
        Serial1.print("New Delay Between Reads: ");
        Serial1.println(delay_minutes);
        writeEEPROM(disk1, 4, buf[4]);
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_GET_DATA
      && buf[2]==NODE_ADDR
      && buf[3]==RELAY_THRESHOLD
      && buf[4]==FEATHER_MSG_END) {
    	   SendSetpoint(NODE_ADDR);
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_GET_DATA
      && buf[2]==NODE_ADDR
      && buf[3]==READ_BATTERY_VOLTAGE
      && buf[4]==FEATHER_MSG_END) {
    	   SendBatVoltage(NODE_ADDR);
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_SET_DATA
      && buf[2]==NODE_ADDR
      && buf[3]==TURN_ON_RELAY
      && buf[4]==FEATHER_MSG_END)	{
        FanMotor(true);
        ModeAuto = false;
        relais_etat = 1;
        Serial1.println("Mode Manuel");
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_SET_DATA
      && buf[2]==NODE_ADDR
      && buf[3]==MODE_AUTO
      && buf[4]==FEATHER_MSG_END) {
        ModeAuto = true;
        Serial1.println("Mode Auto");
        relais_etat = 2;
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_SET_DATA
      && buf[2]==NODE_ADDR
      && buf[3]==TURN_OFF_RELAY
      && buf[4]==FEATHER_MSG_END) {
        FanMotor(false);
        ModeAuto = false;
        relais_etat = 0;
        Serial1.println("Mode Manuel");
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_GET_DATA
      && buf[2]==NODE_ADDR
      && buf[3]==RELAY_STATE
      && buf[4]==FEATHER_MSG_END) {
        SendRelayState(NODE_ADDR);
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_GET_DATA
      && buf[2]==NODE_ADDR
      && buf[3]==LAST_RSSI
      && buf[4]==FEATHER_MSG_END) {
    	   SendLastRssi(NODE_ADDR);
    }
    memset(buf,0,sizeof(buf));
    len = RH_RF95_MAX_MESSAGE_LEN;
}

void SendLastRssi (byte node_address){
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
	radiopacket[2]=node_address;
	radiopacket[3]=LAST_RSSI;
	radiopacket[4]=nodes_data[0].last_rssi;
	radiopacket[5]=nodes_data[1].last_rssi;
	radiopacket[6]=nodes_data[2].last_rssi;
	radiopacket[7]=nodes_data[3].last_rssi;
	radiopacket[8]= FEATHER_MSG_END;
	rf95.send((uint8_t *)radiopacket, 9);
	rf95.waitPacketSent();
}

void SendIncreasePower (byte node_address){
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_INCREASE_RF;
	radiopacket[2]=node_address;
	radiopacket[3]=FEATHER_MSG_END;
	rf95.send((uint8_t *)radiopacket, 4);
	rf95.waitPacketSent();
}
void SSR_ready_for_commands(byte node_address) {
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_READY_FOR_COMMANDS;
	radiopacket[2]=node_address;
	radiopacket[3]=FEATHER_MSG_END;
	rf95.send((uint8_t *)radiopacket, 4);
	rf95.waitPacketSent();
}

void SendallDataReady(byte node_address) {
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_ALL_DATA_READY;
	radiopacket[2]=node_address;
	radiopacket[3]=FEATHER_MSG_END;
	rf95.send((uint8_t *)radiopacket, 4);
	rf95.waitPacketSent();
}


void SendRelayState(byte node_address){
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
	radiopacket[2]=node_address;
	radiopacket[3]=RELAY_STATE;
	radiopacket[4]=relais_etat;
	radiopacket[5]= FEATHER_MSG_END;
	rf95.send((uint8_t *)radiopacket, 6);
	rf95.waitPacketSent();
}

float ReadBattVoltage(void){
	float measuredvbat = analogRead(VBATPIN);
	measuredvbat *= 2;    // we divided by 2, so multiply back
	measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
	measuredvbat /= 1024; // convert to voltage
	return measuredvbat;
}

void SendBatVoltage(byte node_address){
	float result;

	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
	radiopacket[2]=node_address;
	radiopacket[3]=READ_BATTERY_VOLTAGE;
	result = ReadBattVoltage();
	byte* float_array;
	float_array = (byte*) &result;
	radiopacket[4]= float_array[3];
	radiopacket[5]= float_array[2];
	radiopacket[6]= float_array[1];
	radiopacket[7]= float_array[0];
	radiopacket[8]= FEATHER_MSG_END;
	rf95.send((uint8_t *)radiopacket, 9);
	rf95.waitPacketSent();
}


void SendSetpoint (byte node_address) {
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
	radiopacket[2]=node_address;
	radiopacket[3]=RELAY_THRESHOLD;
	byte* float_array;
	float_array = (byte*) &setpoint;
	radiopacket[4]= float_array[3];
	radiopacket[5]= float_array[2];
	radiopacket[6]= float_array[1];
	radiopacket[7]= float_array[0];
	radiopacket[8]= FEATHER_MSG_END;
	rf95.send((uint8_t *)radiopacket, 9);
	rf95.waitPacketSent();
}

void QueryNode(byte node_address) {
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_GET_ALL_DATA;
	radiopacket[2]=node_address;
	radiopacket[3]=READ_ALL_DATA;
	radiopacket[4]= FEATHER_MSG_END;
	rf95.send((uint8_t *)radiopacket, 5);
	rf95.waitPacketSent();
}

void SetNodesToSleep(byte timetowake) {
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_REQUEST_ALL_NODE_SLEEP;
	radiopacket[2]=timetowake;
	radiopacket[3]=FEATHER_MSG_END;
	rf95.send((uint8_t *)radiopacket, 4);
	rf95.waitPacketSent();
}

void SendSSRReady(byte node_adress){
  radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_SSR_READY;
	radiopacket[2]=node_adress;
  radiopacket[3]=delay_minutes;
	radiopacket[4]=FEATHER_MSG_END;
	rf95.send((uint8_t *)radiopacket, 5);
	rf95.waitPacketSent();

}

void print2digits(int number) {
	if (number < 10) {
		Serial1.print("0"); // print a 0 before if the number is < than 10
	}
	Serial1.print(number);
}

void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data )
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();

  delay(5);
}

byte readEEPROM(int deviceaddress, unsigned int eeaddress )
{
  byte rdata = 0xFF;

  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();

  Wire.requestFrom(deviceaddress,1);

  if (Wire.available()) rdata = Wire.read();

  return rdata;
}
