#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <RTCZero.h>

#include "FanCompostMsg.h"

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

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

struct node_data {
  float temp[4];
  float humidity_1;
  float battery_voltage;
  int8_t last_rssi;
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
uint8_t DelayBetweenReads = 1;
unsigned int eeprom_address = 0;

char radiopacket[76];
// Should be a message for us now
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

/* Create an rtc object */
RTCZero rtc;

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 23;
const byte hours = 15;

/* Change these values to set the current initial date */
const byte day = 24;
const byte month = 10;
const byte year = 16;

void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data );
byte readEEPROM(int deviceaddress, unsigned int eeaddress );
void SendLastRssi (byte node_address);
void print2digits(int number);
void clear_nodes_data(void);
void blink_led(uint8_t nb_flash, uint32_t delais);
void QueryNode(byte node_address);
byte ReceiveRFData(void);
void parseRF_data(byte node_address);
void SetNodesToSleep(byte timetowake);
void SendallDataReady(byte node_address);
bool check_delay_ready_for_command(void);
void SSR_ready_for_commands(byte node_address);
void FanMotor(bool motor_state);
void SendSetpoint (byte node_address);
void SendBatVoltage(byte node_address);
void SendRelayState(byte node_address);
void alarmMatch(void);

void setup() {
	// Initialisation du delais d'attente pour envoyer un ready for command
	delay_ready_for_command = 0;
	// Initialisation de la pin de sortie pour le SSR
	pinMode(12, OUTPUT);
	digitalWrite(12, LOW); // motor is off SSR is off.

	// Initialisation de la pin de sortie pour la led rouge et eteint la led
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);

	// Initialisation de la la pin de reset pour le RFM et place la pin en mode non-reset
	pinMode(RFM95_RST, OUTPUT);
	digitalWrite(RFM95_RST, HIGH);
	// manual RFM reset
	digitalWrite(RFM95_RST, LOW);
	delay(10);
	digitalWrite(RFM95_RST, HIGH);
	delay(10);

	while (!rf95.init()) {
		Serial.println("LoRa radio init failed");
		while (1);
	}
	Serial.println("LoRa radio init OK!");

	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	if (!rf95.setFrequency(RF95_FREQ)) {
		Serial.println("setFrequency failed");
	while (1);
	}

	Serial.print("Set Freq to: ");
	Serial.println(RF95_FREQ);
	rf95.setTxPower(23, false); // RF power between 5 (min) and 23 (max)

	// Initialisation du port serie a 9600 baud
	Serial.begin(9600);

	Wire.begin(); // initialisation du i2c pour eeprom externe
	// Initialisation du voltage de reference pour le ADC
	// analogReference(AR_EXTERNAL);


	// Configuration du RTC
	rtc.begin(); //Start RTC library, this is where the clock source is initialized
	rtc.setTime(hours, minutes, seconds); //set time
	rtc.setDate(day, month, year); //set date
	rtc.setAlarmTime(15, 23, 10); //set alarm time to go off in 10 seconds
	rtc.enableAlarm(rtc.MATCH_HHMMSS);
	//rtc.attachInterrupt(alarmMatch);
	//rtc.standbyMode();

	setpoint = 20;
	DelayBetweenReads = 1;


	// Initialisation des valeurs a partir du EEPROM

	if (readEEPROM(disk1, 0) == 255)
		{
		Serial.print("setpoint not initialized!");
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
		Serial.print("New setpoint: ");
		Serial.println(setpoint);
		DelayBetweenReads = readEEPROM(disk1, 4);
	}



	blink_led(10,100);
	delay(2000);
}

void loop() {
	// Si RTC Alarm Match
	if(Wakeup){
		blink_led(4,100);
		i=0;
		t_avg=0;
		clear_nodes_data();
//		delay(1000);

		for(ii=0;ii<NB_NODES;ii++){
			Serial.print("Query node : ");
			Serial.println(nodes_list[ii]);
			for(nb_retry = 0;nb_retry < MAX_ASK_ALL_DATA_RETRY;nb_retry++){
				QueryNode(nodes_list[ii]);
				if(ReceiveRFData()){
					parseRF_data(nodes_list[ii]);
					temp_avg[i++] = nodes_data[nodes_list[ii]].temp[TEMP_1];
					temp_avg[i++] = nodes_data[nodes_list[ii]].temp[TEMP_2];
					Serial.println( temp_avg[i-2] );
					Serial.println( temp_avg[i-1] );
					Serial.print("Nb retry : ");
					Serial.println(nb_retry+1);
					break;
				}
			}

		}


/*

		Serial.println("Query node 0...");
		QueryNode(0);
		if (ReceiveRFData()){
//			LastRSSI_node_0 = rf95.lastRssi();
			Serial.print("RSSI 0: ");
			Serial.println(rf95.lastRssi(), DEC);
			parseRF_data(0);
			temp_avg[i++] = nodes_data[0].temp[TEMP_1];
			temp_avg[i++] = nodes_data[0].temp[TEMP_2];

			Serial.println( temp_avg[i-2] );
			Serial.println( temp_avg[i-1] );
		}

		Serial.println("Query node 1...");
		QueryNode(1);
		if (ReceiveRFData()){
//			LastRSSI_node_1 = rf95.lastRssi();
			Serial.print("RSSI 1: ");
			Serial.println(rf95.lastRssi(), DEC);
			parseRF_data(1);
			temp_avg[i++] = nodes_data[1].temp[TEMP_1];
			temp_avg[i++] = nodes_data[1].temp[TEMP_2];
			Serial.println( temp_avg[i-2] );
			Serial.println( temp_avg[i-1] );
		}

		Serial.println("Query node 2...");
		QueryNode(2);
		if (ReceiveRFData()){
//			LastRSSI_node_2 = rf95.lastRssi();
			Serial.print("RSSI 2: ");
			Serial.println(rf95.lastRssi(), DEC);
			parseRF_data(2);
			temp_avg[i++] = nodes_data[2].temp[TEMP_1];
			temp_avg[i++] = nodes_data[2].temp[TEMP_2];
			Serial.println( temp_avg[i-2] );
			Serial.println( temp_avg[i-1] );
		}

		Serial.println("Query node 3...");
		QueryNode(3);
		if (ReceiveRFData()){
//			LastRSSI_node_3 = rf95.lastRssi();
			Serial.print("RSSI 3: ");
			Serial.println(rf95.lastRssi(), DEC);
			parseRF_data(3);
			temp_avg[i++] = nodes_data[3].temp[TEMP_1];
			temp_avg[i++] = nodes_data[3].temp[TEMP_2];
			Serial.println( temp_avg[i-2] );
			Serial.println( temp_avg[i-1] );
		}
*/
		SetNodesToSleep(DelayBetweenReads);

		for(int j=0;j<i;j++){
			t_avg += temp_avg[j];
		}

		t_avg = t_avg / i;
		Serial.println("Temperature moyenne :");
		Serial.println( t_avg );

		rtc.setTime(0,0,0); //set time
		rtc.setAlarmTime(0, DelayBetweenReads, 0); //set alarm time to go off in x minutes
		rtc.enableAlarm(rtc.MATCH_HHMMSS);
		rtc.attachInterrupt(alarmMatch);

		Wakeup = false;
		SendallDataReady(254);
		delay_ready_for_command = 0;
	}
	// Envoie SSR_READY
	if(	check_delay_ready_for_command()){
		blink_led(2,100);
		SSR_ready_for_commands(254); // send ready for commands to Raspberry
	}
	// Receive RF DATA
	ReceiveRFData();	// check if there is data coming in from the Raspberry, timeout 3 sec.
	// Parse RF DATA
	parseRF_data(254); // and parse it


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
	delay(100);

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
	}
}

void alarmMatch(void){
	Wakeup = true;
	Serial.print("Alarm MAtch!");
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
		digitalWrite(12, HIGH);
	else
		digitalWrite(12, LOW);
}

byte ReceiveRFData(void){
	if (rf95.waitAvailableTimeout(3000)) {
	// Should be a reply message for us now
		if (rf95.recv(buf, &len)) {
			return 1;
		}
		else {
			Serial.println("recv failed");
		}
	}
	else {
		Serial.println("No reply from the node!");
		return 0;
	}
	delay(400);
	return 1;
}

void parseRF_data(byte node_address)
{
	byte float_array[4];

	float t_1,t_2,t_3,h_1,batt_voltage;

	if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_RESPONSE_ALL_DATA && buf[2]==node_address && buf[3]==READ_ALL_DATA ) {
		nodes_data[node_address].last_rssi = rf95.lastRssi();

		float_array[3]=buf[4];
		float_array[2]=buf[5];
		float_array[1]=buf[6];
		float_array[0]=buf[7];
		memcpy(&t_1,&float_array,sizeof(t_1));
		nodes_data[node_address].temp[0]=t_1;

		float_array[3]=buf[8];
		float_array[2]=buf[9];
		float_array[1]=buf[10];
		float_array[0]=buf[11];
		memcpy(&t_2,&float_array,sizeof(t_2));
		nodes_data[node_address].temp[1]=t_2;

		float_array[3]=buf[12];
		float_array[2]=buf[13];
		float_array[1]=buf[14];
		float_array[0]=buf[15];
		memcpy(&t_3,&float_array,sizeof(t_3));
		nodes_data[node_address].temp[2]=t_3;

		float_array[3]=buf[16];
		float_array[2]=buf[17];
		float_array[1]=buf[18];
		float_array[0]=buf[19];
		memcpy(&h_1,&float_array,sizeof(h_1));
		nodes_data[node_address].humidity_1=h_1;

		float_array[3]=buf[20];
		float_array[2]=buf[21];
		float_array[1]=buf[22];
		float_array[0]=buf[23];
		memcpy(&batt_voltage,&float_array,sizeof(batt_voltage));
		nodes_data[node_address].battery_voltage=batt_voltage;
	}

	if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_SEND_ALL_TEMP && buf[2]==node_address && buf[3]==SEND_ALL_TEMP && buf[4]==FEATHER_MSG_END) {
		byte* float_array;
		radiopacket[0]=FEATHER_MSG_HEADER;
		radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
		radiopacket[2]=node_address;
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
		radiopacket[73] = DelayBetweenReads;
		// Mode
		if(ModeAuto)
			radiopacket[74] = 1;
		else
			radiopacket[74] = 0;
		radiopacket[75]= FEATHER_MSG_END;

		rf95.send((uint8_t *)radiopacket, 76);
		rf95.waitPacketSent();
	}

	if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_SET_DATA && buf[2]==node_address && buf[3]==RELAY_THRESHOLD) {
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
		Serial.print("New setpoint: ");
		Serial.println(setpoint);
    }

	if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_SET_DATA && buf[2]==node_address && buf[3]==DELAY_BETWEEN_READS && buf[5]==FEATHER_MSG_END)
	{
		DelayBetweenReads = buf[4];

		Serial.print("New Delay Between Reads: ");
		Serial.println(DelayBetweenReads);
		writeEEPROM(disk1, 4, buf[4]);

	}

    if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_GET_DATA && buf[2]==node_address && buf[3]==RELAY_THRESHOLD && buf[4]==FEATHER_MSG_END) {
    	SendSetpoint(NODE_ADDR);
    }

    if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_GET_DATA && buf[2]==node_address && buf[3]==READ_BATTERY_VOLTAGE && buf[4]==FEATHER_MSG_END) {
    	SendBatVoltage(NODE_ADDR);
    }

	if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_SET_DATA && buf[2]==node_address && buf[3]==TURN_ON_RELAY && buf[4]==FEATHER_MSG_END)	{
		FanMotor(true);
		ModeAuto = false;
		relais_etat = 1;
		Serial.println("Mode Manuel");
	}

    if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_SET_DATA && buf[2]==node_address && buf[3]==MODE_AUTO && buf[4]==FEATHER_MSG_END) {
		ModeAuto = true;
		Serial.println("Mode Auto");
		relais_etat = 2;
	}

	if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_SET_DATA && buf[2]==node_address && buf[3]==TURN_OFF_RELAY && buf[4]==FEATHER_MSG_END) {
		FanMotor(false);
		ModeAuto = false;
		relais_etat = 0;
		Serial.println("Mode Manuel");
	}

	if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_GET_DATA && buf[2]==node_address && buf[3]==RELAY_STATE && buf[4]==FEATHER_MSG_END) {
		SendRelayState(NODE_ADDR);
	}

    if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_GET_DATA && buf[2]==node_address && buf[3]==LAST_RSSI && buf[4]==FEATHER_MSG_END) {
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

void print2digits(int number) {
	if (number < 10) {
		Serial.print("0"); // print a 0 before if the number is < than 10
	}
	Serial.print(number);
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
