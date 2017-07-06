#include <Arduino.h>
#include "node_feather_fan_controller.h"
#include "FanCompostMsg.h"

NodeFeatherFanController::NodeFeatherFanController(){
  Serial1.begin(9600);
  Serial1.println("NodeFeatherFanController()");
}

void NodeFeatherFanController::initialisation(void){
  Serial1.println("initialisation()");
  clock.begin();
  clock.setBattery(true, false);
  clock.enableOutput(false);

  setpoint = 20;
  delay_minutes = 1;
  relais_etat = 0;
  t_avg = 0;
  ModeAuto = true;

  clear_nodes_data();

  dt = clock.getDateTime();
  minutes_alarme = dt.minute + delay_minutes;
  if(minutes_alarme > 59)
    minutes_alarme = minutes_alarme - 60;
  clock.setAlarm1(0, 0, minutes_alarme, (OPERATION_TIME * NB_NODES)+5, DS3231_MATCH_M_S,true);

  // Initialisation de la pin de sortie pour le SSR
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW); // motor is off SSR is off.

  rf95 = new RH_RF95(RFM95_CS, RFM95_INT);

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
	rf95->setTxPower(23, false); // RF power between 5 (min) and 23 (max)
  eeprom_initialisation();
}

void NodeFeatherFanController::loop(void){
  if(ReceiveRFData()){
      Serial1.println("ReceiveRFData()");
      parseRF_data();
  }
  if(clock.isAlarm1(true)){
    Serial1.println("+++++++++++++++++++++++++++++");
    Serial1.println("Alarm1 true : calcule moyenne");

    clear_temp_avg();
    i_t_avg=0;
    t_avg=0;
    for(i_node=0;i_node<NB_NODES;i_node++){
      if(nodes_data[i_t_avg].new_data_received){
        Serial1.println("new_data_received");
        temp_avg[i_t_avg++] = nodes_data[i_node].temp[0];
        temp_avg[i_t_avg++] = nodes_data[i_node].temp[1];
      }
    }
    for(int j=0;j<i_t_avg;j++){
      t_avg += temp_avg[j];
    }

    t_avg = t_avg / (float)i_t_avg;

    Serial1.print("t_avg : ");Serial1.println(t_avg);

  //    EmuleCompostTemperature();
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

    // //  Serial1.println("clock.begin");
      dt = clock.getDateTime();
  //      clock.setBattery(true,false);
  //      clock.enableOutput(false);
    //  Serial1.println("clock.setAlarm1");
      uint8_t minutes_alarme = dt.minute + delay_minutes;
      if(minutes_alarme >59)
        minutes_alarme = minutes_alarme - 60;
      clock.setAlarm1(0, 0, minutes_alarme, (OPERATION_TIME * NB_NODES) + 5, DS3231_MATCH_M_S,true);
      // Envoie SSR_READY
      blink_led(2,100);
    Serial1.println("-------------------------");

    SSR_ready_for_commands(254); // send ready for commands to Raspberry
    if(ReceiveRFData()){
        Serial1.println("ReceiveRFData()");
        parseRF_data();
    }
    clear_nodes_data();
      Serial1.println("############################");
  }


}

void NodeFeatherFanController::clear_nodes_data(void){
	for(int i=0;i<NB_NODES;i++){
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


byte NodeFeatherFanController::readEEPROM(int deviceaddress, unsigned int eeaddress )
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

void NodeFeatherFanController::writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data )
{
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();

  delay(5);
}

void NodeFeatherFanController::eeprom_initialisation(void){
  // Initialisation des valeurs a partir du EEPROM
  Wire.begin(); // initialisation du i2c pour eeprom externe
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


byte NodeFeatherFanController::ReceiveRFData(void){
	if (rf95->waitAvailableTimeout(5000)) {
	// Should be a reply message for us now
		if (rf95->recv(buf, &len)) {
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

void NodeFeatherFanController::parseRF_data(void){
  byte float_array[4];
  byte array_u16[4];
  uint8_t node_id;
  uint16_t conductivite;
	float t_1,t_2,t_3,h_1,batt_voltage,pression;

  uint8_t ii=0;

  if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_NODE_READY && buf[4] == FEATHER_MSG_END){
      Serial1.println("FEATHER_MSG_NODE_READY");
      if(!nodes_data[getNodeID(buf[2])].clock_ok){
        SendSetDateTime(buf[2]);
        nodes_data[getNodeID(buf[2])].clock_ok=true;
      }
      else
        SendSSRReady(buf[2]);
  }

  else if(buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_SET_CLOCK && buf[2]==NODE_ADDR && buf[7]==FEATHER_MSG_END){
    Serial1.println("FEATHER_MSG_SET_CLOCK");
    uint32_t timeFromPC;
    byte uint32_array[4];
    uint32_array[3]=buf[3];
    uint32_array[2]=buf[4];
    uint32_array[1]=buf[5];
    uint32_array[0]=buf[6];
    memcpy(&timeFromPC,&uint32_array,sizeof(timeFromPC));
    Serial1.print("timeFromPC : ");Serial1.println(timeFromPC);
    clock.setDateTime(timeFromPC);
    dt = clock.getDateTime();
    Serial1.println(clock.dateFormat("d-m-Y H:i:s",dt));
  }

  else if (buf[0]==FEATHER_MSG_HEADER
    && buf[1]==FEATHER_MSG_RESPONSE_ALL_DATA
    && buf[3]==READ_ALL_DATA ) {
      Serial1.println("********************************************");
      Serial1.println("parseRF_data : FEATHER_MSG_RESPONSE_ALL_DATA");
      dt = clock.getDateTime();
      Serial1.println(clock.dateFormat("d-m-Y H:i:s - l", dt));

      node_id = getNodeID(buf[2]);
      nodes_data[node_id].node_address=buf[2];
      Serial1.print("Node ID : ");Serial1.println(node_id);
      nodes_data[node_id].last_rssi = rf95->lastRssi();
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
      array_u16[1]=buf[28];
      array_u16[0]=buf[29];
      memcpy(&conductivite,&array_u16,sizeof(conductivite));

      nodes_data[node_id].conductivite=conductivite;

      Serial1.print("CONDUCTIVITE : ");
      Serial1.println(conductivite);

      //TX Power

      nodes_data[node_id].txpower=buf[30];
      Serial1.print("txpower : ");
      Serial1.println(buf[30]);

      // Donnees OK
      nodes_data[node_id].new_data_received=true;



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
        Serial1.println("********************************************");
        Serial1.println("parseRF_data : FEATHER_MSG_SEND_ALL_TEMP");

        byte* float_array;
        byte* array_u16;
        radiopacket[0]=FEATHER_MSG_HEADER;
        radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
        radiopacket[2]=NODE_ADDR;
        radiopacket[3]=SEND_ALL_TEMP;


        // Node 0
        //
        //
        //
        ii=4;
        for(int jj=0;jj<NB_NODES;jj++){
          Serial1.println("============================");
          radiopacket[ii++]=nodes_data[jj].node_address;
          Serial1.println(nodes_data[jj].node_address);

          if(nodes_data[jj].new_data_received){
            radiopacket[ii++]=1;
          }
          else{
            radiopacket[ii++]=0;
          }
          float_array = (byte*) &nodes_data[jj].temp[TEMP_1];
          Serial1.println(nodes_data[jj].temp[TEMP_1]);
          radiopacket[ii++]= float_array[3];
          radiopacket[ii++]= float_array[2];
          radiopacket[ii++]= float_array[1];
          radiopacket[ii++]= float_array[0];

          float_array = (byte*) &nodes_data[jj].temp[TEMP_2];
          Serial1.println(nodes_data[jj].temp[TEMP_2]);
          radiopacket[ii++]= float_array[3];
          radiopacket[ii++]= float_array[2];
          radiopacket[ii++]= float_array[1];
          radiopacket[ii++]= float_array[0];

          float_array = (byte*) &nodes_data[jj].temp[TEMP_3];
          Serial1.println(nodes_data[jj].temp[TEMP_3]);
          radiopacket[ii++]= float_array[3];
          radiopacket[ii++]= float_array[2];
          radiopacket[ii++]= float_array[1];
          radiopacket[ii++]= float_array[0];

          float_array = (byte*) &nodes_data[jj].humidity_1;
          Serial1.println(nodes_data[jj].humidity_1);
          radiopacket[ii++]= float_array[3];
          radiopacket[ii++]= float_array[2];
          radiopacket[ii++]= float_array[1];
          radiopacket[ii++]= float_array[0];

          float_array = (byte*) &nodes_data[jj].pression;
          Serial1.println(nodes_data[jj].pression);
          radiopacket[ii++]= float_array[3];
          radiopacket[ii++]= float_array[2];
          radiopacket[ii++]= float_array[1];
          radiopacket[ii++]= float_array[0];

          array_u16 = (byte*) &nodes_data[jj].conductivite;
          Serial1.println(nodes_data[jj].conductivite);
          radiopacket[ii++]= array_u16[1];
          radiopacket[ii++]= array_u16[0];

          float_array = (byte*) &nodes_data[jj].battery_voltage;
          Serial1.println(nodes_data[jj].battery_voltage);
          radiopacket[ii++]= float_array[3];
          radiopacket[ii++]= float_array[2];
          radiopacket[ii++]= float_array[1];
          radiopacket[ii++]= float_array[0];

          radiopacket[ii++]=nodes_data[jj].txpower;
          Serial1.println(nodes_data[jj].txpower);
          radiopacket[ii++]=nodes_data[jj].last_rssi;
          Serial1.println(nodes_data[jj].last_rssi);
        }


        // Relais
        // Etat du relais
        radiopacket[ii++] = relais_etat;
        // Temperature moyenne
        float_array = (byte*) &t_avg;
        radiopacket[ii++]= float_array[3];
        radiopacket[ii++]= float_array[2];
        radiopacket[ii++]= float_array[1];
        radiopacket[ii++]= float_array[0];
        Serial1.println(t_avg);
        // Temperature consigne
        float_array = (byte*) &setpoint;
        radiopacket[ii++]= float_array[3];
        radiopacket[ii++]= float_array[2];
        radiopacket[ii++]= float_array[1];
        radiopacket[ii++]= float_array[0];
        Serial1.println(setpoint);
        // Delais des prises de temperature
        radiopacket[ii++] = delay_minutes;
        Serial1.println(delay_minutes);
        // Mode
        if(ModeAuto)
          radiopacket[ii++] = 1;
        else
          radiopacket[ii++] = 0;

        radiopacket[ii]= FEATHER_MSG_END;

        rf95->send((uint8_t *)radiopacket, ii);

        Serial1.print("ii : ");
        Serial1.println(ii);

        rf95->waitPacketSent();
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

uint8_t NodeFeatherFanController::getNodeID(uint8_t node_adress){
  uint8_t node_id;
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
  return node_id;
}

void NodeFeatherFanController::SendSetDateTime(byte node_adress){
  radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_SET_CLOCK;
	radiopacket[2]=node_adress;

  dt=clock.getDateTime();
  byte* uint32_array;
  uint32_array = (byte*) &dt.unixtime;

  radiopacket[3]= uint32_array[3];
  radiopacket[4]= uint32_array[2];
  radiopacket[5]= uint32_array[1];
  radiopacket[6]= uint32_array[0];
	radiopacket[7]=FEATHER_MSG_END;
	rf95->send((uint8_t *)radiopacket, 8);
	rf95->waitPacketSent();

}

void NodeFeatherFanController::SendSSRReady(byte node_adress){
  radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_SSR_READY;
	radiopacket[2]=node_adress;
  radiopacket[3]=delay_minutes;
	radiopacket[4]=FEATHER_MSG_END;
	rf95->send((uint8_t *)radiopacket, 5);
	rf95->waitPacketSent();
}

void NodeFeatherFanController::SendSetpoint (byte node_address) {
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
	rf95->send((uint8_t *)radiopacket, 9);
	rf95->waitPacketSent();
}

void NodeFeatherFanController::SendBatVoltage(byte node_address){
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
	rf95->send((uint8_t *)radiopacket, 9);
	rf95->waitPacketSent();
}

float NodeFeatherFanController::ReadBattVoltage(void){
	float measuredvbat = analogRead(VBATPIN);
	measuredvbat *= 2;    // we divided by 2, so multiply back
	measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
	measuredvbat /= 1024; // convert to voltage
	return measuredvbat;
}

void NodeFeatherFanController::FanMotor(bool motor_state){
	if (motor_state)
		digitalWrite(18, HIGH);
	else
		digitalWrite(18, LOW);
}

void NodeFeatherFanController::SendRelayState(byte node_address){
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
	radiopacket[2]=node_address;
	radiopacket[3]=RELAY_STATE;
	radiopacket[4]=relais_etat;
	radiopacket[5]= FEATHER_MSG_END;
	rf95->send((uint8_t *)radiopacket, 6);
	rf95->waitPacketSent();
}

void NodeFeatherFanController::SendLastRssi (byte node_address){
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
	radiopacket[2]=node_address;
	radiopacket[3]=LAST_RSSI;
	radiopacket[4]=nodes_data[0].last_rssi;
	radiopacket[5]=nodes_data[1].last_rssi;
	radiopacket[6]=nodes_data[2].last_rssi;
	radiopacket[7]=nodes_data[3].last_rssi;
	radiopacket[8]= FEATHER_MSG_END;
	rf95->send((uint8_t *)radiopacket, 9);
	rf95->waitPacketSent();
}

void NodeFeatherFanController::clear_temp_avg(void){
  for(int i=0;i<NB_NODES*2;i++){
    temp_avg[i]=0;
  }
}


void NodeFeatherFanController::blink_led(uint8_t nb_flash, uint32_t delais){
	int i;
	for(i=0; i<nb_flash;i++){
		digitalWrite(13, HIGH);
		delay(delais);
		digitalWrite(13, LOW);
		delay(delais);
	}
}

void NodeFeatherFanController::SSR_ready_for_commands(byte node_address) {
  Serial1.println("SSR_READY_FOR_COMMAND");
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_READY_FOR_COMMANDS;
	radiopacket[2]=node_address;
	radiopacket[3]=FEATHER_MSG_END;
	rf95->send((uint8_t *)radiopacket, 4);
	rf95->waitPacketSent();
}

uint8_t NodeFeatherFanController::ventilation(void){
  if(t_avg < PC1){
    return PC_1;
  }
  else if(t_avg >= PC1 && t_avg < PC2 ){
    return PC_2;
  }
  else if(t_avg >= PC2 && t_avg < PC3 ){
    return PC_3;
  }
  else if(t_avg >= PC3 && t_avg < PC4 ){
    return PC_4;
  }
}
