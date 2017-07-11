#include <Arduino.h>
#include "node_feather_fan_controller.h"
#include "FanCompostMsg.h"

RTCZero zerortc;
volatile bool alarmFlag = false; // Start awake

void alarmMatch(void){
  alarmFlag = true;
}


NodeFeatherFanController::NodeFeatherFanController(){
  Serial1.begin(9600);
  Serial1.println("#############################  NodeFeatherFanController()  #################################");
//  Wire.begin(); // initialisation du i2c pour eeprom externe
//  resetEEPROM(I2C_EEPROM_ADDRESS);
  initialisation();
}

void NodeFeatherFanController::initialisation(void){
  Serial1.println("initialisation()");

  // Initialisation du RTCZero pour la gestion de la ventilation
  alarmFlag = false;
  zerortc.begin(true);
  zerortc.attachInterrupt(alarmMatch);
  alarmHours=0;
  alarmMinutes=0;
  alarmSeconds=10;
  resetAlarm();

  // Initialisation du current_PC (Point de Consigne actuelle)
  ssrd.current_PC = 0;

  ssrd.ssr_state = 0;
  ssr_ModeAuto = true;
  do_avg = false;

  // Initialisation des pin pour les led
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, LOW); // LED is off
  pinMode(LED_RED_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW); // LED is off


  // Initialisation de la pin de sortie pour le SSR
  pinMode(SSR_1_PIN, OUTPUT);
  digitalWrite(SSR_1_PIN, LOW); // motor is off SSR is off.

  // Initialisation du epprom et lecture du nfc (Node Fan Configuration)
  eeprom_initialisation();

  // Initialisation des status de nouvelles donnees
  clear_compost_nodes_new_data();

// Initialisation des status des horloge sur les noeuds de compost
  clear_compost_nodes_clock_ok();

  // Utilisation du radio
  rf95 = new RH_RF95(RFM95_CS, RFM95_INT);

  // Initialisation du radio
  while (!rf95->init()) {
    Serial1.println("\tLoRa radio init failed");
    while (1);
  }
  Serial1.println("\tLoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	if (!rf95->setFrequency(RF95_FREQ)) {
		Serial1.println("\tsetFrequency failed");
	while (1);
	}
  Serial1.print("\tSet Freq to: ");
	Serial1.println(RF95_FREQ);
	rf95->setTxPower(23, false); // RF power between 5 (min) and 23 (max)

  // Initialisation de l'horloge
  clock.begin();
  clock.setBattery(true, false);
  clock.enableOutput(false);

  SendGetDateTime(NODE_SSR_ADDR);
  if(ReceiveRFData()){
//      Serial1.println("ReceiveRFData()");
      parseRF_data();
  }


  // On recupere l'heure actuelle
  dt = clock.getDateTime();

  // Calcule de la minute ou il y aura alarme.  La prochaine minute est défini par la minute actuelle + le delais en minutes
  f_minutes_alarme = dt.minute + nfc.delais_minute;
  if(f_minutes_alarme > 59){
    f_minutes_alarme = f_minutes_alarme - 60;
  }

    // On set l'alarme a minutes_alarme + un certain temps en secondes avant de se réveiller
    clock.setAlarm1(0, 0, f_minutes_alarme, (OPERATION_TIME * NB_NODES) + OPERATION_TIME, DS3231_MATCH_M_S,true);
    digitalWrite(LED_GREEN_PIN, HIGH); // LED is on

}

void NodeFeatherFanController::loop(void){
  // Verifie s'il y a des données RF de disponible...
  if(ReceiveRFData()){
      parseRF_data();
  }

  if(alarmFlag){
    alarmFlag=false;
    Serial1.println("loop() : RTC Alarm");

    if(ssr_ModeAuto){
      switch (ssrd.current_PC) {
        case PC0_ID:
          ssrd.ssr_state = 0;
          setFanMotor(false);
          break;
        case PC1_ID:
          if(ssrd.ssr_state){
            setRTCAlarm(nfc.TA1);
            ssrd.ssr_state = 0;
            setFanMotor(false);
          }
          else{
            setRTCAlarm(nfc.TV1);
            ssrd.ssr_state = 1;
            setFanMotor(true);
          }
        break;
        case PC2_ID:
        if(ssrd.ssr_state){
          setRTCAlarm(nfc.TA2);
          ssrd.ssr_state = 0;
          setFanMotor(false);
        }
        else{
          setRTCAlarm(nfc.TV2);
          ssrd.ssr_state = 1;
          setFanMotor(true);
        }
        break;
        case PC3_ID:
        if(ssrd.ssr_state){
          setRTCAlarm(nfc.TA3);
          ssrd.ssr_state = 0;
          setFanMotor(false);
        }
        else{
          setRTCAlarm(nfc.TV3);
          ssrd.ssr_state = 1;
          setFanMotor(true);
        }
        break;
        case PC4_ID:
        if(ssrd.ssr_state){
          setRTCAlarm(nfc.TA4);
          ssrd.ssr_state = 0;
          setFanMotor(false);
        }
        else{
          setRTCAlarm(nfc.TV4);
          ssrd.ssr_state = 1;
          setFanMotor(true);
        }
        break;
      }
      resetAlarm();
    }
  }

  if(clock.isAlarm1(true)){
    Serial1.println("loop() : DS3231 Alarm1 true");
    clear_temp_avg();
    i_t_avg=0;
    ssrd.t_avg=0;
    for(i_node=0;i_node<NB_NODES;i_node++){
//      if(nodes_data[i_t_avg].new_data_received){
      if(compost_node_new_data[i_node]){
        Serial1.print("\tNode : "); Serial1.print(i_node); Serial1.println(" : new_data_received");
        do_avg=true;
        if(nfc.node_compost_cfg[i_node] & SET_T_AVG_SURFACE){
          temp_avg[i_t_avg++] = cnd_array[i_node].ntc_1;
        }
        if(nfc.node_compost_cfg[i_node] & SET_T_AVG_PROFONDEUR){
          temp_avg[i_t_avg++] = cnd_array[i_node].ntc_2;
        }
      }
    }
    if(do_avg){
      for(int j=0;j<i_t_avg;j++){
        ssrd.t_avg += temp_avg[j];
      }

      ssrd.t_avg = ssrd.t_avg / (float)i_t_avg;

      Serial1.print("\tt_avg : ");Serial1.println(ssrd.t_avg);
    }
    set_current_PC();

    dt = clock.getDateTime();
    f_minutes_alarme = dt.minute + nfc.delais_minute;
    if(f_minutes_alarme >59){
      f_minutes_alarme = f_minutes_alarme - 60;
    }
    clock.setAlarm1(0, 0, f_minutes_alarme, (OPERATION_TIME * NB_NODES) + 5, DS3231_MATCH_M_S,true);
    SendSSR_ready_for_commands(NODE_SSR_ADDR); // send ready for commands to Raspberry
    if(ReceiveRFData()){
        parseRF_data();
    }

    blink_led(2,100);
  }
}

void NodeFeatherFanController::clear_compost_nodes_new_data(void){
  Serial1.println("clear_compost_nodes_new_data()");
  for(int i = 0; i< NB_NODES; i++){
    compost_node_new_data[i]=0;
    cnd_array[i].node_address = 0;
    cnd_array[i].node_cfg = 0;
    cnd_array[i].timestamp = 0;
    cnd_array[i].ntc_1 = 0;
    cnd_array[i].ntc_2 = 0;
    cnd_array[i].bme_humidity = 0;
    cnd_array[i].bme_temp = 0;
    cnd_array[i].bme_pression = 0;
    cnd_array[i].conductivite = 0;
    cnd_array[i].batt_voltage = 0;
    cnd_array[i].delay_minutes = 0;
    cnd_array[i].txpower = 0;
    cnd_array[i].last_rssi = 0;
    cnd_array[i].new_data = 0;
    cnd_array[i].clock_ok = 0;
  }
}

void NodeFeatherFanController::clear_compost_nodes_clock_ok(void){
  for(int i = 0; i< NB_NODES; i++){
    compost_node_clock_ok[i]=0;
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

  if (Wire.available()) {
    rdata = Wire.read();
  }

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

void NodeFeatherFanController::resetEEPROM(int deviceaddress){
	Serial1.print("resetEEPROM()");
  for(int eeaddress = 0;eeaddress<EEPROM_MAX_ADDRESS;eeaddress++){
    writeEEPROM(deviceaddress, eeaddress, EEPROM_INIT_VALUE);
    Serial1.println(eeaddress);
  }
}

void NodeFeatherFanController::readAllEEPROM(int deviceaddress){
  int value;
  Serial1.print("readAllEEPROM()");
  for(int eeaddress = 0;eeaddress<EEPROM_MAX_ADDRESS;eeaddress++){
    Serial1.print(eeaddress);
    Serial1.print(" : ");
    value = (int)readEEPROM(deviceaddress, eeaddress);
    Serial1.print("value : ");
    Serial1.println(value);
  }
}

void NodeFeatherFanController::writeEEPROM_NFCConfig(void){
  byte *nfc_pointer = (byte*)&nfc;
  for(uint8_t i=0 ; i < sizeof(struct node_fan_config);i++ ){
    writeEEPROM(I2C_EEPROM_ADDRESS, EEPROM_NODE_FAN_CONFIG + i, nfc_pointer[i]);
  }
}

void NodeFeatherFanController::readEEPROM_NFCConfig(void){
  byte *nfc_pointer = (byte*)&nfc;
  for(uint8_t i=0 ; i < sizeof(struct node_fan_config);i++ ){
    nfc_pointer[i] = readEEPROM(I2C_EEPROM_ADDRESS, EEPROM_NODE_FAN_CONFIG + i);
  }
}


void NodeFeatherFanController::eeprom_initialisation(void){
  // Initialisation des valeurs a partir du EEPROM
//  byte *float_array;
  Serial1.println("eeprom_initialisation()");
  Wire.begin(); // initialisation du i2c pour eeprom externe
//  writeEEPROM(I2C_EEPROM_ADDRESS, EEPROM_ADDR_INIT, 255);

	if (readEEPROM(I2C_EEPROM_ADDRESS, EEPROM_ADDR_INIT) == 255)
		{
		Serial1.println("\tEEPROM non initialise!");
    // Si le EEPROM est non initialise, on effectue une assignation des valeurs de configuration par defaut et on les sauvegarde dans le EEPROM.
    //

    nfc.PC1 = SSR_DEFAULT_PC1;
    nfc.PC2 = SSR_DEFAULT_PC2;
    nfc.PC3 = SSR_DEFAULT_PC3;
    nfc.PC4 = SSR_DEFAULT_PC4;
    nfc.TA1 = SSR_DEFAULT_TA1;
    nfc.TA2 = SSR_DEFAULT_TA2;
    nfc.TA3 = SSR_DEFAULT_TA3;
    nfc.TA4 = SSR_DEFAULT_TA4;
    nfc.TV1 = SSR_DEFAULT_TV1;
    nfc.TV2 = SSR_DEFAULT_TV2;
    nfc.TV3 = SSR_DEFAULT_TV3;
    nfc.TV4 = SSR_DEFAULT_TV4;
    nfc.delais_minute = SSR_DEFAULT_MINUTE_DELAY;
    nfc.node_compost_addr[0] = SSR_DEFAULT_NODE_COMPOST_00_ADDR;
    nfc.node_compost_addr[1] = SSR_DEFAULT_NODE_COMPOST_01_ADDR;
    nfc.node_compost_addr[2] = SSR_DEFAULT_NODE_COMPOST_02_ADDR;
    nfc.node_compost_addr[3] = SSR_DEFAULT_NODE_COMPOST_03_ADDR;
    nfc.node_compost_cfg[0] = SSR_DEFAULT_NODE_COMPOST_00_CONFIG;
    nfc.node_compost_cfg[1] = SSR_DEFAULT_NODE_COMPOST_01_CONFIG;
    nfc.node_compost_cfg[2] = SSR_DEFAULT_NODE_COMPOST_02_CONFIG;
    nfc.node_compost_cfg[3] = SSR_DEFAULT_NODE_COMPOST_03_CONFIG;
    for(int i=0; i<MAX_NODE_TEXT;i++){
      nfc.node_compost_text[0][i]='X';
      nfc.node_compost_text[1][i]='X';
      nfc.node_compost_text[2][i]='X';
      nfc.node_compost_text[3][i]='X';
    }

    writeEEPROM_NFCConfig();

    writeEEPROM(I2C_EEPROM_ADDRESS, EEPROM_ADDR_INIT, 0);
    print_NFC();
    Serial1.println("\tEEPROM : Initialise!");
	}
	else {
    Serial1.println("\tEEPROM Initialise, lecture de la configuration du eeprom...");
    readEEPROM_NFCConfig();
    print_NFC();
	}
}

void NodeFeatherFanController::print_NFC(void){
  Serial1.println("print_NFC()");

  Serial1.print("\tPC1 : ");Serial1.println(nfc.PC1);
  Serial1.print("\tPC2 : ");Serial1.println(nfc.PC2);
  Serial1.print("\tPC3 : ");Serial1.println(nfc.PC3);
  Serial1.print("\tPC4 : ");Serial1.println(nfc.PC4);

  Serial1.print("\tTV1 : ");Serial1.println(nfc.TV1);
  Serial1.print("\tTV2 : ");Serial1.println(nfc.TV2);
  Serial1.print("\tTV3 : ");Serial1.println(nfc.TV3);
  Serial1.print("\tTV4 : ");Serial1.println(nfc.TV4);

  Serial1.print("\tTA1 : ");Serial1.println(nfc.TA1);
  Serial1.print("\tTA2 : ");Serial1.println(nfc.TA2);
  Serial1.print("\tTA3 : ");Serial1.println(nfc.TA3);
  Serial1.print("\tTA4 : ");Serial1.println(nfc.TA4);

  Serial1.print("\tDelais minutes : ");Serial1.println(nfc.delais_minute);

  Serial1.print("\tnode_compost_addr_00 : ");Serial1.println(nfc.node_compost_addr[0]);
  Serial1.print("\tnode_compost_addr_01 : ");Serial1.println(nfc.node_compost_addr[1]);
  Serial1.print("\tnode_compost_addr_02 : ");Serial1.println(nfc.node_compost_addr[2]);
  Serial1.print("\tnode_compost_addr_03 : ");Serial1.println(nfc.node_compost_addr[3]);

  Serial1.print("\tnode_compost_cfg_00 : ");Serial1.println(nfc.node_compost_cfg[0]);
  Serial1.print("\tnode_compost_cfg_01 : ");Serial1.println(nfc.node_compost_cfg[1]);
  Serial1.print("\tnode_compost_cfg_02 : ");Serial1.println(nfc.node_compost_cfg[2]);
  Serial1.print("\tnode_compost_cfg_03 : ");Serial1.println(nfc.node_compost_cfg[3]);

  for(int j=0; j<NB_NODES; j++){
    Serial1.print("\tnode_compost_txt_"); Serial1.print(j);Serial1.print(" : ");
    for(int i=0; i<MAX_NODE_TEXT;i++){
       Serial1.print("[");Serial1.print(nfc.node_compost_text[j][i]); Serial1.print("]");
    }
    Serial1.println();
  }
}


byte NodeFeatherFanController::ReceiveRFData(void){
//  Serial1.println("ReceiveRFData()");
  if (rf95->waitAvailableTimeout(2000)) {
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
  byte u16_array[2];
//  byte uint32_array[4];
  int16_t node_id;
  uint16_t conductivite;
	float t_1,t_2,t_3,h_1,batt_voltage,pression;

  if (buf[0]==FEATHER_MSG_HEADER){
    Serial1.println("parseRF_data() : FEATHER_MSG_HEADER");
  }
  if (buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_NODE_READY && buf[4] == FEATHER_MSG_END){
    Serial1.println("parseRF_data() : FEATHER_MSG_NODE_READY");
    node_id = getNodeID(buf[2]);
    if(node_id != -1){
      if(!compost_node_clock_ok[node_id]){
        SendSetDateTime(buf[2]);
        compost_node_clock_ok[node_id] = 1;
      }
      else{
        SendSSRReady(buf[2]);
      }
    }
    else{
      Serial1.print("\t Node address : "); Serial1.print(buf[2]); Serial1.println(" not valid.");
    }
  }

  else if(buf[0]==FEATHER_MSG_HEADER && buf[1]==FEATHER_MSG_SET_CLOCK && buf[2]==NODE_SSR_ADDR && buf[7]==FEATHER_MSG_END){
    Serial1.println("parseRF_data() : FEATHER_MSG_SET_CLOCK");
    uint32_t timeFromPC;
    byte uint32_array[4];
    uint32_array[3]=buf[3];
    uint32_array[2]=buf[4];
    uint32_array[1]=buf[5];
    uint32_array[0]=buf[6];
    memcpy(&timeFromPC,&uint32_array,sizeof(timeFromPC));
    Serial1.print("\ttimeFromPC : ");Serial1.println(timeFromPC);
    clock.setDateTime(timeFromPC);
    dt = clock.getDateTime();
    Serial1.print("\t");
    Serial1.println(clock.dateFormat("d-m-Y H:i:s",dt));
  }

  else if (buf[0]==FEATHER_MSG_HEADER
    && buf[1]==FEATHER_MSG_COMPOST_NODE_DATA) {
      Serial1.println("parseRF_data() : FEATHER_MSG_Compost_NODE_DATA");
      node_id = getNodeID(buf[2]);
      if(node_id != -1){
        int j=3;
        dt = clock.getDateTime();
        Serial1.print("\t");
        Serial1.println(clock.dateFormat("d-m-Y H:i:s - l", dt));

        byte* cnd_pointer = (byte*)&cnd_array[node_id];
        for(uint16_t i=0;i<sizeof(struct CompostNodeData);i++){
          cnd_pointer[i] = buf[j++];
        }
        cnd_array[node_id].last_rssi = rf95->lastRssi();
        compost_node_new_data[node_id] = 1;
        Serial1.print("\tNODE_ADDRESS : ");Serial1.println(cnd_array[node_id].node_address);
        Serial1.print("\tNODE_CFG : ");Serial1.println(cnd_array[node_id].node_cfg);
        Serial1.print("\tNODE_TIMESTAMP : ");Serial1.println(cnd_array[node_id].timestamp);
        Serial1.print("\tNTC_1 : "); Serial1.println(cnd_array[node_id].ntc_1,2);
        Serial1.print("\tNTC_2 : "); Serial1.println(cnd_array[node_id].ntc_2,2);
        Serial1.print("\tBME_HUMIDITY : "); Serial1.println(cnd_array[node_id].bme_humidity,2);
        Serial1.print("\tBME_TEMP : "); Serial1.println(cnd_array[node_id].bme_temp,2);
        Serial1.print("\tBME_PRESSION : "); Serial1.println(cnd_array[node_id].bme_pression,2);
        Serial1.print("\tCONDUCTIVITE : "); Serial1.println(cnd_array[node_id].conductivite);
        Serial1.print("\tBATT_VOLTAGE : "); Serial1.println(cnd_array[node_id].batt_voltage,2);
        Serial1.print("\tDELAY_MINUTES : "); Serial1.println(cnd_array[node_id].delay_minutes,2);
        Serial1.print("\tLAST_RSSI : "); Serial1.println(cnd_array[node_id].last_rssi);
        Serial1.print("\tTX_POWER : "); Serial1.println(cnd_array[node_id].txpower);
      }
      else{
        Serial1.println("FEATHER_MSG_Compost_NODE_DATA : Not a valid node address");
      }
    }

  else if (buf[0]==FEATHER_MSG_HEADER
    && buf[1]==FEATHER_MSG_RESPONSE_ALL_DATA
    && buf[3]==READ_ALL_DATA ) {
      Serial1.println("parseRF_data() : FEATHER_MSG_RESPONSE_ALL_DATA");
      node_id = getNodeID(buf[2]);
      if(node_id != -1){
        dt = clock.getDateTime();
        Serial1.print("\t");
        Serial1.println(clock.dateFormat("d-m-Y H:i:s - l", dt));

        Serial1.print("\tNode ID : ");Serial1.println(node_id);
        nodes_data[node_id].last_rssi = rf95->lastRssi();
        Serial1.print("\tLAST_RSSI : ");
        Serial1.println(nodes_data[node_id].last_rssi);
        // NTC_1
        float_array[3]=buf[4];
        float_array[2]=buf[5];
        float_array[1]=buf[6];
        float_array[0]=buf[7];
        memcpy(&t_1,&float_array,sizeof(t_1));
        nodes_data[node_id].temp[0]=t_1;
        Serial1.print("\tNTC_1 : ");
        Serial1.println(t_1,2);

        // NTC_2
        float_array[3]=buf[8];
        float_array[2]=buf[9];
        float_array[1]=buf[10];
        float_array[0]=buf[11];
        memcpy(&t_2,&float_array,sizeof(t_2));
        nodes_data[node_id].temp[1]=t_2;
        Serial1.print("\tNTC_2 : ");
        Serial1.println(t_2,2);

        // BME_TEMP
        float_array[3]=buf[12];
        float_array[2]=buf[13];
        float_array[1]=buf[14];
        float_array[0]=buf[15];
        memcpy(&t_3,&float_array,sizeof(t_3));
        nodes_data[node_id].temp[2]=t_3;
        Serial1.print("\tBME_TEMP : ");
        Serial1.println(t_3,2);

        // BME_HUMIDITY
        float_array[3]=buf[16];
        float_array[2]=buf[17];
        float_array[1]=buf[18];
        float_array[0]=buf[19];
        memcpy(&h_1,&float_array,sizeof(h_1));
        nodes_data[node_id].humidity_1=h_1;
        Serial1.print("\tBME_HUMIDITY : ");
        Serial1.println(h_1,2);

        // BATT_VOLTAGE
        float_array[3]=buf[20];
        float_array[2]=buf[21];
        float_array[1]=buf[22];
        float_array[0]=buf[23];
        memcpy(&batt_voltage,&float_array,sizeof(batt_voltage));
        nodes_data[node_id].battery_voltage=batt_voltage;
        Serial1.print("\tBATT_VOLTAGE : ");
        Serial1.println(batt_voltage,2);

        // BME_PRESSION
        float_array[3]=buf[24];
        float_array[2]=buf[25];
        float_array[1]=buf[26];
        float_array[0]=buf[27];
        memcpy(&pression,&float_array,sizeof(pression));
        nodes_data[node_id].pression=pression;
        Serial1.print("\tBME_PRESSION : ");
        Serial1.println(pression,2);

        // CONDUCTIVITE
        u16_array[1]=buf[28];
        u16_array[0]=buf[29];
        memcpy(&conductivite,&u16_array,sizeof(conductivite));

        nodes_data[node_id].conductivite=conductivite;

        Serial1.print("\tCONDUCTIVITE : ");
        Serial1.println(conductivite);

        //TX Power

        nodes_data[node_id].txpower=buf[30];
        Serial1.print("\ttxpower : ");
        Serial1.println(buf[30]);

        // Donnees OK
        nodes_data[node_id].new_data_received=true;
      }
      else{
        Serial1.println("FEATHER_MSG_Compost_NODE_DATA : Not a valid node address");
      }


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
      && buf[1]==FEATHER_MSG_SEND_SSR_NODE_CFG
      && buf[2]==NODE_SSR_ADDR
      && buf[3]==SEND_ALL_CFG
      && buf[4]==FEATHER_MSG_END) {
        Serial1.println("parseRF_data() : SEND_ALL_CFG");
        SendAllCfg();
        if(ReceiveRFData()){
            parseRF_data();
        }

/*
        byte* float_array;
        byte* array_u16;

        int ii=4;

        radiopacket[0]=FEATHER_MSG_HEADER;
        radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
        radiopacket[2]=NODE_SSR_ADDR;
        radiopacket[3]=SEND_ALL_CFG;

        float_array = (byte*) &nfc.PC1;
        radiopacket[ii++]= float_array[3];
        radiopacket[ii++]= float_array[2];
        radiopacket[ii++]= float_array[1];
        radiopacket[ii++]= float_array[0];

        float_array = (byte*) &nfc.PC2;
        radiopacket[ii++]= float_array[3];
        radiopacket[ii++]= float_array[2];
        radiopacket[ii++]= float_array[1];
        radiopacket[ii++]= float_array[0];

        float_array = (byte*) &nfc.PC3;
        radiopacket[ii++]= float_array[3];
        radiopacket[ii++]= float_array[2];
        radiopacket[ii++]= float_array[1];
        radiopacket[ii++]= float_array[0];

        float_array = (byte*) &nfc.PC4;
        radiopacket[ii++]= float_array[3];
        radiopacket[ii++]= float_array[2];
        radiopacket[ii++]= float_array[1];
        radiopacket[ii++]= float_array[0];

        array_u16 = (byte*) &nfc.TV1;
        radiopacket[ii++]= array_u16[1];
        radiopacket[ii++]= array_u16[0];

        array_u16 = (byte*) &nfc.TV2;
        radiopacket[ii++]= array_u16[1];
        radiopacket[ii++]= array_u16[0];

        array_u16 = (byte*) &nfc.TV3;
        radiopacket[ii++]= array_u16[1];
        radiopacket[ii++]= array_u16[0];

        array_u16 = (byte*) &nfc.TV4;
        radiopacket[ii++]= array_u16[1];
        radiopacket[ii++]= array_u16[0];

        array_u16 = (byte*) &nfc.TA1;
        radiopacket[ii++]= array_u16[1];
        radiopacket[ii++]= array_u16[0];

        array_u16 = (byte*) &nfc.TA2;
        radiopacket[ii++]= array_u16[1];
        radiopacket[ii++]= array_u16[0];

        array_u16 = (byte*) &nfc.TA3;
        radiopacket[ii++]= array_u16[1];
        radiopacket[ii++]= array_u16[0];

        array_u16 = (byte*) &nfc.TA4;
        radiopacket[ii++]= array_u16[1];
        radiopacket[ii++]= array_u16[0];

        radiopacket[ii++]= nfc.delais_minute;

        radiopacket[ii++]= nfc.node_compost_addr[0];
        radiopacket[ii++]= nfc.node_compost_addr[1];
        radiopacket[ii++]= nfc.node_compost_addr[2];
        radiopacket[ii++]= nfc.node_compost_addr[3];

        radiopacket[ii++]= nfc.node_compost_cfg[0];
        radiopacket[ii++]= nfc.node_compost_cfg[1];
        radiopacket[ii++]= nfc.node_compost_cfg[2];
        radiopacket[ii++]= nfc.node_compost_cfg[3];

        radiopacket[ii]= FEATHER_MSG_END;
        Serial1.print("packet to send : ");Serial1.println(ii);
        rf95->send((uint8_t *)radiopacket, ii);
        rf95->waitPacketSent();
        */
      }


    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_SEND_ALL_TEMP
      && buf[2]==NODE_SSR_ADDR
      && buf[3]==SEND_ALL_TEMP
      && buf[4]==FEATHER_MSG_END) {
        Serial1.println("parseRF_data() : SEND_ALL_TEMP");
        SendAllTemp();
        if(ReceiveRFData()){
            parseRF_data();
        }


/*
        byte* float_array;
        byte* array_u16;
        byte* uint32_array;

        radiopacket[0]=FEATHER_MSG_HEADER;
        radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
        radiopacket[2]=NODE_SSR_ADDR;
        radiopacket[3]=SEND_ALL_TEMP;

        int ii=4;
        for(int jj=0;jj<NB_NODES;jj++){
          cnd_array[jj].new_data = compost_node_new_data[jj];
          radiopacket[ii++]=cnd_array[jj].node_address;

          radiopacket[ii++]=cnd_array[jj].node_cfg;

          uint32_array = (byte*) &cnd_array[jj].timestamp;
          radiopacket[ii++]= uint32_array[3];
          radiopacket[ii++]= uint32_array[2];
          radiopacket[ii++]= uint32_array[1];
          radiopacket[ii++]= uint32_array[0];

          float_array = (byte*) &cnd_array[jj].ntc_1;
//          Serial1.println(nodes_data[jj].temp[TEMP_1]);
          radiopacket[ii++]= float_array[3];
          radiopacket[ii++]= float_array[2];
          radiopacket[ii++]= float_array[1];
          radiopacket[ii++]= float_array[0];

          float_array = (byte*) &cnd_array[jj].ntc_2;
//          Serial1.println(nodes_data[jj].temp[TEMP_2]);
          radiopacket[ii++]= float_array[3];
          radiopacket[ii++]= float_array[2];
          radiopacket[ii++]= float_array[1];
          radiopacket[ii++]= float_array[0];

          float_array = (byte*) &cnd_array[jj].bme_humidity;
//          Serial1.println(nodes_data[jj].temp[TEMP_3]);
          radiopacket[ii++]= float_array[3];
          radiopacket[ii++]= float_array[2];
          radiopacket[ii++]= float_array[1];
          radiopacket[ii++]= float_array[0];

          float_array = (byte*) &cnd_array[jj].bme_temp;
//          Serial1.println(nodes_data[jj].temp[TEMP_3]);
          radiopacket[ii++]= float_array[3];
          radiopacket[ii++]= float_array[2];
          radiopacket[ii++]= float_array[1];
          radiopacket[ii++]= float_array[0];

          float_array = (byte*) &cnd_array[jj].bme_pression;
//          Serial1.println(nodes_data[jj].temp[TEMP_3]);
          radiopacket[ii++]= float_array[3];
          radiopacket[ii++]= float_array[2];
          radiopacket[ii++]= float_array[1];
          radiopacket[ii++]= float_array[0];

          array_u16 = (byte*) &cnd_array[jj].conductivite;
//          Serial1.println(nodes_data[jj].conductivite);
          radiopacket[ii++]= array_u16[1];
          radiopacket[ii++]= array_u16[0];

          float_array = (byte*) &cnd_array[jj].batt_voltage;
//          Serial1.println(nodes_data[jj].battery_voltage);
          radiopacket[ii++]= float_array[3];
          radiopacket[ii++]= float_array[2];
          radiopacket[ii++]= float_array[1];
          radiopacket[ii++]= float_array[0];

          radiopacket[ii++]=cnd_array[jj].delay_minutes;

          radiopacket[ii++]=cnd_array[jj].txpower;

          radiopacket[ii++]=cnd_array[jj].last_rssi;

          radiopacket[ii++]=cnd_array[jj].new_data;
          radiopacket[ii++]=cnd_array[jj].clock_ok;

        }

        // Relais
        // Temperature moyenne
        float_array = (byte*) &ssrd.t_avg;
        radiopacket[ii++]= float_array[3];
        radiopacket[ii++]= float_array[2];
        radiopacket[ii++]= float_array[1];
        radiopacket[ii++]= float_array[0];
//        Serial1.println(t_avg);


        // Point de consigne actuelle
        radiopacket[ii++] = ssrd.current_PC;

        // Etat du relais
        radiopacket[ii++] = ssrd.ssr_state;
        radiopacket[ii]= FEATHER_MSG_END;
        Serial1.print("packet to send : ");Serial1.println(ii);
        rf95->send((uint8_t *)radiopacket, ii);
        rf95->waitPacketSent();
        clear_compost_nodes_new_data();
*/
      }
      else if (buf[0]==FEATHER_MSG_HEADER
        && buf[1]==FEATHER_MSG_SET_DATA
        && buf[2]==NODE_SSR_ADDR
        && buf[3]==RELAY_THRESHOLD) {
/*          float_array_t_consigne[3]=buf[4];
          float_array_t_consigne[2]=buf[5];
          float_array_t_consigne[1]=buf[6];
          float_array_t_consigne[0]=buf[7];
          writeEEPROM(I2C_EEPROM_ADDRESS, EEPROM_ADDR_SETPOINT + 0, buf[7]);
          writeEEPROM(I2C_EEPROM_ADDRESS, EEPROM_ADDR_SETPOINT + 1, buf[6]);
          writeEEPROM(I2C_EEPROM_ADDRESS, EEPROM_ADDR_SETPOINT + 2, buf[5]);
          writeEEPROM(I2C_EEPROM_ADDRESS, EEPROM_ADDR_SETPOINT + 3, buf[4]);

          memcpy(&t_consigne,&float_array_t_consigne,sizeof(t_consigne));
          ssr_setpoint = t_consigne;
          Serial1.print("New setpoint: ");
          Serial1.println(ssr_setpoint);
          */
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_SET_DATA
      && buf[2]==NODE_SSR_ADDR
      && buf[3]==DELAY_BETWEEN_READS
      && buf[5]==FEATHER_MSG_END){
        nfc.delais_minute = buf[4];
        Serial1.print("parseRF_data() :  DELAY_BETWEEN_READS ");
        Serial1.println(nfc.delais_minute);
        writeEEPROM_NFCConfig();
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_SET_DATA
      && buf[2]==NODE_SSR_ADDR
      && buf[3]==RELAIS_CFG){
        Serial1.println("parseRF_data() : RELAIS_CFG");
        int ii = 4;
        float f_v;
        uint16_t u16_v;

        // PC_1
        float_array[3]=buf[ii++];
        float_array[2]=buf[ii++];
        float_array[1]=buf[ii++];
        float_array[0]=buf[ii++];
        memcpy(&f_v,&float_array,sizeof(f_v));
        nfc.PC1=f_v;
        Serial1.print("\tPC1 : ");
        Serial1.println(f_v,2);

        // PC_2
        float_array[3]=buf[ii++];
        float_array[2]=buf[ii++];
        float_array[1]=buf[ii++];
        float_array[0]=buf[ii++];
        memcpy(&f_v,&float_array,sizeof(f_v));
        nfc.PC2=f_v;
        Serial1.print("\tPC2 : ");
        Serial1.println(f_v,2);

        // PC_3
        float_array[3]=buf[ii++];
        float_array[2]=buf[ii++];
        float_array[1]=buf[ii++];
        float_array[0]=buf[ii++];
        memcpy(&f_v,&float_array,sizeof(f_v));
        nfc.PC3=f_v;
        Serial1.print("\tPC3 : ");
        Serial1.println(f_v,2);

        // PC_4
        float_array[3]=buf[ii++];
        float_array[2]=buf[ii++];
        float_array[1]=buf[ii++];
        float_array[0]=buf[ii++];
        memcpy(&f_v,&float_array,sizeof(f_v));
        nfc.PC4=f_v;
        Serial1.print("\tPC4 : ");
        Serial1.println(f_v,2);

        // TV1
        u16_array[1]=buf[ii++];
        u16_array[0]=buf[ii++];
        memcpy(&u16_v,&u16_array,sizeof(u16_v));
        nfc.TV1=u16_v;
        Serial1.print("\tTV1 : ");
        Serial1.println(u16_v);

        // TV2
        u16_array[1]=buf[ii++];
        u16_array[0]=buf[ii++];
        memcpy(&u16_v,&u16_array,sizeof(u16_v));
        nfc.TV2=u16_v;
        Serial1.print("\tTV2 : ");
        Serial1.println(u16_v);

        // TV3
        u16_array[1]=buf[ii++];
        u16_array[0]=buf[ii++];
        memcpy(&u16_v,&u16_array,sizeof(u16_v));
        nfc.TV3=u16_v;
        Serial1.print("\tTV3 : ");
        Serial1.println(u16_v);

        // TV4
        u16_array[1]=buf[ii++];
        u16_array[0]=buf[ii++];
        memcpy(&u16_v,&u16_array,sizeof(u16_v));
        nfc.TV4=u16_v;
        Serial1.print("\tTV4 : ");
        Serial1.println(u16_v);

        // TA1
        u16_array[1]=buf[ii++];
        u16_array[0]=buf[ii++];
        memcpy(&u16_v,&u16_array,sizeof(u16_v));
        nfc.TA1=u16_v;
        Serial1.print("\tTA1 : ");
        Serial1.println(u16_v);

        // TA2
        u16_array[1]=buf[ii++];
        u16_array[0]=buf[ii++];
        memcpy(&u16_v,&u16_array,sizeof(u16_v));
        nfc.TA2=u16_v;
        Serial1.print("\tTA2 : ");
        Serial1.println(u16_v);

        // TA3
        u16_array[1]=buf[ii++];
        u16_array[0]=buf[ii++];
        memcpy(&u16_v,&u16_array,sizeof(u16_v));
        nfc.TA3=u16_v;
        Serial1.print("\tTA3 : ");
        Serial1.println(u16_v);

        // TA4
        u16_array[1]=buf[ii++];
        u16_array[0]=buf[ii++];
        memcpy(&u16_v,&u16_array,sizeof(u16_v));
        nfc.TA4=u16_v;
        Serial1.print("\tTA4 : ");
        Serial1.println(u16_v);

        // delais_minute
        nfc.delais_minute = buf[ii++];
        Serial1.print("\tDelais Minute : ");
        Serial1.println(nfc.delais_minute);

        nfc.node_compost_addr[0]=buf[ii++];
        nfc.node_compost_addr[1]=buf[ii++];
        nfc.node_compost_addr[2]=buf[ii++];
        nfc.node_compost_addr[3]=buf[ii++];

        nfc.node_compost_cfg[0]=buf[ii++];
        nfc.node_compost_cfg[1]=buf[ii++];
        nfc.node_compost_cfg[2]=buf[ii++];
        nfc.node_compost_cfg[3]=buf[ii++];

        for(int j=0;j<4;j++){
          for(int i = 0;i<16;i++){
            nfc.node_compost_text[j][i]=buf[ii++];
          }
        }

        writeEEPROM_NFCConfig();
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_GET_DATA
      && buf[2]==NODE_SSR_ADDR
      && buf[3]==RELAY_THRESHOLD
      && buf[4]==FEATHER_MSG_END) {
    	   SendSetpoint(NODE_SSR_ADDR);
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_GET_DATA
      && buf[2]==NODE_SSR_ADDR
      && buf[3]==READ_BATTERY_VOLTAGE
      && buf[4]==FEATHER_MSG_END) {
    	   SendBatVoltage(NODE_SSR_ADDR);
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_SET_DATA
      && buf[2]==NODE_SSR_ADDR
      && buf[3]==TURN_ON_RELAY
      && buf[4]==FEATHER_MSG_END)	{
        setFanMotor(true);
        ssr_ModeAuto = false;
        ssrd.ssr_state = 1;
        Serial1.println("Mode Manuel");
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_SET_DATA
      && buf[2]==NODE_SSR_ADDR
      && buf[3]==MODE_AUTO
      && buf[4]==FEATHER_MSG_END) {
        ssr_ModeAuto = true;
        Serial1.println("Mode Auto");
//        ssr_etat = 2;
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_SET_DATA
      && buf[2]==NODE_SSR_ADDR
      && buf[3]==TURN_OFF_RELAY
      && buf[4]==FEATHER_MSG_END) {
        setFanMotor(false);
        ssr_ModeAuto = false;
        ssrd.ssr_state = 0;
        Serial1.println("Mode Manuel : Off");
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_GET_DATA
      && buf[2]==NODE_SSR_ADDR
      && buf[3]==RELAY_STATE
      && buf[4]==FEATHER_MSG_END) {
        SendRelayState(NODE_SSR_ADDR);
    }
    else if (buf[0]==FEATHER_MSG_HEADER
      && buf[1]==FEATHER_MSG_GET_DATA
      && buf[2]==NODE_SSR_ADDR
      && buf[3]==LAST_RSSI
      && buf[4]==FEATHER_MSG_END) {
    	   SendLastRssi(NODE_SSR_ADDR);
    }
    memset(buf,0,sizeof(buf));
    len = RH_RF95_MAX_MESSAGE_LEN;
}

/*
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
*/
void NodeFeatherFanController::SendSetDateTime(byte node_adress){
  Serial1.println("SendSetDateTime() : FEATHER_MSG_SET_CLOCK");
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

void NodeFeatherFanController::SendAllCfg(void){
  byte* float_array;
  byte* array_u16;

  int ii=4;

  radiopacket[0]=FEATHER_MSG_HEADER;
  radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
  radiopacket[2]=NODE_SSR_ADDR;
  radiopacket[3]=SEND_ALL_CFG;

  float_array = (byte*) &nfc.PC1;
  radiopacket[ii++]= float_array[3];
  radiopacket[ii++]= float_array[2];
  radiopacket[ii++]= float_array[1];
  radiopacket[ii++]= float_array[0];

  float_array = (byte*) &nfc.PC2;
  radiopacket[ii++]= float_array[3];
  radiopacket[ii++]= float_array[2];
  radiopacket[ii++]= float_array[1];
  radiopacket[ii++]= float_array[0];

  float_array = (byte*) &nfc.PC3;
  radiopacket[ii++]= float_array[3];
  radiopacket[ii++]= float_array[2];
  radiopacket[ii++]= float_array[1];
  radiopacket[ii++]= float_array[0];

  float_array = (byte*) &nfc.PC4;
  radiopacket[ii++]= float_array[3];
  radiopacket[ii++]= float_array[2];
  radiopacket[ii++]= float_array[1];
  radiopacket[ii++]= float_array[0];

  array_u16 = (byte*) &nfc.TV1;
  radiopacket[ii++]= array_u16[1];
  radiopacket[ii++]= array_u16[0];

  array_u16 = (byte*) &nfc.TV2;
  radiopacket[ii++]= array_u16[1];
  radiopacket[ii++]= array_u16[0];

  array_u16 = (byte*) &nfc.TV3;
  radiopacket[ii++]= array_u16[1];
  radiopacket[ii++]= array_u16[0];

  array_u16 = (byte*) &nfc.TV4;
  radiopacket[ii++]= array_u16[1];
  radiopacket[ii++]= array_u16[0];

  array_u16 = (byte*) &nfc.TA1;
  radiopacket[ii++]= array_u16[1];
  radiopacket[ii++]= array_u16[0];

  array_u16 = (byte*) &nfc.TA2;
  radiopacket[ii++]= array_u16[1];
  radiopacket[ii++]= array_u16[0];

  array_u16 = (byte*) &nfc.TA3;
  radiopacket[ii++]= array_u16[1];
  radiopacket[ii++]= array_u16[0];

  array_u16 = (byte*) &nfc.TA4;
  radiopacket[ii++]= array_u16[1];
  radiopacket[ii++]= array_u16[0];

  radiopacket[ii++]= nfc.delais_minute;

  radiopacket[ii++]= nfc.node_compost_addr[0];
  radiopacket[ii++]= nfc.node_compost_addr[1];
  radiopacket[ii++]= nfc.node_compost_addr[2];
  radiopacket[ii++]= nfc.node_compost_addr[3];

  radiopacket[ii++]= nfc.node_compost_cfg[0];
  radiopacket[ii++]= nfc.node_compost_cfg[1];
  radiopacket[ii++]= nfc.node_compost_cfg[2];
  radiopacket[ii++]= nfc.node_compost_cfg[3];

  for(int j=0;j<4;j++){
    for(int i=0;i<16;i++){
        radiopacket[ii++]= nfc.node_compost_text[j][i];
    }
  }

  radiopacket[ii]= FEATHER_MSG_END;
  Serial1.print("packet to send : ");Serial1.println(ii);
  rf95->send((uint8_t *)radiopacket, ii);
  rf95->waitPacketSent();
}

void NodeFeatherFanController::SendAllTemp(void){
  byte* float_array;
  byte* array_u16;
  byte* uint32_array;

  radiopacket[0]=FEATHER_MSG_HEADER;
  radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
  radiopacket[2]=NODE_SSR_ADDR;
  radiopacket[3]=SEND_ALL_TEMP;

  int ii=4;
  for(int jj=0;jj<NB_NODES;jj++){
    cnd_array[jj].new_data = compost_node_new_data[jj];
    radiopacket[ii++]=cnd_array[jj].node_address;

    radiopacket[ii++]=cnd_array[jj].node_cfg;

    uint32_array = (byte*) &cnd_array[jj].timestamp;
    radiopacket[ii++]= uint32_array[3];
    radiopacket[ii++]= uint32_array[2];
    radiopacket[ii++]= uint32_array[1];
    radiopacket[ii++]= uint32_array[0];

    float_array = (byte*) &cnd_array[jj].ntc_1;
//          Serial1.println(nodes_data[jj].temp[TEMP_1]);
    radiopacket[ii++]= float_array[3];
    radiopacket[ii++]= float_array[2];
    radiopacket[ii++]= float_array[1];
    radiopacket[ii++]= float_array[0];

    float_array = (byte*) &cnd_array[jj].ntc_2;
//          Serial1.println(nodes_data[jj].temp[TEMP_2]);
    radiopacket[ii++]= float_array[3];
    radiopacket[ii++]= float_array[2];
    radiopacket[ii++]= float_array[1];
    radiopacket[ii++]= float_array[0];

    float_array = (byte*) &cnd_array[jj].bme_humidity;
//          Serial1.println(nodes_data[jj].temp[TEMP_3]);
    radiopacket[ii++]= float_array[3];
    radiopacket[ii++]= float_array[2];
    radiopacket[ii++]= float_array[1];
    radiopacket[ii++]= float_array[0];

    float_array = (byte*) &cnd_array[jj].bme_temp;
//          Serial1.println(nodes_data[jj].temp[TEMP_3]);
    radiopacket[ii++]= float_array[3];
    radiopacket[ii++]= float_array[2];
    radiopacket[ii++]= float_array[1];
    radiopacket[ii++]= float_array[0];

    float_array = (byte*) &cnd_array[jj].bme_pression;
//          Serial1.println(nodes_data[jj].temp[TEMP_3]);
    radiopacket[ii++]= float_array[3];
    radiopacket[ii++]= float_array[2];
    radiopacket[ii++]= float_array[1];
    radiopacket[ii++]= float_array[0];

    array_u16 = (byte*) &cnd_array[jj].conductivite;
//          Serial1.println(nodes_data[jj].conductivite);
    radiopacket[ii++]= array_u16[1];
    radiopacket[ii++]= array_u16[0];

    float_array = (byte*) &cnd_array[jj].batt_voltage;
//          Serial1.println(nodes_data[jj].battery_voltage);
    radiopacket[ii++]= float_array[3];
    radiopacket[ii++]= float_array[2];
    radiopacket[ii++]= float_array[1];
    radiopacket[ii++]= float_array[0];

    radiopacket[ii++]=cnd_array[jj].delay_minutes;

    radiopacket[ii++]=cnd_array[jj].txpower;

    radiopacket[ii++]=cnd_array[jj].last_rssi;

    radiopacket[ii++]=cnd_array[jj].new_data;
    radiopacket[ii++]=cnd_array[jj].clock_ok;

  }

  // Relais
  // Temperature moyenne
  float_array = (byte*) &ssrd.t_avg;
  radiopacket[ii++]= float_array[3];
  radiopacket[ii++]= float_array[2];
  radiopacket[ii++]= float_array[1];
  radiopacket[ii++]= float_array[0];
//        Serial1.println(t_avg);


  // Point de consigne actuelle
  radiopacket[ii++] = ssrd.current_PC;

  // Etat du relais
  radiopacket[ii++] = ssrd.ssr_state;
  radiopacket[ii]= FEATHER_MSG_END;
  Serial1.print("packet to send : ");Serial1.println(ii);
  clear_compost_nodes_new_data();
  rf95->send((uint8_t *)radiopacket, ii);
  rf95->waitPacketSent();
}

void NodeFeatherFanController::SendSSRReady(byte node_adress){
  Serial1.println("SendSSRReady() : FEATHER_MSG_SSR_READY");
  radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_SSR_READY;
	radiopacket[2]=node_adress;
  radiopacket[3]=nfc.delais_minute;
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
	float_array = (byte*) &ssr_setpoint;
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

void NodeFeatherFanController::setFanMotor(bool motor_state){
  Serial1.println("setFanMotor()");
	if (motor_state){
		digitalWrite(18, HIGH);
    Serial1.println("\tFanMotor ON");
  }
	else {
		digitalWrite(18, LOW);
    Serial1.println("\tFanMotor OFF");
  }
}

void NodeFeatherFanController::SendRelayState(byte node_address){
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
	radiopacket[2]=node_address;
	radiopacket[3]=RELAY_STATE;
	radiopacket[4]=ssr_etat;
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

void NodeFeatherFanController::SendGetDateTime(byte node_address){
  Serial1.println("SendGetDateTime() : FEATHER_MSG_GET_DATETIME");
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_GET_DATETIME;
	radiopacket[2]=node_address;
	radiopacket[3]=FEATHER_MSG_END;
	rf95->send((uint8_t *)radiopacket, 4);
	rf95->waitPacketSent();
}

void NodeFeatherFanController::SendSSR_ready_for_commands(byte node_address) {
  Serial1.println("SendSSR_ready_for_commands() : SSR_READY_FOR_COMMAND");
	radiopacket[0]=FEATHER_MSG_HEADER;
	radiopacket[1]=FEATHER_MSG_READY_FOR_COMMANDS;
	radiopacket[2]=node_address;
	radiopacket[3]=FEATHER_MSG_END;
	rf95->send((uint8_t *)radiopacket, 4);
	rf95->waitPacketSent();
}


void NodeFeatherFanController::SendCompostNodeData(byte node_address, byte compost_node_id){
  radiopacket[0]=FEATHER_MSG_HEADER;
  radiopacket[1]=FEATHER_MSG_RESPONSE_DATA;
  radiopacket[2]=NODE_SSR_ADDR;
  radiopacket[3]=FEATHER_MSG_SEND_COMPOST_NODE_DATA;
  radiopacket[4]=compost_node_id;
  int j=5;
  //uint8_t node_id = 0;

  cnd_array[compost_node_id].new_data = compost_node_new_data[compost_node_id];
  byte* cnd_pointer = (byte*)&cnd_array[compost_node_id];
  CompostNodeData cnd_p = cnd_array[compost_node_id];

  Serial1.print("\tNODE_ADDRESS : ");Serial1.println(cnd_p.node_address);
  Serial1.print("\tNODE_CFG : ");Serial1.println(cnd_p.node_cfg);
  Serial1.print("\tTIMSTAMP : ");Serial1.println(cnd_p.timestamp);
  Serial1.print("\tNTC_1 : "); Serial1.println(cnd_p.ntc_1,2);
  Serial1.print("\tNTC_2 : "); Serial1.println(cnd_p.ntc_2,2);
  Serial1.print("\tBME_HUMIDITY : "); Serial1.println(cnd_p.bme_humidity,2);
  Serial1.print("\tBME_TEMP : "); Serial1.println(cnd_p.bme_temp,2);
  Serial1.print("\tBME_PRESSION : "); Serial1.println(cnd_p.bme_pression,2);
  Serial1.print("\tCONDUCTIVITE : "); Serial1.println(cnd_p.conductivite);
  Serial1.print("\tBATT_VOLTAGE : "); Serial1.println(cnd_p.batt_voltage,2);
  Serial1.print("\tDELAY_MINUTES : "); Serial1.println(cnd_p.delay_minutes,2);
  Serial1.print("\tLAST_RSSI : "); Serial1.println(cnd_p.last_rssi);
  Serial1.print("\tTX_POWER : "); Serial1.println(cnd_p.txpower);

  for(uint16_t i=0;i<sizeof(struct CompostNodeData);i++){
    radiopacket[j++] = cnd_pointer[i];
  }

  radiopacket[j]= FEATHER_MSG_END;

  Serial1.print("radiopacket size : ");Serial1.println(j);
  rf95->send((uint8_t *)radiopacket, j);
  rf95->waitPacketSent();
}

void NodeFeatherFanController::SendSsrNodeData(byte node_address){

}

void NodeFeatherFanController::set_current_PC(void){
  Serial1.println("set_current_PC()");
  Serial1.print("\tCurrent PC : ");
  if(do_avg){
    if(ssrd.t_avg < nfc.PC1){
      ssrd.current_PC = PC1_ID;
      Serial1.println("PC1");
    }
    else if(ssrd.t_avg >= nfc.PC1 && ssrd.t_avg < nfc.PC2 ){
      ssrd.current_PC = PC2_ID;
      Serial1.println("PC2");
    }
    else if(ssrd.t_avg >= nfc.PC2 && ssrd.t_avg < nfc.PC3 ){
      ssrd.current_PC = PC3_ID;
      Serial1.println("PC3");
    }
    else if(ssrd.t_avg >= nfc.PC3 && ssrd.t_avg < nfc.PC4 ){
      ssrd.current_PC = PC4_ID;
      Serial1.println("PC4");
    }
    do_avg=false;
  }
  else{
    ssrd.current_PC = PC0_ID;
    Serial1.println("PC0");
  }
}

void NodeFeatherFanController::readEEpromConfiguration(void){
//  ssr_setpoint = 20;
//  nfc.delais_minute = 1;
}

void NodeFeatherFanController::resetAlarm(void){
  byte seconds = 0;
  byte minutes = 0;
  byte hours = 0;
  byte day = 1;
  byte month = 1;
  byte year = 1;

  Serial1.println("resetAlarm()");
  Serial1.print("\tSetting alarmSeconds : ");Serial1.println(alarmSeconds);
  Serial1.print("\tSetting alarmMinutes : ");Serial1.println(alarmMinutes);
  zerortc.setTime(hours, minutes, seconds);
  zerortc.setDate(day, month, year);

  zerortc.setAlarmTime(alarmHours, alarmMinutes, alarmSeconds);
  zerortc.enableAlarm(zerortc.MATCH_HHMMSS);
}

void NodeFeatherFanController::setRTCAlarm(byte seconds){
  Serial1.println("setRTCAlarm()");
  byte minutes_alarm;
  byte seconds_alarm;

  if(seconds >=60){
    minutes_alarm = seconds / 60;
    seconds_alarm = seconds - (minutes_alarm * 60);
    alarmSeconds = seconds_alarm;
    alarmMinutes = minutes_alarm;
  }
  else{
    alarmSeconds = seconds;
    alarmMinutes = 0;
  }

  Serial1.print("\tminutes : "); Serial1.println(alarmMinutes);
  Serial1.print("\tseconds : "); Serial1.println(alarmSeconds);
}

int16_t NodeFeatherFanController::getNodeID(uint8_t address){
  int16_t node_id;

  node_id = -1;

  for(int i=0;i<NB_NODES;i++){
//    Serial1.print("\t Node address : "); Serial1.println(nfc.node_compost_addr[i]);
    if(nfc.node_compost_addr[i] == address){
      node_id=i;
      break;
    }
  }
  Serial1.print("\t Node address : "); Serial1.println(buf[2]);
  return node_id;
}
