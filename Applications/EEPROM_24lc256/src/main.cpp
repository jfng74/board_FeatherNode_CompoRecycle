#include <Arduino.h>
#include <Wire.h>

#define disk1 0x50    //Address of 24LC256 eeprom chip

unsigned int address = 0;

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

void setup(void)
{
  Serial.begin(9600);
  while (!Serial)
    {
    }
  Wire.begin();


  //writeEEPROM(disk1, address, 123);
  //writeEEPROM(disk1, address+1, 225);
  //writeEEPROM(disk1, address+2, 12);
  //writeEEPROM(disk1, address+3, 3);

}

void loop(){
  delay(200);
  Serial.println(readEEPROM(disk1, address++), DEC);
  }
