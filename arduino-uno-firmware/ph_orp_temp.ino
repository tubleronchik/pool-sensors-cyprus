#include "DFRobot_PH.h"
#include "DFRobot_ORP_PRO.h"
#include <OneWire.h>

#define PH_PIN A0
#define ORP_PIN A1
#define TEMP_PIN 2
#define ADC_RES 1024.0
#define V_REF 5000

float phVoltage, phValue, orpVoltage, orpValue, temperature = 25;

DFRobot_PH ph;
DFRobot_ORP_PRO ORP(0);
OneWire ds(TEMP_PIN);

void setup()
{
    Serial.begin(115200);  
    ph.begin();
}

void loop()
{
    static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U){                  //time interval: 1s
        timepoint = millis();
        temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
        phVoltage = (unsigned long)analogRead(PH_PIN)/ADC_RES*V_REF;  // read the voltage
        phValue = ph.readPH(phVoltage, temperature);  // convert voltage to pH with temperature compensation
        Serial.print("temperature:");
        Serial.print(temperature, 1);
        Serial.print("^C  pH:");
        Serial.println(phValue, 2);

        // ORP measure
        orpVoltage = ((unsigned long)analogRead(ORP_PIN) * V_REF + ADC_RES / 2) / ADC_RES;
        orpValue = ORP.getORP(orpVoltage);
        Serial.print("ORP value is : ");
        Serial.print(orpValue, 2);
        Serial.println("mV");
    }
    ph.calibration(phVoltage, temperature);           // calibration process by Serail CMD
}

float readTemperature()
{
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}