#include "DFRobot_PH.h"
#include "DFRobot_ORP_PRO.h"
#include <EEPROM.h>

#define PH_PIN A0
#define ORP_PIN A1
#define ADC_RES 1024.0
#define V_REF 5000
float phVoltage, phValue, orpVoltage, orpValue, temperature = 25;
DFRobot_PH ph;
DFRobot_ORP_PRO ORP(0);

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
        //temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
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
  //add your code here to get the temperature from your temperature sensor
}