#include "DFRobot_PH.h"
#include "DFRobot_ORP_PRO.h"
#include <OneWire.h>
#include "esp8266.h"

#define PH_PIN A0
#define ORP_PIN A1
#define TEMP_PIN 2
#define ADC_RES 1024.0
#define V_REF 5000

#define ssid		"test"
#define password	"12345678"

#define	serverPort	8081

#define PH_MESSAGE "ph_request"
#define ORP_MESSAGE "orp_request"
#define TEMP_MESSAGE "temp_request"

float phVoltage, phValue, orpVoltage, orpValue, temperature = 25;
char msg[20];

DFRobot_PH ph;
DFRobot_ORP_PRO ORP(0);
OneWire ds(TEMP_PIN);
Esp8266 wifi;

void setup()
{
    delay(2000);
    Serial.begin(115200);  
    ph.begin();
    wifi.begin(&Serial);
    if (wifi.connectAP(ssid, password)) {
        String ip_addr;
        ip_addr = wifi.getIP();
		Serial.print("connect ap sucessful! IP address: ");
        Serial.println(ip_addr);
	}
    if (wifi.setMultiConnect()) {
		Serial.println("set multi connect!");
	}	
	if (wifi.openTCPServer(serverPort, 180)) {  //180 secondes, if a client send no message in 180s, server will close it
		Serial.println("open TCP Server port "+ String(serverPort) + " OK!");
	}	
}

void loop()
{
    int state = wifi.getState();
    String new_message;
    switch (state) {
	    case WIFI_NEW_MESSAGE: 
            new_message = wifi.getMessage();
            Serial.println(String(wifi.getWorkingID()) + ":" + new_message); // debug
            if (new_message==PH_MESSAGE) {
                temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
                phVoltage = (unsigned long)analogRead(PH_PIN)/ADC_RES*V_REF;  // read the voltage
                phValue = ph.readPH(phVoltage, temperature);  // convert voltage to pH with temperature compensation
                Serial.print("temperature:");
                Serial.print(temperature, 1);
                Serial.print(", pH:");
                Serial.println(phValue, 2);
                snprintf(msg, 20, "%lf", phValue);
                wifi.sendMessage(msg);
            } else if (new_message==ORP_MESSAGE) {
                orpVoltage = ((unsigned long)analogRead(ORP_PIN) * V_REF + ADC_RES / 2) / ADC_RES;
                orpValue = ORP.getORP(orpVoltage);
                Serial.print("ORP value is : ");
                Serial.print(orpValue, 2);
                Serial.println("mV");
                snprintf(msg, 20, "%lf", orpValue);
                wifi.sendMessage(msg);
            } else if (new_message==TEMP_MESSAGE) {
                temperature = readTemperature();         // read your temperature sensor to execute temperature compensation
                Serial.print("temperature:");
                Serial.println(temperature, 1);
                snprintf(msg, 20, "%lf", temperature);
                wifi.sendMessage(msg);
            }
            wifi.setState(WIFI_IDLE);
            break;
	    case WIFI_CLOSED: 	// just print which connect is close, won't reconnect
            Serial.println(String(wifi.getFailConnectID()) + ":connect closed!");
            wifi.setState(WIFI_IDLE);
            break;
	    case WIFI_IDLE:
	    {
	    	int state = wifi.checkMessage(); 
	    	wifi.setState(state);
	    	break;
	    }
	    case WIFI_CLIENT_ON:
	    	Serial.println("New connection");
	    	wifi.setState(WIFI_IDLE);
	    	break;
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