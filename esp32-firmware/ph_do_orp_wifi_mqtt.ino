#include "DFRobot_ESP_PH.h"
#include "DFRobot_ORP_PRO.h"
#include "EEPROM.h"
#include <WiFi.h>
#include <PubSubClient.h>

DFRobot_ESP_PH ph;
DFRobot_ORP_PRO ORP(0);
#define ESPADC 4096.0   //the esp Analog Digital Convertion value
#define ESPVOLTAGE 3300 //the esp voltage supply value
#define PH_PIN 4    //the esp gpio data pin number
#define ORP_PIN 0
#define DO_PIN 2
float phVoltage, phValue, orpVoltage, orpValue, doVoltage, doValue, temperature = 25;

const char* ssid = "";
const char* password = "";

const char* mqtt_server = "";
char msg[20];

#define PH_TOPIC "water_sensors/ph"
#define ORP_TOPIC "water_sensors/orp"
#define DO_TOPIC "water_sensors/do"

WiFiClient espClient;
PubSubClient client(espClient);

void mqttconnect() {
  /* Loop until reconnected */
  while (!client.connected()) {
    Serial.print("MQTT connecting ...");
    /* client ID */
    String clientId = "ESP32Client";
    /* connect now */
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      /* subscribe topic with default QoS 0*/
    } else {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
    }
  }
}

#define TWO_POINT_CALIBRATION 0

#define READ_TEMP (25) //Current water temperature ℃, Or temperature sensor function

//Single point calibration needs to be filled CAL1_V and CAL1_T
#define CAL1_V (131) //mv
#define CAL1_T (25)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};

int16_t readDO(uint32_t voltage_mv, uint8_t temperature_c)
{
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  return (voltage_mv * DO_Table[temperature_c] / V_saturation);
}

void setup()
{
  Serial.begin(115200);
  EEPROM.begin(32);//needed to permit storage of calibration value in eeprom
  ph.begin();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);
}

void loop()
{
  if (!client.connected()) {
    mqttconnect();
  }
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) //time interval: 1s
  {
    timepoint = millis();
    
    //////// Temperature read /////////
    //temperature = readTemperature();  // read your temperature sensor to execute temperature compensation
    Serial.print("temperature:");
    Serial.print(temperature, 1);
    Serial.println("^C");

    //////// pH read /////////
    phVoltage = analogRead(PH_PIN) / ESPADC * ESPVOLTAGE; // read the voltage
    Serial.print("pH voltage:");
    Serial.print(phVoltage, 4);
    phValue = ph.readPH(phVoltage, temperature); // convert voltage to pH with temperature compensation
    Serial.print(", pH value:");
    Serial.println(phValue, 4);
    snprintf(msg, 20, "%lf", phValue);
    client.publish(PH_TOPIC, msg);

    //////// ORP read /////////
    orpVoltage = ((unsigned long)analogRead(ORP_PIN) * ESPVOLTAGE + ESPADC / 2) / ESPADC;
    orpValue = ORP.getORP(orpVoltage);
    Serial.print("ORP value is : ");
    Serial.print(orpValue);
    Serial.println("mV");
    snprintf(msg, 20, "%lf", orpValue);
    client.publish(ORP_TOPIC, msg);


    //////// DO read /////////
    doVoltage = (unsigned long)analogRead(DO_PIN) * ESPVOLTAGE / ESPADC;
    doValue = readDO(doVoltage, temperature);
    Serial.print("DO value is : ");
    Serial.print(doValue);
    Serial.println(" ");
    snprintf(msg, 20, "%lf", doValue);
    client.publish(DO_TOPIC, msg);
  }
  ph.calibration(phVoltage, temperature); // calibration process by Serail CMD
}

float readTemperature()
{
  //add your code here to get the temperature from your temperature sensor
}