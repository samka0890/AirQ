//-----------------------------------------------------------------
#if CONFIG_FREERTOS_UNICORE
#define CORE_0 0
#else
#define CORE_1 1
#endif

//-----------------------------------------------------------------

#include "DHT.h"

#define DHTPIN 14      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22  // DHT 11
DHT dht(DHTPIN, DHTTYPE);

//-----------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1015 ads; /* Use this for the 12-bit version */
int16_t adc0, adc1;

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

//------------------------------------------------------------------------------------------
#include <WiFi.h>
#include <PubSubClient.h>

const char *ssid = "Samka";
const char *password = "lcqq1655";

const char *mqtt_broker = "192.168.94.164";
const char *mqtt_username = "samka";
const char *mqtt_password = "12345678";
const int mqtt_port = 1883;

const char *tempTopic = "sensorUnit1/temp";
const char *humTopic = "sensorUnit1/hum";
const char *coTopic = "sensorUnit1/co";
const char *vocTopic = "sensorUnit1/voc";

WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[50];
//-----------------------------------------------------------------
#include <iostream>
#include <string>
using namespace std;

//-----------------------------------------------------------------
const float VC = 5, V3 = 3.3;

// MQ135
float VRL_135, RL_135 = 1000, RS_135, R0_135 = 1000, x_135;

// MQ7
float VRL_7, RL_7 = 1000, RS_7, R0_7 = 2120, x_7;


//-----------------------------------------------------------------
void DHT22_task(void *pvParameters);
void MQ135_task(void *pvParameters);
void MQ7_task(void *pvParameters);
void MQTT_task(void *pvParameters);

//-----------------------------------------------------------------

struct SensorUnit {
  int temp = 24;
  int hum = 45;
  int CO;
  int VOC;

};

typedef struct SensorUnit SensorUnit;
SensorUnit s1;

void setup() {
  Serial.begin(115200);

  while (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
  }
  ads.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV

  setup_wifi();
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  xTaskCreate(DHT22_task, "DHT22 task", 1100, NULL, 2, NULL);
  xTaskCreate(MQ135_task, "MQ135 task", 1300, NULL, 3, NULL);
  xTaskCreate(MQ7_task, "MQ7 task", 1300, NULL, 3, NULL);
  xTaskCreate(MQTT_task, "MQTT task", 3500, NULL, 3, NULL);
}

void loop() {
}

void DHT22_task(void *pvParameters) {
  dht.begin();
  for (;;) {
    s1.hum = (int)dht.readHumidity();
    s1.temp = (int)dht.readTemperature();
    Serial.print(F("Humidity: "));
    Serial.print(s1.hum);
    Serial.print(F("%  Temperature: "));
    Serial.print(s1.temp);
    Serial.print(F("°C "));
    Serial.println();

    // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.print("DHT_task stack high watermark: ");
    // Serial.println(uxHighWaterMark);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void MQ7_task(void *pvParameters) {

  for (;;) {
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/false);
    while (!ads.conversionComplete())
      ;
    adc0 = ads.getLastConversionResults();
    VRL_7 = ads.computeVolts(adc0);

    Serial.print("VRL_7 = ");
    Serial.println(VRL_7);

    RS_7 = (VC - VRL_7) * RL_7 / VRL_7;
    // // Serial.print("RS = ");
    // // Serial.println(RS);

    x_7 = RS_7 / R0_7;
    s1.CO = (int)(pow((x_7 / 19.709), (-1 / 0.652)) + 10 + 7);

    Serial.print("CO = ");
    Serial.println(s1.CO);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
  }
}

void MQ135_task(void *pvParameters) {
  float test;
  for (;;) {
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_1, /*continuous=*/false);
    while (!ads.conversionComplete())
      ;
    adc1 = ads.getLastConversionResults();
    VRL_135 = ads.computeVolts(adc1);

    Serial.print("VRL_135 = ");
    Serial.println(VRL_135);

    RS_135 = (VC - VRL_135) * RL_135 / VRL_135;
    // Serial.print("RS_135 = ");
    // Serial.println(RS_135);

    x_135 = RS_135 / R0_135;
    test = pow((x_135 / 6.362), (-1 / 0.401)) * 10;
    Serial.print("Test = ");
    Serial.println(test);
    s1.VOC = (int)(pow((x_135 / 6.362), (-1 / 0.401)) * 1000)+23;
    // // CO2 = pow((x/5.1435),(-1/0.348));
    // // CO = pow((x/4.9589),(-1/0.25));
    Serial.print("VOC = ");
    Serial.println(s1.VOC);
    // Serial.print("CO2 = ");
    // Serial.println(CO2);

    // Serial.print("CO = ");
    // Serial.println(CO);

    // Serial.println();


    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    Serial.print("MQ135_task stack high watermark: ");
    Serial.println(uxHighWaterMark);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void MQTT_task(void *pvParameters) {
  string str;
  int last_hum = 0, last_temp = 0;
  int last_CO = 0, last_VOCs = 0;
  for (;;) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

    if (last_temp != s1.temp) {
      str = to_string(s1.temp);
      client.publish(tempTopic, str.c_str());
      last_temp = s1.temp;
      Serial.print(" String Temp: ");
      Serial.println(str.c_str());
    }

    if (last_hum != s1.hum) {
      str = to_string(s1.hum);
      client.publish(humTopic, str.c_str());
      last_hum = s1.hum;
      Serial.print(" String Hum: ");
      Serial.println(str.c_str());
    }
    if(last_CO != s1.CO){
      str = to_string(s1.CO);
      client.publish(coTopic, str.c_str());
      last_CO = s1.CO;
      Serial.print(" String CO: ");
      Serial.println(str.c_str());

    }
    if(last_VOCs != s1.VOC){
      str = to_string(s1.VOC);
      client.publish(vocTopic, str.c_str());
      last_VOCs = s1.VOC;
      Serial.print(" String VOC: ");
      Serial.println(str.c_str());
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup_wifi() {
  delay(10);
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
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, uint8_t *message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Public emqx mqtt broker connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}
