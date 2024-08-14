#include <iostream>
#include <cmath> // for std::modf
#include <string> // for std::to_string
#include <sstream> // for std::stringstream
using namespace std;
//------------------------------------------------------------------------------------------
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Samka";
const char* password = "lcqq1655";

const char *mqtt_broker = "192.168.94.164";
const char *mqtt_username = "samka";
const char *mqtt_password = "12345678";
const int mqtt_port = 1883;

const char *ventTopic = "vent/start";
const char *dirTopic = "vent/dir";
const char *speedTopic = "vent/speed";
const char *speed1Topic = "vent/speed1";
const char *speed2Topic = "vent/speed2";


WiFiClient espClient;
PubSubClient client(espClient);

long lastMsg = 0;
char msg[50];

//------------------------------------------------------------------------------------------
#if CONFIG_FREERTOS_UNICORE
#define CORE_0 0
#else
#define CORE_1 1
#endif

//------------------------------------------------------------------------------------------
#define m_right 15
#define m_left 18

#define zero 14
#define speed 13

//------------------------------------------------------------------------------------------
int ventState[5]={0, 0, 0, 0, 0};
int ventStateLast[5]= {0, 0, 0, 0, 0};

const int freq = 500;
const int ledChannel = 0;
const int resolution = 8;

//------------------------------------------------------------------------------------------

void Motor_task(void *pvParameters);
void MQTT_task(void *pvParameters);

//------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  pinMode(m_right, OUTPUT);
  pinMode(m_left, OUTPUT);

  ledcAttach(speed, freq, resolution);
  // ledcAttach(speed, freq, resolution);
  ledcWrite(speed, 0);
  
  digitalWrite(m_right, 0);
  digitalWrite(m_left, 0);

  digitalWrite(m_right, 0);
  digitalWrite(m_left, 1);

  xTaskCreate(Motor_task,  "Motor task",  3000,  NULL,  3,  NULL);
  xTaskCreate(MQTT_task,  "MQTT task",  2500,  NULL,  2,  NULL);

}

void Motor_task(void *pvParameters)
{
  for(;;){
    if(ventState[0]){
      digitalWrite(m_right, 1);
      digitalWrite(m_left, 0);
      ledcWrite(speed, 200);
    }
    else{
      digitalWrite(m_right, 0);
      digitalWrite(m_left, 0);
      ledcWrite(speed, 0);
    }
    if(ventState[1] && ventState[0] ){

      if(ventState[1]!= ventStateLast[1])
      {
        ventStateLast[1] = ventState[1];
        digitalWrite(m_right, 0);
        digitalWrite(m_left, 0);
        vTaskDelay(6000/portTICK_PERIOD_MS);
      }
      
      digitalWrite(m_right, 0);
      digitalWrite(m_left, 1);
    }
    else if(!ventState[1] && ventState[0]){
      if(ventState[1]!= ventStateLast[1])
      {
        ventStateLast[1] = ventState[1];
        digitalWrite(m_right, 0);
        digitalWrite(m_left, 0);
        vTaskDelay(6000/portTICK_PERIOD_MS);
      }
      digitalWrite(m_right, 1);
      digitalWrite(m_left, 0);
    }
    if(ventState[2] && ventState[0]){
      ledcWrite(speed, 100);
    }
    else if(ventState[3] && ventState[0]){
      ledcWrite(speed, 175);
    }
    else if(ventState[4] && ventState[0]){
      ledcWrite(speed, 250);
    }
    vTaskDelay(400/portTICK_PERIOD_MS);
  }
}
void MQTT_task(void *pvParameters)
{
  for(;;)
  {
     if (!client.connected()) {
      reconnect();
    }
    client.loop();
    vTaskDelay(500/portTICK_PERIOD_MS);
  }
}
void setup_wifi() 
{
  vTaskDelay(10/portTICK_PERIOD_MS);
  
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500/portTICK_PERIOD_MS);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, uint8_t* message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  string messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  if (String(topic) == ventTopic) {
      Serial.print("Message: ");
      // Serial.println(messageTemp);
      ventStateLast[0] = ventState[0];
      ventState[0] = stoi(messageTemp);
      Serial.println(ventState[0]);
  }
  if (String(topic) == dirTopic) {
      Serial.print("Message: ");
      // Serial.println(messageTemp);
      ventStateLast[1] = ventState[1];
      ventState[1] = stoi(messageTemp);
      Serial.println(ventState[1]);
  }
  if (String(topic) == speedTopic) {
      Serial.print("Message: ");
      // Serial.println(messageTemp);
      ventState[2] = stoi(messageTemp);
      Serial.println(ventState[2]);
  }
  if (String(topic) == speed1Topic) {
      Serial.print("Message: ");
      // Serial.println(messageTemp);
      ventState[3] = stoi(messageTemp);
      Serial.println(ventState[3]);
  }
  if (String(topic) == speed2Topic) {
      Serial.print("Message: ");
      // Serial.println(messageTemp);
      ventState[4] = stoi(messageTemp);
      Serial.println(ventState[4]);
  }
}

void reconnect()
{
  while (!client.connected()) {
     String client_id = "esp32-client-";
     client_id += String(WiFi.macAddress());
     Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
     if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
         Serial.println("Public emqx mqtt broker connected");
          client.subscribe(ventTopic);
          client.subscribe(dirTopic);
          client.subscribe(speedTopic);
          client.subscribe(speed1Topic);
          client.subscribe(speed2Topic);
     }
     else {
         Serial.print("failed with state ");
         Serial.print(client.state());
         vTaskDelay(2000/portTICK_PERIOD_MS);
     }
  }
}
void loop() {
 
}
