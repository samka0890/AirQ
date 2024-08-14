#include <iostream>
#include <cmath> // for std::modf
#include <string> // for std::to_string
#include <sstream> // for std::stringstream
using namespace std;

//------------------------------------------------------------------------------------------
#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h> 

TFT_eSPI tft = TFT_eSPI(); // Invoke custom library
TFT_eSPI_Button key[3];

TFT_eSPI_Button vent[5];
int ventState[5] = {0, 0, 1, 0, 0};
bool keyState[3] = {1, 0, 0};
bool keyStateLast[3] = {0, 0, 0};
int onOff = 0;

// This is the file name used to store the calibration data
// You can change this to create new calibration files.
// The SPIFFS file name must start with "/".
#define CALIBRATION_FILE "/TouchCalData2"

// Set REPEAT_CAL to true instead of false to run calibration
// again, otherwise it will only be done once.
// Repeat calibration if you change the screen rotation.
#define REPEAT_CAL false

#define LABEL1_FONT &FreeSansOblique12pt7b // Key label font 1
#define LABEL2_FONT &FreeSansBold12pt7b    // Key label font 2

//------------------------------------------------------------------------------------------
#if CONFIG_FREERTOS_UNICORE
#define CORE_0 0
#else
#define CORE_1 1
#endif

#define co2_pin 25
//-----------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

Adafruit_BME680 bme; // I2C
unsigned long endTime;

//-----------------------------------------------------------------
#include<Arduino.h>
#include<SoftwareSerial.h>

#define MYPORT_TX 17
#define MYPORT_RX 16
SoftwareSerial mySerial(MYPORT_RX, MYPORT_TX);

//-----------------------------------------------------------------
int AQI_h[6]={50, 100, 200, 300, 400, 500};
int AQI_l[6]={0, 51, 101, 201, 301, 401};
int PM2_5_limit_h[6]={35, 50, 150, 250, 350, 500};
int PM2_5_limit_l[6]={0, 36, 51, 151, 251, 351};


//-----------------------------------------------------------------
#include "PMS.h"
PMS pms(mySerial);
PMS::DATA data_pms;

//-----------------------------------------------------------------
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

int16_t adc0, adc1, adc2, adc3;

//-----------------------------------------------------------------
const float VC = 5, V3=3.3;

//CJMCU
float Vco_load, Vno_load;
float Rco_load = 47000, Rno_load = 22000;
float Rco, Rno, Rco_0=1000, Rno_0=2000;
float x1, x2;

// MQ135
float VRL, RL = 1000, RS, R0=1000;
float x, NH4, CO2, CO;

//MQ131
float VRL_131, RL_131 = 4700, RS_131, R0_131=1917.22;
float x3;

//MHZ19c
unsigned long pulseWidth=0;
unsigned long pulseStartTime = 0;

//------------------------------------------------------------------------------------------
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Samka";
const char* password = "lcqq1655";

const char *mqtt_broker = "192.168.94.164";
const char *mqtt_username = "samka";
const char *mqtt_password = "12345678";
const int mqtt_port = 1883;

const char *tempTopic = "sensorUnit1/temp";
const char *humTopic = "sensorUnit1/hum";
const char *coTopic = "sensorUnit1/co";
const char *vocTopic = "sensorUnit1/voc";

const char *tempTopic1 = "sensorUnit2/temp";
const char *humTopic1 = "sensorUnit2/hum";
const char *coTopic1 = "sensorUnit2/co";
const char *vocTopic1 = "sensorUnit2/voc";

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

void display_task(void *pvParameters);
void MQTT_task(void *pvParameters);
void BME_task( void *pvParameters );
void PMS5003_task(void *pvParameters);
void MQ135_task(void *pvParameters);
void MHZ19_task(void *pvParameters);
void CJMCU_task( void *pvParameters );
void MQ131_task( void *pvParameters );

//-----------------------------------------------------------------

struct Sensors{
  float temp=25.3;
  float hum=28.1;
  float airPressure=73;
  float CO2=225;
  float CO = 7;
  float SO2 = 65;
  float NO2 = 58;
  float O3 =23;

  int PM1 = 18;
  int PM2_5 = 23;
  int PM10 = 35; 
  int aqi=25;
};

typedef struct Sensors Sensors;
Sensors a;

struct SensorUnit
{
  float temp;
  float hum;
  float CO;
  float VOC;

  const char* temp_c;
};

typedef struct SensorUnit SensorUnit;
SensorUnit s1, s2;

//------------------------------------------------------------------------------------------

void setup() 
{

  Serial.begin(115200);
  mySerial.begin(9600);

  while (!ads.begin()){
    Serial.println("Failed to initialize ADS.");
    vTaskDelay(100/ portTICK_PERIOD_MS);
  }
  ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV

  pinMode(co2_pin, INPUT);

  setup_wifi();
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);


  tft.init();
  tft.setRotation(3);
  touch_calibrate();
  tft.fillScreen(TFT_BLACK);
                          // x, y, w, h, outline, fill, text
  key[0].initButton(&tft, 80, 300, 150, 40, TFT_BLACK, TFT_BLACK, TFT_WHITE, " ", 1);
  key[1].initButton(&tft, 240, 300, 150, 40, TFT_BLACK, TFT_BLACK, TFT_WHITE, " ", 1);
  key[2].initButton(&tft, 400, 300, 150, 40, TFT_BLACK, TFT_BLACK, TFT_WHITE, " ", 1);
  key[0].drawButton();
  key[1].drawButton();
  key[2].drawButton();
  

  // Home logo 
  tft.fillRect(65, 298, 30, 20, TFT_WHITE);
  tft.fillTriangle(80, 285, 100, 300, 60, 300, TFT_WHITE);

  // Fan logo 
  tft.drawCircle(240, 300, 15, TFT_WHITE);
  tft.fillCircle(240, 300, 3, TFT_WHITE);
  tft.fillTriangle(240, 300, 235, 290, 245, 290, TFT_WHITE);
  tft.fillTriangle(240, 300, 250, 302, 244, 310, TFT_WHITE);
  tft.fillTriangle(240, 300, 230, 302, 236, 310, TFT_WHITE);
  tft.fillCircle(240, 300, 2, TFT_BLACK);

  tft.drawCircle(225, 285, 2, TFT_WHITE);
  tft.drawCircle(255, 285, 2, TFT_WHITE);
  tft.drawCircle(225, 315, 2, TFT_WHITE);
  tft.drawCircle(255, 315, 2, TFT_WHITE);
  
  tft.drawRoundRect(220, 280, 40, 40, 5, TFT_WHITE);
  // Sensor Units
  tft.drawWideLine(385, 285, 415, 285, 3, TFT_WHITE, TFT_BLACK);
  tft.drawWideLine(385, 295, 415, 295, 3, TFT_WHITE, TFT_BLACK);
  tft.drawWideLine(385, 305, 415, 305, 3, TFT_WHITE, TFT_BLACK);

  vent[0].initButton(&tft, 110, 120, 150, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "OFF", 1);
  vent[1].initButton(&tft, 110, 200, 150, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "RIGHT", 1);
  vent[2].initButton(&tft, 350, 105, 150, 40, TFT_WHITE, TFT_GREEN, TFT_WHITE, "50%", 1);
  vent[3].initButton(&tft, 350, 165, 150, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "75%", 1);
  vent[4].initButton(&tft, 350, 225, 150, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "100%", 1);

  xTaskCreatePinnedToCore(display_task,  "Display task 0",  3000,  NULL,  3,  NULL, 1);//2000
  xTaskCreate(MQTT_task,  "MQTT task",  3500,  NULL,  3,  NULL); //2500
  xTaskCreate(BME_task,  "BME task",  1600,  NULL, 3,  NULL);
  xTaskCreate(PMS5003_task, "PMS5003 task", 2000,  NULL,  2,  NULL);//1000
  xTaskCreate(MQ135_task,  "MQ135 task",  2000,  NULL,  3,  NULL);
  xTaskCreate(MHZ19_task, "MHZ19 task" , 2450,  NULL,  2,  NULL);//1450
  xTaskCreate(CJMCU_task, "CJMCU task", 2300, NULL, 2, NULL);//1300
  xTaskCreate(MQ131_task,  "MQ131 task",  1500,  NULL,  3,  NULL);
}

void display_task(void *pvParameters)
{
  uint16_t t_x = 0, t_y = 0; // To store the touch coordinates
  bool pressed;
  for(;;)
  {
    pressed = tft.getTouch(&t_x, &t_y);
    for (uint8_t b = 0; b < 3; b++) {
      if (pressed && key[b].contains(t_x, t_y)) {
        key[b].press(true);  // tell the button it is pressed
      } 
      else {
        key[b].press(false);  // tell the button it is NOT pressed
      }
    }
    for (uint8_t b = 0; b < 3; b++) {
      if (key[b].justReleased())
        key[b].drawButton(true);  // draw invert
    }

    if (key[0].justReleased())
    {
      key[0].drawButton();
      // Home logo 
      tft.fillRect(65, 298, 30, 20, TFT_WHITE);
      tft.fillTriangle(80, 285, 100, 300, 60, 300, TFT_WHITE);

      // Fan logo 
      tft.drawCircle(240, 300, 15, TFT_WHITE);
      tft.fillCircle(240, 300, 3, TFT_WHITE);
      tft.fillTriangle(240, 300, 235, 290, 245, 290, TFT_WHITE);
      tft.fillTriangle(240, 300, 250, 302, 244, 310, TFT_WHITE);
      tft.fillTriangle(240, 300, 230, 302, 236, 310, TFT_WHITE);
      tft.fillCircle(240, 300, 2, TFT_BLACK);

      tft.drawCircle(225, 285, 2, TFT_WHITE);
      tft.drawCircle(255, 285, 2, TFT_WHITE);
      tft.drawCircle(225, 315, 2, TFT_WHITE);
      tft.drawCircle(255, 315, 2, TFT_WHITE);
      
      tft.drawRoundRect(220, 280, 40, 40, 5, TFT_WHITE);
      // Sensor Units
      tft.drawWideLine(385, 285, 415, 285, 3, TFT_WHITE, TFT_BLACK);
      tft.drawWideLine(385, 295, 415, 295, 3, TFT_WHITE, TFT_BLACK);
      tft.drawWideLine(385, 305, 415, 305, 3, TFT_WHITE, TFT_BLACK); 
      keyStateLast[0]= keyState[0];
      keyStateLast[1]= keyState[1];
      keyStateLast[2]= keyState[2];
      keyState[0]=1;
      keyState[1]=0;
      keyState[2]=0;
      for(int i=0; i<3;i++){
        Serial.println(keyState[i]);
      }
    }
    if (key[1].justReleased())
    {
      key[1].drawButton();
      // Home logo 
      tft.fillRect(65, 298, 30, 20, TFT_WHITE);
      tft.fillTriangle(80, 285, 100, 300, 60, 300, TFT_WHITE);

      // Fan logo 
      tft.drawCircle(240, 300, 15, TFT_WHITE);
      tft.fillCircle(240, 300, 3, TFT_WHITE);
      tft.fillTriangle(240, 300, 235, 290, 245, 290, TFT_WHITE);
      tft.fillTriangle(240, 300, 250, 302, 244, 310, TFT_WHITE);
      tft.fillTriangle(240, 300, 230, 302, 236, 310, TFT_WHITE);
      tft.fillCircle(240, 300, 2, TFT_BLACK);

      tft.drawCircle(225, 285, 2, TFT_WHITE);
      tft.drawCircle(255, 285, 2, TFT_WHITE);
      tft.drawCircle(225, 315, 2, TFT_WHITE);
      tft.drawCircle(255, 315, 2, TFT_WHITE);
      
      tft.drawRoundRect(220, 280, 40, 40, 5, TFT_WHITE);
      // Sensor Units
      tft.drawWideLine(385, 285, 415, 285, 3, TFT_WHITE, TFT_BLACK);
      tft.drawWideLine(385, 295, 415, 295, 3, TFT_WHITE, TFT_BLACK);
      tft.drawWideLine(385, 305, 415, 305, 3, TFT_WHITE, TFT_BLACK); 
      keyStateLast[0]= keyState[0];
      keyStateLast[1]= keyState[1];
      keyStateLast[2]= keyState[2];
      keyState[0]=0;
      keyState[1]=1;
      keyState[2]=0;
      for(int i=0; i<3;i++){
        Serial.println(keyState[i]);
      }

    }
    if (key[2].justReleased())
    {
      key[2].drawButton();
      // Home logo 
      tft.fillRect(65, 298, 30, 20, TFT_WHITE);
      tft.fillTriangle(80, 285, 100, 300, 60, 300, TFT_WHITE);

      // Fan logo 
      tft.drawCircle(240, 300, 15, TFT_WHITE);
      tft.fillCircle(240, 300, 3, TFT_WHITE);
      tft.fillTriangle(240, 300, 235, 290, 245, 290, TFT_WHITE);
      tft.fillTriangle(240, 300, 250, 302, 244, 310, TFT_WHITE);
      tft.fillTriangle(240, 300, 230, 302, 236, 310, TFT_WHITE);
      tft.fillCircle(240, 300, 2, TFT_BLACK);

      tft.drawCircle(225, 285, 2, TFT_WHITE);
      tft.drawCircle(255, 285, 2, TFT_WHITE);
      tft.drawCircle(225, 315, 2, TFT_WHITE);
      tft.drawCircle(255, 315, 2, TFT_WHITE);
      
      tft.drawRoundRect(220, 280, 40, 40, 5, TFT_WHITE);
      // Sensor Units
      tft.drawWideLine(385, 285, 415, 285, 3, TFT_WHITE, TFT_BLACK);
      tft.drawWideLine(385, 295, 415, 295, 3, TFT_WHITE, TFT_BLACK);
      tft.drawWideLine(385, 305, 415, 305, 3, TFT_WHITE, TFT_BLACK);
      keyStateLast[0]= keyState[0];
      keyStateLast[1]= keyState[1];
      keyStateLast[2]= keyState[2];
      keyState[0]=0;
      keyState[1]=0;
      keyState[2]=1;
      for(int i=0; i<3;i++){
        Serial.println(keyState[i]);
      }
    }
    if(keyState[0])
    {
      if(keyState[0] != keyStateLast[0])
      {
        keyStateLast[0] = keyState[0];
        tft.fillRect(0, 0, 480, 280, TFT_BLACK);
          
        //-------------------CO CO2 SO2 NO2 --------------------------------
        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(3);
        tft.drawCentreString("CO", 60, 10, 1);
        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(1);
        tft.drawCentreString("ppm", 60, 35, 1);
        // tft.setTextColor(TFT_WHITE);
        // tft.setTextSize(2);
        // tft.setCursor(39, 50, 1);
        // tft.println(a.CO, TFT_BLACK);

        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(3);
        tft.drawCentreString("CO", 180, 10, 1);
        tft.setTextSize(2);
        tft.drawCentreString("2", 205, 20, 1);
        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(1);
        tft.drawCentreString("ppm", 180, 35, 1);
        tft.setTextColor(TFT_WHITE);

        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(3);
        tft.drawCentreString("SO", 300, 10, 1);
        tft.setTextSize(2);
        tft.drawCentreString("2", 325, 20, 1);
        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(1);
        tft.drawCentreString("ppm", 300, 35, 1);
        tft.setTextColor(TFT_WHITE);

        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(3);
        tft.drawCentreString("NO", 420, 10, 1);
        tft.setTextSize(2);
        tft.drawCentreString("2", 445, 20, 1);
        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(1);
        tft.drawCentreString("ppm", 420, 35, 1);
        tft.setTextColor(TFT_WHITE);
        // tft.drawLine(0, 80, 480, 80, TFT_WHITE);

        //-------------------PM1 PM2.5 PM10 O3--------------------------------
        // tft.drawLine(0, 200, 480, 200, TFT_WHITE);

        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(3);
        tft.drawCentreString("PM1", 60, 210, 1);
        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(1);
        tft.drawCentreString("ug/m3", 60, 235, 1);
        // tft.setTextColor(TFT_WHITE);
        // tft.setTextSize(2);
        // tft.setCursor(39, 250, 1);
        // tft.println(a.CO, TFT_BLACK);

        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(3);
        tft.drawCentreString("PM2.5", 180, 210, 1);
        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(1);
        tft.drawCentreString("ug/m3", 180, 235, 1);
        tft.setTextColor(TFT_WHITE);


        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(3);
        tft.drawCentreString("PM10", 300, 210, 1);
        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(1);
        tft.drawCentreString("ug/m3", 300, 235, 1);
        tft.setTextColor(TFT_WHITE);

        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(3);
        tft.drawCentreString("O", 415, 210, 1);
        tft.setTextSize(2);
        tft.drawCentreString("3", 430, 220, 1);
        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(1);
        tft.drawCentreString("ppm", 420, 235, 1);
        tft.setTextColor(TFT_WHITE);

        // tft.drawLine(0, 280, 480, 280, TFT_WHITE);

        //----------------------------------------------------------

        // tft.fillCircle(240, 140, 60, TFT_GREEN);
        // tft.fillCircle(240, 140, 50, TFT_BLACK);
        // tft.fillEllipse(240, 140, 120, 60,TFT_DARKGREEN);
        // tft.fillEllipse(240, 140, 110, 50,TFT_BLACK);

        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(3);
        tft.drawCentreString("AQI", 240, 100, 1);

        // Temperature logo
        tft.drawRoundRect(70, 90, 20, 50, 10, TFT_SILVER),
        tft.drawCircle(80, 140, 12, TFT_SILVER);
        tft.fillCircle(80, 132, 8, TFT_BLACK);

        tft.fillRect(75, 120, 10, 15, TFT_RED),
        tft.fillCircle(80, 140, 7, TFT_RED);

        tft.drawLine(70, 100, 75, 100, TFT_SILVER);
        tft.drawLine(70, 105, 80, 105, TFT_SILVER);
        tft.drawLine(70, 110, 75, 110, TFT_SILVER);
        tft.drawLine(70, 115, 80, 115, TFT_SILVER);

        tft.setTextSize(3);
        tft.drawCentreString("C", 50, 120, 1);
        tft.setTextSize(1);
        tft.drawCentreString("o",60, 110, 2);

        // Humidity logo

        tft.drawCircle(420, 127, 21, TFT_SILVER);
        tft.drawTriangle(420, 94, 440, 120, 400, 120, TFT_SILVER);
        tft.fillCircle(420, 127, 20, TFT_BLUE);
        tft.fillTriangle(420, 95, 439, 120, 401, 120, TFT_BLUE);

        tft.setTextSize(3);
        tft.setTextColor(TFT_SILVER);
        tft.drawCentreString("%", 423, 120, 1);
        
        // Home logo 
        tft.fillRect(65, 298, 30, 20, TFT_WHITE);
        tft.fillTriangle(80, 285, 100, 300, 60, 300, TFT_WHITE);

        // Fan logo 
        tft.drawCircle(240, 300, 15, TFT_WHITE);
        tft.fillCircle(240, 300, 3, TFT_WHITE);
        tft.fillTriangle(240, 300, 235, 290, 245, 290, TFT_WHITE);
        tft.fillTriangle(240, 300, 250, 302, 244, 310, TFT_WHITE);
        tft.fillTriangle(240, 300, 230, 302, 236, 310, TFT_WHITE);
        tft.fillCircle(240, 300, 2, TFT_BLACK);

        tft.drawCircle(225, 285, 2, TFT_WHITE);
        tft.drawCircle(255, 285, 2, TFT_WHITE);
        tft.drawCircle(225, 315, 2, TFT_WHITE);
        tft.drawCircle(255, 315, 2, TFT_WHITE);
        
        tft.drawRoundRect(220, 280, 40, 40, 5, TFT_WHITE);
        // Sensor Units
        tft.drawWideLine(385, 285, 415, 285, 3, TFT_WHITE, TFT_BLACK);
        tft.drawWideLine(385, 295, 415, 295, 3, TFT_WHITE, TFT_BLACK);
        tft.drawWideLine(385, 305, 415, 305, 3, TFT_WHITE, TFT_BLACK);
      }
      int i;
      for(i=0; i<6; i++)
      {
        if(a.PM2_5 <=PM2_5_limit_h[i])
        {
          a.aqi = ((AQI_h[i] - AQI_l[i])/(PM2_5_limit_h[i] - PM2_5_limit_l[i]))*(a.PM2_5-PM2_5_limit_l[i])+PM2_5_limit_h[i]-13;
          break;
        }
      }
      if(i==6 || a.aqi>500){
        a.aqi = 500;
      }
      
      if(a.aqi<50){
        tft.setTextColor(TFT_DARKGREEN);
        tft.setTextSize(3);
        tft.drawCentreString("AQI", 240, 100, 1);
        tft.drawWideLine(178, 180, 300, 180, 10, TFT_DARKGREEN, TFT_BLACK);
        ventState[0] = 0;
      }
      else if(a.aqi<100){
        tft.setTextColor(TFT_YELLOW);
        tft.setTextSize(3);
        tft.drawCentreString("AQI", 240, 100, 1);
        tft.drawWideLine(178, 180, 300, 180, 10, TFT_YELLOW, TFT_BLACK);
        onOff=0;
      }
      else if(a.aqi<200){
        tft.setTextColor(TFT_ORANGE);
        tft.setTextSize(3);
        tft.drawCentreString("AQI", 240, 100, 1);
        tft.drawWideLine(178, 180, 300, 180, 10, TFT_ORANGE, TFT_BLACK);
        onOff=0;
      }
      else if(a.aqi<300){
        tft.setTextColor(TFT_RED);
        tft.setTextSize(3);
        tft.drawCentreString("AQI", 240, 100, 1);
        tft.drawWideLine(178, 180, 300, 180, 10, TFT_RED, TFT_BLACK);
        onOff=0;
       
      }
      else if(a.aqi<400){
        tft.setTextColor(TFT_PINK);
        tft.setTextSize(3);
        tft.drawCentreString("AQI", 240, 100, 1);
        tft.drawWideLine(178, 180, 300, 180, 10, TFT_PINK, TFT_BLACK);
        onOff=0;
      }
      else{
        tft.setTextColor(TFT_BROWN);
        tft.setTextSize(3);
        tft.drawCentreString("AQI", 240, 100, 1);
        tft.drawWideLine(178, 180, 300, 180, 10, TFT_BROWN, TFT_BLACK);
        onOff=1;
      
      }
      
      tft.fillRect(180, 133, 100, 35, TFT_BLACK );
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.setTextSize(4);
      tft.setCursor(240-countI(a.aqi)*11, 135, 1);
      tft.println(a.aqi, TFT_BLACK);

      tft.setTextSize(2);
      tft.fillRect(0, 48, 480, 20, TFT_BLACK );
      tft.setCursor(60-countI(a.CO)*7, 50, 1);
      tft.println(a.CO, TFT_BLACK);//

      tft.setCursor(187-countI(a.CO2)*7, 50, 1);
      tft.println(a.CO2, TFT_BLACK);//

      tft.setCursor(307-countI(a.SO2)*7, 50, 1);
      tft.println(a.SO2, TFT_BLACK);//

      tft.setCursor(427-countI(a.NO2)*7, 50, 1);
      tft.println(a.NO2, TFT_BLACK);//

      tft.fillRect(0, 248, 480, 20, TFT_BLACK );
      tft.setCursor(60-(countI((double)a.PM1))*7, 250, 1);
      tft.println(a.PM1, TFT_BLACK);

      tft.setCursor(180-(countI((double)a.PM2_5))*7, 250, 1);
      tft.println(a.PM2_5, TFT_BLACK);

      tft.setCursor(300-(countI((double)a.PM10))*7, 250, 1);
      tft.println(a.PM10, TFT_BLACK);

      tft.setCursor(425-countI(a.O3)*7, 250, 1);
      tft.println(a.O3, TFT_BLACK);//

      tft.fillRect(0, 163, 108, 17, TFT_BLACK);
      tft.setCursor(65-(countI(a.temp)+1)*7, 165, 1);
      tft.println(a.temp);
      
      // a.temp-=50;
      tft.fillRect(380, 163, 108, 17, TFT_BLACK);
      tft.setCursor(420-(countI(a.hum)+1)*7, 165, 1);
      tft.println(a.hum);
    }
    if(keyState[1])
    {
      if(keyState[1]!=keyStateLast[1]){

        keyStateLast[1]=keyState[1];
        tft.fillRect(0, 0, 480, 280, TFT_BLACK);
        tft.drawRoundRect(5, 50, 230, 220, 10, TFT_SILVER);
        tft.drawRoundRect(245, 50, 230, 220, 10, TFT_SILVER);
        tft.setTextColor(TFT_SILVER, TFT_BLACK);
        tft.setTextSize(3);
        tft.drawCentreString("Ventilation Controls", 240, 5, 1);
        tft.setTextSize(2);
        tft.drawCentreString("Direction Controls", 120, 45, 1);
        tft.drawCentreString("  Speed Controls  ", 360, 45, 1);

        for(int i=0; i<5; i++){
          vent[i].drawButton();
        }
      }

      // Pressed will be set true is there is a valid touch on the screen
      pressed = tft.getTouch(&t_x, &t_y);

      for(int i=0; i<5; i++){
        if (pressed && vent[i].contains(t_x, t_y)) {
          vent[i].press(true);  // tell the button it is pressed
          if(i<2)
            ventState[i] ^= 1;
        } 
        else {
          vent[i].press(false);  // tell the button it is NOT pressed
        }
      }
      
      for(int i=0; i<5; i++){
        if (vent[i].justPressed()) {
            vent[i].drawButton(true);
        }
      }
      if (vent[0].justReleased())
      {
        if(ventState[0]){
          vent[0].initButton(&tft, 110,
                          120, // x, y, w, h, outline, fill, text
                          150, 40, TFT_WHITE, TFT_GREEN, TFT_WHITE, "ON", 1);
          vent[0].drawButton();
        }
        
        else{
          vent[0].initButton(&tft, 110,
                          120, // x, y, w, h, outline, fill, text
                          150, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "OFF", 1);
          vent[0].drawButton();
        }
      }
      if (vent[1].justReleased())
      {
          if(ventState[1]){
            vent[1].initButton(&tft, 110,
                          200, // x, y, w, h, outline, fill, text
                          150, 40, TFT_WHITE, TFT_GREEN, TFT_WHITE, "LEFT", 1);
            vent[1].drawButton();
          }
          else{
            vent[1].initButton(&tft, 110,
                          200, // x, y, w, h, outline, fill, text
                          150, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "RIGHT", 1);
            vent[1].drawButton();
          }
      }
      if (vent[2].justReleased())
      {
          ventState[2]=1;
          ventState[3]=0;
          ventState[4]=0;
          vent[2].initButton(&tft, 350,
                          105, // x, y, w, h, outline, fill, text
                          150, 40, TFT_WHITE, TFT_GREEN, TFT_WHITE, "50%", 1);
          vent[2].drawButton();
          vent[3].initButton(&tft, 350,
                        165, // x, y, w, h, outline, fill, text
                        150, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "75%", 1);
          vent[3].drawButton();
          vent[4].initButton(&tft, 350,
                        225, // x, y, w, h, outline, fill, text
                        150, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "100%", 1);
          vent[4].drawButton();
      }
      if (vent[3].justReleased())
      {
          ventState[2]=0;
          ventState[3]=1;
          ventState[4]=0;
          vent[2].initButton(&tft, 350,
                          105, // x, y, w, h, outline, fill, text
                          150, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "50%", 1);
          vent[2].drawButton();
          vent[3].initButton(&tft, 350,
                        165, // x, y, w, h, outline, fill, text
                        150, 40, TFT_WHITE, TFT_GREEN, TFT_WHITE, "75%", 1);
          vent[3].drawButton();
          vent[4].initButton(&tft, 350,
                        225, // x, y, w, h, outline, fill, text
                        150, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "100%", 1);
          vent[4].drawButton();
      }
      if (vent[4].justReleased())
      {
        ventState[2]=0;
        ventState[3]=0;
        ventState[4]=1;
        vent[2].initButton(&tft, 350,
                        105, // x, y, w, h, outline, fill, text
                        150, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "50%", 1);
        vent[2].drawButton();
        vent[3].initButton(&tft, 350,
                      165, // x, y, w, h, outline, fill, text
                      150, 40, TFT_WHITE, TFT_RED, TFT_WHITE, "75%", 1);
        vent[3].drawButton();
        vent[4].initButton(&tft, 350,
                      225, // x, y, w, h, outline, fill, text
                      150, 40, TFT_WHITE, TFT_GREEN, TFT_WHITE, "100%", 1);
        vent[4].drawButton();
      }
    } 
    if(keyState[2])
    {
      if(keyState[2]!=keyStateLast[2]){

        keyStateLast[2]=keyState[2];
        tft.fillRect(0, 0, 480, 280, TFT_BLACK);
    
        tft.setTextColor(TFT_SILVER);
        tft.setTextSize(2);
        tft.drawCentreString("Sensor", 200, 10, 1);
        tft.drawCentreString("Unit 1", 200, 30, 1);
        tft.drawCentreString("Sensor", 360, 10, 1);
        tft.drawCentreString("Unit 2", 360, 30, 1);

        tft.drawLine(20, 50, 460, 50, TFT_SILVER);
        tft.drawCentreString("Temp (C)", 60, 60, 1);
        tft.drawCentreString("Hum (%)", 60, 120, 1);
        tft.drawCentreString("CO (ppm)", 60, 180, 1);
        tft.drawCentreString("VOCs (ppm)", 65, 240, 1);
        tft.drawLine(20, 270, 460, 270, TFT_SILVER);

      }
      tft.setTextSize(2);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.fillRect(120, 55, 330, 210, TFT_BLACK);

      tft.setCursor(200-(countI(s1.temp))*7, 62, 1);
      tft.println(s1.temp, TFT_BLACK);
      
      tft.setCursor(350-(countI(s2.temp))*7, 62, 1);
      tft.println(s2.temp, TFT_BLACK);


      tft.setCursor(200-(countI(s1.hum))*7, 122, 1);
      tft.println(s1.hum, TFT_BLACK);

      tft.setCursor(350-(countI(s2.hum))*7, 122, 1);
      tft.println(s2.hum, TFT_BLACK);

      tft.setCursor(200-(countI(s1.CO))*7, 182, 1);
      tft.println(s1.CO, TFT_BLACK);

      tft.setCursor(350-(countI(s2.CO))*7, 182, 1);
      tft.println(s2.CO, TFT_BLACK);

      tft.setCursor(200-(countI(s1.VOC))*7, 242, 1);
      tft.println(s1.VOC, TFT_BLACK);

      tft.setCursor(350-(countI(s2.VOC))*7, 242, 1);
      tft.println(s2.VOC, TFT_BLACK);
    }
    Serial.println("Display Task Done");
    // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.print("display_task stack high watermark: ");
    // Serial.println(uxHighWaterMark);
    vTaskDelay(50/ portTICK_PERIOD_MS);                   
  }

}
void MQTT_task(void *pvParameters)
{
  string str;
  for(;;){
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    str = to_string(ventState[0]);
    client.publish(ventTopic, str.c_str());
    str = to_string(ventState[1]);
    client.publish(dirTopic, str.c_str());
    str = to_string(ventState[2]);
    client.publish(speedTopic, str.c_str());
    str = to_string(ventState[3]);
    client.publish(speed1Topic, str.c_str());
    str = to_string(ventState[4]);
    client.publish(speed2Topic, str.c_str());


  
    
    // Serial.print(" String Hum: ");
    // Serial.println(str.c_str());

    // str = to_string(s1.CO);
    // client.publish(coTopic, str.c_str());

    // Serial.print(" String CO: ");
    // Serial.println(str.c_str());

    // str = to_string(s1.VOC);
    // client.publish(vocTopic, str.c_str());

    // Serial.print(" String VOC: ");
    // Serial.println(str.c_str());
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    Serial.print("MQTT task stack high watermark: ");
    Serial.println(uxHighWaterMark);
    vTaskDelay(2000/portTICK_PERIOD_MS);
  }
}
void BME_task( void *pvParameters )
{
  while (!bme.begin()){
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  for(;;){
    endTime = bme.beginReading();
    if (endTime == 0) {
      Serial.println(F("Failed to begin reading :("));
      return;
    }
    // Serial.print(F("Reading started at "));
    // Serial.print(millis());
    // Serial.print(F(" and will finish at "));
    // Serial.println(endTime);

    // Serial.println(F("You can do other work during BME680 measurement."));
    vTaskDelay(50 / portTICK_PERIOD_MS); // This represents parallel work.
    // There's no need to delay() until millis() >= endTime: bme.endReading()
    // takes care of that. It's okay for parallel work to take longer than
    // BME680's measurement time.

    // Obtain measurement results from BME680. Note that this operation isn't
    // instantaneous even if milli() >= endTime due to I2C/SPI latency.
    if (!bme.endReading()) {
      Serial.println(F("Failed to complete reading :("));
      return;
    }
    // Serial.print(F("Reading completed at "));
    // Serial.println(millis());

    // Serial.print(F("Temperature = "));
    a.temp = bme.temperature;
    // Serial.print(a.temp);
    // Serial.println(F(" *C"));

    // Serial.print(F("Pressure = "));
    // Serial.print(bme.pressure / 100.0);
    // Serial.println(F(" hPa"));

    // Serial.print(F("Humidity = "));
    a.hum = bme.humidity;
    // Serial.print(a.hum);
    // Serial.println(F(" %"));
    // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.print("BME_task stack high watermark: ");
    // Serial.println(uxHighWaterMark);
    vTaskDelay(1500 / portTICK_PERIOD_MS);
  }

} 
void PMS5003_task(void *pvParameters)
{
  for(;;)
  {
    if(pms.read(data_pms))
    {
      a.PM1 = data_pms.PM_AE_UG_1_0+15;
      a.PM2_5 = data_pms.PM_AE_UG_2_5+19;
      a.PM10 = data_pms.PM_AE_UG_10_0+20;

      // // Serial.println("Air Quality Monitor");

      // Serial.println("PM1.0 :" + a.PM1 + "(ug/m3)");

      // Serial.println("PM2.5 :" + a.PM2_5 + "(ug/m3)");

      // Serial.println("PM10  :" + a.PM10 + "(ug/m3)");
      UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
      Serial.print("PMS task stack high watermark: ");
      Serial.println(uxHighWaterMark);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
    // vTaskDelay(10 / portTICK_PERIOD_MS);
  }

}
void MQ135_task(void *pvParameters)
{
  for(;;){
    adc0 = ads.readADC_SingleEnded(0);
    VRL = ads.computeVolts(adc0);

    // Serial.print("VRL = ");
    // Serial.println(VRL);

    RS = (VC - VRL)*RL / VRL;
    // Serial.print("RS = ");
    // Serial.println(RS);

    x = RS/R0;
    a.SO2 = pow((x/6.362),(-1/0.401));
    a.SO2 = a.SO2*1001;
    // CO2 = pow((x/5.1435),(-1/0.348));

    // CO = pow((x/4.9589),(-1/0.25));

    Serial.print("SO2 = ");
    Serial.println(a.SO2);

    // Serial.print("CO2 = ");
    // Serial.println(CO2);

    // Serial.print("CO = ");
    // Serial.println(CO);

    // Serial.println();

    // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.print("MQ135_task stack high watermark: ");
    // Serial.println(uxHighWaterMark);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void MHZ19_task(void *pvParameters)
{
  for(;;)
  {
    if (digitalRead(co2_pin) == HIGH) { 
      pulseStartTime = millis(); 
      while (digitalRead(co2_pin) == HIGH) {} 

      pulseWidth = millis() - pulseStartTime;
      Serial.printf("\n");
      Serial.print("Pulse width: ");
      Serial.print(pulseWidth);
      Serial.println(" milliseconds");

      
      if(pulseWidth < 10){
        a.CO2 = (pulseWidth+100)*2;
      }
      else{
        a.CO2 = (pulseWidth-2)*2;
      }
      Serial.print("CO2: ");
      Serial.print(a.CO2);
      Serial.println(" PPM");
      vTaskDelay(1100 / portTICK_PERIOD_MS);
    }
  }
}
void CJMCU_task( void *pvParameters )
{
  for(;;){
    adc2 = ads.readADC_SingleEnded(2);
    Vco_load = ads.computeVolts(adc2);

    adc1 = ads.readADC_SingleEnded(1);
    Vno_load = ads.computeVolts(adc1);

    // Serial.print("Vco_load = ");
    // Serial.println(Vco_load);

    // Serial.print("Vno_load = ");
    // Serial.println(Vno_load);
    Rco = (VC - Vco_load)*Rco_load / Vco_load;
    Rno = (VC - Vno_load)*Rno_load / Vno_load;

    // Serial.print("Rco = ");
    // Serial.println(Rco);
    
    // Serial.print("Rno = ");
    // Serial.println(Rno);

    x1 = Rco/Rco_0;
    x2 = Rno/Rno_0;
    a.CO = pow((x1/3.512),(-1/0.841));
    a.CO *= 1000;
    a.NO2 = pow((x2/6.428),(1/1.0035));
    a.NO2 *= 100;

    Serial.println("CJMCU task just executed");
    Serial.print("a.CO = ");
    Serial.println(a.CO);

    Serial.print("NO2 = ");
    Serial.println(a.NO2);

    // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.print("CJMCU_task stack high watermark: ");
    // Serial.println(uxHighWaterMark);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
void MQ131_task( void *pvParameters )
{
  for(;;)
  {
    adc0 = ads.readADC_SingleEnded(0);
    VRL_131 = ads.computeVolts(adc0);

    // Serial.print("VRL_131 = ");
    // Serial.println(VRL_131);

    RS_131 = (VC - VRL_131)*RL_131/ VRL_131;
    // Serial.print("RS = ");
    // Serial.println(RS_131);

    x3 = RS_131/R0_131;
    a.O3 = (pow((x3/0.4032),(1/0.4251)))/1000;

    Serial.print("O3 = ");
    Serial.println(a.O3);

    // UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    // Serial.print("MQ131 task stack high watermark: ");
    // Serial.println(uxHighWaterMark);

    vTaskDelay(1500/ portTICK_PERIOD_MS);
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
  // for(int i=0; i<)
  if (String(topic) == tempTopic) {
      Serial.print("Message: ");
      // Serial.println(messageTemp);
      s1.temp = stof(messageTemp);
      Serial.println(s1.temp);
  }
  if (String(topic) == humTopic) {
      Serial.print("Message: ");
      // Serial.println(messageTemp);
      s1.hum = stof(messageTemp);
      Serial.println(s1.hum);
  }
  if (String(topic) == coTopic) {
      Serial.print("Message: ");
      // Serial.println(messageTemp);
      s1.CO = stof(messageTemp);
      Serial.println(s1.CO);
  }
  if (String(topic) == vocTopic) {
      Serial.print("Message: ");
      // Serial.println(messageTemp);
      s1.VOC = stof(messageTemp);
      Serial.println(s1.VOC);
  }
  
  if (String(topic) == tempTopic1) {
      Serial.print("Message: ");
      // Serial.println(messageTemp);
      s2.temp = stof(messageTemp);
      Serial.println(s2.temp);
  }
  if (String(topic) == humTopic1) {
      Serial.print("Message: ");
      // Serial.println(messageTemp);
      s2.hum = stof(messageTemp);
      Serial.println(s2.hum);
  }
  if (String(topic) == coTopic1) {
      Serial.print("Message: ");
      // Serial.println(messageTemp);
      s2.CO = stof(messageTemp);
      Serial.println(s2.CO);
  }
  if (String(topic) == vocTopic1) {
      Serial.print("Message: ");
      // Serial.println(messageTemp);
      s2.VOC = stof(messageTemp);
      Serial.println(s2.VOC);
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
          client.subscribe(tempTopic);
          client.subscribe(humTopic);
          client.subscribe(coTopic);
          client.subscribe(vocTopic);
          client.subscribe(tempTopic1);
          client.subscribe(humTopic1);
          client.subscribe(coTopic1);
          client.subscribe(vocTopic1);
     }
     else {
         Serial.print("failed with state ");
         Serial.print(client.state());
         vTaskDelay(2000/portTICK_PERIOD_MS);
     }
  }
}

int countI(double num)
{
    if (num == 0.0) return 1;  // Special case for 0

    int integerPart = static_cast<int>(std::abs(num));  // Get the integer part
    int count = 0;
    while (integerPart != 0) {
        integerPart /= 10;
        count++;
    }
    return count;
}

int countF(double num)
{
    double integerPart, fractionalPart;
    fractionalPart = std::modf(num, &integerPart); // Separate the integer and fractional parts

    if (fractionalPart == 0.0) return 0;  // No fractional part

    // Convert fractional part to string and count the digits
    stringstream ss;
    ss << fractionalPart;
    string fractionalStr = ss.str();

    // Find the position of the decimal point
    size_t pos = fractionalStr.find('.');
    if (pos == string::npos) return 0; // Should not happen

    // Count digits after the decimal point
    return fractionalStr.length() - pos - 1;
}

void loop() {

}


void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("formatting file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {
    // calibration data valid
    tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    if (REPEAT_CAL) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

