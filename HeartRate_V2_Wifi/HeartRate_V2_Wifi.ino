#include "MAX30105.h"
#include "heartRate.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiNINA.h>
#include <avr/dtostrf.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#define WIRE Wire


#define DEVICE_LABEL "arduino-nano-33"
#define TOKEN "BBFF-o63O7Ap7f07GwsPU6Jm4JWDUIF2vPI"

MAX30105 particleSensor;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &WIRE);

char const *SERVER="industrial.api.ubidots.com";

const int HTTPPORT= 443;
char const *AGENT="Arduino Nano 33 IoT";
char const *HTTP_VERSION = " HTTP/1.1\r\n";
char const *VERSION ="1.0";
char const *PATH= "/api/v1.6/devices/";

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

char const * SSID_NAME = "salita_dani"; // Put here your SSID name
char const * SSID_PASS = "dani2001"; // Put here your password

char mqtt_ip[] = "169.55.61.243";
char mqtt_user[] = "BBFF-o63O7Ap7f07GwsPU6Jm4JWDUIF2vPI";

int status = WL_IDLE_STATUS;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

int ledRojo = 3;
int ledVerde = 5;
int ledAzul = 6;

float beatsPerMinute;
int beatAvg;

long blinkInterval = 0;
long prevBlink = 0;
bool power = false;

long printInterval = 5000;
long prevPrint = 0;

bool sleep = false;
char topic[6];

void connectWifi(){
    Serial.println("Connecting to wifi");
    if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Inicializando...");
  display.println("Conectando Wifi...");
  display.display();

  int attempts = 0;
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SSID_NAME);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(SSID_NAME, SSID_PASS);
    // wait 10 seconds for connection:
    delay(2000);
    attempts++;
    if(attempts > 5){
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Conixion Wifi Fallida.");
      display.println("Reinicie el");
      display.println("Dispositivo");
      display.display();
      while(true){};
    }
  }
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Inicializando...");
  display.println("Wifi Conectado");
  display.display();
  analogWrite(ledAzul,255);
  delay(2000);
  analogWrite(ledAzul,0);
}

void connectMqtt(){

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Inicializando...");
  display.println("Wifi Conectado");
  display.println("Conectando MQTT...");
  display.display();

  client.setServer(mqtt_ip,1883);
  
  
  int attempts = 0;
  
  while(!client.connected()){
    client.connect("dgs65MQTT", TOKEN,"");
    delay(2000);
    attempts++;
    
    if(attempts > 5){
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Conixion MQTT Fallida.");
      display.println("Reinicie el");
      display.println("Dispositivo");
      display.display();
      while(true){};
    }
  }

  client.setCallback(callback);
  String topicStr = "Switch";
  topicStr.toCharArray(topic, topicStr.length()+1);
  client.subscribe("/v1.6/devices/arduino-nano-33/Switch/lv");
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Inicializando...");
  display.println("Wifi Conectado");
  display.println("MQTT Conectado");
  display.display();
  analogWrite(ledAzul,255);
  delay(2000);
  analogWrite(ledAzul,0);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing...");
  
  pinMode(ledRojo,OUTPUT);
  pinMode(ledVerde,OUTPUT);
  pinMode(ledAzul,OUTPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  Serial.println("OLED begun"); 

  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Inicializando...");
  display.display(); 
  
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  connectWifi();
  connectMqtt();

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Inicializando...");
  display.display();
  
  analogWrite(ledRojo,255);
  delay(3000);
  analogWrite(ledRojo,0);
  display.clearDisplay();
  display.display();
  prevPrint = -5000;
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived from [");
  Serial.print(topic);
  Serial.print("] ");
  
  if((char)payload[0] == '1'){
    sleep = false;
  }
  else if((char)payload[0] == '0'){
    sleep = true;
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Suspendido");
    display.display();
    analogWrite(ledRojo,0);
    analogWrite(ledVerde,0);
    analogWrite(ledAzul,0);
  }
}

void blink(){

  if(!power){
    analogWrite(ledRojo,0);
    analogWrite(ledVerde,255);
    analogWrite(ledAzul,0);
    power = true;
  }
  else{
    analogWrite(ledRojo,0);
    analogWrite(ledVerde,0);
    analogWrite(ledAzul,0);
    power = false;
  }  
}

void loop()
{
  if(!sleep){
    long irValue = particleSensor.getIR();
    
    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);
  
      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable
  
        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
  
    if(beatsPerMinute < 70){
      blinkInterval = 0;
    }
    else if(beatsPerMinute > 110){
      blinkInterval = 500;
    }
    else{
      blinkInterval = 1000;
    }
    
    if(irValue > 50000 ){
      if(beatsPerMinute == 0){
        analogWrite(ledRojo,255);
        analogWrite(ledVerde,50);
        analogWrite(ledAzul,0);
      }
      else{
        if(blinkInterval == 0){
          power = false;
          blink();
          prevBlink = millis();
        }
        else if((millis() - prevBlink) > blinkInterval){
          blink();
          prevBlink = millis();
        }
      }
      
    }
    else{
      analogWrite(ledRojo,0);
      analogWrite(ledVerde,0);
      analogWrite(ledAzul,0);
    }
  
    if(irValue > 50000){
      if((millis() - prevPrint) > printInterval && !sleep){
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("Promedio BPM: ");
        display.println(beatAvg);
        display.display();
        prevPrint = millis();
        char payload[3];
        String  beatAvgStr = String(beatAvg);
        beatAvgStr.toCharArray(payload, beatAvgStr.length()+1);
        Serial.println(beatAvgStr);
        Serial.println(payload);
        client.publish("/v1.6/devices/arduino-nano-33/Average",payload);
      }
    }
    else if(!sleep){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("ERROR DEDO");
      display.display();
      prevPrint = -5000;  
    }
  
    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);
    
    if (irValue < 50000)
      Serial.print(" No finger?");
  
    Serial.println();
  }
  client.loop();
}
