#include "MAX30105.h"
#include "heartRate.h"
#include <SPI.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <avr/dtostrf.h>

#define WIRE Wire
#define BLE_UUID_TEST_SERVICE            "9A48ECBA-2E92-082F-C079-9E75AAE428B1"
#define BLE_UUID_HEARTRATE               "2713"

BLEService testService (BLE_UUID_TEST_SERVICE);
BLEIntCharacteristic heartRate (BLE_UUID_HEARTRATE, BLERead | BLENotify);

MAX30105 particleSensor;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &WIRE);

const int BLE_LED_PIN = LED_BUILTIN;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

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

void connectBLE(){
  Serial.println("Starting BLE");
  if (!BLE.begin()) {
    Serial.println("BLE module failed!");
  // don't continue
    while (true);
  }
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Inicializando...");
  display.println("Inicializando BLE...");
  display.display();

  BLE.setDeviceName ("Arduino Heart Rate");
  BLE.setLocalName ("Arduino Heart Rate");
  BLE.setAdvertisedService (testService);

  testService.addCharacteristic (heartRate);

  BLE.addService (testService);
  heartRate.writeValue(0);
  
  BLE.advertise();

  digitalWrite( BLE_LED_PIN, HIGH );
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Inicializando...");
  display.println("BLE en Linea");
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

  connectBLE();

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
  long irValue = particleSensor.getIR();

  BLEDevice central = BLE.central();
  
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
    if((millis() - prevPrint) > printInterval){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("Promedio BPM: ");
      display.println(beatAvg);
      display.display();
      
      Serial.print( "Central RSSI: " );
      Serial.println( central.rssi() );
      delay(1000);
      prevPrint = millis();
      if(central.rssi() <= 0){
        heartRate.writeValue(beatAvg);
      }
      else{
        Serial.println("Device not Connected");
        delay(5000);
      }
     
    }
  }
  else{
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
