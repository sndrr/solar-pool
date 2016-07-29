/** 
 * Solar pool control - Arduino port
 *
 * Uses:
 *  SSD1306 oled display
 *  DS18B20 temperature sensors (1wire)
 *  SainSmart relay
 *
 * Version: 1.1, July 29, 2016
 * (c)
 * Author: Sander Ruitenbeek <sander@grids.be>
 */

#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into pin 3 on the Arduino
#define ONE_WIRE_BUS 3

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

DeviceAddress waterThermometer = { 0x28, 0xFF, 0xC5, 0x9A, 0x3E, 0x04, 0x00, 0x53 };
DeviceAddress roofThermometer = { 0x28, 0x81, 0x88, 0x28, 0x06, 0x00, 0x00, 0x8B };

// Name the GPIO pins (they don't change, so consts are in order)
const int pumpActiveLed = 2;
const int onboardLed = 13;
const int pumpRelay = 4;
const int relay2 = 5;

// Set easier name for the display
SSD1306AsciiAvrI2c oled;
// Oled address: 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

// Set the start and stop temperatures
#define ROOF_START_TEMPERATURE 22
#define WATER_STOP_TEMPERATURE 28

float running = 0;

//------------------------------------------------------------------------------
void setup() {         
  // led and relay setup
  pinMode(pumpActiveLed, OUTPUT);   
  pinMode(pumpRelay, OUTPUT);
  pinMode(relay2, OUTPUT);
  // set initial values
  digitalWrite(pumpActiveLed, LOW); // pump = off
  digitalWrite(pumpRelay, HIGH);       // relays are low-switching
  digitalWrite(relay2, HIGH);
  // start serial port
  Serial.begin(9600);
  // Start up the library
  sensors.begin();
  // set the resolution to 12 bit (= max. for DS18B20)
  sensors.setResolution(roofThermometer, 12);
  sensors.setResolution(waterThermometer, 12);
  
  setupOled();
}

void setupOled() {
  // oled setup
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);

  uint32_t m = micros();
  oled.clear();  
  oled.println("Pump control v1.0");
  oled.println();
  oled.set2X();
  oled.println("Let's go!");
  oled.set1X();
  oled.print("\nStartup in: ");
  oled.print(micros() - m);
  oled.println("ms");
  delay(500);
  oled.setScroll(true);
}
//------------------------------------------------------------------------------

float getTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == -127.00) {
    Serial.print("Error getting temperature");
  }
  return tempC;
}

boolean setRelay(float roofTemp, float waterTemp) {
  Serial.print("Setting relay...");
  boolean success;
  if ( (roofTemp == -127.00) || (waterTemp == -127.00) ) {
    success = false;
  }
  else {
    success = true;
    // do we need to pump, based on these temperatures?
    boolean active = decidePump(roofTemp, waterTemp);
    
    if (active) {
      digitalWrite(pumpRelay, LOW);
      Serial.print("[ON] ");
    }
    else {
      digitalWrite(pumpRelay, HIGH);
      Serial.print("[OFF] ");
    }
  }
  if (success) {
    Serial.print("[OK]\r\n");
  }
  else {
    Serial.print("[FAIL]\r\n");
  }

  return success;
}

boolean doProbe = true; // first run is always a probe

boolean decidePump(float roofTemp, float waterTemp) {
  boolean on = false; // default = off

  if ( ! digitalRead(pumpActiveLed) && doProbe ) {
    // Run once to make sure we measure the right water temperature
    on = (roofTemp > ROOF_START_TEMPERATURE);
    doProbe = false;
  }
  else {
    if (waterTemp < WATER_STOP_TEMPERATURE) {
      // added the .3 here to counter for "debouncing"
      on = (roofTemp - 0.3 > ROOF_START_TEMPERATURE);
    }
    else {
      // too hot already, don't heat up and start cooling
      on = (roofTemp < waterTemp);
    }
  }
  return on;
}

void loop() {
  int loopsWithoutProbe = 0; // count the loops to set a probe every ~10 mins
  while (true) {
    // toggle led to show activity
    digitalWrite(onboardLed, !digitalRead(onboardLed));
    Serial.print("Getting temperatures...\n\r");
    sensors.requestTemperatures();
    Serial.print("roof temperature is: ");
    float tempRoof = getTemperature(roofThermometer);
    Serial.println(tempRoof);
    Serial.print("water temperature is: ");
    float tempWater = getTemperature(waterThermometer);
    Serial.println(tempRoof);

    oled.print("d:");
    oled.print(tempRoof);
    oled.print("w:");
    oled.print(tempWater);
    oled.print("p:");
    oled.println(running);
    
    boolean ledOn = setRelay(tempRoof, tempWater);
    digitalWrite(pumpActiveLed, ledOn);
    delay(30000);
    if (ledOn) {
      running += 0.5;
    }
    else {
      running = -1;
      loopsWithoutProbe++;
      if ( loopsWithoutProbe == 20 ) {
        doProbe = true;
        loopsWithoutProbe = 0;
      }
    }
  }
}
