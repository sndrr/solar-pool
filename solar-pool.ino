/** 
 * Solar pool control - Arduino port
 *
 * Uses:
 *  SSD1306 oled display
 *  DS18B20 temperature sensors (1wire)
 *  SainSmart relay
 *
 * Version: 0.1, july 2016
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

// Assign the addresses of your 1-Wire temp sensors.
// See the tutorial on how to obtain these addresses:
// http://www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html

//DeviceAddress roofThermometer = { 0x28, 0x48, 0xBF, 0x28, 0x06, 0x00, 0x00, 0x57 };
DeviceAddress waterThermometer = { 0x28, 0xFF, 0xC5, 0x9A, 0x3E, 0x04, 0x00, 0x53 };
DeviceAddress roofThermometer = { 0x28, 0x81, 0x88, 0x28, 0x06, 0x00, 0x00, 0x8B };
//DeviceAddress waterThermometer = { };

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int active = 2;
int onboardLed = 13;
int relay1 = 4;
int relay2 = 5;

// Set easier name for the display
SSD1306AsciiAvrI2c oled;
// Oled address 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

//------------------------------------------------------------------------------
void setup() {         
  // led and relay setup
  pinMode(active, OUTPUT);   
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  // set initial values
  digitalWrite(active, LOW);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  // start serial port
  Serial.begin(9600);
  // Start up the library
  sensors.begin();
  // set the resolution to 12 bit (good enough?)
  sensors.setResolution(roofThermometer, 12);

  // oled setup
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);  

  uint32_t m = micros();
  oled.clear();  
  oled.println("Pump control v0.1");
  oled.println();
  oled.set2X();
  oled.println("Let's go");
  oled.set1X();
  oled.print("\nStartup in: ");
  oled.print(micros() - m);
  oled.println("ms");
  delay(2000);
  oled.setScroll(true);
  oled.println("Scrolling");
}
//------------------------------------------------------------------------------

float getTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == -127.00) {
    Serial.print("Error getting temperature");
//    tempC = null;
  }
  return tempC;
}

boolean setRelay(float roofTemp, float waterTemp) {
  Serial.print("Setting relay...");
  boolean success;
  if ( (roofTemp == -127.00) || (waterTemp == -127.00) ) {
    success = false;
  }

  if (roofTemp > 22) {
    digitalWrite(active, HIGH);
    digitalWrite(relay1, LOW);
    success =  true;
  } else {
    digitalWrite(active, LOW);    // turn the LED off by making the voltage LOW
    digitalWrite(relay1, HIGH);  // switch the relay off
    success = true;
  }
  if (success) {
    Serial.print("[OK]\r\n");
  } else {
    Serial.print("[FAIL]\r\n");
  }
  return success;
}

void loop() {
  while (true) {
    // toggle led to show activity
    digitalWrite(onboardLed, !digitalRead(onboardLed));
    Serial.print("Getting temperatures...\n\r");
    sensors.requestTemperatures();
    Serial.print("roof temperature is: ");

    float tempRoof = getTemperature(roofThermometer);
    Serial.print(tempRoof);
    Serial.print("\n\r\n\r");
    float tempWater = getTemperature(waterThermometer);

    oled.print("d: ");
    oled.print(tempRoof);
    oled.print(" w: ");
    oled.println(tempWater);
    setRelay(tempRoof, tempWater);
    delay(2000);
  }
}
