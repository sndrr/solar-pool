/** 
 * Solar pool control - Arduino port
 *
 * A thermostat for solar heating a pool.
 *
 * (c) july 2016
 * Author: Sander Ruitenbeek <sander@grids.be>
 */

#include <SSD1306Ascii.h>
#include <SSD1306AsciiAvrI2c.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "solar-pool.h"

// Data wire is plugged into pin 3 on the Arduino
#define ONE_WIRE_BUS 3

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Assign the addresses of your 1-Wire temp sensors.
//DeviceAddress testThermometer = { 0x28, 0x48, 0xBF, 0x28, 0x06, 0x00, 0x00, 0x57 };
DeviceAddress waterThermometer = {0x28, 0xFF, 0xC5, 0x9A, 0x3E, 0x04, 0x00, 0x53};
DeviceAddress roofThermometer = {0x28, 0x81, 0x88, 0x28, 0x06, 0x00, 0x00, 0x8B};
//DeviceAddress waterThermometer = { };

int pumpActive = 2; // pump active, thus fresh water
int onboardLed = 13;
int relay1 = 4;
int relay2 = 5;

// Set easier name for the display
SSD1306AsciiAvrI2c oled;
// OLED address 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

#define ROOF_START_TEMPERATURE 22
#define WATER_STOP_TEMPERATURE 28

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
    // Start up the sensors library
    sensors.begin();
    // set the resolution to 12 bit (=max resolution)
    sensors.setResolution(roofThermometer, 12);

    setupOled();
}
//------------------------------------------------------------------------------

void setupOled() {
    // oled setup
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
    oled.setFont(Adafruit5x7);

    // Show banner
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
}

float getTemperature(DeviceAddress deviceAddress) {
    float tempC = sensors.getTempC(deviceAddress);
    if (tempC == -127.00) {
        Serial.print("Error getting temperature");
    }
    return tempC;
}

boolean setRelay(float roofTemp, float waterTemp) {
    Serial.print("Setting relay...");
    boolean active;
    if ((roofTemp == -127.00) || (waterTemp == -127.00)) {
        active = false;
    }

    active = decidePump(roofTemp, waterTemp);

    try {
        if (active) {
            digitalWrite(relay1, LOW);
            Serial.print("[OK]\r\n");
        } else {
            digitalWrite(relay1, HIGH);
            Serial.print("[FAIL]\r\n");
        }
    }

    return active;
}

boolean decidePump(float roofTemp, float waterTemp) {
    boolean on = false; // safekeeping

    if ( ! digitalRead(pumpActive) ) {
        // Run once to make sure we measure the right water temperature
        on = (roofTemp > ROOF_START_TEMPERATURE);
    } else {
        if (waterTemp < WATER_STOP_TEMPERATURE) {
            on = (roofTemp > ROOF_START_TEMPERATURE);
        } else {
            // too hot already, don't heat up and start cooling
            on = (roofTemp < waterTemp);
        }
    }

    return on;
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

        oled.print("d:");
        oled.print(tempRoof);
        oled.print(" w:");
        oled.println(tempWater);
        // set the relay
        boolean on = setRelay(tempRoof, tempWater);
        // set the green activity led accordingly
        digitalWrite(pumpActive, on);
        delay(2000);
    }
}
