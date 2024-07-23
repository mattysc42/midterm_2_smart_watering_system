/* 
 * Project: Smart_Watering_System
 * Author: Matthew Call
 * Date: 7/23/24
 */

#include "Particle.h"
#include "Adafruit_SSD1306.h"
#include "Grove_Air_quality_Sensor.h"
#include "Adafruit_BME280.h"
#include "Adafruit_MQTT.h"
#include "My_Toggle.h"
#include "My_Timer.h"
#include "linear_conversion.h"

const int SOILMOISTUREPIN = D11;
const int PUMPPIN = D16;
const int AQSENSEPIN = D12;
const int DUSTPIN = D4;
const int SDAPIN = D0;
const int SCLPIN = D1;
const int HALFHOUR = 1800000;




// OLED variables or objects
const int OLED_RESET = -1;
Adafruit_SSD1306 myOLED(OLED_RESET);

// BME objects
Adafruit_BME280 bmeSensor;
byte sensorAddress = 0x76;

// seeed objects
AirQualitySensor aqSensor(AQSENSEPIN);



// toggle and timer objects
MyToggle pumpToggle;

MyTimer timerPump;
MyTimer timerMoistureSensor;
MyTimer timerDustSensor;
MyTimer timerSensorCycle;

SYSTEM_MODE(AUTOMATIC);

// SYSTEM_THREAD(ENABLED);

//SerialLogHandler logHandler(LOG_LEVEL_INFO);

void activatePump(int READPIN);
int checkMoisture(int READPIN);
void getDustConcentration(int READPIN);
float getMappedHumidity();
float getTempFar();
float getPressureMerc();




void setup() {
    Serial.begin(9600);

    pinMode(SOILMOISTUREPIN, INPUT);
    WiFi.on();
    WiFi.connect();

    if (aqSensor.init()) {
        Serial.printf("Sensor ready.");
    } else {
        Serial.printf("Sensor ERROR!");
    }

    // OLED stuff
    myOLED.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    myOLED.clearDisplay();
    myOLED.setTextSize(1);
    myOLED.setTextColor(1, 0);
    myOLED.setCursor(0, 0);
    myOLED.printf("BME Monitor\n------------\n");
    myOLED.display();
    delay(2000);

    // BME stuff
    bool status = bmeSensor.begin(sensorAddress);
    if(status == false) {
        Serial.printf("BME280 at address %02X failed to start", sensorAddress);
    }
}

void loop() {
    if(timerSensorCycle.setTimer(HALFHOUR)) {
        timerSensorCycle.startTimer();
        if(checkMoisture(SOILMOISTUREPIN) > 2800) {
            // toggle pump to on.
            pumpToggle.toggle();
            activatePump(PUMPPIN);
        }
    }
}

void activatePump(int READPIN) {

    if(pumpToggle.toggle() == true) {
        // toggle to false at activation then turn the pump on.
        pumpToggle.toggle();
        digitalWrite(READPIN, HIGH);

        // turn the pump off after 500ms
        if(timerPump.setTimer(500)) {
            timerPump.startTimer();
            digitalWrite(READPIN, LOW);
        }
    }
}

int checkMoisture(int READPIN) {
    int soilMoisture = analogRead(READPIN);
    return soilMoisture;
}

// Uses pulseIn to get and a slope from the seeed library to get dust concentration.
void getDustConcentration(int READPIN) {
    static int duration, lowPulseOccupency;
    int sampleTime = 30000;
    static float ratio, concentration;
 
    duration = pulseIn(READPIN, LOW);
    lowPulseOccupency = lowPulseOccupency + duration;

    if(timerDustSensor.setTimer(sampleTime)) {
        ratio = lowPulseOccupency / (sampleTime * 10.0);
        concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62;
        Serial.printf("\nOccupency is: %i\nRatio is: %0.4f\nConcentration is: %0.4f\n", lowPulseOccupency, ratio, concentration);
        lowPulseOccupency = 0;
        timerDustSensor.startTimer();
    }
}

float getMappedHumidity() {
    return mapFloat(bmeSensor.readHumidity(), 0, 4096, 0, 100);
}

float getTempFar() {
    float tempCel = bmeSensor.readTemperature();
    float outputTempFar = (tempCel * 1.8) + 32.0;
    return outputTempFar;
}

float getPressureMerc() {
    float inputPressure = bmeSensor.readPressure();
    float outputPressure = (inputPressure * 0.00029530) + 5;
    return outputPressure;
}

