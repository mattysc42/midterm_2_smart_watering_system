/* 
 * Project: Smart_Watering_System
 * Author: Matthew Call
 * Date: 7/23/24
 */

#include "Particle.h"
#include "credentials.h"
#include "Adafruit_SSD1306.h"
#include "Grove_Air_quality_Sensor.h"
#include "Adafruit_BME280.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_SPARK.h"
#include "My_Toggle.h"
#include "My_Timer.h"
#include "linear_conversion.h"

const int SOILMOISTUREPIN = D11; //A0
const int PUMPPIN = D9;
const int AQSENSEPIN = D12; //A1
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

// Create the TCP Client
TCPClient TheClient;

// adafruit.io stuff
Adafruit_MQTT_SPARK mqtt (&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe subFeedSeeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/seeeddata");
Adafruit_MQTT_Subscribe subFeedAq = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/email");
Adafruit_MQTT_Publish pubFeedSeeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/seeeddata");
Adafruit_MQTT_Publish pubFeedAq = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/email");



// toggle and timer objects
// MyToggle pumpToggle;

bool pumpToggle = false;

MyTimer timerPump;
MyTimer timerMoistureSensor;
MyTimer timerDustSensor;
MyTimer timerAqSensor;
MyTimer timerSensorCycle;
MyTimer timerSerialPrint;

SYSTEM_MODE(AUTOMATIC);

// SYSTEM_THREAD(ENABLED);

//SerialLogHandler logHandler(LOG_LEVEL_INFO);

//void activatePump(int READPIN, bool toggle);
int checkMoisture(int READPIN);
void getDustConcentration(int READPIN);
float getTempFar();
float getPressureMerc();




void setup() {
    Serial.begin(9600);

    pinMode(SOILMOISTUREPIN, INPUT);
    pinMode(PUMPPIN, OUTPUT);
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
    if(timerMoistureSensor.setTimer(HALFHOUR)) {
        if(checkMoisture(SOILMOISTUREPIN) > 2000) {
            pumpToggle = true;
            Serial.printf("Pump toggle = TRUE\n");
        }
    }

    if(pumpToggle == true) {
        pumpToggle = false;
        digitalWrite(PUMPPIN, HIGH);
        delay(500);
        digitalWrite(PUMPPIN, LOW);
        Serial.printf("Pump toggle = FALSE\n");

    }

    // OLED output
    if(timerSerialPrint.setTimer(1000)) {
        Serial.printf("Humidity is: %0.2f\n", bmeSensor.readHumidity());
        myOLED.printf("Humidity: %0.2f", bmeSensor.readHumidity());
    }
}

int checkMoisture(int READPIN) {
    int soilMoisture = analogRead(READPIN);
    Serial.printf("Soil moisture is: %i\n", analogRead(READPIN));

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
    }
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

void outputAqData() {
    int aqInput = aqSensor.slope();
    if(timerAqSensor.setTimer(30000)) {
        timerAqSensor.startTimer();
        int aqValue = aqSensor.getValue();
        Serial.printf("AQ sensor value is: %i\n", aqValue);

        if (aqInput == AirQualitySensor::HIGH_POLLUTION) {
            Serial.printf("Air quality is poor.\n");
            pubFeedAq.publish("\nAir quality is poor.\n");
            pubFeedAq.publish(aqValue);
        } 
        else if (aqInput == AirQualitySensor::LOW_POLLUTION) {
            Serial.printf("Air quality is good!\n");
            pubFeedAq.publish("\nAir quality is good.\n");
            pubFeedAq.publish(aqValue);
        } 
        else if (aqInput == AirQualitySensor::FRESH_AIR) {
            Serial.printf("Air is fresh!\n");
            pubFeedAq.publish("\nAir quality is great!\n");
            pubFeedAq.publish(aqValue);
        }
        else if (aqInput == AirQualitySensor::FORCE_SIGNAL) {
            Serial.printf("Air quality is VERY poor.\n");
            pubFeedAq.publish("\nAir quality is VERY poor!\n");
            pubFeedAq.publish(aqValue);
        }
    }
}