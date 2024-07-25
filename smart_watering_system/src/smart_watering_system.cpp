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

const int SOILMOISTUREPIN = A1; //A0
const int PUMPPIN = D9;
const int AQSENSEPIN = A5; //A5
const int DUSTPIN = D19;
const int SDAPIN = D0;
const int SCLPIN = D1;
const int HALFHOUR = 1800000;

int soilMoisture;

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
Adafruit_MQTT_Subscribe subFeedAq = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/aqdata");
Adafruit_MQTT_Subscribe subFeedEmail = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/email");

Adafruit_MQTT_Publish pubFeedSeeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/seeeddata");
Adafruit_MQTT_Publish pubFeedAq = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/aqdata");
Adafruit_MQTT_Publish pubFeedEmail = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/email");



// toggle and timer objects
// MyToggle pumpToggle;

bool pumpToggle;
bool toggleOLED;

MyTimer timerMoistureSensor;
MyTimer timerDustSensor;
MyTimer timerAqSensor;
MyTimer timerOLEDPrint;
MyTimer timerDateTime;

SYSTEM_MODE(AUTOMATIC);

// SYSTEM_THREAD(ENABLED);

//SerialLogHandler logHandler(LOG_LEVEL_INFO);

void MQTT_connect();
bool MQTT_ping();
int checkMoisture(int READPIN);
void getDustConcentration(int READPIN);
float getTempFar();
float getPressureMerc();
void aqDataOLED();
void aqDataPublish();
void printTimeOLED();



void setup() {
    Serial.begin(9600);
    Time.zone(-7); // MST = -7, MDT = -6
    Particle.syncTime();
    
    // pinmodes
    pinMode(SOILMOISTUREPIN, INPUT);
    pinMode(DUSTPIN, INPUT_PULLUP);
    pinMode(PUMPPIN, OUTPUT);

    //wifi, mqtt stuff
    WiFi.on();
    WiFi.connect();
    mqtt.subscribe(&subFeedSeeed);
    mqtt.subscribe(&subFeedEmail);
    mqtt.subscribe(&subFeedAq);

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
    MQTT_connect();
    MQTT_ping();
    getDustConcentration(DUSTPIN);
    soilMoisture = analogRead(SOILMOISTUREPIN);
    // pump stuff
    if(timerMoistureSensor.setTimer(HALFHOUR)) {
        if(soilMoisture > 3000) {
            pumpToggle = true;
            Serial.printf("Pump toggle = TRUE\n");
            pubFeedEmail.publish(soilMoisture);
        }
    }

    if(pumpToggle == true) {
        pumpToggle = false;
        digitalWrite(PUMPPIN, HIGH);
        delay(500);
        digitalWrite(PUMPPIN, LOW);
        Serial.printf("Pump toggle = FALSE\n");
    }
    // OLED stuff
    
    if(timerOLEDPrint.setTimer(1000)) {
        if(toggleOLED == false) {
            // second OLED page
            myOLED.setCursor(0, 0);
            aqDataOLED();
            myOLED.printf("Soil moisture: %i\n", soilMoisture);
            printTimeOLED();
            myOLED.display();
        }
        if(toggleOLED == true) {
            // first OLED page
            myOLED.setCursor(0, 0);
            myOLED.printf("Humidity: %0.2f%%\n", bmeSensor.readHumidity());
            myOLED.printf("Pressure: %0.2f\n", getPressureMerc());
            myOLED.printf("Temp: %0.2f F\n", getTempFar());
            printTimeOLED();
            myOLED.display();
        }
        myOLED.clearDisplay();
    }
    if(timerDateTime.setTimer(5000)) {
        toggleOLED = !toggleOLED;
    }
}

int checkMoisture(int READPIN) {
    int soilMoisture = analogRead(READPIN);
    myOLED.printf("Soil moisture: %i\n", soilMoisture);

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
        lowPulseOccupency = 0;
        pubFeedSeeed.publish(concentration);
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

void aqDataOLED() {
    int aqInput = aqSensor.slope();
    int aqValue = aqSensor.getValue();
    myOLED.printf("AQ value: %i\n", aqValue);

    if (aqInput == AirQualitySensor::HIGH_POLLUTION) {
        myOLED.printf("AQ is poor.\n");
    } 
    else if (aqInput == AirQualitySensor::LOW_POLLUTION) {
        myOLED.printf("AQ is good.\n");
    } 
    else if (aqInput == AirQualitySensor::FRESH_AIR) {
        myOLED.printf("AQ is fresh.\n");
    }
    else if (aqInput == AirQualitySensor::FORCE_SIGNAL) {
        myOLED.printf("AQ is VERY poor.\n");
    }
}

void aqDataPublish() {
    int aqInput = aqSensor.slope();
    int aqValue = aqSensor.getValue();
    if(timerAqSensor.setTimer(30000)) {
        if (aqInput == AirQualitySensor::HIGH_POLLUTION) {
            pubFeedAq.publish("\nAir quality is poor.\n");
        } 
        else if (aqInput == AirQualitySensor::LOW_POLLUTION) {
            pubFeedAq.publish("\nAir quality is good.\n");
        } 
        else if (aqInput == AirQualitySensor::FRESH_AIR) {
            pubFeedAq.publish("\nAir quality is great!\n");
        }
        else if (aqInput == AirQualitySensor::FORCE_SIGNAL) {
            pubFeedAq.publish("\nAir quality is VERY poor!\n");
        }
    }
}

void printTimeOLED() {
    String dateTime, timeOnly, dateOnly;
    dateTime = Time.timeStr();
    timeOnly = dateTime.substring(11, 19);
    dateOnly = dateTime.substring(0, 10);
    Serial.printf("\n\n%s\n%s\n", dateOnly.c_str(), timeOnly.c_str());
    myOLED.printf("\n\n%s\n%s\n", dateOnly.c_str(), timeOnly.c_str());
}

void MQTT_connect() {
    int8_t ret;
 
    // Return if already connected.
    if (mqtt.connected()) {
        return;
    }
 
    Serial.print("Connecting to MQTT... ");
 
    while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
    }
    Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
    static unsigned int last;
    bool pingStatus;

    if ((millis()-last)>120001) {
        Serial.printf("Pinging MQTT \n");
        pingStatus = mqtt.ping();
        if(!pingStatus) {
            Serial.printf("Disconnecting \n");
            mqtt.disconnect();
        }
        last = millis();
    }
    return pingStatus;
}
