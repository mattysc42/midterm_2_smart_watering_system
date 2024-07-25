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
#include "My_Timer.h"

// constants
const int SOILMOISTUREPIN = A1; //A0
const int PUMPPIN = D9;
const int AQSENSEPIN = A5; //A5
const int DUSTPIN = D19;
const int HALFHOUR = 1800000;

// global variables
int soilMoisture;
float dustConcentration;

// toggle variables
bool pumpToggle;
bool toggleOLED;
bool toggleEmailButton;

// OLED object
const int OLED_RESET = -1;
Adafruit_SSD1306 myOLED(OLED_RESET);

// BME object
Adafruit_BME280 bmeSensor;
byte sensorAddress = 0x76;

// seeed objects
AirQualitySensor aqSensor(AQSENSEPIN);

// adafruit.io stuff
TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt (&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Subscribe subFeedEmail = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/email");

Adafruit_MQTT_Publish pubFeedSeeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/seeeddata");
Adafruit_MQTT_Publish pubFeedAq = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/aqdata");
Adafruit_MQTT_Publish pubFeedEmail = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/email");
Adafruit_MQTT_Publish pubFeedSoilMoisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soilmoisture");

// timer objects
MyTimer timerMoistureSensor;
MyTimer timerDustSensor;
MyTimer timerAqSensor;
MyTimer timerOLEDPrint;
MyTimer timerDateTime;

SYSTEM_MODE(AUTOMATIC);

// My Functions
void MQTT_connect();
bool MQTT_ping();
float getDustConcentration(int READPIN);
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
    mqtt.subscribe(&subFeedEmail);

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
    myOLED.printf("Smart Planter\n------------\n");
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
    dustConcentration = getDustConcentration(DUSTPIN);
    soilMoisture = analogRead(SOILMOISTUREPIN);

    // pump stuff
    if(timerMoistureSensor.setTimer(HALFHOUR)) {
        if(soilMoisture > 3000) {
            pumpToggle = true;
            Serial.printf("Pump toggle = TRUE\n");
            pubFeedEmail.publish(soilMoisture);
        }
    }

    // turn pumpToggle to true if the pump button in the adafruit dashboard is pressed for 3 seconds.
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(100))) {
        toggleEmailButton = atoi((char *)subFeedEmail.lastread);
        pumpToggle = toggleEmailButton;
    }

    // turn pump on for half a second if pumptoggle == true
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
            myOLED.printf("Dust: %0.2f\n", dustConcentration);
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
            myOLED.printf("Temp: %0.2f F\n\n", getTempFar());
            printTimeOLED();
            myOLED.display();
        }
        myOLED.clearDisplay();
    }
    // publish to adafruit dashboard and toggle OLED output every 6 seconds.
    if(timerDateTime.setTimer(6000)) {
        aqDataPublish();
        pubFeedSoilMoisture.publish(analogRead(SOILMOISTUREPIN));
        toggleOLED = !toggleOLED;
    }
}

// Uses pulseIn to get and a slope from the seeed library to get dust concentration.
float getDustConcentration(int READPIN) {
    static unsigned long duration, lowPulseOccupency;
    int sampleTime = 30000;
    static float ratio, concentration;
    
    duration = pulseIn(READPIN, LOW);
    lowPulseOccupency = lowPulseOccupency + duration;

    if(timerDustSensor.setTimer(sampleTime)) {
        ratio = lowPulseOccupency / (sampleTime * 10.0);
        concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62;
        lowPulseOccupency = 0;
        pubFeedSeeed.publish(dustConcentration);
    }
    return concentration;
}

// get the temperature from the BME and convert it to Fahrenheit
float getTempFar() {
    float tempCel = bmeSensor.readTemperature();
    float outputTempFar = (tempCel * 1.8) + 32.0;
    return outputTempFar;
}

// get the barometric pressure in inches of mercury
float getPressureMerc() {
    float inputPressure = bmeSensor.readPressure();
    float outputPressure = (inputPressure * 0.00029530) + 5;
    return outputPressure;
}

// output the air quality data to the OLED
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

// output the air quality data to the adafruit dashboard
void aqDataPublish() {
    int aqInput = aqSensor.slope();
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

// print the date and time to the minute to the OLED
void printTimeOLED() {
    String dateTime, timeOnly, dateOnly;
    dateTime = Time.timeStr();
    timeOnly = dateTime.substring(11, 16);
    dateOnly = dateTime.substring(0, 10);
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
