/* 
 * Project: Smart_Watering_System
 * Author: Matthew Call
 * Date: 7/23/24
 */

#include "Particle.h"
#include "My_Toggle.h"
#include "My_Timer.h"

MyToggle pumpToggle;

SYSTEM_MODE(AUTOMATIC);

// SYSTEM_THREAD(ENABLED);

SerialLogHandler logHandler(LOG_LEVEL_INFO);

void activatePump();

void setup() {

}

void loop() {
    if (pumpToggle.toggle() == true) {
        pumpToggle.toggle();
        activatePump();
    }
}

