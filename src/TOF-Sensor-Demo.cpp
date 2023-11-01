/*
  TOF Based Occupancy Counter - Demo Code
  By: Chip McClelland
      Alex Bowen
  May 2023 - September 2023
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create the library used in this sketch. Feel like supporting open source hardware? 
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  Also, I got early inspiration from a project by Jan - https://github.com/BasementEngineering/PeopleCounter
  But, I needed to change the logic to better match my way of thinking.  Thank you Jan for getting me started!

  This example proves out some of the TOF sensor capabilitites we need to validate:
  - Abiltiy to Sleep and wake on range interrupt
  - Once awake, to determine if an object is moving parallel, toward or away from the sensor
  - Ability to deal with edge cases: Door closings, partial entries, collisions, chairs, hallway passings, etc.
*/

// v1.00 - For testing purposes put into "testing mode" to get rapid output via usb serial
// v2.00 - Will stay disconnected to simplfy code - Just trying to implement directionality
// v2.01 - First version for testing
// v2.02 - Changing oritentation to match ST Micro example code
// v2.03 - Added a buffer to find the minimum deistance in buffer set in the config file
// v3.00 - Removed most logic in favor of a simple getSignalBySpad() approach.
// v4.00 - Implemented the "magicalStateMap" algorithm, which replaced the FSM. Counts now change when sufficiently BELOW baseline. Helps detect black.
// v5.00 - Added Interrupt to put device to sleep when not detecting a person. Made occupancyZone as small as it would go. Added "DetectionZone", consisting of all 16x16 spads.

#include <Wire.h>
#include "ErrorCodes.h"
#include "TofSensor.h"
#include "PeopleCounter.h"

// Enable logging as we ware looking at messages that will be off-line - need to connect to serial terminal
SerialLogHandler logHandler(LOG_LEVEL_INFO);
// Initialize new Sleep 2.0 Api
SystemSleepConfiguration config;

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

//Optional interrupt and shutdown pins.
const int shutdownPin = D2;                 // Pin to shut down the device - active low
const int intPin =      D3;                 // Hardware interrupt - poliarity set in the library
const int blueLED =     D7;
char statusMsg[64] = "Startup Complete.  Running version 5.0";

void detectionISR(){
  TofSensor::instance().setDetectionMode(1);
}

void setup(void)
{
  Wire.begin();
  waitFor(Serial.isConnected, 10000);       // Primarily interface to this code is serial
  delay(1000);                              // Gives serial time to connect

  pinMode(blueLED,OUTPUT);                  // Set up pin names and modes
  pinMode(intPin,INPUT);
  pinMode(shutdownPin,OUTPUT);              // Not sure if we can use this - messes with Boron i2c bus
  digitalWrite(shutdownPin, LOW);           // Turns on the module
  digitalWrite(blueLED,HIGH);               // Blue led on for Setup

  delay(100);

  TofSensor::instance().setup();
  PeopleCounter::instance().setup();

  Log.info(statusMsg);

  attachInterrupt(intPin, detectionISR, RISING);

  digitalWrite(blueLED, LOW);               // Signal setup complete
}

unsigned long lastLedUpdate = 0;
unsigned long lastDetection = 0;

void loop(void)
{
  if( (millis() - lastLedUpdate) > 1000 ){
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
    lastLedUpdate = millis();
  }

  switch(TofSensor::instance().getDetectionMode()){ // Switch based on what mode we're in
    case 0:                                         // If we are in occupancy mode ... 
        if (TofSensor::instance().loop()) {             // ... and there is new data from the sensor ...
          PeopleCounter::instance().loop();                 // ... then check to see if we need to update the counts.
        }
        if(TofSensor::instance().getOccupancyState() == 0){  // If it has been long enough since we woke up AND noone is under us currently ...
          TofSensor::instance().setDetectionMode(1);            // ... set the device back to detection mode ...
          TofSensor::instance().performDetectionCalibration();  // ... recalibrate the detection zone's baseline range ...
          TofSensor::instance().performOccupancyCalibration();  // ... recalibrate the occupancy zones' baseline ranges ... [!] NEED TO TEST IF IT IS OK TO DO HERE [!]
          detachInterrupt(intPin);                              // ... clear the interrupt ...
          attachInterrupt(intPin, detectionISR, RISING);        // ... attach a new interrupt ...
          config.mode(SystemSleepMode::ULTRA_LOW_POWER);        // ... configure our slumber ...
          SystemSleepResult result = System.sleep(config);      // ... then put the device to sleep.
          Log.info("Person Detected. Device Going to Sleep.");
        }
      break;
    case 1:                                         // If we are in detection mode
      if(TofSensor::instance().loop()) {                // ... and there is new data from the sensor ...
        if(TofSensor::instance().getDetectionState() != 0){ // ... and that data tells us a detection has been made ...
          TofSensor::instance().setDetectionMode(0);            // ... then quickly set to occupancy mode.
        }
        detachInterrupt(intPin);                            // Detach the interrupt.
      }
      break;
  }
}