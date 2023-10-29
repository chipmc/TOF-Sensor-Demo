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

#include <Wire.h>
#include "ErrorCodes.h"
#include "TofSensor.h"
#include "PeopleCounter.h"

// Enable logging as we ware looking at messages that will be off-line - need to connect to serial terminal
SerialLogHandler logHandler(LOG_LEVEL_INFO);

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

//Optional interrupt and shutdown pins.
const int shutdownPin = D2;                       // Pin to shut down the device - active low
const int intPin =      D3;                       // Hardware interrupt - poliarity set in the library
const int blueLED =     D7;
char statusMsg[64] = "Startup Complete.  Running version 4.0";

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

  digitalWrite(blueLED, LOW);                   // Signal setup complete
}

unsigned long lastLedUpdate = 0;

void loop(void)
{
  if( (millis() - lastLedUpdate) > 1000 ){
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
    lastLedUpdate = millis();
  }

  if (TofSensor::instance().loop()) {         // If there is new data from the sensor
    PeopleCounter::instance().loop();         // Then check to see if we need to update the counts
  }
}
