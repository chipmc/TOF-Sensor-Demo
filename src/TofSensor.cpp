// Time of Flight Sensor Class
// Author: Chip McClelland
// Date: May 2023
// License: GPL3
// This is the class for the ST Micro VL53L1X Time of Flight Sensor
// We are using the Sparkfun library which has some shortcomgings
// - It does not implement distance mode medium
// - It does not give access to the factory calibration of the optical center

#include "Particle.h"
#include "ErrorCodes.h"
#include "TofSensorConfig.h"
#include "PeopleCounterConfig.h"
#include "TofSensor.h"

uint8_t opticalCenters[2] = {FRONT_ZONE_CENTER,BACK_ZONE_CENTER}; 
int zoneSignalPerSpad[2][3] = {{0, 0},{0, 0}};
int zoneBaselines[2][2] = {{0,0},{0,0}};
int lastSignal[2] = {0,0};
int lastAmbient[2] = {0,0};
int occupancyState = 0;      // This is the current occupancy state (occupied or not, zone 1 (ones) and zone 2 (twos))
int ready = 0;

TofSensor *TofSensor::_instance;

// [static]
TofSensor &TofSensor::instance() {
  if (!_instance) {
      _instance = new TofSensor();
  }
  return *_instance;
}

TofSensor::TofSensor() {
}

TofSensor::~TofSensor() {
}

SFEVL53L1X myTofSensor;

void TofSensor::setup(){
  if(myTofSensor.begin() != 0){
    Log.info("Sensor error reset in 10 seconds");
    delay(10000);
    System.reset();
  }
  else Log.info("Sensor init successfully");
  
  // Here is where we set the device properties
  myTofSensor.setSigmaThreshold(45);        // Default is 45 - this will make it harder to get a valid result - Range 1 - 16383
  myTofSensor.setSignalThreshold(1500);     // Default is 1500 raising value makes it harder to get a valid results- Range 1-16383
  myTofSensor.setTimingBudgetInMs(20);      // Was 20mSec

  // while (TofSensor::measure() == SENSOR_BUFFRER_NOT_FULL) {delay(10);}; // Wait for the buffer to fill up
  Log.info("Buffer is full - will now calibrate");

  if (TofSensor::performCalibration()) Log.info("Calibration Complete");
  else {
    Log.info("Initial calibration failed - wait 10 secs and reset");
    delay(10000);
    System.reset();
  }

  // myTofSensor.setDistanceModeShort();                     // Once initialized, we are focused on the top half of the door

}

bool TofSensor::performCalibration() {
  TofSensor::measure();
  zoneBaselines[0][0] = zoneSignalPerSpad[0][0];
  zoneBaselines[0][1] = zoneSignalPerSpad[0][1];
  zoneBaselines[1][0] = zoneSignalPerSpad[1][0];
  zoneBaselines[1][1] = zoneSignalPerSpad[1][1];
  for (int i=0; i<NUM_CALIBRATION_LOOPS; i++) {
    TofSensor::measure();
    if(zoneSignalPerSpad[0][0] < zoneBaselines[0][0]) zoneBaselines[0][0] = zoneSignalPerSpad[0][0];
    if(zoneSignalPerSpad[0][1] > zoneBaselines[0][1]) zoneBaselines[0][1] = zoneSignalPerSpad[0][1];
    if(zoneSignalPerSpad[1][0] < zoneBaselines[1][0]) zoneBaselines[1][0] = zoneSignalPerSpad[1][0];
    if(zoneSignalPerSpad[1][1] > zoneBaselines[1][1]) zoneBaselines[1][1] = zoneSignalPerSpad[1][1];
  }
  #if DEBUG_COUNTER
    Log.info("[BASELINE RANGES] Zone1:{Min=%ikbps/SPAD, Max=%ikbps/SPAD} Zone2:{Min=%ikbps/SPAD, Max=%ikbps/SPAD}", zoneBaselines[0][0], zoneBaselines[0][1], zoneBaselines[1][0], zoneBaselines[1][1]);
    Log.info("[OCCUPANCY CHECK] Zone1:{1st=%ikbps/SPAD, 2nd=%ikbps/SPAD} Zone2:{1st=%ikbps/SPAD, 2nd=%ikbps/SPAD} - Occupancy State %i", zoneSignalPerSpad[0][0], zoneSignalPerSpad[0][1], zoneSignalPerSpad[1][0], zoneSignalPerSpad[1][1], occupancyState);
  #endif
  if (occupancyState != 0){
    Log.info("Target zone not clear - will wait 5 seconds and try again");
    delay(5000);
    TofSensor::loop();
    if (occupancyState != 0) return FALSE;
  }
  
  Log.info("Target zone is clear with zone1 range at {MIN: %ikcps/SPAD, MAX: %ikcps/SPAD} and zone2 range at {MIN: %ikcps/SPAD, MAX: %ikcps/SPAD}",zoneBaselines[0][0],zoneBaselines[0][1],zoneBaselines[1][0],zoneBaselines[1][1]);
  return TRUE;
}

int TofSensor::loop(){                         // This function will update the current distance / occupancy for each zone.  It will return true 100% of the time after samples have been taken. Returns error codes also.
  occupancyState = 0;
  int result = TofSensor::measure();
  occupancyState += ((zoneSignalPerSpad[0][0] > (zoneBaselines[0][1]) && zoneSignalPerSpad[0][1] > (zoneBaselines[0][1]))
                    || (zoneSignalPerSpad[0][0] < (zoneBaselines[0][0]) && zoneSignalPerSpad[0][1] < (zoneBaselines[0][0]))) ? 1 : 0;
  
  occupancyState += ((zoneSignalPerSpad[1][0] > (zoneBaselines[1][1]) && zoneSignalPerSpad[1][1] > (zoneBaselines[1][1]))
                    || (zoneSignalPerSpad[1][0] < (zoneBaselines[1][0]) && zoneSignalPerSpad[1][1] < (zoneBaselines[1][0]))) ? 2 : 0;
  #if DEBUG_COUNTER
      Log.info("[BASELINE RANGES] Zone1:{Min=%ikbps/SPAD, Max=%ikbps/SPAD} Zone2:{Min=%ikbps/SPAD, Max=%ikbps/SPAD}", zoneBaselines[0][0], zoneBaselines[0][1], zoneBaselines[1][0], zoneBaselines[1][1]);
      Log.info("[OCCUPANCY CHECK] Zone1:{1st=%ikbps/SPAD, 2nd=%ikbps/SPAD} Zone2:{1st=%ikbps/SPAD, 2nd=%ikbps/SPAD} - Occupancy State %i\n", zoneSignalPerSpad[0][0], zoneSignalPerSpad[0][1], zoneSignalPerSpad[1][0], zoneSignalPerSpad[1][1], occupancyState);
  #endif
  return (result);     // Relay the result of taking our samples.
}

int TofSensor::measure(){
  ready = 0;
  unsigned long startedRanging;
  for (byte samples = 0; samples < 4; samples++){
    int zone = samples % 2;
    myTofSensor.stopRanging();
    myTofSensor.clearInterrupt();
    myTofSensor.setROI(ROWS_OF_SPADS,COLUMNS_OF_SPADS,opticalCenters[zone]);
    delay(1);
    myTofSensor.startRanging();
    startedRanging = millis();
    while(!myTofSensor.checkForDataReady()) {
      if (millis() - startedRanging > SENSOR_TIMEOUT) {
        Log.info("Sensor Timed out");
        return SENSOR_TIMEOUT_ERROR;
      }
    }
    lastSignal[zone] = myTofSensor.getSignalPerSpad();
    lastAmbient[zone] = myTofSensor.getAmbientPerSpad();
    zoneSignalPerSpad[zone][samples < 2 ? 0 : 1] = lastSignal[zone];
    #if DEBUG_COUNTER
     if(samples >= 2){
      // Log.info("[READING] Zone%i (%dx%d %d SPADs with optical center %d)", zone+1, myTofSensor.getROIX(), myTofSensor.getROIY(), myTofSensor.getSpadNb(),opticalCenters[zone]);
      zone == 0 ? Log.info("{1st=%ikbps/SPAD, 2nd=%ikbps/SPAD}", zoneSignalPerSpad[zone][0], zoneSignalPerSpad[zone][1]) : Log.info("                                    {1st=%ikbps/SPAD, 2nd=%ikbps/SPAD}", zoneSignalPerSpad[zone][0], zoneSignalPerSpad[zone][1]);
     }
    #endif
  }
  return(++ready);
}

int TofSensor::getZone1() {
  return lastSignal[0];
}

int TofSensor::getZone2() {
  return lastSignal[1];
}

int TofSensor::getZone1Ambient() {
  return lastAmbient[0];
}

int TofSensor::getZone2Ambient() {
  return lastAmbient[1];
}

int TofSensor::getOccupancyState() {
  return occupancyState;
}



