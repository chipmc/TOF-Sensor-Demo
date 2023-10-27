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
int zoneSignalPerSpad[2] = {0,0};
int zoneBaselines[2] = {0,0};
int occupancyState = 0;      // This is the current occupancy state (occupied or not, zone 1 (ones) and zone 2 (twos))

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
  myTofSensor.setDistanceModeLong();
  myTofSensor.setSigmaThreshold(45);        // Default is 45 - this will make it harder to get a valid result - Range 1 - 16383
  myTofSensor.setSignalThreshold(1500);     // Default is 1500 raising value makes it harder to get a valid results- Range 1-16383
  myTofSensor.setTimingBudgetInMs(20);      // Was 20mSec

  while (TofSensor::loop() == SENSOR_BUFFRER_NOT_FULL) {delay(10);}; // Wait for the buffer to fill up
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
  for (int i=1; i<NUM_CALIBRATION_LOOPS; i++) {
    TofSensor::loop();                  // Get the latest values
    zoneBaselines[0] += TofSensor::getZone1();
    zoneBaselines[1] += TofSensor::getZone2();
  }
  zoneBaselines[0] = zoneBaselines[0]/NUM_CALIBRATION_LOOPS;
  zoneBaselines[1] = zoneBaselines[1]/NUM_CALIBRATION_LOOPS;

  if (occupancyState != 0){
    Log.info("Target zone not clear - will wait ten seconds and try again");
    delay(10000);
    TofSensor::loop();
    if (occupancyState != 0) return FALSE;
  }
  Log.info("Target zone is clear with zone1 at %ikcps/SPAD and zone2 at %ikcps/SPAD",zoneBaselines[0],zoneBaselines[1]);
  return TRUE;
}

int TofSensor::loop(){                         // This function will update the current distance / occupancy for each zone.  It will return true if occupancy changes                    
  int oldOccupancyState = occupancyState;
  occupancyState = 0;

  unsigned long startedRanging;

  for (byte zone = 0; zone < 2; zone++){
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

    #if DEBUG_COUNTER
    Log.info("Zone%d (%dx%d %d SPADs with optical center %d) = %ikcps/SPAD. Signal/SPAD: %d Ambient/SPAD: %d",zone+1,myTofSensor.getROIX(), myTofSensor.getROIY(), myTofSensor.getSpadNb(),opticalCenters[zone],zoneSignalPerSpad[zone],myTofSensor.getSignalPerSpad(), myTofSensor.getAmbientPerSpad());
    delay(500);
    #endif

    zoneSignalPerSpad[zone] = myTofSensor.getSignalPerSpad(); // - getAmbientPerSpad()??
    bool occupied = ((zoneSignalPerSpad[zone] >= (zoneBaselines[zone] + PERSON_THRESHOLD)) || (zoneSignalPerSpad[zone] <= (zoneBaselines[zone] - (PERSON_THRESHOLD))));
    occupancyState += occupied * (zone + 1);
  }

  #if PEOPLECOUNTER_DEBUG
  if (occupancyState != oldOccupancyState) Log.info("Occupancy state changed from %d to %d (%ikcps/SPAD / %ikcps/SPAD)", oldOccupancyState, occupancyState, zoneSignalPerSpad[0], zoneSignalPerSpad[1]);
  #endif

  return (occupancyState != oldOccupancyState);     // Let us know if the occupancy state has changed
}

int TofSensor::getZone1() {
  return zoneSignalPerSpad[0];
}

int TofSensor::getZone2() {
  return zoneSignalPerSpad[1];
}

int TofSensor::getOccupancyState() {
  return occupancyState;
}



